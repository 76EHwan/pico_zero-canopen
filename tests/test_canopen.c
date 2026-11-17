/*
 * PC 용 테스트: CANopen 슬레이브(co_node) + CiA 402(cia402) + slcan 코덱
 *
 *   gcc -std=c99 -Wall -Isrc tests/test_canopen.c src/co_od.c src/co_node.c src/cia402.c \
 *       src/slcan.c src/ak_proto.c -lm -o test_canopen && ./test_canopen
 *
 * 모터는 스텁(co_motor_*)으로 대체하고, 송신 프레임을 가로채서 검사한다.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ak_proto.h"
#include "cia402.h"
#include "co_motor.h"
#include "co_node.h"
#include "config.h"
#include "slcan.h"

static int g_pass, g_fail;
#define CHECK(cond, ...)                                    \
    do {                                                    \
        if (cond) {                                         \
            g_pass++;                                       \
        } else {                                            \
            g_fail++;                                       \
            printf("FAIL line %d: ", __LINE__);             \
            printf(__VA_ARGS__);                            \
            printf("\n");                                   \
        }                                                   \
    } while (0)

/* ---------------- 송신 프레임 캡처 ---------------- */
#define TXMAX 512
static can_frame_t g_tx[TXMAX];
static int g_ntx;

static void sender(const can_frame_t *f, bool important) {
    (void)important;
    if (g_ntx < TXMAX) g_tx[g_ntx++] = *f;
}

static void tx_clear(void) {
    g_ntx = 0;
}

static const can_frame_t *tx_last(uint32_t id) {
    for (int i = g_ntx - 1; i >= 0; i--) {
        if (g_tx[i].id == id) return &g_tx[i];
    }
    return NULL;
}

static uint32_t le32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* ---------------- 모터 스텁 ---------------- */
enum { CALL_NONE = 0, CALL_DISABLE, CALL_MIT, CALL_PV, CALL_VEL };
typedef struct {
    int last;
    float a[5];
    int origin_calls;
    uint8_t can_id, proto;
    co_motor_fb_t fb;
} stub_t;
static stub_t g_m[NUM_MOTORS];

void co_motor_get_fb(int axis, co_motor_fb_t *fb) {
    *fb = g_m[axis].fb;
}
void co_motor_disable(int axis) {
    g_m[axis].last = CALL_DISABLE;
}
void co_motor_mit(int axis, float p, float v, float kp, float kd, float t) {
    stub_t *m = &g_m[axis];
    m->last = CALL_MIT;
    m->a[0] = p, m->a[1] = v, m->a[2] = kp, m->a[3] = kd, m->a[4] = t;
}
void co_motor_pos_spd(int axis, float deg, float erpm, float acc) {
    stub_t *m = &g_m[axis];
    m->last = CALL_PV;
    m->a[0] = deg, m->a[1] = erpm, m->a[2] = acc;
}
void co_motor_vel(int axis, float erpm) {
    g_m[axis].last = CALL_VEL;
    g_m[axis].a[0] = erpm;
}
void co_motor_origin(int axis) {
    g_m[axis].origin_calls++;
}
void co_motor_config(int axis, uint8_t can_id, uint8_t proto) {
    g_m[axis].can_id = can_id;
    g_m[axis].proto = proto;
}

/* ---------------- 노드 / 시간 ---------------- */
#define NODE 5
static co_node_t g_n;
static uint64_t g_now = 1000000;

static void rx(uint32_t id, uint8_t dlc, const uint8_t *d) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    if (dlc) memcpy(f.data, d, dlc);
    co_rx(&g_n, &f, g_now);
}

static void step_ms(int ms) {
    for (int i = 0; i < ms; i++) {
        g_now += 1000;
        cia402_process(&g_n, g_now);
        co_process(&g_n, g_now);
    }
}

static void node_init(void) {
    memset(g_m, 0, sizeof(g_m));
    g_m[0].fb.valid = true;
    tx_clear();
    co_node_init(&g_n, NODE, 0, 0x12345678u, g_now);
}

static void nmt(uint8_t cs) {
    uint8_t d[2] = {cs, NODE};
    rx(0x000, 2, d);
}

/* SDO expedited 쓰기: 성공 0, 실패 abort 코드 */
static uint32_t sdo_write(uint16_t idx, uint8_t sub, uint32_t val, int len) {
    uint8_t d[8] = {(uint8_t)(0x23 | ((4 - len) << 2)), (uint8_t)idx, (uint8_t)(idx >> 8), sub,
                    (uint8_t)val, (uint8_t)(val >> 8), (uint8_t)(val >> 16), (uint8_t)(val >> 24)};
    tx_clear();
    rx(0x600 + NODE, 8, d);
    const can_frame_t *r = tx_last(0x580 + NODE);
    if (!r) return 0xFFFFFFFFu;
    if (r->data[0] == 0x80) return le32(&r->data[4]);
    return r->data[0] == 0x60 ? 0 : 0xFFFFFFFEu;
}

/* SDO expedited 읽기: 성공 0, 실패 abort 코드 */
static uint32_t sdo_read(uint16_t idx, uint8_t sub, uint32_t *val) {
    uint8_t d[8] = {0x40, (uint8_t)idx, (uint8_t)(idx >> 8), sub, 0, 0, 0, 0};
    tx_clear();
    rx(0x600 + NODE, 8, d);
    const can_frame_t *r = tx_last(0x580 + NODE);
    if (!r) return 0xFFFFFFFFu;
    if (r->data[0] == 0x80) return le32(&r->data[4]);
    if ((r->data[0] & 0xE3) != 0x43) return 0xFFFFFFFEu;
    unsigned len = 4 - ((r->data[0] >> 2) & 3);
    uint32_t v = 0;
    memcpy(&v, &r->data[4], len);
    *val = v;
    return 0;
}

static uint16_t sw(void) {
    return g_n.od.statusword;
}

static void rpdo1(uint16_t cw, int8_t mode) {
    uint8_t d[3] = {(uint8_t)cw, (uint8_t)(cw >> 8), (uint8_t)mode};
    rx(0x200 + NODE, 3, d);
}

/* ================================================================== */

static void test_boot_and_sdo(void) {
    node_init();
    const can_frame_t *b = tx_last(0x700 + NODE);
    CHECK(b && b->dlc == 1 && b->data[0] == 0x00, "boot-up frame");
    CHECK(g_n.nmt == CO_NMT_PREOP, "pre-operational after boot");
    CHECK((sw() & 0x6F) == 0x40, "402 starts in switch on disabled, sw=%04X", sw());

    uint32_t v = 0;
    CHECK(sdo_read(0x1000, 0, &v) == 0 && v == 0x00020192u, "device type %08X", v);
    CHECK(sdo_read(0x1018, 4, &v) == 0 && v == 0x12345678u, "serial %08X", v);
    CHECK(sdo_read(0x6502, 0, &v) == 0 && v == 0x3A5u, "supported modes %X", v);
    CHECK(sdo_read(0x1014, 0, &v) == 0 && v == 0x80u + NODE, "emcy cob %X", v);
    CHECK(sdo_read(0x1400, 1, &v) == 0 && v == 0x200u + NODE, "rpdo1 cob %X", v);
    CHECK(sdo_read(0x1403, 1, &v) == 0 && v == 0x80000500u + NODE, "rpdo4 disabled %X", v);
    CHECK(sdo_read(0x2000, 1, &v) == 0 && v == NODE, "motor can id = node id (%u)", v);
    CHECK(g_m[0].can_id == NODE, "motor configured with id %u", g_m[0].can_id);
    CHECK(sdo_read(0x2000, 3, &v) == 0 && v == 0x41200000u, "kp default 10.0");
    CHECK(sdo_read(0x60C2, 2, &v) == 0 && (int8_t)v == -3, "ip time index -3");

    CHECK(sdo_write(0x1017, 0, 250, 2) == 0, "write heartbeat");
    CHECK(sdo_read(0x1017, 0, &v) == 0 && v == 250, "heartbeat readback %u", v);
    CHECK(sdo_write(0x1000, 0, 1, 4) == CO_ABORT_READONLY, "device type read-only");
    CHECK(sdo_write(0x1234, 0, 1, 4) == CO_ABORT_NO_OBJECT, "no object");
    CHECK(sdo_write(0x1018, 9, 1, 4) == CO_ABORT_NO_SUB, "no sub");
    CHECK(sdo_write(0x1017, 0, 1, 4) == CO_ABORT_LEN_HIGH, "length too high");
    CHECK(sdo_write(0x6060, 0, 5, 1) == CO_ABORT_VALUE, "unsupported mode rejected");
    CHECK(sdo_write(0x6098, 0, 1, 1) == CO_ABORT_VALUE, "unsupported homing method");
    CHECK(sdo_write(0x2000, 3, 0x7FC00000u, 4) == CO_ABORT_VALUE, "NaN kp rejected");

    /* block download 은 미지원 */
    uint8_t blk[8] = {0xC6, 0x17, 0x10, 0, 2, 0, 0, 0};
    tx_clear();
    rx(0x600 + NODE, 8, blk);
    const can_frame_t *r = tx_last(0x580 + NODE);
    CHECK(r && r->data[0] == 0x80 && le32(&r->data[4]) == CO_ABORT_CMD, "block transfer aborted");
}

static void test_sdo_segmented(void) {
    node_init();
    /* 업로드: 0x1008 장치 이름 */
    uint8_t d[8] = {0x40, 0x08, 0x10, 0, 0, 0, 0, 0};
    tx_clear();
    rx(0x600 + NODE, 8, d);
    const can_frame_t *r = tx_last(0x580 + NODE);
    CHECK(r && r->data[0] == 0x41, "segmented upload initiate");
    uint32_t size = r ? le32(&r->data[4]) : 0;
    char name[64] = {0};
    uint32_t got = 0;
    uint8_t toggle = 0;
    for (int seg = 0; seg < 20 && got < size; seg++) {
        uint8_t q[8] = {(uint8_t)(0x60 | (toggle << 4)), 0, 0, 0, 0, 0, 0, 0};
        tx_clear();
        rx(0x600 + NODE, 8, q);
        r = tx_last(0x580 + NODE);
        if (!r || (r->data[0] & 0xE0) != 0x00 || ((r->data[0] >> 4) & 1) != toggle) break;
        unsigned cnt = 7 - ((r->data[0] >> 1) & 7);
        memcpy(&name[got], &r->data[1], cnt);
        got += cnt;
        toggle ^= 1;
        if (r->data[0] & 1) break;
    }
    CHECK(got == size && strcmp(name, CO_DEVICE_NAME) == 0, "device name '%s'", name);

    /* 다운로드: 0x1017 = 0x0123 (segmented) */
    uint8_t init[8] = {0x21, 0x17, 0x10, 0, 2, 0, 0, 0};
    tx_clear();
    rx(0x600 + NODE, 8, init);
    r = tx_last(0x580 + NODE);
    CHECK(r && r->data[0] == 0x60, "segmented download initiate");
    uint8_t seg[8] = {(uint8_t)((5 << 1) | 1), 0x23, 0x01, 0, 0, 0, 0, 0};
    tx_clear();
    rx(0x600 + NODE, 8, seg);
    r = tx_last(0x580 + NODE);
    CHECK(r && r->data[0] == 0x20, "segment ack");
    CHECK(g_n.od.hb_producer == 0x0123, "segmented write value %X", g_n.od.hb_producer);

    /* 토글 오류 */
    tx_clear();
    rx(0x600 + NODE, 8, init);
    seg[0] = (uint8_t)(0x10 | (5 << 1) | 1);
    tx_clear();
    rx(0x600 + NODE, 8, seg);
    r = tx_last(0x580 + NODE);
    CHECK(r && r->data[0] == 0x80 && le32(&r->data[4]) == CO_ABORT_TOGGLE, "toggle error");
}

static void test_pdo_remap_and_sync(void) {
    node_init();
    /* Lely / python-canopen 이 쓰는 표준 재매핑 절차 */
    CHECK(sdo_write(0x1A00, 1, 0x60640020, 4) == CO_ABORT_STATE, "entry write needs sub0=0");
    CHECK(sdo_write(0x1800, 1, 0x80000180u + NODE, 4) == 0, "disable TPDO1");
    CHECK(sdo_write(0x1A00, 0, 0, 1) == 0, "clear mapping");
    CHECK(sdo_write(0x1A00, 1, 0x60410010, 4) == 0, "map statusword");
    CHECK(sdo_write(0x1A00, 2, 0x60640020, 4) == 0, "map position");
    CHECK(sdo_write(0x1A00, 3, 0x10000020, 4) == CO_ABORT_NO_MAP, "device type not mappable");
    CHECK(sdo_write(0x1A00, 3, 0x60400010, 4) == CO_ABORT_NO_MAP, "controlword not TPDO-mappable");
    CHECK(sdo_write(0x1A00, 3, 0x606C0020, 4) == 0, "map velocity");
    CHECK(sdo_write(0x1A00, 4, 0x60640020, 4) == 0, "map 4th entry");
    CHECK(sdo_write(0x1A00, 0, 4, 1) == CO_ABORT_MAP_LEN, "more than 64 bits rejected");
    CHECK(sdo_write(0x1A00, 0, 2, 1) == 0, "enable 2 entries");
    CHECK(sdo_write(0x1800, 2, 1, 1) == 0, "transmission type 1 (every SYNC)");
    CHECK(sdo_write(0x1800, 2, 252, 1) == CO_ABORT_VALUE, "RTR-only type rejected");
    CHECK(sdo_write(0x1800, 1, 0x180u + NODE, 4) == 0, "enable TPDO1");
    CHECK(sdo_write(0x1800, 1, 0x190u + NODE, 4) == CO_ABORT_VALUE, "cannot change COB-ID while valid");
    CHECK(sdo_write(0x1803, 1, 0x80000000u | 0x700, 4) == 0, "restricted id ok while disabled");
    CHECK(sdo_write(0x1803, 1, 0x701, 4) == CO_ABORT_VALUE, "restricted id rejected when enabling");

    g_m[0].fb.pos_rad = 0.25f;
    step_ms(1);
    nmt(0x01);
    CHECK(g_n.nmt == CO_NMT_OPERATIONAL, "operational");
    tx_clear();
    step_ms(1);
    CHECK(tx_last(0x180 + NODE) == NULL, "sync TPDO not sent without SYNC");
    rx(0x080, 0, NULL);
    const can_frame_t *p = tx_last(0x180 + NODE);
    CHECK(p && p->dlc == 6, "TPDO1 on SYNC (dlc %d)", p ? p->dlc : -1);
    if (p) {
        uint16_t s = (uint16_t)(p->data[0] | (p->data[1] << 8));
        int32_t pos = (int32_t)le32(&p->data[2]);
        CHECK((s & 0x6F) == 0x40, "statusword in TPDO %04X", s);
        CHECK(pos == 2500, "position in TPDO %d", pos);
    }
    /* TPDO2 (기본: 이벤트 방식) 은 값이 바뀌면 inhibit 간격으로 */
    tx_clear();
    g_m[0].fb.pos_rad = 0.5f;
    step_ms(15); /* inhibit time 10 ms */
    CHECK(tx_last(0x280 + NODE) != NULL, "event TPDO2 sent on change");

    /* NMT reset communication: 매핑 기본값 복원 + boot-up */
    tx_clear();
    nmt(0x82);
    CHECK(g_n.od.tpdo_map[0].count == 2 && g_n.od.tpdo_map[0].map[1] == 0x60610008, "default TPDO1 mapping restored");
    CHECK(g_n.od.tpdo_comm[0].trans_type == 255, "default transmission type restored");
    CHECK(tx_last(0x700 + NODE) && tx_last(0x700 + NODE)->data[0] == 0, "boot-up after reset comm");
}

static void enable_operation(int8_t mode) {
    rpdo1(0x06, mode);
    rpdo1(0x07, mode);
    rpdo1(0x0F, mode);
}

static void test_state_machine_csp(void) {
    node_init();
    g_m[0].fb.pos_rad = 0.5f;
    step_ms(1);
    nmt(0x01);

    rpdo1(0x06, 8);
    CHECK((sw() & 0x6F) == 0x21, "ready to switch on %04X", sw());
    CHECK(g_n.od.mode_display == 8, "mode display 8");
    rpdo1(0x07, 8);
    CHECK((sw() & 0x6F) == 0x23, "switched on %04X", sw());
    CHECK(g_m[0].last == CALL_DISABLE, "motor still disabled before enable");
    rpdo1(0x0F, 8);
    CHECK((sw() & 0x6F) == 0x27, "operation enabled %04X", sw());
    CHECK(sw() & 0x1000, "CSP target used bit");
    CHECK(sw() & 0x0010, "voltage enabled (motor responding)");
    CHECK(g_m[0].last == CALL_MIT && fabsf(g_m[0].a[0] - 0.5f) < 1e-4f, "holds actual position at enable (%f)", g_m[0].a[0]);
    CHECK(g_n.od.target_pos == 5000, "target initialized to actual");

    /* RPDO2: 목표 위치 1.0 rad */
    uint8_t d[8] = {0x10, 0x27, 0, 0, 0, 0, 0, 0}; /* 10000 */
    rx(0x300 + NODE, 8, d);
    CHECK(g_m[0].last == CALL_MIT && fabsf(g_m[0].a[0] - 1.0f) < 1e-4f, "CSP target 1.0 rad (%f)", g_m[0].a[0]);
    CHECK(fabsf(g_m[0].a[2] - 10.0f) < 1e-4f && fabsf(g_m[0].a[3] - 0.5f) < 1e-4f, "default kp/kd");

    /* 운전 중 모터 ID 변경 금지 */
    CHECK(sdo_write(0x2000, 1, 9, 1) == CO_ABORT_STATE, "motor id locked while enabled");

    /* 0x0F -> 0x07: Switched on, 모터 정지 */
    rpdo1(0x07, 8);
    CHECK((sw() & 0x6F) == 0x23 && g_m[0].last == CALL_DISABLE, "disable operation stops motor");
    /* 0x0F 다시 -> 0x02 quick stop -> switch on disabled */
    rpdo1(0x0F, 8);
    rpdo1(0x02, 8);
    CHECK((sw() & 0x6F) == 0x40 && g_m[0].last == CALL_DISABLE, "quick stop -> switch on disabled");
}

static void test_fault(void) {
    node_init();
    step_ms(1);
    nmt(0x01);
    enable_operation(8);
    CHECK(cia402_state(&g_n) == CIA402_OPERATION_ENABLED, "enabled");

    tx_clear();
    g_m[0].fb.err = 2; /* 과전류 */
    step_ms(1);
    CHECK(cia402_state(&g_n) == CIA402_FAULT && (sw() & 0x6F) == 0x08, "fault state %04X", sw());
    CHECK(g_n.od.error_code == 0x2310, "error code %04X", g_n.od.error_code);
    CHECK(g_m[0].last == CALL_DISABLE, "motor stopped on fault");
    const can_frame_t *e = tx_last(0x080 + NODE);
    CHECK(e && e->data[0] == 0x10 && e->data[1] == 0x23 && e->data[2] == (CO_ERR_GENERIC | CO_ERR_CURRENT) && e->data[3] == 2,
          "EMCY frame");

    rpdo1(0x00, 8);
    rpdo1(0x80, 8);
    CHECK(cia402_state(&g_n) == CIA402_FAULT, "fault reset ignored while motor error remains");

    g_m[0].fb.err = 0;
    step_ms(1);
    tx_clear();
    rpdo1(0x00, 8);
    rpdo1(0x80, 8);
    CHECK(cia402_state(&g_n) == CIA402_SWITCH_ON_DISABLED, "fault reset -> switch on disabled");
    e = tx_last(0x080 + NODE);
    CHECK(e && e->data[0] == 0 && e->data[1] == 0 && e->data[2] == 0, "EMCY error reset");
    CHECK(g_n.od.error_code == 0, "error code cleared");

    /* 운전 중 모터 피드백이 끊기면 fault (0x2000:08 = 200 ms) */
    enable_operation(8);
    g_m[0].fb.age_ms = 500;
    step_ms(150);
    CHECK(cia402_state(&g_n) == CIA402_OPERATION_ENABLED, "grace period after enable");
    step_ms(100);
    CHECK(cia402_state(&g_n) == CIA402_FAULT && g_n.od.error_code == 0xFF01, "feedback timeout fault %04X", g_n.od.error_code);
}

static void test_heartbeat_consumer(void) {
    node_init();
    step_ms(1);
    CHECK(sdo_write(0x1016, 1, (0x7Fu << 16) | 100u, 4) == 0, "configure heartbeat consumer");
    nmt(0x01);
    enable_operation(8);
    uint8_t hb = 0x05;
    rx(0x77F, 1, &hb);
    step_ms(80);
    rx(0x77F, 1, &hb);
    step_ms(80);
    CHECK(cia402_state(&g_n) == CIA402_OPERATION_ENABLED, "heartbeat alive");
    tx_clear();
    step_ms(50);
    CHECK(cia402_state(&g_n) == CIA402_FAULT && g_n.od.error_code == 0x8130, "heartbeat timeout fault %04X",
          g_n.od.error_code);
    const can_frame_t *e = tx_last(0x080 + NODE);
    CHECK(e && e->data[0] == 0x30 && e->data[1] == 0x81, "EMCY 0x8130");

    /* NMT stop 도 0x6007 = 1 이면 fault */
    node_init();
    step_ms(1);
    nmt(0x01);
    enable_operation(8);
    nmt(0x02);
    CHECK(cia402_state(&g_n) == CIA402_FAULT, "NMT stop while enabled -> fault");
}

static void test_profile_position_and_homing(void) {
    node_init();
    step_ms(1);
    nmt(0x01);
    CHECK(sdo_write(0x6060, 0, 1, 1) == 0, "mode PP");
    CHECK(sdo_write(0x6040, 0, 0x06, 2) == 0 && sdo_write(0x6040, 0, 0x07, 2) == 0 && sdo_write(0x6040, 0, 0x0F, 2) == 0,
          "enable via SDO");
    CHECK(cia402_state(&g_n) == CIA402_OPERATION_ENABLED, "PP enabled");
    CHECK(g_m[0].last == CALL_PV && fabsf(g_m[0].a[0]) < 1e-3f, "PP holds position before first set-point");

    CHECK(sdo_write(0x607A, 0, 10000, 4) == 0, "target 1 rad");
    CHECK(g_m[0].last == CALL_PV && fabsf(g_m[0].a[0]) < 1e-3f, "target alone does not move (needs bit4)");
    sdo_write(0x6040, 0, 0x1F, 2);
    CHECK(g_m[0].last == CALL_PV && fabsf(g_m[0].a[0] - 57.29578f) < 0.01f, "PP command %f deg", g_m[0].a[0]);
    CHECK(fabsf(g_m[0].a[1] - ak_rad_s_to_erpm(5.0f)) < 1.0f, "profile velocity 5 rad/s -> %f ERPM", g_m[0].a[1]);
    CHECK(sw() & 0x1000, "set-point acknowledge");
    sdo_write(0x6040, 0, 0x0F, 2);
    CHECK(!(sw() & 0x1000), "ack cleared with bit4");
    CHECK(!(sw() & 0x0400), "not yet reached");

    g_m[0].fb.pos_rad = 1.0f;
    step_ms(5);
    CHECK(!(sw() & 0x0400), "inside window but window time not elapsed");
    step_ms(10);
    CHECK(sw() & 0x0400, "target reached");

    /* 상대 이동: +0.5 rad */
    sdo_write(0x607A, 0, 5000, 4);
    sdo_write(0x6040, 0, 0x5F, 2);
    CHECK(fabsf(g_m[0].a[0] - 85.94367f) < 0.01f, "relative move %f deg", g_m[0].a[0]);
    sdo_write(0x6040, 0, 0x4F, 2);

    /* 호밍 (방법 37): 현재 위치를 원점으로 */
    CHECK(sdo_write(0x6060, 0, 6, 1) == 0, "mode homing");
    sdo_write(0x6040, 0, 0x1F, 2);
    CHECK(g_m[0].origin_calls == 1, "origin command sent");
    step_ms(5);
    CHECK(!(sw() & 0x1000), "homing in progress");
    g_m[0].fb.pos_rad = 0.0f;
    step_ms(30);
    CHECK((sw() & 0x1400) == 0x1400, "homing attained + target reached %04X", sw());
    CHECK(g_n.od.pos_actual == 0, "position zero after homing");

    /* CST: 목표 토크 500 permille = 0.65 Nm, max torque 로 제한 */
    CHECK(sdo_write(0x6060, 0, 10, 1) == 0, "mode CST");
    sdo_write(0x6071, 0, 500, 2);
    CHECK(g_m[0].last == CALL_MIT && fabsf(g_m[0].a[4] - 0.65f) < 1e-4f && g_m[0].a[2] == 0.0f, "CST torque %f", g_m[0].a[4]);
    sdo_write(0x6072, 0, 100, 2);
    CHECK(fabsf(g_m[0].a[4] - 0.13f) < 1e-4f, "max torque clamp %f", g_m[0].a[4]);

    /* CSV: 목표 속도 2 rad/s */
    CHECK(sdo_write(0x6060, 0, 9, 1) == 0, "mode CSV");
    sdo_write(0x60FF, 0, 2000, 4);
    CHECK(g_m[0].last == CALL_VEL && fabsf(g_m[0].a[0] - ak_rad_s_to_erpm(2.0f)) < 1.0f, "CSV %f ERPM", g_m[0].a[0]);
}

/* ---------------- slcan ---------------- */
static char g_out[512];
static size_t g_outlen;
static int g_open_calls, g_close_calls, g_frames;
static can_frame_t g_last_frame;

static bool io_write(const char *s, size_t len, bool must) {
    (void)must;
    if (g_outlen + len < sizeof(g_out)) {
        memcpy(&g_out[g_outlen], s, len);
        g_outlen += len;
    }
    return true;
}
static void io_frame(const can_frame_t *f) {
    g_frames++;
    g_last_frame = *f;
}
static void io_open(void) {
    g_open_calls++;
}
static void io_close(void) {
    g_close_calls++;
}

static void test_slcan(void) {
    can_frame_t f;
    bool rtr;
    CHECK(slcan_parse_frame("t1238AABBCCDDEEFF0011", 21, &f, &rtr) && f.id == 0x123 && f.dlc == 8 && !f.ext &&
              f.data[0] == 0xAA && f.data[7] == 0x11 && !rtr,
          "parse std frame");
    CHECK(slcan_parse_frame("T1FFFFFFF0", 10, &f, &rtr) && f.ext && f.id == 0x1FFFFFFF && f.dlc == 0, "parse ext frame");
    CHECK(slcan_parse_frame("r7FF0", 5, &f, &rtr) && rtr, "parse rtr");
    CHECK(!slcan_parse_frame("t12", 3, &f, &rtr), "reject short");
    CHECK(!slcan_parse_frame("t8001", 5, &f, &rtr), "reject std id > 0x7FF");
    CHECK(!slcan_parse_frame("t1239", 5, &f, &rtr), "reject dlc 9");

    char buf[32];
    memset(&f, 0, sizeof(f));
    f.id = 0x705;
    f.dlc = 1;
    size_t n = slcan_format_frame(&f, buf, false, 0);
    CHECK(n == 8 && memcmp(buf, "t705100\r", 8) == 0, "format boot-up");
    f.ext = true;
    f.id = 0x2905;
    f.dlc = 2;
    f.data[0] = 0xAB;
    f.data[1] = 0x01;
    n = slcan_format_frame(&f, buf, true, 0x1234);
    CHECK(n == 19 && memcmp(buf, "T000029052AB011234\r", 19) == 0, "format ext + timestamp '%.*s'", (int)n, buf);

    slcan_io_t io = {io_write, io_frame, io_open, io_close, NULL};
    slcan_init(&io);
    g_outlen = 0;
    can_frame_t out;
    memset(&out, 0, sizeof(out));
    out.id = 0x181;
    CHECK(!slcan_send(&out, true), "send fails while closed");
    const char *in = "C\rS8\rt0000\rO\rt70517F\r"; /* 닫힌 상태의 t 명령은 BEL */
    slcan_feed(in, strlen(in));
    CHECK(g_open_calls == 1 && g_close_calls == 0, "open callback");
    CHECK(g_frames == 1 && g_last_frame.id == 0x705 && g_last_frame.data[0] == 0x7F, "frame delivered");
    CHECK(g_outlen == 6 && memcmp(g_out, "\r\r\a\rz\r", 6) == 0, "replies (%u bytes)", (unsigned)g_outlen);
    g_outlen = 0;
    CHECK(slcan_send(&out, false) && g_outlen == 6 && memcmp(g_out, "t1810\r", 6) == 0, "send frame");
    slcan_feed("C\r", 2);
    CHECK(g_close_calls == 1 && !slcan_is_open(), "close callback");
}

int main(void) {
    co_set_sender(sender);
    test_boot_and_sdo();
    test_sdo_segmented();
    test_pdo_remap_and_sync();
    test_state_machine_csp();
    test_fault();
    test_heartbeat_consumer();
    test_profile_position_and_homing();
    test_slcan();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
