/*
 * PC 용 단위 테스트: src/ak_proto.c 의 인코딩/디코딩을
 * CubeMars V3 매뉴얼 "CAN Port Control Command Examples" (모터 ID 0x68) 와 비교.
 *
 *   gcc -std=c99 -Wall -Isrc tests/test_ak_proto.c src/ak_proto.c -lm -o test_ak_proto
 *   ./test_ak_proto
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ak_proto.h"
#include "config.h"

static int g_fail, g_pass;

static void expect_frame(const char *name, const can_frame_t *f, bool ext, uint32_t id, uint8_t dlc,
                         const uint8_t *data) {
    bool ok = f->ext == ext && f->id == id && f->dlc == dlc && memcmp(f->data, data, dlc) == 0;
    if (ok) {
        g_pass++;
        return;
    }
    g_fail++;
    printf("FAIL %s: got %s %08X dlc=%u [", name, f->ext ? "E" : "S", (unsigned)f->id, f->dlc);
    for (int i = 0; i < f->dlc; i++) printf("%02X ", f->data[i]);
    printf("] want %s %08X dlc=%u [", ext ? "E" : "S", (unsigned)id, dlc);
    for (int i = 0; i < dlc; i++) printf("%02X ", data[i]);
    printf("]\n");
}

static void expect_near(const char *name, float got, float want, float tol) {
    if (fabsf(got - want) <= tol) {
        g_pass++;
        return;
    }
    g_fail++;
    printf("FAIL %s: got %.6f want %.6f (tol %.6f)\n", name, got, want, tol);
}

#define B(...) ((const uint8_t[]){__VA_ARGS__})

static void test_servo_examples(void) {
    can_frame_t f;
    ak_pack_duty(&f, 0x68, 0.2f);
    expect_frame("duty +0.2", &f, true, 0x068, 4, B(0x00, 0x00, 0x4E, 0x20));
    ak_pack_duty(&f, 0x68, -0.2f);
    expect_frame("duty -0.2", &f, true, 0x068, 4, B(0xFF, 0xFF, 0xB1, 0xE0));
    ak_pack_current(&f, 0x68, -4.0f);
    expect_frame("current -4A", &f, true, 0x168, 4, B(0xFF, 0xFF, 0xF0, 0x60));
    ak_pack_current(&f, 0x68, 4.0f);
    expect_frame("current +4A", &f, true, 0x168, 4, B(0x00, 0x00, 0x0F, 0xA0));
    ak_pack_brake(&f, 0x68, 4.0f);
    expect_frame("brake 4A", &f, true, 0x268, 4, B(0x00, 0x00, 0x0F, 0xA0));
    ak_pack_rpm(&f, 0x68, 5000.0f);
    expect_frame("rpm +5000", &f, true, 0x368, 4, B(0x00, 0x00, 0x13, 0x88));
    ak_pack_rpm(&f, 0x68, -5000.0f);
    expect_frame("rpm -5000", &f, true, 0x368, 4, B(0xFF, 0xFF, 0xEC, 0x78));
    ak_pack_pos(&f, 0x68, 600.0f);
    expect_frame("pos +600", &f, true, 0x468, 4, B(0x00, 0x5B, 0x8D, 0x80));
    ak_pack_pos(&f, 0x68, -600.0f);
    expect_frame("pos -600", &f, true, 0x468, 4, B(0xFF, 0xA4, 0x72, 0x80));
    ak_pack_pos_spd(&f, 0x68, 1000.0f, 10000.0f, 10000.0f);
    expect_frame("pos_spd +1000", &f, true, 0x668, 8, B(0x00, 0x98, 0x96, 0x80, 0x03, 0xE8, 0x03, 0xE8));
    ak_pack_pos_spd(&f, 0x68, -1000.0f, -10000.0f, 10000.0f);
    expect_frame("pos_spd -1000", &f, true, 0x668, 8, B(0xFF, 0x67, 0x69, 0x80, 0xFC, 0x18, 0x03, 0xE8));
    ak_pack_origin(&f, 0x68, 1);
    expect_frame("origin perm", &f, true, 0x568, 1, B(0x01));
    ak_pack_disable_v3(&f, 0x68);
    expect_frame("disable", &f, true, 0xF68, 0, B(0));

    /* 클램프: AK40-10 피크 전류 7.3 A */
    ak_pack_current(&f, 1, 100.0f);
    expect_frame("current clamp", &f, true, 0x101, 4, B(0x00, 0x00, 0x1C, 0x84)); /* 7300 */
}

static void test_mit_v3(void) {
    can_frame_t f;
    /* 매뉴얼 예: Kp=2, Kd=2 -> 01 06 66 ... / 속도 0, 토크 0 -> 7F F7 FF */
    ak_pack_mit(&f, AK_PROTO_V3, 0x68, 0.0f, 0.0f, 2.0f, 2.0f, 0.0f);
    expect_frame("mit v3 kp2 kd2 p0", &f, true, 0x868, 8, B(0x01, 0x06, 0x66, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF));

    /* 매뉴얼 토크 예의 앞 5바이트: kp=0 kd=0 p=0 -> 00 00 00 7F FF */
    ak_pack_mit(&f, AK_PROTO_V3, 0x68, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    expect_frame("mit v3 zero", &f, true, 0x868, 8, B(0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF));

    /* 위치 6 rad: 범위 ±AK_V3_P_MAX 로 직접 계산한 값과 비교 */
    ak_pack_mit(&f, AK_PROTO_V3, 0x68, 6.0f, 0.0f, 2.0f, 2.0f, 0.0f);
    uint32_t p = ((uint32_t)f.data[3] << 8) | f.data[4];
    expect_near("mit v3 p=6rad bits", (float)p, (6.0f + AK_V3_P_MAX) / (2 * AK_V3_P_MAX) * 65535.0f, 1.0f);

    /* 최대값에서 비트 넘침 없음 */
    ak_pack_mit(&f, AK_PROTO_V3, 1, 1e9f, 1e9f, 1e9f, 1e9f, 1e9f);
    expect_frame("mit v3 max clamp", &f, true, 0x801, 8, B(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF));
    ak_pack_mit(&f, AK_PROTO_V3, 1, -1e9f, -1e9f, -1e9f, -1e9f, -1e9f);
    expect_frame("mit v3 min clamp", &f, true, 0x801, 8, B(0, 0, 0, 0, 0, 0, 0, 0));
    ak_pack_mit(&f, AK_PROTO_V3, 1, NAN, NAN, NAN, NAN, NAN);
    expect_frame("mit v3 NaN->0", &f, true, 0x801, 8, B(0x00, 0x00, 0x00, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF));
}

static void test_mit_v2(void) {
    can_frame_t f;
    ak_pack_mit(&f, AK_PROTO_V2, 3, 0.0f, 0.0f, 2.0f, 2.0f, 0.0f);
    /* V2 순서: P16 V12 KP12 KD12 T12 */
    expect_frame("mit v2", &f, false, 3, 8, B(0x7F, 0xFF, 0x7F, 0xF0, 0x10, 0x66, 0x67, 0xFF));
    ak_pack_v2_special(&f, 3, AK_V2_ENTER);
    expect_frame("v2 enter", &f, false, 3, 8, B(0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC));
}

static void test_parse(void) {
    can_frame_t f = {0};
    ak_rx_t r;

    /* 0x29 상태: pos 180.0 deg, 14000 ERPM, 2.50 A, 35 C, err 0 */
    f.ext = true;
    f.id = (0x29u << 8) | 2;
    f.dlc = 8;
    const uint8_t d1[8] = {0x07, 0x08, 0x05, 0x78, 0x00, 0xFA, 35, 0};
    memcpy(f.data, d1, 8);
    bool ok = ak_parse(&f, &r);
    expect_near("parse status ok", ok && r.kind == AK_RX_STATUS && r.motor_id == 2, 1, 0);
    expect_near("parse pos", r.pos_rad, 3.14159265f, 1e-4f);
    expect_near("parse vel", r.vel_rad_s, 100.0f * 6.2831853f / 60.0f, 1e-3f); /* 14000/140 = 100 rpm */
    expect_near("parse cur", r.current_a, 2.5f, 1e-4f);
    expect_near("parse torque", r.torque_nm, 2.5f * AK_KT_OUT, 1e-4f);
    expect_near("parse temp", r.temp_c, 35, 0);

    /* 음수 값 */
    const uint8_t d2[8] = {0xF8, 0xF8, 0xFA, 0x88, 0xFF, 0x06, 0xEC, 7};
    memcpy(f.data, d2, 8);
    ak_parse(&f, &r);
    expect_near("parse neg pos", r.pos_rad, -180.0f * 3.14159265f / 180.0f, 1e-4f);
    expect_near("parse neg cur", r.current_a, -2.5f, 1e-4f);
    expect_near("parse neg temp", r.temp_c, -20, 0);
    expect_near("parse err", r.err, 7, 0);

    /* 0x2A 32bit 위치: 12345.67 deg */
    f.id = (0x2Au << 8) | 2;
    f.dlc = 4;
    int32_t v = 1234567;
    f.data[0] = (uint8_t)(v >> 24), f.data[1] = (uint8_t)(v >> 16), f.data[2] = (uint8_t)(v >> 8), f.data[3] = (uint8_t)v;
    ok = ak_parse(&f, &r);
    expect_near("parse pos32", ok && r.kind == AK_RX_POS32 ? r.pos_rad : -1, 12345.67f * 3.14159265f / 180.0f, 1e-2f);

    /* 0x2C 서보 시작 */
    f.id = (0x2Cu << 8) | 1;
    f.dlc = 4;
    ok = ak_parse(&f, &r);
    expect_near("parse servo start", ok && r.kind == AK_RX_SERVO_START && r.motor_id == 1, 1, 0);

    /* 모르는 확장 프레임 */
    f.id = (0x30u << 8) | 1;
    expect_near("parse unknown ext", ak_parse(&f, &r), 0, 0);

    /* V2 MIT 응답: id 3, 중간값 */
    f.ext = false;
    f.id = 0;
    f.dlc = 8;
    const uint8_t d3[8] = {3, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 40 + 30, 0};
    memcpy(f.data, d3, 8);
    ok = ak_parse(&f, &r);
    expect_near("parse v2 ok", ok && r.kind == AK_RX_MIT_V2 && r.motor_id == 3, 1, 0);
    expect_near("parse v2 pos", r.pos_rad, 0.0f, 1e-3f);
    expect_near("parse v2 vel", r.vel_rad_s, 0.0f, 0.02f);
    expect_near("parse v2 torque", r.torque_nm, 0.0f, 0.01f);
    expect_near("parse v2 temp", r.temp_c, 30, 0);
}

static void test_roundtrip(void) {
    /* V3 MIT 인코딩한 값을 매뉴얼의 uint_to_float 로 되돌렸을 때 오차가 1 LSB 이내 */
    const float ps[] = {-12.0f, -3.3f, 0.0f, 0.5f, 7.77f, 12.5f};
    for (unsigned k = 0; k < sizeof(ps) / sizeof(ps[0]); k++) {
        can_frame_t f;
        ak_pack_mit(&f, AK_PROTO_V3, 1, ps[k], 0, 0, 0, 0);
        uint32_t p = ((uint32_t)f.data[3] << 8) | f.data[4];
        float back = (float)p * (2 * AK_V3_P_MAX) / 65535.0f - AK_V3_P_MAX;
        expect_near("mit p roundtrip", back, ps[k], 2 * AK_V3_P_MAX / 65535.0f);
    }
    expect_near("erpm<->rad/s", ak_erpm_to_rad_s(ak_rad_s_to_erpm(3.0f)), 3.0f, 1e-4f);
    expect_near("1 rad/s in erpm", ak_rad_s_to_erpm(1.0f), 60.0f / 6.2831853f * 140.0f, 0.01f);
}

int main(void) {
    test_servo_examples();
    test_mit_v3();
    test_mit_v2();
    test_parse();
    test_roundtrip();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
