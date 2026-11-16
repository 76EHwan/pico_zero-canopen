/*
 * co_od.c - 오브젝트 딕셔너리 테이블 (index 오름차순)
 *
 * 단위는 config.h 의 "CiA 402 단위" 참고.
 */
#include "co_od.h"

#include <string.h>

#include "config.h"

#define FLD(field) ((uint16_t)offsetof(co_od_data_t, field))

/* 저장공간이 있는 서브 엔트리 */
#define S(sub, type, flags, field, name, def) {(sub), (type), (flags), FLD(field), (name), (uint32_t)(def), NULL}
/* 상수 서브 엔트리 */
#define C(sub, type, name, def) {(sub), (type), OD_R | OD_CONST, 0, (name), (uint32_t)(def), NULL}
/* 상수 문자열 */
#define STR(sub, name, str) {(sub), CO_VSTR, OD_R | OD_CONST, 0, (name), 0, (str)}

#define SUBS(...) ((const od_sub_t[]){__VA_ARGS__})
#define NSUB(...) ((uint8_t)(sizeof((const od_sub_t[]){__VA_ARGS__}) / sizeof(od_sub_t)))
#define OBJ(index, code, name, ...) {(index), (code), NSUB(__VA_ARGS__), (name), SUBS(__VA_ARGS__)}

#define VAR(index, name, type, flags, field, def) OBJ(index, OD_VAR, name, S(0, type, flags, field, name, def))
#define VARC(index, name, type, def) OBJ(index, OD_VAR, name, C(0, type, name, def))
#define VARS(index, name, str) OBJ(index, OD_VAR, name, STR(0, name, str))

#define HIGHEST(n) C(0, CO_U8, "Highest sub-index supported", n)

#define RPDO_COMM(i, cob)                                                                            \
    OBJ(0x1400 + (i), OD_RECORD, "RPDO communication parameter", HIGHEST(2),                         \
        S(1, CO_U32, OD_RW | OD_NODEID, rpdo_comm[i].cob_id, "COB-ID used by RPDO", cob),            \
        S(2, CO_U8, OD_RW, rpdo_comm[i].trans_type, "Transmission type", 255))

#define TPDO_COMM(i, cob, inh_, evt_)                                                                \
    OBJ(0x1800 + (i), OD_RECORD, "TPDO communication parameter", HIGHEST(5),                         \
        S(1, CO_U32, OD_RW | OD_NODEID, tpdo_comm[i].cob_id, "COB-ID used by TPDO", cob),            \
        S(2, CO_U8, OD_RW, tpdo_comm[i].trans_type, "Transmission type", 255),                       \
        S(3, CO_U16, OD_RW, tpdo_comm[i].inhibit, "Inhibit time", inh_),                             \
        S(5, CO_U16, OD_RW, tpdo_comm[i].event_timer, "Event timer", evt_))

#define PDO_MAP(base, arr, i, name, cnt, m1, m2, m3, m4)                                             \
    OBJ((base) + (i), OD_RECORD, name,                                                               \
        S(0, CO_U8, OD_RW, arr[i].count, "Number of mapped application objects in PDO", cnt),        \
        S(1, CO_U32, OD_RW, arr[i].map[0], "Application object 1", m1),                              \
        S(2, CO_U32, OD_RW, arr[i].map[1], "Application object 2", m2),                              \
        S(3, CO_U32, OD_RW, arr[i].map[2], "Application object 3", m3),                              \
        S(4, CO_U32, OD_RW, arr[i].map[3], "Application object 4", m4),                              \
        S(5, CO_U32, OD_RW, arr[i].map[4], "Application object 5", 0),                               \
        S(6, CO_U32, OD_RW, arr[i].map[5], "Application object 6", 0),                               \
        S(7, CO_U32, OD_RW, arr[i].map[6], "Application object 7", 0),                               \
        S(8, CO_U32, OD_RW, arr[i].map[7], "Application object 8", 0))

#define RPDO_MAP(i, cnt, m1, m2, m3, m4) PDO_MAP(0x1600, rpdo_map, i, "RPDO mapping parameter", cnt, m1, m2, m3, m4)
#define TPDO_MAP(i, cnt, m1, m2, m3, m4) PDO_MAP(0x1A00, tpdo_map, i, "TPDO mapping parameter", cnt, m1, m2, m3, m4)

#define PDO_OFF 0x80000000u /* COB-ID bit31: PDO 꺼짐 */

/* REAL32 기본값 (IEEE754 비트) */
#define F32_10_0 0x41200000u
#define F32_0_5 0x3F000000u

/* CiA 402 지원 모드 비트: PP(1) PV(3) HM(6) CSP(8) CSV(9) CST(10) */
#define SUPPORTED_MODES ((1u << 0) | (1u << 2) | (1u << 5) | (1u << 7) | (1u << 8) | (1u << 9))

const od_obj_t co_od_table[] = {
    /* ---------------- CiA 301 통신 영역 ---------------- */
    VARC(0x1000, "Device type", CO_U32, 0x00020192u), /* 프로파일 402, 서보 드라이브 */
    VAR(0x1001, "Error register", CO_U8, OD_R | OD_TPDO, error_register, 0),
    VAR(0x1005, "COB-ID SYNC", CO_U32, OD_RW, cob_sync, 0x80),
    VARS(0x1008, "Manufacturer device name", CO_DEVICE_NAME),
    VARS(0x1009, "Manufacturer hardware version", CO_HW_VERSION),
    VARS(0x100A, "Manufacturer software version", FW_VERSION),
    VAR(0x1014, "COB-ID EMCY", CO_U32, OD_R | OD_NODEID, cob_emcy, 0x80),
    OBJ(0x1016, OD_ARRAY, "Consumer heartbeat time", HIGHEST(1),
        S(1, CO_U32, OD_RW, hb_consumer, "Consumer heartbeat time 1", 0)),
    VAR(0x1017, "Producer heartbeat time", CO_U16, OD_RW, hb_producer, CO_HEARTBEAT_MS),
    OBJ(0x1018, OD_RECORD, "Identity object", HIGHEST(4),
        C(1, CO_U32, "Vendor-ID", CO_VENDOR_ID),
        C(2, CO_U32, "Product code", CO_PRODUCT_CODE),
        C(3, CO_U32, "Revision number", CO_REVISION),
        S(4, CO_U32, OD_R, serial, "Serial number", 0)),

    RPDO_COMM(0, 0x200),
    RPDO_COMM(1, 0x300),
    RPDO_COMM(2, 0x400),
    RPDO_COMM(3, PDO_OFF | 0x500),

    /* RPDO1: controlword + 모드 / RPDO2: 목표 위치 + 목표 속도 / RPDO3: 목표 토크 + 토크 오프셋 */
    RPDO_MAP(0, 2, 0x60400010, 0x60600008, 0, 0),
    RPDO_MAP(1, 2, 0x607A0020, 0x60FF0020, 0, 0),
    RPDO_MAP(2, 2, 0x60710010, 0x60B20010, 0, 0),
    RPDO_MAP(3, 0, 0, 0, 0, 0),

    /* 이벤트 방식(255): 값이 바뀌면 전송 (inhibit 로 최소 간격), event timer 마다 강제 전송 */
    TPDO_COMM(0, 0x180, 0, 100),
    TPDO_COMM(1, 0x280, 100, 100),
    TPDO_COMM(2, 0x380, 100, 100),
    TPDO_COMM(3, PDO_OFF | 0x480, 0, 0),

    /* TPDO1: statusword + 모드 표시 / TPDO2: 위치 + 속도 / TPDO3: 토크, 에러 코드, 온도, 모터 에러 */
    TPDO_MAP(0, 2, 0x60410010, 0x60610008, 0, 0),
    TPDO_MAP(1, 2, 0x60640020, 0x606C0020, 0, 0),
    TPDO_MAP(2, 4, 0x60770010, 0x603F0010, 0x20000510, 0x20000608),
    TPDO_MAP(3, 0, 0, 0, 0, 0),

    /* ---------------- 제조사 영역 ---------------- */
    OBJ(0x2000, OD_RECORD, "AK motor", HIGHEST(8),
        S(1, CO_U8, OD_RW | OD_NODEID, motor_can_id, "Motor CAN ID", 0),
        S(2, CO_U8, OD_RW, motor_proto, "Motor protocol", MOTOR_DEFAULT_PROTO),
        S(3, CO_R32, OD_RW, mit_kp, "MIT Kp", F32_10_0),
        S(4, CO_R32, OD_RW, mit_kd, "MIT Kd", F32_0_5),
        S(5, CO_I16, OD_R | OD_TPDO, motor_temp, "Motor temperature", 0),
        S(6, CO_U8, OD_R | OD_TPDO, motor_err, "Motor error code", 0),
        S(7, CO_U16, OD_R | OD_TPDO, fb_age, "Feedback age", 0xFFFF),
        S(8, CO_U16, OD_RW, fb_timeout, "Feedback timeout", 200)),

    /* ---------------- CiA 402 드라이브 프로파일 ---------------- */
    VAR(0x6007, "Abort connection option code", CO_I16, OD_RW, abort_conn, 1),
    VAR(0x603F, "Error code", CO_U16, OD_R | OD_TPDO, error_code, 0),
    VAR(0x6040, "Controlword", CO_U16, OD_RW | OD_RPDO, controlword, 0),
    VAR(0x6041, "Statusword", CO_U16, OD_R | OD_TPDO, statusword, 0),
    VARC(0x605A, "Quick stop option code", CO_I16, 0),
    VAR(0x6060, "Modes of operation", CO_I8, OD_RW | OD_RPDO, mode, 0),
    VAR(0x6061, "Modes of operation display", CO_I8, OD_R | OD_TPDO, mode_display, 0),
    VAR(0x6064, "Position actual value", CO_I32, OD_R | OD_TPDO, pos_actual, 0),
    VAR(0x6067, "Position window", CO_U32, OD_RW, pos_window, 100),
    VAR(0x6068, "Position window time", CO_U16, OD_RW, pos_window_time, 10),
    VAR(0x606C, "Velocity actual value", CO_I32, OD_R | OD_TPDO, vel_actual, 0),
    VAR(0x606D, "Velocity window", CO_U16, OD_RW, vel_window, 100),
    VAR(0x606E, "Velocity window time", CO_U16, OD_RW, vel_window_time, 10),
    VAR(0x6071, "Target torque", CO_I16, OD_RW | OD_RPDO, target_torque, 0),
    VAR(0x6072, "Max torque", CO_U16, OD_RW, max_torque, AK_MAX_TORQUE_PERMILLE),
    VARC(0x6075, "Motor rated current", CO_U32, AK_RATED_CURRENT_MA),
    VARC(0x6076, "Motor rated torque", CO_U32, AK_RATED_TORQUE_MNM),
    VAR(0x6077, "Torque actual value", CO_I16, OD_R | OD_TPDO, torque_actual, 0),
    VAR(0x6078, "Current actual value", CO_I16, OD_R | OD_TPDO, current_actual, 0),
    VAR(0x607A, "Target position", CO_I32, OD_RW | OD_RPDO, target_pos, 0),
    VAR(0x607C, "Home offset", CO_I32, OD_RW, home_offset, 0),
    VAR(0x6081, "Profile velocity", CO_U32, OD_RW | OD_RPDO, profile_vel, 5000),
    VAR(0x6083, "Profile acceleration", CO_U32, OD_RW | OD_RPDO, profile_acc, 20000),
    VAR(0x6084, "Profile deceleration", CO_U32, OD_RW | OD_RPDO, profile_dec, 20000),
    VAR(0x6098, "Homing method", CO_I8, OD_RW, homing_method, 37),
    VAR(0x60B1, "Velocity offset", CO_I32, OD_RW | OD_RPDO, vel_offset, 0),
    VAR(0x60B2, "Torque offset", CO_I16, OD_RW | OD_RPDO, torque_offset, 0),
    OBJ(0x60C2, OD_RECORD, "Interpolation time period", HIGHEST(2),
        S(1, CO_U8, OD_RW, ip_time_units, "Interpolation time period value", 1),
        S(2, CO_I8, OD_RW, ip_time_index, "Interpolation time index", (uint8_t)-3)),
    VAR(0x60FF, "Target velocity", CO_I32, OD_RW | OD_RPDO, target_vel, 0),
    VARC(0x6502, "Supported drive modes", CO_U32, SUPPORTED_MODES),
};

const size_t co_od_table_len = sizeof(co_od_table) / sizeof(co_od_table[0]);

const od_obj_t *od_find(uint16_t index) {
    for (size_t i = 0; i < co_od_table_len; i++) {
        if (co_od_table[i].index == index) return &co_od_table[i];
    }
    return NULL;
}

const od_sub_t *od_find_sub(const od_obj_t *o, uint8_t sub) {
    if (!o) return NULL;
    for (uint8_t i = 0; i < o->nsub; i++) {
        if (o->subs[i].sub == sub) return &o->subs[i];
    }
    return NULL;
}

const od_sub_t *od_lookup(uint16_t index, uint8_t sub) {
    return od_find_sub(od_find(index), sub);
}

size_t od_type_size(uint8_t type) {
    switch (type) {
    case CO_BOOL:
    case CO_I8:
    case CO_U8: return 1;
    case CO_I16:
    case CO_U16: return 2;
    case CO_I32:
    case CO_U32:
    case CO_R32: return 4;
    default: return 0;
    }
}

/* RP2040/RP2350(ARM) 과 PC(x86) 모두 little-endian 이라 CAN 바이트 순서와 같다 */
uint32_t od_get(const co_od_data_t *d, const od_sub_t *s) {
    if (s->flags & OD_CONST) return s->def;
    uint32_t v = 0;
    memcpy(&v, (const uint8_t *)d + s->offset, od_type_size(s->type));
    return v;
}

void od_set(co_od_data_t *d, const od_sub_t *s, uint32_t raw) {
    if (s->flags & OD_CONST) return;
    memcpy((uint8_t *)d + s->offset, &raw, od_type_size(s->type));
}

int32_t od_to_signed(uint32_t raw, uint8_t type) {
    switch (type) {
    case CO_I8: return (int8_t)(uint8_t)raw;
    case CO_I16: return (int16_t)(uint16_t)raw;
    default: return (int32_t)raw;
    }
}

void od_load_defaults(co_od_data_t *d, uint8_t node_id, uint16_t lo, uint16_t hi) {
    for (size_t i = 0; i < co_od_table_len; i++) {
        const od_obj_t *o = &co_od_table[i];
        if (o->index < lo || o->index > hi) continue;
        for (uint8_t k = 0; k < o->nsub; k++) {
            const od_sub_t *s = &o->subs[k];
            if ((s->flags & OD_CONST) || s->type == CO_VSTR) continue;
            uint32_t v = s->def;
            if (s->flags & OD_NODEID) v += node_id;
            od_set(d, s, v);
        }
    }
}
