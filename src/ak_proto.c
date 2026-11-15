/*
 * ak_proto.c - CubeMars AK 시리즈 CAN 프로토콜 인코딩/디코딩
 *
 * 근거 문서:
 *  - AK Series Motor Module V3 Driver Software User Manual (V1.0.1, AK40-10 포함)
 *  - AK Series Module Driver Manual V1.0.18 (AK 2.0, AK40-10 포함)
 */
#include "ak_proto.h"

#include <math.h>
#include <string.h>

#include "config.h"

#define AK_TWO_PI 6.28318530717958647692f
#define AK_DEG2RAD (AK_TWO_PI / 360.0f)

static const ak_mit_range_t k_range_v3 = {AK_V3_P_MAX, AK_V3_V_MAX, AK_V3_T_MAX, AK_KP_MAX, AK_KD_MAX};
static const ak_mit_range_t k_range_v2 = {AK_V2_P_MAX, AK_V2_V_MAX, AK_V2_T_MAX, AK_KP_MAX, AK_KD_MAX};

const ak_mit_range_t *ak_mit_range(ak_proto_t proto) {
    return proto == AK_PROTO_V2 ? &k_range_v2 : &k_range_v3;
}

/* ---------------------------------------------------------------- */
/* 헬퍼                                                              */
/* ---------------------------------------------------------------- */

/* NaN 은 0 으로 취급한 뒤 [lo, hi] 로 자른다 */
static float clampf(float x, float lo, float hi) {
    if (x != x) x = 0.0f;
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static int32_t round_i32(float x) {
    return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

/* 매뉴얼의 float_to_uint 와 동일 (스케일 (2^bits - 1) / span, 버림) */
static uint32_t f2u(float x, float lo, float hi, unsigned bits) {
    const uint32_t max = (1u << bits) - 1u;
    x = clampf(x, lo, hi);
    if (x >= hi) return max; /* float 오차로 max-1 이 되는 것 방지 */
    uint32_t v = (uint32_t)((x - lo) * ((float)max / (hi - lo)));
    return v > max ? max : v;
}

/* 매뉴얼의 uint_to_float 와 동일 */
static float u2f(uint32_t v, float lo, float hi, unsigned bits) {
    return (float)v * (hi - lo) / (float)((1u << bits) - 1u) + lo;
}

static void put_be32(uint8_t *b, int32_t v) {
    uint32_t u = (uint32_t)v;
    b[0] = (uint8_t)(u >> 24);
    b[1] = (uint8_t)(u >> 16);
    b[2] = (uint8_t)(u >> 8);
    b[3] = (uint8_t)u;
}

static void put_be16(uint8_t *b, int16_t v) {
    uint16_t u = (uint16_t)v;
    b[0] = (uint8_t)(u >> 8);
    b[1] = (uint8_t)u;
}

static int16_t get_be16(const uint8_t *b) {
    return (int16_t)(uint16_t)(((uint16_t)b[0] << 8) | b[1]);
}

static int32_t get_be32(const uint8_t *b) {
    return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3]);
}

static void ext_frame(can_frame_t *f, uint32_t pkt, uint8_t id, uint8_t dlc) {
    memset(f, 0, sizeof(*f));
    f->ext = true;
    f->id = (pkt << 8) | id;
    f->dlc = dlc;
}

float ak_erpm_to_rad_s(float erpm) {
    return erpm / ((float)AK_POLE_PAIRS * AK_GEAR_RATIO) * (AK_TWO_PI / 60.0f);
}

float ak_rad_s_to_erpm(float rad_s) {
    return rad_s * (60.0f / AK_TWO_PI) * ((float)AK_POLE_PAIRS * AK_GEAR_RATIO);
}

/* ---------------------------------------------------------------- */
/* Servo 모드 명령                                                   */
/* ---------------------------------------------------------------- */

void ak_pack_duty(can_frame_t *f, uint8_t id, float duty) {
    ext_frame(f, AK_PKT_SET_DUTY, id, 4);
    put_be32(f->data, round_i32(clampf(duty, -AK_MAX_DUTY, AK_MAX_DUTY) * 100000.0f));
}

void ak_pack_current(can_frame_t *f, uint8_t id, float amps) {
    ext_frame(f, AK_PKT_SET_CURRENT, id, 4);
    put_be32(f->data, round_i32(clampf(amps, -AK_MAX_CURRENT_A, AK_MAX_CURRENT_A) * 1000.0f));
}

void ak_pack_brake(can_frame_t *f, uint8_t id, float amps) {
    ext_frame(f, AK_PKT_SET_CURRENT_BRAKE, id, 4);
    put_be32(f->data, round_i32(clampf(amps, 0.0f, AK_MAX_CURRENT_A) * 1000.0f));
}

void ak_pack_rpm(can_frame_t *f, uint8_t id, float erpm) {
    ext_frame(f, AK_PKT_SET_RPM, id, 4);
    put_be32(f->data, round_i32(clampf(erpm, -AK_MAX_ERPM, AK_MAX_ERPM)));
}

void ak_pack_pos(can_frame_t *f, uint8_t id, float deg) {
    ext_frame(f, AK_PKT_SET_POS, id, 4);
    put_be32(f->data, round_i32(clampf(deg, -AK_MAX_POS_DEG, AK_MAX_POS_DEG) * 10000.0f));
}

void ak_pack_pos_spd(can_frame_t *f, uint8_t id, float deg, float erpm, float erpm_per_s) {
    ext_frame(f, AK_PKT_SET_POS_SPD, id, 8);
    put_be32(&f->data[0], round_i32(clampf(deg, -AK_MAX_POS_DEG, AK_MAX_POS_DEG) * 10000.0f));
    /* 속도: int16, 1 단위 = 10 ERPM / 가속도: int16 (0~32767), 1 단위 = 10 ERPM/s.
     * 가속도는 크기로 취급 (음수가 오면 절대값) */
    put_be16(&f->data[4], (int16_t)round_i32(clampf(erpm, -327670.0f, 327670.0f) / 10.0f));
    put_be16(&f->data[6], (int16_t)round_i32(clampf(fabsf(erpm_per_s), 0.0f, 327670.0f) / 10.0f));
}

void ak_pack_origin(can_frame_t *f, uint8_t id, uint8_t mode) {
    /* 0 = 임시 원점(전원 끄면 사라짐), 1 = 영구 원점(플래시 저장) */
    ext_frame(f, AK_PKT_SET_ORIGIN, id, 1);
    f->data[0] = mode ? 1 : 0;
}

void ak_pack_disable_v3(can_frame_t *f, uint8_t id) {
    ext_frame(f, AK_PKT_DISABLE, id, 0);
}

/* ---------------------------------------------------------------- */
/* MIT 모드 명령                                                     */
/* ---------------------------------------------------------------- */

void ak_pack_mit(can_frame_t *f, ak_proto_t proto, uint8_t id,
                 float p_rad, float v_rad_s, float kp, float kd, float t_nm) {
    const ak_mit_range_t *r = ak_mit_range(proto);
    uint32_t p = f2u(p_rad, -r->p_max, r->p_max, 16);
    uint32_t v = f2u(v_rad_s, -r->v_max, r->v_max, 12);
    uint32_t kp_i = f2u(kp, 0.0f, r->kp_max, 12);
    uint32_t kd_i = f2u(kd, 0.0f, r->kd_max, 12);
    uint32_t t = f2u(t_nm, -r->t_max, r->t_max, 12);

    memset(f, 0, sizeof(*f));
    f->dlc = 8;
    uint8_t *d = f->data;

    if (proto == AK_PROTO_V2) {
        f->ext = false;
        f->id = id;
        d[0] = (uint8_t)(p >> 8);
        d[1] = (uint8_t)p;
        d[2] = (uint8_t)(v >> 4);
        d[3] = (uint8_t)(((v & 0xF) << 4) | (kp_i >> 8));
        d[4] = (uint8_t)kp_i;
        d[5] = (uint8_t)(kd_i >> 4);
        d[6] = (uint8_t)(((kd_i & 0xF) << 4) | (t >> 8));
        d[7] = (uint8_t)t;
    } else {
        f->ext = true;
        f->id = ((uint32_t)AK_PKT_SET_MIT << 8) | id;
        d[0] = (uint8_t)(kp_i >> 4);
        d[1] = (uint8_t)(((kp_i & 0xF) << 4) | (kd_i >> 8));
        d[2] = (uint8_t)kd_i;
        d[3] = (uint8_t)(p >> 8);
        d[4] = (uint8_t)p;
        d[5] = (uint8_t)(v >> 4);
        d[6] = (uint8_t)(((v & 0xF) << 4) | (t >> 8));
        d[7] = (uint8_t)t;
    }
}

void ak_pack_v2_special(can_frame_t *f, uint8_t id, uint8_t code) {
    memset(f, 0, sizeof(*f));
    f->ext = false;
    f->id = id;
    f->dlc = 8;
    memset(f->data, 0xFF, 7);
    f->data[7] = code;
}

/* ---------------------------------------------------------------- */
/* 수신                                                              */
/* ---------------------------------------------------------------- */

bool ak_parse(const can_frame_t *f, ak_rx_t *o) {
    memset(o, 0, sizeof(*o));
    const uint8_t *d = f->data;

    if (f->ext) {
        uint32_t fn = f->id >> 8;
        o->motor_id = (uint8_t)(f->id & 0xFF);
        switch (fn) {
        case AK_FB_STATUS:
            if (f->dlc < 8) return false;
            o->kind = AK_RX_STATUS;
            o->pos_rad = (float)get_be16(&d[0]) * 0.1f * AK_DEG2RAD; /* 0.1 deg */
            o->vel_rad_s = ak_erpm_to_rad_s((float)get_be16(&d[2]) * 10.0f); /* 10 ERPM */
            o->current_a = (float)get_be16(&d[4]) * 0.01f;                    /* 0.01 A */
            o->torque_nm = o->current_a * AK_KT_OUT;
            o->temp_c = (int8_t)d[6];
            o->err = d[7];
            return true;
        case AK_FB_POS32:
            if (f->dlc < 4) return false;
            o->kind = AK_RX_POS32;
            o->pos_rad = (float)get_be32(&d[0]) * 0.01f * AK_DEG2RAD; /* 0.01 deg */
            return true;
        case AK_FB_SERVO_START:
            o->kind = AK_RX_SERVO_START;
            return true;
        default:
            return false;
        }
    }

    /* 표준 프레임: V2 MIT 응답 [id][P16][V12|I12][temp][err] */
    if (f->dlc < 6) return false;
    const ak_mit_range_t *r = ak_mit_range(AK_PROTO_V2);
    uint32_t p = ((uint32_t)d[1] << 8) | d[2];
    uint32_t v = ((uint32_t)d[3] << 4) | (d[4] >> 4);
    uint32_t i = ((uint32_t)(d[4] & 0xF) << 8) | d[5];
    o->kind = AK_RX_MIT_V2;
    o->motor_id = d[0];
    o->pos_rad = u2f(p, -r->p_max, r->p_max, 16);
    o->vel_rad_s = u2f(v, -r->v_max, r->v_max, 12);
    o->torque_nm = u2f(i, -r->t_max, r->t_max, 12);
    o->current_a = o->torque_nm / AK_KT_OUT;
    if (f->dlc >= 8) {
        o->temp_c = (int16_t)d[6] - 40; /* 매뉴얼: -40 ~ 215 */
        o->err = d[7];
    }
    return true;
}

const char *ak_err_name(uint8_t err) {
    switch (err) {
    case 0: return "none";
    case 1: return "motor_over_temp";
    case 2: return "over_current";
    case 3: return "over_voltage";
    case 4: return "under_voltage";
    case 5: return "encoder";
    case 6: return "mosfet_over_temp";
    case 7: return "motor_stall";
    case AK_ERR_DISABLED_ACK: return "disabled";
    default: return "unknown";
    }
}
