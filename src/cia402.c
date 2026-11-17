/*
 * cia402.c - CiA 402 드라이브 프로파일
 *
 * 상태머신 (statusword 0x6041 하위 비트 / controlword 0x6040 명령):
 *
 *   Switch on disabled (0x40) --Shutdown 0x06--> Ready to switch on (0x21)
 *   Ready to switch on --Switch on 0x07--> Switched on (0x23)
 *   Switched on --Enable operation 0x0F--> Operation enabled (0x27)   <- 이때만 모터 구동
 *   Operation enabled --0x07--> Switched on / --0x06--> Ready / --0x00--> Switch on disabled
 *   Quick stop (0x02) -> 즉시 정지 후 Switch on disabled (0x605A = 0)
 *   고장 -> Fault reaction active (0x0F) -> Fault (0x08) --bit7 상승 에지--> Switch on disabled
 */
#include "cia402.h"

#include <math.h>
#include <string.h>

#include "ak_proto.h"
#include "co_motor.h"
#include "config.h"

#define PI_F 3.14159265358979f
#define RAD2DEG (180.0f / PI_F)

enum { M_NONE = 0, M_PP = 1, M_PV = 3, M_HM = 6, M_CSP = 8, M_CSV = 9, M_CST = 10 };
enum { HM_IDLE = 0, HM_BUSY, HM_DONE, HM_ERROR };

#define CMD_PERIOD_US 1000u       /* 루프에서 모터 명령을 다시 계산하는 주기 */
#define HOMING_SETTLE_US 20000u   /* 원점 명령 후 피드백을 기다리는 최소 시간 */
#define HOMING_TIMEOUT_US 1000000u
#define HOMING_POS_TOL_RAD 0.01f

typedef struct {
    cia402_state_t st;
    uint16_t last_cw;
    bool halt;
    int32_t hold_pos;      /* PP 에서 목표가 없거나 halt 일 때 유지할 위치 */
    bool pp_has_target;
    int32_t pp_target;     /* 래치된 절대 목표 (위치 단위, home offset 포함) */
    bool pp_ack;           /* statusword bit12: set-point acknowledge */
    bool target_reached;   /* statusword bit10 */
    bool win_in;
    uint64_t win_since;
    uint8_t hm;
    uint64_t hm_t;
    uint64_t oe_since;
    uint64_t now;
    uint64_t next_cmd;
    bool fb_fresh;
} axis_t;

static axis_t s_ax[NUM_MOTORS];
#define AX(n) (&s_ax[(n)->axis])

static int32_t round_i32(float x) {
    return (int32_t)(x >= 0.0f ? x + 0.5f : x - 0.5f);
}

static int16_t sat_i16(int32_t v) {
    return (int16_t)(v > 32767 ? 32767 : (v < -32767 ? -32767 : v));
}

static uint32_t abs32(int32_t v) {
    return v < 0 ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
}

/* 정격 토크 천분율 -> Nm (0x6072 Max torque 로 제한) */
static float permille_to_nm(const co_od_data_t *od, int32_t pm) {
    int32_t lim = od->max_torque;
    if (pm > lim) pm = lim;
    if (pm < -lim) pm = -lim;
    return (float)pm * ((float)AK_RATED_TORQUE_MNM / 1000.0f) / 1000.0f;
}

/* ------------------------------------------------------------------ */
/* 모터 명령                                                            */
/* ------------------------------------------------------------------ */

static void command(co_node_t *n) {
    axis_t *a = AX(n);
    const co_od_data_t *od = &n->od;
    int ax = n->axis;
    if (a->st != CIA402_OPERATION_ENABLED) {
        co_motor_disable(ax);
        return;
    }
    float off_rad = (float)od->home_offset * CO_POS_UNIT_RAD;
    switch (od->mode) {
    case M_PP: {
        int32_t tgt = (a->halt || !a->pp_has_target) ? a->hold_pos : a->pp_target;
        float rad = (float)tgt * CO_POS_UNIT_RAD - off_rad;
        co_motor_pos_spd(ax, rad * RAD2DEG, ak_rad_s_to_erpm((float)od->profile_vel * CO_VEL_UNIT_RAD_S),
                         ak_rad_s_to_erpm((float)od->profile_acc * CO_VEL_UNIT_RAD_S));
        break;
    }
    case M_PV:
        co_motor_vel(ax, a->halt ? 0.0f : ak_rad_s_to_erpm((float)od->target_vel * CO_VEL_UNIT_RAD_S));
        break;
    case M_CSV:
        co_motor_vel(ax, ak_rad_s_to_erpm(((float)od->target_vel + (float)od->vel_offset) * CO_VEL_UNIT_RAD_S));
        break;
    case M_CSP:
        co_motor_mit(ax, (float)od->target_pos * CO_POS_UNIT_RAD - off_rad, (float)od->vel_offset * CO_VEL_UNIT_RAD_S,
                     od->mit_kp, od->mit_kd, permille_to_nm(od, od->torque_offset));
        break;
    case M_CST:
        co_motor_mit(ax, 0.0f, 0.0f, 0.0f, 0.0f, permille_to_nm(od, (int32_t)od->target_torque + od->torque_offset));
        break;
    default: /* 모드 없음 / 호밍 */
        co_motor_disable(ax);
        break;
    }
}

/* ------------------------------------------------------------------ */
/* statusword / 상태 전이                                               */
/* ------------------------------------------------------------------ */

static void update_statusword(co_node_t *n) {
    static const uint16_t base[] = {0x0000, 0x0040, 0x0021, 0x0023, 0x0027, 0x0007, 0x000F, 0x0008};
    axis_t *a = AX(n);
    co_od_data_t *od = &n->od;
    uint16_t sw = base[a->st] | 0x0200; /* bit9 remote */
    if (a->fb_fresh) sw |= 0x0010;      /* bit4 voltage enabled = 모터 응답 중 */
    if (a->st == CIA402_OPERATION_ENABLED) {
        if (a->target_reached) sw |= 0x0400;
        switch (od->mode) {
        case M_PP:
            if (a->pp_ack) sw |= 0x1000;
            break;
        case M_PV:
            if (abs32(od->vel_actual) <= od->vel_window) sw |= 0x1000; /* speed = 0 */
            break;
        case M_HM:
            if (a->hm == HM_DONE) sw |= 0x1000;
            if (a->hm == HM_ERROR) sw |= 0x2000;
            break;
        case M_CSP:
        case M_CSV:
        case M_CST:
            sw |= 0x1000; /* 목표값을 제어 입력으로 사용 중 */
            break;
        default:
            break;
        }
    }
    od->statusword = sw;
}

static void enter_operation(co_node_t *n) {
    axis_t *a = AX(n);
    co_od_data_t *od = &n->od;
    /* 켜지는 순간 튀지 않도록 목표를 현재 위치로 초기화 */
    a->hold_pos = od->pos_actual;
    od->target_pos = od->pos_actual;
    od->target_vel = 0;
    od->target_torque = 0;
    a->pp_has_target = false;
    a->pp_ack = false;
    a->target_reached = false;
    a->win_in = false;
    a->hm = HM_IDLE;
    a->halt = (od->controlword & 0x0100) != 0;
    a->oe_since = a->now;
}

static void set_state(co_node_t *n, cia402_state_t st) {
    axis_t *a = AX(n);
    if (a->st == st) return;
    cia402_state_t old = a->st;
    a->st = st;
    if (st == CIA402_OPERATION_ENABLED) enter_operation(n);
    if (old == CIA402_OPERATION_ENABLED) {
        a->pp_ack = false;
        a->hm = HM_IDLE;
        a->target_reached = false;
    }
    update_statusword(n);
    command(n);
}

static void motor_err_info(uint8_t err, uint16_t *code, uint8_t *reg) {
    switch (err) {
    case 1: *code = 0x4310, *reg = CO_ERR_TEMP; break;    /* 모터 과열 */
    case 2: *code = 0x2310, *reg = CO_ERR_CURRENT; break; /* 과전류 */
    case 3: *code = 0x3210, *reg = CO_ERR_VOLTAGE; break; /* 과전압 */
    case 4: *code = 0x3220, *reg = CO_ERR_VOLTAGE; break; /* 저전압 */
    case 5: *code = 0x7305, *reg = CO_ERR_GENERIC; break; /* 엔코더 */
    case 6: *code = 0x4210, *reg = CO_ERR_TEMP; break;    /* MOSFET 과열 */
    case 7: *code = 0x7121, *reg = CO_ERR_GENERIC; break; /* 모터 구속(스톨) */
    default: *code = 0xFF00, *reg = CO_ERR_MANUF; break;
    }
}

static void fault(co_node_t *n, uint16_t code, uint8_t reg, uint8_t msef) {
    axis_t *a = AX(n);
    if (a->st == CIA402_FAULT || a->st == CIA402_FAULT_REACTION_ACTIVE) return;
    n->od.error_code = code;
    set_state(n, CIA402_FAULT_REACTION_ACTIVE); /* 모터 정지 */
    co_emcy(n, code, reg, msef);
    set_state(n, CIA402_FAULT);
}

static bool fault_cleared(co_node_t *n) {
    co_motor_fb_t fb;
    co_motor_get_fb(n->axis, &fb);
    return !(fb.valid && fb.err);
}

/* Operation enabled 상태에서 controlword 의 모드별 비트 (bit4 new set-point, bit6 상대, bit8 halt) */
static void op_mode_bits(co_node_t *n, uint16_t cw, uint16_t prev) {
    axis_t *a = AX(n);
    co_od_data_t *od = &n->od;
    bool rising4 = (cw & 0x0010) && !(prev & 0x0010);
    bool halt = (cw & 0x0100) != 0;
    if (halt && !a->halt) a->hold_pos = od->pos_actual;
    a->halt = halt;

    if (od->mode == M_PP) {
        if (rising4) {
            int32_t tgt = od->target_pos;
            if (cw & 0x0040) tgt += a->pp_has_target ? a->pp_target : od->pos_actual;
            a->pp_target = tgt;
            a->pp_has_target = true;
            a->pp_ack = true;
            a->target_reached = false;
            a->win_in = false;
        }
        if (!(cw & 0x0010)) a->pp_ack = false;
    } else if (od->mode == M_HM) {
        if (rising4 && a->hm != HM_BUSY) {
            co_motor_origin(n->axis); /* 방법 35/37: 현재 위치를 원점으로 */
            a->hm = HM_BUSY;
            a->hm_t = a->now;
            a->target_reached = false;
        }
    }
}

static void on_controlword(co_node_t *n) {
    axis_t *a = AX(n);
    uint16_t cw = n->od.controlword;
    uint16_t prev = a->last_cw;
    a->last_cw = cw;

    bool disable_voltage = (cw & 0x0082) == 0x0000;
    bool quick_stop = (cw & 0x0086) == 0x0002;
    bool shutdown = (cw & 0x0087) == 0x0006;
    bool switch_on = (cw & 0x008F) == 0x0007;
    bool enable_op = (cw & 0x008F) == 0x000F;

    switch (a->st) {
    case CIA402_FAULT:
        if ((cw & 0x0080) && !(prev & 0x0080) && fault_cleared(n)) {
            n->od.error_code = 0;
            co_emcy_clear(n);
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        }
        break;
    case CIA402_SWITCH_ON_DISABLED:
        if (shutdown) set_state(n, CIA402_READY_TO_SWITCH_ON);
        break;
    case CIA402_READY_TO_SWITCH_ON:
        if (disable_voltage || quick_stop) {
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        } else if (switch_on) {
            set_state(n, CIA402_SWITCHED_ON);
        } else if (enable_op) { /* 0x0F 를 바로 보내면 3, 4 전이를 연달아 */
            set_state(n, CIA402_SWITCHED_ON);
            set_state(n, CIA402_OPERATION_ENABLED);
        }
        break;
    case CIA402_SWITCHED_ON:
        if (disable_voltage || quick_stop) {
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        } else if (shutdown) {
            set_state(n, CIA402_READY_TO_SWITCH_ON);
        } else if (enable_op) {
            set_state(n, CIA402_OPERATION_ENABLED);
        }
        break;
    case CIA402_OPERATION_ENABLED:
        if (disable_voltage) {
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        } else if (quick_stop) { /* 0x605A = 0: 구동 정지 후 Switch on disabled */
            set_state(n, CIA402_QUICK_STOP_ACTIVE);
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        } else if (shutdown) {
            set_state(n, CIA402_READY_TO_SWITCH_ON);
        } else if (switch_on) {
            set_state(n, CIA402_SWITCHED_ON);
        } else {
            op_mode_bits(n, cw, prev);
            update_statusword(n);
            command(n);
        }
        break;
    default:
        break;
    }
}

static void window_check(axis_t *a, bool in, uint16_t time_ms) {
    if (!in) {
        a->win_in = false;
        a->target_reached = false;
        return;
    }
    if (!a->win_in) {
        a->win_in = true;
        a->win_since = a->now;
    }
    a->target_reached = (a->now - a->win_since) >= (uint64_t)time_ms * 1000u;
}

/* ------------------------------------------------------------------ */
/* 주기 처리                                                            */
/* ------------------------------------------------------------------ */

void cia402_process(co_node_t *n, uint64_t now) {
    axis_t *a = AX(n);
    co_od_data_t *od = &n->od;
    a->now = now;

    co_motor_fb_t fb;
    co_motor_get_fb(n->axis, &fb);
    a->fb_fresh = fb.valid && fb.age_ms <= FB_STALE_MS;
    if (fb.valid) {
        od->pos_actual = round_i32(fb.pos_rad / CO_POS_UNIT_RAD) + od->home_offset;
        od->vel_actual = round_i32(fb.vel_rad_s / CO_VEL_UNIT_RAD_S);
        od->torque_actual = sat_i16(round_i32(fb.torque_nm * 1.0e6f / (float)AK_RATED_TORQUE_MNM));
        od->current_actual = sat_i16(round_i32(fb.current_a * 1.0e6f / (float)AK_RATED_CURRENT_MA));
        od->motor_temp = fb.temp_c;
        od->motor_err = fb.err;
    }
    od->fb_age = (!fb.valid || fb.age_ms > 0xFFFF) ? 0xFFFF : (uint16_t)fb.age_ms;

    /* 고장 감지: 모터 에러 코드 / 운전 중 피드백 끊김 */
    if (fb.valid && fb.err) {
        uint16_t code;
        uint8_t reg;
        motor_err_info(fb.err, &code, &reg);
        fault(n, code, reg, fb.err);
    }
    uint64_t fb_to_us = (uint64_t)od->fb_timeout * 1000u;
    if (a->st == CIA402_OPERATION_ENABLED && fb_to_us && now - a->oe_since > fb_to_us &&
        (!fb.valid || (uint64_t)fb.age_ms * 1000u > fb_to_us)) {
        fault(n, 0xFF01, CO_ERR_MANUF | CO_ERR_COMM, 0);
    }

    /* 모드별 target reached / 호밍 진행 */
    if (a->st == CIA402_OPERATION_ENABLED) {
        switch (od->mode) {
        case M_PP:
            if (a->halt) {
                a->target_reached = abs32(od->vel_actual) <= od->vel_window;
            } else if (!a->pp_has_target) {
                a->target_reached = true;
            } else {
                window_check(a, abs32(od->pos_actual - a->pp_target) <= od->pos_window, od->pos_window_time);
            }
            break;
        case M_PV: {
            int32_t tgt = a->halt ? 0 : od->target_vel;
            window_check(a, abs32(od->vel_actual - tgt) <= od->vel_window, od->vel_window_time);
            break;
        }
        case M_HM:
            if (a->hm == HM_BUSY) {
                bool newer = fb.valid && (uint64_t)fb.age_ms * 1000u < now - a->hm_t; /* 원점 명령 이후 피드백 */
                if (now - a->hm_t >= HOMING_SETTLE_US && newer && fabsf(fb.pos_rad) < HOMING_POS_TOL_RAD) {
                    a->hm = HM_DONE;
                } else if (now - a->hm_t > HOMING_TIMEOUT_US) {
                    a->hm = HM_ERROR;
                }
            }
            a->target_reached = (a->hm == HM_DONE || a->hm == HM_ERROR);
            break;
        default:
            a->target_reached = false;
            break;
        }
    } else {
        a->target_reached = false;
    }

    update_statusword(n);
    if (now >= a->next_cmd) {
        a->next_cmd = now + CMD_PERIOD_US;
        command(n);
    }
}

cia402_state_t cia402_state(const co_node_t *n) {
    return s_ax[n->axis].st;
}

/* ------------------------------------------------------------------ */
/* co_node 애플리케이션 훅                                              */
/* ------------------------------------------------------------------ */

void co_app_reset(co_node_t *n) {
    axis_t *a = AX(n);
    memset(a, 0, sizeof(*a));
    a->st = CIA402_SWITCH_ON_DISABLED; /* Not ready -> Switch on disabled 는 자동 전이 */
    n->od.error_code = 0;
    n->od.mode_display = n->od.mode;
    co_motor_config(n->axis, n->od.motor_can_id, n->od.motor_proto);
    co_motor_disable(n->axis);
    update_statusword(n);
}

uint32_t co_app_check(co_node_t *n, uint16_t index, uint8_t sub, uint32_t raw) {
    switch (index) {
    case 0x6060: {
        int8_t m = (int8_t)raw;
        bool ok = m == M_NONE || m == M_PP || m == M_PV || m == M_HM || m == M_CSP || m == M_CSV || m == M_CST;
        return ok ? 0 : CO_ABORT_VALUE;
    }
    case 0x6098: {
        int8_t m = (int8_t)raw;
        return (m == 35 || m == 37) ? 0 : CO_ABORT_VALUE;
    }
    case 0x6007: {
        int16_t v = (int16_t)raw;
        return (v >= 0 && v <= 3) ? 0 : CO_ABORT_VALUE;
    }
    case 0x2000:
        if ((sub == 1 || sub == 2) && AX(n)->st == CIA402_OPERATION_ENABLED) return CO_ABORT_STATE;
        if (sub == 2 && raw > 1) return CO_ABORT_VALUE;
        if (sub == 3 || sub == 4) {
            float f;
            memcpy(&f, &raw, sizeof(f));
            float max = (sub == 3) ? AK_KP_MAX : AK_KD_MAX;
            if (!(f >= 0.0f && f <= max)) return CO_ABORT_VALUE; /* NaN 도 거부 */
        }
        return 0;
    default:
        return 0;
    }
}

void co_app_written(co_node_t *n, uint16_t index, uint8_t sub) {
    axis_t *a = AX(n);
    switch (index) {
    case 0x6040:
        on_controlword(n);
        break;
    case 0x6060:
        n->od.mode_display = n->od.mode;
        a->pp_has_target = false;
        a->pp_ack = false;
        a->hm = HM_IDLE;
        a->target_reached = false;
        a->win_in = false;
        a->hold_pos = n->od.pos_actual;
        update_statusword(n);
        command(n);
        break;
    case 0x2000:
        if (sub == 1 || sub == 2) {
            co_motor_config(n->axis, n->od.motor_can_id, n->od.motor_proto);
        } else if (sub == 3 || sub == 4) {
            command(n);
        }
        break;
    case 0x6071:
    case 0x6072:
    case 0x607A:
    case 0x6081:
    case 0x6083:
    case 0x60B1:
    case 0x60B2:
    case 0x60FF:
        command(n);
        break;
    default:
        break;
    }
}

void co_app_nmt(co_node_t *n, co_nmt_t from, co_nmt_t to) {
    /* Stopped 에서는 마스터가 PDO/SDO 로 모터를 멈출 수 없으므로 통신 끊김과 같이 처리 */
    if (from == CO_NMT_OPERATIONAL && to == CO_NMT_STOPPED) co_app_comm_lost(n, 0x8100);
}

void co_app_comm_lost(co_node_t *n, uint16_t code) {
    axis_t *a = AX(n);
    switch (n->od.abort_conn) {
    case 1: /* Fault signal */
        fault(n, code, CO_ERR_COMM, 0);
        break;
    case 2: /* Disable voltage */
    case 3: /* Quick stop */
        if (a->st == CIA402_READY_TO_SWITCH_ON || a->st == CIA402_SWITCHED_ON ||
            a->st == CIA402_OPERATION_ENABLED || a->st == CIA402_QUICK_STOP_ACTIVE) {
            set_state(n, CIA402_SWITCH_ON_DISABLED);
        }
        break;
    default: /* 0: 아무것도 안 함 */
        break;
    }
}
