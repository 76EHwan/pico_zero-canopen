/*
 * motor_ctrl.c - core1 제어 루프
 *
 *  - core0 가 보낸 명령(host_cmd_t)을 받아 모터별 목표값/모드를 갱신
 *  - CTRL 주기마다 활성 모터에 현재 목표값을 CAN 으로 전송
 *  - 모터 피드백(0x29 등)을 해석해서 공유 메모리에 저장
 *  - 호스트 명령이 TIMEOUT 동안 끊기면 전체 모터 정지
 */
#include "motor_ctrl.h"

#include <string.h>

#include "hardware/sync.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"

typedef struct {
    uint8_t can_id;
    ak_proto_t proto;
    motor_mode_t mode;
    float sp[5];
    bool v2_mit_entered;
    uint8_t last_err;
} motor_state_t;

/* ---- 코어 간 공유 ---- */
static queue_t s_cmd_q; /* core0 -> core1 */
static queue_t s_evt_q; /* core1 -> core0 */
static queue_t s_mon_q; /* core1 -> core0 (CAN 모니터) */
static spin_lock_t *s_lock;
static motor_fb_t s_fb[NUM_MOTORS]; /* s_lock 으로 보호 */
static ctrl_status_t s_status;      /* s_lock 으로 보호 */
static volatile uint32_t s_heartbeat;
static volatile uint32_t s_host_ms;
static volatile bool s_monitor;

/* ---- core1 전용 ---- */
static motor_state_t s_m[NUM_MOTORS];
static uint32_t s_rate_hz;
static uint32_t s_period_us;
static uint32_t s_timeout_ms;
static uint32_t s_tick_skip;
static uint32_t s_tick_overrun;
static bool s_stalled;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void push_evt(uint8_t type, uint8_t motor, uint16_t code) {
    motor_evt_t e = {type, motor, code};
    queue_try_add(&s_evt_q, &e);
}

static void fill_status(ctrl_status_t *st) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        st->can_id[i] = s_m[i].can_id;
        st->proto[i] = (uint8_t)s_m[i].proto;
        st->mode[i] = (uint8_t)s_m[i].mode;
    }
    st->rate_hz = s_rate_hz;
    st->timeout_ms = s_timeout_ms;
    st->bus_stalled = s_stalled;
    st->tick_skip = s_tick_skip;
    st->tick_overrun = s_tick_overrun;
}

static void publish_status(void) {
    ctrl_status_t st;
    memset(&st, 0, sizeof(st));
    fill_status(&st);
    can_bus_get_stats(&st.can);
    uint32_t save = spin_lock_blocking(s_lock);
    s_status = st;
    spin_unlock(s_lock, save);
}

static void clear_fb(int i) {
    uint32_t save = spin_lock_blocking(s_lock);
    memset(&s_fb[i], 0, sizeof(s_fb[i]));
    spin_unlock(s_lock, save);
}

/* 모터를 멈추고 스트리밍을 중단한다 */
static void safe_stop(int i) {
    motor_state_t *m = &s_m[i];
    can_frame_t f;
    if (m->proto == AK_PROTO_V3) {
#if AK_V3_STOP_MODE == 0
        ak_pack_disable_v3(&f, m->can_id); /* 응답: 0x29 DATA[7] = 0x77 */
#else
        ak_pack_current(&f, m->can_id, 0.0f);
#endif
    } else if (m->v2_mit_entered) {
        ak_pack_v2_special(&f, m->can_id, AK_V2_EXIT);
        m->v2_mit_entered = false;
    } else {
        ak_pack_current(&f, m->can_id, 0.0f); /* V2 servo: 전류 0 = 토크 해제 */
    }
    can_bus_send_priority(&f);
    m->mode = MODE_IDLE;
}

static bool build_setpoint(const motor_state_t *m, can_frame_t *f) {
    const float *sp = m->sp;
    switch (m->mode) {
    case MODE_MIT: ak_pack_mit(f, m->proto, m->can_id, sp[0], sp[1], sp[2], sp[3], sp[4]); return true;
    case MODE_POS: ak_pack_pos(f, m->can_id, sp[0]); return true;
    case MODE_POS_SPD: ak_pack_pos_spd(f, m->can_id, sp[0], sp[1], sp[2]); return true;
    case MODE_VEL: ak_pack_rpm(f, m->can_id, sp[0]); return true;
    case MODE_CUR: ak_pack_current(f, m->can_id, sp[0]); return true;
    case MODE_BRAKE: ak_pack_brake(f, m->can_id, sp[0]); return true;
    case MODE_DUTY: ak_pack_duty(f, m->can_id, sp[0]); return true;
    default: return false;
    }
}

static void v2_enter_mit(motor_state_t *m) {
    can_frame_t f;
    ak_pack_v2_special(&f, m->can_id, AK_V2_ENTER);
    can_bus_send_priority(&f);
    m->v2_mit_entered = true;
}

static void handle_cmd(const host_cmd_t *c) {
    switch (c->type) {
    case CMD_SET_RATE: {
        uint32_t hz = c->u;
        if (hz < CTRL_RATE_HZ_MIN) hz = CTRL_RATE_HZ_MIN;
        if (hz > CTRL_RATE_HZ_MAX) hz = CTRL_RATE_HZ_MAX;
        s_rate_hz = hz;
        s_period_us = 1000000u / hz;
        return;
    }
    case CMD_SET_TIMEOUT:
        s_timeout_ms = c->u;
        return;
    case CMD_RAW:
        can_bus_send_priority(&c->raw);
        return;
    default:
        break;
    }

    int first, last;
    if (c->motor == MOTOR_ALL) {
        first = 0;
        last = NUM_MOTORS - 1;
    } else if (c->motor < NUM_MOTORS) {
        first = last = c->motor;
    } else {
        return;
    }

    for (int i = first; i <= last; i++) {
        motor_state_t *m = &s_m[i];
        can_frame_t f;
        switch (c->type) {
        case CMD_SETPOINT:
            if (c->mode == MODE_IDLE || c->mode >= MODE_COUNT) break;
            if (m->proto == AK_PROTO_V2 && c->mode == MODE_MIT && !m->v2_mit_entered) v2_enter_mit(m);
            m->mode = (motor_mode_t)c->mode;
            memcpy(m->sp, c->sp, sizeof(m->sp));
            break;
        case CMD_ENABLE:
            if (m->proto == AK_PROTO_V2) v2_enter_mit(m);
            break;
        case CMD_DISABLE:
            safe_stop(i);
            break;
        case CMD_ORIGIN:
            if (m->proto == AK_PROTO_V2 && m->v2_mit_entered) {
                ak_pack_v2_special(&f, m->can_id, AK_V2_ZERO);
            } else {
                ak_pack_origin(&f, m->can_id, (uint8_t)c->u);
            }
            can_bus_send_priority(&f);
            break;
        case CMD_SET_ID:
        case CMD_SET_PROTO:
            if (m->mode != MODE_IDLE || m->v2_mit_entered) safe_stop(i);
            if (c->type == CMD_SET_ID) {
                m->can_id = (uint8_t)c->u;
            } else {
                m->proto = c->u ? AK_PROTO_V2 : AK_PROTO_V3;
            }
            m->v2_mit_entered = false;
            m->last_err = 0;
            clear_fb(i);
            break;
        default:
            break;
        }
    }
}

static int find_motor(uint8_t can_id, bool v2_only) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (s_m[i].can_id == can_id && (!v2_only || s_m[i].proto == AK_PROTO_V2)) return i;
    }
    return -1;
}

static void handle_rx(const can_frame_t *f, uint32_t t_ms) {
    if (s_monitor) queue_try_add(&s_mon_q, f);

    ak_rx_t r;
    if (!ak_parse(f, &r)) return;
    int i = find_motor(r.motor_id, r.kind == AK_RX_MIT_V2);
    if (i < 0) return;
    motor_state_t *m = &s_m[i];

    if (r.kind == AK_RX_SERVO_START) {
        push_evt(EVT_BOOT, (uint8_t)i, 0);
        return;
    }

    if (r.kind == AK_RX_POS32) {
        uint32_t save = spin_lock_blocking(s_lock);
        s_fb[i].pos_rad = r.pos_rad;
        s_fb[i].has_pos32 = true;
        spin_unlock(s_lock, save);
        return;
    }

    /* AK_RX_STATUS 또는 AK_RX_MIT_V2 */
    bool disabled_ack = (r.err == AK_ERR_DISABLED_ACK);
    if (disabled_ack) {
        push_evt(EVT_DISABLED, (uint8_t)i, 0);
    } else if (r.err != m->last_err) {
        push_evt(r.err ? EVT_FAULT : EVT_FAULT_CLEAR, (uint8_t)i, r.err);
        m->last_err = r.err;
    }

    uint32_t save = spin_lock_blocking(s_lock);
    motor_fb_t *fb = &s_fb[i];
    if (!fb->has_pos32) fb->pos_rad = r.pos_rad;
    fb->vel_rad_s = r.vel_rad_s;
    fb->torque_nm = r.torque_nm;
    fb->current_a = r.current_a;
    fb->temp_c = r.temp_c;
    if (!disabled_ack) fb->err = r.err;
    fb->last_ms = t_ms;
    fb->count++;
    fb->valid = true;
    spin_unlock(s_lock, save);
}

static void control_tick(uint32_t t_ms) {
    bool active = false;
    for (int i = 0; i < NUM_MOTORS; i++) active |= (s_m[i].mode != MODE_IDLE);
    if (!active) return;

    if (s_timeout_ms && (int32_t)(t_ms - s_host_ms) > (int32_t)s_timeout_ms) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (s_m[i].mode != MODE_IDLE) safe_stop(i);
        }
        push_evt(EVT_TIMEOUT, MOTOR_ALL, 0);
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        can_frame_t f;
        if (!build_setpoint(&s_m[i], &f)) continue;
        if (!can_bus_try_send(&f)) s_tick_skip++;
    }
}

/* 100 ms 마다: ACK 없음(버스 정지) / 수신 오버플로 감지 */
static void bus_diag(void) {
    static uint32_t prev_attempt, prev_total, prev_ovf;
    can_bus_stats_t cs;
    can_bus_get_stats(&cs);

    bool attempted = cs.tx_attempt != prev_attempt;
    bool progressed = cs.tx_total != prev_total;
    if (attempted && !progressed) {
        if (!s_stalled) {
            s_stalled = true;
            push_evt(EVT_CAN_STALL, MOTOR_ALL, 0);
        }
    } else if (progressed && s_stalled) {
        s_stalled = false;
        push_evt(EVT_CAN_OK, MOTOR_ALL, 0);
    }

    uint32_t ovf = cs.rx_overflow + cs.rx_error;
    if (ovf != prev_ovf) push_evt(EVT_CAN_OVERFLOW, MOTOR_ALL, (uint16_t)(ovf - prev_ovf));

    prev_attempt = cs.tx_attempt;
    prev_total = cs.tx_total;
    prev_ovf = ovf;
}

void motor_ctrl_core1_entry(void) {
    can_bus_init();

    /* 부팅(또는 워치독 리셋) 직후엔 모든 모터를 알려진 정지 상태로 */
    for (int i = 0; i < NUM_MOTORS; i++) safe_stop(i);

    uint64_t next_tick = time_us_64() + s_period_us;
    uint32_t next_status = 0, next_diag = 0;

    for (;;) {
        s_heartbeat++;

        host_cmd_t c;
        while (queue_try_remove(&s_cmd_q, &c)) handle_cmd(&c);

        uint32_t t_ms = now_ms();
        can_frame_t f;
        for (int n = 0; n < 64 && can_bus_recv(&f); n++) handle_rx(&f, t_ms);
        can_bus_poll();

        uint64_t t_us = time_us_64();
        if ((int64_t)(t_us - next_tick) >= 0) {
            next_tick += s_period_us;
            if ((int64_t)(t_us - next_tick) >= 0) {
                s_tick_overrun++;
                next_tick = t_us + s_period_us;
            }
            control_tick(t_ms);
        }

        if ((int32_t)(t_ms - next_status) >= 0) {
            next_status = t_ms + 20;
            publish_status();
        }
        if ((int32_t)(t_ms - next_diag) >= 0) {
            next_diag = t_ms + 100;
            bus_diag();
        }
    }
}

/* ------------------------------------------------------------------ */
/* core0 쪽 API                                                         */
/* ------------------------------------------------------------------ */

void motor_ctrl_init(void) {
    queue_init(&s_cmd_q, sizeof(host_cmd_t), 64);
    queue_init(&s_evt_q, sizeof(motor_evt_t), 32);
    queue_init(&s_mon_q, sizeof(can_frame_t), 128);
    s_lock = spin_lock_init(spin_lock_claim_unused(true));

    const uint8_t ids[NUM_MOTORS] = {MOTOR_DEFAULT_ID_1, MOTOR_DEFAULT_ID_2, MOTOR_DEFAULT_ID_3};
    for (int i = 0; i < NUM_MOTORS; i++) {
        memset(&s_m[i], 0, sizeof(s_m[i]));
        s_m[i].can_id = ids[i];
        s_m[i].proto = MOTOR_DEFAULT_PROTO ? AK_PROTO_V2 : AK_PROTO_V3;
        s_m[i].mode = MODE_IDLE;
    }
    s_rate_hz = CTRL_RATE_HZ_DEFAULT;
    s_period_us = 1000000u / s_rate_hz;
    s_timeout_ms = HOST_TIMEOUT_MS_DEFAULT;
    s_host_ms = now_ms();

    memset(&s_status, 0, sizeof(s_status));
    fill_status(&s_status); /* core1 가 시작되기 전이라 잠금 불필요 */
}

bool motor_ctrl_post(const host_cmd_t *c) {
    s_host_ms = now_ms();
    return queue_try_add(&s_cmd_q, c);
}

void motor_ctrl_touch_host(void) {
    s_host_ms = now_ms();
}

void motor_ctrl_get_fb(int idx, motor_fb_t *out) {
    uint32_t save = spin_lock_blocking(s_lock);
    *out = s_fb[idx];
    spin_unlock(s_lock, save);
}

void motor_ctrl_get_status(ctrl_status_t *out) {
    uint32_t save = spin_lock_blocking(s_lock);
    *out = s_status;
    spin_unlock(s_lock, save);
}

bool motor_ctrl_pop_event(motor_evt_t *e) {
    return queue_try_remove(&s_evt_q, e);
}

bool motor_ctrl_pop_monitor(can_frame_t *f) {
    return queue_try_remove(&s_mon_q, f);
}

void motor_ctrl_set_monitor(bool on) {
    s_monitor = on;
    if (!on) {
        can_frame_t f;
        while (queue_try_remove(&s_mon_q, &f)) {
        }
    }
}

uint32_t motor_ctrl_heartbeat(void) {
    return s_heartbeat;
}

const char *motor_mode_name(uint8_t mode) {
    static const char *const names[MODE_COUNT] = {"IDLE", "MIT", "POS", "PV", "VEL", "CUR", "BRK", "DUTY"};
    return mode < MODE_COUNT ? names[mode] : "?";
}
