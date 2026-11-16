/*
 * co_motor_pico.c - co_motor.h 의 펌웨어 구현: core1 의 motor_ctrl 로 명령 전달
 *
 * 같은 명령이 반복되면 큐에 다시 넣지 않고, 50 ms 마다만 갱신한다
 * (motor_ctrl 의 HOST_TIMEOUT 감시가 운전 중에 끊기지 않게).
 */
#include <string.h>

#include "co_motor.h"
#include "config.h"
#include "motor_ctrl.h"
#include "pico/stdlib.h"

#define REFRESH_MS 50

typedef struct {
    bool valid;
    uint8_t type;
    uint8_t mode;
    float sp[5];
    uint32_t t_ms;
} last_cmd_t;

static last_cmd_t s_last[NUM_MOTORS];

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void post(int axis, uint8_t type, uint8_t mode, const float sp[5]) {
    if (axis < 0 || axis >= NUM_MOTORS) return;
    last_cmd_t *l = &s_last[axis];
    uint32_t t = now_ms();
    if (l->valid && l->type == type && l->mode == mode && memcmp(l->sp, sp, sizeof(l->sp)) == 0) {
        if (type == CMD_DISABLE) return;         /* 정지는 한 번만 */
        if (t - l->t_ms < REFRESH_MS) return;    /* 같은 목표는 주기적으로만 */
    }
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = type;
    c.motor = (uint8_t)axis;
    c.mode = mode;
    memcpy(c.sp, sp, sizeof(c.sp));
    if (motor_ctrl_post(&c)) {
        l->valid = true;
        l->type = type;
        l->mode = mode;
        memcpy(l->sp, sp, sizeof(l->sp));
        l->t_ms = t;
    }
}

static void post_u(int axis, uint8_t type, uint32_t u) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = type;
    c.motor = (uint8_t)axis;
    c.u = u;
    motor_ctrl_post(&c);
    s_last[axis].valid = false;
}

void co_motor_get_fb(int axis, co_motor_fb_t *fb) {
    motor_fb_t m;
    motor_ctrl_get_fb(axis, &m);
    fb->valid = m.valid;
    fb->age_ms = m.valid ? now_ms() - m.last_ms : UINT32_MAX;
    fb->pos_rad = m.pos_rad;
    fb->vel_rad_s = m.vel_rad_s;
    fb->torque_nm = m.torque_nm;
    fb->current_a = m.current_a;
    fb->temp_c = m.temp_c;
    fb->err = m.err;
}

void co_motor_disable(int axis) {
    const float sp[5] = {0};
    post(axis, CMD_DISABLE, MODE_IDLE, sp);
}

void co_motor_mit(int axis, float p_rad, float v_rad_s, float kp, float kd, float t_nm) {
    const float sp[5] = {p_rad, v_rad_s, kp, kd, t_nm};
    post(axis, CMD_SETPOINT, MODE_MIT, sp);
}

void co_motor_pos_spd(int axis, float deg, float erpm, float erpm_per_s) {
    const float sp[5] = {deg, erpm, erpm_per_s, 0, 0};
    post(axis, CMD_SETPOINT, MODE_POS_SPD, sp);
}

void co_motor_vel(int axis, float erpm) {
    const float sp[5] = {erpm, 0, 0, 0, 0};
    post(axis, CMD_SETPOINT, MODE_VEL, sp);
}

void co_motor_origin(int axis) {
    if (axis < 0 || axis >= NUM_MOTORS) return;
    post_u(axis, CMD_ORIGIN, 0); /* 0 = 임시 원점 (전원 끄면 사라짐) */
}

void co_motor_config(int axis, uint8_t can_id, uint8_t proto) {
    if (axis < 0 || axis >= NUM_MOTORS) return;
    post_u(axis, CMD_SET_ID, can_id);
    post_u(axis, CMD_SET_PROTO, proto);
}
