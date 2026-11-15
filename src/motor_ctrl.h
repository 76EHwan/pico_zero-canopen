/*
 * motor_ctrl.h - 모터 3개 제어 루프 (core1) 와 core0 사이의 공유 인터페이스
 *
 * core0 (USB 시리얼)  --host_cmd_t 큐-->  core1 (CAN 제어 루프)
 * core0               <--피드백/상태 (spinlock), 이벤트 큐--  core1
 */
#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "ak_proto.h"
#include "can_bus.h"
#include "config.h"

#define MOTOR_ALL 0xFF

typedef enum {
    MODE_IDLE = 0, /* 아무것도 안 보냄 (정지 상태) */
    MODE_MIT,      /* sp = p[rad], v[rad/s], kp, kd, t[Nm] */
    MODE_POS,      /* sp = deg */
    MODE_POS_SPD,  /* sp = deg, ERPM, ERPM/s */
    MODE_VEL,      /* sp = ERPM */
    MODE_CUR,      /* sp = A */
    MODE_BRAKE,    /* sp = A */
    MODE_DUTY,     /* sp = duty */
    MODE_COUNT
} motor_mode_t;

typedef enum {
    CMD_SETPOINT = 1, /* mode + sp[] */
    CMD_ENABLE,       /* V2: MIT 진입(0xFC) */
    CMD_DISABLE,      /* 정지 + 스트리밍 중단 */
    CMD_ORIGIN,       /* u = 0 임시 / 1 영구 */
    CMD_SET_ID,       /* u = CAN ID */
    CMD_SET_PROTO,    /* u = ak_proto_t */
    CMD_SET_RATE,     /* u = Hz */
    CMD_SET_TIMEOUT,  /* u = ms */
    CMD_RAW,          /* raw 프레임 1회 전송 */
} host_cmd_type_t;

typedef struct {
    uint8_t type;
    uint8_t motor; /* 0..NUM_MOTORS-1 또는 MOTOR_ALL */
    uint8_t mode;
    uint32_t u;
    float sp[5];
    can_frame_t raw;
} host_cmd_t;

typedef struct {
    bool valid;
    uint32_t last_ms; /* 마지막 피드백 수신 시각 (부팅 후 ms) */
    uint32_t count;
    float pos_rad;
    float vel_rad_s;
    float torque_nm;
    float current_a;
    int16_t temp_c;
    uint8_t err;
    bool has_pos32;
} motor_fb_t;

typedef enum {
    EVT_BOOT = 1,    /* 모터가 0x2C(servo 시작) 프레임을 보냄 = 전원 켜짐/리셋 */
    EVT_DISABLED,    /* disable 확인 응답 (0x77) */
    EVT_FAULT,       /* code = 에러 코드 */
    EVT_FAULT_CLEAR,
    EVT_TIMEOUT,     /* 호스트 명령 끊김 -> 전체 정지 */
    EVT_CAN_OVERFLOW,/* 수신 버퍼 넘침 */
    EVT_CAN_STALL,   /* 송신이 ACK 를 못 받음 (배선/전원/종단저항 확인) */
    EVT_CAN_OK,
} motor_evt_type_t;

typedef struct {
    uint8_t type;
    uint8_t motor;
    uint16_t code;
} motor_evt_t;

typedef struct {
    uint8_t can_id[NUM_MOTORS];
    uint8_t proto[NUM_MOTORS];
    uint8_t mode[NUM_MOTORS];
    uint32_t rate_hz;
    uint32_t timeout_ms;
    bool bus_stalled;
    can_bus_stats_t can;
    uint32_t tick_skip;    /* TX 큐가 차서 이번 주기 전송을 건너뛴 횟수 */
    uint32_t tick_overrun; /* 제어 주기를 놓친 횟수 */
} ctrl_status_t;

/* ---- core0 에서 호출 ---- */
void motor_ctrl_init(void); /* core1 시작 전에 호출 */
bool motor_ctrl_post(const host_cmd_t *c);
void motor_ctrl_touch_host(void); /* 호스트 활동 표시 (워치독) */
void motor_ctrl_get_fb(int idx, motor_fb_t *out);
void motor_ctrl_get_status(ctrl_status_t *out);
bool motor_ctrl_pop_event(motor_evt_t *e);
bool motor_ctrl_pop_monitor(can_frame_t *f);
void motor_ctrl_set_monitor(bool on);
uint32_t motor_ctrl_heartbeat(void);

/* ---- core1 진입점 ---- */
void motor_ctrl_core1_entry(void);

const char *motor_mode_name(uint8_t mode);

#endif /* MOTOR_CTRL_H */
