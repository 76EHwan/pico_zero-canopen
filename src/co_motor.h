/*
 * co_motor.h - CiA 402 계층이 실제 모터를 움직이는 인터페이스
 *
 * 펌웨어: src/co_motor_pico.c (core1 의 motor_ctrl 로 명령 전달)
 * PC 테스트: tests/test_canopen.c 의 스텁
 * axis = 0..NUM_MOTORS-1
 */
#ifndef CO_MOTOR_H
#define CO_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;      /* 피드백을 한 번이라도 받았는지 */
    uint32_t age_ms; /* 마지막 피드백 이후 경과 시간 */
    float pos_rad;   /* 출력축 (모터 원점 기준) */
    float vel_rad_s;
    float torque_nm;
    float current_a;
    int16_t temp_c;
    uint8_t err; /* AK 에러 코드 (0 = 정상) */
} co_motor_fb_t;

void co_motor_get_fb(int axis, co_motor_fb_t *fb);
void co_motor_disable(int axis);
void co_motor_mit(int axis, float p_rad, float v_rad_s, float kp, float kd, float t_nm);
void co_motor_pos_spd(int axis, float deg, float erpm, float erpm_per_s);
void co_motor_vel(int axis, float erpm);
void co_motor_origin(int axis);
void co_motor_config(int axis, uint8_t can_id, uint8_t proto);

#endif /* CO_MOTOR_H */
