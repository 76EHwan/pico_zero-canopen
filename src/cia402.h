/*
 * cia402.h - CiA 402 드라이브 프로파일 (상태머신 + 운전 모드 -> AK 명령 변환)
 *
 * co_node.h 의 애플리케이션 훅(co_app_*)을 구현한다.
 *
 * 지원 모드 (0x6060):
 *   1  Profile Position (PP)  -> AK Servo 위치-속도 모드 (0x6081 속도, 0x6083 가속)
 *   3  Profile Velocity (PV)  -> AK Servo 속도 모드
 *   6  Homing (HM)            -> 방법 35/37 (현재 위치를 원점으로)
 *   8  Cyclic Sync Position   -> AK MIT (kp/kd = 0x2000:03/04, 범위 +-12.56 rad)
 *   9  Cyclic Sync Velocity   -> AK Servo 속도 모드
 *   10 Cyclic Sync Torque     -> AK MIT (토크만)
 */
#ifndef CIA402_H
#define CIA402_H

#include <stdbool.h>
#include <stdint.h>

#include "co_node.h"

typedef enum {
    CIA402_NOT_READY = 0,
    CIA402_SWITCH_ON_DISABLED,
    CIA402_READY_TO_SWITCH_ON,
    CIA402_SWITCHED_ON,
    CIA402_OPERATION_ENABLED,
    CIA402_QUICK_STOP_ACTIVE,
    CIA402_FAULT_REACTION_ACTIVE,
    CIA402_FAULT,
} cia402_state_t;

/* 모터 피드백 반영, 고장 감지, statusword 갱신, 모터 명령 재전송. 루프마다 호출 */
void cia402_process(co_node_t *n, uint64_t now_us);

cia402_state_t cia402_state(const co_node_t *n);

#endif /* CIA402_H */
