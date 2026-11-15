/*
 * config.h - 빌드 시 설정값 (핀, CAN 속도, 모터 ID/모델 파라미터, 주기)
 *
 * 모든 값은 #ifndef 로 감싸져 있어서 CMake 의
 * target_compile_definitions(... CAN_GPIO_RX=2 ...) 로도 덮어쓸 수 있다.
 */
#ifndef CONFIG_H
#define CONFIG_H

#define FW_VERSION "1.0.0"

/* ------------------------------------------------------------------ */
/* CAN (can2040 + SN65HVD230)                                          */
/* ------------------------------------------------------------------ */
/* SN65HVD230 의 R(CRX) 핀 -> MCU RX, D(CTX) 핀 <- MCU TX               */
#ifndef CAN_GPIO_RX
#define CAN_GPIO_RX 4
#endif
#ifndef CAN_GPIO_TX
#define CAN_GPIO_TX 5
#endif

/* can2040 이 통째로 점유하는 PIO 블록 (0 또는 1, RP2350 은 2 도 가능) */
#ifndef CAN_PIO_NUM
#define CAN_PIO_NUM 0
#endif

/* AK 시리즈 기본 CAN 속도: 1 Mbps */
#ifndef CAN_BITRATE
#define CAN_BITRATE 1000000
#endif

/* ------------------------------------------------------------------ */
/* 모터                                                                */
/* ------------------------------------------------------------------ */
#define NUM_MOTORS 3

/* CubeMars 상위 프로그램에서 설정한 각 모터의 CAN ID (Driver ID) */
#ifndef MOTOR_DEFAULT_ID_1
#define MOTOR_DEFAULT_ID_1 1
#endif
#ifndef MOTOR_DEFAULT_ID_2
#define MOTOR_DEFAULT_ID_2 2
#endif
#ifndef MOTOR_DEFAULT_ID_3
#define MOTOR_DEFAULT_ID_3 3
#endif

/* 기본 프로토콜: 0 = V3 (AK40-10 V3.0 KV170, 드라이버 AK48-2405-2D-A2)
 *               1 = V2 (구형 AK 2.0 / AK-DRV 드라이버, MIT 는 표준 프레임) */
#ifndef MOTOR_DEFAULT_PROTO
#define MOTOR_DEFAULT_PROTO 0
#endif

/* AK40-10 V3.0 KV170 사양 (cubemars.com 제품 페이지 / V3 드라이버 매뉴얼) */
#define AK_POLE_PAIRS 14     /* 극쌍 수: ERPM -> 모터 RPM 변환 */
#define AK_GEAR_RATIO 10.0f  /* 감속비 10:1 */
#define AK_KT_OUT 0.4894f    /* 출력축 기준 토크 상수 [Nm/A] (T = Kt * Iq) */

/* V3 정지 방식: 0 = disable 프레임(ID 15, 모터가 0x77 로 확인)
 *               1 = 전류 0 A 명령 (disable 후 재가동이 안 되는 펌웨어일 때) */
#ifndef AK_V3_STOP_MODE
#define AK_V3_STOP_MODE 0
#endif

/* 안전 클램프 (Servo 모드 명령) */
#define AK_MAX_CURRENT_A 7.3f  /* AK40-10 피크 전류 */
#define AK_MAX_DUTY 0.95f
#define AK_MAX_ERPM 100000.0f  /* 속도 모드 프로토콜 한계 */
#define AK_MAX_POS_DEG 36000.0f

/* MIT(힘 제어) 모드 값 범위. 모터 펌웨어의 범위와 반드시 같아야 한다.
 * V3: "AK Series Motor Module V3 Driver Software User Manual" 의 AK40-10 행
 * V2: "AK Series Module Driver Manual V1.0.18" 의 AK40-10 열              */
#define AK_V3_P_MAX 12.56f /* rad   */
#define AK_V3_V_MAX 45.0f  /* rad/s */
#define AK_V3_T_MAX 5.0f   /* Nm    */
#define AK_V2_P_MAX 12.5f
#define AK_V2_V_MAX 45.5f
#define AK_V2_T_MAX 5.0f
#define AK_KP_MAX 500.0f
#define AK_KD_MAX 5.0f

/* ------------------------------------------------------------------ */
/* 주기 / 안전                                                          */
/* ------------------------------------------------------------------ */
/* 활성 모터에 현재 목표값을 CAN 으로 다시 보내는 주기 [Hz] */
#ifndef CTRL_RATE_HZ_DEFAULT
#define CTRL_RATE_HZ_DEFAULT 500
#endif
#define CTRL_RATE_HZ_MIN 10
#define CTRL_RATE_HZ_MAX 1000

/* 이 시간 동안 USB 로 아무 명령도 안 오면 모든 모터를 정지(disable) [ms]
 * 0 이면 끔 (권장하지 않음) */
#ifndef HOST_TIMEOUT_MS_DEFAULT
#define HOST_TIMEOUT_MS_DEFAULT 500
#endif

/* 텔레메트리(FB 줄) 출력 주기 [Hz], 0 = 끔 */
#ifndef TELEM_RATE_HZ_DEFAULT
#define TELEM_RATE_HZ_DEFAULT 100
#endif
#define TELEM_RATE_HZ_MAX 500

/* 이 시간보다 오래된 피드백은 "끊김" 으로 본다 (LED 표시용) [ms] */
#define FB_STALE_MS 200

/* 하드웨어 워치독 [ms] */
#define WATCHDOG_MS 1000

#endif /* CONFIG_H */
