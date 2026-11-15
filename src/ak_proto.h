/*
 * ak_proto.h - CubeMars AK 시리즈(AK40-10) CAN 프로토콜 인코딩/디코딩
 *
 * 하드웨어 의존성이 없는 순수 함수만 있다 (PC 에서 단위 테스트 가능).
 *
 * 프로토콜 요약 (모두 CAN 2.0, 1 Mbps):
 *  - Servo 명령   : 확장 프레임, ID = (packet << 8) | driver_id, 빅엔디언
 *  - MIT 명령 (V3): 확장 프레임, ID = (8 << 8) | driver_id
 *                   [KP 12][KD 12][P 16][V 12][T 12]
 *  - MIT 명령 (V2): 표준 프레임, ID = driver_id
 *                   [P 16][V 12][KP 12][KD 12][T 12], 0xFC 로 진입 / 0xFD 해제
 *  - 피드백        : 확장 프레임 0x29xx (pos 0.1deg, ERPM/10, 0.01A, 온도, 에러)
 *                   V2 MIT 응답은 표준 프레임 [id][P16][V12][I12][T][err]
 */
#ifndef AK_PROTO_H
#define AK_PROTO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t id; /* 11-bit (std) 또는 29-bit (ext) */
    bool ext;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

typedef enum {
    AK_PROTO_V3 = 0, /* AK40-10 V3.0 KV170 (현행) */
    AK_PROTO_V2 = 1, /* 구형 AK 2.0 / AK-DRV 드라이버 */
} ak_proto_t;

/* Servo 모드 packet ID (확장 ID 의 bit 28..8) */
enum {
    AK_PKT_SET_DUTY = 0,
    AK_PKT_SET_CURRENT = 1,
    AK_PKT_SET_CURRENT_BRAKE = 2,
    AK_PKT_SET_RPM = 3,
    AK_PKT_SET_POS = 4,
    AK_PKT_SET_ORIGIN = 5,
    AK_PKT_SET_POS_SPD = 6,
    AK_PKT_SET_MIT = 8,  /* V3 */
    AK_PKT_DISABLE = 15, /* V3 */
};

/* 모터 -> 마스터 업로드 function ID */
enum {
    AK_FB_STATUS = 0x29,      /* 주기 상태 피드백 */
    AK_FB_POS32 = 0x2A,       /* (옵션) 32bit 위치, 0.01 deg */
    AK_FB_SERVO_START = 0x2C, /* 부팅 후 servo 모드 진입 알림 */
};

/* V2 MIT 특수 명령 (마지막 바이트) */
enum {
    AK_V2_ENTER = 0xFC,
    AK_V2_EXIT = 0xFD,
    AK_V2_ZERO = 0xFE,
};

/* disable(15) 명령 성공 시 0x29 프레임의 DATA[7] 값 */
#define AK_ERR_DISABLED_ACK 0x77

typedef struct {
    float p_max, v_max, t_max, kp_max, kd_max;
} ak_mit_range_t;

const ak_mit_range_t *ak_mit_range(ak_proto_t proto);

/* 단위 변환 (출력축 기준) */
float ak_erpm_to_rad_s(float erpm);
float ak_rad_s_to_erpm(float rad_s);

/* ---- 명령 프레임 생성 ---- */
void ak_pack_duty(can_frame_t *f, uint8_t id, float duty);
void ak_pack_current(can_frame_t *f, uint8_t id, float amps);
void ak_pack_brake(can_frame_t *f, uint8_t id, float amps);
void ak_pack_rpm(can_frame_t *f, uint8_t id, float erpm);
void ak_pack_pos(can_frame_t *f, uint8_t id, float deg);
void ak_pack_pos_spd(can_frame_t *f, uint8_t id, float deg, float erpm, float erpm_per_s);
void ak_pack_origin(can_frame_t *f, uint8_t id, uint8_t mode);
void ak_pack_disable_v3(can_frame_t *f, uint8_t id);
void ak_pack_mit(can_frame_t *f, ak_proto_t proto, uint8_t id,
                 float p_rad, float v_rad_s, float kp, float kd, float t_nm);
void ak_pack_v2_special(can_frame_t *f, uint8_t id, uint8_t code);

/* ---- 수신 프레임 해석 ---- */
typedef enum {
    AK_RX_NONE = 0,
    AK_RX_STATUS,      /* 0x29 */
    AK_RX_POS32,       /* 0x2A */
    AK_RX_SERVO_START, /* 0x2C */
    AK_RX_MIT_V2,      /* V2 MIT 응답 (표준 프레임) */
} ak_rx_kind_t;

typedef struct {
    ak_rx_kind_t kind;
    uint8_t motor_id;
    float pos_rad;   /* 출력축 */
    float vel_rad_s; /* 출력축 */
    float current_a; /* Iq */
    float torque_nm; /* 출력축 */
    int16_t temp_c;
    uint8_t err;
} ak_rx_t;

bool ak_parse(const can_frame_t *f, ak_rx_t *out);

const char *ak_err_name(uint8_t err);

#endif /* AK_PROTO_H */
