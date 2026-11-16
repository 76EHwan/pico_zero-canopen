/*
 * co_od.h - CANopen 오브젝트 딕셔너리 (CiA 301 통신 + CiA 402 드라이브 + 제조사 영역)
 *
 * 테이블(co_od_table)이 유일한 정의다. 펌웨어의 SDO/PDO 처리와
 * EDS 파일 생성(tools/gen_eds.c) 이 모두 이 테이블을 읽는다.
 * 하드웨어 의존성이 없다 (PC 에서 테스트/EDS 생성 가능).
 */
#ifndef CO_OD_H
#define CO_OD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* CiA 301 데이터 타입 코드 */
enum {
    CO_BOOL = 0x01,
    CO_I8 = 0x02,
    CO_I16 = 0x03,
    CO_I32 = 0x04,
    CO_U8 = 0x05,
    CO_U16 = 0x06,
    CO_U32 = 0x07,
    CO_R32 = 0x08,
    CO_VSTR = 0x09,
};

/* 오브젝트 코드 */
enum { OD_VAR = 0x07, OD_ARRAY = 0x08, OD_RECORD = 0x09 };

/* 서브 엔트리 플래그 */
#define OD_R 0x01      /* 읽기 가능 */
#define OD_W 0x02      /* 쓰기 가능 */
#define OD_RW (OD_R | OD_W)
#define OD_TPDO 0x04   /* TPDO 에 매핑 가능 (드라이브 -> 마스터) */
#define OD_RPDO 0x08   /* RPDO 에 매핑 가능 (마스터 -> 드라이브) */
#define OD_CONST 0x10  /* 값이 테이블의 def 고정 (저장공간 없음) */
#define OD_NODEID 0x20 /* 기본값 = def + 노드 ID (EDS: $NODEID+def) */

#define CO_PDO_COUNT 4
#define CO_PDO_MAX_MAP 8

typedef struct {
    uint32_t cob_id;      /* sub1, bit31 = 1 이면 PDO 꺼짐 */
    uint8_t trans_type;   /* sub2 */
    uint16_t inhibit;     /* sub3 (TPDO), 100 us 단위 */
    uint16_t event_timer; /* sub5 (TPDO), ms */
} co_pdo_comm_t;

typedef struct {
    uint8_t count;                /* sub0 */
    uint32_t map[CO_PDO_MAX_MAP]; /* sub1..8 = index<<16 | sub<<8 | bits */
} co_pdo_map_t;

/* 노드 하나의 오브젝트 값 저장소 */
typedef struct {
    /* 통신 영역 0x1000-0x1FFF */
    uint8_t error_register; /* 0x1001 */
    uint32_t cob_sync;      /* 0x1005 */
    uint32_t cob_emcy;      /* 0x1014 */
    uint32_t hb_consumer;   /* 0x1016:01 = node_id<<16 | ms */
    uint16_t hb_producer;   /* 0x1017 */
    uint32_t serial;        /* 0x1018:04 */
    co_pdo_comm_t rpdo_comm[CO_PDO_COUNT]; /* 0x1400.. */
    co_pdo_map_t rpdo_map[CO_PDO_COUNT];   /* 0x1600.. */
    co_pdo_comm_t tpdo_comm[CO_PDO_COUNT]; /* 0x1800.. */
    co_pdo_map_t tpdo_map[CO_PDO_COUNT];   /* 0x1A00.. */

    /* 제조사 영역 0x2000 */
    uint8_t motor_can_id; /* 0x2000:01 */
    uint8_t motor_proto;  /* 0x2000:02 */
    float mit_kp;         /* 0x2000:03 */
    float mit_kd;         /* 0x2000:04 */
    int16_t motor_temp;   /* 0x2000:05 */
    uint8_t motor_err;    /* 0x2000:06 */
    uint16_t fb_age;      /* 0x2000:07 */
    uint16_t fb_timeout;  /* 0x2000:08 */

    /* CiA 402 영역 0x6000- */
    int16_t abort_conn;       /* 0x6007 */
    uint16_t error_code;      /* 0x603F */
    uint16_t controlword;     /* 0x6040 */
    uint16_t statusword;      /* 0x6041 */
    int8_t mode;              /* 0x6060 */
    int8_t mode_display;      /* 0x6061 */
    int32_t pos_actual;       /* 0x6064 */
    uint32_t pos_window;      /* 0x6067 */
    uint16_t pos_window_time; /* 0x6068 */
    int32_t vel_actual;       /* 0x606C */
    uint16_t vel_window;      /* 0x606D */
    uint16_t vel_window_time; /* 0x606E */
    int16_t target_torque;    /* 0x6071 */
    uint16_t max_torque;      /* 0x6072 */
    int16_t torque_actual;    /* 0x6077 */
    int16_t current_actual;   /* 0x6078 */
    int32_t target_pos;       /* 0x607A */
    int32_t home_offset;      /* 0x607C */
    uint32_t profile_vel;     /* 0x6081 */
    uint32_t profile_acc;     /* 0x6083 */
    uint32_t profile_dec;     /* 0x6084 */
    int8_t homing_method;     /* 0x6098 */
    int32_t vel_offset;       /* 0x60B1 */
    int16_t torque_offset;    /* 0x60B2 */
    uint8_t ip_time_units;    /* 0x60C2:01 */
    int8_t ip_time_index;     /* 0x60C2:02 */
    int32_t target_vel;       /* 0x60FF */
} co_od_data_t;

typedef struct {
    uint8_t sub;
    uint8_t type;
    uint8_t flags;
    uint16_t offset; /* co_od_data_t 안의 위치 (OD_CONST 면 무시) */
    const char *name;
    uint32_t def;    /* 기본값 (raw, little-endian 해석) */
    const char *str; /* CO_VSTR 상수 문자열 */
} od_sub_t;

typedef struct {
    uint16_t index;
    uint8_t code; /* OD_VAR / OD_ARRAY / OD_RECORD */
    uint8_t nsub;
    const char *name;
    const od_sub_t *subs;
} od_obj_t;

extern const od_obj_t co_od_table[];
extern const size_t co_od_table_len;

const od_obj_t *od_find(uint16_t index);
const od_sub_t *od_find_sub(const od_obj_t *o, uint8_t sub);
const od_sub_t *od_lookup(uint16_t index, uint8_t sub);
size_t od_type_size(uint8_t type); /* CO_VSTR 은 0 */
uint32_t od_get(const co_od_data_t *d, const od_sub_t *s);
void od_set(co_od_data_t *d, const od_sub_t *s, uint32_t raw);
int32_t od_to_signed(uint32_t raw, uint8_t type);

/* index 가 [lo, hi] 인 오브젝트를 기본값으로 */
void od_load_defaults(co_od_data_t *d, uint8_t node_id, uint16_t lo, uint16_t hi);

#endif /* CO_OD_H */
