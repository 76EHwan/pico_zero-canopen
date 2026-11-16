/*
 * co_node.h - CANopen 슬레이브 (CiA 301): NMT, Heartbeat, SDO 서버, PDO, SYNC, EMCY
 *
 * 노드 하나 = co_node_t 하나. 이 펌웨어는 모터 축마다 노드를 하나씩 둔다.
 * 프레임 송신은 co_set_sender() 로 등록한 콜백으로 나간다 (여기서는 slcan).
 * 하드웨어 의존성이 없다 (PC 에서 테스트 가능).
 *
 * 지원: NMT 슬레이브, heartbeat 생산자/소비자(1개), SDO expedited + segmented
 *       (block 전송은 미지원 -> abort), RPDO/TPDO 각 4개 + 동적 매핑,
 *       전송 타입 0~240(SYNC), 254/255(이벤트, inhibit, event timer), EMCY.
 */
#ifndef CO_NODE_H
#define CO_NODE_H

#include <stdbool.h>
#include <stdint.h>

#include "ak_proto.h" /* can_frame_t */
#include "co_od.h"

typedef enum {
    CO_NMT_INIT = 0x00,
    CO_NMT_STOPPED = 0x04,
    CO_NMT_OPERATIONAL = 0x05,
    CO_NMT_PREOP = 0x7F,
} co_nmt_t;

/* SDO abort 코드 (CiA 301) */
#define CO_ABORT_TOGGLE 0x05030000u
#define CO_ABORT_TIMEOUT 0x05040000u
#define CO_ABORT_CMD 0x05040001u
#define CO_ABORT_WRITEONLY 0x06010001u
#define CO_ABORT_READONLY 0x06010002u
#define CO_ABORT_NO_OBJECT 0x06020000u
#define CO_ABORT_NO_MAP 0x06040041u
#define CO_ABORT_MAP_LEN 0x06040042u
#define CO_ABORT_TYPE_LEN 0x06070010u
#define CO_ABORT_LEN_HIGH 0x06070012u
#define CO_ABORT_LEN_LOW 0x06070013u
#define CO_ABORT_NO_SUB 0x06090011u
#define CO_ABORT_VALUE 0x06090030u
#define CO_ABORT_STATE 0x08000022u

/* 에러 레지스터(0x1001) 비트 */
#define CO_ERR_GENERIC 0x01
#define CO_ERR_CURRENT 0x02
#define CO_ERR_VOLTAGE 0x04
#define CO_ERR_TEMP 0x08
#define CO_ERR_COMM 0x10
#define CO_ERR_MANUF 0x80

typedef struct {
    uint8_t n;   /* 매핑된 오브젝트 수 (0 = 사용 안 함) */
    uint8_t len; /* 바이트 */
    uint16_t index[CO_PDO_MAX_MAP];
    const od_sub_t *ent[CO_PDO_MAX_MAP];
} co_pdo_cache_t;

typedef struct {
    uint8_t node_id;
    uint8_t axis;
    uint32_t serial;
    co_nmt_t nmt;
    co_od_data_t od;

    co_pdo_cache_t rpdo[CO_PDO_COUNT];
    co_pdo_cache_t tpdo[CO_PDO_COUNT];
    struct {
        bool pending; /* SYNC 를 기다리는 RPDO 데이터 */
        uint8_t data[8];
    } rpdo_buf[CO_PDO_COUNT];
    struct {
        uint8_t last[8];
        bool sent;
        uint64_t last_us;
        uint64_t next_evt_us;
        uint8_t sync_cnt;
    } tpdo_rt[CO_PDO_COUNT];

    struct {
        uint8_t state;
        uint16_t index;
        uint8_t sub;
        uint8_t toggle;
        uint8_t buf[8];
        uint32_t size;
        uint32_t pos;
        const char *src;
        uint64_t t_us;
    } sdo;

    uint64_t hb_next_us;
    uint64_t hbc_last_us;
    bool hbc_active;
} co_node_t;

typedef void (*co_send_fn_t)(const can_frame_t *f, bool important);

void co_set_sender(co_send_fn_t fn);

/* 전원 투입과 같음: 애플리케이션 + 통신 초기화, boot-up 송신, pre-operational */
void co_node_init(co_node_t *n, uint8_t node_id, uint8_t axis, uint32_t serial, uint64_t now_us);
void co_reset_node(co_node_t *n, uint64_t now_us);
void co_reset_comm(co_node_t *n, uint64_t now_us);

void co_rx(co_node_t *n, const can_frame_t *f, uint64_t now_us);
void co_process(co_node_t *n, uint64_t now_us);

/* EMCY 송신 + 에러 레지스터 설정 / 에러 해제(EMCY 0x0000) */
void co_emcy(co_node_t *n, uint16_t code, uint8_t reg_bits, uint8_t msef0);
void co_emcy_clear(co_node_t *n);

/* 오브젝트 쓰기 (SDO/RPDO 공용). check_access=false 면 쓰기 권한 검사 생략. 성공 시 0, 실패 시 abort 코드 */
uint32_t co_od_write(co_node_t *n, uint16_t index, uint8_t sub, const uint8_t *data, size_t len, bool check_access);

/* ---- 애플리케이션 훅 (cia402.c 에서 구현) ---- */
void co_app_reset(co_node_t *n);                                                /* NMT reset node */
uint32_t co_app_check(co_node_t *n, uint16_t index, uint8_t sub, uint32_t raw); /* 값 검증 */
void co_app_written(co_node_t *n, uint16_t index, uint8_t sub);                 /* 쓰기 후처리 */
void co_app_nmt(co_node_t *n, co_nmt_t from, co_nmt_t to);
void co_app_comm_lost(co_node_t *n, uint16_t emcy_code); /* heartbeat 끊김 등 */

#endif /* CO_NODE_H */
