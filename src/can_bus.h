/*
 * can_bus.h - can2040 (PIO 소프트웨어 CAN) 래퍼
 *
 * 모든 함수는 core1 에서만 호출한다 (PIO IRQ 도 core1 에 등록됨).
 */
#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "ak_proto.h"

typedef struct {
    /* can2040 통계 */
    uint32_t rx_total;
    uint32_t tx_total;
    uint32_t tx_attempt; /* 재전송 포함. tx_total 보다 계속 커지면 ACK 없음 */
    uint32_t parse_error;
    /* 이 모듈 통계 */
    uint32_t rx_overflow; /* RX 링버퍼가 꽉 차서 버린 프레임 */
    uint32_t rx_error;    /* can2040 NOTIFY_ERROR (PIO FIFO 오버플로) */
    uint32_t prio_drop;   /* 일회성 명령 큐가 꽉 차서 버린 프레임 */
} can_bus_stats_t;

void can_bus_init(void);

/* 일회성 명령(enable/disable/origin/raw)용. 큐에 넣고 순서대로 보낸다. */
bool can_bus_send_priority(const can_frame_t *f);

/* 주기 목표값용. 일회성 큐가 비어 있고 can2040 TX 큐에 자리가 있을 때만 보낸다. */
bool can_bus_try_send(const can_frame_t *f);

/* 일회성 큐를 can2040 으로 밀어 넣는다. 루프마다 호출. */
void can_bus_poll(void);

/* 수신 프레임 하나 꺼내기 (없으면 false) */
bool can_bus_recv(can_frame_t *f);

void can_bus_get_stats(can_bus_stats_t *s);

#endif /* CAN_BUS_H */
