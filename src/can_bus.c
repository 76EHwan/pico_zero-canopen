/*
 * can_bus.c - can2040 래퍼 (core1 전용)
 *
 * can2040 은 PIO 상태머신 3개 + 명령어 메모리를 통째로 쓰고,
 * 인터럽트 지연에 민감하다. 그래서
 *  - 펌웨어 전체를 RAM 에서 실행 (CMake: pico_set_binary_type copy_to_ram)
 *  - core1 에서 IRQ 를 최고 우선순위로 처리 (USB IRQ 는 core0)
 */
#include "can_bus.h"

#include <string.h>

#include "can2040.h"
#include "config.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

static struct can2040 s_cbus;

/* IRQ -> 루프 (같은 core1 이라 SPSC 링버퍼면 충분) */
#define RX_RING_SIZE 128 /* 2 의 거듭제곱 */
static can_frame_t s_rx_ring[RX_RING_SIZE];
static volatile uint32_t s_rx_head;
static uint32_t s_rx_tail;
static volatile uint32_t s_rx_overflow;
static volatile uint32_t s_rx_error;

/* 일회성 명령 큐 (루프 안에서만 사용) */
#define PRIO_SIZE 32 /* 2 의 거듭제곱 */
static can_frame_t s_prio[PRIO_SIZE];
static uint32_t s_prio_head, s_prio_tail;
static uint32_t s_prio_drop;

static void can_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {
    (void)cd;
    if (notify == CAN2040_NOTIFY_RX) {
        if (msg->id & CAN2040_ID_RTR) return;
        uint32_t head = s_rx_head;
        if (head - s_rx_tail >= RX_RING_SIZE) {
            s_rx_overflow++;
            return;
        }
        can_frame_t *f = &s_rx_ring[head & (RX_RING_SIZE - 1)];
        f->ext = (msg->id & CAN2040_ID_EFF) != 0;
        f->id = msg->id & (f->ext ? 0x1FFFFFFFu : 0x7FFu);
        f->dlc = (uint8_t)(msg->dlc > 8 ? 8 : msg->dlc);
        memcpy(f->data, msg->data, 8);
        __compiler_memory_barrier();
        s_rx_head = head + 1;
    } else if (notify == CAN2040_NOTIFY_ERROR) {
        s_rx_error++;
    }
}

static void pio_irq_handler(void) {
    can2040_pio_irq_handler(&s_cbus);
}

void can_bus_init(void) {
    can2040_setup(&s_cbus, CAN_PIO_NUM);
    can2040_callback_config(&s_cbus, can_cb);

#if CAN_PIO_NUM == 0
    const uint irq = PIO0_IRQ_0;
#elif CAN_PIO_NUM == 1
    const uint irq = PIO1_IRQ_0;
#elif CAN_PIO_NUM == 2 && PICO_RP2350
    const uint irq = PIO2_IRQ_0;
#else
#error "CAN_PIO_NUM 은 0, 1 (RP2350 은 2 도 가능) 이어야 합니다"
#endif
    irq_set_exclusive_handler(irq, pio_irq_handler);
    irq_set_priority(irq, PICO_HIGHEST_IRQ_PRIORITY);
    irq_set_enabled(irq, true);

    can2040_start(&s_cbus, clock_get_hz(clk_sys), CAN_BITRATE, CAN_GPIO_RX, CAN_GPIO_TX);
}

static bool hw_transmit(const can_frame_t *f) {
    if (!can2040_check_transmit(&s_cbus)) return false;
    struct can2040_msg m;
    m.id = f->ext ? ((f->id & 0x1FFFFFFFu) | CAN2040_ID_EFF) : (f->id & 0x7FFu);
    m.dlc = f->dlc > 8 ? 8 : f->dlc;
    memcpy(m.data, f->data, 8);
    return can2040_transmit(&s_cbus, &m) == 0;
}

void can_bus_poll(void) {
    while (s_prio_tail != s_prio_head) {
        if (!hw_transmit(&s_prio[s_prio_tail & (PRIO_SIZE - 1)])) break;
        s_prio_tail++;
    }
}

bool can_bus_send_priority(const can_frame_t *f) {
    if (s_prio_head - s_prio_tail >= PRIO_SIZE) {
        s_prio_drop++;
        return false;
    }
    s_prio[s_prio_head & (PRIO_SIZE - 1)] = *f;
    s_prio_head++;
    can_bus_poll();
    return true;
}

bool can_bus_try_send(const can_frame_t *f) {
    can_bus_poll();
    if (s_prio_tail != s_prio_head) return false; /* 일회성 명령이 먼저 나가야 함 */
    return hw_transmit(f);
}

bool can_bus_recv(can_frame_t *f) {
    if (s_rx_tail == s_rx_head) return false;
    __compiler_memory_barrier();
    *f = s_rx_ring[s_rx_tail & (RX_RING_SIZE - 1)];
    __compiler_memory_barrier();
    s_rx_tail++;
    return true;
}

void can_bus_get_stats(can_bus_stats_t *s) {
    struct can2040_stats cs;
    can2040_get_statistics(&s_cbus, &cs);
    s->rx_total = cs.rx_total;
    s->tx_total = cs.tx_total;
    s->tx_attempt = cs.tx_attempt;
    s->parse_error = cs.parse_error;
    s->rx_overflow = s_rx_overflow;
    s->rx_error = s_rx_error;
    s->prio_drop = s_prio_drop;
}
