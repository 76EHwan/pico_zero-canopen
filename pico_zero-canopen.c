/*
 * pico_zero-canopen
 *
 * RP2040-Zero / RP2350-Zero 를 CANopen(CiA 301/402) 드라이브 게이트웨이로 만든다.
 *
 *   CANopen 마스터 (Jetson Orin: ros2_canopen / python-canopen)
 *        │ USB CDC 시리얼 위의 slcan (리눅스: slcand -> SocketCAN slcan0)
 *   RP2350-Zero : CANopen 노드 3개 (노드 ID 1,2,3 = CiA 402 단일 축 드라이브 3대)
 *        │ CAN 1 Mbps, CubeMars 자체 프로토콜 (Servo / MIT)
 *   AK40-10 x3
 *
 *  core0: slcan, CANopen 스택, CiA 402 상태머신, 상태 LED, 워치독
 *  core1: can2040 IRQ + 고정 주기 모터 제어 루프 (src/motor_ctrl.c)
 */
#include "cia402.h"
#include "co_node.h"
#include "config.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "motor_ctrl.h"
#include "pico/multicore.h"
#include "pico/status_led.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "slcan.h"
#include "usb_io.h"

static co_node_t s_node[NUM_MOTORS];
static const uint8_t k_node_id[NUM_MOTORS] = {CO_NODE_ID_1, CO_NODE_ID_2, CO_NODE_ID_3};
static bool s_bus_stalled;

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

/* ---- slcan <-> CANopen 연결 ---- */

static void co_send_cb(const can_frame_t *f, bool important) {
    slcan_send(f, important);
}

static void on_host_frame(const can_frame_t *f) {
    uint64_t t = time_us_64();
    for (int i = 0; i < NUM_MOTORS; i++) co_rx(&s_node[i], f, t);
}

static void on_open(void) {
    /* 호스트가 채널을 열면 전원 투입처럼: 노드 초기화 + boot-up 송신 */
    uint64_t t = time_us_64();
    for (int i = 0; i < NUM_MOTORS; i++) co_reset_node(&s_node[i], t);
}

static void on_close(void) {
    /* 마스터와 연결이 끊김 -> 0x6007 에 따라 정지 */
    for (int i = 0; i < NUM_MOTORS; i++) co_app_comm_lost(&s_node[i], 0x8100);
}

static const slcan_io_t k_slcan_io = {
    .write = usb_io_write,
    .on_frame = on_host_frame,
    .on_open = on_open,
    .on_close = on_close,
    .now_ms = now_ms,
};

/* 상태 LED (보드의 WS2812, GP16)
 *  파랑: slcan 채널 닫힘 / 노랑: 열림, 운전 중인 축 없음
 *  초록: 운전 중(Operation enabled) / 빨강: 고장 또는 모터 CAN ACK 없음 */
static void update_led(void) {
    static uint32_t last = 0xFFFFFFFFu;
    bool fault = s_bus_stalled, enabled = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        cia402_state_t st = cia402_state(&s_node[i]);
        fault |= (st == CIA402_FAULT || st == CIA402_FAULT_REACTION_ACTIVE);
        enabled |= (st == CIA402_OPERATION_ENABLED);
    }
    uint32_t color;
    if (!slcan_is_open()) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 0, 24);
    } else if (fault) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(40, 0, 0);
    } else if (enabled) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 30, 0);
    } else {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(30, 18, 0);
    }
    if (color == last) return;
    last = color;
    if (colored_status_led_supported()) {
        colored_status_led_set_on_with_color(color);
    } else if (status_led_supported()) {
        status_led_set_state(slcan_is_open() && !fault);
    }
}

int main(void) {
    stdio_init_all();

    /* can2040 이 이 PIO 블록 전체를 쓴다. 상태 LED 가 이 블록을 고르지 않도록 먼저 예약 */
    pio_claim_sm_mask(pio_get_instance(CAN_PIO_NUM), 0xF);

    motor_ctrl_init();
    multicore_launch_core1(motor_ctrl_core1_entry);

    status_led_init();
    watchdog_enable(WATCHDOG_MS, true);

    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    uint32_t serial = ((uint32_t)uid.id[4] << 24) | ((uint32_t)uid.id[5] << 16) | ((uint32_t)uid.id[6] << 8) | uid.id[7];

    co_set_sender(co_send_cb);
    slcan_init(&k_slcan_io);
    uint64_t t0 = time_us_64();
    for (int i = 0; i < NUM_MOTORS; i++) co_node_init(&s_node[i], k_node_id[i], (uint8_t)i, serial + (uint32_t)i, t0);

    uint32_t next_wd_ms = 0, next_led_ms = 0;
    uint32_t last_hb = motor_ctrl_heartbeat();

    for (;;) {
        char buf[256];
        size_t n = usb_io_read(buf, sizeof(buf));
        if (n) slcan_feed(buf, n);
        if (slcan_is_open() && !usb_io_connected()) slcan_close(); /* 포트가 닫힘 (DTR off) */

        uint64_t t = time_us_64();
        for (int i = 0; i < NUM_MOTORS; i++) {
            cia402_process(&s_node[i], t);
            if (slcan_is_open()) co_process(&s_node[i], t);
        }

        motor_evt_t e;
        while (motor_ctrl_pop_event(&e)) {
            if (e.type == EVT_CAN_STALL) s_bus_stalled = true;
            if (e.type == EVT_CAN_OK) s_bus_stalled = false;
        }

        uint32_t tm = now_ms();
        if ((int32_t)(tm - next_led_ms) >= 0) {
            next_led_ms = tm + 100;
            update_led();
        }
        /* core1 루프가 살아 있을 때만 워치독을 먹인다 */
        if ((int32_t)(tm - next_wd_ms) >= 0) {
            next_wd_ms = tm + 50;
            uint32_t hb = motor_ctrl_heartbeat();
            if (hb != last_hb) {
                last_hb = hb;
                watchdog_update();
            }
        }
    }
}
