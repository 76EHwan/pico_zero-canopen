/*
 * pico_zero-canopen
 *
 * RP2040-Zero / RP2350-Zero + SN65HVD230 로 CubeMars AK40-10 3대를 CAN 제어.
 * 중앙 제어부 -> USB CDC 시리얼(텍스트 명령) -> 이 보드 -> CAN 1 Mbps -> 모터 3대
 *
 * 참고: AK40-10 은 CANopen(CiA 301/402) 이 아니라 CubeMars 자체 CAN 프로토콜
 *       (Servo 모드 / MIT 모드) 을 쓴다. src/ak_proto.c 참고.
 *
 *  core0: USB 시리얼 파서, 텔레메트리/이벤트 출력, 상태 LED, 워치독
 *  core1: can2040 IRQ + 고정 주기 제어 루프 (src/motor_ctrl.c)
 */
#include <stdio.h>

#include "config.h"
#include "hardware/pio.h"
#include "hardware/watchdog.h"
#include "motor_ctrl.h"
#include "pico/multicore.h"
#include "pico/status_led.h"
#include "pico/stdlib.h"
#include "serial_cmd.h"
#include "usb_io.h"

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void print_event(const motor_evt_t *e) {
    int m = e->motor + 1;
    switch (e->type) {
    case EVT_BOOT: usb_io_printf(true, "EVT BOOT %d\n", m); break;
    case EVT_DISABLED: usb_io_printf(true, "EVT DISABLED %d\n", m); break;
    case EVT_FAULT: usb_io_printf(true, "EVT FAULT %d %u %s\n", m, e->code, ak_err_name((uint8_t)e->code)); break;
    case EVT_FAULT_CLEAR: usb_io_printf(true, "EVT FAULT_CLEAR %d\n", m); break;
    case EVT_TIMEOUT: usb_io_printf(true, "EVT TIMEOUT all_motors_stopped\n"); break;
    case EVT_CAN_OVERFLOW: usb_io_printf(true, "EVT CAN_OVERFLOW %u\n", e->code); break;
    case EVT_CAN_STALL: usb_io_printf(true, "EVT CAN_STALL no_ack\n"); break;
    case EVT_CAN_OK: usb_io_printf(true, "EVT CAN_OK\n"); break;
    default: break;
    }
}

static void print_monitor(const can_frame_t *f) {
    char buf[48];
    int n = snprintf(buf, sizeof(buf), f->ext ? "RX E %08lX %u " : "RX S %03lX %u ", (unsigned long)f->id, f->dlc);
    for (int i = 0; i < f->dlc && n < (int)sizeof(buf) - 3; i++) n += snprintf(buf + n, sizeof(buf) - n, "%02X", f->data[i]);
    buf[n++] = '\n';
    usb_io_write(buf, (size_t)n, false);
}

/* 상태 LED (보드의 WS2812, GP16)
 *  파랑: USB 미연결 / 노랑: 연결됨, 모터 피드백 없음
 *  초록: 모터 피드백 수신 중 / 빨강: 모터 에러 또는 CAN ACK 없음 */
static void update_led(bool connected) {
    static uint32_t last_color = 0xFFFFFFFFu;
    uint32_t t = now_ms();
    bool fresh = false, fault = false;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_fb_t fb;
        motor_ctrl_get_fb(i, &fb);
        if (fb.valid && (t - fb.last_ms) < FB_STALE_MS) fresh = true;
        if (fb.valid && fb.err != 0) fault = true;
    }
    ctrl_status_t st;
    motor_ctrl_get_status(&st);

    uint32_t color;
    if (fault || st.bus_stalled) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(40, 0, 0);
    } else if (!connected) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 0, 24);
    } else if (fresh) {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(0, 30, 0);
    } else {
        color = PICO_COLORED_STATUS_LED_COLOR_FROM_RGB(30, 18, 0);
    }
    if (color == last_color) return;
    last_color = color;

    if (colored_status_led_supported()) {
        colored_status_led_set_on_with_color(color);
    } else if (status_led_supported()) {
        status_led_set_state(connected && fresh && !fault);
    }
}

int main(void) {
    bool wd_reboot = watchdog_caused_reboot();
    stdio_init_all();

    /* can2040 이 이 PIO 블록의 상태머신/명령어 메모리를 전부 쓴다.
     * 다른 PIO 사용자(상태 LED)가 이 블록을 고르지 않도록 먼저 예약. */
    pio_claim_sm_mask(pio_get_instance(CAN_PIO_NUM), 0xF);

    motor_ctrl_init();
    serial_cmd_init();
    multicore_launch_core1(motor_ctrl_core1_entry);

    status_led_init();
    watchdog_enable(WATCHDOG_MS, true);

    bool was_connected = false;
    uint64_t next_telem_us = 0;
    uint32_t next_wd_ms = 0, next_led_ms = 0;
    uint32_t last_hb = motor_ctrl_heartbeat();

    for (;;) {
        uint32_t t = now_ms();

        usb_io_poll(serial_cmd_handle_line);

        bool connected = usb_io_connected();
        if (connected && !was_connected) {
            usb_io_printf(true, "READY pico_zero-canopen fw=%s motors=%d (HELP 로 명령 목록)\n", FW_VERSION, NUM_MOTORS);
            if (wd_reboot) usb_io_printf(true, "EVT RESET watchdog\n");
        }
        was_connected = connected;

        motor_evt_t e;
        for (int n = 0; n < 16 && motor_ctrl_pop_event(&e); n++) print_event(&e);

        can_frame_t f;
        for (int n = 0; n < 32 && motor_ctrl_pop_monitor(&f); n++) print_monitor(&f);

        uint32_t hz = serial_cmd_telem_hz();
        if (hz && connected) {
            uint64_t t_us = time_us_64();
            if (t_us >= next_telem_us) {
                next_telem_us = t_us + 1000000u / hz;
                for (int i = 0; i < NUM_MOTORS; i++) serial_cmd_print_fb(i, false);
            }
        }

        if ((int32_t)(t - next_led_ms) >= 0) {
            next_led_ms = t + 100;
            update_led(connected);
        }

        /* core1 루프가 살아 있을 때만 워치독을 먹인다 */
        if ((int32_t)(t - next_wd_ms) >= 0) {
            next_wd_ms = t + 50;
            uint32_t hb = motor_ctrl_heartbeat();
            if (hb != last_hb) {
                last_hb = hb;
                watchdog_update();
            }
        }
    }
}
