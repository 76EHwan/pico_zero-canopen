/*
 * usb_io.c - pico_stdio_usb 위의 raw 입출력
 *
 * 출력 전에 TinyUSB CDC TX FIFO 여유 공간을 확인해서, 호스트가 포트를
 * 열어놓고 읽지 않는 경우에도 core0 루프가 멈추지 않게 한다.
 * (CMake 에서 CFG_TUD_CDC_TX_BUFSIZE 를 1024 로 키움)
 */
#include "usb_io.h"

#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "tusb.h"

#define MUST_WAIT_US 5000

size_t usb_io_read(char *buf, size_t max) {
    size_t n = 0;
    while (n < max) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c < 0) break;
        buf[n++] = (char)c;
    }
    return n;
}

bool usb_io_connected(void) {
    return stdio_usb_connected();
}

bool usb_io_write(const char *s, size_t len, bool must) {
    if (len == 0) return true;
    if (!stdio_usb_connected() || len > CFG_TUD_CDC_TX_BUFSIZE) return false;

    if (tud_cdc_write_available() < len) {
        if (!must) return false;
        absolute_time_t until = make_timeout_time_us(MUST_WAIT_US);
        while (tud_cdc_write_available() < len) {
            if (time_reached(until) || !stdio_usb_connected()) return false;
            tight_loop_contents();
        }
    }
    stdio_put_string(s, (int)len, false, false); /* slcan 은 '\r' 만 쓰므로 CRLF 변환 없음 */
    stdio_flush();
    return true;
}
