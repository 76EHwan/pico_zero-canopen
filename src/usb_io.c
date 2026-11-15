/*
 * usb_io.c - pico_stdio_usb 위에서 줄 단위 입출력
 *
 * 출력은 TinyUSB CDC TX FIFO 여유 공간을 먼저 확인해서, 호스트가 포트를
 * 열어놓고 읽지 않는 경우에도 core0 루프가 멈추지 않게 한다.
 * (CMake 에서 CFG_TUD_CDC_TX_BUFSIZE 를 1024 로 키움)
 */
#include "usb_io.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "tusb.h"

#define MUST_WAIT_US 5000

static char s_line[USB_LINE_MAX];
static size_t s_len;
static bool s_overflow;

void usb_io_poll(usb_line_cb_t cb) {
    for (int n = 0; n < 1024; n++) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c < 0) return;
        if (c == '\r' || c == '\n') {
            if (s_overflow) {
                usb_io_printf(true, "ERR line_too_long\n");
            } else if (s_len > 0) {
                s_line[s_len] = '\0';
                cb(s_line);
            }
            s_len = 0;
            s_overflow = false;
            continue;
        }
        if (s_len < USB_LINE_MAX - 1) {
            s_line[s_len++] = (char)c;
        } else {
            s_overflow = true;
        }
    }
}

bool usb_io_connected(void) {
    return stdio_usb_connected();
}

/* '\n' 은 stdio 의 CRLF 변환으로 2 바이트가 된다 */
static size_t wire_len(const char *s, size_t len) {
    size_t n = len;
    for (size_t i = 0; i < len; i++) n += (s[i] == '\n');
    return n;
}

bool usb_io_write(const char *s, size_t len, bool must) {
    if (len == 0) return true;
    if (!stdio_usb_connected()) return false;
    size_t need = wire_len(s, len);
    if (need > CFG_TUD_CDC_TX_BUFSIZE) return false;

    if (tud_cdc_write_available() < need) {
        if (!must) return false;
        absolute_time_t until = make_timeout_time_us(MUST_WAIT_US);
        while (tud_cdc_write_available() < need) {
            if (time_reached(until) || !stdio_usb_connected()) return false;
            tight_loop_contents();
        }
    }
    stdio_put_string(s, (int)len, false, true);
    stdio_flush();
    return true;
}

bool usb_io_printf(bool must, const char *fmt, ...) {
    char buf[320];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return false;
    if ((size_t)n >= sizeof(buf)) n = sizeof(buf) - 1;
    return usb_io_write(buf, (size_t)n, must);
}
