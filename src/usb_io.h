/*
 * usb_io.h - USB CDC 시리얼 줄 단위 입출력 (core0 전용)
 */
#ifndef USB_IO_H
#define USB_IO_H

#include <stdbool.h>
#include <stddef.h>

#define USB_LINE_MAX 256

typedef void (*usb_line_cb_t)(char *line);

/* 수신된 문자를 처리하고 완성된 줄마다 cb 호출 (non-blocking) */
void usb_io_poll(usb_line_cb_t cb);

bool usb_io_connected(void);

/* must = true : 응답/이벤트. 공간이 없으면 잠깐(수 ms) 기다렸다가 그래도 없으면 버림
 * must = false: 텔레메트리. 공간이 없으면 즉시 버림 (호스트가 안 읽어도 막히지 않음) */
bool usb_io_write(const char *s, size_t len, bool must);
bool usb_io_printf(bool must, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#endif /* USB_IO_H */
