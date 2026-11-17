/*
 * usb_io.h - USB CDC 시리얼 raw 입출력 (core0 전용)
 */
#ifndef USB_IO_H
#define USB_IO_H

#include <stdbool.h>
#include <stddef.h>

/* 지금 도착해 있는 바이트를 최대 max 개 읽는다 (non-blocking) */
size_t usb_io_read(char *buf, size_t max);

bool usb_io_connected(void);

/* must = true : 응답. 공간이 없으면 잠깐(수 ms) 기다렸다가 그래도 없으면 버림
 * must = false: PDO/heartbeat 등. 공간이 없으면 즉시 버림 (호스트가 안 읽어도 막히지 않음) */
bool usb_io_write(const char *s, size_t len, bool must);

#endif /* USB_IO_H */
