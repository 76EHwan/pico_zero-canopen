/*
 * slcan.h - Lawicel(slcan) 프로토콜: USB 시리얼 위로 CAN 프레임을 텍스트로 주고받는다.
 *
 * 하드웨어 의존성이 없다 (입출력은 콜백). 리눅스에서는
 *   slcand -o -c -s8 /dev/ttyACM0 slcan0 && ip link set slcan0 up
 * 로 이 보드가 SocketCAN 인터페이스 slcan0 이 된다. python-can 의 'slcan'
 * 인터페이스(Windows 포함)로도 바로 열 수 있다.
 *
 * 이 펌웨어에서 slcan "버스" 는 가상이다: 호스트가 보낸 프레임은 보드 안의
 * CANopen 노드들이 받고, CANopen 노드들이 만든 프레임은 호스트로 간다.
 */
#ifndef SLCAN_H
#define SLCAN_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ak_proto.h" /* can_frame_t */

typedef struct {
    bool (*write)(const char *s, size_t len, bool must);
    void (*on_frame)(const can_frame_t *f); /* 호스트 -> 보드 데이터 프레임 */
    void (*on_open)(void);                  /* 'O' 또는 'L' */
    void (*on_close)(void);                 /* 'C' 또는 slcan_close() */
    uint32_t (*now_ms)(void);               /* 타임스탬프용 */
} slcan_io_t;

void slcan_init(const slcan_io_t *io);

/* 호스트에서 받은 바이트 */
void slcan_feed(const char *data, size_t len);

/* 보드 -> 호스트 프레임. 채널이 닫혀 있거나 USB 버퍼가 차면 false */
bool slcan_send(const can_frame_t *f, bool must);

bool slcan_is_open(void);

/* USB 연결이 끊겼을 때 등 강제로 닫기 */
void slcan_close(void);

/* 보내지 못하고 버린 프레임 수 */
uint32_t slcan_drops(void);

/* ---- 순수 코덱 (테스트용으로 공개) ---- */
/* "t1238AABB..." / "T12345678..." 줄 해석. RTR 이면 *rtr = true */
bool slcan_parse_frame(const char *line, size_t len, can_frame_t *f, bool *rtr);
/* 프레임 -> "t...\r" 문자열. 반환값은 길이 (out 은 32 바이트 이상) */
size_t slcan_format_frame(const can_frame_t *f, char *out, bool timestamp, uint16_t ts_ms);

#endif /* SLCAN_H */
