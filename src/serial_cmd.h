/*
 * serial_cmd.h - USB 시리얼 텍스트 명령 파서 (core0)
 */
#ifndef SERIAL_CMD_H
#define SERIAL_CMD_H

#include <stdbool.h>
#include <stdint.h>

void serial_cmd_init(void);

/* 한 줄 처리. ';' 로 여러 명령을 한 줄에 보낼 수 있다. */
void serial_cmd_handle_line(char *line);

/* 텔레메트리 주기 (0 = 끔) */
uint32_t serial_cmd_telem_hz(void);

/* FB 줄 출력. must=false 면 USB 버퍼가 차 있을 때 버린다. */
void serial_cmd_print_fb(int idx, bool must);

#endif /* SERIAL_CMD_H */
