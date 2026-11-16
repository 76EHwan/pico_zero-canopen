/*
 * slcan.c - Lawicel(slcan) ASCII 프로토콜 (CAN232/CANUSB 호환 부분집합)
 *
 * 호스트 -> 보드 명령 (끝은 '\r'):
 *   Sn      비트레이트 (0~8, 가상 버스라 저장만 함)    응답 '\r'
 *   sxxyy   BTR 직접 설정 (무시)                      응답 '\r'
 *   O / L   채널 열기 / listen-only 로 열기            응답 '\r'
 *   C       채널 닫기                                 응답 '\r'
 *   tiiildd...  표준 프레임 송신                      응답 'z\r'
 *   Tiiiiiiiildd...  확장 프레임 송신                 응답 'Z\r'
 *   riiil / Riiiiiiiil  RTR 프레임 (CANopen 에서 안 씀, 무시)
 *   V / v / N  버전 / 시리얼 번호
 *   F       상태 플래그 (bit3 = 오버런)
 *   Zn      타임스탬프 끄기/켜기
 *   잘못된 명령은 BEL('\a')
 *
 * 보드 -> 호스트: "tiiildd..\r" / "Tiiiiiiiildd..\r" (+ 타임스탬프 4자리)
 */
#include "slcan.h"

#include <string.h>

#define LINE_MAX 40

static const slcan_io_t *s_io;
static char s_line[LINE_MAX];
static size_t s_len;
static bool s_overflow;
static bool s_open;
static bool s_listen_only;
static bool s_timestamp;
static uint32_t s_drops;
static bool s_overrun_flag;

static const char k_hex[] = "0123456789ABCDEF";

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static bool parse_hex(const char *s, size_t n, uint32_t *out) {
    uint32_t v = 0;
    for (size_t i = 0; i < n; i++) {
        int h = hexval(s[i]);
        if (h < 0) return false;
        v = (v << 4) | (uint32_t)h;
    }
    *out = v;
    return true;
}

static void reply(const char *s) {
    if (s_io && s_io->write) s_io->write(s, strlen(s), true);
}

bool slcan_parse_frame(const char *line, size_t len, can_frame_t *f, bool *rtr) {
    if (len < 1) return false;
    char c = line[0];
    bool ext = (c == 'T' || c == 'R');
    bool is_rtr = (c == 'r' || c == 'R');
    if (c != 't' && c != 'T' && c != 'r' && c != 'R') return false;

    size_t id_len = ext ? 8 : 3;
    if (len < 1 + id_len + 1) return false;
    uint32_t id, dlc;
    if (!parse_hex(&line[1], id_len, &id) || !parse_hex(&line[1 + id_len], 1, &dlc)) return false;
    if (dlc > 8) return false;
    if (id > (ext ? 0x1FFFFFFFu : 0x7FFu)) return false;

    size_t data_pos = 2 + id_len;
    size_t need = data_pos + (is_rtr ? 0 : dlc * 2);
    /* 끝에 타임스탬프(4자리)가 붙어 있어도 허용 */
    if (len != need && len != need + 4) return false;

    memset(f, 0, sizeof(*f));
    f->id = id;
    f->ext = ext;
    f->dlc = (uint8_t)dlc;
    if (!is_rtr) {
        for (uint32_t i = 0; i < dlc; i++) {
            uint32_t b;
            if (!parse_hex(&line[data_pos + 2 * i], 2, &b)) return false;
            f->data[i] = (uint8_t)b;
        }
    }
    *rtr = is_rtr;
    return true;
}

size_t slcan_format_frame(const can_frame_t *f, char *out, bool timestamp, uint16_t ts_ms) {
    size_t n = 0;
    uint8_t dlc = f->dlc > 8 ? 8 : f->dlc;
    if (f->ext) {
        out[n++] = 'T';
        for (int s = 28; s >= 0; s -= 4) out[n++] = k_hex[(f->id >> s) & 0xF];
    } else {
        out[n++] = 't';
        for (int s = 8; s >= 0; s -= 4) out[n++] = k_hex[(f->id >> s) & 0xF];
    }
    out[n++] = k_hex[dlc];
    for (uint8_t i = 0; i < dlc; i++) {
        out[n++] = k_hex[f->data[i] >> 4];
        out[n++] = k_hex[f->data[i] & 0xF];
    }
    if (timestamp) {
        for (int s = 12; s >= 0; s -= 4) out[n++] = k_hex[(ts_ms >> s) & 0xF];
    }
    out[n++] = '\r';
    return n;
}

static void do_close(void) {
    if (!s_open) return;
    s_open = false;
    s_listen_only = false;
    if (s_io && s_io->on_close) s_io->on_close();
}

static void handle_line(const char *line, size_t len) {
    switch (line[0]) {
    case 'O':
    case 'L':
        if (!s_open) {
            s_open = true;
            s_listen_only = (line[0] == 'L');
            reply("\r");
            if (s_io && s_io->on_open) s_io->on_open();
        } else {
            reply("\r");
        }
        return;
    case 'C':
        do_close();
        reply("\r");
        return;
    case 'S':
        reply(len == 2 && line[1] >= '0' && line[1] <= '8' ? "\r" : "\a");
        return;
    case 's':
    case 'M':
    case 'm':
    case 'W':
    case 'X':
    case 'Q':
    case 'U':
        reply("\r");
        return;
    case 'Z':
        if (len == 2 && (line[1] == '0' || line[1] == '1')) {
            s_timestamp = (line[1] == '1');
            reply("\r");
        } else {
            reply("\a");
        }
        return;
    case 'V':
        reply("V0101\r");
        return;
    case 'v':
        reply("v0101\r");
        return;
    case 'N':
        reply("NPZ01\r");
        return;
    case 'F': {
        char buf[5] = {'F', '0', s_overrun_flag ? '8' : '0', '\r', '\0'};
        s_overrun_flag = false;
        reply(buf);
        return;
    }
    case 't':
    case 'T':
    case 'r':
    case 'R': {
        can_frame_t f;
        bool rtr;
        if (!s_open || s_listen_only || !slcan_parse_frame(line, len, &f, &rtr)) {
            reply("\a");
            return;
        }
        reply(line[0] == 't' || line[0] == 'r' ? "z\r" : "Z\r");
        if (!rtr && s_io && s_io->on_frame) s_io->on_frame(&f);
        return;
    }
    default:
        reply("\a");
        return;
    }
}

void slcan_init(const slcan_io_t *io) {
    s_io = io;
    s_len = 0;
    s_overflow = false;
    s_open = false;
    s_listen_only = false;
    s_timestamp = false;
    s_drops = 0;
    s_overrun_flag = false;
}

void slcan_feed(const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        char c = data[i];
        if (c == '\r' || c == '\n') {
            if (s_overflow) {
                reply("\a");
            } else if (s_len > 0) {
                handle_line(s_line, s_len);
            }
            s_len = 0;
            s_overflow = false;
        } else if (s_len < LINE_MAX - 1) {
            s_line[s_len++] = c;
        } else {
            s_overflow = true;
        }
    }
}

bool slcan_send(const can_frame_t *f, bool must) {
    if (!s_open || !s_io || !s_io->write) return false;
    char buf[32];
    uint16_t ts = 0;
    if (s_timestamp && s_io->now_ms) ts = (uint16_t)(s_io->now_ms() % 60000u);
    size_t n = slcan_format_frame(f, buf, s_timestamp, ts);
    if (!s_io->write(buf, n, must)) {
        s_drops++;
        s_overrun_flag = true;
        return false;
    }
    return true;
}

bool slcan_is_open(void) {
    return s_open;
}

void slcan_close(void) {
    do_close();
}

uint32_t slcan_drops(void) {
    return s_drops;
}
