/*
 * serial_cmd.c - USB 시리얼 텍스트 명령 파서 (core0)
 *
 * 형식: 한 줄 = 명령 하나 또는 ';' 로 구분된 여러 명령, 끝은 '\n'
 *       토큰 구분은 공백 또는 ','.  대소문자 무시.
 *       모터 번호 m = 1..3, 또는 '*' / A / ALL / 0 = 전체
 *
 * 목표값 명령(MIT/POS/PV/VEL/CUR/BRK/DUTY)은 성공 시 응답하지 않는다
 * (ACK 1 로 켜면 OK 응답). 실패하면 항상 "ERR <명령> <이유>".
 * 설정 명령은 "OK <명령>" 으로 응답한다.
 */
#include "serial_cmd.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "config.h"
#include "hardware/clocks.h"
#include "motor_ctrl.h"
#include "pico/stdlib.h"
#include "usb_io.h"

#define MAX_ARGS 24

static uint32_t s_telem_hz = TELEM_RATE_HZ_DEFAULT;
static bool s_ack;

/* ------------------------------------------------------------------ */
/* 파싱 헬퍼                                                            */
/* ------------------------------------------------------------------ */

static void reply_ok(const char *name) {
    usb_io_printf(true, "OK %s\n", name);
}

static void reply_err(const char *name, const char *why) {
    usb_io_printf(true, "ERR %s %s\n", name, why);
}

static bool parse_f(const char *s, float *out) {
    char *end;
    float v = strtof(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) return false;
    *out = v;
    return true;
}

static bool parse_u(const char *s, uint32_t lo, uint32_t hi, uint32_t *out) {
    char *end;
    bool hex = s[0] == '0' && (s[1] == 'x' || s[1] == 'X'); /* 0x 접두사만 16진, 나머지는 10진 */
    unsigned long v = strtoul(hex ? s + 2 : s, &end, hex ? 16 : 10);
    if (end == (hex ? s + 2 : s) || *end != '\0' || *s == '-' || v < lo || v > hi) return false;
    *out = (uint32_t)v;
    return true;
}

static bool parse_motor(const char *s, uint8_t *out, bool allow_all) {
    if (allow_all && (!strcmp(s, "*") || !strcmp(s, "0") || !strcasecmp(s, "A") || !strcasecmp(s, "ALL"))) {
        *out = MOTOR_ALL;
        return true;
    }
    uint32_t v;
    if (!parse_u(s, 1, NUM_MOTORS, &v)) return false;
    *out = (uint8_t)(v - 1);
    return true;
}

static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void post(const char *name, const host_cmd_t *c, bool ack) {
    if (!motor_ctrl_post(c)) {
        reply_err(name, "busy");
        return;
    }
    if (ack) reply_ok(name);
}

/* ------------------------------------------------------------------ */
/* 출력                                                                */
/* ------------------------------------------------------------------ */

void serial_cmd_print_fb(int i, bool must) {
    motor_fb_t fb;
    motor_ctrl_get_fb(i, &fb);
    if (!fb.valid) {
        if (must) usb_io_printf(true, "FB %d nodata\n", i + 1);
        return;
    }
    uint32_t age = to_ms_since_boot(get_absolute_time()) - fb.last_ms;
    usb_io_printf(must, "FB %d %.4f %.3f %.3f %.2f %d %u %lu\n", i + 1, (double)fb.pos_rad, (double)fb.vel_rad_s,
                  (double)fb.torque_nm, (double)fb.current_a, fb.temp_c, fb.err, (unsigned long)age);
}

uint32_t serial_cmd_telem_hz(void) {
    return s_telem_hz;
}

static void print_info(void) {
    ctrl_status_t st;
    motor_ctrl_get_status(&st);
#if PICO_RP2350
    const char *chip = "RP2350";
#else
    const char *chip = "RP2040";
#endif
    usb_io_printf(true, "INFO fw=%s chip=%s sysclk=%lu can_bitrate=%d rx=GP%d tx=GP%d rate=%lu timeout=%lu telem=%lu ack=%d\n",
                  FW_VERSION, chip, (unsigned long)clock_get_hz(clk_sys), CAN_BITRATE, CAN_GPIO_RX, CAN_GPIO_TX,
                  (unsigned long)st.rate_hz, (unsigned long)st.timeout_ms, (unsigned long)s_telem_hz, s_ack ? 1 : 0);
    for (int i = 0; i < NUM_MOTORS; i++) {
        usb_io_printf(true, "INFO m%d id=%u proto=%s mode=%s\n", i + 1, st.can_id[i],
                      st.proto[i] == AK_PROTO_V2 ? "V2" : "V3", motor_mode_name(st.mode[i]));
    }
}

static void print_stat(void) {
    ctrl_status_t st;
    motor_ctrl_get_status(&st);
    usb_io_printf(true,
                  "STAT rx=%lu tx=%lu tx_attempt=%lu parse_err=%lu rx_ovf=%lu rx_err=%lu prio_drop=%lu "
                  "tick_skip=%lu overrun=%lu stalled=%d\n",
                  (unsigned long)st.can.rx_total, (unsigned long)st.can.tx_total, (unsigned long)st.can.tx_attempt,
                  (unsigned long)st.can.parse_error, (unsigned long)st.can.rx_overflow,
                  (unsigned long)st.can.rx_error, (unsigned long)st.can.prio_drop, (unsigned long)st.tick_skip,
                  (unsigned long)st.tick_overrun, st.bus_stalled ? 1 : 0);
}

static void print_help(void) {
    static const char *const lines[] = {
        "HELP 목표값 (응답 없음, ACK 1 로 켜기 가능). m = 1..3 또는 * (전체)",
        "HELP   MIT m p v kp kd t   MIT: p[rad] v[rad/s] kp[0-500] kd[0-5] t[Nm]",
        "HELP   POS m deg           Servo 위치 (출력축 deg)",
        "HELP   PV m deg erpm acc   Servo 위치+속도 (ERPM, ERPM/s)",
        "HELP   VEL m erpm          Servo 속도 (ERPM = 출력 rpm x 140)",
        "HELP   CUR m A | BRK m A | DUTY m d",
        "HELP 제어",
        "HELP   DIS m | STOP [m]    정지 (V3: disable)  | EN m  (V2: MIT 진입)",
        "HELP   ORIGIN m [0|1]      원점 설정 0=임시 1=영구",
        "HELP 설정",
        "HELP   ID m canid | PROTO m V3|V2 | RATE hz | TIMEOUT ms | TELEM hz | ACK 0|1",
        "HELP 진단",
        "HELP   PING | INFO | STAT | GET [m] | MON 0|1 | RAW E|S id_hex [data_hex] | K",
        "HELP 출력: FB m pos_rad vel_rad_s torque_Nm current_A temp_C err age_ms",
    };
    for (size_t i = 0; i < sizeof(lines) / sizeof(lines[0]); i++) usb_io_printf(true, "%s\n", lines[i]);
}

/* ------------------------------------------------------------------ */
/* 명령 핸들러                                                          */
/* ------------------------------------------------------------------ */

typedef void (*cmd_fn_t)(const char *name, int argc, char **argv);

static void setpoint(const char *name, int argc, char **argv, motor_mode_t mode, int nvals) {
    if (argc != 1 + nvals) {
        reply_err(name, "args");
        return;
    }
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_SETPOINT;
    c.mode = (uint8_t)mode;
    if (!parse_motor(argv[0], &c.motor, true)) {
        reply_err(name, "motor");
        return;
    }
    for (int k = 0; k < nvals; k++) {
        if (!parse_f(argv[1 + k], &c.sp[k])) {
            reply_err(name, "value");
            return;
        }
    }
    post(name, &c, s_ack);
}

static void cmd_mit(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_MIT, 5); }
static void cmd_pos(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_POS, 1); }
static void cmd_pv(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_POS_SPD, 3); }
static void cmd_vel(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_VEL, 1); }
static void cmd_cur(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_CUR, 1); }
static void cmd_brk(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_BRAKE, 1); }
static void cmd_duty(const char *n, int argc, char **argv) { setpoint(n, argc, argv, MODE_DUTY, 1); }

static void simple_motor_cmd(const char *name, int argc, char **argv, host_cmd_type_t type, bool default_all) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = (uint8_t)type;
    if (argc == 0 && default_all) {
        c.motor = MOTOR_ALL;
    } else if (argc != 1 || !parse_motor(argv[0], &c.motor, true)) {
        reply_err(name, "motor");
        return;
    }
    post(name, &c, true);
}

static void cmd_en(const char *n, int argc, char **argv) { simple_motor_cmd(n, argc, argv, CMD_ENABLE, false); }
static void cmd_dis(const char *n, int argc, char **argv) { simple_motor_cmd(n, argc, argv, CMD_DISABLE, false); }
static void cmd_stop(const char *n, int argc, char **argv) { simple_motor_cmd(n, argc, argv, CMD_DISABLE, true); }

static void cmd_origin(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_ORIGIN;
    if (argc < 1 || argc > 2 || !parse_motor(argv[0], &c.motor, true)) {
        reply_err(name, "args");
        return;
    }
    if (argc == 2 && !parse_u(argv[1], 0, 1, &c.u)) {
        reply_err(name, "mode");
        return;
    }
    post(name, &c, true);
}

static void cmd_id(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_SET_ID;
    if (argc != 2 || !parse_motor(argv[0], &c.motor, false) || !parse_u(argv[1], 0, 255, &c.u)) {
        reply_err(name, "args");
        return;
    }
    ctrl_status_t st;
    motor_ctrl_get_status(&st);
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (i != c.motor && st.can_id[i] == c.u) {
            reply_err(name, "duplicate");
            return;
        }
    }
    post(name, &c, true);
}

static void cmd_proto(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_SET_PROTO;
    if (argc != 2 || !parse_motor(argv[0], &c.motor, true)) {
        reply_err(name, "args");
        return;
    }
    if (!strcasecmp(argv[1], "V3") || !strcmp(argv[1], "3")) {
        c.u = AK_PROTO_V3;
    } else if (!strcasecmp(argv[1], "V2") || !strcmp(argv[1], "2")) {
        c.u = AK_PROTO_V2;
    } else {
        reply_err(name, "proto");
        return;
    }
    post(name, &c, true);
}

static void cmd_rate(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_SET_RATE;
    if (argc != 1 || !parse_u(argv[0], CTRL_RATE_HZ_MIN, CTRL_RATE_HZ_MAX, &c.u)) {
        reply_err(name, "range");
        return;
    }
    post(name, &c, true);
}

static void cmd_timeout(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_SET_TIMEOUT;
    if (argc != 1 || !parse_u(argv[0], 0, 60000, &c.u)) {
        reply_err(name, "range");
        return;
    }
    post(name, &c, true);
}

static void cmd_telem(const char *name, int argc, char **argv) {
    uint32_t hz;
    if (argc != 1 || !parse_u(argv[0], 0, TELEM_RATE_HZ_MAX, &hz)) {
        reply_err(name, "range");
        return;
    }
    s_telem_hz = hz;
    reply_ok(name);
}

static void cmd_ack(const char *name, int argc, char **argv) {
    uint32_t v;
    if (argc != 1 || !parse_u(argv[0], 0, 1, &v)) {
        reply_err(name, "args");
        return;
    }
    s_ack = v != 0;
    reply_ok(name);
}

static void cmd_mon(const char *name, int argc, char **argv) {
    uint32_t v;
    if (argc != 1 || !parse_u(argv[0], 0, 1, &v)) {
        reply_err(name, "args");
        return;
    }
    motor_ctrl_set_monitor(v != 0);
    reply_ok(name);
}

/* RAW E|S id_hex [data_hex ...]  예) RAW E 0F01   RAW S 1 FFFFFFFFFFFFFFFC */
static void cmd_raw(const char *name, int argc, char **argv) {
    host_cmd_t c;
    memset(&c, 0, sizeof(c));
    c.type = CMD_RAW;
    if (argc < 2) {
        reply_err(name, "args");
        return;
    }
    if (!strcasecmp(argv[0], "E")) {
        c.raw.ext = true;
    } else if (!strcasecmp(argv[0], "S")) {
        c.raw.ext = false;
    } else {
        reply_err(name, "type");
        return;
    }
    char *end;
    unsigned long id = strtoul(argv[1], &end, 16);
    if (end == argv[1] || *end != '\0' || id > (c.raw.ext ? 0x1FFFFFFFul : 0x7FFul)) {
        reply_err(name, "id");
        return;
    }
    c.raw.id = (uint32_t)id;
    for (int a = 2; a < argc; a++) {
        const char *h = argv[a];
        if (h[0] == '0' && (h[1] == 'x' || h[1] == 'X')) h += 2;
        size_t len = strlen(h);
        if (len % 2) {
            reply_err(name, "data");
            return;
        }
        for (size_t k = 0; k < len; k += 2) {
            int hi = hexval(h[k]), lo = hexval(h[k + 1]);
            if (hi < 0 || lo < 0 || c.raw.dlc >= 8) {
                reply_err(name, "data");
                return;
            }
            c.raw.data[c.raw.dlc++] = (uint8_t)((hi << 4) | lo);
        }
    }
    post(name, &c, true);
}

static void cmd_get(const char *name, int argc, char **argv) {
    uint8_t m = MOTOR_ALL;
    if (argc > 1 || (argc == 1 && !parse_motor(argv[0], &m, true))) {
        reply_err(name, "motor");
        return;
    }
    for (int i = 0; i < NUM_MOTORS; i++) {
        if (m == MOTOR_ALL || m == i) serial_cmd_print_fb(i, true);
    }
}

static void cmd_ping(const char *n, int argc, char **argv) {
    (void)n, (void)argc, (void)argv;
    usb_io_printf(true, "OK PONG\n");
}
static void cmd_keep(const char *n, int argc, char **argv) {
    (void)n, (void)argc, (void)argv; /* 호스트 생존 신호만 (handle_line 에서 처리됨) */
}
static void cmd_info(const char *n, int argc, char **argv) {
    (void)n, (void)argc, (void)argv;
    print_info();
}
static void cmd_stat(const char *n, int argc, char **argv) {
    (void)n, (void)argc, (void)argv;
    print_stat();
}
static void cmd_help(const char *n, int argc, char **argv) {
    (void)n, (void)argc, (void)argv;
    print_help();
}

static const struct {
    const char *name;
    cmd_fn_t fn;
} k_cmds[] = {
    {"MIT", cmd_mit},       {"POS", cmd_pos},     {"PV", cmd_pv},     {"VEL", cmd_vel},       {"CUR", cmd_cur},
    {"BRK", cmd_brk},       {"DUTY", cmd_duty},   {"EN", cmd_en},     {"DIS", cmd_dis},       {"STOP", cmd_stop},
    {"ORIGIN", cmd_origin}, {"ID", cmd_id},       {"PROTO", cmd_proto}, {"RATE", cmd_rate},   {"TIMEOUT", cmd_timeout},
    {"TELEM", cmd_telem},   {"ACK", cmd_ack},     {"MON", cmd_mon},   {"RAW", cmd_raw},       {"GET", cmd_get},
    {"PING", cmd_ping},     {"K", cmd_keep},      {"INFO", cmd_info}, {"STAT", cmd_stat},     {"HELP", cmd_help},
    {"?", cmd_help},
};

static void handle_one(char *s) {
    char *argv[MAX_ARGS];
    int argc = 0;
    char *p = s;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == ',') *p++ = '\0';
        if (!*p) break;
        if (argc == MAX_ARGS) {
            reply_err(argv[0], "too_many_args");
            return;
        }
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t' && *p != ',') p++;
    }
    if (argc == 0) return;

    for (size_t i = 0; i < sizeof(k_cmds) / sizeof(k_cmds[0]); i++) {
        if (!strcasecmp(argv[0], k_cmds[i].name)) {
            k_cmds[i].fn(k_cmds[i].name, argc - 1, argv + 1);
            return;
        }
    }
    reply_err(argv[0], "unknown");
}

void serial_cmd_handle_line(char *line) {
    motor_ctrl_touch_host();
    char *seg = line;
    for (;;) {
        char *semi = strchr(seg, ';');
        if (semi) *semi = '\0';
        handle_one(seg);
        if (!semi) break;
        seg = semi + 1;
    }
}

void serial_cmd_init(void) {
    s_telem_hz = TELEM_RATE_HZ_DEFAULT;
    s_ack = false;
}
