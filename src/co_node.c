/*
 * co_node.c - CANopen 슬레이브 프로토콜 (CiA 301)
 *
 * COB-ID (기본 연결 세트, 노드 ID = n):
 *   0x000      NMT (마스터 -> 슬레이브)
 *   0x080      SYNC
 *   0x080 + n  EMCY
 *   0x180 + n  TPDO1 ... 0x480 + n TPDO4
 *   0x200 + n  RPDO1 ... 0x500 + n RPDO4
 *   0x580 + n  SDO 응답 / 0x600 + n SDO 요청
 *   0x700 + n  Heartbeat / boot-up
 */
#include "co_node.h"

#include <string.h>

enum { SDO_IDLE = 0, SDO_DOWN, SDO_UP };

#define SDO_TIMEOUT_US 1000000u
#define COB_INVALID 0x80000000u

static co_send_fn_t s_send;

void co_set_sender(co_send_fn_t fn) {
    s_send = fn;
}

static void send(uint32_t id, const uint8_t *data, uint8_t dlc, bool important) {
    if (!s_send) return;
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id & 0x7FF;
    f.ext = false;
    f.dlc = dlc;
    if (dlc) memcpy(f.data, data, dlc);
    s_send(&f, important);
}

static uint32_t le32(const uint8_t *b) {
    return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
}

/* ------------------------------------------------------------------ */
/* PDO 매핑 캐시                                                        */
/* ------------------------------------------------------------------ */

static void rebuild_cache(co_pdo_cache_t *c, const co_pdo_map_t *m) {
    memset(c, 0, sizeof(*c));
    uint8_t len = 0;
    for (uint8_t k = 0; k < m->count && k < CO_PDO_MAX_MAP; k++) {
        uint32_t e = m->map[k];
        const od_sub_t *s = od_lookup((uint16_t)(e >> 16), (uint8_t)(e >> 8));
        if (!s) { /* 검증된 매핑이라 일어나지 않아야 함 */
            memset(c, 0, sizeof(*c));
            return;
        }
        c->index[k] = (uint16_t)(e >> 16);
        c->ent[k] = s;
        len += (uint8_t)od_type_size(s->type);
    }
    c->n = m->count;
    c->len = len;
}

static void reset_tpdo_rt(co_node_t *n, int i) {
    memset(&n->tpdo_rt[i], 0, sizeof(n->tpdo_rt[i]));
}

static void rebuild_rpdo(co_node_t *n, int i) {
    rebuild_cache(&n->rpdo[i], &n->od.rpdo_map[i]);
    n->rpdo_buf[i].pending = false;
}

static void rebuild_tpdo(co_node_t *n, int i) {
    rebuild_cache(&n->tpdo[i], &n->od.tpdo_map[i]);
    reset_tpdo_rt(n, i);
}

/* ------------------------------------------------------------------ */
/* 통신 오브젝트 값 검증                                                */
/* ------------------------------------------------------------------ */

/* CiA 301 이 예약한 COB-ID (PDO 로 쓰면 안 됨) */
static bool cob_restricted(uint32_t id) {
    return id <= 0x07F || (id >= 0x101 && id <= 0x180) || (id >= 0x581 && id <= 0x5FF) ||
           (id >= 0x601 && id <= 0x67F) || (id >= 0x6E0 && id <= 0x6FF) || id >= 0x701;
}

static uint32_t check_pdo_cob(uint32_t old, uint32_t raw) {
    if (raw & 0x20000000u) return CO_ABORT_VALUE; /* 29-bit ID 미지원 */
    if (raw & 0x1FFFF800u) return CO_ABORT_VALUE;
    bool old_valid = !(old & COB_INVALID);
    bool new_valid = !(raw & COB_INVALID);
    uint32_t id = raw & 0x7FF;
    if (new_valid && cob_restricted(id)) return CO_ABORT_VALUE;
    /* 켜져 있는 PDO 의 ID 를 바로 바꾸는 것은 금지 (먼저 bit31 로 끄고 바꿔야 함) */
    if (old_valid && new_valid && (old & 0x7FF) != id) return CO_ABORT_VALUE;
    return 0;
}

static uint32_t check_map_entry(uint32_t e, bool tpdo, unsigned *bits) {
    if (e == 0) return 0;
    const od_sub_t *s = od_lookup((uint16_t)(e >> 16), (uint8_t)(e >> 8));
    if (!s) return CO_ABORT_NO_MAP;
    if (!(s->flags & (tpdo ? OD_TPDO : OD_RPDO))) return CO_ABORT_NO_MAP;
    unsigned len = e & 0xFF;
    if (len != od_type_size(s->type) * 8) return CO_ABORT_NO_MAP;
    if (bits) *bits += len;
    return 0;
}

static uint32_t check_map_count(const co_pdo_map_t *m, uint32_t count, bool tpdo) {
    if (count > CO_PDO_MAX_MAP) return CO_ABORT_VALUE;
    unsigned bits = 0;
    for (uint32_t k = 0; k < count; k++) {
        if (m->map[k] == 0) return CO_ABORT_NO_MAP;
        uint32_t ab = check_map_entry(m->map[k], tpdo, &bits);
        if (ab) return ab;
    }
    return bits > 64 ? CO_ABORT_MAP_LEN : 0;
}

static bool valid_trans_type(uint32_t t) {
    return t <= 240 || t == 254 || t == 255;
}

static uint32_t comm_check(co_node_t *n, uint16_t index, uint8_t sub, uint32_t raw) {
    co_od_data_t *od = &n->od;
    if (index == 0x1005) {
        /* SYNC 소비자 전용: 생성 비트(30), 29-bit 금지 */
        return (raw & 0x7FFFF800u) ? CO_ABORT_VALUE : 0;
    }
    if (index >= 0x1400 && index < 0x1400 + CO_PDO_COUNT) {
        int i = index - 0x1400;
        if (sub == 1) return check_pdo_cob(od->rpdo_comm[i].cob_id, raw);
        if (sub == 2) return valid_trans_type(raw) ? 0 : CO_ABORT_VALUE;
    }
    if (index >= 0x1800 && index < 0x1800 + CO_PDO_COUNT) {
        int i = index - 0x1800;
        if (sub == 1) return check_pdo_cob(od->tpdo_comm[i].cob_id, raw);
        if (sub == 2) return valid_trans_type(raw) ? 0 : CO_ABORT_VALUE;
    }
    bool rmap = index >= 0x1600 && index < 0x1600 + CO_PDO_COUNT;
    bool tmap = index >= 0x1A00 && index < 0x1A00 + CO_PDO_COUNT;
    if (rmap || tmap) {
        const co_pdo_map_t *m = rmap ? &od->rpdo_map[index - 0x1600] : &od->tpdo_map[index - 0x1A00];
        if (sub == 0) return check_map_count(m, raw, tmap);
        /* 매핑 항목은 sub0 = 0 (매핑 해제) 상태에서만 바꿀 수 있다 (CiA 301 절차) */
        if (m->count != 0) return CO_ABORT_STATE;
        return check_map_entry(raw, tmap, NULL);
    }
    return 0;
}

static void comm_written(co_node_t *n, uint16_t index) {
    if (index >= 0x1400 && index < 0x1400 + CO_PDO_COUNT) rebuild_rpdo(n, index - 0x1400);
    if (index >= 0x1600 && index < 0x1600 + CO_PDO_COUNT) rebuild_rpdo(n, index - 0x1600);
    if (index >= 0x1800 && index < 0x1800 + CO_PDO_COUNT) rebuild_tpdo(n, index - 0x1800);
    if (index >= 0x1A00 && index < 0x1A00 + CO_PDO_COUNT) rebuild_tpdo(n, index - 0x1A00);
    if (index == 0x1016) n->hbc_active = false;
    if (index == 0x1017) n->hb_next_us = 0;
}

uint32_t co_od_write(co_node_t *n, uint16_t index, uint8_t sub, const uint8_t *data, size_t len, bool check_access) {
    const od_obj_t *o = od_find(index);
    if (!o) return CO_ABORT_NO_OBJECT;
    const od_sub_t *s = od_find_sub(o, sub);
    if (!s) return CO_ABORT_NO_SUB;
    if ((check_access && !(s->flags & OD_W)) || (s->flags & OD_CONST) || s->type == CO_VSTR) return CO_ABORT_READONLY;
    size_t sz = od_type_size(s->type);
    if (len != sz) return len > sz ? CO_ABORT_LEN_HIGH : CO_ABORT_LEN_LOW;

    uint32_t raw = 0;
    memcpy(&raw, data, sz);
    uint32_t ab = comm_check(n, index, sub, raw);
    if (!ab) ab = co_app_check(n, index, sub, raw);
    if (ab) return ab;

    od_set(&n->od, s, raw);
    comm_written(n, index);
    co_app_written(n, index, sub);
    return 0;
}

/* ------------------------------------------------------------------ */
/* NMT / boot-up / heartbeat                                           */
/* ------------------------------------------------------------------ */

static void set_nmt(co_node_t *n, co_nmt_t st) {
    co_nmt_t old = n->nmt;
    if (old == st) return;
    if (st == CO_NMT_OPERATIONAL) {
        for (int i = 0; i < CO_PDO_COUNT; i++) {
            reset_tpdo_rt(n, i);
            n->rpdo_buf[i].pending = false;
        }
    }
    if (st == CO_NMT_STOPPED) n->sdo.state = SDO_IDLE;
    n->nmt = st;
    co_app_nmt(n, old, st);
}

static void boot_up(co_node_t *n, uint64_t now) {
    uint8_t d = 0x00;
    n->nmt = CO_NMT_INIT;
    send(0x700 + n->node_id, &d, 1, true);
    n->hb_next_us = now + (uint64_t)n->od.hb_producer * 1000u;
    set_nmt(n, CO_NMT_PREOP);
}

void co_reset_comm(co_node_t *n, uint64_t now) {
    uint8_t err_reg = n->od.error_register; /* 에러 상태는 애플리케이션 소관이라 유지 */
    od_load_defaults(&n->od, n->node_id, 0x1000, 0x1FFF);
    n->od.error_register = err_reg;
    n->od.serial = n->serial;
    for (int i = 0; i < CO_PDO_COUNT; i++) {
        rebuild_rpdo(n, i);
        rebuild_tpdo(n, i);
    }
    memset(&n->sdo, 0, sizeof(n->sdo));
    n->hbc_active = false;
    boot_up(n, now);
}

void co_reset_node(co_node_t *n, uint64_t now) {
    od_load_defaults(&n->od, n->node_id, 0x2000, 0xFFFF);
    n->od.error_register = 0;
    co_app_reset(n);
    co_reset_comm(n, now);
}

void co_node_init(co_node_t *n, uint8_t node_id, uint8_t axis, uint32_t serial, uint64_t now) {
    memset(n, 0, sizeof(*n));
    n->node_id = node_id;
    n->axis = axis;
    n->serial = serial;
    n->nmt = CO_NMT_INIT;
    co_reset_node(n, now);
}

static void handle_nmt(co_node_t *n, const can_frame_t *f, uint64_t now) {
    if (f->dlc < 2) return;
    uint8_t target = f->data[1];
    if (target != 0 && target != n->node_id) return;
    switch (f->data[0]) {
    case 0x01: set_nmt(n, CO_NMT_OPERATIONAL); break;
    case 0x02: set_nmt(n, CO_NMT_STOPPED); break;
    case 0x80: set_nmt(n, CO_NMT_PREOP); break;
    case 0x81: co_reset_node(n, now); break;
    case 0x82: co_reset_comm(n, now); break;
    default: break;
    }
}

/* ------------------------------------------------------------------ */
/* EMCY                                                                */
/* ------------------------------------------------------------------ */

static void emcy_send(co_node_t *n, uint16_t code, uint8_t msef0) {
    if (n->nmt == CO_NMT_INIT || n->nmt == CO_NMT_STOPPED) return;
    uint32_t cob = n->od.cob_emcy;
    if (cob & COB_INVALID) return;
    uint8_t d[8] = {(uint8_t)code, (uint8_t)(code >> 8), n->od.error_register, msef0, 0, 0, 0, 0};
    send(cob, d, 8, true);
}

void co_emcy(co_node_t *n, uint16_t code, uint8_t reg_bits, uint8_t msef0) {
    n->od.error_register |= (uint8_t)(reg_bits | CO_ERR_GENERIC);
    emcy_send(n, code, msef0);
}

void co_emcy_clear(co_node_t *n) {
    n->od.error_register = 0;
    emcy_send(n, 0x0000, 0);
}

/* ------------------------------------------------------------------ */
/* SDO 서버                                                             */
/* ------------------------------------------------------------------ */

#define ABORT(i_, s_, c_) do { sdo_abort(n, (i_), (s_), (c_)); return; } while (0)

static void sdo_reply(co_node_t *n, const uint8_t *d) {
    send(0x580 + n->node_id, d, 8, true);
}

static void sdo_abort(co_node_t *n, uint16_t index, uint8_t sub, uint32_t code) {
    uint8_t d[8] = {0x80, (uint8_t)index, (uint8_t)(index >> 8), sub,
                    (uint8_t)code, (uint8_t)(code >> 8), (uint8_t)(code >> 16), (uint8_t)(code >> 24)};
    sdo_reply(n, d);
    n->sdo.state = SDO_IDLE;
}

static void sdo_initiate_upload(co_node_t *n, uint16_t index, uint8_t sub) {
    const od_obj_t *o = od_find(index);
    if (!o) ABORT(index, sub, CO_ABORT_NO_OBJECT);
    const od_sub_t *s = od_find_sub(o, sub);
    if (!s) ABORT(index, sub, CO_ABORT_NO_SUB);
    if (!(s->flags & OD_R)) ABORT(index, sub, CO_ABORT_WRITEONLY);

    uint8_t r[8] = {0, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
    if (s->type == CO_VSTR) {
        const char *str = s->str ? s->str : "";
        uint32_t len = (uint32_t)strlen(str);
        if (len <= 4) {
            r[0] = (uint8_t)(0x43 | ((4 - len) << 2));
            memcpy(&r[4], str, len);
        } else {
            r[0] = 0x41; /* segmented, 크기 표시 */
            r[4] = (uint8_t)len, r[5] = (uint8_t)(len >> 8), r[6] = (uint8_t)(len >> 16), r[7] = (uint8_t)(len >> 24);
            n->sdo.state = SDO_UP;
            n->sdo.index = index;
            n->sdo.sub = sub;
            n->sdo.toggle = 0;
            n->sdo.src = str;
            n->sdo.size = len;
            n->sdo.pos = 0;
        }
    } else {
        uint32_t v = od_get(&n->od, s);
        uint32_t len = (uint32_t)od_type_size(s->type);
        r[0] = (uint8_t)(0x43 | ((4 - len) << 2)); /* expedited, 크기 표시 */
        memcpy(&r[4], &v, len);
    }
    sdo_reply(n, r);
}

static void sdo_upload_segment(co_node_t *n, const uint8_t *d) {
    if (n->sdo.state != SDO_UP) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_CMD);
    uint8_t t = (d[0] >> 4) & 1;
    if (t != n->sdo.toggle) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_TOGGLE);
    uint32_t rem = n->sdo.size - n->sdo.pos;
    uint8_t cnt = (uint8_t)(rem > 7 ? 7 : rem);
    bool last = (n->sdo.pos + cnt >= n->sdo.size);
    uint8_t r[8] = {0};
    r[0] = (uint8_t)((t << 4) | ((7 - cnt) << 1) | (last ? 1 : 0));
    memcpy(&r[1], n->sdo.src + n->sdo.pos, cnt);
    n->sdo.pos += cnt;
    n->sdo.toggle ^= 1;
    if (last) n->sdo.state = SDO_IDLE;
    sdo_reply(n, r);
}

static void sdo_initiate_download(co_node_t *n, const uint8_t *d, uint16_t index, uint8_t sub) {
    if (d[0] & 0x02) { /* expedited */
        size_t len;
        if (d[0] & 0x01) {
            len = 4 - ((d[0] >> 2) & 3);
        } else { /* 크기 미표시: 오브젝트 타입 크기 사용 */
            const od_sub_t *s = od_lookup(index, sub);
            len = s ? od_type_size(s->type) : 4;
            if (len == 0) len = 4;
        }
        uint32_t ab = co_od_write(n, index, sub, &d[4], len, true);
        if (ab) ABORT(index, sub, ab);
        uint8_t r[8] = {0x60, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
        { sdo_reply(n, r); return; }
    }

    /* segmented: 오브젝트 존재/쓰기 권한부터 확인 */
    const od_obj_t *o = od_find(index);
    if (!o) ABORT(index, sub, CO_ABORT_NO_OBJECT);
    const od_sub_t *s = od_find_sub(o, sub);
    if (!s) ABORT(index, sub, CO_ABORT_NO_SUB);
    if (!(s->flags & OD_W) || (s->flags & OD_CONST)) ABORT(index, sub, CO_ABORT_READONLY);
    uint32_t size = (d[0] & 0x01) ? le32(&d[4]) : 0;
    if (size > sizeof(n->sdo.buf)) ABORT(index, sub, CO_ABORT_LEN_HIGH);

    n->sdo.state = SDO_DOWN;
    n->sdo.index = index;
    n->sdo.sub = sub;
    n->sdo.toggle = 0;
    n->sdo.size = size;
    n->sdo.pos = 0;
    uint8_t r[8] = {0x60, (uint8_t)index, (uint8_t)(index >> 8), sub, 0, 0, 0, 0};
    sdo_reply(n, r);
}

static void sdo_download_segment(co_node_t *n, const uint8_t *d) {
    if (n->sdo.state != SDO_DOWN) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_CMD);
    uint8_t t = (d[0] >> 4) & 1;
    if (t != n->sdo.toggle) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_TOGGLE);
    uint8_t cnt = (uint8_t)(7 - ((d[0] >> 1) & 7));
    if (n->sdo.pos + cnt > sizeof(n->sdo.buf)) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_LEN_HIGH);
    memcpy(&n->sdo.buf[n->sdo.pos], &d[1], cnt);
    n->sdo.pos += cnt;

    if (d[0] & 0x01) { /* 마지막 세그먼트: 쓰기에 실패하면 응답 대신 abort */
        if (n->sdo.size && n->sdo.pos != n->sdo.size) ABORT(n->sdo.index, n->sdo.sub, CO_ABORT_TYPE_LEN);
        uint32_t ab = co_od_write(n, n->sdo.index, n->sdo.sub, n->sdo.buf, n->sdo.pos, true);
        if (ab) ABORT(n->sdo.index, n->sdo.sub, ab);
        n->sdo.state = SDO_IDLE;
    }
    uint8_t r[8] = {(uint8_t)(0x20 | (t << 4)), 0, 0, 0, 0, 0, 0, 0};
    n->sdo.toggle ^= 1;
    sdo_reply(n, r);
}

static void handle_sdo(co_node_t *n, const can_frame_t *f, uint64_t now) {
    if (f->dlc < 8) return;
    const uint8_t *d = f->data;
    uint16_t index = (uint16_t)(d[1] | (d[2] << 8));
    uint8_t sub = d[3];
    n->sdo.t_us = now;

    switch (d[0] >> 5) {
    case 0: sdo_download_segment(n, d); break;
    case 1:
        n->sdo.state = SDO_IDLE;
        sdo_initiate_download(n, d, index, sub);
        break;
    case 2:
        n->sdo.state = SDO_IDLE;
        sdo_initiate_upload(n, index, sub);
        break;
    case 3: sdo_upload_segment(n, d); break;
    case 4: n->sdo.state = SDO_IDLE; break; /* 클라이언트 abort */
    default: sdo_abort(n, index, sub, CO_ABORT_CMD); break; /* block 전송 등 */
    }
}

/* ------------------------------------------------------------------ */
/* PDO / SYNC                                                          */
/* ------------------------------------------------------------------ */

static void rpdo_apply(co_node_t *n, int i, const uint8_t *data) {
    const co_pdo_cache_t *c = &n->rpdo[i];
    size_t off = 0;
    for (uint8_t k = 0; k < c->n; k++) {
        size_t sz = od_type_size(c->ent[k]->type);
        co_od_write(n, c->index[k], c->ent[k]->sub, &data[off], sz, false);
        off += sz;
    }
}

static void handle_rpdo(co_node_t *n, int i, const can_frame_t *f) {
    if (f->dlc < n->rpdo[i].len) return; /* 길이 부족: 무시 */
    if (n->od.rpdo_comm[i].trans_type <= 240) {
        memcpy(n->rpdo_buf[i].data, f->data, 8);
        n->rpdo_buf[i].pending = true; /* 다음 SYNC 에서 적용 */
    } else {
        rpdo_apply(n, i, f->data);
    }
}

static uint8_t tpdo_pack(const co_node_t *n, int i, uint8_t *out) {
    const co_pdo_cache_t *c = &n->tpdo[i];
    uint8_t off = 0;
    for (uint8_t k = 0; k < c->n; k++) {
        uint32_t v = od_get(&n->od, c->ent[k]);
        uint8_t sz = (uint8_t)od_type_size(c->ent[k]->type);
        memcpy(&out[off], &v, sz);
        off += sz;
    }
    return off;
}

static void tpdo_send(co_node_t *n, int i, const uint8_t *data, uint8_t len, uint64_t now) {
    send(n->od.tpdo_comm[i].cob_id, data, len, false);
    memcpy(n->tpdo_rt[i].last, data, len);
    n->tpdo_rt[i].sent = true;
    n->tpdo_rt[i].last_us = now;
    uint16_t evt = n->od.tpdo_comm[i].event_timer;
    n->tpdo_rt[i].next_evt_us = evt ? now + (uint64_t)evt * 1000u : 0;
}

static bool tpdo_enabled(const co_node_t *n, int i) {
    return !(n->od.tpdo_comm[i].cob_id & COB_INVALID) && n->tpdo[i].n > 0;
}

static void handle_sync(co_node_t *n, uint64_t now) {
    /* 1) SYNC 시점의 실제값으로 동기 TPDO 송신 */
    for (int i = 0; i < CO_PDO_COUNT; i++) {
        if (!tpdo_enabled(n, i)) continue;
        uint8_t tt = n->od.tpdo_comm[i].trans_type;
        if (tt > 240) continue;
        uint8_t buf[8];
        uint8_t len = tpdo_pack(n, i, buf);
        if (tt == 0) { /* 비주기 동기: 값이 바뀌었을 때만 */
            if (!n->tpdo_rt[i].sent || memcmp(buf, n->tpdo_rt[i].last, len)) tpdo_send(n, i, buf, len, now);
        } else if (++n->tpdo_rt[i].sync_cnt >= tt) {
            n->tpdo_rt[i].sync_cnt = 0;
            tpdo_send(n, i, buf, len, now);
        }
    }
    /* 2) SYNC 전에 받아 둔 동기 RPDO 적용 */
    for (int i = 0; i < CO_PDO_COUNT; i++) {
        if (n->rpdo_buf[i].pending) {
            n->rpdo_buf[i].pending = false;
            rpdo_apply(n, i, n->rpdo_buf[i].data);
        }
    }
}

/* ------------------------------------------------------------------ */
/* 수신 디스패치 / 주기 처리                                            */
/* ------------------------------------------------------------------ */

void co_rx(co_node_t *n, const can_frame_t *f, uint64_t now) {
    if (f->ext) return; /* CANopen 은 11-bit ID */
    uint32_t id = f->id;

    if (id == 0x000) { handle_nmt(n, f, now); return; }
    if (n->nmt == CO_NMT_INIT) return;

    if (id == (n->od.cob_sync & 0x7FF)) {
        if (n->nmt == CO_NMT_OPERATIONAL) handle_sync(n, now);
        return;
    }
    if (id == 0x600u + n->node_id) {
        if (n->nmt != CO_NMT_STOPPED) handle_sdo(n, f, now);
        return;
    }
    uint8_t hb_node = (uint8_t)((n->od.hb_consumer >> 16) & 0x7F);
    if (hb_node && id == 0x700u + hb_node && f->dlc >= 1) {
        n->hbc_last_us = now;
        n->hbc_active = true;
        return;
    }
    if (n->nmt != CO_NMT_OPERATIONAL) return;
    for (int i = 0; i < CO_PDO_COUNT; i++) {
        uint32_t cob = n->od.rpdo_comm[i].cob_id;
        if (!(cob & COB_INVALID) && (cob & 0x7FF) == id && n->rpdo[i].n > 0) handle_rpdo(n, i, f);
    }
}

void co_process(co_node_t *n, uint64_t now) {
    if (n->nmt == CO_NMT_INIT) return;

    /* heartbeat 생산자 */
    uint16_t hb = n->od.hb_producer;
    if (hb && (n->hb_next_us == 0 || now >= n->hb_next_us)) {
        uint8_t st = (uint8_t)n->nmt;
        send(0x700 + n->node_id, &st, 1, false);
        n->hb_next_us = now + (uint64_t)hb * 1000u;
    }

    /* heartbeat 소비자: 첫 heartbeat 를 받은 뒤부터 감시 */
    uint32_t hbc = n->od.hb_consumer;
    uint32_t hbc_ms = hbc & 0xFFFF;
    if (((hbc >> 16) & 0x7F) && hbc_ms && n->hbc_active && now - n->hbc_last_us > (uint64_t)hbc_ms * 1000u) {
        n->hbc_active = false;
        co_app_comm_lost(n, 0x8130);
    }

    /* SDO 타임아웃 */
    if (n->sdo.state != SDO_IDLE && now - n->sdo.t_us > SDO_TIMEOUT_US) {
        sdo_abort(n, n->sdo.index, n->sdo.sub, CO_ABORT_TIMEOUT);
    }

    /* 이벤트 방식 TPDO (254/255) */
    if (n->nmt != CO_NMT_OPERATIONAL) return;
    for (int i = 0; i < CO_PDO_COUNT; i++) {
        if (!tpdo_enabled(n, i)) continue;
        const co_pdo_comm_t *c = &n->od.tpdo_comm[i];
        if (c->trans_type < 254) continue;
        uint8_t buf[8];
        uint8_t len = tpdo_pack(n, i, buf);
        bool changed = !n->tpdo_rt[i].sent || memcmp(buf, n->tpdo_rt[i].last, len) != 0;
        bool timer = n->tpdo_rt[i].next_evt_us && now >= n->tpdo_rt[i].next_evt_us;
        bool inhibit_ok = !n->tpdo_rt[i].sent || now - n->tpdo_rt[i].last_us >= (uint64_t)c->inhibit * 100u;
        if ((changed || timer) && inhibit_ok) tpdo_send(n, i, buf, len, now);
    }
}
