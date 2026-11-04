/*
 * Experimental PhoneEmulator CP transport. GPL-2.0-or-later.
 * No firmware CPU, SIM, registration, calls or SMS service is emulated here.
 * No CP_TASKMSGS_READY property is asserted. Explicit opt-in only.
 * All shared pointers are validated ARM32 offsets; no guest callbacks executed.
 */
#ifndef TOTORO_MODEM_TRANSPORT_H
#define TOTORO_MODEM_TRANSPORT_H
#include "totoro-capi2-codec.h"

#define TM_SIZE 0x200000u
#define TM_CTRL 91732u
#define TM_ENDPOINTS 16504u
#define TM_FIRST 16712u
#define TM_LAST 16716u
#define TM_CP_SEND 56u
#define TM_CP_FREE (56u + 4112u)
#define TM_AP_SEND (56u + 8224u)
#define TM_AP_FREE (56u + 8224u + 4112u)
#define TM_COUNT 16u
#define TM_DATA 512u
#define TM_STRIDE (60u + 4u + TM_DATA)
#define TM_ALLOC (108u + TM_COUNT * TM_STRIDE)
#define TM_CONFIGURED 0x12344321u

typedef struct TotoroModemTransport {
    bool active, faulted;
    uint32_t pool;
    bool busy[TM_COUNT];
    uint64_t requests, replies, rejected, returned, backpressure;
    TotoroCAPI2Radio radio;
} TotoroModemTransport;

static inline uint32_t tm_get(const uint8_t *m, uint32_t o)
{
    return (uint32_t)m[o] | ((uint32_t)m[o + 1] << 8) |
           ((uint32_t)m[o + 2] << 16) | ((uint32_t)m[o + 3] << 24);
}

static inline void tm_put(uint8_t *m, uint32_t o, uint32_t v)
{
    m[o] = v; m[o + 1] = v >> 8; m[o + 2] = v >> 16; m[o + 3] = v >> 24;
}

static inline bool tm_range(uint32_t o, uint64_t n, uint32_t low, uint32_t high)
{
    return o >= low && o <= high && n <= (uint64_t)high - o;
}

static inline uint32_t tm_handle(const TotoroModemTransport *s, unsigned i)
{
    return s->pool + 108 + i * TM_STRIDE;
}

static inline bool tm_fifo(const uint8_t *m, uint32_t o)
{
    return tm_get(m, o) < 1024 && tm_get(m, o + 4) < 1024;
}

static inline unsigned tm_space(const uint8_t *m, uint32_t o)
{
    return (tm_get(m, o) - tm_get(m, o + 4) - 1) & 1023;
}

static inline uint32_t tm_peek(const uint8_t *m, uint32_t o)
{
    return tm_get(m, o + 16 + tm_get(m, o) * 4);
}

static inline void tm_pop(uint8_t *m, uint32_t o)
{
    tm_put(m, o, (tm_get(m, o) + 1) & 1023);
}

static inline void tm_push(uint8_t *m, uint32_t o, uint32_t h)
{
    uint32_t w = tm_get(m, o + 4);
    tm_put(m, o + 16 + w * 4, h);
    tm_put(m, o + 4, (w + 1) & 1023);
    tm_put(m, o + 8, tm_get(m, o + 8) + 1);
    uint32_t count = (w + 1 - tm_get(m, o)) & 1023;
    if (count > tm_get(m, o + 12)) {
        tm_put(m, o + 12, count);
    }
}

/* Rebuild our own bounded free list; never follow guest-owned links. */
static inline void tm_free_list(uint8_t *m, const TotoroModemTransport *s)
{
    uint32_t head = s->pool + 4, prev = head, free_count = 0;
    tm_put(m, head, head); tm_put(m, head + 4, head);
    for (unsigned i = 0; i < TM_COUNT; i++) {
        uint32_t h = tm_handle(s, i);
        tm_put(m, h, h); tm_put(m, h + 4, h);
        if (!s->busy[i]) {
            tm_put(m, prev, h); tm_put(m, h + 4, prev); tm_put(m, h, head);
            prev = h; free_count++;
        }
    }
    tm_put(m, head + 4, prev);
    tm_put(m, s->pool + 68, free_count);
    /* Purely local CP allocator flow state; no remote callback is claimed. */
    tm_put(m, s->pool + 36, free_count ? 0 : 1); /* IPC_FLOW_START=0 */
}

static inline bool tm_initialise(uint8_t *m, TotoroModemTransport *s)
{
    uint32_t bottom = tm_get(m, 44), top = tm_get(m, 40);
    uint32_t first = tm_get(m, TM_FIRST), last = tm_get(m, TM_LAST);
    if (tm_get(m, 24) != 0 || tm_get(m, 91728) != TM_SIZE ||
        !tm_range(bottom, 0, TM_CTRL, TM_SIZE) ||
        top != TM_SIZE || bottom > top || TM_ALLOC > top - bottom ||
        tm_get(m, TM_ENDPOINTS + 16) != 2 ||
        tm_get(m, TM_ENDPOINTS + 16 + 4) == 0 ||
        tm_get(m, TM_ENDPOINTS + 16 + 12) != 4 ||
        !tm_fifo(m, TM_CP_SEND) || !tm_fifo(m, TM_CP_FREE) ||
        !tm_fifo(m, TM_AP_SEND) || !tm_fifo(m, TM_AP_FREE) ||
        (first == 0) != (last == 0) ||
        (last && (!tm_range(last, 108, TM_CTRL, bottom) ||
                  (last & 3) || tm_get(m, last + 44) != 0))) {
        return false;
    }
    uint32_t pool = (top - TM_ALLOC) & ~3u;
    if (pool < bottom) {
        return false;
    }
    /* All checks complete: publish pool before readiness flag. */
    memset(m + pool, 0, TM_ALLOC);
    s->pool = pool;
    memset(s->busy, 0, sizeof(s->busy));
    tm_put(m, pool, 1);                    /* IPC_CP_CPU */
    tm_put(m, pool + 28, 2);               /* CP CAPI2 source */
    tm_put(m, pool + 32, 1);               /* AP CAPI2 destination */
    tm_put(m, pool + 48, TM_DATA);
    tm_put(m, pool + 52, 4);
    tm_put(m, pool + 56, 1);
    tm_put(m, pool + 72, TM_COUNT);
    tm_put(m, pool + 76, TM_COUNT);
    for (unsigned i = 0; i < TM_COUNT; i++) {
        uint32_t h = tm_handle(s, i);
        tm_put(m, h + 8, h);
        tm_put(m, h + 12, pool);
        tm_put(m, h + 16, h + 60);
        tm_put(m, h + 20, h + 64);
        tm_put(m, h + 32, i);
    }
    tm_put(m, pool + 16, pool + 16);
    tm_put(m, pool + 20, pool + 16);
    tm_put(m, pool + 24, pool);
    tm_put(m, pool + 12, pool);
    tm_free_list(m, s);
    if (last) {
        tm_put(m, last + 44, pool);
    } else {
        tm_put(m, TM_FIRST, pool);
    }
    tm_put(m, TM_LAST, pool);
    tm_put(m, 40, pool);
    tm_put(m, 48, TM_COUNT);
    tm_put(m, TM_ENDPOINTS + 32, 1);
    /* CP delivery function is never executed on AP; nonzero marker only. */
    tm_put(m, TM_ENDPOINTS + 36, 1);
    tm_put(m, TM_ENDPOINTS + 44, 4);
    tm_put(m, 24, TM_CONFIGURED);
    s->active = true;
    s->radio = (TotoroCAPI2Radio){ .system_state = 2, .rxlev = 0, .rxqual = 0 };
    return true;
}

static inline bool tm_ap_request(const uint8_t *m, const TotoroModemTransport *s,
                                 uint32_t h, uint32_t *data)
{
    uint32_t bottom = tm_get(m, 44);
    if (!tm_range(bottom, 0, TM_CTRL, s->pool) ||
        !tm_range(h, 60, TM_CTRL, bottom) || (h & 3)) {
        return false;
    }
    uint32_t p = tm_get(m, h + 12);
    if (!tm_range(p, 108, TM_CTRL, bottom) || (p & 3)) {
        return false;
    }
    uint32_t maxdata = tm_get(m, p + 48), maxheader = tm_get(m, p + 52);
    uint32_t count = tm_get(m, p + 72), id = tm_get(m, h + 32);
    uint64_t stride = 60u + (uint64_t)maxdata + maxheader;
    if (tm_get(m, p) != 2 || tm_get(m, p + 28) != 1 ||
        tm_get(m, p + 32) != 2 || maxheader != 4 ||
        !count || count >= 1024 || id >= count ||
        !tm_range(p, 108 + stride * count, TM_CTRL, bottom) ||
        p + 108 + stride * id != h ||
        tm_get(m, h + 8) != h || tm_get(m, h + 16) != h + 60 ||
        tm_get(m, h + 20) != h + 64 || tm_get(m, h + 28) != 4 ||
        tm_get(m, h + 24) < 24 || tm_get(m, h + 24) > maxdata ||
        tm_get(m, h + 40) != 2) {
        return false;
    }
    *data = h + 64;
    return true;
}

static inline unsigned tm_available(const TotoroModemTransport *s)
{
    unsigned n = 0;
    for (unsigned i = 0; i < TM_COUNT; i++) {
        n += !s->busy[i];
    }
    return n;
}

static inline void tm_send(uint8_t *m, TotoroModemTransport *s,
                            const uint8_t *bytes, size_t len, const uint8_t *channel)
{
    unsigned i;
    for (i = 0; i < TM_COUNT && s->busy[i]; i++) {
        /* Capacity checked by caller. */
    }
    assert(i < TM_COUNT && len <= TM_DATA);
    uint32_t h = tm_handle(s, i);
    s->busy[i] = true;
    memcpy(m + h + 60, channel, 4);
    memcpy(m + h + 64, bytes, len);
    tm_put(m, h + 24, len);
    tm_put(m, h + 28, 4);
    tm_put(m, h + 40, 2);
    tm_push(m, TM_AP_SEND, h);
    tm_put(m, s->pool + 84, tm_get(m, s->pool + 84) + 1);
    tm_put(m, s->pool + 100, tm_get(m, s->pool + 100) + len + 4);
    tm_free_list(m, s);
    s->replies++;
}

/* Called by timer or A2C interrupt, max32 requests/frees per invocation.
 * Returns true only when AP should receive C2A IRQ.
 * Invalid descriptors never permit arbitrary guest memory writes.
 */
static inline bool totoro_modem_transport_step(uint8_t *m, size_t size,
                                               TotoroModemTransport *s)
{
    bool irq = false;
    if (size != TM_SIZE) {
        return false;
    }
    uint32_t ap = tm_get(m, 28);
    if (ap == 0 || ap == 0x12345678) {
        /* New guest generation: local bookkeeping reset, no shared writes. */
        memset(s, 0, sizeof(*s));
        return false;
    }
    if (ap != TM_CONFIGURED || s->faulted) {
        return false;
    }
    if (!s->active) {
        return tm_initialise(m, s);
    }
    if (!tm_range(s->pool, TM_ALLOC, TM_CTRL, TM_SIZE) ||
        tm_get(m, 24) != TM_CONFIGURED || tm_get(m, 40) != s->pool ||
        tm_get(m, s->pool) != 1 ||
        !tm_fifo(m, TM_CP_SEND) || !tm_fifo(m, TM_CP_FREE) ||
        !tm_fifo(m, TM_AP_SEND) || !tm_fifo(m, TM_AP_FREE)) {
        s->faulted = true;
        s->rejected++;
        return false;
    }
    for (unsigned k = 0; k < 32 && tm_get(m, TM_CP_FREE) != tm_get(m, TM_CP_FREE + 4); k++) {
        uint32_t h = tm_peek(m, TM_CP_FREE), start = s->pool + 108;
        if (h < start || h - start >= TM_COUNT * TM_STRIDE ||
            (h - start) % TM_STRIDE) {
            s->faulted = true; s->rejected++;
            return irq;
        }
        unsigned i = (h - start) / TM_STRIDE;
        if (!s->busy[i] || tm_get(m, h + 40) != 0 ||
            tm_get(m, h + 12) != s->pool || tm_get(m, h + 8) != h) {
            s->faulted = true; s->rejected++;
            return irq;
        }
        s->busy[i] = false;
        tm_pop(m, TM_CP_FREE);
        tm_put(m, h + 24, 0); tm_put(m, h + 28, 0);
        tm_free_list(m, s);
        s->returned++;
    }
    for (unsigned k = 0; k < 32 && tm_get(m, TM_CP_SEND) != tm_get(m, TM_CP_SEND + 4); k++) {
        uint32_t h = tm_peek(m, TM_CP_SEND), data;
        uint8_t ack[32], response[68];
        if (!tm_ap_request(m, s, h, &data)) {
            s->faulted = true; s->rejected++;
            return irq;
        }
        uint32_t len = tm_get(m, h + 24);
        /* Validate common RPC prefix before constructing ACK. */
        if (totoro_capi2_get32(m + data) != 0 ||
            totoro_capi2_get32(m + data + 4) > 255 ||
            totoro_capi2_get32(m + data + 12) > 255 ||
            totoro_capi2_get32(m + data + 16) > 65535 ||
            totoro_capi2_get32(m + data + 20) > 1) {
            s->faulted = true; s->rejected++;
            return irq;
        }
        size_t n = totoro_capi2_radio_reply(&s->radio, m + data, len,
                                           response, sizeof(response));
        if (!n) {
            n = totoro_sysrpc_battmgr_reply(m + data, len,
                                            response, sizeof(response));
        }
        if (!n) {
            n = totoro_sysrpc_adc_reply(m + data, len,
                                        response, sizeof(response));
        }
        if (!n) {
            n = totoro_sysrpc_pmu_batt_level_reply(m + data, len,
                                                   response, sizeof(response));
        }
        unsigned needed = n ? 2 : 1;
        if (tm_available(s) < needed || tm_space(m, TM_AP_SEND) < needed ||
            !tm_space(m, TM_AP_FREE)) {
            s->backpressure++;
            break; /* Do not consume input or invent ACK before capacity exists. */
        }
        memcpy(ack, m + data, 24);
        totoro_capi2_put32(ack, 2);
        totoro_capi2_put32(ack + 16, 0x3ffe);
        totoro_capi2_put32(ack + 20, 1);
        totoro_capi2_put32(ack + 24, 0);       /* ackUsrData */
        totoro_capi2_put32(ack + 28, n ? 0 : 1); /* ACK_FAILED for unsupported */
        tm_send(m, s, ack, sizeof(ack), m + data - 4);
        if (n) {
            tm_send(m, s, response, n, m + data - 4);
        }
        tm_put(m, h + 40, 0);
        tm_push(m, TM_AP_FREE, h);
        tm_pop(m, TM_CP_SEND);
        s->requests++;
        irq = true;
    }
    return irq;
}
#endif