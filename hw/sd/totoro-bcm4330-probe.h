/*
 * BCM4330 SDIO enumeration/backplane foundation, GPL-2.0-or-later.
 * Opt-in only. No firmware CPU, association/auth or network backend.
 * TOTORO_WIFI_TEST_SSID enables one synthetic open AP scan result.
 * TOTORO_WIFI_CDC_PROBE=1 enables a bounded diagnostic SDPCM/CDC responder;
 * unknown commands return BCME_UNSUPPORTED, never a fake successful link.
 */
typedef struct TotoroSDIO {
    SDState parent;
    uint8_t fn0[0x20000];
    uint8_t fn1[0x20000];
    /* Synthetic, bounded backplane; not a CPU or an operational WLAN.
     * Register constants follow the stock GPL dhd headers. RAM geometry and
     * EROM topology are a driver-facing test configuration, not a chip dump. */
    uint8_t ram[288 * 1024];
    uint8_t cores[4][4096];
    uint8_t wrappers[4][4096];
    uint8_t erom[4096];
    NICConf conf;
    NICState *nic;
    bool network_enabled, associated;
    uint32_t wsec, wpa_auth, auth, restrict_unencrypted;
    uint8_t bssid[6];
    uint8_t rx_queue[32][2048];
    uint16_t rx_size[32];
    unsigned queue_head, queue_count;
    uint8_t tx_credit;
    unsigned pending_events;
    uint64_t net_tx_packets, net_rx_packets, net_drops;
    qemu_irq hostwake;
    bool cdc_enabled, arm_started, irq_level, oob;
    uint32_t ram_high;
    uint8_t tx[8192], rx[8192], event_mask[16];
    unsigned tx_len, rx_len, rx_pos, cdc_trace_count;
    uint8_t rx_seq;
    uint32_t pm, radio, scan_status, passive_scan, scan_channel_time, scan_unassoc_time;
    uint32_t scalar_config[16];
    uint8_t mac[6], multicast[96];
    unsigned multicast_len;
    uint8_t scan_ssid[32];
    unsigned scan_ssid_len;
    unsigned channel;
    int rssi;
    bool scan_started, scan_aborted;
    bool station_up;
    unsigned trace_count;
    uint32_t address;
    uint32_t remaining;
    uint8_t function;
    bool increment;
    bool writing;
} TotoroSDIO;


enum {
    TOTORO_WIFI_CC = 0x18000000,
    TOTORO_WIFI_ARM = 0x18002000,
    TOTORO_WIFI_RAM = 0x18004000,
    TOTORO_WIFI_SDIO = 0x18011000,
    TOTORO_WIFI_EROM = 0x18010000,
    TOTORO_WIFI_WRAP = 0x18100000,
};
static const uint32_t totoro_wifi_core_base[] = {
    TOTORO_WIFI_CC, TOTORO_WIFI_ARM, TOTORO_WIFI_RAM, TOTORO_WIFI_SDIO,
};
static void totoro_wifi_put32(uint8_t *p, uint32_t v)
{
    for (unsigned i = 0; i < 4; i++) {
        p[i] = v >> (8 * i);
    }
}
static uint32_t totoro_wifi_get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void totoro_wifi_trace(TotoroSDIO *s, const char *op, uint32_t a, uint32_t v)
{
    if (g_getenv("TOTORO_WIFI_TRACE") && s->trace_count++ < 512) {
        qemu_log_mask(LOG_UNIMP, "totoro-wifi %s addr=0x%08x value=0x%08x\n",
                      op, a, v);
    }
}

static void totoro_wifi_irq_update(TotoroSDIO *s)
{
    uint32_t status = totoro_wifi_get32(s->cores[3] + 0x20);
    uint32_t mask = totoro_wifi_get32(s->cores[3] + 0x24);
    bool pending = s->cdc_enabled && s->arm_started &&
                   (s->fn0[3] & 4) && (status & mask);
    s->fn0[5] = pending ? 2 : 0;
    bool level = pending && s->oob;
    if (level != s->irq_level) {
        s->irq_level = level;
        qemu_set_irq(s->hostwake, level);
    }
}
static void totoro_wifi_pending(TotoroSDIO *s, uint32_t bits)
{
    totoro_wifi_put32(s->cores[3] + 0x20,
                     totoro_wifi_get32(s->cores[3] + 0x20) | bits);
    totoro_wifi_irq_update(s);
}
static void totoro_wifi_cdc_reset(TotoroSDIO *s)
{
    s->associated = false;
    s->wsec = s->wpa_auth = s->auth = s->restrict_unencrypted = 0;
    memset(s->bssid, 0, sizeof(s->bssid));
    s->queue_head = s->queue_count = s->pending_events = 0;
    s->tx_credit = 8;
    s->net_tx_packets = s->net_rx_packets = s->net_drops = 0;
    s->cdc_enabled = g_getenv("TOTORO_WIFI_CDC_PROBE") &&
                    !strcmp(g_getenv("TOTORO_WIFI_CDC_PROBE"), "1");
    s->arm_started = s->oob = s->irq_level = s->station_up = false;
    s->ram_high = s->tx_len = s->rx_len = s->rx_pos = 0;
    s->rx_seq = 0; s->cdc_trace_count = 0; s->pm = 0; s->radio = 0;
    memset(s->event_mask, 0, sizeof(s->event_mask));
    const uint8_t mac[] = {2, 0x43, 0x30, 0, 0, 1};
    memcpy(s->mac, mac, sizeof(mac));
    s->multicast_len = 0; memset(s->multicast, 0, sizeof(s->multicast));
    s->scan_started = s->scan_aborted = false;
    s->scan_status = s->passive_scan = 0;
    s->scan_channel_time = 40; s->scan_unassoc_time = 80;
    memset(s->scalar_config, 0, sizeof(s->scalar_config));
    s->scalar_config[10] = 1;
    s->channel = 6; s->rssi = -45;
    const char *channel = g_getenv("TOTORO_WIFI_CHANNEL");
    const char *rssi = g_getenv("TOTORO_WIFI_RSSI");
    char *end;
    if (channel && *channel) {
        long v = strtol(channel, &end, 10);
        if (!*end && v >= 1 && v <= 11) s->channel = v;
    }
    if (rssi && *rssi) {
        long v = strtol(rssi, &end, 10);
        if (!*end && v >= -100 && v <= -10) s->rssi = v;
    }
    const char *ssid = g_getenv("TOTORO_WIFI_TEST_SSID");
    s->scan_ssid_len = ssid ? strnlen(ssid, 33) : 0;
    if (s->scan_ssid_len > 32) s->scan_ssid_len = 0;
    memset(s->scan_ssid, 0, sizeof(s->scan_ssid));
    if (s->scan_ssid_len) memcpy(s->scan_ssid, ssid, s->scan_ssid_len);
    qemu_set_irq(s->hostwake, 0);
}


static void totoro_wifi_frame_header(TotoroSDIO *s, uint8_t *p,
                                    unsigned len, unsigned channel)
{
    p[0] = len; p[1] = len >> 8; p[2] = ~len; p[3] = (~len) >> 8;
    p[4] = s->rx_seq++; p[5] = channel; p[7] = 12;
    p[9] = s->tx_credit; p[10] = 4;
}
static bool totoro_wifi_enqueue(TotoroSDIO *s, const uint8_t *frame,
                                unsigned len, bool data)
{
    if (len > sizeof(s->rx_queue[0]) || len < 12 ||
        s->queue_count >= (data ? 24 : 32)) {
        s->net_drops++; return false;
    }
    if (!s->rx_len) {
        memcpy(s->rx, frame, len);
        s->rx_len = len; s->rx_pos = 0;
    } else {
        unsigned tail = (s->queue_head + s->queue_count) % 32;
        memcpy(s->rx_queue[tail], frame, len);
        s->rx_size[tail] = len; s->queue_count++;
    }
    totoro_wifi_pending(s, 0x40);
    return true;
}
static void totoro_wifi_promote(TotoroSDIO *s)
{
    if (!s->rx_len && s->queue_count) {
        unsigned head = s->queue_head;
        unsigned len = s->rx_size[head];
        memcpy(s->rx, s->rx_queue[head], len);
        s->rx_len = len; s->rx_pos = 0;
        s->queue_head = (head + 1) % 32; s->queue_count--;
    }
    if (s->rx_len) totoro_wifi_pending(s, 0x40);
}
static void totoro_wifi_put16be(uint8_t *p, uint16_t v)
{
    p[0] = v >> 8; p[1] = v;
}
static void totoro_wifi_put32be(uint8_t *p, uint32_t v)
{
    p[0]=v>>24; p[1]=v>>16; p[2]=v>>8; p[3]=v;
}
static void totoro_wifi_event(TotoroSDIO *s, unsigned type, bool link)
{
    if (!s->network_enabled || type >= 128 ||
        !(s->event_mask[type / 8] & (1 << (type % 8)))) return;
    uint8_t frame[88] = {0}; /* SDPCM12+BDC4+ether14+BRCM10+event48 */
    totoro_wifi_frame_header(s, frame, sizeof(frame), 1);
    frame[12] = 0x20; /* BDC v2, interface0, no checksum claim */
    uint8_t *eth = frame + 16, *hdr = eth + 14, *event = hdr + 10;
    memcpy(eth, s->mac, 6);
    const uint8_t ap[] = {2,0x43,0x30,0,0,2};
    memcpy(eth+6, ap, 6);
    totoro_wifi_put16be(eth+12, 0x886c);
    totoro_wifi_put16be(hdr, 0x8001);
    totoro_wifi_put16be(hdr+2, 56);
    hdr[5]=0; hdr[6]=0x10; hdr[7]=0x18; /* OUI00:10:18 */
    totoro_wifi_put16be(hdr+8, 1);
    totoro_wifi_put16be(event, 2);
    totoro_wifi_put16be(event+2, link ? 1 : 0);
    totoro_wifi_put32be(event+4, type);
    memcpy(event+24, ap, 6); memcpy(event+30,"eth0",4);
    totoro_wifi_enqueue(s, frame, sizeof(frame), false);
}
static void totoro_wifi_disconnect(TotoroSDIO *s)
{
    if (s->associated) s->pending_events |= 2; /* LINK down after CDC */
    s->associated = false; memset(s->bssid,0,6);
    s->queue_head = s->queue_count = 0; /* drop stale data/events */
}
static void totoro_wifi_cdc_reply(TotoroSDIO *s);
static void totoro_wifi_tx_frame(TotoroSDIO *s)
{
    unsigned n = s->tx_len;
    if (n < 12) return;
    unsigned len = s->tx[0] | (s->tx[1] << 8);
    unsigned check = s->tx[2] | (s->tx[3] << 8);
    unsigned off = s->tx[7];
    if ((len ^ check) != 0xffff || len > n || len < 12 ||
        off < 12 || off > len) { s->net_drops++; return; }
    s->tx_credit = s->tx[4] + 8;
    unsigned chan = s->tx[5] & 15;
    if (chan == 0) {
        totoro_wifi_cdc_reply(s);
        return;
    }
    if (chan != 2 || !s->network_enabled || !s->nic || !s->associated ||
        !s->station_up || len - off < 18) { s->net_drops++; return; }
    const uint8_t *bdc = s->tx + off;
    unsigned skip = 4 + 4 * bdc[3];
    if ((bdc[0] >> 4) != 2 || (bdc[2] & 15) ||
        (bdc[0] & 8) || skip > len - off ||
        len - off - skip < 14 || len - off - skip > 1518) {
        /* checksum-offload requests not implemented, never silently corrupt */
        s->net_drops++; return;
    }
    const uint8_t *eth = bdc + skip;
    unsigned size = len - off - skip;
    /* Every data TX emits updated credits even if the backend sends no reply. */
    uint8_t credit[12] = {0};
    totoro_wifi_frame_header(s, credit, sizeof(credit), 0);
    totoro_wifi_enqueue(s, credit, sizeof(credit), false);
    s->net_tx_packets++;
    qemu_send_packet(qemu_get_queue(s->nic), eth, size);
}
static unsigned totoro_wifi_scan_result(TotoroSDIO *s, uint8_t *out, bool iscan)
{
    /* ARM32 wlioctl.h v109 wire layout: fixed record 128 bytes, then SSID IE.
     * Synthetic OPEN AP is explicitly opt-in; no host radio, auth or link.
     * No pointer casts to packed host structs: the wire format is LE. */
    unsigned start = iscan ? 4 : 0;
    unsigned size = 12;
    bool present = s->scan_started && !s->scan_aborted && s->scan_ssid_len;
    if (iscan) totoro_wifi_put32(out, s->scan_aborted ? 3 : 0);
    uint8_t *results = out + start;
    if (present) {
        uint8_t *bss = results + 12;
        unsigned ie_len = 2 + s->scan_ssid_len;
        unsigned bss_len = 128 + ie_len;
        totoro_wifi_put32(bss, 109);
        totoro_wifi_put32(bss + 4, bss_len);
        const uint8_t bssid[] = {2, 0x43, 0x30, 0, 0, 2};
        memcpy(bss + 8, bssid, 6);
        bss[14] = 100; bss[16] = 1; /* beacon100, ESS not privacy */
        bss[18] = s->scan_ssid_len; memcpy(bss + 19, s->scan_ssid, s->scan_ssid_len);
        totoro_wifi_put32(bss + 52, 4);
        bss[56] = 0x82; bss[57] = 0x84; bss[58] = 0x8b; bss[59] = 0x96;
        bss[72] = s->channel; bss[73] = 0x2b; /* channel6,2GHz,no sideband,20MHz */
        bss[76] = 1; /* DTIM */
        bss[78] = (uint8_t)s->rssi; bss[79] = 0xff;
        bss[80] = (uint8_t)-95; bss[88] = s->channel;
        bss[116] = 128; totoro_wifi_put32(bss + 120, ie_len);
        bss[124] = 50;
        bss[128] = 0; bss[129] = s->scan_ssid_len;
        memcpy(bss + 130, s->scan_ssid, s->scan_ssid_len);
        size += bss_len;
    }
    totoro_wifi_put32(results, size);
    totoro_wifi_put32(results + 4, 109);
    totoro_wifi_put32(results + 8, present ? 1 : 0);
    return start + size;
}

static void totoro_wifi_cdc_reply(TotoroSDIO *s)
{
    /* Exactly one bounded control reply. No packet forwarding or RF success. */
    unsigned n = s->tx_len;
    if (n < 28 || s->queue_count >= 30) {
        return;
    }
    unsigned frame_len = s->tx[0] | (s->tx[1] << 8);
    unsigned check = s->tx[2] | (s->tx[3] << 8);
    unsigned off = s->tx[7];
    if ((frame_len ^ check) != 0xffff || frame_len > n ||
        frame_len < 28 || off < 12 || off > frame_len - 16 ||
        (s->tx[5] & 15) != 0) {
        return;
    }
    const uint8_t *cdc = s->tx + off;
    uint32_t cmd = totoro_wifi_get32(cdc);
    uint32_t want = totoro_wifi_get32(cdc + 4) & 0xffff;
    uint32_t flags = totoro_wifi_get32(cdc + 8);
    /* Vendor internal ioctls may omit CDCF_IOC_SET. WLC command numbers
     * encode these setters unambiguously; retain the original flags in reply. */
    bool set = (flags & 2) || cmd == 263 || cmd == 2 || cmd == 3 ||
               cmd == 10 || cmd == 20 || cmd == 30 || cmd == 38 ||
               cmd == 49 || cmd == 50 || cmd == 52 || cmd == 86 ||
               cmd == 118 || cmd == 185 || cmd == 187 || cmd == 26 ||
               cmd == 22 || cmd == 45 || cmd == 134;
    const uint8_t *input = cdc + 16;
    unsigned available = frame_len - off - 16;
    uint8_t output[512] = {0};
    unsigned out_len = 0;
    int32_t status = -23; /* BCME_UNSUPPORTED; explicit, not blanket success */

    if (cmd == 262 || cmd == 263) {
        const uint8_t *end = memchr(input, 0, available);
        if (!end) {
            status = -2; /* BADARG */
        } else {
            const char *name = (const char *)input;
            unsigned name_len = end - input + 1;
            const uint8_t *value = input + name_len;
            unsigned value_len = available - name_len;
            if (!set && cmd == 262 && !strcmp(name, "ver")) {
                const char version[] = "Totoro diagnostic CDC model; no RF/network";
                out_len = sizeof(version); memcpy(output, version, out_len);
                status = 0;
            } else if (!set && cmd == 262 && !strcmp(name, "cur_etheraddr")) {
                memcpy(output, s->mac, sizeof(s->mac)); out_len = sizeof(s->mac); status = 0;
            } else if (set && cmd == 263 && !strcmp(name, "cur_etheraddr")) {
                if (value_len >= 6 && !(value[0] & 1)) {
                    memcpy(s->mac, value, 6); status = 0;
                } else status = -2;
            } else if (s->network_enabled &&
                       (!strcmp(name,"wsec") || !strcmp(name,"wpa_auth") ||
                        !strcmp(name,"auth") || !strcmp(name,"wsec_restrict"))) {
                uint32_t *reg = !strcmp(name,"wsec") ? &s->wsec :
                    !strcmp(name,"wpa_auth") ? &s->wpa_auth :
                    !strcmp(name,"auth") ? &s->auth : &s->restrict_unencrypted;
                if (set && value_len >= 4) {
                    uint32_t v = totoro_wifi_get32(value);
                    if (v == 0 || (!strcmp(name,"wsec_restrict") && v <= 1)) {
                        *reg = v; status = 0;
                    } else status = -23; /* only genuine open-system supported */
                } else if (!set) {
                    totoro_wifi_put32(output,*reg); out_len=4; status=0;
                } else status=-2;
            } else if (s->network_enabled && set &&
                       (!strcmp(name,"tkip_countermeasures") ||
                        !strcmp(name,"rx_unencrypted_eapol"))) {
                status = value_len && value[0] == 0 ? 0 : -23;
            } else if (s->network_enabled && !strcmp(name,"wpaie") && set) {
                status = value_len == 0 ? 0 : -23;
            } else if (!strcmp(name, "iscan")) {
                if (set && cmd == 263 && value_len >= 8 &&
                    totoro_wifi_get32(value) == 1) {
                    unsigned action = value[4] | (value[5] << 8);
                    if (action >= 1 && action <= 3) {
                        s->scan_started = true; s->scan_aborted = action == 3; status = 0;
                    } else status = -2;
                } else status = -2;
            } else if (!set && cmd == 262 && !strcmp(name, "iscanresults")) {
                out_len = totoro_wifi_scan_result(s, output, true); status = 0;
            } else if (!set && (!strcmp(name, "nmode") || !strcmp(name, "mimo_bw_cap") ||
                                 !strcmp(name, "sgi_tx"))) {
                totoro_wifi_put32(output, 0); out_len = 4; status = 0;
            } else if (!strcmp(name, "mcast_list")) {
                if (set && value_len >= 4) {
                    uint32_t count = totoro_wifi_get32(value);
                    if (count <= 16 && value_len >= 4 + count * 6) {
                        s->multicast_len = count * 6;
                        memcpy(s->multicast, value + 4, s->multicast_len); status = 0;
                    } else status = -2;
                } else if (!set) {
                    totoro_wifi_put32(output, s->multicast_len / 6);
                    memcpy(output + 4, s->multicast, s->multicast_len);
                    out_len = 4 + s->multicast_len; status = 0;
                }
            } else if (!strcmp(name, "allmulti") || !strcmp(name, "roam_off") ||
                       !strcmp(name, "fullroamperiod") || !strcmp(name, "bcn_timeout") ||
                       !strcmp(name, "bus:txglomalign") || !strcmp(name, "bus:txglom") ||
                       !strcmp(name, "arpoe") || !strcmp(name, "arp_ol") ||
                       !strcmp(name, "assoc_retry_max")) {
                const char *names[] = {"allmulti", "roam_off", "fullroamperiod", "bcn_timeout",
                    "bus:txglomalign", "bus:txglom", "arpoe", "arp_ol", "assoc_retry_max"};
                unsigned idx = 0;
                while (idx < 9 && strcmp(name, names[idx])) idx++;
                if (idx < 9 && set && value_len >= 4) {
                    /* Config register only, not offload execution or RF action. */
                    s->scalar_config[idx] = totoro_wifi_get32(value); status = 0;
                } else if (idx < 9 && !set) {
                    totoro_wifi_put32(output, s->scalar_config[idx]); out_len = 4; status = 0;
                } else status = -2;
            } else if (!strcmp(name, "event_msgs")) {
                if (set && cmd == 263 && value_len >= sizeof(s->event_mask)) {
                    memcpy(s->event_mask, value, sizeof(s->event_mask)); status = 0;
                } else if (!set && cmd == 262) {
                    memcpy(output, s->event_mask, sizeof(s->event_mask));
                    out_len = sizeof(s->event_mask); status = 0;
                } else {
                    status = -2;
                }
            }
            /* Other settings/features remain unsupported until modelled. */
        }
    } else if (!set && cmd == 0) {
        totoro_wifi_put32(output, 0x14e46c77); out_len = 4; status = 0;
    } else if (!set && cmd == 1) {
        totoro_wifi_put32(output, 1); out_len = 4; status = 0;
    } else if (!set && cmd == 98) {
        const uint32_t rev[16] = {0x14e4, 0x4330, 0, 3, 0, 0, 0, 0,
                                 0, 0, 3, 0x4330, 0, 0, 0, 0};
        for (unsigned i = 0; i < 16; i++) totoro_wifi_put32(output + i * 4, rev[i]);
        out_len = sizeof(rev); status = 0;
    } else if (set && (cmd == 2 || cmd == 3)) {
        s->station_up = cmd == 2;
        if (!s->station_up) totoro_wifi_disconnect(s);
        status = 0;
    } else if (cmd == 217 && !set) {
        /* Diagnostic AP channel6 only; no host RF capability advertised. */
        totoro_wifi_put32(output, 1); totoro_wifi_put32(output + 4, s->channel);
        out_len = 8; status = 0;
    } else if (cmd == 114 && !set) {
        totoro_wifi_put32(output, 4);
        output[4]=0x82; output[5]=0x84; output[6]=0x8b; output[7]=0x96;
        out_len = 20; status = 0;
    } else if (cmd == 39 && !set) {
        totoro_wifi_put32(output, 2); out_len = 4; status = 0; /* legacy B PHY */
    } else if ((cmd == 19 || cmd == 117) && !set) {
        totoro_wifi_put32(output, cmd == 19 ? s->scalar_config[10] : 0);
        out_len = 4; status = 0;
    } else if (cmd == 20 && set && available >= 4) {
        uint32_t v = totoro_wifi_get32(input);
        if (v <= 1) { s->scalar_config[10] = v; status = 0; }
        else status = -2;
    } else if (cmd == 118 && set && available >= 4) {
        status = totoro_wifi_get32(input) == 0 ? 0 : -23; /* no AP mode */
    } else if (cmd == 29 && !set) {
        totoro_wifi_put32(output, s->channel); totoro_wifi_put32(output + 4, s->channel);
        totoro_wifi_put32(output + 8, s->channel); out_len = 12; status = 0;
    } else if (cmd == 52) {
        /* Already disconnected: no association exists to tear down. */
        totoro_wifi_disconnect(s); status = 0;
    } else if (cmd == 9 && !set) {
        totoro_wifi_put32(output, s->scalar_config[9]); out_len = 4; status = 0;
    } else if (cmd == 10 && set && available >= 4) {
        s->scalar_config[9] = totoro_wifi_get32(input); status = 0;
    } else if (cmd == 48 && !set) {
        totoro_wifi_put32(output, s->passive_scan); out_len = 4; status = 0;
    } else if (cmd == 49 && set && available >= 4) {
        s->passive_scan = totoro_wifi_get32(input); status = 0;
    } else if (cmd == 50 && set && available >= 36) {
        s->scan_started = true; s->scan_aborted = false; status = 0;
    } else if (cmd == 51 && !set) {
        out_len = totoro_wifi_scan_result(s, output, false); status = 0;
    } else if ((cmd == 184 || cmd == 186) && !set) {
        totoro_wifi_put32(output, cmd == 184 ? s->scan_channel_time : s->scan_unassoc_time);
        out_len = 4; status = 0;
    } else if ((cmd == 185 || cmd == 187) && set && available >= 4) {
        if (cmd == 185) s->scan_channel_time = totoro_wifi_get32(input);
        else s->scan_unassoc_time = totoro_wifi_get32(input);
        status = 0;
    } else if (cmd == 26 && set && s->network_enabled && available >= 36) {
        unsigned count = totoro_wifi_get32(input);
        if (!count) { totoro_wifi_disconnect(s); status=0; }
        else if (count > 32 || count != s->scan_ssid_len ||
                 memcmp(input+4,s->scan_ssid,count)) status=-30; /* SSID not found */
        else if (!s->station_up || s->radio || !s->nic) status=-4;
        else if (s->wsec || s->wpa_auth || s->auth) status=-23;
        else {
            const uint8_t ap[] = {2,0x43,0x30,0,0,2};
            memcpy(s->bssid,ap,6); s->associated=true;
            s->pending_events |= 3; status=0;
        }
    } else if (cmd == 23 && !set) {
        if (s->associated) { memcpy(output,s->bssid,6);out_len=6;status=0; }
        else status = -17;
    } else if (cmd == 25 && !set) {
        if (s->associated) {
            totoro_wifi_put32(output,s->scan_ssid_len);
            memcpy(output+4,s->scan_ssid,s->scan_ssid_len);
        }
        out_len = 36; status = 0;
    } else if (s->network_enabled && (cmd == 21 || cmd == 133) && !set) {
        totoro_wifi_put32(output,cmd==21?s->auth:s->wsec);out_len=4;status=0;
    } else if (s->network_enabled && (cmd == 22 || cmd == 134) &&
               set && available >= 4) {
        if (totoro_wifi_get32(input)==0) {
            if (cmd==22) s->auth=0; else s->wsec=0;status=0;
        } else status=-23;
    } else if (s->network_enabled && cmd == 45 && set && available >= 8) {
        status=totoro_wifi_get32(input+4)==0?0:-23; /* remove key only */
    } else if (s->network_enabled && (cmd == 12 || cmd == 127) && !set) {
        if (s->associated) {
            totoro_wifi_put32(output,cmd==12?22:(uint32_t)s->rssi);out_len=4;status=0;
        } else status=-17;
    } else if (cmd == 85 && !set) {
        totoro_wifi_put32(output, s->pm); out_len = 4; status = 0;
    } else if (cmd == 86 && set && available >= 4) {
        s->pm = totoro_wifi_get32(input); status = 0;
    } else if (cmd == 37 && !set) {
        totoro_wifi_put32(output, s->radio); out_len = 4; status = 0;
    } else if (cmd == 38 && set && available >= 4) {
        uint32_t radio = totoro_wifi_get32(input);
        uint32_t mask = radio >> 16;
        s->radio = (s->radio & ~mask) | (radio & mask); status = 0;
    }
    if (out_len > want) {
        status = -14; out_len = 0; /* BCME_BUFTOOSHORT */
    }
    if (status) out_len = 0;
    uint8_t reply[540] = {0};
    unsigned len = 28 + out_len;
    totoro_wifi_frame_header(s, reply, len, 0);
    totoro_wifi_put32(reply + 12, cmd);
    totoro_wifi_put32(reply + 16, out_len);
    totoro_wifi_put32(reply + 20, (flags & ~1u) | (status ? 1 : 0));
    totoro_wifi_put32(reply + 24, (uint32_t)status);
    memcpy(reply + 28, output, out_len);
    totoro_wifi_enqueue(s, reply, len, false);
    if (g_getenv("TOTORO_WIFI_TRACE") && s->cdc_trace_count++ < 256) {
        qemu_log_mask(LOG_UNIMP, "totoro-wifi CDC cmd=%u set=%u status=%d output=%u\n",
                      cmd, set, status, out_len);
    }
    if (s->pending_events & 1) totoro_wifi_event(s, 0, true);
    if (s->pending_events & 2) totoro_wifi_event(s, 16, s->associated);
    s->pending_events = 0;
    totoro_wifi_pending(s, 0x40);
}

static void totoro_wifi_backplane_reset(TotoroSDIO *s)
{
    const unsigned cid[] = {0x800, 0x82a, 0x80e, 0x829};
    const unsigned rev[] = {36, 3, 10, 7};
    memset(s->ram, 0, sizeof(s->ram));
    memset(s->cores, 0, sizeof(s->cores));
    memset(s->wrappers, 0, sizeof(s->wrappers));
    memset(s->erom, 0, sizeof(s->erom));
    s->trace_count = 0;
    totoro_wifi_cdc_reset(s);
    for (unsigned i = 0; i < 4; i++) {
        unsigned n = i * 16;
        /* Broadcom CIA/CIB, one slave port and one slave wrapper. */
        totoro_wifi_put32(s->erom + n, (0x4bf << 20) | (cid[i] << 8) | 1);
        totoro_wifi_put32(s->erom + n + 4,
                          (rev[i] << 24) | (1 << 19) | (1 << 9) | 1);
        totoro_wifi_put32(s->erom + n + 8, totoro_wifi_core_base[i] | 5);
        totoro_wifi_put32(s->erom + n + 12,
                          (TOTORO_WIFI_WRAP + i * 4096) | 0x85);
        totoro_wifi_put32(s->wrappers[i] + 0x408, 1); /* CLOCK_EN */
    }
    totoro_wifi_put32(s->erom + 64, 0xf); /* END | VALID */
    totoro_wifi_put32(s->cores[0], 0x14034330); /* AI,4 cores,rev3,4330 */
    totoro_wifi_put32(s->cores[0] + 4, 0x10000000); /* PMU */
    totoro_wifi_put32(s->cores[0] + 0xfc, TOTORO_WIFI_EROM);
    totoro_wifi_put32(s->cores[0] + 0x604, 12); /* test PMU revision */
    totoro_wifi_put32(s->cores[0] + 0x608, 7); /* ILP/ALP/HT available */
    totoro_wifi_put32(s->cores[2], 5 << 4); /* five RAM banks */
}
static uint8_t totoro_wifi_backplane_read(TotoroSDIO *s, uint32_t a)
{
    if (a < sizeof(s->ram)) {
        return s->ram[a];
    }
    if (a >= TOTORO_WIFI_EROM && a - TOTORO_WIFI_EROM < sizeof(s->erom)) {
        return s->erom[a - TOTORO_WIFI_EROM];
    }
    for (unsigned i = 0; i < 4; i++) {
        if (a >= totoro_wifi_core_base[i] &&
            a - totoro_wifi_core_base[i] < 4096) {
            unsigned off = a - totoro_wifi_core_base[i];
            if (i == 2 && off >= 0x40 && off < 0x44) {
                uint32_t idx = totoro_wifi_get32(s->cores[2] + 0x10);
                /* 4 * 64KiB + 32KiB; no ROM/devram banks advertised. */
                uint32_t info = idx < 4 ? 7 : idx == 4 ? 3 : 0;
                return info >> ((off - 0x40) * 8);
            }
            return s->cores[i][off];
        }
        uint32_t wrap = TOTORO_WIFI_WRAP + i * 4096;
        if (a >= wrap && a - wrap < 4096) {
            return s->wrappers[i][a - wrap];
        }
    }
    totoro_wifi_trace(s, "unmapped-read", a, 0xff);
    return 0xff;
}
static void totoro_wifi_backplane_write(TotoroSDIO *s, uint32_t a, uint8_t v)
{
    if (a < sizeof(s->ram)) {
        s->ram[a] = v;
        if (a + 1 > s->ram_high && a < sizeof(s->ram) - 4096) s->ram_high = a + 1;
        return;
    }
    if (a >= TOTORO_WIFI_EROM && a - TOTORO_WIFI_EROM < sizeof(s->erom)) {
        return; /* EROM is read-only. */
    }
    for (unsigned i = 0; i < 4; i++) {
        if (a >= totoro_wifi_core_base[i] &&
            a - totoro_wifi_core_base[i] < 4096) {
            unsigned off = a - totoro_wifi_core_base[i];
            if ((i == 0 && (off < 8 || (off >= 0xfc && off < 0x100) ||
                           (off >= 0x604 && off < 0x60c))) ||
                (i == 2 && (off < 4 || (off >= 0x40 && off < 0x44)))) {
                return;
            }
            if (i == 3 && off >= 0x20 && off < 0x24) {
                s->cores[i][off] &= ~v; /* W1C byte, not ordinary register RAM */
            } else {
                s->cores[i][off] = v;
            }
            if (i == 3 && off == 0x40) {
                if (v & 2) totoro_wifi_put32(s->cores[3] + 0x4c, 0);
                if (v & 4) s->oob = true;
                if (v & 1) { /* retransmit the buffered frame, if any */
                    s->rx_pos = 0;
                    if (s->rx_len) totoro_wifi_pending(s, 0x40);
                }
            }
            if (i == 3) totoro_wifi_irq_update(s);
            return;
        }
        uint32_t wrap = TOTORO_WIFI_WRAP + i * 4096;
        if (a >= wrap && a - wrap < 4096) {
            unsigned off = a - wrap;
            uint8_t old = s->wrappers[i][off];
            s->wrappers[i][off] = v;
            if (i == 1 && off == 0x800) {
                if (v & 1) {
                    s->arm_started = false; s->fn0[3] &= ~4;
                    s->rx_len = s->rx_pos = s->tx_len = 0;
                    s->queue_head = s->queue_count = 0;
                    s->associated = false;
                } else if (old & 1) {
                    uint32_t token = totoro_wifi_get32(s->ram + sizeof(s->ram) - 4);
                    unsigned words = token & 0xffff;
                    s->arm_started = s->cdc_enabled && s->ram_high >= 65536 &&
                        ((token >> 16) ^ words) == 0xffff && words && words < 1024;
                    /* This starts the host CDC model, NOT the downloaded CPU code. */
                }
                totoro_wifi_irq_update(s);
            }
            return;
        }
    }
    totoro_wifi_trace(s, "unmapped-write", a, v);
}
static uint32_t totoro_wifi_window(TotoroSDIO *s, uint32_t a)
{
    return ((uint32_t)(s->fn1[0x1000a] & 0x80) << 8) |
           ((uint32_t)s->fn1[0x1000b] << 16) |
           ((uint32_t)s->fn1[0x1000c] << 24) | (a & 0x7fff);
}
static void totoro_sdio_reset(DeviceState *dev)
{
    TotoroSDIO *s = (TotoroSDIO *)dev;
    memset(s->fn0, 0, sizeof(s->fn0));
    memset(s->fn1, 0, sizeof(s->fn1));
    s->remaining = 0;
    totoro_wifi_backplane_reset(s);
    s->fn1[0x1000c] = 0x18;
    s->fn0[0x00] = 0x22; /* CCCR2.0 / SDIO2.0 */
    s->fn0[0x01] = 0x02;
    s->fn0[0x08] = 0x02; /* multi-block */
    s->fn0[0x09] = 0x00; s->fn0[0x0a] = 0x10; /* common CIS @1000 */
    s->fn0[0x100] = 0; /* vendor-specific function1 */
    s->fn0[0x109] = 0x40; s->fn0[0x10a] = 0x10;
    s->fn0[0x200] = 0; /* vendor-specific function2 */
    s->fn0[0x209] = 0x80; s->fn0[0x20a] = 0x10;
    for (unsigned a = 0x1000; a <= 0x1080; a += 0x40) {
        const uint8_t manfid[] = {0x20, 4, 0xd0, 0x02, 0x30, 0x43, 0xff};
        memcpy(s->fn0 + a, manfid, sizeof(manfid));
    }
    /* CIS function extension tuples required by stock Linux sdio_cis.c.
     * Advertise transport block limits, never an operational network. */
    const uint8_t common_funce[] = {0x22, 4, 0, 0, 2, 0x32, 0xff};
    memcpy(s->fn0 + 0x1006, common_funce, sizeof(common_funce));
    for (unsigned a = 0x1046; a <= 0x1086; a += 0x40) {
        s->fn0[a] = 0x22;
        s->fn0[a + 1] = 42;
        s->fn0[a + 2] = 1; /* function tuple */
        s->fn0[a + 2 + 12] = 0;
        s->fn0[a + 2 + 13] = 2; /* max block512 */
        s->fn0[a + 2 + 28] = 10; /* timeout100ms */
        s->fn0[a + 2 + 42] = 0xff;
    }
    s->fn0[0x110] = 0; s->fn0[0x111] = 2; /* fn1 blocksize512 */
    s->fn0[0x210] = 0; s->fn0[0x211] = 2;
}

static uint8_t totoro_sdio_reg_read(TotoroSDIO *s, unsigned fn, uint32_t a)
{
    a &= 0x1ffff;
    if (fn == 0) {
        return s->fn0[a];
    }
    if (fn == 1) {
        if (a < 0x10000) {
            uint32_t bp = totoro_wifi_window(s, a);
            uint8_t v = totoro_wifi_backplane_read(s, bp);
            if (!(bp & 3)) {
                totoro_wifi_trace(s, "bp-read", bp, v);
            }
            return v;
        }
        if (a == 0x1000e) {
            uint8_t v = s->fn1[a] & 0x3f;
            if (v & (0x01 | 0x08 | 0x02 | 0x10)) {
                v |= 0x40;
            }
            if (v & (0x02 | 0x10)) {
                v |= 0x80;
            }
            return v;
        }
        if (a == 0x1001b || a == 0x1001c) {
            unsigned left = s->rx_len - s->rx_pos;
            return a == 0x1001b ? left : left >> 8;
        }
        return s->fn1[a];
    }
    return 0xff;
}

static void totoro_sdio_reg_write(TotoroSDIO *s, unsigned fn, uint32_t a, uint8_t v)
{
    a &= 0x1ffff;
    if (fn == 0) {
        if (a == 0x06 && (v & 8)) {
            totoro_sdio_reset(DEVICE(s));
        } else if (a == 0x02) {
            s->fn0[a] = v & 6;
            s->fn0[0x03] = v & (s->arm_started ? 6 : 2);
            if (!(v & 4)) {
                s->tx_len = s->rx_len = s->rx_pos = 0;
                totoro_wifi_put32(s->cores[3] + 0x20, 0);
            } else if (s->arm_started) {
                totoro_wifi_put32(s->cores[3] + 0x4c, (4 << 16) | 0xa);
                totoro_wifi_pending(s, 0x80);
            }
            totoro_wifi_irq_update(s);
        } else if (a == 0x04 || a == 0x07 || a == 0x13 ||
                   a == 0x110 || a == 0x111 || a == 0x210 || a == 0x211) {
            s->fn0[a] = v;
        }
    } else if (fn == 1) {
        if (a < 0x10000) {
            uint32_t bp = totoro_wifi_window(s, a);
            totoro_wifi_backplane_write(s, bp, v);
            if (!(bp & 3)) {
                totoro_wifi_trace(s, "bp-write", bp, v);
            }
        } else {
            if (a == 0x1000d && (v & 1)) s->oob = true;
            if (a == 0x1000d) totoro_wifi_irq_update(s);
            if (a == 0x1000f) {
                if (v & 1) {
                    s->rx_pos = s->rx_len = 0; totoro_wifi_promote(s);
                }
                if (v & 2) s->tx_len = 0;
                totoro_wifi_irq_update(s);
            }
            s->fn1[a] = a == 0x1000a ? v & 0x80 :
                         a == 0x1000e ? v & 0x3f : v;
            if (a == 0x1000e) {
                totoro_wifi_trace(s, "clock-write", a, v);
            }
        }
    }
}

static size_t totoro_sdio_command(SDState *sd, SDRequest *req,
                                  uint8_t *resp, size_t respsz)
{
    TotoroSDIO *s = (TotoroSDIO *)sd;
    uint32_t arg = req->arg, result = 0;
    unsigned fn = (arg >> 28) & 7;
    if (respsz < 4) {
        return 0;
    }
    switch (req->cmd) {
    case 0:
        totoro_sdio_reset(DEVICE(s));
        return 0;
    case 5:
        totoro_wifi_trace(s, "CMD5", arg, 0);
        result = 0xa0ff8000; /* ready,2 functions,2.7..3.6V; no memory present */
        break;
    case 3:
        result = 0x00010000; /* RCA1 */
        break;
    case 7:
        result = 0;
        break;
    case 52: {
        uint32_t address = (arg >> 9) & 0x1ffff;
        if (fn > 1) {
            result = 0x200; /* R5 function-number error: fn2 not implemented */
            break;
        }
        if (arg & 0x80000000) {
            totoro_sdio_reg_write(s, fn, address, arg & 0xff);
        }
        result = (arg & 0x80000000) && !(arg & 0x08000000) ?
                 arg & 0xff : totoro_sdio_reg_read(s, fn, address);
        break;
    }
    case 53: {
        unsigned count = arg & 0x1ff;
        s->remaining = 0;
        if (fn > 2 || (fn == 2 && (!s->arm_started || !(s->fn0[3] & 4)))) {
            result = 0x200;
            break;
        }
        if (!count) {
            count = 512;
        }
        if (arg & 0x08000000) {
            unsigned block = s->fn0[fn * 0x100 + 0x10] |
                             (s->fn0[fn * 0x100 + 0x11] << 8);
            if (!block || block > 2048) {
                result = 0x100; /* R5 out-of-range */
                break;
            }
            count *= block;
        }
        s->function = fn;
        s->address = (arg >> 9) & 0x1ffff;
        s->increment = !!(arg & 0x04000000);
        s->writing = !!(arg & 0x80000000);
        if (fn == 2 && count > sizeof(s->tx)) {
            result = 0x100;
            break;
        }
        if (fn == 2 && (arg & 0x80000000)) s->tx_len = 0;
        s->remaining = count;
        break;
    }
    default:
        return 0;
    }
    stl_be_p(resp, result);
    return 4;
}

static size_t totoro_sdio_read_data(SDState *sd, void *buf, size_t len)
{
    TotoroSDIO *s = (TotoroSDIO *)sd;
    uint8_t *p = buf;
    for (size_t i = 0; i < len; i++) {
        p[i] = 0xff;
        if (s->remaining && !s->writing) {
            if (s->function == 2) {
                p[i] = s->rx_pos < s->rx_len ? s->rx[s->rx_pos++] : 0;
            } else {
                p[i] = totoro_sdio_reg_read(s, s->function, s->address);
            }
        }
        if (s->remaining && !s->writing) {
            s->remaining--;
            if (s->increment) {
                s->address = (s->address + 1) & 0x1ffff;
            }
        }
    }
    if (!s->remaining && s->function == 2) {
        if (s->rx_pos == s->rx_len) {
            s->rx_len = s->rx_pos = 0;
            totoro_wifi_promote(s);
            if (s->nic) qemu_flush_queued_packets(qemu_get_queue(s->nic));
        } else if (s->rx_len) {
            totoro_wifi_pending(s, 0x40);
        }
        totoro_wifi_irq_update(s);
    }
    return len;
}

static size_t totoro_sdio_write_data(SDState *sd, const void *buf, size_t len)
{
    TotoroSDIO *s = (TotoroSDIO *)sd;
    const uint8_t *p = buf;
    for (size_t i = 0; i < len; i++) {
        if (s->remaining && s->writing) {
            if (s->function == 2) {
                if (s->tx_len < sizeof(s->tx)) s->tx[s->tx_len++] = p[i];
            } else {
                totoro_sdio_reg_write(s, s->function, s->address, p[i]);
            }
            s->remaining--;
            if (s->increment) {
                s->address = (s->address + 1) & 0x1ffff;
            }
        }
    }
    if (!s->remaining && s->function == 2) {
        totoro_wifi_tx_frame(s);
        s->tx_len = 0;
    }
    return len;
}
static bool totoro_sdio_data_ready(SDState *sd)
{
    TotoroSDIO *s = (TotoroSDIO *)sd;
    return s->remaining && !s->writing;
}
static bool totoro_sdio_receive_ready(SDState *sd)
{
    TotoroSDIO *s = (TotoroSDIO *)sd;
    return s->remaining && s->writing;
}
static bool totoro_sdio_inserted(SDState *sd) { return true; }
static bool totoro_sdio_readonly(SDState *sd) { return false; }
static uint8_t totoro_sdio_dat(SDState *sd) { return 15; }
static bool totoro_sdio_cmdline(SDState *sd) { return true; }
static void totoro_sdio_voltage(SDState *sd, uint16_t mv) { }


static bool totoro_wifi_net_can_receive(NetClientState *nc)
{
    TotoroSDIO *s = qemu_get_nic_opaque(nc);
    return s->network_enabled && s->associated && s->station_up &&
           s->arm_started && (s->fn0[3]&4) && s->queue_count < 24;
}
static ssize_t totoro_wifi_net_receive(NetClientState *nc,
                                       const uint8_t *buf, size_t len)
{
    TotoroSDIO *s = qemu_get_nic_opaque(nc);
    if (!totoro_wifi_net_can_receive(nc)) return 0;
    if (len < 14 || len > 1518) { s->net_drops++; return len; }
    uint8_t frame[1534] = {0};
    totoro_wifi_frame_header(s,frame,len+16,2);
    frame[12]=0x20; memcpy(frame+16,buf,len);
    if (!totoro_wifi_enqueue(s,frame,len+16,true)) return 0;
    s->net_rx_packets++;return len;
}
static NetClientInfo totoro_wifi_net_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = totoro_wifi_net_can_receive,
    .receive = totoro_wifi_net_receive,
};
static const Property totoro_wifi_net_properties[] = {
    DEFINE_NIC_PROPERTIES(TotoroSDIO, conf),
};
static void totoro_wifi_unrealize(DeviceState *dev)
{
    TotoroSDIO *s = (TotoroSDIO *)dev;
    if (s->nic) { qemu_del_nic(s->nic); s->nic=NULL; }
}
static void totoro_sdio_realize(DeviceState *dev, Error **errp)
{
    TotoroSDIO *s = (TotoroSDIO *)dev;
    s->network_enabled = !g_strcmp0(g_getenv("TOTORO_WIFI_NETWORK"), "1") &&
                         s->conf.peers.ncs[0];
    if (s->network_enabled) {
        const uint8_t mac[] = {2,0x43,0x30,0,0,1};
        memcpy(s->conf.macaddr.a,mac,6);
        s->nic=qemu_new_nic(&totoro_wifi_net_info,&s->conf,
            object_get_typename(OBJECT(dev)),dev->id,&dev->mem_reentrancy_guard,s);
        qemu_format_nic_info_str(qemu_get_queue(s->nic),mac);
    }
    totoro_sdio_reset(dev);
    sdbus_set_inserted(SD_BUS(qdev_get_parent_bus(dev)), true);
}

static void totoro_sdio_instance_init(Object *obj)
{
    TotoroSDIO *s = (TotoroSDIO *)obj;
    qdev_init_gpio_out_named(DEVICE(obj), &s->hostwake, "hostwake", 1);
}
static void totoro_sdio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SDCardClass *sc = SDMMC_COMMON_CLASS(klass);
    dc->realize = totoro_sdio_realize;
    dc->unrealize = totoro_wifi_unrealize;
    device_class_set_props(dc,totoro_wifi_net_properties);
    dc->vmsd = NULL; /* inherited storage migration state is inappropriate */
    device_class_set_legacy_reset(dc, totoro_sdio_reset);
    dc->desc = "Totoro experimental BCM4330 SDIO probe (no Wi-Fi)";
    sc->do_command = totoro_sdio_command;
    sc->read_data = totoro_sdio_read_data;
    sc->write_data = totoro_sdio_write_data;
    sc->data_ready = totoro_sdio_data_ready;
    sc->receive_ready = totoro_sdio_receive_ready;
    sc->get_inserted = totoro_sdio_inserted;
    sc->get_readonly = totoro_sdio_readonly;
    sc->get_dat_lines = totoro_sdio_dat;
    sc->get_cmd_line = totoro_sdio_cmdline;
    sc->set_voltage = totoro_sdio_voltage;
}
static const TypeInfo totoro_sdio_type = {
    .name = "totoro-bcm4330-probe",
    .parent = TYPE_SDMMC_COMMON,
    .instance_size = sizeof(TotoroSDIO),
    .instance_init = totoro_sdio_instance_init,
    .class_init = totoro_sdio_class_init,
};
static void totoro_sdio_register(void) { type_register_static(&totoro_sdio_type); }
type_init(totoro_sdio_register)
