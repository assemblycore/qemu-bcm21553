/*
 * PhoneEmulator minimal CAPI2 signal/power codec, GPL-2.0-or-later.
 * Standalone foundation; NOT connected to guest IPC or SIM registration.
 * Uses byte arrays: host ABI/endianness must never affect XDR encoding.
 */
#ifndef TOTORO_CAPI2_CODEC_H
#define TOTORO_CAPI2_CODEC_H

typedef struct TotoroCAPI2Radio {
    uint32_t system_state; /* Vendor OFF=0, ON=1, ON_NO_RF=2 */
    uint8_t rxlev;         /* GSM RXLEV 0..63, not Android icon bars */
    uint8_t rxqual;        /* 0..7 */
} TotoroCAPI2Radio;

static inline uint32_t totoro_capi2_get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) | p[3];
}

static inline void totoro_capi2_put32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}

/* Zero result means no response generated (unsupported/malformed request).
 * Caller must not acknowledge/drop such a request as successful.
 * Fixed getters only; calls, SMS, SIM and registration are NOT implemented. */
static inline size_t totoro_capi2_radio_reply(TotoroCAPI2Radio *radio,
                                             const uint8_t *request, size_t len,
                                             uint8_t *out, size_t capacity)
{
    uint32_t msg, words[2];
    size_t count;
    if (len != 60 || totoro_capi2_get32(request) != 0 ||
        totoro_capi2_get32(request + 4) > 255 ||
        totoro_capi2_get32(request + 12) > 255 ||
        totoro_capi2_get32(request + 20) != 1 ||
        totoro_capi2_get32(request + 24) != totoro_capi2_get32(request + 12) ||
        totoro_capi2_get32(request + 40) != totoro_capi2_get32(request + 12) ||
        totoro_capi2_get32(request + 36) != 0 ||
        !(totoro_capi2_get32(request + 52) <= 2 ||
          totoro_capi2_get32(request + 52) == 255) ||
        radio->system_state > 2 || radio->rxlev > 63 || radio->rxqual > 7) {
        return 0;
    }
    msg = totoro_capi2_get32(request + 16);
    if (totoro_capi2_get32(request + 28) != msg) {
        return 0;
    }
    switch (msg) {
    case 0x403c: /* GetSystemState */
        words[0] = radio->system_state; count = 1;
        break;
    case 0x4040: /* GetRxSignalInfo: two XDR UInt8 words */
        words[0] = radio->system_state == 1 ? radio->rxlev : 0;
        words[1] = radio->system_state == 1 ? radio->rxqual : 0;
        count = 2;
        break;
    default:
        return 0;
    }
    if (totoro_capi2_get32(request + 32) != msg + 1 ||
        capacity < 60 + 4 * count) {
        return 0;
    }
    /* Fully validated before any output mutation. Allow in-place decoding. */
    memmove(out, request, 60);
    totoro_capi2_put32(out, 1);              /* RPC_TYPE_RESPONSE */
    totoro_capi2_put32(out + 16, msg + 1);  /* outer message ID */
    totoro_capi2_put32(out + 28, msg + 1);  /* CAPI2 message ID */
    totoro_capi2_put32(out + 36, 0);        /* RESULT_OK */
    for (size_t i = 0; i < count; i++) {
        totoro_capi2_put32(out + 60 + 4 * i, words[i]);
    }
    return 60 + 4 * count;
}
/*
 * SYSRPC MSG_HAL_EM_BATTMGR_REGISTER_EVENT_REQ (0x4B38) -> RSP (0x4B39).
 * Wire layout confirmed from the vendor totoro kernel sources (all XDR
 * big-endian 4-byte words; Boolean = xdr_u_char, enums = xdr_int):
 *   fuse_rpc/rpc_CIB/arpc/src/rpc_msg.c      xdr_RPC_InternalMsg_t / RPC_Msg_t
 *   sysinterface/sysrpc/src/sys_gen_rpc.c     xdr_SYS_ReqRep_t, Req/Rsp structs
 *   sysinterface/sysrpc/src/sys_hal_rpc.c     Handle_CAPI2_SYS_HAL_EM_BATTMGR_RegisterEventCB
 * Request (40 bytes):
 *   [msgType=0 REQUEST, clientIndex, tid, clientID, msgId=0x4B38, ptr=1,
 *    respId=0x4B39, result=0, event (HAL_EM_BATTMGR_Events_en_t 0..8),
 *    validCbk (Boolean 0/1)]
 * Response (36 bytes), as the CP handler builds it (memset SYS_ReqRep_t,
 * result=RESULT_OK, val=SYS_HAL_EM_BATTMGR_RegisterEventCB() whose default
 * is BATTMGR_SUCCESS; Send_SYS_RspForRequest keeps tid/clientID):
 *   [msgType=1 RESPONSE, clientIndex, tid, clientID, msgId=0x4B39, ptr=1,
 *    respId=0, result=RESULT_OK(0), val=BATTMGR_SUCCESS(0)]
 * This is a typed answer for exactly one message, not a generic success:
 * anything that does not match the layout above is rejected (returns 0,
 * the transport then sends ACK_FAILED and the AP assert stays visible).
 */
#define TOTORO_SYSRPC_BATTMGR_REG_REQ   0x4b38u
#define TOTORO_SYSRPC_BATTMGR_REG_RSP   0x4b39u
#define TOTORO_SYSRPC_BATTMGR_EVENT_MAX 8u   /* BATTMGR_CHARGER_TIMEOUT_EVENT */

static inline size_t totoro_sysrpc_battmgr_reply(const uint8_t *request,
                                                 size_t len, uint8_t *out,
                                                 size_t capacity)
{
    if (!request || !out || len != 40 || capacity < 36 ||
        totoro_capi2_get32(request) != 0 ||                 /* REQUEST */
        totoro_capi2_get32(request + 4) > 255 ||            /* clientIndex */
        totoro_capi2_get32(request + 12) > 255 ||           /* clientID */
        totoro_capi2_get32(request + 16) != TOTORO_SYSRPC_BATTMGR_REG_REQ ||
        totoro_capi2_get32(request + 20) != 1 ||            /* payload ptr */
        totoro_capi2_get32(request + 24) != TOTORO_SYSRPC_BATTMGR_REG_RSP ||
        totoro_capi2_get32(request + 28) != 0 ||            /* result */
        totoro_capi2_get32(request + 32) > TOTORO_SYSRPC_BATTMGR_EVENT_MAX ||
        totoro_capi2_get32(request + 36) > 1) {             /* validCbk */
        return 0;
    }
    /* Fully validated before any output mutation. Allow in-place decoding. */
    memmove(out, request, 16);               /* type, clientIndex, tid, clientID */
    totoro_capi2_put32(out, 1);              /* RPC_TYPE_RESPONSE */
    totoro_capi2_put32(out + 16, TOTORO_SYSRPC_BATTMGR_REG_RSP);
    totoro_capi2_put32(out + 20, 1);         /* payload present */
    totoro_capi2_put32(out + 24, 0);         /* respId (memset by CP handler) */
    totoro_capi2_put32(out + 28, 0);         /* RESULT_OK */
    totoro_capi2_put32(out + 32, 0);         /* BATTMGR_SUCCESS */
    return 36;
}

/*
 * SYSRPC MSG_HAL_ADC_CTRL_REQ 0x4B32 -> MSG_HAL_ADC_CTRL_RSP 0x4B33.
 * Vendor sources: sysrpc/src/sys_gen_rpc.c xdr_CAPI2_SYSRPC_HAL_ADC_Ctrl_Req_t,
 * sys_msg.c xdr_HAL_ADC_ReadConfig_st_t, CP handler sys_hal_rpc.c:226/349,
 * AP caller common/drivers/char/broadcom/bcm_auxadc.c auxadc_access_through_cp().
 *
 * Request (72 bytes, 18 XDR BE words):
 *   0 type=0  1 clientIndex  2 tid  3 clientID  4 msgId=0x4B32  5 ptr=1
 *   6 respId=0x4B33  7 result=0  8 action  9 inReq ptr=1
 *   10 ch(0..4)  11 pAdcData ptr=1  12 *pAdcData(u16)  13 cb_res(uninit on AP)
 *   14 cntxt  15 mrd_cnt(u16)  16 trg(0..4)  17 dly(u16)
 * Response (64 bytes, 16 words): CP memsets the reply, runs HAL_ADC_Ctrl and
 * copies only pAdcData, so: hdr(type=1,msgId=0x4B33,ptr=1,respId=0,result=0),
 *   ch=0, ptr=1, value, cb_res=HAL_ADC_CB_PASS(0), cntxt=0, mrd=0, trg=0, dly=0.
 *
 * Only the one-shot read used by the AP (action GET_UNALIGN=0, mrd_cnt=0) is
 * emulated; multi-read / RF-aligned requests are rejected (ACK_FAILED stays
 * visible). Values are emulated 10-bit sensor readings taken from the
 * board-totoro.c tables, not a generic "success":
 *   ch0 battery temperature: TempAdcTable 25 C ~= 373 -> 370
 *   ch1 battery VF: 0x200 (>>2 = 0x80, battery present; 0xFF means removed)
 *   ch2 ear/headset ADC: 0x3FF (open line, no headset/button)
 *   ch3 battery voltage: bat_adc 0x32C = 3.95 V
 *   ch4 unused on totoro: 0
 */
/*
 * SYSRPC MSG_PMU_BATT_LEVEL_REQ 0x4B00 -> MSG_PMU_BATT_LEVEL_RSP 0x4B01.
 * Vendor: public/sys_gen_mids.i (ids), sys_gen_rpc.c:84 CAPI2_PMU_BattADCReq
 * (AP side, payload xdr_default_proc = no arguments), CP handler
 * sys_rpc.c:198 Handle_CAPI2_PMU_BattADCReq: memset reply, result=RESULT_OK,
 * Send_SYS_RspForRequest(MSG_PMU_BATT_LEVEL_RSP); RSP payload default_proc.
 * Request 32 bytes: type=0, ci, tid, cid, 0x4B00, ptr=1, respId=0x4B01, result=0.
 * Response 32 bytes: type=1, ci, tid, cid, 0x4B01, ptr=1, respId=0, RESULT_OK.
 */
#define TOTORO_SYSRPC_PMU_BATT_LEVEL_REQ 0x4b00u
#define TOTORO_SYSRPC_PMU_BATT_LEVEL_RSP 0x4b01u

static inline size_t totoro_sysrpc_pmu_batt_level_reply(const uint8_t *request,
                                                        size_t len, uint8_t *out,
                                                        size_t capacity)
{
    if (!request || !out || len != 32 || capacity < 32 ||
        totoro_capi2_get32(request) != 0 ||
        totoro_capi2_get32(request + 4) > 255 ||
        totoro_capi2_get32(request + 12) > 255 ||
        totoro_capi2_get32(request + 16) != TOTORO_SYSRPC_PMU_BATT_LEVEL_REQ ||
        totoro_capi2_get32(request + 20) != 1 ||
        totoro_capi2_get32(request + 24) != TOTORO_SYSRPC_PMU_BATT_LEVEL_RSP ||
        totoro_capi2_get32(request + 28) != 0) {
        return 0;
    }
    memmove(out, request, 16);
    totoro_capi2_put32(out, 1);
    totoro_capi2_put32(out + 16, TOTORO_SYSRPC_PMU_BATT_LEVEL_RSP);
    totoro_capi2_put32(out + 20, 1);
    totoro_capi2_put32(out + 24, 0);
    totoro_capi2_put32(out + 28, 0);
    return 32;
}

#define TOTORO_SYSRPC_ADC_CTRL_REQ  0x4b32u
#define TOTORO_SYSRPC_ADC_CTRL_RSP  0x4b33u
#define TOTORO_SYSRPC_ADC_CH_MAX    4u
#define TOTORO_SYSRPC_ADC_TRG_MAX   4u

static inline uint32_t totoro_sysrpc_adc_value(uint32_t ch)
{
    static const uint16_t values[TOTORO_SYSRPC_ADC_CH_MAX + 1] = {
        370, 0x200, 0x3ff, 0x32c, 0,
    };
    return ch <= TOTORO_SYSRPC_ADC_CH_MAX ? values[ch] : 0;
}

static inline size_t totoro_sysrpc_adc_reply(const uint8_t *request,
                                             size_t len, uint8_t *out,
                                             size_t capacity)
{
    if (!request || !out || len != 72 || capacity < 64 ||
        totoro_capi2_get32(request) != 0 ||                 /* REQUEST */
        totoro_capi2_get32(request + 4) > 255 ||            /* clientIndex */
        totoro_capi2_get32(request + 12) > 255 ||           /* clientID */
        totoro_capi2_get32(request + 16) != TOTORO_SYSRPC_ADC_CTRL_REQ ||
        totoro_capi2_get32(request + 20) != 1 ||            /* payload ptr */
        totoro_capi2_get32(request + 24) != TOTORO_SYSRPC_ADC_CTRL_RSP ||
        totoro_capi2_get32(request + 28) != 0 ||            /* result */
        totoro_capi2_get32(request + 32) != 0 ||            /* GET_UNALIGN */
        totoro_capi2_get32(request + 36) != 1 ||            /* inReq ptr */
        totoro_capi2_get32(request + 40) > TOTORO_SYSRPC_ADC_CH_MAX ||
        totoro_capi2_get32(request + 44) != 1 ||            /* pAdcData ptr */
        totoro_capi2_get32(request + 48) > 0xffff ||        /* u16 data */
        /* +52 cb_res is uninitialised on the AP side: not validated */
        totoro_capi2_get32(request + 60) != 0 ||            /* mrd_cnt */
        totoro_capi2_get32(request + 64) > TOTORO_SYSRPC_ADC_TRG_MAX ||
        totoro_capi2_get32(request + 68) > 0xffff) {        /* dly */
        return 0;
    }
    uint32_t value = totoro_sysrpc_adc_value(totoro_capi2_get32(request + 40));
    /* Fully validated and value derived before any output mutation. */
    memmove(out, request, 16);
    totoro_capi2_put32(out, 1);              /* RPC_TYPE_RESPONSE */
    totoro_capi2_put32(out + 16, TOTORO_SYSRPC_ADC_CTRL_RSP);
    totoro_capi2_put32(out + 20, 1);         /* payload present */
    totoro_capi2_put32(out + 24, 0);         /* respId */
    totoro_capi2_put32(out + 28, 0);         /* RESULT_OK */
    totoro_capi2_put32(out + 32, 0);         /* ch (memset by CP) */
    totoro_capi2_put32(out + 36, 1);         /* pAdcData present */
    totoro_capi2_put32(out + 40, value);     /* *pAdcData */
    totoro_capi2_put32(out + 44, 0);         /* HAL_ADC_CB_PASS */
    totoro_capi2_put32(out + 48, 0);         /* cntxt */
    totoro_capi2_put32(out + 52, 0);         /* mrd_cnt */
    totoro_capi2_put32(out + 56, 0);         /* trg */
    totoro_capi2_put32(out + 60, 0);         /* dly */
    return 64;
}

#endif