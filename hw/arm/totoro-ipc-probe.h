/*
 * BCM21553 CP transport observer, GPL-2.0-or-later.
 * ARM32 vendor ABI with FUSE_IPC_CRASH_SUPPORT.
 * No readiness advertisement, FIFO consumption or fabricated RIL responses.
 */
#define TOTORO_IPC_BASE 0x81400000
#define TOTORO_IPC_SIZE 0x200000
#define TOTORO_IPC_FIFO 56
#define TOTORO_IPC_FIFO_BYTES 4112

static bool totoro_ipc_enabled;
static unsigned totoro_ipc_observations;

static bool totoro_ipc_range(uint32_t off, uint32_t len)
{
    return !(off & 3) && off >= 56 + 4 * TOTORO_IPC_FIFO_BYTES &&
           off <= TOTORO_IPC_SIZE && len <= TOTORO_IPC_SIZE - off;
}

/* IPC uses little-endian 32-bit shared-memory offsets. RPC/XDR is big-endian.
 * xdr_u_char occupies a 32-bit word, not one byte. */
typedef struct TotoroRPCHeader {
    uint32_t type, client_index, tid, client_id, msg_id, present;
    uint32_t response_id;
} TotoroRPCHeader;

static bool totoro_ipc_data_range(uint32_t off, uint32_t len)
{
    return off >= 56 + 4 * TOTORO_IPC_FIFO_BYTES &&
           off <= TOTORO_IPC_SIZE && len <= TOTORO_IPC_SIZE - off;
}

static bool totoro_ipc_parse_rpc(const uint8_t *data, size_t len,
                                  TotoroRPCHeader *out)
{
    TotoroRPCHeader h = {0};
    if (len < 24) {
        return false;
    }
    h.type = ldl_be_p(data); h.client_index = ldl_be_p(data + 4);
    h.tid = ldl_be_p(data + 8); h.client_id = ldl_be_p(data + 12);
    h.msg_id = ldl_be_p(data + 16); h.present = ldl_be_p(data + 20);
    if (h.type > 3 || h.client_index > 255 || h.client_id > 255 ||
        h.msg_id > 65535 || h.present > 1) {
        return false;
    }
    if (h.present && h.msg_id >= 0x4000) {
        /* CAPI2_ReqRep: clientID, msgId, respId, result, ClientInfo_t:
         * clientId, clientRef, dialogId, simId, reserved (nine XDR words). */
        if (len < 60 || ldl_be_p(data + 24) != h.client_id ||
            ldl_be_p(data + 28) != h.msg_id ||
            ldl_be_p(data + 32) > 65535 ||
            ldl_be_p(data + 40) != h.client_id ||
            !(ldl_be_p(data + 52) <= 2 || ldl_be_p(data + 52) == 255)) {
            return false;
        }
        h.response_id = ldl_be_p(data + 32);
    }
    *out = h;
    return true;
}

static void totoro_ipc_observe(void)
{
    uint8_t ctrl[56], fifo[16], descriptor[60];
    uint32_t read_index, write_index, handle;
    if (!totoro_ipc_enabled) {
        return;
    }
    /* Bounded diagnostic work per IRQ, don't spin on an unimplemented CP. */
    if (++totoro_ipc_observations > 128) {
        return;
    }
    if (address_space_read(&address_space_memory, TOTORO_IPC_BASE,
                           MEMTXATTRS_UNSPECIFIED, ctrl, sizeof(ctrl)) != MEMTX_OK) {
        return;
    }
    uint32_t ap = ldl_le_p(ctrl + 28);
    uint32_t cp = ldl_le_p(ctrl + 24);
    qemu_log_mask(LOG_UNIMP, "totoro-ipc: A2C observation AP=%08x CP=%08x\n", ap, cp);
    if (ap != 0x12345678 && ap != 0x12344321) {
        return;
    }
    /* CP/index0 send FIFO. Handles are shared-memory offsets, never host pointers. */
    if (address_space_read(&address_space_memory, TOTORO_IPC_BASE + TOTORO_IPC_FIFO,
                           MEMTXATTRS_UNSPECIFIED, fifo, sizeof(fifo)) != MEMTX_OK) {
        return;
    }
    read_index = ldl_le_p(fifo);
    write_index = ldl_le_p(fifo + 4);
    if (read_index >= 1024 || write_index >= 1024) {
        qemu_log_mask(LOG_UNIMP, "totoro-ipc: rejected FIFO index %u/%u\n",
                      read_index, write_index);
        return;
    }
    if (read_index == write_index) {
        return;
    }
    uint8_t raw_handle[4];
    if (address_space_read(&address_space_memory,
                           TOTORO_IPC_BASE + TOTORO_IPC_FIFO + 16 + read_index * 4,
                           MEMTXATTRS_UNSPECIFIED, raw_handle, 4) != MEMTX_OK) {
        return;
    }
    handle = ldl_le_p(raw_handle);
    if (!totoro_ipc_range(handle, sizeof(descriptor))) {
        qemu_log_mask(LOG_UNIMP, "totoro-ipc: rejected handle %08x\n", handle);
        return;
    }
    if (address_space_read(&address_space_memory, TOTORO_IPC_BASE + handle,
                           MEMTXATTRS_UNSPECIFIED, descriptor, sizeof(descriptor)) != MEMTX_OK) {
        return;
    }
    /* Observe only: never advertise a CP or consume a request without service. */
    uint8_t pool[108], rpc[60];
    uint32_t poff = ldl_le_p(descriptor + 12);
    uint32_t start = ldl_le_p(descriptor + 16);
    uint32_t data = ldl_le_p(descriptor + 20);
    uint32_t bytes = ldl_le_p(descriptor + 24);
    uint32_t header = ldl_le_p(descriptor + 28);
    if (!totoro_ipc_range(poff, sizeof(pool)) ||
        address_space_read(&address_space_memory, TOTORO_IPC_BASE + poff,
                           MEMTXATTRS_UNSPECIFIED, pool, sizeof(pool)) != MEMTX_OK ||
        ldl_le_p(descriptor + 8) != handle ||
        start != handle + sizeof(descriptor) ||
        !totoro_ipc_data_range(start, 0) || data < start ||
        ldl_le_p(pool + 48) > TOTORO_IPC_SIZE ||
        ldl_le_p(pool + 52) > TOTORO_IPC_SIZE ||
        !totoro_ipc_data_range(start,
            (uint64_t)ldl_le_p(pool + 48) + ldl_le_p(pool + 52)) ||
        (uint64_t)(data - start) + bytes >
            (uint64_t)ldl_le_p(pool + 48) + ldl_le_p(pool + 52) ||
        header > data - start ||
        !totoro_ipc_data_range(data - header, header) ||
        !totoro_ipc_data_range(data, bytes) ||
        bytes > ldl_le_p(pool + 48) || header > ldl_le_p(pool + 52) ||
        ldl_le_p(descriptor + 40) != 2) {
        qemu_log_mask(LOG_UNIMP, "totoro-ipc: rejected descriptor %08x (not consumed)\n",
                      handle);
        return;
    }
    if (bytes < 24 ||
        address_space_read(&address_space_memory, TOTORO_IPC_BASE + data,
                           MEMTXATTRS_UNSPECIFIED, rpc, MIN(bytes, sizeof(rpc))) != MEMTX_OK) {
        qemu_log_mask(LOG_UNIMP, "totoro-ipc: short RPC (not consumed)\n");
        return;
    }
    TotoroRPCHeader parsed;
    if (!totoro_ipc_parse_rpc(rpc, MIN(bytes, sizeof(rpc)), &parsed)) {
        qemu_log_mask(LOG_UNIMP, "totoro-ipc: rejected RPC header (not consumed)\n");
        return;
    }
    /* No payload, phone number, IMSI or SMS content in diagnostic logs. */
    qemu_log_mask(LOG_UNIMP,
                  "totoro-ipc: RPC type=%u client-index=%u tid=%u msg=%04x bytes=%u"
                  " (not consumed)\n", parsed.type, parsed.client_index,
                  parsed.tid, parsed.msg_id, bytes);
}
