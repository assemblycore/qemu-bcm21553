/*
 * Totoro OneNAND model (Samsung 4Gbit, 4 KiB page).
 * GPL-2.0-or-later.
 *
 * Explicit opt-in via TOTORO_ONENAND_PROBE=1.
 *   TOTORO_ONENAND_IMAGE=<path>   backing image (page aligned, <=512 MiB)
 *   TOTORO_ONENAND_OOB=<path>     optional separate 256-byte/page spare image
 *   TOTORO_ONENAND_RW=1           allow program/erase (default: read only)
 *   TOTORO_ONENAND_TRACE=1        log every command to stderr
 *
 * Geometry: 4096-byte main page + 256-byte spare, 64 pages per 256 KiB block,
 * 2048 blocks (512 MiB).
 */
#define ONENAND_PAGE_SIZE      4096u
#define ONENAND_SPARE_SIZE      256u
#define ONENAND_PAGES_PER_BLK    64u
#define ONENAND_BLOCK_SIZE     (ONENAND_PAGES_PER_BLK * ONENAND_PAGE_SIZE)

/* INT2 (reg[0x241]) */
#define ONENAND_INT_READ       0x0080u  /* read complete */
#define ONENAND_INT_RSTI       0x0010u  /* reset complete */
#define ONENAND_INT_ERSI       0x0020u  /* erase interrupt */
#define ONENAND_INT_PRI        0x0040u  /* program interrupt */
#define ONENAND_INT_MASTER     0x8000u  /* command complete */

/* Controller status (reg[0x240]) */
#define ONENAND_CS_ERROR       0x0400u

typedef struct TotoroOneNAND {
    MemoryRegion mr;
    uint16_t regs[0x1000];
    uint8_t data[2 * ONENAND_PAGE_SIZE];
    uint8_t spare[ONENAND_SPARE_SIZE];
    int fd;
    int oob_fd;
    bool writable;
    uint64_t media_size;
    bool otp;
} TotoroOneNAND;

static bool totoro_onenand_trace(void)
{
    const char *t = g_getenv("TOTORO_ONENAND_TRACE");

    return t && (!strcmp(t, "1") || !strcmp(t, "2"));
}

static bool totoro_onenand_trace_regs(void)
{
    return !g_strcmp0(g_getenv("TOTORO_ONENAND_TRACE"), "2");
}

/* Portable positional I/O (MinGW has no pread/pwrite). */
static ssize_t totoro_onenand_pread(int fd, void *buf, size_t len, uint64_t off)
{
#ifdef _WIN32
    size_t done = 0;
    if (_lseeki64(fd, (__int64)off, SEEK_SET) < 0) {
        return -1;
    }
    while (done < len) {
        int r = read(fd, (uint8_t *)buf + done, len - done);
        if (r <= 0) {
            return done ? (ssize_t)done : -1;
        }
        done += r;
    }
    return done;
#else
    return pread(fd, buf, len, off);
#endif
}

static ssize_t totoro_onenand_pwrite(int fd, const void *buf, size_t len,
                                     uint64_t off)
{
#ifdef _WIN32
    size_t done = 0;
    if (_lseeki64(fd, (__int64)off, SEEK_SET) < 0) {
        return -1;
    }
    while (done < len) {
        int r = write(fd, (const uint8_t *)buf + done, len - done);
        if (r <= 0) {
            return done ? (ssize_t)done : -1;
        }
        done += r;
    }
    return done;
#else
    return pwrite(fd, buf, len, off);
#endif
}

static void totoro_onenand_reset(void *opaque)
{
    TotoroOneNAND *s = opaque;

    s->otp = false;
    memset(s->regs, 0, sizeof(s->regs));
    memset(s->data, 0xff, sizeof(s->data));
    memset(s->spare, 0xff, sizeof(s->spare));
    s->regs[0x000] = 0x00ec;   /* Samsung */
    s->regs[0x001] = 0x0050;   /* 4 Gbit */
    s->regs[0x002] = 0x0010;
    s->regs[0x003] = ONENAND_PAGE_SIZE;
    s->regs[0x004] = 512;      /* boot buffer size */
    s->regs[0x005] = 1;        /* one data buffer */
    s->regs[0x006] = 1;        /* MLC */
    s->regs[0x241] = ONENAND_INT_MASTER | ONENAND_INT_RSTI;
    s->regs[0x24e] = 4;        /* unlocked */
}

static uint64_t totoro_onenand_offset(TotoroOneNAND *s)
{
    uint64_t block = s->regs[0x100] & 0x7ff;
    unsigned page = (s->regs[0x107] >> 2) & (ONENAND_PAGES_PER_BLK - 1);

    return block * ONENAND_BLOCK_SIZE + (uint64_t)page * ONENAND_PAGE_SIZE;
}

static bool totoro_onenand_in_media(TotoroOneNAND *s, uint64_t off, size_t len)
{
    return off + len <= s->media_size;
}

static void totoro_onenand_cmd(TotoroOneNAND *s, uint16_t cmd)
{
    int buffer = (s->regs[0x200] >> 8) & 1;
    uint16_t int2 = ONENAND_INT_MASTER;
    uint64_t off;

    s->regs[0x240] = 0;

    switch (cmd) {
    case 0xf0: /* reset */
        totoro_onenand_reset(s);
        return;

    case 0x65: /* OTP access: signal RSTI (Reset Interrupt) as completion */
        s->otp = true;
        int2 |= ONENAND_INT_RSTI;
        break;

    case 0x2a: /* lock */
    case 0x2c: /* lock tight */
        s->regs[0x24e] = cmd == 0x2c ? 2 : 1;
        break;
    case 0x23: /* unlock */
    case 0x27: /* unlock all */
        s->regs[0x24e] = 4;
        break;

    case 0x00: /* load main page into data buffer */
    case 0x13: { /* load spare (OOB) */
        off = totoro_onenand_offset(s);
        memset(s->spare, 0xff, sizeof(s->spare));
        if (s->oob_fd >= 0 && !s->otp &&
            totoro_onenand_pread(s->oob_fd, s->spare, sizeof(s->spare),
                                off / ONENAND_PAGE_SIZE * ONENAND_SPARE_SIZE)
                != sizeof(s->spare)) {
            s->regs[0x240] = ONENAND_CS_ERROR;
        }
        if (cmd == 0x00) {
            memset(s->data + buffer * ONENAND_PAGE_SIZE, 0xff, ONENAND_PAGE_SIZE);
            if (s->fd >= 0 && !s->otp &&
                totoro_onenand_in_media(s, off, ONENAND_PAGE_SIZE) &&
                totoro_onenand_pread(s->fd, s->data + buffer * ONENAND_PAGE_SIZE,
                                    ONENAND_PAGE_SIZE, off) != ONENAND_PAGE_SIZE) {
                s->regs[0x240] = ONENAND_CS_ERROR;
            }
        }
        int2 |= ONENAND_INT_READ;
        break;
    }

    case 0x7f: /* cache program: synchronous completion, no queued page */
    case 0x80: /* program main page from data buffer */
    case 0x1a: /* program spare */
        int2 |= ONENAND_INT_PRI;
        off = totoro_onenand_offset(s);
        if (!s->writable || s->fd < 0) {
            s->regs[0x240] = ONENAND_CS_ERROR;
            int2 |= ONENAND_INT_MASTER;
            break;
        }
        if (cmd != 0x1a) {
            if (!totoro_onenand_in_media(s, off, ONENAND_PAGE_SIZE) ||
                totoro_onenand_pwrite(s->fd, s->data + buffer * ONENAND_PAGE_SIZE,
                                      ONENAND_PAGE_SIZE, off) != ONENAND_PAGE_SIZE) {
                s->regs[0x240] = ONENAND_CS_ERROR;
                int2 |= ONENAND_INT_MASTER;
            }
        }
        /* Main program also commits the spare buffer. Never write OOB into
         * the following main page of a main-only image. */
        if (s->oob_fd >= 0) {
            if (totoro_onenand_pwrite(s->oob_fd, s->spare, sizeof(s->spare),
                                     off / ONENAND_PAGE_SIZE * ONENAND_SPARE_SIZE)
                != sizeof(s->spare)) {
                s->regs[0x240] = ONENAND_CS_ERROR;
            }
        } else if (cmd == 0x1a) {
            s->regs[0x240] = ONENAND_CS_ERROR;
        }
        int2 |= ONENAND_INT_PRI;
        break;

    case 0x94: { /* erase one block */
        uint64_t block = (s->regs[0x100] & 0x7ff) * ONENAND_BLOCK_SIZE;
        uint8_t erase[ONENAND_BLOCK_SIZE];

        int2 |= ONENAND_INT_ERSI;

        if (!s->writable || s->fd < 0 ||
            !totoro_onenand_in_media(s, block, ONENAND_BLOCK_SIZE)) {
            s->regs[0x240] = ONENAND_CS_ERROR;
            int2 |= ONENAND_INT_MASTER;
            break;
        }
        memset(erase, 0xff, sizeof(erase));
        if (totoro_onenand_pwrite(s->fd, erase, sizeof(erase), block) !=
            (ssize_t)sizeof(erase)) {
            s->regs[0x240] = ONENAND_CS_ERROR;
            int2 |= ONENAND_INT_MASTER;
            break;
        }
        if (s->oob_fd >= 0 &&
            totoro_onenand_pwrite(s->oob_fd, erase,
                ONENAND_PAGES_PER_BLK * ONENAND_SPARE_SIZE,
                block / ONENAND_PAGE_SIZE * ONENAND_SPARE_SIZE) !=
                ONENAND_PAGES_PER_BLK * ONENAND_SPARE_SIZE) {
            s->regs[0x240] = ONENAND_CS_ERROR;
        }
        memset(s->data, 0xff, sizeof(s->data));
        memset(s->spare, 0xff, sizeof(s->spare));
        int2 |= ONENAND_INT_ERSI;
        break;
    }

    default:
        s->regs[0x240] = ONENAND_CS_ERROR;
        int2 |= ONENAND_INT_MASTER;
        qemu_log_mask(LOG_UNIMP, "totoro-onenand: unsupported command 0x%04x\n", cmd);
        break;
    }

    s->regs[0x241] = int2;
    if (totoro_onenand_trace()) {
        fprintf(stderr, "onenand: cmd=0x%04x blk=0x%04x page=0x%02x buf=%u "
                "cs=0x%04x int2=0x%04x\n", cmd,
                s->regs[0x100] & 0x7ff, (s->regs[0x107] >> 2) & 63,
                (unsigned)((s->regs[0x200] >> 8) & 1),
                s->regs[0x240], s->regs[0x241]);
    }
}

static uint8_t totoro_onenand_byte(TotoroOneNAND *s, hwaddr off)
{
    if (off >= 0x1e000 && off < 0x20000) {
        uint16_t val = s->regs[(off - 0x1e000) >> 1];
        return val >> ((off & 1) * 8);
    }
    if (off >= 0x400 && off < 0x400 + sizeof(s->data)) {
        return s->data[off - 0x400];
    }
    if (off >= 0x10020 && off < 0x10020 + sizeof(s->spare)) {
        return s->spare[off - 0x10020];
    }
    return 0xff;
}

static uint64_t totoro_onenand_read(void *opaque, hwaddr off, unsigned size)
{
    TotoroOneNAND *s = opaque;
    uint64_t result = 0;

    for (unsigned i = 0; i < size; i++) {
        result |= (uint64_t)totoro_onenand_byte(s, off + i) << (i * 8);
    }
    if (totoro_onenand_trace_regs() && off >= 0x1e200 && off < 0x1e500) {
        fprintf(stderr, "onenand: R reg[0x%03x] = 0x%04x\n",
                (unsigned)((off - 0x1e000) >> 1), (unsigned)(result & 0xffff));
    }
    return result;
}

static void totoro_onenand_write(void *opaque, hwaddr off, uint64_t val,
                                 unsigned size)
{
    TotoroOneNAND *s = opaque;

    for (unsigned i = 0; i < size; i++) {
        hwaddr a = off + i;
        uint8_t byte = val >> (i * 8);

        if (a >= 0x1e000 && a < 0x20000) {
            unsigned reg = (a - 0x1e000) >> 1;
            unsigned shift = (a & 1) * 8;

            /* Interrupt status is writable: the driver clears it before a command. */
            if (reg == 0x240 || reg >= 0xf00) {
                continue;
            }
            s->regs[reg] = (s->regs[reg] & ~(0xffu << shift)) | (byte << shift);
        } else if (a >= 0x400 && a < 0x400 + sizeof(s->data)) {
            s->data[a - 0x400] = byte;
        } else if (a >= 0x10020 && a < 0x10020 + sizeof(s->spare)) {
            s->spare[a - 0x10020] = byte;
        }
    }

    if (off <= 0x1e440 && off + size > 0x1e440) {
        totoro_onenand_cmd(s, s->regs[0x220]);
    }
}

static const MemoryRegionOps totoro_onenand_ops = {
    .read = totoro_onenand_read,
    .write = totoro_onenand_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 4 },
};

static void totoro_onenand_probe_init(MemoryRegion *sysmem)
{
    TotoroOneNAND *s;
    const char *path;
    struct stat st;
    int flags = O_RDONLY | O_BINARY;

    if (g_strcmp0(g_getenv("TOTORO_ONENAND_PROBE"), "1")) {
        return;
    }

    s = g_new0(TotoroOneNAND, 1);
    s->fd = -1;
    s->oob_fd = -1;
    s->writable = g_strcmp0(g_getenv("TOTORO_ONENAND_RW"), "1") == 0;
    path = g_getenv("TOTORO_ONENAND_IMAGE");
    if (path && *path) {
        if (s->writable) {
            flags = O_RDWR | O_BINARY;
        }
        s->fd = open(path, flags);
        if (s->fd < 0 || fstat(s->fd, &st) || !S_ISREG(st.st_mode) ||
            st.st_size <= 0 || st.st_size > 512ULL * 1024 * 1024 ||
            st.st_size % ONENAND_PAGE_SIZE) {
            error_report("totoro-onenand: backing must be a %s page-aligned image <=512MiB",
                         s->writable ? "writable" : "read-only");
            exit(1);
        }
        s->media_size = st.st_size;
    }

    path = g_getenv("TOTORO_ONENAND_OOB");
    if (path && *path) {
        s->oob_fd = open(path, flags);
        if (s->fd < 0 || s->oob_fd < 0 || fstat(s->oob_fd, &st) ||
            !S_ISREG(st.st_mode) || st.st_size !=
                s->media_size / ONENAND_PAGE_SIZE * ONENAND_SPARE_SIZE) {
            error_report("totoro-onenand: OOB must contain 256 bytes per main page");
            exit(1);
        }
    }
    path = g_getenv("TOTORO_ONENAND_IMAGE");
    totoro_onenand_reset(s);
    memory_region_init_io(&s->mr, NULL, &totoro_onenand_ops, s,
                          "totoro-onenand", 0x20000);
    /* SBL accesses the register block at phys 0x0041e000. */
    memory_region_add_subregion_overlap(sysmem, 0x00400000, &s->mr, 3);
    qemu_register_reset(totoro_onenand_reset, s);

    if (totoro_onenand_trace()) {
        fprintf(stderr, "onenand: %s, %" PRIu64 " bytes%s\n",
                path ? path : "(no image)", s->media_size,
                s->writable ? ", writable" : ", read-only");
    }
}
