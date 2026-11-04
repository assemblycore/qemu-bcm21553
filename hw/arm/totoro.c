/*
 * Samsung Galaxy Y GT-S5360 "totoro" board (QEMU).
 * The SoC lives in hw/arm/bcm21553.c; this file only wires the phone:
 * MAX8986 PMIC/RTC/MUIC, Silabs F760 touch, Bosch BMA222 accelerometer,
 * SD card detect, LCD framebuffer, kernel boot parameters.
 */
#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu/bswap.h"
#include <glib/gstdio.h>
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/boards.h"
#include "hw/arm/boot.h"
#include "hw/i2c/i2c.h"
#include "ui/input.h"
#include "ui/console.h"
#include "system/reset.h"
#include "target/arm/cpu-qom.h"
#include "target/arm/cpu.h"
#include "qom/object.h"
#include "hw/arm/bcm21553.h"

/* Bounded, JPEG-only decoder; license/provenance in LICENSES/STB-IMAGE-NOTICE.txt. */
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#define STBI_NO_HDR
#define STBI_NO_LINEAR
#define STBI_MAX_DIMENSIONS 320
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-prototypes"
#include "hw/arm/totoro-stb-image.h"
#pragma GCC diagnostic pop


#define TOTORO_KERNEL_BASE    0x81600000   /* PHYS_OFFSET */
#define TOTORO_KERNEL_RAM     (362 * MiB)  /* mem=362M */
#define TOTORO_BOARD_ID       477          /* MACH_BCM1160 */
#define TOTORO_FB_BASE        0x93500000   /* lcdc.c lcd_alloc_fb */
#define TOTORO_LCD_W          240
#define TOTORO_LCD_H          320

/* board GPIO usage (board-totoro.c) */
#define TOTORO_SD_CD_PIN 16     /* SDHC3 card detect, active low */
#define TSP_SDA_PIN 27          /* i2c-gpio bus 3: Silabs F760 @0x20 */
#define TSP_SCL_PIN 26
#define TSP_INT_PIN 30          /* falling edge */
#define ACC_SDA_PIN 15          /* i2c-gpio bus 4: BMA222 @0x08 */
#define ACC_SCL_PIN 7

/* ---- touch event queue ---- */
typedef struct { int down, x, y, key; } TspState;   /* key: 1 menu, 2 back */
static TspState tsp_q[32];
static int tsp_qh, tsp_qn;
static bool tsp_pending;
static TspState tsp_in;
static bool tsp_in_dirty, tsp_last_down;

/* INT line is low while a report is waiting to be read */
static void tsp_set_pending(bool p)
{
    tsp_pending = p;
    bcm21553_gpio_set_input(TSP_INT_PIN, !p);
    if (p) {
        bcm21553_gpio_pend(TSP_INT_PIN);
    }
}

static void tsp_push(TspState st)
{
    if (tsp_qn > 0) {
        TspState *last = &tsp_q[(tsp_qh + tsp_qn - 1) % 32];
        if (last->down && st.down && !last->key && !st.key) {        /* coalesce moves */
            *last = st;
            goto out;
        }
    }
    if (tsp_qn < 32) {
        tsp_q[(tsp_qh + tsp_qn) % 32] = st;
        tsp_qn++;
    }
out:
    if (!tsp_pending) {
        tsp_set_pending(true);
    }
}

#define TYPE_TOTORO_F760 "totoro-f760"
OBJECT_DECLARE_SIMPLE_TYPE(TotoroF760, TOTORO_F760)

struct TotoroF760 {
    I2CSlave parent_obj;
    uint8_t reg;
    int nsend;
    uint8_t out[16];
    int outlen, outpos;
};

static int f760_event(I2CSlave *i2c, enum i2c_event e)
{
    TotoroF760 *s = TOTORO_F760(i2c);

    if (e == I2C_START_SEND) {
        s->nsend = 0;
    } else if (e == I2C_START_RECV) {
        memset(s->out, 0, sizeof(s->out));
        s->outpos = 0;
        s->outlen = 16;
        if (s->reg == 0x02) {               /* touch report, 13 bytes */
            TspState st = { 0 };
            if (tsp_qn > 0) {
                st = tsp_q[tsp_qh];
                tsp_qh = (tsp_qh + 1) % 32;
                tsp_qn--;
            }
            if (st.key) {                   /* touch key: buf1 = key | press<<4 */
                s->out[1] = st.key | (st.down ? 0x10 : 0);
            } else if (st.down) {
                s->out[0] = 1;              /* touch_num */
                s->out[2] = 0x10;           /* id0 = 1 */
                s->out[3] = st.x >> 8;
                s->out[4] = st.x & 0xff;
                s->out[5] = st.y >> 8;
                s->out[6] = st.y & 0xff;
                s->out[7] = 0x30;           /* strength */
            }
            if (tsp_qn > 0) {
                bcm21553_gpio_pend(TSP_INT_PIN);     /* more events queued */
            } else {
                tsp_set_pending(false);
            }
        } else if (s->reg == 0x1E) {        /* ESD watchdog: must change every read */
            static uint8_t wdog;
            s->out[0] = ++wdog;
        } else if (s->reg == 0x1F) {        /* fw ver, ?, module id (YTE new) */
            s->out[0] = 0x20;
            s->out[1] = 0x01;
            s->out[2] = 0x0A;
        }
    }
    return 0;
}

static int f760_send(I2CSlave *i2c, uint8_t d)
{
    TotoroF760 *s = TOTORO_F760(i2c);

    if (s->nsend == 1) {
        s->reg = d;                         /* after ESCAPE 0xAA */
    }
    s->nsend++;
    return 0;
}

static uint8_t f760_recv(I2CSlave *i2c)
{
    TotoroF760 *s = TOTORO_F760(i2c);
    return s->outpos < s->outlen ? s->out[s->outpos++] : 0;
}

static void f760_class_init(ObjectClass *oc, const void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);
    k->event = f760_event;
    k->send = f760_send;
    k->recv = f760_recv;
}

static const TypeInfo f760_info = {
    .name = TYPE_TOTORO_F760,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TotoroF760),
    .class_init = f760_class_init,
};

/* ---- BMA222 slave: axes from $HOME/totoro/run/acc.txt "x y z" (64 LSB/g) ---- */
#define TYPE_TOTORO_BMA222 "totoro-bma222"
OBJECT_DECLARE_SIMPLE_TYPE(TotoroBMA222, TOTORO_BMA222)

struct TotoroBMA222 {
    I2CSlave parent_obj;
    uint8_t regs[64];
    uint8_t ptr;
    bool first;
};

static void bma222_refresh(TotoroBMA222 *s)
{
    int x = 0, y = 64, z = 0;
    char path[512];
    FILE *f;

    snprintf(path, sizeof(path), "%s/totoro/run/acc.txt", getenv("HOME") ? getenv("HOME") : "/tmp");
    f = fopen(path, "r");
    if (f) {
        if (fscanf(f, "%d %d %d", &x, &y, &z) != 3) {
            x = 0; y = 64; z = 0;
        }
        fclose(f);
    }
    s->regs[0x02] = 0x01; s->regs[0x03] = (uint8_t)(int8_t)x;
    s->regs[0x04] = 0x01; s->regs[0x05] = (uint8_t)(int8_t)y;
    s->regs[0x06] = 0x01; s->regs[0x07] = (uint8_t)(int8_t)z;
}

static int bma222_event(I2CSlave *i2c, enum i2c_event e)
{
    TotoroBMA222 *s = TOTORO_BMA222(i2c);

    if (e == I2C_START_SEND) {
        s->first = true;
    } else if (e == I2C_START_RECV && s->ptr <= 0x07) {
        bma222_refresh(s);
    }
    return 0;
}

static int bma222_send(I2CSlave *i2c, uint8_t d)
{
    TotoroBMA222 *s = TOTORO_BMA222(i2c);

    if (s->first) {
        s->ptr = d & 0x3f;
        s->first = false;
    } else {
        if (s->ptr > 0x08) {
            s->regs[s->ptr] = d;
        }
        s->ptr = (s->ptr + 1) & 0x3f;
    }
    return 0;
}

static uint8_t bma222_recv(I2CSlave *i2c)
{
    TotoroBMA222 *s = TOTORO_BMA222(i2c);
    uint8_t v;

    s->regs[0x00] = 0x03;                   /* chip id */
    v = s->regs[s->ptr];
    s->ptr = (s->ptr + 1) & 0x3f;
    return v;
}

static void bma222_class_init(ObjectClass *oc, const void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);
    k->event = bma222_event;
    k->send = bma222_send;
    k->recv = bma222_recv;
}

static const TypeInfo bma222_info = {
    .name = TYPE_TOTORO_BMA222,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TotoroBMA222),
    .class_init = bma222_class_init,
};

/* ---------------------------------------------------------------- */
/* MAX8986 PM @0x08 with interrupts: INT1..3 (0x01..0x03) are        */
/* read-to-clear, INTMSK1..3 (0x0B..0x0D) 1 = masked, IRQ line is     */
/* PMU_IRQ GPIO34 (low while an unmasked interrupt is pending).      */
/* ---------------------------------------------------------------- */
#define PMU_IRQ_PIN       34
#define PM_INT1_PWRONBR   0x01      /* power key released */
#define PM_INT1_PWRONBF   0x02      /* power key pressed  */

#define TYPE_TOTORO_PM "totoro-max8986-pm"
OBJECT_DECLARE_SIMPLE_TYPE(TotoroPM, TOTORO_PM)

struct TotoroPM {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t ptr;
    bool first, line, pend_release;
};
static TotoroPM *totoro_pm;

static void pm_update_line(TotoroPM *s)
{
    bool a = ((s->regs[1] & ~s->regs[0x0b]) | (s->regs[2] & ~s->regs[0x0c]) |
              (s->regs[3] & ~s->regs[0x0d])) != 0;

    bcm21553_gpio_set_input(PMU_IRQ_PIN, !a);
    if (a && !s->line) {
        bcm21553_gpio_pend(PMU_IRQ_PIN);
    }
    s->line = a;
}

static int pm_event(I2CSlave *i2c, enum i2c_event e)
{
    TotoroPM *s = TOTORO_PM(i2c);

    if (e == I2C_START_SEND) {
        s->first = true;
    }
    return 0;
}

static int pm_send(I2CSlave *i2c, uint8_t d)
{
    TotoroPM *s = TOTORO_PM(i2c);

    if (s->first) {
        s->ptr = d;
        s->first = false;
    } else {
        /* Register 0 is the read-only device identification register. */
        if (s->ptr != 0) {
            s->regs[s->ptr] = d;
        }
        s->ptr++;
        pm_update_line(s);
    }
    return 0;
}

static uint8_t pm_recv(I2CSlave *i2c)
{
    TotoroPM *s = TOTORO_PM(i2c);
    uint8_t r = s->ptr, v = r == 0 ? 0x32 : s->regs[r];

    if (r >= 1 && r <= 3) {
        s->regs[r] = 0;
        if (r == 1 && s->pend_release) {    /* deliver release after press was seen */
            s->regs[1] |= PM_INT1_PWRONBR;
            s->pend_release = false;
        }
        pm_update_line(s);
    }
    s->ptr++;
    return v;
}

static void totoro_pm_class_init(ObjectClass *oc, const void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = pm_event;
    k->send = pm_send;
    k->recv = pm_recv;
}

static const TypeInfo totoro_pm_info = {
    .name = TYPE_TOTORO_PM,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TotoroPM),
    .class_init = totoro_pm_class_init,
};

static void totoro_power_key(bool down)
{
    TotoroPM *s = totoro_pm;

    if (!s) {
        return;
    }
    if (down) {
        s->regs[1] |= PM_INT1_PWRONBF;
    } else if (s->regs[1] & PM_INT1_PWRONBF) {
        s->pend_release = true;             /* press not read yet */
    } else {
        s->regs[1] |= PM_INT1_PWRONBR;
    }
    pm_update_line(s);
}

/* ---------------------------------------------------------------- */
/* Phone keys                                                       */
/*  home/volup/voldown: BCM keypad matrix R2C0 / R0C0 / R1C0         */
/*  menu/back: F760 touch keys, power: MAX8986 ponkey                */
/* ---------------------------------------------------------------- */
enum { K_HOME, K_MENU, K_BACK, K_VOLUP, K_VOLDN, K_POWER, K_NUM };
static const char *const key_names[K_NUM] = {
    "home", "menu", "back", "volup", "voldown", "power",
};
static bool key_down[K_NUM];
static bool key_keyboard[K_NUM], key_mouse[K_NUM], key_latched[K_NUM];
static bool host_key_held[K_NUM], host_shift_left, host_shift_right;
static void skin_controls_refresh(void);

static void totoro_key(int k, bool down);

static void totoro_key_combined(int k)
{
    if (k >= 0 && k < K_NUM) {
        totoro_key(k, key_keyboard[k] || key_mouse[k] || key_latched[k]);
    }
}


static void totoro_key(int k, bool down)
{
    if (k < 0 || k >= K_NUM || key_down[k] == down) {
        return;
    }
    key_down[k] = down;
    switch (k) {
    case K_HOME:
        bcm21553_keypad_set(2, 0, down);
        break;
    case K_VOLUP:
        bcm21553_keypad_set(0, 0, down);
        break;
    case K_VOLDN:
        bcm21553_keypad_set(1, 0, down);
        break;
    case K_MENU:
    case K_BACK: {
        TspState st = { .down = down, .key = (k == K_MENU) ? 1 : 2 };
        tsp_push(st);
        break;
    }
    case K_POWER:
        totoro_power_key(down);
        break;
    }
}

/* ---------------------------------------------------------------- */
/* Skin: layout file "totoro.skin"                                   */
/*   image <file.ppm>      background (binary P6)                    */
/*   screen <x> <y> <scale>                                          */
/*   key <name> <x> <y> <w> <h>                                      */
/* ---------------------------------------------------------------- */
typedef struct { int k, x, y, w, h; } SkinKey;
static struct {
    uint32_t *px, *original;
    int w, h, sx, sy, sc;
    SkinKey keys[16];
    int nkeys;
} skin = { .sc = 1 };
static char *totoro_skin_path;

static char *totoro_get_skin(Object *obj, Error **errp)
{
    return g_strdup(totoro_skin_path ? totoro_skin_path : "");
}

static void totoro_set_skin(Object *obj, const char *v, Error **errp)
{
    g_free(totoro_skin_path);
    totoro_skin_path = g_strdup(v);
}

static bool load_ppm(const char *path, uint32_t **px, int *w, int *h)
{
    gchar *buf = NULL;
    gsize len = 0, need, i;
    const uint8_t *p;
    char *q, *end;
    long v[3];
    int n = 0;
    bool ok = false;

    if (!g_file_get_contents(path, &buf, &len, NULL)) {
        return false;
    }
    if (len < 3 || buf[0] != 'P' || buf[1] != '6') {
        goto out;
    }
    q = buf + 2;
    while (n < 3) {
        while (q < buf + len && (g_ascii_isspace(*q) || *q == '#')) {
            if (*q == '#') {
                while (q < buf + len && *q != '\n') {
                    q++;
                }
            } else {
                q++;
            }
        }
        if (q >= buf + len || !g_ascii_isdigit(*q)) {
            goto out;
        }
        v[n++] = strtol(q, &end, 10);
        q = end;
    }
    q++;
    if (v[0] <= 0 || v[1] <= 0 || v[0] > 4096 || v[1] > 4096 || v[2] != 255) {
        goto out;
    }
    need = (gsize)v[0] * v[1] * 3;
    if ((gsize)(q - buf) + need > len) {
        goto out;
    }
    *w = v[0];
    *h = v[1];
    *px = g_new(uint32_t, (gsize)v[0] * v[1]);
    p = (const uint8_t *)q;
    for (i = 0; i < (gsize)v[0] * v[1]; i++) {
        (*px)[i] = 0xff000000u | (p[3 * i] << 16) | (p[3 * i + 1] << 8) | p[3 * i + 2];
    }
    ok = true;
out:
    g_free(buf);
    return ok;
}

static void totoro_load_skin(const char *layout)
{
    gchar *txt = NULL, **lines, *dir;
    int i, k;

    if (!layout || !*layout) {
        return;
    }
    if (!g_file_get_contents(layout, &txt, NULL, NULL)) {
        warn_report("totoro: cannot read skin layout '%s'", layout);
        return;
    }
    dir = g_path_get_dirname(layout);
    lines = g_strsplit(txt, "\n", -1);
    for (i = 0; lines[i]; i++) {
        char name[32], file[512];
        int a, b, c, d;
        char *l = g_strstrip(lines[i]);

        if (l[0] == '#' || !l[0]) {
            continue;
        }
        if (sscanf(l, "image %511s", file) == 1) {
            char *p = g_path_is_absolute(file) ? g_strdup(file)
                                              : g_build_filename(dir, file, NULL);
            if (!load_ppm(p, &skin.px, &skin.w, &skin.h)) {
                warn_report("totoro: cannot load skin image '%s' (binary PPM P6)", p);
            }
            g_free(p);
        } else if (sscanf(l, "screen %d %d %d", &a, &b, &c) == 3) {
            skin.sx = a;
            skin.sy = b;
            skin.sc = c > 0 ? c : 1;
        } else if (sscanf(l, "key %31s %d %d %d %d", name, &a, &b, &c, &d) == 5 &&
                   skin.nkeys < 16) {
            for (k = 0; k < K_NUM; k++) {
                if (!strcmp(name, key_names[k])) {
                    skin.keys[skin.nkeys++] = (SkinKey){ k, a, b, c, d };
                }
            }
        }
    }
    g_strfreev(lines);
    g_free(dir);
    g_free(txt);
    if (skin.px && (skin.sx < 0 || skin.sy < 0 ||
                    skin.sx + TOTORO_LCD_W * skin.sc > skin.w ||
                    skin.sy + TOTORO_LCD_H * skin.sc > skin.h)) {
        warn_report("totoro: skin screen rectangle does not fit the image");
        g_free(skin.px);
        skin.px = NULL;
    }
    if (skin.px) {
        skin.original = g_memdup2(skin.px, (size_t)skin.w * skin.h * 4);
    }
    if (!skin.px) {
        skin.sx = skin.sy = 0;
        skin.sc = 1;
        skin.nkeys = 0;
    }
}

/*
 * Authentic artwork, not an SBL implementation.  Direct-kernel launches may
 * supply their own read-only param.lfs.  Only validated J4FS file headers are
 * considered; do not take the first JPEG EOI (EXIF may contain thumbnails).
 */
static char *totoro_boot_param;
static uint32_t *totoro_logo;

static char *totoro_get_boot_param(Object *obj, Error **errp)
{
    return g_strdup(totoro_boot_param ? totoro_boot_param : "");
}

static void totoro_set_boot_param(Object *obj, const char *value, Error **errp)
{
    g_free(totoro_boot_param);
    totoro_boot_param = g_strdup(value);
}

static void totoro_load_logo(void)
{
    gchar *data = NULL;
    gsize size = 0, off;
    GStatBuf statbuf;
    int w, h, components;

    if (!totoro_boot_param || !*totoro_boot_param) {
        return;
    }
    if (g_stat(totoro_boot_param, &statbuf) ||
        statbuf.st_size < 160 || statbuf.st_size > 16 * MiB ||
        !g_file_get_contents(totoro_boot_param, &data, &size, NULL)) {
        warn_report("totoro: cannot read boot-param (max 16 MiB)");
        return;
    }
    for (off = 0; off + 160 <= size; off += 2048) {
        const uint8_t *header = (const uint8_t *)data + off;
        uint32_t flags = ldl_le_p(header + 16);
        uint32_t length = ldl_le_p(header + 28);
        int unit;

        if (ldl_le_p(header + 8) != 0x12345678 ||
            (flags & 1) != ((flags >> 1) & 1) ||
            memcmp(header + 32, "logo.jpg", 9) ||
            length < 4 || length > MiB) {
            continue;
        }
        for (unit = 2048; unit <= 4096; unit += 2048) {
            gsize start = off + unit;
            const uint8_t *jpeg;
            unsigned char *rgb;
            size_t i;

            if (start > size || length > size - start) {
                continue;
            }
            jpeg = (const uint8_t *)data + start;
            if (jpeg[0] != 0xff || jpeg[1] != 0xd8 ||
                !stbi_info_from_memory(jpeg, length, &w, &h, &components) ||
                w != TOTORO_LCD_W || h != TOTORO_LCD_H) {
                continue;
            }
            rgb = stbi_load_from_memory(jpeg, length, &w, &h, &components, 3);
            if (!rgb) {
                continue;
            }
            totoro_logo = g_new(uint32_t, TOTORO_LCD_W * TOTORO_LCD_H);
            for (i = 0; i < TOTORO_LCD_W * TOTORO_LCD_H; i++) {
                totoro_logo[i] = 0xff000000u | (rgb[3*i] << 16) |
                                (rgb[3*i+1] << 8) | rgb[3*i+2];
            }
            stbi_image_free(rgb);
            g_free(data);
            bcm21553_lcd_set_boot_logo(totoro_logo);
            return;
        }
    }
    warn_report("totoro: no valid 240x320 logo.jpg in boot-param; splash disabled");
    g_free(data);
}

static int skin_key_at(int x, int y)
{
    int i;

    for (i = 0; i < skin.nkeys; i++) {
        SkinKey *k = &skin.keys[i];
        if (x >= k->x && x < k->x + k->w && y >= k->y && y < k->y + k->h) {
            return k->k;
        }
    }
    return -1;
}

/* Host-only control overlay: never writes into the guest framebuffer. */
static int mouse_x, mouse_y, mouse_key = -1;
static bool mouse_touch, mouse_button_down;

/* 3x5 capitals used to label narrow side buttons without external fonts. */
static const char *skin_font_chars = "ABDEHLMNOPRUWV+-";
static const uint16_t skin_font[] = {
    0x2bed, 0x6bae, 0x6b6e, 0x79a7, 0x5bed, 0x4927, 0x5fed, 0x5ffd, 0x2b6a, 0x6ba4, 0x6bad, 0x5b6f, 0x5bfd, 0x5b6a, 0x05d0, 0x01c0,
};

static void skin_pixel(int x, int y, uint32_t color)
{
    if (x >= 0 && y >= 0 && x < skin.w && y < skin.h &&
        !(x >= skin.sx && x < skin.sx + TOTORO_LCD_W * skin.sc &&
          y >= skin.sy && y < skin.sy + TOTORO_LCD_H * skin.sc)) {
        skin.px[(size_t)y * skin.w + x] = color;
    }
}

static void skin_controls_refresh(void)
{
    int i, x, y;
    static const char *const labels[K_NUM] = {
        "HOME", "MENU", "BACK", "VOL+", "VOL-", "POWER",
    };

    if (!skin.px || !skin.original) {
        return;
    }
    memcpy(skin.px, skin.original, (size_t)skin.w * skin.h * 4);
    for (i = 0; i < skin.nkeys; i++) {
        SkinKey *z = &skin.keys[i];
        bool hover = mouse_x >= z->x && mouse_x < z->x + z->w &&
                     mouse_y >= z->y && mouse_y < z->y + z->h;
        uint32_t col = key_latched[z->k] ? 0xffffae36 :
                       key_down[z->k] ? 0xff50ee80 :
                       hover ? 0xff59cfff : 0xffa0b7c8;
        int thick = MAX(1, skin.sc);
        /* Bounded loops even for malformed third-party layouts. */
        int left = MAX(0, z->x), right = MIN(skin.w, z->x + z->w);
        int top = MAX(0, z->y), bottom = MIN(skin.h, z->y + z->h);
        for (y = top; y < bottom; y++) {
            for (x = left; x < right; x++) {
                if (x < left + thick || x >= right - thick ||
                    y < top + thick || y >= bottom - thick) {
                    skin_pixel(x, y, col);
                } else if (key_down[z->k] || hover) {
                    uint32_t old = skin.px[(size_t)y * skin.w + x];
                    /* 25% colored tint; keep the original photograph visible. */
                    skin_pixel(x, y, 0xff000000 |
                        ((((old >> 16) & 255) * 3 + ((col >> 16) & 255)) / 4 << 16) |
                        ((((old >> 8) & 255) * 3 + ((col >> 8) & 255)) / 4 << 8) |
                        (((old & 255) * 3 + (col & 255)) / 4));
                }
            }
        }
        const char *text = labels[z->k];
        int n = strlen(text);
        bool vertical = z->w < n * 4 * skin.sc + 4;
        int scale = vertical ? MAX(1, skin.sc) : MAX(2, skin.sc);
        int tx = vertical ? z->x + (z->w - 3 * scale) / 2 :
                            z->x + (z->w - (n * 4 - 1) * scale) / 2;
        int ty = vertical ? z->y + (z->h - n * 6 * scale) / 2 :
                            z->y + z->h - 7 * scale;
        int c, a, b, dx, dy;
        for (c = 0; c < n; c++) {
            const char *f = strchr(skin_font_chars, text[c]);
            if (!f) {
                continue;
            }
            uint16_t bits = skin_font[f - skin_font_chars];
            for (b = 0; b < 5; b++) {
                for (a = 0; a < 3; a++) {
                    if (!(bits & (1 << (14 - b * 3 - a)))) {
                        continue;
                    }
                    for (dy = 0; dy < scale; dy++) {
                        for (dx = 0; dx < scale; dx++) {
                            int xx = tx + (vertical ? 0 : c * 4 * scale) + a * scale + dx;
                            int yy = ty + (vertical ? c * 6 * scale : 0) + b * scale + dy;
                            skin_pixel(xx + 1, yy + 1, 0xff000000);
                            skin_pixel(xx, yy, col);
                        }
                    }
                }
            }
        }
    }
    bcm21553_lcd_set_skin(skin.px, skin.w, skin.h,
                         skin.sx, skin.sy, skin.sc);
}

static void tsp_input_event(DeviceState *dev, QemuConsole *src, QemuInputEvent *evt)
{
    int cw = skin.px ? skin.w : TOTORO_LCD_W;
    int ch = skin.px ? skin.h : TOTORO_LCD_H;

    switch (evt->type) {
    case INPUT_EVENT_KIND_ABS: {
        InputMoveEvent *m = &evt->abs;
        int old_hover = skin_key_at(mouse_x, mouse_y);
        if (m->axis == INPUT_AXIS_X) {
            mouse_x = qemu_input_scale_axis(m->value, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, cw - 1);
        } else if (m->axis == INPUT_AXIS_Y) {
            mouse_y = qemu_input_scale_axis(m->value, INPUT_EVENT_ABS_MIN, INPUT_EVENT_ABS_MAX, 0, ch - 1);
        }
        if (mouse_touch) {
            tsp_in_dirty = true;
        }
        if (old_hover != skin_key_at(mouse_x, mouse_y)) {
            skin_controls_refresh();
        }
        break;
    }
    case INPUT_EVENT_KIND_BTN: {
        InputBtnEvent *b = &evt->btn;
        if (b->button != INPUT_BUTTON_LEFT || b->down == mouse_button_down) {
            break;
        }
        mouse_button_down = b->down;
        if (b->down) {
            if (mouse_x >= skin.sx && mouse_x < skin.sx + TOTORO_LCD_W * skin.sc &&
                mouse_y >= skin.sy && mouse_y < skin.sy + TOTORO_LCD_H * skin.sc) {
                mouse_touch = true;
                tsp_in.down = 1;
                tsp_in_dirty = true;
            } else {
                int k = skin_key_at(mouse_x, mouse_y);
                if (k >= 0) {
                    if (host_shift_left || host_shift_right) {
                        key_latched[k] = !key_latched[k];
                    } else if (key_latched[k]) {
                        key_latched[k] = false; /* ordinary click releases latch */
                    } else {
                        key_mouse[k] = true;
                        mouse_key = k;
                    }
                    totoro_key_combined(k);
                }
            }
        } else {
            if (mouse_key >= 0) {
                key_mouse[mouse_key] = false;
                totoro_key_combined(mouse_key);
                mouse_key = -1;
            }
            if (mouse_touch) {
                mouse_touch = false;
                tsp_in.down = 0;
                tsp_in_dirty = true;
            }
        }
        skin_controls_refresh();
        break;
    }
    default:
        break;
    }
}

static void tsp_input_sync(DeviceState *dev)
{
    if (!tsp_in_dirty) {
        return;
    }
    tsp_in_dirty = false;
    tsp_in.x = MIN(MAX((mouse_x - skin.sx) / skin.sc, 0), TOTORO_LCD_W - 1);
    tsp_in.y = MIN(MAX((mouse_y - skin.sy) / skin.sc, 0), TOTORO_LCD_H - 1);
    tsp_in.key = 0;
    if (tsp_in.down || tsp_last_down) {
        tsp_push(tsp_in);
    }
    tsp_last_down = tsp_in.down;
}

static const QemuInputHandler tsp_input_handler = {
    .name  = "totoro-f760-touch",
    .mask  = INPUT_EVENT_MASK_BTN | INPUT_EVENT_MASK_ABS,
    .event = tsp_input_event,
    .sync  = tsp_input_sync,
};

/*
 * Host keyboard -> phone keys (Linux key codes):
 *   Home=HOME, Esc/Backspace=BACK, F1/Menu=MENU, PgUp=VOL+, PgDn=VOL-,
 *   End/F7=POWER.  Also usable from QMP input-send-event (qcode home ...).
 */
static void kbd_input_event(DeviceState *dev, QemuConsole *src, QemuInputEvent *evt)
{
    int k = -1;

    if (evt->type != INPUT_EVENT_KIND_KEY) {
        return;
    }
    switch (evt->key.key) {
    case 42:
        host_shift_left = evt->key.down;
        return;
    case 54:
        host_shift_right = evt->key.down;
        return;
    case 102:                   k = K_HOME;  break;     /* KEY_HOME */
    case 1: case 14: case 158:  k = K_BACK;  break;     /* ESC, BACKSPACE, BACK */
    case 59: case 127: case 139: k = K_MENU; break;     /* F1, COMPOSE, MENU */
    case 104: case 115:         k = K_VOLUP; break;     /* PAGEUP, VOLUMEUP */
    case 109: case 114:         k = K_VOLDN; break;     /* PAGEDOWN, VOLUMEDOWN */
    case 107: case 65: case 116: k = K_POWER; break;    /* END, F7, POWER */
    }
    if (k < 0) {
        return;
    }
    if (evt->key.down) {
        if (!host_key_held[k]) {
            host_key_held[k] = true;
            if (host_shift_left || host_shift_right) {
                key_latched[k] = !key_latched[k];
            } else {
                key_keyboard[k] = true;
            }
        }
    } else {
        host_key_held[k] = false;
        key_keyboard[k] = false;
    }
    totoro_key_combined(k);
    skin_controls_refresh();
}

static const QemuInputHandler kbd_input_handler = {
    .name  = "totoro-keys",
    .mask  = INPUT_EVENT_MASK_KEY,
    .event = kbd_input_event,
};
/* ---------------------------------------------------------------- */
/* Machine                                                          */
/* ---------------------------------------------------------------- */
static struct arm_boot_info totoro_binfo;

static void totoro_controls_reset(void *opaque)
{
    int k;
    for (k = 0; k < K_NUM; k++) {
        key_keyboard[k] = key_mouse[k] = key_latched[k] = false;
        host_key_held[k] = false;
        totoro_key_combined(k);
    }
    host_shift_left = host_shift_right = false;
    mouse_key = -1;
    mouse_touch = mouse_button_down = false;
    tsp_in.down = tsp_last_down = 0;
    bcm21553_lcd_set_boot_logo(totoro_logo);
    skin_controls_refresh();
}

static void totoro_init(MachineState *machine)
{
    static const BCM21553Config cfg = {
        .fb_base = TOTORO_FB_BASE, .lcd_w = TOTORO_LCD_W, .lcd_h = TOTORO_LCD_H,
    };
    BCM21553State *soc = bcm21553_init(machine, &cfg);
    QemuInputHandlerState *hs;
    soc->cpu->totoro_fast_delay = g_strcmp0(g_getenv("TOTORO_FAST_DELAY"), "1") == 0;

    /* MAX8986 PMIC on BSC1: PM @0x08, RTC @0x68, MUIC @0x25 */
    {
        /* PM reg 0x4D (ENV2) bit0 = power-on key held: SBL powers off otherwise */
        totoro_pm = TOTORO_PM(i2c_slave_create_simple(soc->bsc[0], TYPE_TOTORO_PM, 0x08));
        totoro_pm->regs[0x4D] = 0x01;
        memset(&totoro_pm->regs[0x0b], 0xff, 3);   /* INTMSK1..3 masked at reset */
    }
    {
        /*
         * RTC time regs 0x10.. (binary): sec, min, hour, DOW (one-hot! 0
         * makes max8986_rtc_read_time() loop forever), month, year-2000, mday
         */
        BCM21553I2CRegs *rtc = BCM21553_I2C_REGS(
            i2c_slave_create_simple(soc->bsc[0], TYPE_BCM21553_I2C_REGS, 0x68));
        static const uint8_t t[7] = { 0, 0, 12, 0x10, 10, 26, 8 };
        memcpy(&rtc->regs[0x10], t, sizeof(t));
    }
    i2c_slave_create_simple(soc->bsc[0], TYPE_BCM21553_I2C_REGS, 0x25);

    bcm21553_gpio_set_input(TOTORO_SD_CD_PIN, 0);   /* SD card present */

    i2c_slave_create_simple(bcm21553_gpio_i2c_bus(soc, "tsp-i2c", TSP_SDA_PIN, TSP_SCL_PIN),
                            TYPE_TOTORO_F760, 0x20);
    i2c_slave_create_simple(bcm21553_gpio_i2c_bus(soc, "acc-i2c", ACC_SDA_PIN, ACC_SCL_PIN),
                            TYPE_TOTORO_BMA222, 0x08);
    hs = qemu_input_handler_register(soc->intc, &tsp_input_handler);
    qemu_input_handler_activate(hs);
    hs = qemu_input_handler_register(soc->intc, &kbd_input_handler);
    qemu_input_handler_activate(hs);

    totoro_load_logo();
    totoro_load_skin(totoro_skin_path);
    if (skin.px) {
        bcm21553_lcd_set_skin(skin.px, skin.w, skin.h, skin.sx, skin.sy, skin.sc);
    }

    skin_controls_refresh();
    qemu_register_reset(totoro_controls_reset, NULL);

    totoro_binfo.loader_start = TOTORO_KERNEL_BASE;
    totoro_binfo.ram_size = TOTORO_KERNEL_RAM;
    totoro_binfo.board_id = TOTORO_BOARD_ID;
    arm_load_kernel(soc->cpu, machine, &totoro_binfo);
}

static void totoro_machine_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "Samsung Galaxy Y GT-S5360 (totoro, Broadcom BCM21553)";
    mc->init = totoro_init;
    mc->ignore_memory_transaction_failures = true;
    mc->default_cpu_type = ARM_CPU_TYPE_NAME("arm1136");
    mc->default_ram_size = 512 * MiB;
    mc->default_ram_id = "totoro.sdram";
    object_class_property_add_str(oc, "boot-param",
                                  totoro_get_boot_param, totoro_set_boot_param);
    object_class_property_set_description(oc, "boot-param",
        "Read-only param.lfs for authentic startup artwork during direct kernel boot");
    object_class_property_add_str(oc, "skin", totoro_get_skin, totoro_set_skin);
    object_class_property_set_description(oc, "skin",
        "Phone skin layout file (totoro.skin): body image + key areas");
}

static const TypeInfo totoro_machine_info = {
    .name = MACHINE_TYPE_NAME("totoro"),
    .parent = TYPE_MACHINE,
    .class_init = totoro_machine_class_init,
};

static void totoro_register_types(void)
{
    type_register_static(&f760_info);
    type_register_static(&totoro_pm_info);
    type_register_static(&bma222_info);
    type_register_static(&totoro_machine_info);
}
type_init(totoro_register_types);
