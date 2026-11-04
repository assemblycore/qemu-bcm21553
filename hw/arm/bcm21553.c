/*
 * Broadcom BCM21553 SoC model (QEMU). See include/hw/arm/bcm21553.h.
 */

#include "qemu/osdep.h"
#include "net/net.h"
#include "qemu/units.h"
#include "qemu/log.h"
#include "qemu/rcu.h"
#include "qemu/main-loop.h"
#include "system/reset.h"
#include "system/runstate.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/core/sysbus.h"
#include "hw/core/boards.h"
#include "hw/core/irq.h"
#include "hw/core/ptimer.h"
#include "hw/arm/boot.h"
#include "hw/char/serial-mm.h"
#include "hw/misc/unimp.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/bitbang_i2c.h"
#include "ui/input.h"
#include "target/arm/cpregs.h"
#include "ui/console.h"
#include "hw/sd/sdhci.h"
#include "hw/sd/sd.h"
#include "system/blockdev.h"
#include "system/block-backend.h"
#include "ui/surface.h"
#include "system/address-spaces.h"
#include "system/system.h"
#include "target/arm/cpu-qom.h"
#include "qom/object.h"

#include "totoro-onenand-probe.h"
#include "totoro-ipc-probe.h"
#include "totoro-modem-transport.h"

/* ---------------------------------------------------------------- */
/* Memory map                                                       */
/* ---------------------------------------------------------------- */
#include "hw/arm/bcm21553.h"

#define BCM21553_ONENAND_BASE 0x00000000
#define BCM21553_L210_BASE    0x084A1000
#define BCM21553_INTC_BASE    0x08810000
#define BCM21553_UART_A_BASE  0x08820000
#define BCM21553_UART_B_BASE  0x08821000
#define BCM21553_UART_C_BASE  0x08822000
#define BCM21553_GPT_BASE     0x08830100

#define IRQ_UARTA  2
#define IRQ_UARTB  3
#define IRQ_GPT1   11
#define IRQ_GPT2   68
#define IRQ_UARTC  72

/* ---------------------------------------------------------------- */
/* INTC                                                             */
/* ---------------------------------------------------------------- */
#define TYPE_BCM21553_INTC "bcm21553-intc"
OBJECT_DECLARE_SIMPLE_TYPE(BCM21553IntcState, BCM21553_INTC)

#define INTC_NBANKS 3
#define INTC_NREGS  32      /* 0x80 bytes per bank */

#define INTC_IMR   0x00
#define INTC_ISR   0x04
#define INTC_ICR   0x08
#define INTC_IMSR  0x0c
#define INTC_SWIR  0x30
#define INTC_SICR  0x34

struct BCM21553IntcState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    uint32_t level[INTC_NBANKS];
    uint32_t soft[INTC_NBANKS];
    uint32_t regs[INTC_NBANKS][INTC_NREGS];
    QEMUTimer *modem_timer;
    uint8_t *modem_memory;
    TotoroModemTransport modem;
};

static const hwaddr intc_bank_base[INTC_NBANKS] = { 0x000, 0x100, 0x180 };

static uint32_t intc_status(BCM21553IntcState *s, int b)
{
    return s->level[b] | s->soft[b];
}

static void intc_update(BCM21553IntcState *s)
{
    int b, pending = 0;

    for (b = 0; b < INTC_NBANKS; b++) {
        if (intc_status(s, b) & s->regs[b][INTC_IMR / 4]) {
            pending = 1;
        }
    }
    qemu_set_irq(s->irq, pending);
}

static void intc_set_irq(void *opaque, int n, int level)
{
    BCM21553IntcState *s = opaque;
    int b = n / 32;
    uint32_t bit = 1u << (n % 32);

    if (level) {
        s->level[b] |= bit;
    } else {
        s->level[b] &= ~bit;
    }
    intc_update(s);
}

static int intc_decode(hwaddr off, hwaddr *reg)
{
    int b = off >= 0x180 ? 2 : off >= 0x100 ? 1 : 0;

    *reg = off - intc_bank_base[b];
    return *reg < INTC_NREGS * 4 ? b : -1;
}

static uint64_t intc_read(void *opaque, hwaddr off, unsigned size)
{
    BCM21553IntcState *s = opaque;
    hwaddr reg;
    int b = intc_decode(off, &reg);

    if (b < 0) {
        qemu_log_mask(LOG_UNIMP, "bcm21553-intc: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
    switch (reg) {
    case INTC_ISR:
        return intc_status(s, b);
    case INTC_IMSR:
        return intc_status(s, b) & s->regs[b][INTC_IMR / 4];
    case INTC_SWIR:
        return s->soft[b];
    default:
        return s->regs[b][reg / 4];
    }
}

static void intc_modem_step(BCM21553IntcState *s)
{
    if (s->modem_memory &&
        totoro_modem_transport_step(s->modem_memory, TM_SIZE, &s->modem)) {
        s->soft[2] |= 1u << 31; /* C2A IRQ95 */
        intc_update(s);
    }
}

static void intc_modem_tick(void *opaque)
{
    BCM21553IntcState *s = opaque;
    intc_modem_step(s);
    timer_mod(s->modem_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000000);
}

static void intc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    BCM21553IntcState *s = opaque;
    hwaddr reg;
    int b = intc_decode(off, &reg);

    if (b < 0) {
        qemu_log_mask(LOG_UNIMP, "bcm21553-intc: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        return;
    }
    switch (reg) {
    case INTC_ISR:
    case INTC_IMSR:
        break;                          /* read only */
    case INTC_ICR:
    case INTC_SICR:
        s->soft[b] &= ~(uint32_t)val;   /* write-1-to-clear */
        break;
    case INTC_SWIR:
        if (b == 2 && (val & (1u << 30))) {
            totoro_ipc_observe();
            intc_modem_step(s);
        }
        s->soft[b] |= (uint32_t)val;
        break;
    default:
        s->regs[b][reg / 4] = val;
        break;
    }
    intc_update(s);
}

static const MemoryRegionOps intc_ops = {
    .read = intc_read,
    .write = intc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void intc_init(Object *obj)
{
    BCM21553IntcState *s = BCM21553_INTC(obj);

    memory_region_init_io(&s->iomem, obj, &intc_ops, s, TYPE_BCM21553_INTC, 0x200);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    qdev_init_gpio_in(DEVICE(obj), intc_set_irq, INTC_NBANKS * 32);
}

static const TypeInfo intc_info = {
    .name = TYPE_BCM21553_INTC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM21553IntcState),
    .instance_init = intc_init,
};

/* ---------------------------------------------------------------- */
/* GPT                                                              */
/* ---------------------------------------------------------------- */
#define TYPE_BCM21553_GPT "bcm21553-gpt"
OBJECT_DECLARE_SIMPLE_TYPE(BCM21553GptState, BCM21553_GPT)

#define GPT_NUM 6
#define GPT_ISR 0x00
#define GPT_CSR_EN          (1u << 31)
#define GPT_CSR_CLKSEL      (1u << 30)
#define GPT_CSR_INT_EN      (1u << 27)
#define GPT_CSR_INT2_ASSIGN (1u << 26)
#define GPT_CSR_CLKSEL1     (1u << 25)
#define GPT_CSR_INT_FLAG    (1u << 0)

typedef struct BCM21553GptState BCM21553GptState;

typedef struct {
    BCM21553GptState *s;
    int idx;
} GptTimerRef;

struct BCM21553GptState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq[2];
    ptimer_state *timer[GPT_NUM];
    GptTimerRef ref[GPT_NUM];
    uint32_t csr[GPT_NUM];
    uint32_t reload[GPT_NUM];
    uint32_t freq[GPT_NUM];
};

static void gpt_update(BCM21553GptState *s)
{
    int i, line[2] = { 0, 0 };

    for (i = 0; i < GPT_NUM; i++) {
        if ((s->csr[i] & GPT_CSR_INT_FLAG) && (s->csr[i] & GPT_CSR_INT_EN)) {
            line[(s->csr[i] & GPT_CSR_INT2_ASSIGN) ? 1 : 0] = 1;
        }
    }
    qemu_set_irq(s->irq[0], line[0]);
    qemu_set_irq(s->irq[1], line[1]);
}

static void gpt_tick(void *opaque)
{
    GptTimerRef *r = opaque;

    r->s->csr[r->idx] |= GPT_CSR_INT_FLAG;
    gpt_update(r->s);
}

static uint32_t gpt_csr_freq(uint32_t csr)
{
    if (csr & GPT_CSR_CLKSEL) {
        return 26000000;
    }
    if (csr & GPT_CSR_CLKSEL1) {
        return 1000000;
    }
    return 32768;
}

static uint64_t gpt_read(void *opaque, hwaddr off, unsigned size)
{
    BCM21553GptState *s = opaque;
    int i;
    uint32_t v = 0;

    if (off == GPT_ISR) {
        for (i = 0; i < GPT_NUM; i++) {
            if (s->csr[i] & GPT_CSR_INT_FLAG) {
                v |= 1u << i;
            }
        }
        return v;
    }
    i = (off - 4) / 0x10;
    if (i >= GPT_NUM) {
        qemu_log_mask(LOG_UNIMP, "bcm21553-gpt: read 0x%" HWADDR_PRIx "\n", off);
        return 0;
    }
    switch ((off - 4) % 0x10) {
    case 0x0:
        return s->csr[i];
    case 0x4:
        return s->reload[i];
    case 0x8:
        return ptimer_get_count(s->timer[i]);
    }
    return 0;
}

static void gpt_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    BCM21553GptState *s = opaque;
    int i;
    uint32_t old, flag, freq;

    if (off == GPT_ISR) {
        return;
    }
    i = (off - 4) / 0x10;
    if (i >= GPT_NUM) {
        qemu_log_mask(LOG_UNIMP, "bcm21553-gpt: write 0x%" HWADDR_PRIx
                      " = 0x%" PRIx64 "\n", off, val);
        return;
    }
    ptimer_transaction_begin(s->timer[i]);
    switch ((off - 4) % 0x10) {
    case 0x0:                               /* CSR */
        old = s->csr[i];
        flag = (val & GPT_CSR_INT_FLAG) ? 0 : (old & GPT_CSR_INT_FLAG);
        s->csr[i] = ((uint32_t)val & ~GPT_CSR_INT_FLAG) | flag;
        freq = gpt_csr_freq(s->csr[i]);
        if (freq != s->freq[i]) {
            s->freq[i] = freq;
            ptimer_set_freq(s->timer[i], freq);
        }
        if ((val & GPT_CSR_EN) && !(old & GPT_CSR_EN)) {
            ptimer_set_limit(s->timer[i], s->reload[i] ? s->reload[i] : 1, 1);
            ptimer_run(s->timer[i], 0);
        } else if (!(val & GPT_CSR_EN) && (old & GPT_CSR_EN)) {
            ptimer_stop(s->timer[i]);
        }
        break;
    case 0x4:                               /* RELOAD */
        s->reload[i] = val;
        ptimer_set_limit(s->timer[i], val ? val : 1, 1);
        break;
    case 0x8:                               /* VALUE */
        ptimer_set_count(s->timer[i], val);
        break;
    }
    ptimer_transaction_commit(s->timer[i]);
    gpt_update(s);
}

static const MemoryRegionOps gpt_ops = {
    .read = gpt_read,
    .write = gpt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 4, .max_access_size = 4 },
};

static void gpt_realize(DeviceState *dev, Error **errp)
{
    BCM21553GptState *s = BCM21553_GPT(dev);
    int i;

    for (i = 0; i < GPT_NUM; i++) {
        s->ref[i].s = s;
        s->ref[i].idx = i;
        s->timer[i] = ptimer_init(gpt_tick, &s->ref[i],
                                  PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                                  PTIMER_POLICY_CONTINUOUS_TRIGGER |
                                  PTIMER_POLICY_NO_IMMEDIATE_TRIGGER |
                                  PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                                  PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);
        ptimer_transaction_begin(s->timer[i]);
        s->freq[i] = 32768;
        ptimer_set_freq(s->timer[i], 32768);
        ptimer_set_limit(s->timer[i], 0xffffffff, 1);
        ptimer_transaction_commit(s->timer[i]);
    }
}

static void gpt_init(Object *obj)
{
    BCM21553GptState *s = BCM21553_GPT(obj);

    memory_region_init_io(&s->iomem, obj, &gpt_ops, s, TYPE_BCM21553_GPT, 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[0]);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq[1]);
}

static void gpt_class_init(ObjectClass *oc, const void *data)
{
    DEVICE_CLASS(oc)->realize = gpt_realize;
}

static const TypeInfo gpt_info = {
    .name = TYPE_BCM21553_GPT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM21553GptState),
    .instance_init = gpt_init,
    .class_init = gpt_class_init,
};

/* ---------------------------------------------------------------- */
/* BSC (I2C master, "bcm1161" driver)                               */
/* ---------------------------------------------------------------- */
#define TYPE_BCM21553_BSC "bcm21553-bsc"
OBJECT_DECLARE_SIMPLE_TYPE(BCM21553BscState, BCM21553_BSC)

#define BSC_CS      0x00
#define BSC_DAT     0x08
#define BSC_FCR     0x1c
#define BSC_IER     0x24
#define BSC_ISR     0x28
#define BSC_SFTRST  0x40
#define BSC_CS_EN       0x01
#define BSC_CS_CMDMASK  0x06
#define BSC_CS_ACK      0x08
#define BSC_CS_SCL      0x40
#define BSC_CS_SDA      0x80
#define BSC_ISR_NOACK    0x01
#define BSC_ISR_SES_DONE 0x08

struct BCM21553BscState {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;
    uint8_t regs[0x80];
    bool addr_phase;
    bool in_xfer;
    bool nack;
};

static void bsc_update(BCM21553BscState *s)
{
    qemu_set_irq(s->irq, (s->regs[BSC_ISR] & s->regs[BSC_IER] & 0x0f) != 0);
}

static void bsc_done(BCM21553BscState *s, bool nack)
{
    s->regs[BSC_ISR] |= BSC_ISR_SES_DONE | (nack ? BSC_ISR_NOACK : 0);
}

static void bsc_cmd(BCM21553BscState *s, uint8_t cs)
{
    uint8_t d;

    switch (cs & BSC_CS_CMDMASK) {
    case 0x02:                      /* START / RESTART */
        s->addr_phase = true;
        bsc_done(s, false);
        break;
    case 0x04:                      /* STOP */
        if (s->in_xfer) {
            i2c_end_transfer(s->bus);
            s->in_xfer = false;
        }
        s->addr_phase = false;
        bsc_done(s, false);
        break;
    case 0x06:                      /* READ byte */
        d = s->in_xfer ? i2c_recv(s->bus) : 0xff;
        if (s->in_xfer && (cs & BSC_CS_ACK)) {
            i2c_nack(s->bus);
        }
        s->regs[BSC_DAT] = d;
        bsc_done(s, false);
        break;
    }
}

static void bsc_send(BCM21553BscState *s, uint8_t v)
{
    bool nack;

    if (s->addr_phase) {
        nack = i2c_start_transfer(s->bus, v >> 1, v & 1) != 0;
        s->in_xfer = !nack;
        s->addr_phase = false;
    } else if (s->in_xfer) {
        nack = i2c_send(s->bus, v) != 0;
    } else {
        nack = true;
    }
    s->nack = nack;
    bsc_done(s, nack);
}

static uint64_t bsc_read(void *opaque, hwaddr off, unsigned size)
{
    BCM21553BscState *s = opaque;

    switch (off) {
    case BSC_CS:
        return (s->regs[BSC_CS] & ~(BSC_CS_ACK | BSC_CS_SDA | BSC_CS_SCL)) |
               BSC_CS_SDA | BSC_CS_SCL | (s->nack ? BSC_CS_ACK : 0);
    case BSC_SFTRST:
        return 0;
    case BSC_FCR:
        return s->regs[BSC_FCR] & ~0x87;
    default:
        return off < sizeof(s->regs) ? s->regs[off] : 0;
    }
}

static void bsc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    BCM21553BscState *s = opaque;

    switch (off) {
    case BSC_CS:
        s->regs[BSC_CS] = val;
        if ((val & BSC_CS_EN) && (val & BSC_CS_CMDMASK)) {
            bsc_cmd(s, val);
        }
        break;
    case BSC_DAT:
        bsc_send(s, val);
        break;
    case BSC_ISR:
        s->regs[BSC_ISR] &= ~(uint8_t)val;
        break;
    case BSC_SFTRST:
        if (val & 1) {
            if (s->in_xfer) {
                i2c_end_transfer(s->bus);
            }
            s->in_xfer = s->addr_phase = s->nack = false;
            s->regs[BSC_ISR] = 0;
        }
        break;
    default:
        if (off < sizeof(s->regs)) {
            s->regs[off] = val;
        }
        break;
    }
    bsc_update(s);
}

static const MemoryRegionOps bsc_ops = {
    .read = bsc_read,
    .write = bsc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = { .min_access_size = 1, .max_access_size = 4 },
    .impl = { .min_access_size = 1, .max_access_size = 1 },
};

static void bsc_init(Object *obj)
{
    BCM21553BscState *s = BCM21553_BSC(obj);

    memory_region_init_io(&s->iomem, obj, &bsc_ops, s, TYPE_BCM21553_BSC, 0x80);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);
    s->bus = i2c_init_bus(DEVICE(obj), "i2c");
}

static const TypeInfo bsc_info = {
    .name = TYPE_BCM21553_BSC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BCM21553BscState),
    .instance_init = bsc_init,
};

/* ---------------------------------------------------------------- */
/* Generic I2C register-file slave (stub for PMIC etc.)                */
/* ---------------------------------------------------------------- */

static int i2cregs_event(I2CSlave *i2c, enum i2c_event e)
{
    BCM21553I2CRegs *s = BCM21553_I2C_REGS(i2c);

    if (e == I2C_START_SEND) {
        s->first = true;
    }
    return 0;
}

static int i2cregs_send(I2CSlave *i2c, uint8_t d)
{
    BCM21553I2CRegs *s = BCM21553_I2C_REGS(i2c);

    if (s->first) {
        s->ptr = d;
        s->first = false;
    } else {
        qemu_log_mask(LOG_UNIMP, "i2c-regs@0x%02x: write reg 0x%02x = 0x%02x\n",
                      i2c->address, s->ptr, d);
        s->regs[s->ptr++] = d;
    }
    return 0;
}

static uint8_t i2cregs_recv(I2CSlave *i2c)
{
    BCM21553I2CRegs *s = BCM21553_I2C_REGS(i2c);

    qemu_log_mask(LOG_UNIMP, "i2c-regs@0x%02x: read reg 0x%02x = 0x%02x\n",
                  i2c->address, s->ptr, s->regs[s->ptr]);
    return s->regs[s->ptr++];
}

static void i2cregs_class_init(ObjectClass *oc, const void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(oc);

    k->event = i2cregs_event;
    k->send = i2cregs_send;
    k->recv = i2cregs_recv;
}

static const TypeInfo i2cregs_info = {
    .name = TYPE_BCM21553_I2C_REGS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BCM21553I2CRegs),
    .class_init = i2cregs_class_init,
};

/* ---------------------------------------------------------------- */
/* GPIO @0x088CE000 (plat-bcmap/gpio.c), 64 pins                     */
/* IOTR 0x00+: 2 bits/pin (2 = out); GPOR 0x10/0x14; GPIPS 0x18/0x1c */
/* GPIMR 0x30/0x34 (1 = unmasked); GPICR 0x38/0x3c (W1C);            */
/* GPITR 0x40+; GPISR 0x50/0x54. Port IRQ chained on INTC 6.         */
/* Inputs read as pulled up unless the board overrides the level.    */
/* Boards can hang Linux i2c-gpio (open-drain by direction) buses on */
/* any SDA/SCL pin pair via bcm21553_gpio_i2c_bus().                 */
/* ---------------------------------------------------------------- */
#define BCM21553_GPIO_BASE 0x088CE000
#define BCM21553_GPIO_NI2C 4

static uint32_t gpio_regs[0x100 / 4];
static uint32_t gpio_isr[2];
static uint64_t gpio_in_level = ~0ULL;
static qemu_irq gpio_parent_irq;

/*
 * Keypad matrix @0x088CE080 (= GPIO block +0x80), IRQ 5 (bcm_keypad.c):
 * KPCR 0, KPIOR 4, KPEMR0-3 0x10-0x1c, KPSSR0/1 0x20/0x24 (status,
 * active low: 0 = pressed), KPIMR0/1 0x30/0x34 (1 = enabled),
 * KPICR0/1 0x38/0x3c (W1C), KPISR0/1 0x40/0x44. bit = col + (row%4)*8,
 * rows 0-3 in word 0, rows 4-7 in word 1.
 */
#define KP_OFF  0x80
#define KP_SIZE 0x48
static uint32_t kp_regs[KP_SIZE / 4];
static uint32_t kp_ssr[2] = { ~0u, ~0u };
static uint32_t kp_isr[2];
static qemu_irq kp_irq;

static void kp_update_irq(void)
{
    uint32_t p = (kp_isr[0] & kp_regs[0x30 / 4]) | (kp_isr[1] & kp_regs[0x34 / 4]);

    if (kp_irq) {
        qemu_set_irq(kp_irq, p != 0);
    }
}

void bcm21553_keypad_set(int row, int col, bool pressed)
{
    int w = (row / 4) & 1;
    uint32_t bit = 1u << ((col & 7) + (row % 4) * 8);

    if (pressed) {
        kp_ssr[w] &= ~bit;
    } else {
        kp_ssr[w] |= bit;
    }
    kp_isr[w] |= bit;
    kp_update_irq();
}

static uint64_t kp_read(hwaddr off)
{
    switch (off) {
    case 0x20: case 0x24:
        return kp_ssr[(off - 0x20) / 4];
    case 0x40: case 0x44:
        return kp_isr[(off - 0x40) / 4];
    default:
        return kp_regs[off / 4];
    }
}

static void kp_write(hwaddr off, uint32_t val)
{
    if (off == 0x38 || off == 0x3c) {
        kp_isr[(off - 0x38) / 4] &= ~val;
    } else {
        kp_regs[off / 4] = val;
    }
    kp_update_irq();
}

typedef struct {
    bitbang_i2c_interface bb;
    I2CBus *bus;
    int sda, scl, dev_out;
} BCM21553GpioI2C;

static BCM21553GpioI2C gi2c[BCM21553_GPIO_NI2C];
static int gi2c_n;

static bool gpio_is_out(int pin)
{
    return ((gpio_regs[(0x00 + (pin >> 4) * 4) / 4] >> ((pin & 15) * 2)) & 3) == 2;
}

static void gpio_update_irq(void)
{
    uint32_t p = (gpio_isr[0] & gpio_regs[0x30 / 4]) |
                 (gpio_isr[1] & gpio_regs[0x34 / 4]);
    if (gpio_parent_irq) {
        qemu_set_irq(gpio_parent_irq, p != 0);
    }
}

void bcm21553_gpio_pend(int pin)
{
    gpio_isr[pin >> 5] |= 1u << (pin & 31);
    gpio_update_irq();
}

static void bcm21553_wifi_hostwake(void *opaque, int n, int level)
{
    bcm21553_gpio_set_input(18, !level); /* OOB host-wake is active low */
    if (level) bcm21553_gpio_pend(18);
}

void bcm21553_gpio_set_input(int pin, int level)
{
    if (level) {
        gpio_in_level |= 1ULL << pin;
    } else {
        gpio_in_level &= ~(1ULL << pin);
    }
}

/* level the SoC drives: output-low pulls the wire down, input releases */
static int gpio_master_level(int pin)
{
    int out = (gpio_regs[(0x10 + (pin >> 5) * 4) / 4] >> (pin & 31)) & 1;
    return gpio_is_out(pin) ? out : 1;
}

static void gpio_i2c_update(void)
{
    int i;

    for (i = 0; i < gi2c_n; i++) {
        BCM21553GpioI2C *g = &gi2c[i];
        g->dev_out = bitbang_i2c_set(&g->bb, BITBANG_I2C_SCL, gpio_master_level(g->scl));
        g->dev_out = bitbang_i2c_set(&g->bb, BITBANG_I2C_SDA, gpio_master_level(g->sda));
    }
}

static int gpio_pin_level(int pin)
{
    int i;

    for (i = 0; i < gi2c_n; i++) {
        if (pin == gi2c[i].sda) {
            return gpio_master_level(pin) & gi2c[i].dev_out;
        }
        if (pin == gi2c[i].scl) {
            return gpio_master_level(pin);
        }
    }
    if (gpio_is_out(pin)) {
        return (gpio_regs[(0x10 + (pin >> 5) * 4) / 4] >> (pin & 31)) & 1;
    }
    return (gpio_in_level >> pin) & 1;
}

I2CBus *bcm21553_gpio_i2c_bus(BCM21553State *s, const char *name, int sda, int scl)
{
    BCM21553GpioI2C *g;

    assert(gi2c_n < BCM21553_GPIO_NI2C);
    g = &gi2c[gi2c_n++];
    g->sda = sda;
    g->scl = scl;
    g->dev_out = 1;
    g->bus = i2c_init_bus(s->intc, name);
    bitbang_i2c_init(&g->bb, g->bus);
    return g->bus;
}

static uint64_t gpio_read(void *opaque, hwaddr off, unsigned size)
{
    if (off >= KP_OFF && off < KP_OFF + KP_SIZE) {
        return kp_read(off - KP_OFF);
    }
    if (off == 0x18 || off == 0x1c) {           /* GPIPS: pin state */
        int bank = (off - 0x18) / 4, i;
        uint32_t v = 0, out = gpio_regs[(0x10 / 4) + bank];

        for (i = 0; i < 32; i++) {
            int pin = bank * 32 + i;
            v |= gpio_pin_level(pin) << i; (void)out;
        }
        return v;
    }
    if (off == 0x50 || off == 0x54) {           /* GPISR */
        return gpio_isr[(off - 0x50) / 4];
    }
    return off < sizeof(gpio_regs) ? gpio_regs[off / 4] : 0;
}

static void gpio_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (off >= KP_OFF && off < KP_OFF + KP_SIZE) {
        kp_write(off - KP_OFF, val);
        return;
    }
    if (off == 0x38 || off == 0x3c) {           /* GPICR: write-1-clear */
        gpio_isr[(off - 0x38) / 4] &= ~(uint32_t)val;
        gpio_update_irq();
        return;
    }
    if (off < sizeof(gpio_regs)) {
        gpio_regs[off / 4] = val;
    }
    if (off < 0x18) {                           /* IOTR/GPOR: bit-bang lines */
        gpio_i2c_update();
    } else if (off == 0x30 || off == 0x34) {    /* GPIMR */
        gpio_update_irq();
    }
}

static const MemoryRegionOps gpio_ops = {
    .read = gpio_read,
    .write = gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* CLKPWR power islands @0x08140148..0x08140157: bit2 = POWER_OK */
static uint32_t pwrisl_regs[4];

static uint64_t pwrisl_read(void *opaque, hwaddr off, unsigned size)
{
    /* bit0: 1 = POWER_OFF, 0 = POWER_ON; bit2 POWER_OK mirrors "on" */
    uint32_t v = pwrisl_regs[(off >> 2) & 3] & ~(1u << 2);
    return (v & 1) ? v : (v | (1u << 2));
}

static void pwrisl_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    pwrisl_regs[(off >> 2) & 3] = val;
}

static const MemoryRegionOps pwrisl_ops = {
    .read = pwrisl_read,
    .write = pwrisl_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---------------------------------------------------------------- */
/* LCDC @0x08030000 (parallel Z80/DBI bus to the panel)              */
/* Commands/data are accepted instantly; TE toggles on every read.   */
/* ---------------------------------------------------------------- */
#define BCM21553_LCDC_BASE 0x08030000
#define LCDC_CMDR    0x00
#define LCDC_DATR    0x04
#define LCDC_RREQ    0x08
#define LCDC_STATUS  0x1c
#define LCDC_ST_FIFO_EMPTY (1u << 31)
#define LCDC_ST_READ_READY (1u << 14)
#define LCDC_ST_TE         (1u << 11)

static uint32_t lcdc_regs[0x40 / 4];
static uint32_t lcdc_te;
static uint64_t lcdc_cmds, lcdc_data;
/* DCS panel GRAM, filled by SBL's programmed-I/O transfers. */
static uint32_t lcdc_gram[240 * 320];
static bool lcdc_gram_valid;
static uint8_t lcdc_command, lcdc_args[4], lcdc_arg_count;
static uint16_t lcdc_x0, lcdc_y0, lcdc_x1 = 239, lcdc_y1 = 319;
static uint16_t lcdc_x, lcdc_y;
/* panel state from DCS: 0x28 DISPOFF / 0x10 SLPIN blank, 0x29 / 0x11 unblank */
static bool lcdc_panel_on = true;
static const uint32_t *bcm21553_boot_logo;
static bool bcm21553_boot_logo_active;

static uint64_t lcdc_read(void *opaque, hwaddr off, unsigned size)
{
    switch (off) {
    case LCDC_STATUS:
        lcdc_te ^= LCDC_ST_TE;
        return LCDC_ST_FIFO_EMPTY | LCDC_ST_READ_READY | lcdc_te;
    case LCDC_DATR:
    case LCDC_RREQ:
        return 0;
    default:
        return off < sizeof(lcdc_regs) ? lcdc_regs[off / 4] : 0;
    }
}

static void lcdc_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    switch (off) {
    case LCDC_CMDR:
        lcdc_command = val & 0xff;
        lcdc_arg_count = 0;
        if (lcdc_command == 0x2c) {
            lcdc_x = lcdc_x0;
            lcdc_y = lcdc_y0;
        }
        switch (val & 0xff) {
        case 0x28: case 0x10:
            lcdc_panel_on = false;
            bcm21553_boot_logo_active = false;
            break;
        case 0x29: case 0x11:
            lcdc_panel_on = true;
            break;
        }
        if (lcdc_cmds++ < 200) {
            qemu_log_mask(LOG_UNIMP, "lcdc: cmd 0x%02" PRIx64 "\n", val);
        }
        break;
    case LCDC_DATR:
        lcdc_data++;
        if ((lcdc_command == 0x2a || lcdc_command == 0x2b) &&
            lcdc_arg_count < 4) {
            lcdc_args[lcdc_arg_count++] = val & 0xff;
            if (lcdc_arg_count == 4) {
                uint16_t first = (lcdc_args[0] << 8) | lcdc_args[1];
                uint16_t last = (lcdc_args[2] << 8) | lcdc_args[3];
                if (lcdc_command == 0x2a) {
                    lcdc_x0 = first;
                    lcdc_x1 = last;
                } else {
                    lcdc_y0 = first;
                    lcdc_y1 = last;
                }
            }
        } else if (lcdc_command == 0x2c &&
                   lcdc_x0 <= lcdc_x1 && lcdc_y0 <= lcdc_y1) {
            if (lcdc_x < 240 && lcdc_y < 320) {
                lcdc_gram[lcdc_y * 240 + lcdc_x] = 0xff000000 | (val & 0xffffff);
                lcdc_gram_valid = true;
            }
            if (lcdc_x == lcdc_x1) {
                lcdc_x = lcdc_x0;
                lcdc_y = lcdc_y == lcdc_y1 ? lcdc_y0 : lcdc_y + 1;
            } else {
                lcdc_x++;
            }
        }
        break;
    default:
        if (off < sizeof(lcdc_regs)) {
            lcdc_regs[off / 4] = val;
        }
        break;
    }
}

static void lcdc_reset(void *opaque)
{
    memset(lcdc_regs, 0, sizeof(lcdc_regs));
    memset(lcdc_gram, 0, sizeof(lcdc_gram));
    lcdc_gram_valid = false;
    lcdc_command = lcdc_arg_count = 0;
    lcdc_x0 = lcdc_y0 = lcdc_x = lcdc_y = 0;
    lcdc_x1 = 239;
    lcdc_y1 = 319;
    lcdc_panel_on = true;
    lcdc_te = 0;
}

static const MemoryRegionOps lcdc_ops = {
    .read = lcdc_read,
    .write = lcdc_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---------------------------------------------------------------- */
/* OTP (fuses) @0x0C0C9000: commands complete instantly, all rows 0  */
/* ---------------------------------------------------------------- */
#define BCM21553_OTP_BASE 0x0C0C9000
#define OTP_REG_STATUS 0x10
#define OTP_ST_CMD_DONE (1u << 0)
#define OTP_ST_DATA_RDY (1u << 1)

static uint32_t otp_regs[0x40 / 4];

static uint64_t otp_read(void *opaque, hwaddr off, unsigned size)
{
    if (off == OTP_REG_STATUS) {
        return OTP_ST_CMD_DONE | OTP_ST_DATA_RDY;
    }
    if (off == 0x14) {          /* RDATA */
        return 0;
    }
    return off < sizeof(otp_regs) ? otp_regs[off / 4] : 0;
}

static void otp_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    if (off < sizeof(otp_regs) && off != OTP_REG_STATUS) {
        otp_regs[off / 4] = val;
    }
}

static const MemoryRegionOps otp_ops = {
    .read = otp_read,
    .write = otp_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

/* ---------------------------------------------------------------- */
/* LCD panel display: scan out the kernel framebuffer (lcdc.c        */
/* lcd_alloc_fb: phys 0x93500000, 240x320, 32 bpp) into a QEMU       */
/* graphic console.                                                   */
/* ---------------------------------------------------------------- */

static QemuConsole *bcm21553_lcd_con;
static uint32_t *bcm21553_lcd_buf;
static hwaddr bcm21553_fb_base;
static int bcm21553_lcd_w, bcm21553_lcd_h;

/* optional board skin: background image with the panel drawn at (x,y)*scale */
static const uint32_t *bcm21553_skin_px;
static int bcm21553_skin_w, bcm21553_skin_h, bcm21553_skin_x, bcm21553_skin_y;
static int bcm21553_skin_sc = 1;
static bool bcm21553_lcd_full = true;
static uint32_t *bcm21553_lcd_previous;

void bcm21553_lcd_set_boot_logo(const uint32_t *pixels)
{
    bcm21553_boot_logo = pixels;
    bcm21553_boot_logo_active = pixels != NULL;
    bcm21553_lcd_full = true;
}

void bcm21553_lcd_set_skin(const uint32_t *px, int w, int h, int x, int y, int scale)
{
    bcm21553_skin_px = px;
    bcm21553_skin_w = w;
    bcm21553_skin_h = h;
    bcm21553_skin_x = x;
    bcm21553_skin_y = y;
    bcm21553_skin_sc = scale > 0 ? scale : 1;
    bcm21553_lcd_full = true;
}

/*
 * Bus-master reads of the software LCD/V3D models.  A zero/garbage GPU
 * address (e.g. an untextured shader's texture uniform) used to read up to
 * 8 MiB from the OneNAND unimplemented-device window at phys 0, 8 bytes per
 * MMIO dispatch, on every primitive.  Unimplemented devices return zero, so
 * zero-filling this known read-as-zero range keeps output identical while avoiding the
 * per-word MMIO path.  RAM ranges are still read through the address space.
 */
static void bcm21553_dma_read(hwaddr addr, MemTxAttrs attrs, void *buf, hwaddr len)
{
    uint8_t *p = buf;
    while (len) {
        hwaddr xlat, l = len;
        MemoryRegion *mr;
        bool zero_stub;
        RCU_READ_LOCK_GUARD();
        mr = address_space_translate(&address_space_memory, addr, &xlat, &l,
                                     false, attrs);
        if (!l) {
            l = len;
        }
        zero_stub = !memory_region_is_ram(mr) &&
                    !strcmp(memory_region_name(mr), "bcm21553-onenand");
        if (zero_stub) {
            memset(p, 0, l);
        } else {
            address_space_read(&address_space_memory, addr, attrs, p, l);
        }
        p += l; addr += l; len -= l;
    }
}

/* True unless the entire range is the known read-as-zero OneNAND stub. */
static bool bcm21553_dma_needs_read(hwaddr addr, hwaddr len)
{
    while (len) {
        hwaddr xlat, l = len;
        MemoryRegion *mr;
        RCU_READ_LOCK_GUARD();
        mr = address_space_translate(&address_space_memory, addr, &xlat, &l,
                                     false, MEMTXATTRS_UNSPECIFIED);
        if (memory_region_is_ram(mr) ||
            strcmp(memory_region_name(mr), "bcm21553-onenand")) {
            return true;
        }
        if (!l) {
            l = len;
        }
        addr += l; len -= l;
    }
    return false;
}

/*
 * Shared read-only zero texture for primitives whose texture uniform points
 * at the read-as-zero OneNAND stub.  The previous code allocated, zero-filled and "read" up to
 * 8 MiB per primitive; sampling an all-zero buffer is byte-for-byte identical.
 */
static uint8_t *v3d_zero_tex;

static bool bcm21553_lcd_update(void *opaque)
{
    bool sk = bcm21553_skin_px != NULL;
    int cw = sk ? bcm21553_skin_w : bcm21553_lcd_w;
    int ch = sk ? bcm21553_skin_h : bcm21553_lcd_h;
    int sc = sk ? bcm21553_skin_sc : 1;
    int ox = sk ? bcm21553_skin_x : 0, oy = sk ? bcm21553_skin_y : 0;
    DisplaySurface *s = qemu_console_surface(bcm21553_lcd_con);
    uint8_t *d;
    int stride, y, x, i, j;

    if (!s || surface_width(s) != cw || surface_height(s) != ch) {
        qemu_console_resize(bcm21553_lcd_con, cw, ch);
        s = qemu_console_surface(bcm21553_lcd_con);
        bcm21553_lcd_full = true;
    }
    d = surface_data(s);
    stride = surface_stride(s);
    if (bcm21553_lcd_full && sk) {
        for (y = 0; y < ch; y++) {
            memcpy(d + y * stride, bcm21553_skin_px + (size_t)y * cw, (size_t)cw * 4);
        }
    }
    if (lcdc_panel_on && lcdc_gram_valid && !(lcdc_regs[0x18 / 4] & (1u << 31)) &&
        bcm21553_lcd_w == 240 && bcm21553_lcd_h == 320) {
        memcpy(bcm21553_lcd_buf, lcdc_gram, sizeof(lcdc_gram));
    } else if (lcdc_panel_on) {
        bcm21553_dma_read(bcm21553_fb_base,
                           MEMTXATTRS_UNSPECIFIED, bcm21553_lcd_buf,
                           (size_t)bcm21553_lcd_w * bcm21553_lcd_h * 4);
    } else {
        memset(bcm21553_lcd_buf, 0, (size_t)bcm21553_lcd_w * bcm21553_lcd_h * 4);
    }
    if (bcm21553_boot_logo_active) {
        bool guest_frame = false;
        size_t pixel;
        for (pixel = 0; pixel < (size_t)bcm21553_lcd_w * bcm21553_lcd_h; pixel++) {
            if (bcm21553_lcd_buf[pixel] & 0x00ffffff) {
                guest_frame = true;
                break;
            }
        }
        if (guest_frame || !lcdc_panel_on) {
            bcm21553_boot_logo_active = false;
        } else {
            memcpy(bcm21553_lcd_buf, bcm21553_boot_logo,
                   (size_t)bcm21553_lcd_w * bcm21553_lcd_h * 4);
        }
    }
    if (!bcm21553_lcd_previous) {
        bcm21553_lcd_previous = g_new0(uint32_t,
                                       bcm21553_lcd_w * bcm21553_lcd_h);
    }
    if (!bcm21553_lcd_full &&
        !memcmp(bcm21553_lcd_buf, bcm21553_lcd_previous,
                (size_t)bcm21553_lcd_w * bcm21553_lcd_h * 4)) {
        return true;
    }
    memcpy(bcm21553_lcd_previous, bcm21553_lcd_buf,
           (size_t)bcm21553_lcd_w * bcm21553_lcd_h * 4);
    for (y = 0; y < bcm21553_lcd_h; y++) {
        const uint32_t *src = &bcm21553_lcd_buf[y * bcm21553_lcd_w];
        for (j = 0; j < sc; j++) {
            uint32_t *dst = (uint32_t *)(d + (oy + y * sc + j) * stride) + ox;
            for (x = 0; x < bcm21553_lcd_w; x++) {
                /* guest: little-endian ARGB8888 == host x8r8g8b8 */
                uint32_t v = src[x] | 0xff000000u;
                for (i = 0; i < sc; i++) {
                    dst[x * sc + i] = v;
                }
            }
        }
    }
    if (bcm21553_lcd_full) {
        qemu_console_update(bcm21553_lcd_con, 0, 0, cw, ch);
    } else {
        qemu_console_update(bcm21553_lcd_con, ox, oy,
                            bcm21553_lcd_w * sc, bcm21553_lcd_h * sc);
    }
    bcm21553_lcd_full = false;
    return true;
}

static void bcm21553_lcd_invalidate(void *opaque)
{
    bcm21553_lcd_full = true;
}

static const GraphicHwOps bcm21553_lcd_ops = {
    .invalidate = bcm21553_lcd_invalidate,
    .gfx_update = bcm21553_lcd_update,
};

/* ------------------------------------------------------------------ */
/* V3D (VideoCore IV 3D) @0x08950000, IRQ 16                           */
/* Stage 1: register model + job completion + control-list dumps.      */
/* ------------------------------------------------------------------ */

typedef struct BCM21553V3D {
    MemoryRegion mr;
    qemu_irq irq;
    uint32_t r[0x1000 / 4];
    uint32_t intctl, intena, dbqitc, bfc, rfc;
    unsigned jobs;
} BCM21553V3D;

static BCM21553V3D bcm21553_v3d;

static const uint8_t v3d_pkt_len[256] = {
    [0] = 1, [1] = 1, [4] = 1, [5] = 1, [6] = 1, [7] = 1, [8] = 1,
    [16] = 5, [17] = 5, [18] = 1, [24] = 1, [25] = 1, [26] = 5, [27] = 5,
    [28] = 7, [29] = 7, [32] = 14, [33] = 10, [34] = 1, [35] = 1, [36] = 1, [48] = 2, [56] = 2,
    [64] = 5, [65] = 5, [66] = 5, [96] = 4, [97] = 5, [98] = 5, [99] = 5,
    [100] = 3, [101] = 5, [102] = 9, [103] = 5, [104] = 9, [105] = 9,
    [106] = 9, [112] = 16, [113] = 11, [114] = 14, [115] = 3,
};

static void v3d_update_irq(BCM21553V3D *s)
{
    qemu_set_irq(s->irq, ((s->intctl & s->intena & 0xf) != 0) || s->dbqitc);
}

static void v3d_dump_words(const char *tag, uint32_t a, int n)
{
    uint32_t w[32];
    int i;
    if (n > 32) { n = 32; }
    bcm21553_dma_read(a, MEMTXATTRS_UNSPECIFIED, w, n * 4);
    fprintf(stderr, "v3d:   %s @%08x:", tag, a);
    for (i = 0; i < n; i++) {
        union { uint32_t u; float f; } c = { .u = w[i] };
        fprintf(stderr, " %08x(%g)", w[i], c.f);
    }
    fprintf(stderr, "\n");
}

static void v3d_dump_shader(uint32_t a)
{
    uint8_t rec[36 + 8 * 8];
    uint32_t base = a & ~0xfu;
    int na = a & 7, i;
    if (!na) { na = 8; }
    bcm21553_dma_read(base, MEMTXATTRS_UNSPECIFIED, rec, 36 + 8 * na);
    fprintf(stderr, "v3d:  shader rec %08x flags=%04x fsvary=%u fs=%08x fsu=%08x vsnu=%u vssel=%02x vssz=%u vs=%08x vsu=%08x cs=%08x csu=%08x\n",
            base, lduw_le_p(rec), rec[3], ldl_le_p(rec + 4), ldl_le_p(rec + 8),
            lduw_le_p(rec + 12), rec[14], rec[15], ldl_le_p(rec + 16), ldl_le_p(rec + 20),
            ldl_le_p(rec + 28), ldl_le_p(rec + 32));
    for (i = 0; i < na; i++) {
        uint8_t *r = rec + 36 + 8 * i;
        fprintf(stderr, "v3d:   attr%d base=%08x bytes=%u stride=%u vpm=%u/%u\n", i,
                ldl_le_p(r), r[4] + 1, r[5], r[6], r[7]);
        v3d_dump_words("attrdata", ldl_le_p(r), 24);
    }
    v3d_dump_words("fs_unif", ldl_le_p(rec + 8), 8);
    v3d_dump_words("vs_unif", ldl_le_p(rec + 20), 8);
    v3d_dump_words("fs_code", ldl_le_p(rec + 4), 24);
}

static void v3d_dump_list(BCM21553V3D *s, const char *kind, uint32_t ca, uint32_t ea)
{
    uint32_t len = ea > ca ? ea - ca : 0;
    uint8_t *buf;
    char path[256];
    uint32_t i, n;

    if (!g_getenv("TOTORO_V3D_DEBUG") || s->jobs >= 64 || len == 0) {
        return;
    }
    if (len > 0x100000) {
        len = 0x100000;
    }
    buf = g_malloc0(len + 16);
    bcm21553_dma_read(ca, MEMTXATTRS_UNSPECIFIED, buf, len);
    snprintf(path, sizeof(path), "%s/totoro/run/v3d/job%03u_%s_%08x.bin",
             g_get_home_dir(), s->jobs, kind, ca);
    g_file_set_contents(path, (const char *)buf, len, NULL);
    fprintf(stderr, "v3d: job %u %s ca=%08x ea=%08x len=%u ops:", s->jobs, kind, ca, ea, len);
    for (i = 0, n = 0; i < len && n < 64; n++) {
        uint8_t op = buf[i];
        fprintf(stderr, " %u", op);
        if (op == 113) {
            fprintf(stderr, "[fb=%08x %ux%u f=%x]", ldl_le_p(buf + i + 1),
                    lduw_le_p(buf + i + 5), lduw_le_p(buf + i + 7), lduw_le_p(buf + i + 9));
        } else if (op == 112) {
            fprintf(stderr, "[ta=%08x sz=%x st=%08x %ux%u f=%x]", ldl_le_p(buf + i + 1),
                    ldl_le_p(buf + i + 5), ldl_le_p(buf + i + 9), buf[i + 13], buf[i + 14], buf[i + 15]);
        } else if (op == 64 || op == 65 || op == 16 || op == 17) {
            fprintf(stderr, "[%08x]", ldl_le_p(buf + i + 1));
        }
        if (op == 65 && s->jobs < 6) {
            v3d_dump_shader(ldl_le_p(buf + i + 1));
        }
        if (!v3d_pkt_len[op]) {
            fprintf(stderr, "?");
            break;
        }
        i += v3d_pkt_len[op];
    }
    fprintf(stderr, "\n");
    g_free(buf);
}

/* ---- V3D stage 2: software rasteriser for SurfaceFlinger NV-shader draws ---- */
typedef struct { uint32_t rec; uint8_t mode; uint32_t count, first; } V3DPrim;
#define V3D_MAXPRIM 64
static V3DPrim v3d_prims[V3D_MAXPRIM];
static int v3d_nprims;
static unsigned v3d_frames;

typedef struct { uint8_t *d; uint32_t size, w, h, type, swap, tfmt; int bpp; } V3DTex;

/*
 * Asynchronous rendering.
 *
 * The guest vendor driver (v3d_opt.c) posts a binning job (CT0EA), waits for
 * FLDONE, posts the render job (CT1EA) and waits for FRDONE with a 750 ms
 * timeout, killing the job on timeout.  Completion therefore does not have to
 * be synchronous: the rasterisation is moved into a worker thread while the
 * vCPU keeps running.  The strict requirement is that FRDONE is only raised
 * AFTER the framebuffer writeback is complete (SurfaceFlinger reads the fb
 * right after WAIT_JOB), and that binned primitive lists are snapshotted at
 * CT0EA time because the next binning job overwrites them.
 *
 * locking.  Ordering rules:
 *   - main loop takes BQL then v3d_worker.lock (submit, reset, exit, BH);
 *   - worker takes v3d_worker.lock only.
 * The worker never calls address_space_* APIs and never needs the BQL or RCU,
 * so joining it from the exit notifier (which runs with the BQL held) cannot
 * deadlock.  Guest RAM is read by the worker through host pointers captured
 * while the submit phase holds the BQL: every range used by a job is resolved
 * then; ranges that are not contiguous direct-access memory (MMIO or sparse)
 * are copied via address_space_read() right in the submit phase, exactly like
 * the old synchronous code did.  Any captured host pointer stays valid until
 * job completion: RAM blocks of this machine are allocated at init and never
 * freed while the machine exists, and the worker is joined in the exit
 * notifier after VM shutdown.  Concurrent guest writes to the same RAM are
 * the normal unserialised DMA-read model: the raster sees guest data at an
 * arbitrary point of its own generation, which is functionally equivalent to
 * the previous synchronous read right before rasterisation.
 */
typedef struct V3DWRange {
    const uint8_t *host;    /* captured RAM host pointer, or NULL if copied */
    uint8_t *copy;          /* submit-phase copy for non-RAM ranges */
    hwaddr addr;
    uint32_t len;
} V3DWRange;

struct V3DJob {
    /* control list of the render job (resolved at submit) */
    V3DWRange cl;
    /* LOAD_GENERAL tile source, resolved at submit (lda != 0 only) */
    V3DWRange load;
    /* binned primitives snapshot (taken at CT0EA) */
    V3DPrim prims[V3D_MAXPRIM];
    int nprims;
    bool has_prims;         /* binning jobs always precede render jobs */
    /* result written back by the completion BH */
    uint8_t *out;
    uint32_t fba, osize;
    uint32_t gen;           /* generation at submit; BH drops stale jobs */
    unsigned frame;
    /* ranges used by v3d_draw_prim per primitive, resolved at submit */
    V3DWRange *ra;          /* nprims * (rec, u, vb, tex) ranges */
};
typedef struct V3DJob V3DJob;

typedef struct V3DWorker {
    QemuThread thread;
    QemuMutex lock;
    QemuCond cond;
    QEMUBH *done_bh;
    GQueue queue;           /* of V3DJob*, FIFO, guarded by lock */
    GQueue done;            /* of V3DJob*, completed, waiting for BH */
    bool stopping;
    bool thread_running;
    uint32_t gen;           /* bumped on machine reset */
    bool async;             /* false => legacy synchronous path */
} V3DWorker;

static V3DWorker v3d_worker;

/*
 * Snapshot helpers, called with the BQL held (submit phase).
 *
 * v3d_snapshot_range() resolves [addr, addr+len) guest memory for a job.
 * Contiguous direct-access memory (plain RAM of this machine) is referenced
 * via a host pointer; anything else - MMIO windows, sparse mappings crossing
 * region boundaries (garbage uniforms do this) - is copied immediately with
 * address_space_read(), byte-for-byte identical to the synchronous path.
 */
static bool v3d_snapshot_range(V3DWRange *rg, hwaddr addr, uint32_t len)
{
    rg->host = NULL;
    rg->copy = NULL;
    rg->addr = addr;
    rg->len = len;
    if (!len) {
        return true;
    }
    WITH_RCU_READ_LOCK_GUARD() {
        hwaddr xlat, l = len;
        MemoryRegion *mr = address_space_translate(&address_space_memory,
                                                   addr, &xlat, &l, false,
                                                   MEMTXATTRS_UNSPECIFIED);
        if (l >= len && memory_access_is_direct(mr, false,
                                                MEMTXATTRS_UNSPECIFIED)) {
            rg->host = memory_region_get_ram_ptr(mr) + xlat;
            return true;
        }
    }
    rg->copy = g_malloc(len);
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       rg->copy, len);
    return true;
}

static const uint8_t *v3d_range_ptr(const V3DWRange *rg)
{
    return rg->copy ? rg->copy : rg->host;
}

static void v3d_range_clear(V3DWRange *rg)
{
    g_free(rg->copy);
    rg->copy = NULL;
}

static void v3d_job_free(V3DJob *j)
{
    if (!j) {
        return;
    }
    v3d_range_clear(&j->cl);
    v3d_range_clear(&j->load);
    for (int k = 0; k < j->nprims; k++) {
        V3DWRange *r = &j->ra[k * 4];
        for (int q = 0; q < 4; q++) {
            v3d_range_clear(&r[q]);
        }
    }
    g_free(j->out);
    g_free(j->ra);
    g_free(j);
}

static void v3d_parse_bin(uint32_t ca, uint32_t ea)
{
    uint32_t len = ea > ca ? ea - ca : 0, i = 0, cur = 0;
    uint8_t *b;
    v3d_nprims = 0;
    if (!len || len > 0x10000) {
        return;
    }
    b = g_malloc0(len + 16);
    bcm21553_dma_read(ca, MEMTXATTRS_UNSPECIFIED, b, len);
    while (i < len) {
        uint8_t op = b[i];
        if (!v3d_pkt_len[op]) {
            break;
        }
        if (op == 65) {
            cur = ldl_le_p(b + i + 1) & ~0xfu;
        } else if (op == 33 && cur && v3d_nprims < V3D_MAXPRIM) {
            V3DPrim *p = &v3d_prims[v3d_nprims++];
            p->rec = cur; p->mode = b[i + 1];
            p->count = ldl_le_p(b + i + 2); p->first = ldl_le_p(b + i + 6);
        }
        i += v3d_pkt_len[op];
    }
    g_free(b);
}

/* VideoCore IV T-format (4K tiles, boustrophedon rows, 1K subtiles, 64B microtiles) */
static uint32_t v3d_t_off(uint32_t x, uint32_t y, uint32_t tw, int bpp)
{
    static const uint8_t odd[4] = {2, 1, 3, 0}, even[4] = {0, 3, 1, 2};
    /* bpp is 2 or 4; all microtile dimensions are powers of two. */
    uint32_t ushift = bpp == 4 ? 2 : 3, uw = 1u << ushift;
    uint32_t ux = x >> ushift, uy = y >> 2;
    uint32_t tpr = (tw + uw * 8 - 1) >> (ushift + 3);
    uint32_t tx = ux >> 3, ty = uy >> 3;
    uint32_t toff = 4096 * (ty * tpr + ((ty & 1) ? tpr - 1 - tx : tx));
    uint32_t sx = (ux >> 2) & 1, sy = (uy >> 2) & 1;
    uint32_t si = (ty & 1) ? odd[sy * 2 + sx] : even[sy * 2 + sx];
    return toff + 1024 * si + 64 * ((uy & 3) * 4 + (ux & 3)) + ((y & 3) * uw + (x & (uw - 1))) * bpp;
}

static void v3d_texel(V3DTex *t, int x, int y, uint8_t c[4])
{
    uint32_t off, v;
    uint8_t tmp;
    if (x < 0) { x = 0; }
    if (y < 0) { y = 0; }
    if (x >= (int)t->w) { x = t->w - 1; }
    if (y >= (int)t->h) { y = t->h - 1; }
    off = t->tfmt ? v3d_t_off(x, y, t->w, t->bpp) : (y * t->w + x) * t->bpp;
    if (off + t->bpp > t->size) {
        c[0] = c[1] = c[2] = 0; c[3] = 255;
        return;
    }
    if (t->bpp == 4) {
        memcpy(c, t->d + off, 4);
        if (t->type == 1) { c[3] = 255; }
    } else {
        v = lduw_le_p(t->d + off);
        switch (t->type) {
        case 2:
            c[0] = (v >> 12) * 17; c[1] = ((v >> 8) & 15) * 17;
            c[2] = ((v >> 4) & 15) * 17; c[3] = (v & 15) * 17;
            break;
        case 3:
            c[0] = ((v >> 11) & 31) * 255 / 31; c[1] = ((v >> 6) & 31) * 255 / 31;
            c[2] = ((v >> 1) & 31) * 255 / 31; c[3] = (v & 1) ? 255 : 0;
            break;
        default:
            c[0] = ((v >> 11) & 31) * 255 / 31; c[1] = ((v >> 5) & 63) * 255 / 63;
            c[2] = (v & 31) * 255 / 31; c[3] = 255;
        }
    }
    if (t->swap) { tmp = c[0]; c[0] = c[2]; c[2] = tmp; }
}

static void v3d_tri(uint8_t *fb, int W, int H, V3DTex *t, const float *vx, const float *vy,
                    const float *vs, const float *vt, int a, int b, int c)
{
    float area = (vx[b] - vx[a]) * (vy[c] - vy[a]) - (vx[c] - vx[a]) * (vy[b] - vy[a]);
    float mnx, mxx, mny, mxy;
    int x, y, x0, x1, y0, y1;
    int blend = (t->type == 0 || t->type == 2 || t->type == 3);
    if (area > -1e-6f && area < 1e-6f) {
        return;
    }
    const float inv_area = 1.0f / area;
    mnx = MIN(vx[a], MIN(vx[b], vx[c])); mxx = MAX(vx[a], MAX(vx[b], vx[c]));
    mny = MIN(vy[a], MIN(vy[b], vy[c])); mxy = MAX(vy[a], MAX(vy[b], vy[c]));
    x0 = mnx < 0 ? 0 : (int)mnx; y0 = mny < 0 ? 0 : (int)mny;
    x1 = mxx > W ? W : (int)mxx + 1; y1 = mxy > H ? H : (int)mxy + 1;
    for (y = y0; y < y1; y++) {
        for (x = x0; x < x1; x++) {
            float px = x + 0.5f, py = y + 0.5f, w0, w1, w2, s, tt;
            uint8_t col[4], *d;
            w0 = ((vx[b] - px) * (vy[c] - py) - (vx[c] - px) * (vy[b] - py)) * inv_area;
            w1 = ((vx[c] - px) * (vy[a] - py) - (vx[a] - px) * (vy[c] - py)) * inv_area;
            w2 = 1.0f - w0 - w1;
            if (w0 < -1e-4f || w1 < -1e-4f || w2 < -1e-4f) {
                continue;
            }
            s = w0 * vs[a] + w1 * vs[b] + w2 * vs[c];
            tt = w0 * vt[a] + w1 * vt[b] + w2 * vt[c];
            v3d_texel(t, (int)(s * t->w), (int)(tt * t->h), col);
            d = fb + (y * W + x) * 4;
            if (blend && col[3] < 255) {
                int k, ia = 255 - col[3];
                for (k = 0; k < 4; k++) {
                    int o = col[k] + d[k] * ia / 255;
                    d[k] = o > 255 ? 255 : o;
                }
            } else {
                memcpy(d, col, 4);
            }
        }
    }
}

/*
 * Render one primitive from a job snapshot.
 *
 * ra[idx*4 + q] ranges were resolved at submit time:
 *   q=0: shader record (16 bytes at p->rec)
 *   q=1: uniforms    (20 bytes at rec[2])
 *   q=2: vertices    (stride * n at rec[3] + first * stride)
 *   q=3: texture     (t.size at u[0] & ~0xfffu)
 * Every range is either a captured RAM host pointer or a submit-phase copy;
 * the texture may also be the shared v3d_zero_tex for read-as-zero ranges.
 */
static void v3d_draw_prim(V3DJob *j, int idx, uint8_t *fb, int W, int H, int dbg)
{
    V3DPrim *p = &j->prims[idx];
    V3DWRange *r = &j->ra[idx * 4];
    const uint8_t *recb = v3d_range_ptr(&r[0]);
    const uint8_t *ub = v3d_range_ptr(&r[1]);
    uint32_t u0, u1, u4, rec0;
    V3DTex t = {0};
    float vx[64], vy[64], vs[64], vt[64];
    const uint8_t *vb;
    int stride, nv, n, k;

    if (!recb || !ub) {
        return;
    }
    rec0 = ldl_le_p(recb);
    u0 = ldl_le_p(ub);
    u1 = ldl_le_p(ub + 4);
    u4 = ldl_le_p(ub + 16);
    stride = (rec0 >> 8) & 0xff; nv = (rec0 >> 24) & 0xff; n = p->count;
    if (stride < 12 || stride > 64 || n < 3 || n > 64 || p->first > 64) {
        if (dbg) {
            fprintf(stderr, "v3d:  skip prim rec=%08x cfg=%08x n=%d\n", p->rec, rec0, n);
        }
        return;
    }
    vb = v3d_range_ptr(&r[2]);
    if (!vb) {
        return;
    }
    for (k = 0; k < n; k++) {
        const uint8_t *v = vb + k * stride;
        uint32_t xy = ldl_le_p(v);
        vx[k] = (int16_t)(xy & 0xffff) / 16.0f;
        vy[k] = (int16_t)(xy >> 16) / 16.0f;
        vs[k] = vt[k] = 0;
        if (nv >= 2 && stride >= 20) {
            memcpy(&vt[k], v + 12, 4);
            memcpy(&vs[k], v + 16, 4);
        }
    }
    t.type = (u0 >> 4) & 0xf; t.w = (u1 >> 8) & 0x7ff; t.h = (u1 >> 20) & 0x7ff;
    if (!t.w) { t.w = 2048; }
    if (!t.h) { t.h = 2048; }
    t.bpp = t.type <= 1 ? 4 : 2;
    t.swap = (u4 == 16);
    t.tfmt = getenv("TOTORO_V3D_LINEAR") == NULL;
    if (t.tfmt) {
        uint32_t uw = t.bpp == 4 ? 4 : 8;
        t.size = ((t.w + uw * 8 - 1) / (uw * 8)) * ((t.h + 31) / 32) * 4096;
    } else {
        t.size = t.w * t.h * t.bpp;
    }
    if (t.size > 8 * 1024 * 1024) { t.size = 8 * 1024 * 1024; }
    t.d = (uint8_t *)v3d_range_ptr(&r[3]);
    if (!t.d) {
        /* submit phase found the texture range entirely inside the
         * read-as-zero OneNAND stub: sample the shared zero texture */
        if (!v3d_zero_tex) {
            v3d_zero_tex = g_malloc0(8 * 1024 * 1024);
        }
        t.d = v3d_zero_tex;
    }
    if (dbg) {
        fprintf(stderr, "v3d:  prim mode=%u n=%d cfg=%08x code=%08x tex=%08x type=%u %ux%u u4=%u "
                "v0=(%.1f,%.1f st=%.3f,%.3f) v2=(%.1f,%.1f st=%.3f,%.3f) texel0=%02x%02x%02x%02x\n",
                p->mode, n, rec0, ldl_le_p(recb + 4), u0 & ~0xfffu, t.type, t.w, t.h, u4,
                vx[0], vy[0], vs[0], vt[0], vx[2], vy[2], vs[2], vt[2],
                t.d[0], t.d[1], t.d[2], t.d[3]);
    }
    if (p->mode == 4) {
        for (k = 0; k + 2 < n; k += 3) { v3d_tri(fb, W, H, &t, vx, vy, vs, vt, k, k + 1, k + 2); }
    } else if (p->mode == 5) {
        for (k = 0; k + 2 < n; k++) { v3d_tri(fb, W, H, &t, vx, vy, vs, vt, k, k + 1, k + 2); }
    } else {
        for (k = 1; k + 1 < n; k++) { v3d_tri(fb, W, H, &t, vx, vy, vs, vt, 0, k, k + 1); }
    }
}

static uint32_t v3d_img_size(uint32_t W, uint32_t H, int bpp, int tfmt)
{
    if (tfmt) {
        uint32_t uw = bpp == 4 ? 4 : 8;
        uint32_t tpr = (W + uw * 8 - 1) / (uw * 8), rows = (H + 31) / 32;
        return 4096 * tpr * rows;
    }
    return W * H * bpp;
}

/*
 * RGBA8888 rows within a T-format microtile contain four contiguous pixels.
 * Copy a whole microtile row instead of recomputing the layout per pixel.
 * This is a pure layout conversion, not a shader/rasterisation shortcut.
 */
static void v3d_rgba_layout(uint8_t *linear, uint8_t *tiled,
                            uint32_t W, uint32_t H, uint32_t size,
                            bool to_tiled)
{
    uint32_t x, y;
    for (y = 0; y < H; y++) {
        for (x = 0; x < W; x += 4) {
            uint32_t bytes = MIN(4u, W - x) * 4;
            uint32_t off = v3d_t_off(x, y, W, 4);
            uint8_t *row = linear + (y * W + x) * 4;
            if (off + bytes > size) {
                continue;
            }
            if (to_tiled) {
                memcpy(tiled + off, row, bytes);
            } else {
                memcpy(row, tiled + off, bytes);
            }
        }
    }
}

/*
 * Worker phase: rasterise a fully snapshotted job.
 *
 * In the asynchronous path this runs on the worker thread with no locks,
 * no BQL, no address_space access: job->cl and the per-primitive ranges are
 * either captured RAM host pointers or private copies.  Sets j->out to the
 * rasterised image (the buffer the completion BH writes back to guest
 * memory).
 *
 * In the legacy synchronous path (v3d_render_sync) the very same function
 * runs in the vCPU thread right after the submit phase, which is exactly
 * where the old inline code used to be.
 */
static void v3d_job_raster(V3DJob *j)
{
    uint32_t i = 0, fba = 0, clr = 0, W = 0, H = 0, px;
    uint32_t lda = 0, ldfmt = 0, ldlay = 0, fmt = 0, x, y, osize;
    const uint8_t *b;
    uint8_t *fb, *out;
    int k, obpp, otile;
    int dbg = (v3d_frames < 4 || (v3d_frames % 300) == 0) &&
              g_getenv("TOTORO_V3D_DEBUG");

    b = v3d_range_ptr(&j->cl);
    if (!b) {
        return;
    }
    while (i < j->cl.len) {
        uint8_t op = b[i];
        if (!v3d_pkt_len[op]) {
            break;
        }
        if (op == 114) {
            clr = ldl_le_p(b + i + 1);
        } else if (op == 113 && !fba) {
            fba = ldl_le_p(b + i + 1); W = lduw_le_p(b + i + 5); H = lduw_le_p(b + i + 7);
            fmt = b[i + 9];
        } else if (op == 29 && !lda) {
            ldlay = b[i + 1]; ldfmt = b[i + 2]; lda = ldl_le_p(b + i + 3);
        }
        i += v3d_pkt_len[op];
    }
    if (!fba || !W || !H || W > 1024 || H > 1024) {
        return;
    }
    fb = g_malloc(W * H * 4);
    if (lda) {
        /*
         * LOAD_GENERAL: tile buffer starts as the source image.  The submit
         * phase stored the whole source in the extra range attached after
         * the primitive ranges (cl reuse is not possible: the control list
         * and the tile source live at different guest addresses).
         */
        V3DTex t;
        const uint8_t *src;
        memset(&t, 0, sizeof(t));
        t.bpp = (ldfmt & 3) == 2 ? 2 : 4;
        t.type = t.bpp == 2 ? 4 : 0;
        t.tfmt = (ldlay & 0x10) != 0;
        t.w = W; t.h = H;
        t.size = v3d_img_size(W, H, t.bpp, t.tfmt);
        src = v3d_range_ptr(&j->load);
        if (!src) {
            g_free(fb);
            return;
        }
        t.d = (uint8_t *)src;
        if (t.bpp == 4 && t.tfmt) {
            v3d_rgba_layout(fb, t.d, W, H, t.size, false);
        } else if (t.bpp == 4) {
            memcpy(fb, t.d, W * H * 4);
        } else {
            for (y = 0; y < H; y++) {
                for (x = 0; x < W; x++) {
                    v3d_texel(&t, x, y, fb + (y * W + x) * 4);
                }
            }
        }
    } else {
        for (px = 0; px < W * H; px++) {
            stl_le_p(fb + px * 4, clr);
        }
    }
    /* tile-conversion jobs (LOAD_GENERAL present) are pure load/store: no prims */
    for (k = 0; !lda && k < j->nprims; k++) {
        v3d_draw_prim(j, k, fb, W, H, dbg);
    }
    /* STORE: op113 format bits[3:2] 1=RGBA8888, 0/2=565; bit6 = T-format */
    obpp = ((fmt >> 2) & 3) == 1 ? 4 : 2;
    otile = (fmt & 0x40) != 0;
    osize = v3d_img_size(W, H, obpp, otile);
    if (obpp == 4 && !otile) {
        out = fb;
    } else {
        out = g_malloc0(osize);
        if (obpp == 4 && otile) {
            v3d_rgba_layout(fb, out, W, H, osize, true);
        }
        for (y = 0; obpp != 4 && y < H; y++) {
            for (x = 0; x < W; x++) {
                uint8_t *c = fb + (y * W + x) * 4;
                uint32_t off = otile ? v3d_t_off(x, y, W, obpp) : (y * W + x) * obpp;
                if (off + obpp > osize) {
                    continue;
                }
                if (obpp == 4) {
                    memcpy(out + off, c, 4);
                } else {
                    stw_le_p(out + off, ((c[0] >> 3) << 11) | ((c[1] >> 2) << 5) | (c[2] >> 3));
                }
            }
        }
        g_free(fb);
    }
    j->out = out;
    j->fba = fba;
    j->osize = osize;
    j->frame = v3d_frames;
    if (dbg) {
        fprintf(stderr, "v3d: frame %u fb=%08x %ux%u fmt=%02x clr=%08x prims=%d load=%08x/%02x/%02x\n",
                v3d_frames, fba, W, H, fmt, clr, j->nprims, lda, ldlay, ldfmt);
    }
    v3d_frames++;
}

/* ------------------------------------------------------------------ */
/* Async submit (BQL, vCPU thread) / worker thread / completion BH     */
/* ------------------------------------------------------------------ */

/*
 * Snapshot phase for one render job, called with the BQL held.  Resolves
 * every guest-memory range the raster will read into the job.  Returns true
 * on success; on failure the job is still freeable with v3d_job_free().
 */
static bool v3d_submit_snap(V3DJob *j, uint32_t ca, uint32_t ea)
{
    uint32_t len = ea > ca ? ea - ca : 0, i;
    const uint8_t *b;
    bool ok = false, lda_present = false, ld2bpp = false, ldtfmt = false;
    uint32_t lda = 0;

    if (!len || len > 0x10000) {
        return false;
    }
    v3d_snapshot_range(&j->cl, ca, len);
    b = v3d_range_ptr(&j->cl);
    if (!b) {
        return false;
    }
    /* pass 1: find LOAD_GENERAL (op 29) */
    i = 0;
    while (i < len) {
        uint8_t op = b[i];
        if (!v3d_pkt_len[op]) {
            break;
        }
        if (op == 29) {
            lda = ldl_le_p(b + i + 3);
            ld2bpp = ((b[i + 2] & 3) == 2);
            ldtfmt = (b[i + 1] & 0x10) != 0;
            lda_present = lda != 0;
            break;
        }
        i += v3d_pkt_len[op];
    }
    /* resolve per-primitive ranges; shader record bases were parsed at
     * CT0EA time into prims[].rec.  Tile-conversion jobs (op 29 present)
     * draw no primitives, mirroring the raster's !lda gate. */
    ok = true;
    for (int pi = 0; ok && !lda_present && pi < j->nprims; pi++) {
        V3DPrim *p = &j->prims[pi];
        V3DWRange *r = &j->ra[pi * 4];
        uint32_t rec[4] = {0}, u[5] = {0};

        v3d_snapshot_range(&r[0], p->rec, 16);
        if (v3d_range_ptr(&r[0])) {
            memcpy(rec, v3d_range_ptr(&r[0]), 16);
        } else {
            return false;
        }
        v3d_snapshot_range(&r[1], rec[2], sizeof(u));
        if (v3d_range_ptr(&r[1])) {
            memcpy(u, v3d_range_ptr(&r[1]), sizeof(u));
        }
        {
            uint32_t stride = (rec[0] >> 8) & 0xff, n = p->count;
            if (stride < 12 || stride > 64 || n < 3 || n > 64 ||
                p->first > 64) {
                /* primitive will be skipped by the raster; only the
                 * record range is needed */
                continue;
            }
            v3d_snapshot_range(&r[2], rec[3] + p->first * stride,
                               stride * n);
        }
        /* texture size depends on uniforms; mirror the raster's size
         * computation exactly */
        {
            uint32_t tw = (u[1] >> 8) & 0x7ff, th = (u[1] >> 20) & 0x7ff;
            uint32_t type = (u[0] >> 4) & 0xf, texa = u[0] & ~0xfffu;
            int bpp = type <= 1 ? 4 : 2;
            int tfmt = getenv("TOTORO_V3D_LINEAR") == NULL;
            uint32_t tsize;
            if (!tw) { tw = 2048; }
            if (!th) { th = 2048; }
            if (tfmt) {
                uint32_t uw = bpp == 4 ? 4 : 8;
                tsize = ((tw + uw * 8 - 1) / (uw * 8)) * ((th + 31) / 32) * 4096;
            } else {
                tsize = tw * th * bpp;
            }
            if (tsize > 8 * 1024 * 1024) {
                tsize = 8 * 1024 * 1024;
            }
            if (bcm21553_dma_needs_read(texa, tsize)) {
                v3d_snapshot_range(&r[3], texa, tsize);
                if (!v3d_range_ptr(&r[3])) {
                    return false;
                }
            }
            /* else: read-as-zero OneNAND stub => r[3] stays empty, the
             * raster samples the shared zero texture */
        }
    }
    /* LOAD_GENERAL tile source for tile-conversion jobs */
    if (ok && lda_present) {
        uint32_t tsize = v3d_img_size(1024, 1024, ld2bpp ? 2 : 4, ldtfmt);
        if (tsize > 8 * 1024 * 1024) {
            tsize = 8 * 1024 * 1024;
        }
        if (bcm21553_dma_needs_read(lda, tsize)) {
            v3d_snapshot_range(&j->load, lda, tsize);
            ok = v3d_range_ptr(&j->load) != NULL;
        }
    }
    return ok;
}

/*
 * Submit one render job: snapshot (BQL) and hand to the worker.  Returns
 * true if the job was queued.
 */
static bool v3d_submit_job(BCM21553V3D *s, uint32_t ca, uint32_t ea)
{
    uint32_t len = ea > ca ? ea - ca : 0;
    V3DJob *j;

    if (!len || len > 0x10000) {
        return false;
    }
    j = g_new0(V3DJob, 1);
    j->nprims = v3d_nprims;
    memcpy(j->prims, v3d_prims, sizeof(V3DPrim) * v3d_nprims);
    j->ra = g_new0(V3DWRange, (size_t)V3D_MAXPRIM * 4);
    if (!v3d_submit_snap(j, ca, ea)) {
        v3d_job_free(j);
        return false;
    }
    qemu_mutex_lock(&v3d_worker.lock);
    if (v3d_worker.stopping) {
        /* exit in progress: drop silently */
        qemu_mutex_unlock(&v3d_worker.lock);
        v3d_job_free(j);
        return false;
    }
    j->gen = v3d_worker.gen;
    g_queue_push_tail(&v3d_worker.queue, j);
    qemu_cond_signal(&v3d_worker.cond);
    qemu_mutex_unlock(&v3d_worker.lock);
    return true;
}

static void v3d_done_bh(void *opaque);

/* Worker thread: rasterise jobs FIFO, one at a time.  Never touches the
 * BQL, the address space, or any QEMU core state - only job snapshots. */
static void *v3d_worker_thread(void *opaque)
{
    V3DWorker *w = opaque;

    qemu_mutex_lock(&w->lock);
    while (!w->stopping) {
        V3DJob *j = g_queue_pop_head(&w->queue);
        if (!j) {
            qemu_cond_wait(&w->cond, &w->lock);
            continue;
        }
        qemu_mutex_unlock(&w->lock);
        v3d_job_raster(j);
        qemu_mutex_lock(&w->lock);
        /* order: raster done -> queue for BH; the BH does the writeback and
         * raises FRDONE in the main loop */
        g_queue_push_tail(&w->done, j);
        aio_bh_schedule_oneshot(qemu_get_aio_context(), v3d_done_bh, w);
    }
    qemu_mutex_unlock(&w->lock);
    return NULL;
}

/* Completion BH, main loop with the BQL held.  Order per job is strictly:
 * writeback -> rfc++ -> FRDONE -> s->jobs++ -> IRQ update (via the normal
 * v3d_update_irq at the end).  Jobs whose generation no longer matches
 * (machine reset in between) are dropped: no writeback, no IRQ - as if the
 * job never completed. */
static void v3d_done_bh(void *opaque)
{
    V3DWorker *w = opaque;
    BCM21553V3D *s = &bcm21553_v3d;
    V3DJob *j;
    bool completed = false;

    for (;;) {
        qemu_mutex_lock(&w->lock);
        j = g_queue_pop_head(&w->done);
        uint32_t gen = w->gen;
        qemu_mutex_unlock(&w->lock);
        if (!j) {
            break;
        }
        if (j->gen == gen && j->out && j->osize) {
            address_space_write(&address_space_memory, j->fba,
                                MEMTXATTRS_UNSPECIFIED, j->out, j->osize);
            s->rfc = (s->rfc + 1) & 0xff;
            s->intctl |= 1;             /* FRDONE */
            s->jobs++;
            completed = true;
        }
        v3d_job_free(j);
    }

    if (completed) {
        v3d_update_irq(s);
        qemu_notify_event();    /* kick TCG in case the vCPU is idle-waiting */
    }
}

/*
 * Legacy synchronous path (TOTORO_V3D_SYNC=1): identical job lifecycle to
 * the async path, but submit, raster and writeback all happen inline in the
 * vCPU thread under the BQL - byte-for-byte the pre-async behaviour.
 */
static void v3d_render_sync(uint32_t ca, uint32_t ea)
{
    V3DJob *j;
    uint32_t len = ea > ca ? ea - ca : 0;

    if (!len || len > 0x10000) {
        return;
    }
    j = g_new0(V3DJob, 1);
    j->nprims = v3d_nprims;
    memcpy(j->prims, v3d_prims, sizeof(V3DPrim) * v3d_nprims);
    j->ra = g_new0(V3DWRange, (size_t)V3D_MAXPRIM * 4);
    if (v3d_submit_snap(j, ca, ea)) {
        v3d_job_raster(j);
        if (j->out && j->osize) {
            address_space_write(&address_space_memory, j->fba,
                                MEMTXATTRS_UNSPECIFIED, j->out, j->osize);
        }
    }
    v3d_job_free(j);
}

static uint64_t bcm21553_v3d_read(void *opaque, hwaddr off, unsigned size)
{
    BCM21553V3D *s = opaque;
    switch (off) {
    case 0x000: return 0x02443356;      /* IDENT0 */
    case 0x030: return s->intctl;
    case 0x034:
    case 0x038: return s->intena;
    case 0x100:                         /* CT0CS */
    case 0x104:                         /* CT1CS */
    case 0x130: return 0;               /* PCS: idle */
    case 0x134: return s->bfc;
    case 0x138: return s->rfc;
    case 0xe30: return s->dbqitc;
    case 0xf20: return 0x1000;          /* ERRSTAT */
    default:    return s->r[(off & 0xffc) / 4];
    }
}static void bcm21553_v3d_write(void *opaque, hwaddr off, uint64_t val, unsigned size)
{
    BCM21553V3D *s = opaque;
    switch (off) {
    case 0x030: s->intctl &= ~val; break;
    case 0x034: s->intena |= val; break;
    case 0x038: s->intena &= ~val; break;
    case 0x134: if (val & 1) { s->bfc = 0; } break;
    case 0x138: if (val & 1) { s->rfc = 0; } break;
    case 0xe30: s->dbqitc &= ~val; break;
    case 0x108:                         /* CT0EA: start binner thread */
        s->r[0x108 / 4] = val;
        v3d_dump_list(s, "bin", s->r[0x110 / 4], val);
        v3d_parse_bin(s->r[0x110 / 4], val);
        s->r[0x110 / 4] = val;
        s->bfc = (s->bfc + 1) & 0xff;
        s->intctl |= 2;                 /* FLDONE */
        break;
    case 0x10c:                         /* CT1EA: start render thread */
        s->r[0x10c / 4] = val;
        v3d_dump_list(s, "rend", s->r[0x114 / 4], val);
        if (v3d_worker.async) {
            v3d_submit_job(s, s->r[0x114 / 4], val);
        } else {
            /* legacy synchronous path (TOTORO_V3D_SYNC=1) */
            v3d_render_sync(s->r[0x114 / 4], val);
            s->rfc = (s->rfc + 1) & 0xff;
            s->intctl |= 1;             /* FRDONE */
            s->jobs++;
        }
        break;
    default:
        s->r[(off & 0xffc) / 4] = val;
        break;
    }
    v3d_update_irq(s);
}

static const MemoryRegionOps bcm21553_v3d_ops = {
    .read = bcm21553_v3d_read,
    .write = bcm21553_v3d_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 4,
    .valid.max_access_size = 4,
};


/* ------------------------------------------------------------------ */
/* V3D worker lifecycle: init / machine reset / QEMU exit              */
/* ------------------------------------------------------------------ */

/* Machine reset, main loop with the BQL held (qemu_register_reset).  Any
 * jobs still queued or being rasterised are invalidated: the completion BH
 * drops them by generation, so no late writeback or IRQ can cross the
 * reset boundary.  The raster of an in-flight job keeps reading its private
 * snapshot, which is safe (host pointers into machine RAM stay mapped). */
static void v3d_worker_reset(void *opaque)
{
    V3DWorker *w = &v3d_worker;
    QEMUBH *bh = w->done_bh;

    if (!w->thread_running) {
        return;
    }
    qemu_mutex_lock(&w->lock);
    w->gen++;
    g_queue_foreach(&w->queue, (GFunc)v3d_job_free, NULL);
    g_queue_clear(&w->queue);
    qemu_mutex_unlock(&w->lock);
    if (bh) {
        qemu_bh_schedule(bh);   /* drain done-queue of stale jobs promptly */
    }
}

/* QEMU exit, called via atexit with the BQL held (qemu_run_exit_notifiers).
 * Stops and joins the worker.  No deadlock is possible: the worker never
 * takes the BQL, it only waits on v3d_worker.lock/cond, which we release
 * around the join. */
static void v3d_exit_notifier(Notifier *notifier, void *data)
{
    V3DWorker *w = &v3d_worker;

    if (!w->thread_running) {
        return;
    }
    qemu_bh_delete(w->done_bh);
    w->done_bh = NULL;
    qemu_mutex_lock(&w->lock);
    w->stopping = true;
    qemu_cond_signal(&w->cond);
    qemu_mutex_unlock(&w->lock);
    qemu_thread_join(&w->thread);
    w->thread_running = false;

    /* free anything still queued or completed-but-not-reaped */
    qemu_mutex_lock(&w->lock);
    g_queue_foreach(&w->queue, (GFunc)v3d_job_free, NULL);
    g_queue_clear(&w->queue);
    g_queue_foreach(&w->done, (GFunc)v3d_job_free, NULL);
    g_queue_clear(&w->done);
    qemu_mutex_unlock(&w->lock);
}

static Notifier v3d_exit_notifier_node = {
    .notify = v3d_exit_notifier,
};

/*
 * Two-phase BH: v3d_done_bh is scheduled by the worker with the oneshot
 * helper; done_bh is a persistent BH reserved for the reset path (drain of
 * stale done jobs without completion).
 */
static void v3d_reset_drain_bh(void *opaque)
{
    v3d_done_bh(opaque);
}

static void v3d_worker_init(void)
{
    V3DWorker *w = &v3d_worker;

    w->async = g_strcmp0(g_getenv("TOTORO_V3D_SYNC"), "1") != 0;
    qemu_mutex_init(&w->lock);
    qemu_cond_init(&w->cond);
    g_queue_init(&w->queue);
    g_queue_init(&w->done);
    w->gen = 0;
    w->stopping = false;
    w->done_bh = qemu_bh_new(v3d_reset_drain_bh, w);
    if (w->async) {
        qemu_thread_create(&w->thread, "v3d-worker", v3d_worker_thread,
                           w, QEMU_THREAD_JOINABLE);
        w->thread_running = true;
        warn_report("v3d: asynchronous rendering enabled (TOTORO_V3D_SYNC=1 "
                    "restores the synchronous path)");
    }
    qemu_register_reset(v3d_worker_reset, NULL);
    qemu_add_exit_notifier(&v3d_exit_notifier_node);
}

/* ---------------------------------------------------------------- */
/* SoC instantiation                                                 */
/* ---------------------------------------------------------------- */
static MemoryRegion otp_mr;
static MemoryRegion lcdc_mr;

qemu_irq bcm21553_irq(BCM21553State *s, int n)
{
    return qdev_get_gpio_in(s->intc, n);
}

BCM21553State *bcm21553_init(MachineState *machine, const BCM21553Config *cfg)
{
    MemoryRegion *sysmem = get_system_memory();
    BCM21553State *s = g_new0(BCM21553State, 1);
    ARMCPU *cpu = ARM_CPU(cpu_create(machine->cpu_type));
    DeviceState *intc;
    int i;

    s->cpu = cpu;
    totoro_ipc_enabled = !g_strcmp0(g_getenv("TOTORO_IPC_PROBE"), "1");
    totoro_ipc_observations = 0;
    memory_region_add_subregion(sysmem, BCM21553_SDRAM_BASE, machine->ram);

    /* LCD scanout of the panel framebuffer chosen by the board */
    bcm21553_fb_base = cfg->fb_base;
    bcm21553_lcd_w = cfg->lcd_w;
    bcm21553_lcd_h = cfg->lcd_h;
    bcm21553_lcd_buf = g_new0(uint32_t, cfg->lcd_w * cfg->lcd_h);
    bcm21553_lcd_con = qemu_graphic_console_create(NULL, 0, &bcm21553_lcd_ops, NULL);
    qemu_console_resize(bcm21553_lcd_con, bcm21553_lcd_w, bcm21553_lcd_h);

    intc = sysbus_create_simple(TYPE_BCM21553_INTC, BCM21553_INTC_BASE,
                                qdev_get_gpio_in(DEVICE(cpu), ARM_CPU_IRQ));
    s->intc = intc;
    if (!g_strcmp0(g_getenv("TOTORO_MODEM_TRANSPORT"), "1")) {
        BCM21553IntcState *ic = BCM21553_INTC(intc);
        ic->modem_memory = (uint8_t *)memory_region_get_ram_ptr(machine->ram)
                           + (TOTORO_IPC_BASE - BCM21553_SDRAM_BASE);
        ic->modem_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, intc_modem_tick, ic);
        timer_mod(ic->modem_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 20000000);
        warn_report("totoro: experimental CP transport; no SIM/calls/SMS or CP task readiness");
    }
    sysbus_create_varargs(TYPE_BCM21553_GPT, BCM21553_GPT_BASE,
                          qdev_get_gpio_in(intc, IRQ_GPT1),
                          qdev_get_gpio_in(intc, IRQ_GPT2), NULL);

    serial_mm_init(sysmem, BCM21553_UART_A_BASE, 2, qdev_get_gpio_in(intc, IRQ_UARTA),
                   13000000 / 16, serial_hd(0), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(sysmem, BCM21553_UART_B_BASE, 2, qdev_get_gpio_in(intc, IRQ_UARTB),
                   13000000 / 16, serial_hd(1), DEVICE_LITTLE_ENDIAN);
    serial_mm_init(sysmem, BCM21553_UART_C_BASE, 2, qdev_get_gpio_in(intc, IRQ_UARTC),
                   13000000 / 16, serial_hd(2), DEVICE_LITTLE_ENDIAN);

    /*
     * BCM21553 exposes a second L210 register window for reads at +0x1000.
     * The vendor kernel uses it for CTRL/AUX and every cache completion poll.
     * Previously these accesses fell through to the catch-all unimp device.
     * Both windows refer to the SAME cache-controller state.
     */
    {
        DeviceState *l2 = sysbus_create_simple("l2x0", BCM21553_L210_BASE, NULL);
        static MemoryRegion l2_read_window;
        memory_region_init_alias(&l2_read_window, NULL, "bcm21553-l210-read",
                                 sysbus_mmio_get_region(SYS_BUS_DEVICE(l2), 0),
                                 0, 0x1000);
        memory_region_add_subregion_overlap(sysmem, BCM21553_L210_BASE + 0x1000,
                                            &l2_read_window, 1);
    }

    {   /* BSC (hardware I2C) x3 */
        static const struct { hwaddr base; int irq; } bsc[3] = {
            { 0x088A0020, 7 }, { 0x088B0020, 13 }, { 0x088F0020, 29 },
        };
        for (i = 0; i < 3; i++) {
            DeviceState *b = sysbus_create_simple(TYPE_BCM21553_BSC, bsc[i].base,
                                                  qdev_get_gpio_in(intc, bsc[i].irq));
            s->bsc[i] = BCM21553_BSC(b)->bus;
        }
    }

    /*
     * Read-back register files (plain RAM) for blocks where the kernel
     * only needs written values to stick: EMI (REQUEST_MASK semaphore at
     * +0x60 used by cpufreq/armahb mode switch), CLKPWR, SYSCFG.
     */
    {
        static const struct { const char *name; hwaddr base; } rf[] = {
            { "bcm21553-emi",    0x08420000 },
            { "bcm21553-clkpwr", 0x08140000 },
            { "bcm21553-syscfg", 0x08880000 },
        };
        static MemoryRegion rf_mr[ARRAY_SIZE(rf)];

        for (i = 0; i < ARRAY_SIZE(rf); i++) {
            memory_region_init_ram(&rf_mr[i], NULL, rf[i].name, 0x1000, &error_fatal);
            memory_region_add_subregion_overlap(sysmem, rf[i].base, &rf_mr[i], 1);
        }
        /* CLKPWR +0x2b8 bits[7:4] == 0x5: APPLL locked */
        stl_le_p((uint8_t *)memory_region_get_ram_ptr(&rf_mr[1]) + 0x2b8, 0x50);
        /*
         * SYSCFG PUMR (+0x18) = 0x40 AP_ONLY_BOOT: the modem CP is not
         * emulated, so the fuse IPC/RPC/RIL drivers don't wait for it.
         */
        stl_le_p((uint8_t *)memory_region_get_ram_ptr(&rf_mr[2]) + 0x18, 0x40);
    }

    /* CLKPWR power islands (V3D +0x148, ...): reads report POWER_OK */
    {
        static MemoryRegion pwrisl_mr;
        memory_region_init_io(&pwrisl_mr, NULL, &pwrisl_ops, NULL,
                              "bcm21553-clkpwr-island", 0x10);
        memory_region_add_subregion_overlap(sysmem, 0x08140148, &pwrisl_mr, 2);
    }

    {
        static MemoryRegion gpio_mr;
        memory_region_init_io(&gpio_mr, NULL, &gpio_ops, NULL, "bcm21553-gpio", 0x1000);
        memory_region_add_subregion_overlap(sysmem, BCM21553_GPIO_BASE, &gpio_mr, 1);
        gpio_parent_irq = qdev_get_gpio_in(intc, 6);   /* IRQ_GPIO */
        kp_irq = qdev_get_gpio_in(intc, 5);            /* IRQ_KEYPAD */
    }

    /* ARM1136 L1 DMA (CP15 c11) is not modelled by QEMU: RAZ/WI */
    {
        static const ARMCPRegInfo arm1136_l1dma_cp_reginfo[] = {
            { .name = "L1DMA", .cp = 15, .crn = 11, .crm = CP_ANY,
              .opc1 = CP_ANY, .opc2 = CP_ANY, .access = PL1_RW,
              .type = ARM_CP_CONST, .resetvalue = 0 },
        };
        define_arm_cp_regs(cpu, arm1136_l1dma_cp_reginfo);
    }

    qemu_register_reset(lcdc_reset, NULL);
    memory_region_init_io(&lcdc_mr, NULL, &lcdc_ops, NULL, "bcm21553-lcdc", 0x1000);
    memory_region_add_subregion_overlap(sysmem, BCM21553_LCDC_BASE, &lcdc_mr, 1);

    memory_region_init_io(&otp_mr, NULL, &otp_ops, NULL, "bcm21553-otp", 0x1000);
    memory_region_add_subregion_overlap(sysmem, BCM21553_OTP_BASE, &otp_mr, 1);

    memory_region_init_io(&bcm21553_v3d.mr, NULL, &bcm21553_v3d_ops, &bcm21553_v3d,
                          "bcm21553-v3d", 0x1000);
    memory_region_add_subregion_overlap(sysmem, 0x08950000, &bcm21553_v3d.mr, 1);
    bcm21553_v3d.irq = qdev_get_gpio_in(intc, 16);
    v3d_worker_init();

    {   /* SDHC3 @0x08400000, IRQ 57: SD card slot (-drive if=sd) */
        DeviceState *sdh = qdev_new(TYPE_SYSBUS_SDHCI), *card;
        DriveInfo *di = drive_get(IF_SD, 0, 0);
        BlockBackend *blk = di ? blk_by_legacy_dinfo(di) : NULL;
        qdev_prop_set_uint8(sdh, "sd-spec-version", 2);
        qdev_prop_set_uint64(sdh, "capareg", 0x016032b2);   /* +SDMA (bit22): avoids the PIO sg_miter WARN storm */
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sdh), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(sdh), 0, 0x08400000, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(sdh), 0, qdev_get_gpio_in(intc, 57));
        card = qdev_new(TYPE_SD_CARD);
        qdev_prop_set_drive_err(card, "drive", blk, &error_fatal);
        qdev_realize_and_unref(card, qdev_get_child_bus(sdh, "sd-bus"), &error_fatal);
    }

    {   /* SDHC1 @0x08110000, IRQ 27: SDIO slot of the BCM4330 WLAN chip.
         * The WLAN chip itself is not modelled, so the slot is an ordinary
         * SDHCI host with no card: the stock driver probes it successfully
         * (it needs a capability register with supported voltages, else it
         * bails out before initialising its wakelock and later BUGs when the
         * vendor dhd module triggers a rescan) and every command then times
         * out, which the mmc core handles as "no card". */
        DeviceState *sdh1 = qdev_new(TYPE_SYSBUS_SDHCI);
        qdev_prop_set_uint8(sdh1, "sd-spec-version", 2);
        qdev_prop_set_uint64(sdh1, "capareg", 0x016032b2);
        sysbus_realize_and_unref(SYS_BUS_DEVICE(sdh1), &error_fatal);
        sysbus_mmio_map_overlap(SYS_BUS_DEVICE(sdh1), 0, 0x08110000, 1);
        sysbus_connect_irq(SYS_BUS_DEVICE(sdh1), 0, qdev_get_gpio_in(intc, 27));
        if (!g_strcmp0(g_getenv("TOTORO_WIFI_PROBE"), "1")) {
            DeviceState *wifi = qdev_new("totoro-bcm4330-probe");
            if (!g_strcmp0(g_getenv("TOTORO_WIFI_NETWORK"), "1")) {
                qemu_configure_nic_device(wifi, true, NULL);
            }
            qdev_realize_and_unref(wifi, qdev_get_child_bus(sdh1, "sd-bus"), &error_fatal);
            qdev_connect_gpio_out_named(wifi, "hostwake", 0,
                qemu_allocate_irq(bcm21553_wifi_hostwake, NULL, 0));
            bcm21553_gpio_set_input(18, 1);
        }
    }

    totoro_onenand_probe_init(sysmem);

    /* Everything not modelled yet: logged with -d unimp */
    create_unimplemented_device("bcm21553-onenand", BCM21553_ONENAND_BASE, 0x01000000);
    create_unimplemented_device("bcm21553-low", 0x01000000, 0x07000000);
    create_unimplemented_device("bcm21553-io", 0x08000000, 0x01000000);
    create_unimplemented_device("bcm21553-high", 0x09000000, 0x77000000);
    return s;
}

static void bcm21553_register_types(void)
{
    type_register_static(&intc_info);
    type_register_static(&gpt_info);
    type_register_static(&bsc_info);
    type_register_static(&i2cregs_info);
}
type_init(bcm21553_register_types);
