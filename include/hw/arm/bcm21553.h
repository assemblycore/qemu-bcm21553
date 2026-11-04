/*
 * Broadcom BCM21553 SoC (ARM1136JF-S AP + VideoCore IV V3D), as used in
 * the Samsung Galaxy Y GT-S5360 "totoro" and other BCM21553 phones.
 *
 * A board calls bcm21553_init() and then attaches its own peripherals:
 * slaves on the BSC I2C buses, GPIO bit-bang I2C buses, GPIO input
 * levels (card detect, interrupt lines), the LCD framebuffer address.
 */
#ifndef HW_ARM_BCM21553_H
#define HW_ARM_BCM21553_H

#include "hw/core/boards.h"
#include "hw/i2c/i2c.h"
#include "target/arm/cpu-qom.h"
#include "qom/object.h"

#define BCM21553_SDRAM_BASE 0x80000000

typedef struct BCM21553Config {
    hwaddr fb_base;         /* LCD framebuffer scanned out to the console */
    int lcd_w, lcd_h;       /* panel size, 32 bpp */
} BCM21553Config;

typedef struct BCM21553State {
    ARMCPU *cpu;
    DeviceState *intc;
    I2CBus *bsc[3];         /* BSC1..3 hardware I2C */
} BCM21553State;

/* Generic I2C register-file slave (PMIC/RTC/MUIC stubs etc.) */
#define TYPE_BCM21553_I2C_REGS "bcm21553-i2c-regs"
OBJECT_DECLARE_SIMPLE_TYPE(BCM21553I2CRegs, BCM21553_I2C_REGS)

struct BCM21553I2CRegs {
    I2CSlave parent_obj;
    uint8_t regs[256];
    uint8_t ptr;
    bool first;
};

BCM21553State *bcm21553_init(MachineState *machine, const BCM21553Config *cfg);
qemu_irq bcm21553_irq(BCM21553State *s, int n);
/* Linux i2c-gpio style bus on a GPIO pin pair */
I2CBus *bcm21553_gpio_i2c_bus(BCM21553State *s, const char *name, int sda, int scl);
/* level seen on a GPIO configured as input (default 1 = pulled up) */
void bcm21553_gpio_set_input(int pin, int level);
/* latch a GPIO interrupt (GPISR) and raise the port IRQ if unmasked */
void bcm21553_gpio_pend(int pin);

/* keypad matrix key (row, col) state; raises the keypad IRQ */
void bcm21553_keypad_set(int row, int col, bool pressed);
/* draw the panel inside a board skin image (0xffRRGGBB, w x h) */
void bcm21553_lcd_set_skin(const uint32_t *px, int w, int h, int x, int y, int scale);

/* Host startup artwork, retired permanently on first guest frame / panel-off. */
void bcm21553_lcd_set_boot_logo(const uint32_t *pixels);

#endif
