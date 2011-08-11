/*
 *  linux/arch/arm/mach-mmp/pxa910.c
 *
 *  Code specific to PXA910
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>

#include <asm/mach/time.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/mfp.h>
#include <mach/devices.h>

#include <linux/platform_device.h>
#include <linux/mfd/ds1wm.h>
#include <linux/memblock.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)
#define RIPC0_VIRT_BASE	(APB_VIRT_BASE + 0x3D000)
#define RIPC0_STATUS	(RIPC0_VIRT_BASE + 0x00)

static struct mfp_addr_map pxa910_mfp_addr_map[] __initdata =
{
	MFP_ADDR_X(GPIO0, GPIO54, 0xdc),
	MFP_ADDR_X(GPIO67, GPIO98, 0x1b8),
	MFP_ADDR_X(GPIO100, GPIO109, 0x238),

	MFP_ADDR(GPIO123, 0xcc),
	MFP_ADDR(GPIO124, 0xd0),

	MFP_ADDR(DF_IO0, 0x40),
	MFP_ADDR(DF_IO1, 0x3c),
	MFP_ADDR(DF_IO2, 0x38),
	MFP_ADDR(DF_IO3, 0x34),
	MFP_ADDR(DF_IO4, 0x30),
	MFP_ADDR(DF_IO5, 0x2c),
	MFP_ADDR(DF_IO6, 0x28),
	MFP_ADDR(DF_IO7, 0x24),
	MFP_ADDR(DF_IO8, 0x20),
	MFP_ADDR(DF_IO9, 0x1c),
	MFP_ADDR(DF_IO10, 0x18),
	MFP_ADDR(DF_IO11, 0x14),
	MFP_ADDR(DF_IO12, 0x10),
	MFP_ADDR(DF_IO13, 0xc),
	MFP_ADDR(DF_IO14, 0x8),
	MFP_ADDR(DF_IO15, 0x4),

	MFP_ADDR(DF_nCS0_SM_nCS2, 0x44),
	MFP_ADDR(DF_nCS1_SM_nCS3, 0x48),
	MFP_ADDR(SM_nCS0, 0x4c),
	MFP_ADDR(SM_nCS1, 0x50),
	MFP_ADDR(DF_WEn, 0x54),
	MFP_ADDR(DF_REn, 0x58),
	MFP_ADDR(DF_CLE_SM_OEn, 0x5c),
	MFP_ADDR(DF_ALE_SM_WEn, 0x60),
	MFP_ADDR(SM_SCLK, 0x64),
	MFP_ADDR(DF_RDY0, 0x68),
	MFP_ADDR(SM_BE0, 0x6c),
	MFP_ADDR(SM_BE1, 0x70),
	MFP_ADDR(SM_ADV, 0x74),
	MFP_ADDR(DF_RDY1, 0x78),
	MFP_ADDR(SM_ADVMUX, 0x7c),
	MFP_ADDR(SM_RDY, 0x80),

	MFP_ADDR_X(MMC1_DAT7, MMC1_WP, 0x84),

	MFP_ADDR_END,
};

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x09c)

static void __init pxa910_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_PXA910_GPIO);

	/* unmask GPIO edge detection for all 4 banks - APMASKx */
	for (i = 0; i < 4; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_PXA910_AP_GPIO, 0, 127, NULL);
}

void __init pxa910_init_irq(void)
{
	icu_init_irq();
	pxa910_init_gpio();
}

void pxa910_ripc_lock(void)
{
	while (__raw_readl(RIPC0_STATUS))
		cpu_relax();
}

int pxa910_ripc_trylock(void)
{
	return !__raw_readl(RIPC0_STATUS);
}

void pxa910_ripc_unlock(void)
{
	__raw_writel(1, RIPC0_STATUS);
}

/* APB peripheral clocks */
static APBC_CLK(uart0, PXA910_UART0, 1, 14745600);
static APBC_CLK(uart1, PXA910_UART1, 1, 14745600);
static APBC_CLK(uart2, PXA910_UART2, 1, 14745600);
static APBC_CLK(twsi0, PXA910_TWSI0, 0, 33000000);
static APBC_CLK(twsi1, PXA910_TWSI1, 0, 33000000);
static APBC_CLK(pwm1, PXA910_PWM1, 1, 13000000);
static APBC_CLK(pwm2, PXA910_PWM2, 1, 13000000);
static APBC_CLK(pwm3, PXA910_PWM3, 1, 13000000);
static APBC_CLK(pwm4, PXA910_PWM4, 1, 13000000);
static APBC_CLK(ssp1, PXA910_SSP1, 4, 3250000);
static APBC_CLK(ssp2, PXA910_SSP2, 0, 0);
static APBC_CLK(rtc, PXA910_RTC, 0x8, 1);
static APBC_CLK(keypad, PXA910_KPC, 0, 32000);
static APBC_CLK(1wire,  PXA910_ONEWIRE,  0, 26000000);

static APMU_CLK(nand, NAND, 0x19b, 156000000);
static APMU_CLK(u2o, USB, 0x1b, 480000000);
static APMU_CLK(sdh0, SDH0, 0x001b, 44500000);
static APMU_CLK(sdh1, SDH1, 0x001b, 44500000);
static APMU_CLK(sdh2, SDH2, 0x001b, 44500000);

/* device and clock bindings */
static struct clk_lookup pxa910_clkregs[] = {
	INIT_CLKREG(&clk_uart0, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_twsi0, "pxa910-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa910-i2c.1", NULL),
	INIT_CLKREG(&clk_pwm1, "pxa910-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "pxa910-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "pxa910-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "pxa910-pwm.3", NULL),
	INIT_CLKREG(&clk_ssp1, "pxa910-ssp.0", NULL),
	INIT_CLKREG(&clk_ssp2, "pxa910-ssp.1", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_u2o, "pxa-u2o", "U2OCLK"),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_rtc, NULL, "MMP-RTC"),
	INIT_CLKREG(&clk_1wire, NULL, "PXA-W1"),
	INIT_CLKREG(&clk_sdh0, "sdhci-pxav2.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxav2.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxav2.2", "PXA-SDHCLK"),
};

/*
 * ACIPC clock is initialized by CP, enable the clock by default
 * and this clock is always enabled.
 */
static void pxa910_init_acipc_clock(void)
{
	__raw_writel(0x3, APBC_PXA910_IPC);
}

static int __init pxa910_init(void)
{
	if (cpu_is_pxa910()) {
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(pxa910_mfp_addr_map);
		pxa_init_dma(IRQ_PXA910_DMA_INT0, 32);
		clkdev_add_table(ARRAY_AND_SIZE(pxa910_clkregs));

		/* enable ac-ipc clock */
		pxa910_init_acipc_clock();
	}

	return 0;
}
postcore_initcall(pxa910_init);

/* system timer - clock enabled, 3.25MHz */
#define TIMER_CLK_RST	(APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(3))

static void __init pxa910_timer_init(void)
{
	/* reset and configure */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_PXA910_TIMERS);
	__raw_writel(TIMER_CLK_RST, APBC_PXA910_TIMERS);

	timer_init(IRQ_PXA910_AP1_TIMER1, IRQ_PXA910_AP1_TIMER2);
}

struct sys_timer pxa910_timer = {
	.init	= pxa910_timer_init,
};

/* on-chip devices */

/* NOTE: there are totally 3 UARTs on PXA910:
 *
 *   UART2   - Slow UART (can be used both by AP and CP)
 *   UART0/1 - Fast UART
 *
 * To be backward compatible with the legacy FFUART/BTUART/STUART sequence,
 * they are re-ordered as:
 *
 *   pxa910_device_uart0 - UART0 as FFUART
 *   pxa910_device_uart1 - UART1 as BTUART
 *   pxa910_device_uart2 - UART2 as GPS
 */
PXA910_DEVICE(uart0, "pxa2xx-uart", 0, UART0, 0xd4017000, 0x30, 21, 22);
PXA910_DEVICE(uart1, "pxa2xx-uart", 1, UART1, 0xd4018000, 0x30, 23, 24);
PXA910_DEVICE(uart2, "pxa2xx-uart", 2, UART2, 0xd4036000, 0x30, 4, 5);
PXA910_DEVICE(twsi0, "pxa910-i2c", 0, TWSI0, 0xd4011000, 0x40);
PXA910_DEVICE(twsi1, "pxa910-i2c", 1, TWSI1, 0xd4037000, 0x40);
PXA910_DEVICE(pwm1, "pxa910-pwm", 0, NONE, 0xd401a000, 0x10);
PXA910_DEVICE(pwm2, "pxa910-pwm", 1, NONE, 0xd401a400, 0x10);
PXA910_DEVICE(pwm3, "pxa910-pwm", 2, NONE, 0xd401a800, 0x10);
PXA910_DEVICE(pwm4, "pxa910-pwm", 3, NONE, 0xd401ac00, 0x10);
PXA910_DEVICE(ssp0, "pxa910-ssp", 0, SSP1, 0xd401b000, 0x90, 52, 53);
PXA910_DEVICE(ssp1, "pxa910-ssp", 1, SSP2, 0xd42a0c00, 0x90, 1, 2);
PXA910_DEVICE(ssp2, "pxa910-ssp", 2, SSP3, 0xd401C000, 0x90, 60, 61);
PXA910_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x80, 97, 99);
PXA910_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
PXA910_DEVICE(sdh0, "sdhci-pxav2", 0, MMC, 0xd4280000, 0x120);
PXA910_DEVICE(sdh1, "sdhci-pxav2", 1, MMC, 0xd4280800, 0x120);
PXA910_DEVICE(sdh2, "sdhci-pxav2", 2, MMC, 0xd4281000, 0x120);
PXA910_DEVICE(cnm, "pxa-cnm", -1, CNM, 0xd420d000, 0x1000);

static struct resource pxa910_resource_rtc[] = {
	{ 0xd4010000, 0xd40100ff, NULL, IORESOURCE_MEM, },
	{ IRQ_PXA910_RTC_INT, IRQ_PXA910_RTC_INT, NULL, IORESOURCE_IRQ, },
	{ IRQ_PXA910_RTC_ALARM, IRQ_PXA910_RTC_ALARM, NULL, IORESOURCE_IRQ, },
};

struct platform_device pxa910_device_rtc = {
	.name		= "mmp-rtc",
	.id		= -1,
	.resource	= pxa910_resource_rtc,
	.num_resources	= ARRAY_SIZE(pxa910_resource_rtc),
};

static struct resource pxa910_resource_1wire[] = {
	{ 0xd4011800, 0xd4011814, NULL, IORESOURCE_MEM, },
	{ IRQ_PXA910_ONEWIRE, IRQ_PXA910_ONEWIRE, NULL,	\
	IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE, },
};

struct platform_device pxa910_device_1wire = {
	.name		= "pxa-w1",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(pxa910_resource_1wire),
	.resource	= pxa910_resource_1wire,
};

void pxa910_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = APMU_PXA910_KP_WAKE_CLR;

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

static unsigned _cp_area_addr;
static unsigned _cp_area_size;
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0x7000000;
	size = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	_cp_area_addr = (unsigned)start;
	_cp_area_size = (unsigned)size;

	return 0;
}
early_param("cpmem", setup_cpmem);

static void __init pxa910_reserve_cpmem_memblock(void)
{
	if (!_cp_area_size)
		return;

	BUG_ON(memblock_reserve(_cp_area_addr, _cp_area_size) != 0);
	memblock_free(_cp_area_addr, _cp_area_size);
	memblock_remove(_cp_area_addr, _cp_area_size);
	printk(KERN_INFO "Reserving CP memory: %dM at %.8x\n",
				(unsigned)_cp_area_size/0x100000,
				(unsigned)_cp_area_addr);
}

void __init pxa910_reserve(void)
{
	/* Reserve memory for CP */
	pxa910_reserve_cpmem_memblock();

}
