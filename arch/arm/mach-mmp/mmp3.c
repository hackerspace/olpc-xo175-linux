/*
 * linux/arch/arm/mach-mmp/mmp3.c
 *
 * code name MMP3
 *
 * Copyright (C) 2009 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/usb/mv_usb.h>

#include <asm/smp_twd.h>
#include <asm/mach/time.h>
#include <asm/hardware/gic.h>
#include <asm/hardware/cache-l2x0.h>

#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/dma.h>
#include <mach/devices.h>
#include <mach/mmp3.h>
#include <mach/regs-usb.h>
#include <mach/mmp2_dma.h>

#include <linux/platform_device.h>

#include <plat/mfp.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)

static struct mfp_addr_map mmp3_addr_map[] __initdata = {
	MFP_ADDR_X(GPIO0, GPIO58, 0x54),
	MFP_ADDR_X(GPIO59, GPIO73, 0x280),
	MFP_ADDR_X(GPIO74, GPIO101, 0x170),
	MFP_ADDR_X(GPIO102, GPIO103, 0x0),
	MFP_ADDR_X(GPIO115, GPIO122, 0x260),
	MFP_ADDR_X(GPIO124, GPIO141, 0xc),
	MFP_ADDR_X(GPIO143, GPIO151, 0x220),
	MFP_ADDR_X(GPIO152, GPIO153, 0x248),
	MFP_ADDR_X(GPIO154, GPIO155, 0x254),

	MFP_ADDR(GPIO142, 0x8),
	MFP_ADDR(GPIO114, 0x164),
	MFP_ADDR(GPIO123, 0x148),

	MFP_ADDR(GPIO168, 0x1e0),
	MFP_ADDR(GPIO167, 0x1e4),
	MFP_ADDR(GPIO166, 0x1e8),
	MFP_ADDR(GPIO165, 0x1ec),
	MFP_ADDR(GPIO107, 0x1f0),
	MFP_ADDR(GPIO106, 0x1f4),
	MFP_ADDR(GPIO105, 0x1f8),
	MFP_ADDR(GPIO104, 0x1fc),
	MFP_ADDR(GPIO111, 0x200),
	MFP_ADDR(GPIO164, 0x204),
	MFP_ADDR(GPIO163, 0x208),
	MFP_ADDR(GPIO162, 0x20c),
	MFP_ADDR(GPIO161, 0x210),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO109, 0x218),
	MFP_ADDR(GPIO108, 0x21c),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO112, 0x244),
	MFP_ADDR(GPIO160, 0x250),
	MFP_ADDR(GPIO113, 0x25c),
	/* FIXME: Zx does not have this pin, define here will not impact */
	MFP_ADDR(GPIO171, 0x2c8),

	MFP_ADDR_X(TWSI1_SCL, TWSI1_SDA, 0x140),
	MFP_ADDR_X(TWSI4_SCL, TWSI4_SDA, 0x2bc),
	MFP_ADDR(PMIC_INT, 0x2c4),
	MFP_ADDR(CLK_REQ, 0x160),

	MFP_ADDR_END,
};

static void __init mmp3_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_MMP2_GPIO);

	/* unmask GPIO edge detection for all 6 banks -- APMASKx */
	for (i = 0; i < 6; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_MMP3_GPIO, 0, 167, NULL);
}

static void uart_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = __raw_readl(clk->clk_rst);
	clk_rst |= APBC_FNCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst |= APBC_APBCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst &= ~(APBC_RST);
	__raw_writel(clk_rst, clk->clk_rst);
}


static void uart_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
	mdelay(1);
}

static int uart_clk_setrate(struct clk *clk, unsigned long val)
{
	uint32_t clk_rst;

	if (val == clk->rate) {
		/* choose vctcxo */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		clk_rst |= APBC_FNCLKSEL(0x1);
		__raw_writel(clk_rst, clk->clk_rst);

	} else if (val > clk->rate) {
		/* set m/n for high speed */
		unsigned int numer = 27;
		unsigned int denom = 16;

		/*
		 *      n/d = base_clk/(2*out_clk)
		 *      base_clk = 199.33M, out_clk=199.33*16/27/2=59.06M
		 *      buadrate = clk/(16*divisor)
		 */

		/* Bit(s) PMUM_SUCCR_RSRV_31_29 reserved */
		/* UART Clock Generation Programmable Divider Numerator Value */
#define PMUM_SUCCR_UARTDIVN_MSK                 (0x1fff << 16)
#define PMUM_SUCCR_UARTDIVN_BASE                16
		/* Bit(s) PMUM_SUCCR_RSRV_15_13 reserved */
		/* UART Clock Generation Programmable Divider Denominator Value */
#define PMUM_SUCCR_UARTDIVD_MSK                 (0x1fff)
#define PMUM_SUCCR_UARTDIVD_BASE                0

		clk_rst = __raw_readl(MPMU_SUCCR);
		clk_rst &=
		    ~(PMUM_SUCCR_UARTDIVN_MSK + PMUM_SUCCR_UARTDIVD_MSK);
		clk_rst |=
		    (numer << PMUM_SUCCR_UARTDIVN_BASE) | (denom <<
							   PMUM_SUCCR_UARTDIVD_BASE);
		__raw_writel(clk_rst, MPMU_SUCCR);
		/* choose programmable clk */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		__raw_writel(clk_rst, clk->clk_rst);
	}


	return 0;
}

struct clkops uart_clk_ops = {
	.enable = uart_clk_enable,
	.disable = uart_clk_disable,
	.setrate = uart_clk_setrate,
};

static void rtc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	clk_rst |= 1 << 7;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void rtc_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

static void sdhc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst |= clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void sdhc_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst &= ~clk->enable_val;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops sdhc_clk_ops = {
	.enable		= sdhc_clk_enable,
	.disable	= sdhc_clk_disable,
};

struct clkops rtc_clk_ops = {
	.enable         = rtc_clk_enable,
	.disable        = rtc_clk_disable,
};

/* usb: hsic clock */
static void hsic_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst |= 0x1b;
	__raw_writel(clk_rst, clk->clk_rst);
}

static void hsic_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst &= ~0x18;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops hsic_clk_ops = {
	.enable         = hsic_clk_enable,
	.disable        = hsic_clk_disable,
};

static void turn_on_pll3(void)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CTRL2);

	/* set SEL_VCO_CLK_SE in PMUM_PLL3_CTRL2 register*/
	__raw_writel(tmp | 0x00000001, PMUM_PLL3_CTRL2);

	/* PLL3 control register 1 - program VCODIV_SEL_SE = 2,
	 * ICP = 4, KVCO = 5 and VCRNG = 4*/
	__raw_writel(0x05290499, PMUM_PLL3_CTRL1);

	/*MPMU_PLL3CR: Program PLL3 VCO for 2.0Ghz -REFD = 3;*/
	tmp = (__raw_readl(APMU_USBFSIC) >> 8) & 0xF;
	if (tmp == 0xD)
		/* 26MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xE6
		 */
		__raw_writel(0x001B9A00, PMUM_PLL3_CR);
	else
		/* 25MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xF6
		 */
		__raw_writel(0x001BdA00, PMUM_PLL3_CR);

	/* PLL3 Control register -Enable SW PLL3*/
	tmp = __raw_readl(PMUM_PLL3_CR);
	__raw_writel(tmp | 0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock*/
	udelay(500);

	/* PMUM_PLL3_CTRL1: take PLL3 out of reset*/
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp | 0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);
}

static void turn_off_pll3(void)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CR);

	/* PLL3 Control register -disable SW PLL3*/
	__raw_writel(tmp & ~0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock*/
	udelay(500);

	/* PMUM_PLL3_CTRL1: put PLL3 into reset*/
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp & ~0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);
}

static void lcd_pn1_clk_enable(struct clk *clk)
{
	u32 tmp;

	/* DSI clock enable*/
	turn_on_pll3();

	tmp = __raw_readl(clk->clk_rst);
	tmp |= 0x103f;
	__raw_writel(tmp, clk->clk_rst);
}

static void lcd_pn1_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	/* tmp &= ~0xf9fff; */
	tmp &= ~0x1038; /* release from reset to keep register setting */
	__raw_writel(tmp, clk->clk_rst);

	/* DSI clock disable*/
	turn_off_pll3();
}

static int lcd_clk_setrate(struct clk *clk, unsigned long val)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	/* u32 pll2clk = pll2_get_clk(); */

	tmp &= ~((0x1f << 15) | (0xf << 8) | (0x3 << 6));
	tmp |= (0x1a << 15);

	switch (val) {
	case 400000000:
		/* DSP1clk = PLL1/2 = 400MHz */
		tmp |= (0x2 << 8) | (0x0 << 6);
		break;
	case 500000000:
		/* DSP1clk = PLL2/x, about 500MHz, temply workaround */
		tmp |= (0x2 << 8) | (0x2 << 6);
		break;
	default:
		printk(KERN_ERR"%s %d not supported\n", __func__, (int) val);
		return -1;
	}

	__raw_writel(tmp, clk->clk_rst);
	return 0;
}

static unsigned long lcd_clk_getrate(struct clk *clk)
{
	u32 lcdclk = clk->rate;

	return lcdclk;
}

struct clkops lcd_pn1_clk_ops = {
	.enable = lcd_pn1_clk_enable,
	.disable = lcd_pn1_clk_disable,
	.setrate = lcd_clk_setrate,
	.getrate = lcd_clk_getrate,
};

#define GC_CLK_DIV(n)	((n & 0xF) << 24)
#define GC_CLK_DIV_GET(n)	((n >> 24) & 0xF)
#define GC_CLK_DIV_MSK	GC_CLK_DIV(0xF)
#define GC360_CLK_EN	(1 << 23)
#define GC360_SW_RST	(1 << 22)
#define GC360_CLK_SRC_SE(n)	((n & 3) << 20)
#define GC360_CLK_SRC_SE_MSK	GC360_CLK_SRC_SE(3)
#define		CS_PLL1		0
#define		CS_PLL2		1
#define		CS_PLL1_COP	2
#define		CS_PLL2_COP	3
#define GC360_AXICLK_EN	(1 << 19)
#define GC360_AXI_RST	(1 << 18)
#define GC_PWRUP(n)		((n & 3) << 9)
#define GC_PWRUP_MSK	GC_PWRUP(3)
#define		PWR_OFF		0
#define		PWR_SLOW_RAMP	1
#define		PWR_ON		3
#define GC_ISB			(1 << 8)
#define GC_CLK_SRC_SEL(n)	((n & 3) << 6)
#define GC_CLK_SRC_SEL_MSK	GC_CLK_SRC_SEL(3)
#define GC_ACLK_SEL(n)	((n & 3) << 4)
#define GC_ACLK_SEL_MSK	GC_ACLK_SEL(3)
#define		PLL1D4		0
#define		PLL1D6		1
#define		PLL1D2		2
#define		PLL2D2		3
#define GC_CLK_EN		(1 << 3)
#define GC_AXICLK_EN	(1 << 2)
#define GC_RST			(1 << 1)
#define GC_AXI_RST		(1 << 0)

#define GC_CLK_RATE(div, src, aclk) (GC_CLK_DIV(div) |\
	GC_CLK_SRC_SEL(src) | GC_ACLK_SEL(aclk))

#define GC_CLK_RATE_MSK	(GC_CLK_DIV_MSK |\
	GC_CLK_SRC_SEL_MSK | GC_ACLK_SEL_MSK)

#define GC_SET_BITS(set, clear)	{\
	unsigned long tmp;\
	tmp = __raw_readl(clk->clk_rst);\
	tmp &= ~clear;\
	tmp |= set;\
	__raw_writel(tmp, clk->clk_rst);\
}

struct clk_gc_cfg {
	unsigned long rate;
	unsigned long cfg;
};

static struct clk_gc_cfg mmp3_gc_clk[] = {
	{ 99000000,	GC_CLK_RATE(8,	CS_PLL1,	PLL1D6) }, /* 0 */
	{ 199000000,	GC_CLK_RATE(4,	CS_PLL1,	PLL1D4) }, /* 1 */
	{ 354000000,	GC_CLK_RATE(3,	CS_PLL1_COP,	PLL1D2) }, /* 2 */
	{ 398000000,	GC_CLK_RATE(2,	CS_PLL1,	PLL1D2) }, /* 3 */
	{ 531000000,	GC_CLK_RATE(2,	CS_PLL1_COP,	PLL1D2) }, /* 4 */
	{ 667000000,	GC_CLK_RATE(2,	CS_PLL2,	PLL2D2) }, /* 5 */
	{ 797000000,	GC_CLK_RATE(1,	CS_PLL1,	PLL1D2) }, /* 6 */
	{ 889000000,	GC_CLK_RATE(1,	CS_PLL2_COP,	PLL2D2) }, /* 7 */
	{ 1063000000,	GC_CLK_RATE(1,	CS_PLL1_COP,	PLL1D2) }, /* 8 */
	{ 1334000000,	GC_CLK_RATE(1,	CS_PLL2_COP,	PLL2D2) }, /* 9 */
};

#define SZ_GC_TABLE (sizeof(mmp3_gc_clk)/sizeof(struct clk_gc_cfg))

unsigned long gc_clk_rate_index = 3;

static void gc_clk_enable(struct clk *clk)
{
	unsigned long gc_rate;

	GC_SET_BITS(GC_PWRUP(PWR_SLOW_RAMP), -1);
	GC_SET_BITS(GC_PWRUP(PWR_ON), GC_PWRUP_MSK);

	gc_rate = mmp3_gc_clk[gc_clk_rate_index].cfg;
	gc_rate &= GC_CLK_RATE_MSK;
	GC_SET_BITS(gc_rate, GC_CLK_RATE_MSK);

	GC_SET_BITS(GC_CLK_EN, 0);
	udelay(100);

	GC_SET_BITS(GC_AXICLK_EN, 0);
	udelay(100);

	GC_SET_BITS(GC_ISB, 0);
	GC_SET_BITS(GC_RST, 0);
	GC_SET_BITS(GC_AXI_RST, 0);
}

static void gc_clk_disable(struct clk *clk)
{
	GC_SET_BITS(0, GC_ISB);
	GC_SET_BITS(0, GC_RST | GC_AXI_RST);
	GC_SET_BITS(0, GC_CLK_EN | GC_AXICLK_EN);
	GC_SET_BITS(0, GC_PWRUP_MSK);
}

static int gc_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long i;

	for (i=0; (i < SZ_GC_TABLE) && (rate > mmp3_gc_clk[i].rate); i++);
	gc_clk_rate_index = i;

	return 0;
}

static unsigned long gc_clk_getrate(struct clk *clk)
{
	return mmp3_gc_clk[gc_clk_rate_index].rate;
}

struct clkops gc_clk_ops = {
	.enable		= gc_clk_enable,
	.disable	= gc_clk_disable,
	.setrate	= gc_clk_setrate,
	.getrate	= gc_clk_getrate,
};

void __init mmp3_init_irq(void)
{
	gic_init(0, 29, (void __iomem *) GIC_DIST_VIRT_BASE, (void __iomem *) GIC_CPU_VIRT_BASE);

	mmp3_init_gic();
}

/* APB peripheral clocks */
static APBC_CLK_OPS(uart1, MMP2_UART1, 1, 26000000, &uart_clk_ops);
static APBC_CLK_OPS(uart2, MMP2_UART2, 1, 26000000, &uart_clk_ops);
static APBC_CLK_OPS(uart3, MMP2_UART3, 1, 26000000, &uart_clk_ops);
static APBC_CLK_OPS(uart4, MMP2_UART4, 1, 26000000, &uart_clk_ops);
static APBC_CLK_OPS(rtc, MMP2_RTC, 0, 32768, &rtc_clk_ops);

static APBC_CLK(twsi1, MMP2_TWSI1, 0, 26000000);
static APBC_CLK(twsi2, MMP2_TWSI2, 0, 26000000);
static APBC_CLK(twsi3, MMP2_TWSI3, 0, 26000000);
static APBC_CLK(twsi4, MMP2_TWSI4, 0, 26000000);
static APBC_CLK(twsi5, MMP2_TWSI5, 0, 26000000);
static APBC_CLK(twsi6, MMP2_TWSI6, 0, 26000000);
static APBC_CLK(pwm1, MMP2_PWM0, 0, 26000000);
static APBC_CLK(pwm2, MMP2_PWM1, 0, 26000000);
static APBC_CLK(pwm3, MMP2_PWM2, 0, 26000000);
static APBC_CLK(pwm4, MMP2_PWM3, 0, 26000000);
static APBC_CLK(keypad, MMP2_KPC, 0, 32768);

static APMU_CLK(nand, NAND, 0xbf, 100000000);
static APMU_CLK(u2o, USB, 0x9, 480000000);

static APMU_CLK_OPS(hsic1, USBHSIC1, 0x1b, 480000000, &hsic_clk_ops);
static APMU_CLK_OPS(sdh0, SDH0, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh1, SDH1, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh2, SDH2, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh3, SDH3, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(lcd, LCD, 0, 500000000, &lcd_pn1_clk_ops);
static APMU_CLK_OPS(gc, GC, 0, 0, &gc_clk_ops);

static struct clk_lookup mmp3_clkregs[] = {
	INIT_CLKREG(&clk_uart1, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_uart2, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_uart3, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_uart4, "pxa2xx-uart.3", NULL),
	INIT_CLKREG(&clk_twsi1, "pxa2xx-i2c.0", NULL),
	INIT_CLKREG(&clk_twsi2, "pxa2xx-i2c.1", NULL),
	INIT_CLKREG(&clk_twsi3, "pxa2xx-i2c.2", NULL),
	INIT_CLKREG(&clk_twsi4, "pxa2xx-i2c.3", NULL),
	INIT_CLKREG(&clk_twsi5, "pxa2xx-i2c.4", NULL),
	INIT_CLKREG(&clk_twsi6, "pxa2xx-i2c.5", NULL),
	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
	INIT_CLKREG(&clk_pwm1, "mmp2-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "mmp2-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "mmp2-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "mmp2-pwm.3", NULL),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_sdh0, "sdhci-pxav3.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxav3.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxav3.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "sdhci-pxav3.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_rtc, "mmp-rtc", NULL),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_hsic1, NULL, "HSIC1CLK"),
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
};

#ifdef CONFIG_CACHE_L2X0
static void mmp3_init_l2x0(void)
{
	void __iomem *l2x0_base = ioremap(SL2C_PHYS_BASE, SZ_4K);
	if (IS_ERR(l2x0_base)) {
		printk(KERN_ERR "L2 map failed %ld\n", PTR_ERR(l2x0_base));
	} else {
		/* Auxiliary Control:
		   TODO: According to the manual, this register should be
			written in secure access, we may need to move the
			configuration in early stage of boot if TZ enabled

		   [ 0.. 2]	cycles of latency of data RAM read
		   [ 3.. 5]	cycles of latency of data RAM write
		   [ 6.. 8]	cycles of latency of tag RAM
		   [ 9..11]	cycles of latency of dirty RAM
		   [12]		exclusive cache op, 0:disable,1:enable
		   [13..16]	associativity
		   [17..19]	way-size
		   [20]		event monitor bus enable
		   [21]		parity enable
		   [22]		shared attribute override enable
		   [23..24]	force write allocate
				0: use AWCACHE
				1: force no WA
				2: force WA on
				3: internal mapped
		   [25]		reserved, SBO/RAO
		   [26]		Non-secure lockdown enable
		   [27]		Non-secure interrupt access enable
		   [28..31]	reserved, SBZ
		*/
		/*
		   forece NO WA, for A0 memory performance, bug in WA
		   64KB way-size
		   clear bit[16] to make sure l2x0_init call take it as 8-way
		*/
		l2x0_init(l2x0_base, 0x00860000, 0xE200FFFF);
	}
}
#else
static void mmp3_init_l2x0(void) {}
#endif

static void __init mmp3_timer_init(void)
{
	uint32_t clk_rst;

#ifdef CONFIG_LOCAL_TIMERS
	twd_base = (void __iomem *)TWD_VIRT_BASE;
#endif

	/* this is early, we have to initialize the CCU registers by
	 * ourselves instead of using clk_* API. Clock rate is defined
	 * by APBC_TIMERS_FNCLKSEL and enabled free-running
	 */
	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMP2_TIMERS);

	/* 6.5MHz, bus/functional clock enabled, release reset */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_MMP2_TIMERS);

	timer_init(IRQ_MMP3_TIMER1, IRQ_MMP3_TIMER2);
}

struct sys_timer mmp3_timer = {
	.init   = mmp3_timer_init,
};

#define PJ4B_WCB_MIN_MSK	(0x3f)
#define PJ4B_WCB_MIN_SHFT	(1)
#define PJ4B_WCB_MAX_MSK	(0x3f)
#define PJ4B_WCB_MAX_SHFT	(7)
#define PJ4B_WCB_EVCT_MSK	(0x7fff)
#define PJ4B_WCB_EVCT_SHFT	(13)
#define OMITFLD			((unsigned long)-1)
#define UPDATE_ON_VALID(lval, rval, msk, shft)		\
	do if (rval != OMITFLD) {			\
		lval &= ~((msk) << (shft));		\
		lval |= (((rval) & (msk)) << (shft));	\
	} while (0)

static unsigned long pj4b_wcb_config(unsigned long min, unsigned long max,
			unsigned long evct)
{
	register unsigned long regval;
	__asm__("mrc p15, 1, %0, c15, c2, 1" : "=r" (regval));
	UPDATE_ON_VALID(regval, min, PJ4B_WCB_MIN_MSK, PJ4B_WCB_MIN_SHFT);
	UPDATE_ON_VALID(regval, max, PJ4B_WCB_MAX_MSK, PJ4B_WCB_MAX_SHFT);
	UPDATE_ON_VALID(regval, evct, PJ4B_WCB_EVCT_MSK, PJ4B_WCB_EVCT_SHFT);
	__asm__("mcr p15, 1, %0, c15, c2, 1" : : "r" (regval));
	return regval;
}

static int __init mmp3_init(void)
{
	/*
	  let's make minimum WCB open entries to 2 to boost memory access
	*/
	pj4b_wcb_config(2, OMITFLD, OMITFLD);

	mmp3_init_l2x0();

	mfp_init_base(MFPR_VIRT_BASE);
	mfp_init_addr(mmp3_addr_map);

	mmp3_init_gpio();

	pxa688_init_dma(IRQ_MMP3_DMA_RIQ);

	clkdev_add_table(ARRAY_AND_SIZE(mmp3_clkregs));

	return 0;
}

postcore_initcall(mmp3_init);

/* on-chip devices */
MMP3_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4030000, 0x30, 4, 5);
MMP3_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 20, 21);
MMP3_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 22, 23);
MMP3_DEVICE(uart4, "pxa2xx-uart", 3, UART4, 0xd4016000, 0x30, 18, 19);
MMP3_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x100, 28, 29);
MMP3_DEVICE(sdh0, "sdhci-pxav3", 0, MMC, 0xd4280000, 0x120);
MMP3_DEVICE(sdh1, "sdhci-pxav3", 1, MMC2, 0xd4280800, 0x120);
MMP3_DEVICE(sdh2, "sdhci-pxav3", 2, MMC3, 0xd4281000, 0x120);
MMP3_DEVICE(sdh3, "sdhci-pxav3", 3, MMC4, 0xd4281800, 0x120);
MMP3_DEVICE(twsi1, "pxa2xx-i2c", 0, TWSI1, 0xd4011000, 0x70);
MMP3_DEVICE(twsi2, "pxa2xx-i2c", 1, TWSI2, 0xd4031000, 0x70);
MMP3_DEVICE(twsi3, "pxa2xx-i2c", 2, TWSI3, 0xd4032000, 0x70);
MMP3_DEVICE(twsi4, "pxa2xx-i2c", 3, TWSI4, 0xd4033000, 0x70);
MMP3_DEVICE(twsi5, "pxa2xx-i2c", 4, TWSI5, 0xd4033800, 0x70);
MMP3_DEVICE(twsi6, "pxa2xx-i2c", 5, TWSI6, 0xd4034000, 0x70);
MMP3_DEVICE(pwm1, "mmp2-pwm", 0, NONE, 0xd401a000, 0x10);
MMP3_DEVICE(pwm2, "mmp2-pwm", 1, NONE, 0xd401a400, 0x10);
MMP3_DEVICE(pwm3, "mmp2-pwm", 2, NONE, 0xd401a800, 0x10);
MMP3_DEVICE(pwm4, "mmp2-pwm", 3, NONE, 0xd401ac00, 0x10);
MMP3_DEVICE(keypad, "pxa27x-keypad", -1, KEYPAD, 0xd4012000, 0x4c);
MMP3_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(v4l2_ovly, "pxa168-v4l2_ovly", 0, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(fb_tv, "pxa168-fb", 1, LCD, 0xd420b000, 0x500);
MMP3_DEVICE(hdmi, "mmp3-hdmi", -1, HDMI, 0xd420b000, 0x1fff);

void mmp3_clear_keypad_wakeup(void)
{
	uint32_t val;
	uint32_t mask = (1 << 5);

	/* wake event clear is needed in order to clear keypad interrupt */
	val = __raw_readl(APMU_WAKE_CLR);
	__raw_writel(val | mask, APMU_WAKE_CLR);
}

static struct resource mmp3_resource_rtc[] = {
	{ 0xd4010000, 0xd40100ff, NULL, IORESOURCE_MEM, },
	{ IRQ_MMP3_RTC, IRQ_MMP3_RTC, NULL, IORESOURCE_IRQ, },
	{ IRQ_MMP3_RTC_ALARM, IRQ_MMP3_RTC_ALARM, NULL, IORESOURCE_IRQ, },
};

struct platform_device mmp3_device_rtc = {
	.name           = "mmp-rtc",
	.id             = -1,
	.resource       = mmp3_resource_rtc,
	.num_resources  = ARRAY_SIZE(mmp3_resource_rtc),
};

#ifdef CONFIG_USB_SUPPORT
static u64 usb_dma_mask = ~(u32)0;

#ifdef CONFIG_USB_PXA_U2O
struct resource mmp3_u2o_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2o = {
	.name		= "pxa-u2o",
	.id		= -1,
	.resource	= mmp3_u2o_resources,
	.num_resources	= ARRAY_SIZE(mmp3_u2o_resources),
	.dev		=  {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	}
};
#endif

#ifdef CONFIG_USB_EHCI_PXA
#ifdef CONFIG_USB_EHCI_PXA_U2O
struct resource mmp3_u2oehci_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2oehci = {
	.name		= "pxa-u2oehci",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(mmp3_u2oehci_resources),
	.resource	= mmp3_u2oehci_resources,
};

#ifdef CONFIG_USB_PXA_U2O_OTG
struct resource mmp3_u2ootg_resources[] = {
	/* regbase */
	[0] = {
		.start	= PXA168_U2O_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= PXA168_U2O_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= PXA168_U2O_PHYBASE,
		.end	= PXA168_U2O_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_OTG,
		.end	= IRQ_MMP3_USB_OTG,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_device_u2ootg = {
	.name		= "pxa-otg",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= 0xffffffff,
	},

	.num_resources	= ARRAY_SIZE(mmp3_u2ootg_resources),
	.resource	= mmp3_u2ootg_resources,
};

#endif	/* CONFIG_USB_PXA_U2O_OTG */
#endif  /* CONFIG_USB_EHCI_PXA_U2O */

#ifdef CONFIG_USB_EHCI_PXA_U2H_HSIC
static struct resource mmp3_hsic1_resources[] = {
	/* reg base */
	[0] = {
		.start	= MMP3_HSIC1_REGBASE + U2x_CAPREGS_OFFSET,
		.end	= MMP3_HSIC1_REGBASE + USB_REG_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "capregs",
	},
	/* phybase */
	[1] = {
		.start	= MMP3_HSIC1_PHYBASE,
		.end	= MMP3_HSIC1_PHYBASE + USB_PHY_RANGE,
		.flags	= IORESOURCE_MEM,
		.name	= "phyregs",
	},
	[2] = {
		.start	= IRQ_MMP3_USB_HS1,
		.end	= IRQ_MMP3_USB_HS1,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device mmp3_hsic1_device = {
	.name		= "mmp3-hsic",
	.id		= -1,
	.dev		= {
		.dma_mask		= &usb_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},

	.num_resources	= ARRAY_SIZE(mmp3_hsic1_resources),
	.resource	= mmp3_hsic1_resources,
};

int mmp3_hsic_private_init(struct mv_op_regs *opregs, unsigned int phyregs)
{
	u32 hsic_int;
	u32 status;
	int count;

	hsic_int = __raw_readl(phyregs + HSIC_INT);
	status = __raw_readl(&opregs->portsc[0]);
	/*disable connect irq*/
	hsic_int &= ~HSIC_INT_CONNECT_INT_EN;
	__raw_writel(hsic_int, phyregs + HSIC_INT);

	/* enable port power and reserved bit 25 */
	status = __raw_readl(&opregs->portsc[0]);
	status |= (0x00001000) | (1 << 25);
	/* Clear bits 30:31 for HSIC to be enabled */
	status &= ~(0x3 << 30);
	__raw_writel(status, &opregs->portsc[0]);

	/* test mode: force enable hs */
	status = __raw_readl(&opregs->portsc[0]);
	status &= ~(0xf << 16);
	status |= (0x5 << 16);
	__raw_writel(status, &opregs->portsc[0]);

	/* disable test mode */
	status = __raw_readl(&opregs->portsc[0]);
	status &= ~(0xf << 16);
	__raw_writel(status, &opregs->portsc[0]);

	/* check HS ready */
	count = 0x10000;
	do {
		hsic_int = __raw_readl(phyregs + HSIC_INT);
		status = __raw_readl(&opregs->portsc[0]);
		count--;
	} while ((count >= 0) && !(hsic_int & HSIC_INT_HS_READY)
		&& !(hsic_int & HSIC_INT_CONNECT));
	if (count <= 0) {
		printk(KERN_INFO "HSIC_INT_HS_READY not set: hsic_int 0x%x\n",
			hsic_int);
		return -EAGAIN;
	}

	/* issue port reset */
	status = __raw_readl(&opregs->portsc[0]);
	status |= (1<<8);
	__raw_writel(status, &opregs->portsc[0]);

	/* check reset done */
	count = 0x10000;
	do {
		status = __raw_readl(&opregs->portsc[0]);
		count--;
	} while ((count >= 0) && !(status & (1<<8)));
	if (count <= 0) {
		printk(KERN_INFO "port reset not done: portsc 0x%x\n", status);
		return -EAGAIN;
	}
	return 0;
}

#endif  /* CONFIG_USB_EHCI_PXA_U2H_HSIC */

#endif  /* CONFIG_USB_EHCI_PXA */
#endif  /* CONFIG_USB_SUPPORT */
