/*
 * linux/arch/arm/mach-mmp/mmp2.c
 *
 * code name MMP2
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
#include <linux/platform_device.h>
#include <linux/usb/mv_usb.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <asm/hardware/cache-tauros2.h>

#include <asm/mach/time.h>
#include <mach/addr-map.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/irqs.h>
#include <mach/dma.h>
#include <plat/mfp.h>
#include <mach/gpio.h>
#include <mach/devices.h>
#include <mach/mmp2.h>
#include <mach/regs-usb.h>

#include "common.h"
#include "clock.h"

#define MFPR_VIRT_BASE	(APB_VIRT_BASE + 0x1e000)

#define APMASK(i)	(GPIO_REGS_VIRT + BANK_OFF(i) + 0x9c)

static struct mfp_addr_map mmp2_addr_map[] __initdata = {

	MFP_ADDR_X(GPIO0, GPIO58, 0x54),
	MFP_ADDR_X(GPIO59, GPIO73, 0x280),
	MFP_ADDR_X(GPIO74, GPIO101, 0x170),

	MFP_ADDR(GPIO102, 0x0),
	MFP_ADDR(GPIO103, 0x4),
	MFP_ADDR(GPIO104, 0x1fc),
	MFP_ADDR(GPIO105, 0x1f8),
	MFP_ADDR(GPIO106, 0x1f4),
	MFP_ADDR(GPIO107, 0x1f0),
	MFP_ADDR(GPIO108, 0x21c),
	MFP_ADDR(GPIO109, 0x218),
	MFP_ADDR(GPIO110, 0x214),
	MFP_ADDR(GPIO111, 0x200),
	MFP_ADDR(GPIO112, 0x244),
	MFP_ADDR(GPIO113, 0x25c),
	MFP_ADDR(GPIO114, 0x164),
	MFP_ADDR_X(GPIO115, GPIO122, 0x260),

	MFP_ADDR(GPIO123, 0x148),
	MFP_ADDR_X(GPIO124, GPIO141, 0xc),

	MFP_ADDR(GPIO142, 0x8),
	MFP_ADDR_X(GPIO143, GPIO151, 0x220),
	MFP_ADDR_X(GPIO152, GPIO153, 0x248),
	MFP_ADDR_X(GPIO154, GPIO155, 0x254),
	MFP_ADDR_X(GPIO156, GPIO159, 0x14c),

	MFP_ADDR(GPIO160, 0x250),
	MFP_ADDR(GPIO161, 0x210),
	MFP_ADDR(GPIO162, 0x20c),
	MFP_ADDR(GPIO163, 0x208),
	MFP_ADDR(GPIO164, 0x204),
	MFP_ADDR(GPIO165, 0x1ec),
	MFP_ADDR(GPIO166, 0x1e8),
	MFP_ADDR(GPIO167, 0x1e4),
	MFP_ADDR(GPIO168, 0x1e0),

	MFP_ADDR_X(TWSI1_SCL, TWSI1_SDA, 0x140),
	MFP_ADDR_X(TWSI4_SCL, TWSI4_SDA, 0x2bc),

	MFP_ADDR(PMIC_INT, 0x2c4),
	MFP_ADDR(CLK_REQ, 0x160),

	MFP_ADDR_END,
};

void mmp2_clear_pmic_int(void)
{
	unsigned long mfpr_pmic, data;

	mfpr_pmic = APB_VIRT_BASE + 0x1e000 + 0x2c4;
	data = __raw_readl(mfpr_pmic);
	__raw_writel(data | (1 << 6), mfpr_pmic);
	__raw_writel(data, mfpr_pmic);
}

static void __init mmp2_init_gpio(void)
{
	int i;

	/* enable GPIO clock */
	__raw_writel(APBC_APBCLK | APBC_FNCLK, APBC_MMP2_GPIO);

	/* unmask GPIO edge detection for all 6 banks -- APMASKx */
	for (i = 0; i < 6; i++)
		__raw_writel(0xffffffff, APMASK(i));

	pxa_init_gpio(IRQ_MMP2_GPIO, 0, 167, NULL);
}

void __init mmp2_init_irq(void)
{
	mmp2_init_icu();
	mmp2_init_gpio();
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

static struct clk clk_pwm1;
static void pwm2_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	/* FIXME enable pwm1 clock because of dependency */
	clk_enable(&clk_pwm1);

	clk_rst = __raw_readl(clk->clk_rst);

	clk_rst &= ~(APBC_FNCLKSEL(0x7));
	clk_rst |= APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst |= APBC_APBCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	clk_rst &= ~(APBC_RST);
	__raw_writel(clk_rst, clk->clk_rst);
}

static void pwm2_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = __raw_readl(clk->clk_rst);

	clk_rst &= APBC_APBCLK;
	__raw_writel(clk_rst, clk->clk_rst);
	mdelay(1);

	__raw_writel(0, clk->clk_rst);
	mdelay(1);

	/* FIXME disable pwm1 clock because of dependency */
	clk_disable(&clk_pwm1);
}

struct clkops pwm2_clk_ops = {
	.enable		= pwm2_clk_enable,
	.disable	= pwm2_clk_disable,
};

static unsigned long pll_clk_calculate(u32 refdiv, u32 fbdiv)
{
	u32 input_clk;
	u32 output_clk;
	u32 M, N;

	switch (refdiv) {
	case 3:
		M = 5;
		input_clk = 19200000;
		break;
	case 4:
		M = 6;
		input_clk = 26000000;
		break;
	default:
		printk(KERN_WARNING "The PLL REFDIV should be 0x03 or 0x04\n");
		return 0;
	}

	N = fbdiv + 2;
	output_clk = input_clk/10 * N / M *10;
	return output_clk;
}

static unsigned long pll1_get_clk(void)
{
	u32 mpmu_fccr;
	u32 mpmu_posr;
	u32 refdiv;
	u32 fbdiv;
	u32 pll1_en;

	mpmu_fccr = __raw_readl(MPMU_FCCR);
	pll1_en   = (mpmu_fccr >> 14) & 0x1;
	mpmu_posr = __raw_readl(MPMU_POSR);

	if (pll1_en) {
		refdiv    = (mpmu_posr >> 9) & 0x1f;
		fbdiv     = mpmu_posr & 0x1ff;
		return pll_clk_calculate(refdiv, fbdiv);
	} else {
		return 797330000;
	}
}

static unsigned long pll2_get_clk(void)
{
	u32 mpmu_posr;
	u32 refdiv;
	u32 fbdiv;

	mpmu_posr = __raw_readl(MPMU_POSR);
	refdiv      = (mpmu_posr >> 23) & 0x1f;
	fbdiv       = (mpmu_posr >> 14) & 0x1ff;
	return pll_clk_calculate(refdiv, fbdiv);
}

static void disp1_axi_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	/* enable Display1 AXI clock */
	tmp |= (1<<3);
	__raw_writel(tmp, clk->clk_rst);

	/* release from reset */
	tmp |= 1;
	__raw_writel(tmp, clk->clk_rst);
}

static void disp1_axi_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	/* reset display1 AXI clock */
	tmp &= ~1;
	__raw_writel(tmp, clk->clk_rst);

	/* disable display1 AXI clock */
	tmp &= ~(1<<3);
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops disp1_axi_clk_ops = {
	.enable		= disp1_axi_clk_enable,
	.disable	= disp1_axi_clk_disable,
};
static struct clk clk_disp1_axi;

static void lcd_pn1_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	/* enable Display1 peripheral clock */
	tmp |= (1<<4);
	__raw_writel(tmp, clk->clk_rst);

	/* enable DSI clock */
	tmp |= (1<<12) | (1<<5);
	__raw_writel(tmp, clk->clk_rst);

	/* release from reset */
	tmp |= 7;
	__raw_writel(tmp, clk->clk_rst);

	/* enable display1 AXI clock */
	clk_enable(&clk_disp1_axi);
}
static void lcd_pn1_clk_disable(struct clk *clk)
{
	u32 tmp;

	/* reset & disable axi clk first, otherwise panel may resume error */
	clk_disable(&clk_disp1_axi);

	tmp = __raw_readl(clk->clk_rst);
	/* disable DSI clock */
	tmp &= ~((1<<12) | (1<<5));
	__raw_writel(tmp, clk->clk_rst);

	/* disable Display1 peripheral clock */
	tmp &= ~(1<<4);
	__raw_writel(tmp, clk->clk_rst);
}

static int lcd_clk_setrate(struct clk *clk, unsigned long val)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	u32 pll2clk = pll2_get_clk();

	tmp &= ~((0x1f << 15) | (0xf << 8) | (0x3 << 6) | (1<<20));
	tmp |= (0x1a << 15);

	switch (val) {
	case 260000000:
		/* enable PLL1/1.5 divider */
		__raw_writel(0x1, MPMU_PLL1_CTRL);

		/* DSP1clk = PLL1/1.5/2 = 533MHz */
		tmp |= (0x2 << 8) | (0x2 << 6 | 1 << 20);
		break;
	case 400000000:
		/* DSP1clk = PLL1/2 = 400MHz */
		tmp |= (0x2 << 8) | (0x0 << 6);
		break;
	case 520000000:
		/* DSP1clk = PLL2/x = 500MHz */
		pr_debug("%s pll2clk: %d\n\n\n", __func__, pll2clk);
		if (pll2clk > 900000000)
			tmp |= (0x2 << 8) | (0x2 << 6);
		else
			tmp |= (0x1 << 8) | (0x2 << 6);
		break;
	case 800000000:
		/* DSP1clk = PLL1 = 800MHz */
		tmp |= (0x1 << 8) | (0x0 << 6);
		break;

	default:
		printk("%s %d not supported\n", __func__, (int)val);
		return -1;
	}

	__raw_writel(tmp, clk->clk_rst);
	return 0;
}
static unsigned long lcd_clk_getrate(struct clk *clk)
{
	u32 lcdclk = __raw_readl(clk->clk_rst);
	u32 div = (lcdclk >> 8) & 0xf;
	u32 src = (lcdclk >> 6) & 0x3;
	u32 sel = (lcdclk >> 20) & 1;
	u32 pll1clk;
	u32 pll2clk;

	pll1clk = pll1_get_clk();
	pll2clk = pll2_get_clk();

	pr_debug("%s pll1clk %d pll2clk %d\n", __func__, pll1clk, pll2clk);
	if (sel) {
		switch (src) {
			case 2: /* PLL1/1.5 */
				lcdclk = pll1clk*2/3;
				break;
			case 3: /* PLL2/1.5 */
				lcdclk = pll2clk*2/3;
				break;
			case 0: /* USB PLL */
			case 1:
				lcdclk = 480000000;
				break;
			default:
				break;
			}
	} else {
		switch (src) {
			case 0: /* PLL1 */
				lcdclk = pll1clk;
				break;
			case 1: /* PLL1/16 */
				lcdclk = pll1clk/16;
				break;
			case 2: /* PLL2 */
				lcdclk = pll2clk;
				break;
			case 3: /* PLL2/2 */
				lcdclk = 66000000;
				break;
			default:
				break;
			}
	}

	lcdclk /= div;

	pr_debug("lcd clk get %d (clkrst 0x%x div %d src %d sel %d)\n",
		lcdclk, __raw_readl(clk->clk_rst), div, src, sel);
	return lcdclk;
}

struct clkops lcd_pn1_clk_ops = {
	.enable		= lcd_pn1_clk_enable,
	.disable	= lcd_pn1_clk_disable,
	.setrate	= lcd_clk_setrate,
	.getrate	= lcd_clk_getrate,
};

/* APB peripheral clocks */
static APBC_CLK(uart1, MMP2_UART1, 1, 26000000);
static APBC_CLK(uart2, MMP2_UART2, 1, 26000000);
static APBC_CLK(uart3, MMP2_UART3, 1, 26000000);
static APBC_CLK(uart4, MMP2_UART4, 1, 26000000);
static APBC_CLK(twsi1, MMP2_TWSI1, 0, 26000000);
static APBC_CLK(twsi2, MMP2_TWSI2, 0, 26000000);
static APBC_CLK(twsi3, MMP2_TWSI3, 0, 26000000);
static APBC_CLK(twsi4, MMP2_TWSI4, 0, 26000000);
static APBC_CLK(twsi5, MMP2_TWSI5, 0, 26000000);
static APBC_CLK(twsi6, MMP2_TWSI6, 0, 26000000);
static APBC_CLK(pwm1, MMP2_PWM0, 0, 26000000);
static APBC_CLK_OPS(pwm2, MMP2_PWM1, 0, 26000000, &pwm2_clk_ops);
static APBC_CLK(pwm3, MMP2_PWM2, 0, 26000000);
static APBC_CLK(pwm4, MMP2_PWM3, 0, 26000000);

static APMU_CLK(nand, NAND, 0xbf, 100000000);
static APMU_CLK(u2o, USB, 0x9, 480000000);
static APMU_CLK_OPS(sdh0, SDH0, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh1, SDH1, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh2, SDH2, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh3, SDH3, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(disp1_axi, LCD, 0, 0, &disp1_axi_clk_ops);
static APMU_CLK_OPS(lcd, LCD, 0, 0, &lcd_pn1_clk_ops);

static struct clk_lookup mmp2_clkregs[] = {
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
	INIT_CLKREG(&clk_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_sdh0, "sdhci-pxa.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh1, "sdhci-pxa.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh2, "sdhci-pxa.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_sdh3, "sdhci-pxa.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_pwm1, "mmp2-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "mmp2-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "mmp2-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "mmp2-pwm.3", NULL),
	INIT_CLKREG(&clk_disp1_axi, NULL, "DISP1AXICLK"),
	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
};

static int __init mmp2_init(void)
{
	if (cpu_is_mmp2()) {
#ifdef CONFIG_CACHE_TAUROS2
		tauros2_init();
#endif
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(mmp2_addr_map);
		pxa_init_dma(IRQ_MMP2_DMA_RIQ, 16);
		clkdev_add_table(ARRAY_AND_SIZE(mmp2_clkregs));
	}

	return 0;
}
postcore_initcall(mmp2_init);

static void __init mmp2_timer_init(void)
{
	unsigned long clk_rst;

	__raw_writel(APBC_APBCLK | APBC_RST, APBC_MMP2_TIMERS);

	/*
	 * enable bus/functional clock, enable 6.5MHz (divider 4),
	 * release reset
	 */
	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(1);
	__raw_writel(clk_rst, APBC_MMP2_TIMERS);

	timer_init(IRQ_MMP2_TIMER1, IRQ_MMP2_TIMER2);
}

struct sys_timer mmp2_timer = {
	.init	= mmp2_timer_init,
};

/* on-chip devices */
MMP2_DEVICE(uart1, "pxa2xx-uart", 0, UART1, 0xd4030000, 0x30, 4, 5);
MMP2_DEVICE(uart2, "pxa2xx-uart", 1, UART2, 0xd4017000, 0x30, 20, 21);
MMP2_DEVICE(uart3, "pxa2xx-uart", 2, UART3, 0xd4018000, 0x30, 22, 23);
MMP2_DEVICE(uart4, "pxa2xx-uart", 3, UART4, 0xd4016000, 0x30, 18, 19);
MMP2_DEVICE(twsi1, "pxa2xx-i2c", 0, TWSI1, 0xd4011000, 0x70);
MMP2_DEVICE(twsi2, "pxa2xx-i2c", 1, TWSI2, 0xd4031000, 0x70);
MMP2_DEVICE(twsi3, "pxa2xx-i2c", 2, TWSI3, 0xd4032000, 0x70);
MMP2_DEVICE(twsi4, "pxa2xx-i2c", 3, TWSI4, 0xd4033000, 0x70);
MMP2_DEVICE(twsi5, "pxa2xx-i2c", 4, TWSI5, 0xd4033800, 0x70);
MMP2_DEVICE(twsi6, "pxa2xx-i2c", 5, TWSI6, 0xd4034000, 0x70);
MMP2_DEVICE(nand, "pxa3xx-nand", -1, NAND, 0xd4283000, 0x100, 28, 29);
MMP2_DEVICE(sdh0, "sdhci-pxa", 0, MMC, 0xd4280000, 0x120);
MMP2_DEVICE(sdh1, "sdhci-pxa", 1, MMC2, 0xd4280800, 0x120);
MMP2_DEVICE(sdh2, "sdhci-pxa", 2, MMC3, 0xd4281000, 0x120);
MMP2_DEVICE(sdh3, "sdhci-pxa", 3, MMC4, 0xd4281800, 0x120);
MMP2_DEVICE(pwm1, "mmp2-pwm", 0, NONE, 0xd401a000, 0x10);
MMP2_DEVICE(pwm2, "mmp2-pwm", 1, NONE, 0xd401a400, 0x10);
MMP2_DEVICE(pwm3, "mmp2-pwm", 2, NONE, 0xd401a800, 0x10);
MMP2_DEVICE(pwm4, "mmp2-pwm", 3, NONE, 0xd401ac00, 0x10);
MMP2_DEVICE(fb, "pxa168-fb", 0, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(fb_ovly, "pxa168fb_ovly", 0, LCD, 0xd420b000, 0x500);
