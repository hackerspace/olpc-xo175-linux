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
#include <linux/syscore_ops.h>
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
#include <mach/mmp_dma.h>
#include <plat/pmem.h>
#include <mach/soc_vmeta.h>
#include <mach/regs-zsp.h>
#include <mach/mmp-zsp.h>
#include <mach/regs-sspa.h>
#include <mach/regs-icu.h>

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

static int wtm_has_init;
/* wtm clock */
static void wtm_clk_enable(struct clk *clk)
{
	unsigned int val = 0;
	unsigned int enable_flag = (APMU_AXICLK_EN | APMU_FNCLK_EN);

	val = __raw_readl(clk->clk_rst);
	if ((val & enable_flag) == enable_flag) {
		wtm_has_init = 1;
		printk(KERN_INFO "wtm has been enabled by secure core!\n");
		return ;
	}

	val  = APMU_AXICLK_EN;
	__raw_writel(val, clk->clk_rst);
	val |= APMU_AXIRST_DIS;
	__raw_writel(val, clk->clk_rst);
	val |= APMU_FNCLK_EN;
	__raw_writel(val, clk->clk_rst);
	val |= APMU_FNRST_DIS;
	__raw_writel(val, clk->clk_rst);

	return ;
}

static void wtm_clk_disable(struct clk *clk)
{
	unsigned int val;

	if (wtm_has_init)
		return;

	val = __raw_readl(clk->clk_rst);
	val &= ~(APMU_AXICLK_EN);
	val &= ~(APMU_FNCLK_EN);
	__raw_writel(val, clk->clk_rst);

	return ;
}

struct clkops wtm_clk_ops = {
	.enable		= wtm_clk_enable,
	.disable	= wtm_clk_disable,
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

static void gc800_clk_enable(struct clk *clk)
{
	u32 tmp;

	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~(0xf<<4);
	tmp &= ~((1<<12) | (1<<14));

#if 0
	/* FIXME enable OTG PHY clock to feed GC*/
	clk_enable(&clk_usb_phy);
	tmp |= ((2<<4) | (1<<6));
	tmp |= ((1<<12) | (1<<14));
	printk(KERN_DEBUG "Bus Clock: USB PLL 480MHz\n");
	printk(KERN_DEBUG "GC Controller Clock: USB PLL 480MHz\n");
#else
	tmp |= ((0<<4) | (0<<6));
	printk(KERN_DEBUG "Bus Clock: PLL1/4\n");
	printk(KERN_DEBUG "GC Controller Clock: PLL1/2\n");
#endif

	tmp |= (3<<9);
	__raw_writel(tmp, clk->clk_rst);

	tmp |= ((1<<3) | (1<<2));
	__raw_writel(tmp, clk->clk_rst);

	udelay(150);

	tmp |= (1<<8);
	__raw_writel(tmp, clk->clk_rst);

	udelay(1);

	if((tmp & (1<<15)) == 0){
		tmp |= (1<<15);
		__raw_writel(tmp, clk->clk_rst);
	}

	tmp |= (1<<0);
	__raw_writel(tmp, clk->clk_rst);

	udelay(100);

	tmp |= (1<<1);
	__raw_writel(tmp, clk->clk_rst);
	udelay(100);

	return;
}

static void gc800_clk_disable(struct clk *clk)
{
	u32 tmp;

	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~(1<<0);
	__raw_writel(tmp, clk->clk_rst);

	tmp &= ~(1<<1);
	__raw_writel(tmp, clk->clk_rst);

	tmp &= ~(1<<8);
	__raw_writel(tmp, clk->clk_rst);

	tmp &= ~((1<<2) | (1<<3));
	__raw_writel(tmp, clk->clk_rst);

	tmp &= ~(3<<9);
	__raw_writel(tmp, clk->clk_rst);

#if 0
	/* FIXME disable OTG PHY clock */
	clk_disable(&clk_usb_phy);
#endif

	return;
}

static int gc800_clk_setrate(struct clk *clk, unsigned long target_rate)
{
	return 0;
}

static unsigned long gc800_clk_getrate(struct clk *clk)
{
	u32 apmu_gc;
	u32 tmp;
	u32 gc_clk;
	u32 pll1clk;
	u32 pll2clk;

	gc_clk = 0;
	pll1clk = pll1_get_clk();
	pll2clk = pll2_get_clk();

	/* GC800 */
	apmu_gc = readl(APMU_GC);
	if (((apmu_gc >> 9) & 0x3) == 0x3) {
		if((apmu_gc >> 12) & 0x1){
			tmp = (apmu_gc >> 6) & 0x3;
			switch (tmp) {
			case 0: /* PLL2/4 */
				gc_clk = pll2clk/4;
				break;
			case 1: /* USB PLL */
				gc_clk = 480000000;
				break;
			default:
				break;
			}
		}else{
			tmp = (apmu_gc >> 6) & 0x3;
			switch (tmp) {
			case 0: /* PLL1/2 */
				gc_clk = pll1clk/2;
				break;
			case 1: /* PLL2/3 */
				gc_clk = pll2clk/3;
				break;
			case 2: /* PLL2 */
				gc_clk = pll2clk;
				break;
			case 3: /* PLL2/2 */
				gc_clk = pll2clk/2;
				break;
			default:
				break;
			}
		}
	}

	return gc_clk;
}

struct clkops gc800_clk_ops = {
	.enable		= gc800_clk_enable,
	.disable	= gc800_clk_disable,
	.setrate	= gc800_clk_setrate,
	.getrate	= gc800_clk_getrate,
};

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

static void lcd_tv_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);

	tmp |= (0x3 << 13);
	__raw_writel(tmp, clk->clk_rst);

	/* enable display1 AXI clock */
	clk_enable(&clk_disp1_axi);
}

static void lcd_tv_clk_disable(struct clk *clk)
{
	u32 tmp;

	/* reset & disable axi clk */
	clk_disable(&clk_disp1_axi);

	tmp = __raw_readl(clk->clk_rst);
	tmp &= ~(0x3 << 13);
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

struct clkops lcd_tv_clk_ops = {
	.enable		= lcd_tv_clk_enable,
	.disable	= lcd_tv_clk_disable,
};

static int vmeta_usb_phy_clk_enabled;
static int vmeta_usb_phy_clk_enable(int en)
{
	struct clk *clk = clk_get(NULL, "USBPHYCLK");
	if (IS_ERR(clk)) {
		pr_err("%s:failed to get USB PHY clock\n", __func__);
		return PTR_ERR(clk);
	}
	if (en) {
		if (!vmeta_usb_phy_clk_enabled) {
			clk_enable(clk);
			vmeta_usb_phy_clk_enabled = 1;
		}
	} else {
		if (vmeta_usb_phy_clk_enabled) {
			clk_disable(clk);
			vmeta_usb_phy_clk_enabled = 0;
		}
	}
	return 0;
}

static void vmeta_clk_enable(struct clk *clk)
{
	int reg;
	reg = readl(clk->clk_rst);
	if (((reg & APMU_MMP2_VMETA_CLK_SEL_MASK) ==
		(0x5 << APMU_MMP2_VMETA_CLK_SEL_SHIFT)) ||
		((reg & APMU_MMP2_VMETA_ACLK_SEL_MASK) ==
		(0x6 << APMU_MMP2_VMETA_ACLK_SEL_SHIFT))) {
		/* If selected usb pll as clk source, enable usb phy clk */
		vmeta_usb_phy_clk_enable(1);
	}
	/* Peripheral Clock Enable */
	reg |= APMU_MMP2_VMETA_CLK_EN;
	writel(reg, clk->clk_rst);
	/* AXI Clock Enable */
	reg |= APMU_MMP2_VMETA_AXICLK_EN;
	writel(reg, clk->clk_rst);
}

static void vmeta_clk_disable(struct clk *clk)
{
	int reg;
	/* Disable Clock */
	reg = readl(clk->clk_rst);
	reg &= ~APMU_MMP2_VMETA_CLK_EN;
	writel(reg, clk->clk_rst);
	reg &= ~APMU_MMP2_VMETA_AXICLK_EN;
	writel(reg, clk->clk_rst);
	/* Disable USB phy clock */
	vmeta_usb_phy_clk_enable(0);
}

struct clkops vmeta_clk_ops = {
	.enable  = vmeta_clk_enable,
	.disable = vmeta_clk_disable,
};

#ifdef CONFIG_UIO_VMETA
void vmeta_pwr(unsigned int en)
{
	u32 clk_rst = APMU_VMETA_CLK_RES_CTRL;
	int reg = readl(clk_rst);
	if (VMETA_PWR_ENABLE == en) {
		/* Power on */
		/* Enable Hardware power mode */
		reg |= APMU_MMP2_VMETA_PWR_CTRL;
		reg |= APMU_MMP2_VMETA_PWRUP;
		writel(reg, clk_rst);

		reg |= APMU_MMP2_VMETA_INP_ISB;
		writel(reg, clk_rst);

		reg |= APMU_MMP2_VMETA_ISB;
		writel(reg, clk_rst);

		/* Peripheral Reset */
		reg &= ~APMU_MMP2_VMETA_RST1;
		writel(reg, clk_rst);
		reg = readl(clk_rst);
		/* Release from reset */
		reg |= APMU_MMP2_VMETA_RST1;
		writel(reg, clk_rst);
		reg = readl(clk_rst);
		/* Release AXI from reset */
		reg |= APMU_MMP2_VMETA_AXI_RST;
		writel(reg, clk_rst);
		reg = readl(clk_rst);
		/* Select 400M(PLL1/2) Vmeta clock by default when power on,
		 * and it will be updated w/ actual vop passed by vmeta lib */
		reg &= ~APMU_MMP2_VMETA_CLK_SEL_MASK;
		reg &= ~APMU_MMP2_VMETA_ACLK_SEL_MASK;
		reg |= (0 << APMU_MMP2_VMETA_CLK_SEL_SHIFT);
		reg |= (0x5 << APMU_MMP2_VMETA_ACLK_SEL_SHIFT);
		writel(reg, clk_rst);
	} else if (VMETA_PWR_DISABLE == en) {
		/* Power off */
		reg &= ~APMU_MMP2_VMETA_ISB;
		writel(reg, clk_rst);

		reg &= ~APMU_MMP2_VMETA_INP_ISB;
		writel(reg, clk_rst);

		reg &= ~APMU_MMP2_VMETA_PWRUP;
		writel(reg, clk_rst);
		/* Hold vmeta in reset */
		reg = readl(clk_rst);
		reg &= ~APMU_MMP2_VMETA_RST1;
		writel(reg, clk_rst);
	}
}
#endif

static void mmp_zsp_domain_halt(void)
{
	/* reset audio accelerator processor and subsystem */
	int value;
	value = readl(APMU_AUDIO_CLK_RES_CTRL);
	value &= ~(1 << 30);
	writel(value, APMU_AUDIO_CLK_RES_CTRL);
	mdelay(5);
	value = readl(APMU_AUDIO_CLK_RES_CTRL);
	value &= ~(1 << 16);
	writel(value, APMU_AUDIO_CLK_RES_CTRL);
	mdelay(5);
}
static void mmp_zsp_start_core(void)
{
	int value;
	writel(0x0000, ZSP_CONFIG_SVTADDR);
	mdelay(5);
	/* audio accelerator processor release from reset */
	value = readl(APMU_AUDIO_CLK_RES_CTRL) | (1 << 30);
	writel(value, APMU_AUDIO_CLK_RES_CTRL);
	mdelay(5);
}
static void mmp_zsp_domain_on(int spd, int src, int asclk)
{
	int value;
	u32 use_audio_pll;
	u32 core_source;
	u32 core_devider;
	static struct {
		u32 core_source;
		u32 core_devider;
	} core_clock_matrix[MMP_ZSP_CORECLKSRC_NUMBER][MMP_ZSP_SPD_NUMBER] = {
		{ /* use PMU*/
			{ 0, 0},	/* fast, PLL1/2 ~400/1, 400MHz */
			{ 0, 2},	/* medium,  PLL1/2 ~400/3, 133Mhz */
			{ 3, 2},	/* slow,  PLL1/8 ~100/3, 33MHz */
		},
		{ /* use Audio PLL */
			{ 0, 0},	/* fast, /1, ~271/295MHz */
			{ 0, 1},	/* medium, /2, 133Mhz */
			{ 0, 7},	/* slow, /8, ~33MHz */
		},
	};

	/* check and prepare parameters*/
	use_audio_pll = 0x1;
	if ((src != MMP_ZSP_CORECLKSRC_AUDIOPLL)) {
		/*
		HW issue, a0/a1 cannot switch to use audio PLL
		for Z stepping, the code will not reach here, the device
		will not be registered in the case
		let's fix invalid values to use PMU
		*/
		src = MMP_ZSP_CORECLKSRC_PMU;
		use_audio_pll = 0x0;
	}
	if (spd >= MMP_ZSP_SPD_NUMBER) {
		spd = MMP_ZSP_SPD_BALANCED;
	}
	core_source = core_clock_matrix[src][spd].core_source;
	core_devider = core_clock_matrix[src][spd].core_devider;

	/* select core clock source*/
	value = readl(APMU_AUDIO_CLK_RES_CTRL);
	value = mmp_zsp_set_bit_range(value, core_source, 0x3, 6);
	writel(value, APMU_AUDIO_CLK_RES_CTRL);
	mdelay(5);
	/* audio accelerator subsystem release from reset */
	value = readl(APMU_AUDIO_CLK_RES_CTRL) | (1 << 16);
	writel(value, APMU_AUDIO_CLK_RES_CTRL);
	mdelay(5);
	/* bus clock select */
	value = readl(ZSP_CONFIG_REG0) & 0xFFFFBFFF;
	writel(value, ZSP_CONFIG_REG0);
	mdelay(5);

	/* config audio PLL, mclk*2 */
	switch (asclk) {
	case MMP_ZSP_ASCLK_22579200: /* 44.1k group */
		/* 11289600 mclk */
		writel(0x108A1881, SSPA_AUD_PLL_CTRL0);
		writel(0x30801, SSPA_AUD_PLL_CTRL1);
		break;
	case MMP_ZSP_ASCLK_24576000: /* 48k group */
	default:
		/* 12288000 mclk*/
		writel(0x100DA189, SSPA_AUD_PLL_CTRL0);
		writel(0x30801, SSPA_AUD_PLL_CTRL1);
		break;
	}

	/* set up core clock source */
	/* step 1, set core clock divider*/
	value = readl(ZSP_CONFIG_REG0);
	value = mmp_zsp_set_bit_range(value, core_devider, 0x7, 2);
	writel(value, ZSP_CONFIG_REG0);
	mdelay(5);
	/* step 2, audio PLL or PMU*/
	value = readl(ZSP_CONFIG_REG0);
	value = mmp_zsp_set_bit_range(value, use_audio_pll, 0x1, 0);
	writel(value, ZSP_CONFIG_REG0);
	mdelay(5);

	/* set up interrupts */
	writel(0x003c0000, ICU_DMAIRQ_MASK); /* to ZSP not PJ */

	/* only route IPC interrupt to host, mask others */
	writel(0x39ff, ZSP_INT_MASK0);
	writel(0x3fff, ZSP_INT_MASK1);

	printk("%s: CLKRESCTL: 0x%08x, CONFIG0:0x%08x, PLL:0x%08x-0x%08x\n"
			, __func__
			, readl(APMU_AUDIO_CLK_RES_CTRL)
			, readl(ZSP_CONFIG_REG0)
			, readl(SSPA_AUD_PLL_CTRL0)
			, readl(SSPA_AUD_PLL_CTRL1)
		);

}

static struct mmp_zsp_platform_device mmp_zsp_op = {
	.clkcfg = {
		.spd	= MMP_ZSP_SPD_PERFORMACE,
		.src	= MMP_ZSP_CORECLKSRC_AUDIOPLL,
		.asclk	= MMP_ZSP_ASCLK_24576000,
	},
	.domain_halt	= mmp_zsp_domain_halt,
	.domain_on	= mmp_zsp_domain_on,
	.start_core	= mmp_zsp_start_core,
};
static struct resource mmp_zsp_resources[] = {
	/* reg base:*/
	[0] = {
		.start	= 0xd42a0000,
		.end	= 0xd42a1fff,
		.flags	= IORESOURCE_MEM,
		.name	= "audio-device",
	},
	/* ZSP iTCM window */
	[1] = {
		.start	= 0xc0040000,
		.end	= 0xc004ffff,
		.flags	= IORESOURCE_MEM,
		.name	= "audio-itcm",
	},
	/* ZSP dTCM window */
	[2] = {
		.start	= 0xc0000000,
		.end	= 0xc001ffff,
		.flags	= IORESOURCE_MEM,
		.name	= "audio-dtcm",
	},
	/* Audio IPC IRQ*/
	[3] = {
		.start	= IRQ_MMP2_PZIPC,
		.end	= IRQ_MMP2_PZIPC,
		.flags	= IORESOURCE_IRQ,
		.name	= "audio-irq",
	},
};
static u64 mmp_zsp_dmamask = DMA_BIT_MASK(32);
static struct platform_device mmp_zsp_device = {
	.name		= "mmp-zsp",
	.id		= -1,
	.dev		= {
		.dma_mask		= &mmp_zsp_dmamask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
	.num_resources	= ARRAY_SIZE(mmp_zsp_resources),
	.resource	= mmp_zsp_resources,
};
void mmp_zsp_platform_device_init(void)
{
	struct platform_device *pdev = &mmp_zsp_device;
	int ret;

	if (cpu_is_mmp2()) {

		ret = platform_device_add_data(pdev, &mmp_zsp_op,
			sizeof(mmp_zsp_op));
		if (ret) {
			printk("mmp_zsp_platform_device_init fail: %d\n", ret);
			return;
		}
		platform_device_register(pdev);
		printk("ZSP device registered\n");
	} else {
		printk("ZSP device does not exist\n");
	}
}

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
static APBC_CLK(thsens, MMP2_THSENS1, 0, 0);
static APBC_CLK(pwm1, MMP2_PWM0, 0, 26000000);
static APBC_CLK_OPS(pwm2, MMP2_PWM1, 0, 26000000, &pwm2_clk_ops);
static APBC_CLK(pwm3, MMP2_PWM2, 0, 26000000);
static APBC_CLK(pwm4, MMP2_PWM3, 0, 26000000);
static APBC_CLK(ssp3, MMP2_SSP3, 1, 13000000);
static APBC_CLK(keypad, MMP2_KPC, 0, 32768);
static APBC_CLK(rtc, MMP2_RTC, 0x8, 32768);

static APMU_CLK(nand, NAND, 0xbf, 100000000);
static APMU_CLK(u2o, USB, 0x9, 480000000);
static APMU_CLK_OPS(sdh0, SDH0, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh1, SDH1, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh2, SDH2, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(sdh3, SDH3, 0x1b, 200000000, &sdhc_clk_ops);
static APMU_CLK_OPS(gc, GC, 0, 0, &gc800_clk_ops);
static APMU_CLK_OPS(disp1_axi, LCD, 0, 0, &disp1_axi_clk_ops);
static APMU_CLK_OPS(lcd, LCD, 0, 0, &lcd_pn1_clk_ops);
static APMU_CLK_OPS(tv, LCD, 0, 0, &lcd_tv_clk_ops);
static APMU_CLK_OPS(vmeta, VMETA_CLK_RES_CTRL, 0, 0, &vmeta_clk_ops);
static AUD_CLK_OPS(audio, &audio_clk_ops);
static AUD_CLK_OPS(sspa1, &sspa1_clk_ops);
static AUD_CLK_OPS(sspa2, &sspa2_clk_ops);
static AUD_CLK_OPS(sysclk, &sysclk_ops);

/* wtm clock */
static APMU_CLK_OPS(wtm, GEU, 0, 0, &wtm_clk_ops);

/* USB PHY PLL clock enable/disable */
extern void pxa_usb_phy_clk_enable(void);
extern void pxa_usb_phy_clk_disable(void);
static void usb_phy_clk_enable(struct clk *clk)
{
	clk_enable(&clk_u2o);
	pxa_usb_phy_clk_enable();
	clk_disable(&clk_u2o);
}

static void usb_phy_clk_disable(struct clk *clk)
{
	clk_enable(&clk_u2o);
	pxa_usb_phy_clk_disable();
	clk_disable(&clk_u2o);
}

static struct clkops usb_phy_clk_ops = {
	.enable		= usb_phy_clk_enable,
	.disable	= usb_phy_clk_disable,
};

static struct clk clk_usb_phy = {
	.ops = &usb_phy_clk_ops,
};

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
	INIT_CLKREG(&clk_gc, NULL, "GCCLK"),
	INIT_CLKREG(&clk_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_pwm1, "mmp2-pwm.0", NULL),
	INIT_CLKREG(&clk_pwm2, "mmp2-pwm.1", NULL),
	INIT_CLKREG(&clk_pwm3, "mmp2-pwm.2", NULL),
	INIT_CLKREG(&clk_pwm4, "mmp2-pwm.3", NULL),
	INIT_CLKREG(&clk_disp1_axi, NULL, "DISP1AXICLK"),
	INIT_CLKREG(&clk_lcd, NULL, "LCDCLK"),
	INIT_CLKREG(&clk_tv, NULL, "HDMICLK"),
	INIT_CLKREG(&clk_audio, NULL, "mmp-audio"),
	INIT_CLKREG(&clk_thsens, "mmp2-thermal", NULL),
	INIT_CLKREG(&clk_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_vmeta, NULL, "VMETA_CLK"),
	INIT_CLKREG(&clk_wtm, NULL, "mmp2-wtm"),
	INIT_CLKREG(&clk_rtc, "mmp-rtc", NULL),
	INIT_CLKREG(&clk_usb_phy, NULL, "USBPHYCLK"),
	INIT_CLKREG(&clk_ssp3, "mmp-ssp.3", NULL),
	INIT_CLKREG(&clk_sysclk, NULL, "mmp-sysclk"),
	INIT_CLKREG(&clk_sspa1, "mmp2-sspa.0", NULL),
	INIT_CLKREG(&clk_sspa2, "mmp2-sspa.1", NULL),
};

#define MCB_SLFST_SEL		0xD0000570
#define MCB_SLFST_CTRL2		0xD00005A0
void __init mmp2_init_mcb(void)
{
	void *mcb_slfst_sel = ioremap(MCB_SLFST_SEL, 4);
	void *mcb_slfst_ctrl2 = ioremap(MCB_SLFST_CTRL2, 4);

	/* select MCB1 */
	__raw_writel(0x1, mcb_slfst_sel);
	iounmap(mcb_slfst_sel);

	/* to reduce LCD underflow, priority enable stage 2,
	 * fast has higher priority then slow */
	__raw_writel(0xc0000, mcb_slfst_ctrl2);
	iounmap(mcb_slfst_ctrl2);
}

extern struct syscore_ops pxa_audio_syscore_ops;
static int __init mmp2_init(void)
{
	if (cpu_is_mmp2()) {
#ifdef CONFIG_CACHE_TAUROS2
		tauros2_init();
#endif
		mfp_init_base(MFPR_VIRT_BASE);
		mfp_init_addr(mmp2_addr_map);
		pxa_init_dma(IRQ_MMP2_DMA_RIQ, 16);
		mmp_init_dma(IRQ_MMP2_DMA_RIQ);
		mmp2_init_mcb();
		clkdev_add_table(ARRAY_AND_SIZE(mmp2_clkregs));
		register_syscore_ops(&pxa_audio_syscore_ops);
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

void __init mmp2_reserve(void)
{
	/*reserve memory for pmem*/
	pxa_reserve_pmem_memblock();
}

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
MMP2_DEVICE(fb_tv, "pxa168-fb", 1, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(fb_tv_ovly, "pxa168fb_ovly", 1, LCD, 0xd420b000, 0x500);
MMP2_DEVICE(hdmi, "mmp-hdmi", -1, HDMI, 0xd420b000, 0x1fff);
MMP2_DEVICE(sspa1, "mmp2-sspa", 0, SSPA1, 0xd42a0c00, 0xb0, ADMA1_CH1, ADMA1_CH0);
MMP2_DEVICE(sspa2, "mmp2-sspa", 1, SSPA2, 0xd42a0d00, 0xb0, ADMA2_CH1, ADMA2_CH0);
MMP2_DEVICE(ssp1, "mmp-ssp", 1, SSP1, 0xd4035000, 0x40, 6, 7);
MMP2_DEVICE(ssp2, "mmp-ssp", 2, SSP2, 0xd4036000, 0x40, 10, 11);
MMP2_DEVICE(ssp3, "mmp-ssp", 3, SSP3, 0xd4037000, 0x40, 12, 13);
MMP2_DEVICE(ssp4, "mmp-ssp", 4, SSP4, 0xd4039000, 0x40, 14, 15);
MMP2_DEVICE(audiosram, "mmp-sram", 0, NONE, 0xe0000000, 0x4000);
MMP2_DEVICE(videosram, "mmp-sram", 1, NONE, 0xd1020000, 0x16800);
MMP2_DEVICE(thsens, "mmp2-thermal", -1, THERMAL, 0xd4013200, 0x20);
MMP2_DEVICE(keypad, "pxa27x-keypad", -1, KPC, 0xd4012000, 0x4c);
MMP2_DEVICE(fuse, "mmp2-fuse", -1, NONE, 0xd4290000, 0x3100);

struct platform_device mmp_device_asoc_sspa1 = {
	.name		= "mmp-sspa-dai",
	.id		= 0,
};

struct platform_device mmp_device_asoc_sspa2 = {
	.name		= "mmp-sspa-dai",
	.id		= 1,
};

struct platform_device mmp_device_asoc_platform = {
	.name		= "mmp-pcm-audio",
	.id		= -1,
};

struct platform_device mmp_device_asoc_hdmi = {
	.name		= "dummy-codec",
	.id		= -1,
};

static struct resource mmp2_resource_rtc[] = {
	[0] = {
		.start  = 0xd4010000,
		.end    = 0xD40100ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_MMP2_RTC,
		.end    = IRQ_MMP2_RTC,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_1HZ",
	},

	[2] = {
		.start  = IRQ_MMP2_RTC_ALARM,
		.end    = IRQ_MMP2_RTC_ALARM,
		.flags  = IORESOURCE_IRQ,
		.name   = "RTC_ALARM",
	},

};

struct platform_device mmp2_device_rtc = {
	.name           = "mmp-rtc",
	.id             = -1,
	.resource       = mmp2_resource_rtc,
	.num_resources  = ARRAY_SIZE(mmp2_resource_rtc),
};

/* Update vmeta clock source and freqency according to vop */
int mmp_update_vmeta_clk(struct vmeta_instance *vi)
{
	u32 clk_rst = APMU_VMETA_CLK_RES_CTRL;
	int reg;
	if (!vi)
		return -1;
	reg = readl(clk_rst);
	/* Clock source select */
	reg &= ~APMU_MMP2_VMETA_CLK_SEL_MASK;
	reg &= ~APMU_MMP2_VMETA_ACLK_SEL_MASK;
	if (vi->vop <= VMETA_OP_720P_MAX) {
		/* 400M: PLL1 devided by 2 */
		reg |= (0 << APMU_MMP2_VMETA_CLK_SEL_SHIFT);
		reg |= (0x5 << APMU_MMP2_VMETA_ACLK_SEL_SHIFT);
	} else {
		/* 480M: USB PHY PLL */
		reg |= (0x5 << APMU_MMP2_VMETA_CLK_SEL_SHIFT);
		reg |= (0x6 << APMU_MMP2_VMETA_ACLK_SEL_SHIFT);
	}
	writel(reg, clk_rst);
	return 0;
}
