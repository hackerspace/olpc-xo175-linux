/*
 *  linux/arch/arm/mach-mmp/clock-mmp3.c
 *
 *  Author:	Raul Xiong <xjian@marvell.com>
 *		Alan Zhu <wzhu10@marvell.com>
 *  Copyright:	(C) 2011 Marvell International Ltd.
 *
 *  based on arch/arm/mach-tegra/tegra2_clocks.c
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <mach/mmp3_pm.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <plat/clock.h>
#include "common.h"

#define CORE_NUM			3
#define PMUA_CC				APMU_REG(0x4)
#define PMUA_CC_2			APMU_REG(0x150)
#define PMUA_CC_3			APMU_REG(0x188)
#define PMUA_BUS			APMU_REG(0x6c)
#define PMUA_DM_CC			APMU_REG(0xc)
#define PMUA_DM_2_CC			APMU_REG(0x158)

#define MMP3_PROTECT_CC(x)		(((x) & 0x0003fe3f) | 0x00bc0000)
#define MMP3_PROTECT_CC2(x)		((x) & 0xfffffe07)
#define MMP3_PROTECT_CC3(x)		((x) & 0x0effff1f)
#define MMP3_PROTECT_BUSCLKRST(x)	((x) & 0x000001c3)
#define MMP3_PROTECT_FCCR(x)		((x) & 0xff83ffff)

/* Dynamic Frequency Change Part */
enum {
	MMP3_FREQ_OP_GET = 0,
	MMP3_FREQ_OP_UPDATE = 1,
	MMP3_FREQ_OP_SHOW = 2,

	MMP3_FREQ_PH_D_BY_4 = 1,
	MMP3_FREQ_PH_D_BY_6 = 2,
	MMP3_FREQ_PH_D_BY_8 = 3,
	MMP3_FREQ_PH_D_BY_10 = 4,
	MMP3_FREQ_PH_D_BY_12 = 5,
	MMP3_FREQ_PH_D_BY_14 = 6,
	MMP3_FREQ_PH_D_BY_15 = 7,

	MMP3_FREQCH_VOLTING = (1u << 27),
	MMP3_FREQCH_CORE = (1u << 24),
	MMP3_FREQCH_DRAM = (9u << 22),	/* both channel */
	MMP3_FREQCH_AXI = (1u << 26),
};

static struct clk mmp3_clk_pll1_d_2 = {
	.name = "pll1_d_2",
	.rate = 400000000,
	.ops = NULL,
};

static struct clk mmp3_clk_pll1 = {
	.name = "pll1",
	.rate = 800000000,
	.ops = NULL,
};

static int mmp3_clk_pll2_enable(struct clk *clk)
{
	u32 value;

	/* disable PLL2 */
	value = __raw_readl(MPMU_PLL2CR);
	value &= ~(1 << 8);
	__raw_writel(value, MPMU_PLL2CR);

	/* select VCO as clock source */
	value = __raw_readl(MPMU_PLL2_CTRL2);
	value |= 1 << 0;
	__raw_writel(value, MPMU_PLL2_CTRL2);

	/*
	 * PLL2 control register 1 - program VCODIV_SEL_SE = 0,
	 * ICP = 4, KVCO = 1 and VCRNG = 0
	 */
	__raw_writel(0x01090099, MPMU_PLL2_CTRL1);
	/* MPMU_PLL2CR: Program PLL2 VCO for 1334Mhz -REFD = 3, FBD = 0x9A */
	__raw_writel(0x001A6A00, MPMU_PLL2CR);

	/* enable PLL2 */
	value = __raw_readl(MPMU_PLL2CR);
	value |= 1 << 8;
	__raw_writel(value, MPMU_PLL2CR);

	udelay(500);

	/* take PLL2 out of reset */
	value = __raw_readl(MPMU_PLL2_CTRL1);
	value |= 1 << 29;
	__raw_writel(value, MPMU_PLL2_CTRL1);

	udelay(500);

	return 0;
}

static void mmp3_clk_pll2_disable(struct clk *clk)
{
	u32 value;

	/* PLL2 Control register, disable SW PLL2 */
	value = __raw_readl(MPMU_PLL2CR);
	value &= ~(1 << 8);
	__raw_writel(value, MPMU_PLL2CR);

	/* wait for PLLs to lock */
	udelay(500);

	/* MPMU_PLL2_CTRL1: put PLL2 into reset */
	value = __raw_readl(MPMU_PLL2_CTRL1);
	value = ~(1 << 29);
	__raw_writel(value, MPMU_PLL2_CTRL1);

	udelay(500);
}

static struct clkops mmp3_clk_pll2_ops = {
	.enable = mmp3_clk_pll2_enable,
	.disable = mmp3_clk_pll2_disable,
};

static struct clk mmp3_clk_pll2 = {
	.name = "pll2",
	.rate = 1334000000,
	.ops = &mmp3_clk_pll2_ops,
};

static int mmp3_clk_pll1_clkoutp_enable(struct clk *clk)
{
	u32 value = __raw_readl(PMUM_PLL_DIFF_CTRL);
	/* Set PLL1 CLKOUTP post VCO divider as 1.5 */
	value &= ~(0xf << 0);
	value |= 1 << 0;
	value |= 1 << 4;
	__raw_writel(value, PMUM_PLL_DIFF_CTRL);

	return 0;
}

static void mmp3_clk_pll1_clkoutp_disable(struct clk *clk)
{
	u32 value = __raw_readl(PMUM_PLL_DIFF_CTRL);
	__raw_writel(value & ~(1 << 4), PMUM_PLL_DIFF_CTRL);
}

static struct clkops mmp3_clk_pll1_clkoutp_ops = {
	.enable = mmp3_clk_pll1_clkoutp_enable,
	.disable = mmp3_clk_pll1_clkoutp_disable,
};

/*
 * NOTE: actually pll1_clkoutp and pll1 share the same clock source
 * pll1_VCO, which is 1600MHz. And pll1 has a post didiver 2, pll1_clkoutp
 * has a post didiver 1.5. Since we don't expose pll1_VCO as a visible
 * clock, here we use pll1 as its parent to workaround. So we have to
 * set the div as 3 and mul as 4.
 */
static struct clk mmp3_clk_pll1_clkoutp = {
	.name = "pll1_clkoutp",
	.rate = 1066000000,
	.parent = &mmp3_clk_pll1,
	.mul = 4,
	.div = 3,
	.ops = &mmp3_clk_pll1_clkoutp_ops,
};

static struct clk mmp3_clk_vctcxo = {
	.name = "vctcxo",
	.rate = 26000000,
	.ops = NULL,
};

static struct clk mmp3_clk_32k = {
	.name = "32k",
	.rate = 32768,
	.ops = NULL,
};

static struct clk mmp3_clk_pll1_d_4 = {
	.name = "pll1_d_4",
	.rate = 200000000,
	.ops = NULL,
};

static int mmp3_clk_pll3_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CTRL2);

	/* set SEL_VCO_CLK_SE in PMUM_PLL3_CTRL2 register */
	__raw_writel(tmp | 0x00000001, PMUM_PLL3_CTRL2);

	/*
	 * PLL3 control register 1 - program VCODIV_SEL_SE = 2,
	 * ICP = 4, KVCO = 5 and VCRNG = 4
	 */
	__raw_writel(0x05290499, PMUM_PLL3_CTRL1);

	/* MPMU_PLL3CR: Program PLL3 VCO for 2.0Ghz -REFD = 3 */
	tmp = (__raw_readl(APMU_FSIC3_CLK_RES_CTRL) >> 8) & 0xF;
	if (tmp == 0xD)
		/*
		 * 26MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xE6
		 */
		__raw_writel(0x001B9A00, PMUM_PLL3_CR);
	else
		/*
		 * 25MHz ref clock to HDMI\DSI\USB PLLs,
		 * MPMU_PLL3CR ;FBD = 0xF6
		 */
		__raw_writel(0x001BdA00, PMUM_PLL3_CR);

	/* PLL3 Control register -Enable SW PLL3 */
	tmp = __raw_readl(PMUM_PLL3_CR);
	__raw_writel(tmp | 0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock */
	udelay(500);

	/* PMUM_PLL3_CTRL1: take PLL3 out of reset */
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp | 0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);

	return 0;
}

static void mmp3_clk_pll3_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(PMUM_PLL3_CR);

	/* PLL3 Control register, disable SW PLL3 */
	__raw_writel(tmp & ~0x00000100, PMUM_PLL3_CR);

	/* wait for PLLs to lock */
	udelay(500);

	/* PMUM_PLL3_CTRL1: put PLL3 into reset */
	tmp = __raw_readl(PMUM_PLL3_CTRL1);
	__raw_writel(tmp & ~0x20000000, PMUM_PLL3_CTRL1);

	udelay(500);
}

static struct clkops mmp3_clk_pll3_ops = {
	.enable = mmp3_clk_pll3_enable,
	.disable = mmp3_clk_pll3_disable,
};

static struct clk mmp3_clk_pll3 = {
	.name = "pll3",
	.rate = 1000000000,
	.ops = &mmp3_clk_pll3_ops,
};

static struct clk_mux_sel mux_pll1_pll2_vctcxo[] = {
	{.input = &mmp3_clk_pll1_d_2, .value = 0},
	{.input = &mmp3_clk_pll1, .value = 1},
	{.input = &mmp3_clk_pll2, .value = 2},
	{.input = &mmp3_clk_pll1_clkoutp, .value = 3},
	{.input = &mmp3_clk_vctcxo, .value = 4},
	{0, 0},
};

static void mmp3_core_clk_trigger_change(void)
{
	u32 val;
	val = __raw_readl(PMUA_CC);
	val = MMP3_PROTECT_CC(val);	/* set reserved */
	val = val | MMP3_FREQCH_CORE | MMP3_FREQCH_VOLTING;
	/* A0 need to all cores run, we use coherent broadcasts */
	dsb();
	/* trigger change */
	__raw_writel(val, PMUA_CC);
	/* done, PJ_RD_STATUS should have been cleared by HW */
}

static void mmp3_clk_source_init(struct clk *c)
{
	u32 val, source;
	const struct clk_mux_sel *sel;

	c->dynamic_change = 1;
	c->mul = 1;
	c->div = 1;

	val = __raw_readl(c->reg_data[SOURCE][STATUS].reg);
	source = (val >> c->reg_data[SOURCE][STATUS].reg_shift)
	    & c->reg_data[SOURCE][STATUS].reg_mask;
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->value == source)
			break;
	}
	BUG_ON(sel->input == NULL);
	c->parent = sel->input;
}

static int mmp3_clk_set_parent(struct clk *c, struct clk *p)
{
	u32 val;
	const struct clk_mux_sel *sel;

	val = __raw_readl(c->reg_data[SOURCE][CONTROL].reg);
	for (sel = c->inputs; sel->input != NULL; sel++) {
		if (sel->input == p) {
			if (c->reg_data[SOURCE][CONTROL].reg == MPMU_FCCR)
				val = MMP3_PROTECT_FCCR(val);
			else if (c->reg_data[SOURCE][CONTROL].reg == PMUA_BUS)
				val = MMP3_PROTECT_BUSCLKRST(val);
			val &= ~(c->reg_data[SOURCE][CONTROL].reg_mask
				 << c->reg_data[SOURCE][CONTROL].reg_shift);
			val |= sel->value
			    << c->reg_data[SOURCE][CONTROL].reg_shift;
			__raw_writel(val, c->reg_data[SOURCE][CONTROL].reg);
			/*
			 * FIXME: DO NOT trigger here since
			 * we will triggered the changes together later
			 */
			/* mmp3_core_clk_trigger_change(); */
			clk_reparent(c, p);
			return 0;
		}
	}

	return -EINVAL;
}

static struct clkops mmp3_clk_root_ops = {
	.init = mmp3_clk_source_init,
	.set_parent = mmp3_clk_set_parent,
};

static struct clk mmp3_clk_core_root = {
	.name = "core_root",
	.inputs = mux_pll1_pll2_vctcxo,
	.ops = &mmp3_clk_root_ops,
	.reg_data = {
		     { {MPMU_FCCR, 29, 0x7}, {MPMU_FCCR, 29, 0x7} },
		     { {0, 0, 0}, {0, 0, 0} } },
};

static void mmp3_clk_div_init(struct clk *c)
{
	u32 val;

	c->dynamic_change = 1;
	c->mul = 1;

	val = __raw_readl(c->reg_data[DIV][STATUS].reg);
	c->div = ((val >> c->reg_data[DIV][STATUS].reg_shift)
		  & c->reg_data[DIV][STATUS].reg_mask) + 1;
}

static int mmp3_clk_set_rate(struct clk *c, unsigned long rate)
{
	int i;
	int max_div = c->reg_data[DIV][CONTROL].reg_mask + 1;
	u32 val = __raw_readl(c->reg_data[DIV][CONTROL].reg);
	unsigned long parent_rate = clk_get_rate(c->parent);

	for (i = 1; i <= max_div; i++) {
		if (rate == parent_rate / i) {
			if (c->reg_data[DIV][CONTROL].reg == PMUA_CC)
				val = MMP3_PROTECT_CC(val);
			else if (c->reg_data[DIV][CONTROL].reg == PMUA_CC_2)
				val = MMP3_PROTECT_CC2(val);
			else if (c->reg_data[DIV][CONTROL].reg == PMUA_CC_3)
				val = MMP3_PROTECT_CC3(val);

			val &= ~(c->reg_data[DIV][CONTROL].reg_mask
				 << c->reg_data[DIV][CONTROL].reg_shift);
			val |= (i - 1) << c->reg_data[DIV][CONTROL].reg_shift;
			__raw_writel(val, c->reg_data[DIV][CONTROL].reg);
			/*
			 * FIXME: DO NOT trigger here since
			 * we will triggered the changes together later
			 */
			/* mmp3_core_clk_trigger_change(); */
			c->div = i;
			c->mul = 1;
			return 0;
		}
	}
	return -EINVAL;
}

static struct clkops mmp3_clk_div_ops = {
	.init = mmp3_clk_div_init,
	.setrate = mmp3_clk_set_rate,
};

static struct clk mmp3_clk_virtual_pj = {
	.name = "pj",
	.parent = &mmp3_clk_core_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_CC, 0, 0x7}, {PMUA_CC, 0, 0x7} } },
};

static struct clk mmp3_clk_mp1 = {
	.name = "mp1",
	.parent = &mmp3_clk_virtual_pj,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 9, 0xf}, {PMUA_CC_2, 9, 0xf} } },
};

static struct clk mmp3_clk_mp2 = {
	.name = "mp2",
	.parent = &mmp3_clk_virtual_pj,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 13, 0xf}, {PMUA_CC_2, 13, 0xf} } },
};

static struct clk mmp3_clk_mm = {
	.name = "mm",
	.parent = &mmp3_clk_virtual_pj,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 17, 0xf}, {PMUA_CC_2, 17, 0xf} } },
};

static struct clk mmp3_clk_aclk = {
	.name = "aclk",
	.parent = &mmp3_clk_virtual_pj,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 21, 0xf}, {PMUA_CC_2, 21, 0xf} } },
};

static void mmp3_core_periph_init(struct clk *c)
{
	u32 val, div_val;

	c->dynamic_change = 1;
	c->mul = 1;

	val = __raw_readl(c->reg_data[DIV][STATUS].reg);
	div_val = (val >> c->reg_data[DIV][STATUS].reg_shift)
	    & c->reg_data[DIV][STATUS].reg_mask;
	if (div_val != 7)
		c->div = (div_val + 1) * 2;
	else
		c->div = 15;
}

static int mmp3_core_periph_set_rate(struct clk *c, unsigned long rate)
{
	int i;
	int max_div = 15;
	u32 val = __raw_readl(c->reg_data[DIV][CONTROL].reg);
	unsigned long parent_rate = clk_get_rate(c->parent);

	for (i = 4; i < max_div; i += 2) {
		if (rate == parent_rate / i) {
			val = MMP3_PROTECT_CC(val);
			val &= ~(c->reg_data[DIV][CONTROL].reg_mask
				 << c->reg_data[DIV][CONTROL].reg_shift);
			val |= (i / 2 - 1)
			    << c->reg_data[DIV][CONTROL].reg_shift;
			__raw_writel(val, c->reg_data[DIV][CONTROL].reg);
			/*
			 * FIXME: DO NOT trigger here since
			 * we will triggered the changes together later
			 */
			/* mmp3_core_clk_trigger_change(); */
			c->div = i;
			c->mul = 1;
			return 0;
		}
	}

	if (rate == parent_rate / max_div) {
		val = MMP3_PROTECT_CC(val);
		val &= ~(c->reg_data[DIV][CONTROL].reg_mask
			 << c->reg_data[DIV][CONTROL].reg_shift);
		val |= 7 << c->reg_data[DIV][CONTROL].reg_shift;
		__raw_writel(val, c->reg_data[DIV][CONTROL].reg);
		/*
		 * FIXME: DO NOT trigger here since
		 * we will triggered the changes together later
		 */
		/* mmp3_core_clk_trigger_change(); */
		c->div = max_div;
		c->mul = 1;
		return 0;
	}

	return -EINVAL;
}

static struct clkops mmp3_core_periph_ops = {
	.init = mmp3_core_periph_init,
	.setrate = mmp3_core_periph_set_rate,
};

static struct clk mmp3_clk_core_periph = {
	.name = "periph",
	.parent = &mmp3_clk_core_root,
	.ops = &mmp3_core_periph_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 25, 0x7}, {PMUA_CC, 9, 0x7} } },
};

static struct clk mmp3_clk_atclk = {
	.name = "atclk",
	.parent = &mmp3_clk_core_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_CC, 3, 0x7}, {PMUA_CC, 3, 0x7} } },
};

static struct {
	struct clk *source_clk;
	unsigned long pj_rate;
	unsigned long mp1_rate;
	unsigned long mp2_rate;
	unsigned long mm_rate;
	unsigned long aclk_rate;
	unsigned long ph_rate;
	unsigned long atclk_rate;
} mmp3_op_table[] = {
	{
	&mmp3_clk_pll1_d_2, 200000000, 200000000,
		    200000000, 200000000, 200000000, 100000000, 200000000}, {
	&mmp3_clk_pll1, 400000000, 400000000,
		    400000000, 400000000, 400000000, 200000000, 400000000}, {
	&mmp3_clk_pll1, 800000000, 800000000,
		    800000000, 400000000, 400000000, 200000000, 400000000}, {
	&mmp3_clk_pll1_clkoutp, 1062000000, 1062000000,
		    1062000000, 531000000, 531000000, 177000000, 354000000}, {
	    /* must be the last line! */
	NULL, 0, 0, 0, 0, 0, 0, 0} };

static void mmp3_cpu_clk_init(struct clk *c)
{
	c->dynamic_change = 1;
	c->mul = 1;
	c->div = 1;
}

static int mmp3_cpu_clk_set_rate(struct clk *c, unsigned long rate)
{
	int ret, index, i;

	for (i = 0; mmp3_op_table[i].source_clk != NULL; i++) {
		if (mmp3_op_table[i].mp1_rate >= rate) {
			index = i;
			break;
		}
	}
	if (mmp3_op_table[i].source_clk == NULL)
		return -EINVAL;

	/* obtain FC onwership, should always pass */
	i = 1000;
	while ((__raw_readl(PMUA_DM_CC) & (1u << 24)) != 0) {
		i--;
		if (i <= 0) {
			pr_err("Cannot gain owner of PMU DFC\n");
			return -EAGAIN;
		}
	}

	ret = clk_set_parent(&mmp3_clk_core_root,
			     mmp3_op_table[index].source_clk);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_virtual_pj, mmp3_op_table[index].pj_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_mp1, mmp3_op_table[index].mp1_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_mp2, mmp3_op_table[index].mp2_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_mm, mmp3_op_table[index].mm_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_aclk, mmp3_op_table[index].aclk_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_core_periph, mmp3_op_table[index].ph_rate);
	if (ret)
		return ret;

	ret = clk_set_rate(&mmp3_clk_atclk, mmp3_op_table[index].atclk_rate);
	if (ret)
		return ret;

	mmp3_core_clk_trigger_change();

	return 0;
}

static struct clkops mmp3_cpu_ops = {
	.init = mmp3_cpu_clk_init,
	.setrate = mmp3_cpu_clk_set_rate,
};

static struct clk mmp3_clk_virtual_cpu __maybe_unused = {
	.name = "cpu",
	.parent = &mmp3_clk_virtual_pj,
	.ops = &mmp3_cpu_ops,
};

/* ddr clock definitions */
static struct clk mmp3_clk_ddr_root = {
	.name = "ddr_root",
	.inputs = mux_pll1_pll2_vctcxo,
	.ops = &mmp3_clk_root_ops,
	.reg_data = {
		     { {MPMU_FCCR, 23, 0x7}, {MPMU_FCCR, 23, 0x7} },
		     { {0, 0, 0}, {0, 0, 0} } },
};

static struct clk mmp3_clk_ddr1 = {
	.name = "ddr1",
	.parent = &mmp3_clk_ddr_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_CC, 12, 0x7}, {PMUA_CC, 12, 0x7} } },
};

static struct clk mmp3_clk_ddr2 = {
	.name = "ddr2",
	.parent = &mmp3_clk_ddr_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_CC, 9, 0x7}, {PMUA_CC_3, 17, 0x7} } },
};

/* axi clock definitions */
static struct clk mmp3_clk_axi_root = {
	.name = "axi_root",
	.inputs = mux_pll1_pll2_vctcxo,
	.ops = &mmp3_clk_root_ops,
	.reg_data = {
		     { {PMUA_BUS, 6, 0x7}, {PMUA_BUS, 6, 0x7} },
		     { {0, 0, 0}, {0, 0, 0} } },
};

static struct clk mmp3_clk_axi1 = {
	.name = "axi1",
	.parent = &mmp3_clk_axi_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_CC, 15, 0x7}, {PMUA_CC, 15, 0x7} } },
};

static struct clk mmp3_clk_axi2 = {
	.name = "axi2",
	.parent = &mmp3_clk_axi_root,
	.ops = &mmp3_clk_div_ops,
	.reg_data = {
		     { {0, 0, 0}, {0, 0, 0} },
		     { {PMUA_DM_2_CC, 0, 0x7}, {PMUA_CC_2, 0, 0x7} } },
};

static int clk_cpu_setrate(struct clk *clk, unsigned long val)
{
	/*
	 * FIXME this need change when smp process id to real process id
	 * mapping is ready, then we can do frequency table for each core
	 * currently the mapping depends on which core to boot.
	 * Also currently we assume only MP1/2 are active and they always runs
	 * at the same frequency. so we only use and trigger DFC target on MP1
	 */
	mmp3_setfreq(MMP3_CLK_MP1, val);
	return 0;
}

static unsigned long clk_cpu_getrate(struct clk *clk)
{
	return mmp3_getfreq(MMP3_CLK_MP1);
}

static struct clkops clk_cpu_ops = {
	.setrate = clk_cpu_setrate,
	.getrate = clk_cpu_getrate,
};

static struct clk mmp3_clk_cpu = {
	.name = "cpu",
	.ops = &clk_cpu_ops,
};


#define GC_CLK_DIV(n)		((n & 0xF) << 24)
#define GC_CLK_DIV_GET(n)	((n >> 24) & 0xF)
#define GC_CLK_DIV_MSK		GC_CLK_DIV(0xF)
#define GC_PWRUP(n)		((n & 3) << 9)
#define GC_PWRUP_MSK		GC_PWRUP(3)
#define		PWR_OFF		0
#define		PWR_SLOW_RAMP	1
#define		PWR_ON		3
#define GC_ISB			(1 << 8)
#define GC_CLK_SRC_SEL(n)	((n & 3) << 6)
#define GC_CLK_SRC_SEL_MSK	GC_CLK_SRC_SEL(3)
#define		CS_PLL1		0
#define		CS_PLL2		1
#define		CS_PLL1_COP	2
#define		CS_PLL2_COP	3
#define GC_ACLK_SEL(n)		((n & 3) << 4)
#define GC_ACLK_SEL_MSK		GC_ACLK_SEL(3)
#define		PLL1D4		0
#define		PLL1D6		1
#define		PLL1D2		2
#define		PLL2D2		3
#define GC_CLK_EN		(1 << 3)
#define GC_AXICLK_EN		(1 << 2)
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

static void gc_clk_init(struct clk *clk)
{
	clk->rate = clk_get_rate(&mmp3_clk_pll1_clkoutp)/2;
	clk->enable_val = PLL1D2; /* reuse this field for the ACLK setting */
	clk->div = 2;
	clk->mul = 1;
	clk_reparent(clk, &mmp3_clk_pll1_clkoutp);
}

static int gc_clk_enable(struct clk *clk)
{
	unsigned long gc_rate_cfg;
	unsigned long i;

	/* TODO may need to request for different core voltage according to
	 * different gc clock rate.
	 */

	GC_SET_BITS(GC_PWRUP(PWR_SLOW_RAMP), -1);
	GC_SET_BITS(GC_PWRUP(PWR_ON), GC_PWRUP_MSK);

	i = 0;
	while ((clk->inputs[i].input != clk->parent) && clk->inputs[i].input)
		i++;

	if (clk->inputs[i].input == 0) {
		pr_err("%s: unexpected gc clock source\n", __func__);
		return -1;
	}

	gc_rate_cfg = GC_CLK_RATE(clk->div,
		clk->inputs[i].value, clk->enable_val);
	gc_rate_cfg &= GC_CLK_RATE_MSK;
	GC_SET_BITS(gc_rate_cfg, GC_CLK_RATE_MSK);

	GC_SET_BITS(GC_CLK_EN, 0);
	udelay(100);

	GC_SET_BITS(GC_AXICLK_EN, 0);
	udelay(100);

	GC_SET_BITS(GC_ISB, 0);
	GC_SET_BITS(GC_RST, 0);
	GC_SET_BITS(GC_AXI_RST, 0);

	return 0;
}

static void gc_clk_disable(struct clk *clk)
{
	GC_SET_BITS(0, GC_ISB);
	GC_SET_BITS(0, GC_RST | GC_AXI_RST);
	GC_SET_BITS(0, GC_CLK_EN | GC_AXICLK_EN);
	GC_SET_BITS(0, GC_PWRUP_MSK);
}

static long gc_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (rate <= clk_get_rate(&mmp3_clk_pll1)/8)
		return clk_get_rate(&mmp3_clk_pll1)/8; /* 100M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1)/4)
		return clk_get_rate(&mmp3_clk_pll1)/4; /* 200M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1_clkoutp)/3)
		return clk_get_rate(&mmp3_clk_pll1_clkoutp)/3; /* 354M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1)/2)
		return clk_get_rate(&mmp3_clk_pll1)/2; /* 400M */
	else
		return clk_get_rate(&mmp3_clk_pll1_clkoutp)/2; /* 531M */
}

static int gc_clk_setrate(struct clk *clk, unsigned long rate)
{
	clk->mul = 1;
	if (rate == clk_get_rate(&mmp3_clk_pll1)/8) {
		clk->enable_val = PLL1D6;
		clk->div = 8;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1)/4) {
		clk->enable_val = PLL1D4;
		clk->div = 4;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1_clkoutp)/3) {
		clk->enable_val = PLL1D2;
		clk->div = 3;
		clk_reparent(clk, &mmp3_clk_pll1_clkoutp);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1)/2) {
		clk->enable_val = PLL1D2;
		clk->div = 2;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1_clkoutp)/2) {
		clk->enable_val = PLL1D2;
		clk->div = 2;
		clk_reparent(clk, &mmp3_clk_pll1_clkoutp);
	} else {
		pr_err("%s: unexpected gc clock rate %ld\n", __func__, rate);
		BUG();
	}

	return 0;
}

struct clkops gc_clk_ops = {
	.init		= gc_clk_init,
	.enable		= gc_clk_enable,
	.disable	= gc_clk_disable,
	.setrate	= gc_clk_setrate,
	.round_rate	= gc_clk_round_rate,
	.set_parent	= mmp3_clk_set_parent,
};

static struct clk_mux_sel gc_mux_pll1_pll2[] = {
	{.input = &mmp3_clk_pll1, .value = 0},
	{.input = &mmp3_clk_pll2, .value = 1},
	{.input = &mmp3_clk_pll1_clkoutp, .value = 2},
	{0, 0},
};

static struct clk mmp3_clk_gc = {
	.name = "gc",
	.inputs = gc_mux_pll1_pll2,
	.lookup = {
		.con_id = "GCCLK",
	},
	.clk_rst = (void __iomem *)APMU_GC,
	.ops = &gc_clk_ops,
};

static int disp1_axi_clk_enable(struct clk *clk)
{
	u32 val = __raw_readl(clk->clk_rst);

	/* enable Display1 AXI clock */
	val |= (1<<3);
	__raw_writel(val, clk->clk_rst);

	/* release from reset */
	val |= 1;
	__raw_writel(val, clk->clk_rst);
	return 0;
}

static void disp1_axi_clk_disable(struct clk *clk)
{
	u32 val = __raw_readl(clk->clk_rst);

	/* reset display1 AXI clock */
	val &= ~1;
	__raw_writel(val, clk->clk_rst);

	/* disable display1 AXI clock */
	val &= ~(1<<3);
	__raw_writel(val, clk->clk_rst);
}

struct clkops disp1_axi_clk_ops = {
	.enable		= disp1_axi_clk_enable,
	.disable	= disp1_axi_clk_disable,
};

static struct clk mmp3_clk_disp1_axi = {
	.name = "disp1_axi",
	.lookup = {
		.con_id = "DISP1AXI",
	},
	.clk_rst = (void __iomem *)APMU_LCD_CLK_RES_CTRL,
	.ops = &disp1_axi_clk_ops,
};

static struct clk *disp_depend_clk[] = {
	&mmp3_clk_disp1_axi,
};

static int lcd_pn1_clk_enable(struct clk *clk)
{
	u32 val;

	val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);

	/* enable Display1 peripheral clock */
	val |= (1 << 4);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	/* enable DSI clock */
	val |= (1 << 12) | (1 << 5);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	/* release from reset */
	val |= (1 << 1) | (1 << 2);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	return 0;
}

static void lcd_pn1_clk_disable(struct clk *clk)
{
	u32 val;

	val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);

	/* disable DSI clock */
	val &= ~((1<<12) | (1<<5));
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	/* disable Display1 peripheral clock */
	val &= ~(1<<4);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);
}

static long lcd_clk_round_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * lcd actually has four clock source: pll1, pll1/16, pll2 and vctcxo.
	 * the range of the divider is 1 to 15.
	 * here the policy is try to not use pll2 as possile as it can.
	 * since the divider can be as large as 15 so it don't need pll1/16
	 * as the clock source. For the very low rate requirement, we can
	 * just use vctcxo as the clock source.
	 */
	int i;
	unsigned long parent_rate;

	/* for those which is less than 26M, use vctcxo as clock source */
	if (rate <= clk_get_rate(&mmp3_clk_vctcxo)) {
		parent_rate = clk_get_rate(&mmp3_clk_vctcxo);
		for (i = 2; i < 16; i++) {
			if (rate > parent_rate / i)
				break;
		}

		return parent_rate / (i - 1);
	/* for those which is less than 800M, use pll1 as clock source */
	} else if (rate <= clk_get_rate(&mmp3_clk_pll1)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1);
		for (i = 2; i < 16; i++) {
			if (rate > parent_rate / i)
				break;
		}

		return parent_rate / (i - 1);
	/* for those which is larger than 800M, use pll2 as clock source */
	} else
		return clk_get_rate(&mmp3_clk_pll2);
}

#define LCD_PN1_DSI_PHYSLOW_PRER	0x1A
#define LCD_PN1_DSI_PHYSLOW_PRER_SHIFT	15
#define LCD_PN1_DSI_PHYSLOW_PRER_MASK	0x1F

static int lcd_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long parent_rate;
	const struct clk_mux_sel *sel;
	u32 val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);

	/* use fixed prescaler for DSI PHY slow clock */
	val &= ~(LCD_PN1_DSI_PHYSLOW_PRER_MASK <<
			LCD_PN1_DSI_PHYSLOW_PRER_SHIFT);
	val |= (LCD_PN1_DSI_PHYSLOW_PRER << LCD_PN1_DSI_PHYSLOW_PRER_SHIFT);

	if (rate <= clk_get_rate(&mmp3_clk_vctcxo)) {
		parent_rate = clk_get_rate(&mmp3_clk_vctcxo);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_vctcxo);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_vctcxo)
				break;
		}
		if (sel->input == 0) {
			pr_err("lcd: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	} else if (rate <= clk_get_rate(&mmp3_clk_pll1)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_pll1);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_pll1)
				break;
		}
		if (sel->input == 0) {
			pr_err("lcd: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	} else if (rate <= clk_get_rate(&mmp3_clk_pll2)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll2);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_pll2);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_pll2)
				break;
		}
		if (sel->input == 0) {
			pr_err("lcd: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	}

	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	return 0;
}

struct clkops lcd_pn1_clk_ops = {
	.enable = lcd_pn1_clk_enable,
	.disable = lcd_pn1_clk_disable,
	.round_rate = lcd_clk_round_rate,
	.setrate = lcd_clk_setrate,
};

static struct clk_mux_sel lcd_pn1_clk_mux[] = {
	{.input = &mmp3_clk_pll1, .value = 0},
	{.input = &mmp3_clk_pll2, .value = 2},
	{.input = &mmp3_clk_vctcxo, .value = 3},
	{0, 0},
};

static struct clk mmp3_clk_lcd1 = {
	.name = "lcd1",
	.lookup = {
		.con_id = "LCDCLK",
	},
	.ops = &lcd_pn1_clk_ops,
	.dependence = disp_depend_clk,
	.dependence_count = ARRAY_SIZE(disp_depend_clk),
	.inputs = lcd_pn1_clk_mux,
	.reg_data = {
		     { {APMU_LCD_CLK_RES_CTRL, 6, 0x3},
			{APMU_LCD_CLK_RES_CTRL, 6, 0x3} },
		     {{APMU_LCD_CLK_RES_CTRL, 8, 0xf},
			{APMU_LCD_CLK_RES_CTRL, 8, 0xf} } },
};

static int hdmi_clk_enable(struct clk *clk)
{
	/*
	 *hdmi clock enable is done by user space,
	 * to control it's dependence clock disp1_clk
	 * here we must define this clk_enable function
	 */
	return 0;
};

static void hdmi_clk_disable(struct clk *clk)
{
	/*
	 * hdmi clock disable is done by user space,
	 * to control it's dependence clock disp1_clk
	 * here we must define this clk_disable function
	 */

	return;
};

struct clkops hdmi_clk_ops = {
	.enable = hdmi_clk_enable,
	.disable = hdmi_clk_disable,
};

static struct clk mmp3_clk_hdmi = {
	.name = "hdmi",
	.lookup = {
		.con_id = "HDMICLK",
	},
	.ops = &hdmi_clk_ops,
	.dependence = disp_depend_clk,
	.dependence_count = ARRAY_SIZE(disp_depend_clk),
};

#define LCD_SCLK_DIV	(APB_VIRT_BASE + 0x0020B1A8)

static int lcd_sclk_enable(struct clk *clk)
{
	u32 val;

	val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);
	/* enable path clock */
	val &= ~(1 << 28);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	return 0;
}

static void lcd_sclk_disable(struct clk *clk)
{
	u32 val;

	val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);
	/* disable path clock */
	val |= (1 << 28);
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);
}

static long lcd_sclk_round_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * lcd sclk actually has five clock souce: axi, display 1,
	 * display 2, HDMI PLL and PLL3. Here we just fix it to be
	 * PLL3 since it makes things simple and PLL3 satisfy it well.
	 */
	int i;
	unsigned long parent_rate = clk_get_rate(&mmp3_clk_pll3);

	/* the divider is from 2 to 255 */
	for (i = 3; i < 256; i++) {
		if (rate > parent_rate / i)
			break;
	}

	return parent_rate / (i - 1);
}

static int lcd_sclk_setrate(struct clk *clk, unsigned long rate)
{
	const struct clk_mux_sel *sel;
	unsigned long parent_rate = clk_get_rate(&mmp3_clk_pll3);
	u32 val = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);

	clk->mul = 1;
	clk->div = parent_rate / rate;

	clk_reparent(clk, &mmp3_clk_pll3);

	val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
		 << clk->reg_data[DIV][CONTROL].reg_shift);
	val |= clk->div
	    << clk->reg_data[DIV][CONTROL].reg_shift;

	for (sel = clk->inputs; sel->input != 0; sel++) {
		if (sel->input == &mmp3_clk_pll3)
			break;
	}
	if (sel->input == 0) {
		pr_err("lcd: no matched input for this parent!\n");
		BUG();
	}

	val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
		<< clk->reg_data[SOURCE][CONTROL].reg_shift);
	val |= sel->value
		<< clk->reg_data[SOURCE][CONTROL].reg_shift;

	val |= 0x10001200;
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	return 0;
}

struct clkops lcd_sclk_ops = {
	.enable = lcd_sclk_enable,
	.disable = lcd_sclk_disable,
	.round_rate = lcd_sclk_round_rate,
	.setrate = lcd_sclk_setrate,
};

/* Note: does not expose AXI/Display 2/HDMI PLL */
static struct clk_mux_sel lcd_sclk_mux[] = {
	{.input = &mmp3_clk_lcd1, .value = 1},
	{.input = &mmp3_clk_pll3, .value = 7},
	{0, 0},
};

static struct clk mmp3_lcd_sclk = {
	.name = "lcd_sclk",
	.lookup = {
		.dev_id = "pxa168-fb.0",
		.con_id = "LCDSCLK",
	},
	.ops = &lcd_sclk_ops,
	.inputs = lcd_sclk_mux,
	.reg_data = {
		     { {LCD_SCLK_DIV, 29, 0x7},
			{LCD_SCLK_DIV, 29, 0x7} },
		     {{LCD_SCLK_DIV, 0, 0xff},
			{LCD_SCLK_DIV, 0, 0xff} } },
};

static void sdhc_clk_init(struct clk *clk)
{
	const struct clk_mux_sel *sel;
	u32 val = 0;

	clk->mul = 1;
	clk->div = 1;

	clk_reparent(clk, &mmp3_clk_pll1_d_4);

	val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
		 << clk->reg_data[DIV][CONTROL].reg_shift);
	val |= clk->div
	    << clk->reg_data[DIV][CONTROL].reg_shift;

	for (sel = clk->inputs; sel->input != 0; sel++) {
		if (sel->input == &mmp3_clk_pll1_d_4)
			break;
	}
	if (sel->input == 0) {
		pr_err("sdh: no matched input for this parent!\n");
		BUG();
	}

	val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
		<< clk->reg_data[SOURCE][CONTROL].reg_shift);
	val |= sel->value
		<< clk->reg_data[SOURCE][CONTROL].reg_shift;

	clk->enable_val = val;
}

static int sdhc_clk_enable(struct clk *clk)
{
	u32 val;

	val = clk->enable_val | 0x1b;
	__raw_writel(val, clk->reg_data[SOURCE][CONTROL].reg);

	return 0;
}

static void sdhc_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->reg_data[SOURCE][CONTROL].reg);
}

static long sdhc_clk_round_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * SDH has four clock source: PLL1, PLL1/2, PLL1/4 and PLL2.
	 * Here PLL1/2 is not used since PLL1 and PLL1/4 can cover it.
	 * Only use PLL2 as clock source if the rate is larger than PLL1.
	 */
	int i;
	unsigned long parent_rate;

	/* for those which is less than pll1/4, use pll1/4 as clock source */
	if (rate <= clk_get_rate(&mmp3_clk_pll1_d_4)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1_d_4);
		for (i = 2; i < 16; i++) {
			if (rate > parent_rate / i)
				break;
		}

		return parent_rate / (i - 1);
	/* else for those which is less than pll1, use pll1 as clock source */
	} else if (rate <= clk_get_rate(&mmp3_clk_pll1)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1);
		for (i = 2; i < 16; i++) {
			if (rate > parent_rate / i)
				break;
		}

		return parent_rate / (i - 1);
	/* else for those which is larger than pll1, use pll2 as clock source */
	} else {
		parent_rate = clk_get_rate(&mmp3_clk_pll2);
		for (i = 2; i < 16; i++) {
			if (rate > parent_rate / i)
				break;
		}
	}
		return parent_rate / (i - 1);
}

static int sdhc_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long parent_rate;
	const struct clk_mux_sel *sel;
	u32 val = 0;

	if (rate <= clk_get_rate(&mmp3_clk_pll1_d_4)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1_d_4);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_pll1_d_4);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_pll1_d_4)
				break;
		}
		if (sel->input == 0) {
			pr_err("sdh: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	} else if (rate <= clk_get_rate(&mmp3_clk_pll1)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_pll1);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_pll1)
				break;
		}
		if (sel->input == 0) {
			pr_err("sdh: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	} else if (rate <= clk_get_rate(&mmp3_clk_pll2)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll2);
		clk->mul = 1;
		clk->div = parent_rate / rate;

		clk_reparent(clk, &mmp3_clk_pll2);

		val &= ~(clk->reg_data[DIV][CONTROL].reg_mask
			 << clk->reg_data[DIV][CONTROL].reg_shift);
		val |= clk->div
		    << clk->reg_data[DIV][CONTROL].reg_shift;

		for (sel = clk->inputs; sel->input != 0; sel++) {
			if (sel->input == &mmp3_clk_pll2)
				break;
		}
		if (sel->input == 0) {
			pr_err("sdh: no matched input for this parent!\n");
			BUG();
		}

		val &= ~(clk->reg_data[SOURCE][CONTROL].reg_mask
			<< clk->reg_data[SOURCE][CONTROL].reg_shift);
		val |= sel->value
			<< clk->reg_data[SOURCE][CONTROL].reg_shift;
	}

	clk->enable_val = val;

	return 0;
}

struct clkops sdhc_clk_ops = {
	.init = sdhc_clk_init,
	.enable = sdhc_clk_enable,
	.disable = sdhc_clk_disable,
	.round_rate = sdhc_clk_round_rate,
	.setrate = sdhc_clk_setrate,
};

static struct clk_mux_sel sdhc_clk_mux[] = {
	{.input = &mmp3_clk_pll1_d_4, .value = 0},
	{.input = &mmp3_clk_pll2, .value = 1},
	{.input = &mmp3_clk_pll1, .value = 3},
	{0, 0},
};

static struct clk mmp3_clk_sdh0 = {
	.name = "sdh0",
	.lookup = {
		.dev_id = "sdhci-pxav3.0",
		.con_id = "PXA-SDHCLK",
	},
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.reg_data = {
		     { {APMU_SDH0, 8, 0x3},
			{APMU_SDH0, 8, 0x3} },
		     {{APMU_SDH0, 10, 0xf},
			{APMU_SDH0, 10, 0xf} } },
};

static struct clk mmp3_clk_sdh1 = {
	.name = "sdh1",
	.lookup = {
		.dev_id = "sdhci-pxav3.1",
		.con_id = "PXA-SDHCLK",
	},
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.reg_data = {
		     { {APMU_SDH1, 8, 0x3},
			{APMU_SDH1, 8, 0x3} },
		     {{APMU_SDH1, 10, 0xf},
			{APMU_SDH1, 10, 0xf} } },
};

static struct clk mmp3_clk_sdh2 = {
	.name = "sdh2",
	.lookup = {
		.dev_id = "sdhci-pxav3.2",
		.con_id = "PXA-SDHCLK",
	},
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.reg_data = {
		     { {APMU_SDH2, 8, 0x3},
			{APMU_SDH2, 8, 0x3} },
		     {{APMU_SDH2, 10, 0xf},
			{APMU_SDH2, 10, 0xf} } },
};

static struct clk mmp3_clk_sdh3 = {
	.name = "sdh3",
	.lookup = {
		.dev_id = "sdhci-pxav3.3",
		.con_id = "PXA-SDHCLK",
	},
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.reg_data = {
		     { {APMU_SDH3, 8, 0x3},
			{APMU_SDH3, 8, 0x3} },
		     {{APMU_SDH3, 10, 0xf},
			{APMU_SDH3, 10, 0xf} } },
};

static struct clk *mmp3_clks_ptr[] = {
	&mmp3_clk_pll1_d_2,
	&mmp3_clk_pll1,
	&mmp3_clk_pll2,
	&mmp3_clk_pll1_clkoutp,
	&mmp3_clk_vctcxo,
	&mmp3_clk_32k,
	&mmp3_clk_core_root,
	&mmp3_clk_virtual_pj,
	&mmp3_clk_mp1,
	&mmp3_clk_mp2,
	&mmp3_clk_mm,
	&mmp3_clk_cpu,
	&mmp3_clk_aclk,
	&mmp3_clk_core_periph,
	&mmp3_clk_atclk,
	&mmp3_clk_ddr_root,
	&mmp3_clk_ddr1,
	&mmp3_clk_ddr2,
	&mmp3_clk_axi_root,
	&mmp3_clk_axi1,
	&mmp3_clk_axi2,
	&mmp3_clk_pll1_d_4,
	&mmp3_clk_pll3,
	&mmp3_clk_gc,
	&mmp3_clk_lcd1,
	&mmp3_lcd_sclk,
	&mmp3_clk_sdh0,
	&mmp3_clk_sdh1,
	&mmp3_clk_sdh2,
	&mmp3_clk_sdh3,
	&mmp3_clk_disp1_axi,
	&mmp3_clk_hdmi,
};

static int apbc_clk_enable(struct clk *clk)
{
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLKSEL(7));
	data |= APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	__raw_writel(data, clk->clk_rst);
	/*
	 * delay two cycles of the solwest clock between the APB bus clock
	 * and the functional module clock.
	 */
	udelay(10);

	data |= APBC_APBCLK;
	__raw_writel(data, clk->clk_rst);
	udelay(10);

	data &= ~APBC_RST;
	__raw_writel(data, clk->clk_rst);

	return 0;
}

static void apbc_clk_disable(struct clk *clk)
{
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLK | APBC_FNCLKSEL(7));
	__raw_writel(data, clk->clk_rst);
	udelay(10);

	data &= ~APBC_APBCLK;
	__raw_writel(data, clk->clk_rst);
}

struct clkops apbc_clk_ops = {
	.enable = apbc_clk_enable,
	.disable = apbc_clk_disable,
};

#define APBC_CLK(_name, _dev, _con, _reg, _fnclksel, _rate, _parent)\
{							\
	.name = _name,					\
	.lookup = {					\
		.dev_id = _dev,				\
		.con_id = _con,				\
	},						\
	.clk_rst = (void __iomem *)APBC_##_reg,		\
	.fnclksel = _fnclksel,				\
	.rate = _rate,					\
	.ops = &apbc_clk_ops,				\
	.parent = _parent,				\
}

#define APBC_CLK_OPS(_name, _dev, _con, _reg, _fnclksel, _rate, _parent, _ops)\
{							\
	.name = _name,					\
	.lookup = {					\
		.dev_id = _dev,				\
		.con_id = _con,				\
	},						\
	.clk_rst = (void __iomem *)APBC_##_reg,		\
	.fnclksel = _fnclksel,				\
	.rate = _rate,					\
	.ops = _ops,					\
	.parent = _parent,				\
}

static void uart_clk_init(struct clk *clk)
{
	clk->mul = clk->div = 1;
	/*
	 * Bit(s) PMUM_SUCCR_RSRV_31_29 reserved
	 * UART Clock Generation Programmable Divider
	 * Numerator Value
	 */
	clk->reg_data[DIV][CONTROL].reg = MPMU_SUCCR;
	clk->reg_data[DIV][CONTROL].reg_shift = 16;
	clk->reg_data[DIV][CONTROL].reg_mask = 0x1fff;
	/*
	 * Bit(s) PMUM_SUCCR_RSRV_15_13 reserved
	 * UART Clock Generation Programmable Divider
	 * Denominator Value
	 */
	clk->reg_data[MUL][CONTROL].reg = MPMU_SUCCR;
	clk->reg_data[MUL][CONTROL].reg_shift = 0;
	clk->reg_data[MUL][CONTROL].reg_mask = 0x1fff;
}

static int uart_clk_enable(struct clk *clk)
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

	if (clk->rate == clk_get_rate(&mmp3_clk_vctcxo)) {
		/* choose vctcxo */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		clk_rst |= APBC_FNCLKSEL(0x1);
		__raw_writel(clk_rst, clk->clk_rst);
	} else {
		/* choose programmable clk */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		__raw_writel(clk_rst, clk->clk_rst);
	}

	return 0;
}

static void uart_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
	mdelay(1);
}

static long uart_clk_round_rate(struct clk *clk, unsigned long rate)
{
	unsigned long parent_rate;

	if (rate >= clk_get_rate(&mmp3_clk_vctcxo)) {
		parent_rate = clk_get_rate(&mmp3_clk_pll1_d_4);
		return parent_rate * 16 / 27 / 2;
	} else
		return clk_get_rate(&mmp3_clk_vctcxo);
}

static int uart_clk_setrate(struct clk *clk, unsigned long val)
{
	uint32_t clk_rst;

	if (val == clk_get_rate(&mmp3_clk_vctcxo)) {
		/* choose vctcxo */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		clk_rst |= APBC_FNCLKSEL(0x1);
		__raw_writel(clk_rst, clk->clk_rst);

		clk->div = clk->mul = 1;

		clk_reparent(clk, &mmp3_clk_vctcxo);
	} else {
		/* set m/n for high speed */
		unsigned int numer = 27;
		unsigned int denom = 16;

		/*
		 * n/d = base_clk/(2*out_clk)
		 * base_clk = 199.33M, out_clk=199.33*16/27/2=59.06M
		 * buadrate = clk/(16*divisor)
		 */

		clk_rst = __raw_readl(clk->reg_data[DIV][CONTROL].reg);
		clk_rst &= ~(clk->reg_data[DIV][CONTROL].reg_mask <<
				clk->reg_data[DIV][CONTROL].reg_shift);
		clk_rst |= numer << clk->reg_data[DIV][CONTROL].reg_shift;
		clk_rst &= ~(clk->reg_data[MUL][CONTROL].reg_mask <<
				clk->reg_data[MUL][CONTROL].reg_shift);
		clk_rst |= denom << clk->reg_data[MUL][CONTROL].reg_shift;
		__raw_writel(clk_rst, clk->reg_data[DIV][CONTROL].reg);

		/* choose programmable clk */
		clk_rst = __raw_readl(clk->clk_rst);
		clk_rst &= ~(APBC_FNCLKSEL(0x7));
		__raw_writel(clk_rst, clk->clk_rst);

		clk->div = numer * 2;
		clk->mul = denom;

		clk_reparent(clk, &mmp3_clk_pll1_d_4);
	}

	return 0;
}

struct clkops uart_clk_ops = {
	.init = uart_clk_init,
	.enable = uart_clk_enable,
	.disable = uart_clk_disable,
	.round_rate = uart_clk_round_rate,
	.setrate = uart_clk_setrate,
};

static int rtc_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst = APBC_APBCLK | APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	clk_rst |= 1 << 7;
	__raw_writel(clk_rst, clk->clk_rst);

	return 0;
}

static void rtc_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

static void nand_clk_init(struct clk *clk)
{
	clk->div = 8;
	clk->mul = 1;

	/* by default select pll1 as clock source and divider 8 */
	clk->enable_val = 0x80;
}

static int nand_clk_enable(struct clk *clk)
{
	__raw_writel(clk->enable_val | 0x2d, clk->clk_rst);

	return 0;
}

static void nand_clk_disable(struct clk *clk)
{
	__raw_writel(0x0, clk->clk_rst);
}

static long nand_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (rate <= clk_get_rate(&mmp3_clk_vctcxo) / 2)
		return clk_get_rate(&mmp3_clk_vctcxo) / 2; /* 13M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1) / 12)
		return clk_get_rate(&mmp3_clk_pll1) / 12; /* 67M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1) / 8)
		return clk_get_rate(&mmp3_clk_pll1) / 8; /* 100M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll2) / 12)
		return clk_get_rate(&mmp3_clk_pll2) / 12; /* 111M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1) / 6)
		return clk_get_rate(&mmp3_clk_pll1) / 6; /* 133M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll2) / 8)
		return clk_get_rate(&mmp3_clk_pll2) / 8; /* 167M */
	else if (rate <= clk_get_rate(&mmp3_clk_pll1) / 4)
		return clk_get_rate(&mmp3_clk_pll1) / 4; /* 200M */
	else
		return clk_get_rate(&mmp3_clk_pll2) / 6; /* 222M */
}

static int nand_clk_setrate(struct clk *clk, unsigned long rate)
{
	if (rate == clk_get_rate(&mmp3_clk_vctcxo) / 2) {
		clk->enable_val = 0x1c0;
		clk->div = 2;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_vctcxo);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1) / 12) {
		clk->enable_val = 0xc0;
		clk->div = 12;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1) / 8) {
		clk->enable_val = 0x80;
		clk->div = 8;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll2) / 12) {
		clk->enable_val = 0x180;
		clk->div = 12;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll2);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1) / 6) {
		clk->enable_val = 0x40;
		clk->div = 6;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll2) / 8) {
		clk->enable_val = 0x140;
		clk->div = 8;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll2);
	} else if (rate == clk_get_rate(&mmp3_clk_pll1) / 4) {
		clk->enable_val = 0x0;
		clk->div = 4;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll1);
	} else if (rate == clk_get_rate(&mmp3_clk_pll2) / 6) {
		clk->enable_val = 0x100;
		clk->div = 6;
		clk->mul = 1;
		clk_reparent(clk, &mmp3_clk_pll2);
	} else {
		pr_err("%s: unexpected clock rate %ld\n", __func__, rate);
		BUG();
	}

	return 0;
}

struct clkops nand_clk_ops = {
	.init = nand_clk_init,
	.enable = nand_clk_enable,
	.disable = nand_clk_disable,
	.round_rate = nand_clk_round_rate,
	.setrate = nand_clk_setrate,
};

struct clkops rtc_clk_ops = {
	.enable = rtc_clk_enable,
	.disable = rtc_clk_disable,
};

static int apmu_clk_enable(struct clk *clk)
{
	__raw_writel(clk->enable_val, clk->clk_rst);

	return 0;
}

static void apmu_clk_disable(struct clk *clk)
{
	__raw_writel(0, clk->clk_rst);
}

static int apmu_clk_setrate(struct clk *clk, unsigned long rate)
{
	__raw_writel(rate, clk->clk_rst);

	return 0;
}

struct clkops apmu_clk_ops = {
	.enable = apmu_clk_enable,
	.disable = apmu_clk_disable,
	.setrate = apmu_clk_setrate,
};

#define APMU_CLK(_name, _dev, _con, _reg, _eval, _rate, _parent)\
{								\
	.name = _name,						\
	.lookup = {						\
		.dev_id = _dev,					\
		.con_id = _con,					\
	},							\
	.clk_rst = (void __iomem *)APMU_##_reg,			\
	.enable_val = _eval,					\
	.rate = _rate,						\
	.ops = &apmu_clk_ops,					\
	.parent = _parent,					\
}

#define APMU_CLK_OPS(_name, _dev, _con, _reg, _eval, _rate, _parent, _ops)\
{								\
	.name = _name,						\
	.lookup = {						\
		.dev_id = _dev,					\
		.con_id = _con,					\
	},							\
	.clk_rst = (void __iomem *)APMU_##_reg,			\
	.enable_val = _eval,					\
	.rate = _rate,						\
	.parent = _parent,					\
	.ops = _ops,						\
}

#ifdef CONFIG_UIO_VMETA

static int vmeta_clk_enable(struct clk *clk)
{
	int reg;

	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	reg &= ~APMU_VMETA_CLK_SEL_MASK;
	reg &= ~APMU_VMETA_CLK_DIV_MASK;
	reg |= APMU_VMETA_CLK_PLL2;
	reg |= APMU_VMETA_CLK_DIV_2;

	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	reg |= APMU_VMETA_AXICLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	reg |= APMU_VMETA_CLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	reg &= ~APMU_VMETA_AXI_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);
	reg |= APMU_VMETA_AXI_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	reg &= ~APMU_VMETA_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);
	reg |= APMU_VMETA_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg = readl(APMU_VMETA_CLK_RES_CTRL);

	return 0;
}

static void vmeta_clk_disable(struct clk *clk)
{
	int reg;

	reg = readl(APMU_VMETA_CLK_RES_CTRL);
	reg &= ~APMU_VMETA_AXI_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg &= ~APMU_VMETA_RST;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);

	reg = readl(APMU_VMETA_CLK_RES_CTRL);
	reg &= ~APMU_VMETA_AXICLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
	reg &= ~APMU_VMETA_CLK_EN;
	writel(reg, APMU_VMETA_CLK_RES_CTRL);
}

static int vmeta_clk_setrate(struct clk *clk, unsigned long rate)
{
	return 0;
}

struct clkops vmeta_clk_ops = {
	.enable         = vmeta_clk_enable,
	.disable        = vmeta_clk_disable,
	.setrate        = vmeta_clk_setrate,
};
#endif

static int ccic_rst_clk_enable(struct clk *clk)
{
	__raw_writel(0x2e838, clk->clk_rst);
	__raw_writel(0x3e909, clk->clk_rst);
	__raw_writel(0x3eb39, clk->clk_rst);
	__raw_writel(0x3eb3f, clk->clk_rst);
#if 0
	__raw_writel(0x000387FF, clk->clk_rst);
#endif

	return 0;
}

static void ccic_rst_clk_disable(struct clk *clk)
{
	__raw_writel(0x3eb3f, clk->clk_rst);
	__raw_writel(0x3eb39, clk->clk_rst);
	__raw_writel(0x3e909, clk->clk_rst);
	__raw_writel(0x2e838, clk->clk_rst);
	__raw_writel(0x0, clk->clk_rst);
}

struct clkops ccic_rst_clk_ops = {
	.enable         = ccic_rst_clk_enable,
	.disable        = ccic_rst_clk_disable,
};

static int ccic_dbg_clk_enable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	tmp |= (1 << 25) | (1 << 27);
	__raw_writel(tmp, clk->clk_rst);

	return 0;
}

static void ccic_dbg_clk_disable(struct clk *clk)
{
	u32 tmp = __raw_readl(clk->clk_rst);
	tmp &= ~((1 << 25) | (1 << 27));
	__raw_writel(tmp, clk->clk_rst);
}

struct clkops ccic_dbg_clk_ops = {
	.enable         = ccic_dbg_clk_enable,
	.disable        = ccic_dbg_clk_disable,
};

/* usb: hsic clock */
static int hsic_clk_enable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst |= 0x1b;
	__raw_writel(clk_rst, clk->clk_rst);

	return 0;
}

static void hsic_clk_disable(struct clk *clk)
{
	uint32_t clk_rst;

	clk_rst  =  __raw_readl(clk->clk_rst);
	clk_rst &= ~0x18;
	__raw_writel(clk_rst, clk->clk_rst);
}

struct clkops hsic_clk_ops = {
	.enable		= hsic_clk_enable,
	.disable	= hsic_clk_disable,
};

static struct clk mmp3_list_clks[] = {
	APBC_CLK("twsi1", "pxa2xx-i2c.0", NULL, MMP2_TWSI1,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("twsi2", "pxa2xx-i2c.1", NULL, MMP2_TWSI2,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("twsi3", "pxa2xx-i2c.2", NULL, MMP2_TWSI3,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("twsi4", "pxa2xx-i2c.3", NULL, MMP2_TWSI4,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("twsi5", "pxa2xx-i2c.4", NULL, MMP2_TWSI5,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("twsi6", "pxa2xx-i2c.5", NULL, MMP2_TWSI6,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("pwm1", "mmp2-pwm.0", NULL, MMP2_PWM0,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("pwm2", "mmp2-pwm.1", NULL, MMP2_PWM1,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("pwm3", "mmp2-pwm.2", NULL, MMP2_PWM2,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("pwm4", "mmp2-pwm.3", NULL, MMP2_PWM3,
			0, 26000000, &mmp3_clk_vctcxo),
	APBC_CLK("keypad", "pxa27x-keypad", NULL, MMP2_KPC,
			0, 32768, &mmp3_clk_32k),
	APBC_CLK_OPS("uart1", "pxa2xx-uart.0", NULL, MMP2_UART1,
			1, 26000000, &mmp3_clk_vctcxo, &uart_clk_ops),
	APBC_CLK_OPS("uart2", "pxa2xx-uart.1", NULL, MMP2_UART2,
			1, 26000000, &mmp3_clk_vctcxo, &uart_clk_ops),
	APBC_CLK_OPS("uart3", "pxa2xx-uart.2", NULL, MMP2_UART3,
			1, 26000000, &mmp3_clk_vctcxo, &uart_clk_ops),
	APBC_CLK_OPS("uart4", "pxa2xx-uart.3", NULL, MMP2_UART4,
			1, 26000000, &mmp3_clk_vctcxo, &uart_clk_ops),
	APBC_CLK_OPS("rtc", "mmp-rtc", NULL, MMP2_RTC,
			0, 32768, &mmp3_clk_32k, &rtc_clk_ops),
	APMU_CLK("u2o", NULL, "U2OCLK", USB,
			0x9, 480000000, NULL),
	APMU_CLK("ccic_gate", "mv-camera.0", "CCICGATECLK", CCIC_GATE,
			0xffff, 0, NULL),
	APMU_CLK("ccic_gate", "mv-camera.1", "CCICGATECLK", CCIC_GATE,
			0xffff, 0, NULL),
	APMU_CLK_OPS("nand", "pxa3xx-nand", NULL, NAND,
			0xbf, 100000000, &mmp3_clk_pll1, &nand_clk_ops),
#ifdef CONFIG_UIO_VMETA
	APMU_CLK_OPS("vmeta", NULL, "VMETA_CLK", VMETA,
			0, 0, NULL, &vmeta_clk_ops),
#endif
	APMU_CLK_OPS("ccic_rst", "mv-camera.0", "CCICRSTCLK", CCIC_RST,
			0, 312000000, NULL, &ccic_rst_clk_ops),
	APMU_CLK_OPS("ccic_rst", "mv-camera.1", "CCICRSTCLK", CCIC_RST,
			0, 312000000, NULL, &ccic_rst_clk_ops),
	APMU_CLK_OPS("ccic_dbg", "mv-camera.0", "CCICDBGCLK", CCIC_DBG,
			0, 312000000, NULL, &ccic_dbg_clk_ops),
	APMU_CLK_OPS("ccic_dbg", "mv-camera.1", "CCICDBGCLK", CCIC_DBG,
			0, 312000000, NULL, &ccic_dbg_clk_ops),
	APMU_CLK_OPS("hsic1", NULL, "HSIC1CLK", USBHSIC1,
			0x1b, 480000000, NULL, &hsic_clk_ops),
};

static void mmp3_init_one_clock(struct clk *c)
{
	clk_init(c);
	INIT_LIST_HEAD(&c->shared_bus_list);
	if (!c->lookup.dev_id && !c->lookup.con_id)
		c->lookup.con_id = c->name;
	c->lookup.clk = c;
	clkdev_add(&c->lookup);
}

static int __init mmp3_clk_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(mmp3_clks_ptr); i++)
		mmp3_init_one_clock(mmp3_clks_ptr[i]);
	for (i = 0; i < ARRAY_SIZE(mmp3_list_clks); i++)
		mmp3_init_one_clock(&mmp3_list_clks[i]);

	return 0;
}

postcore_initcall(mmp3_clk_init);

