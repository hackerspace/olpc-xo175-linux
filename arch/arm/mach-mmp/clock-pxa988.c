/*
 *  linux/arch/arm/mach-mmp/clock-pxa988.c
 *
 *  Author:	Zhoujie Wu <zjwu@marvell.com>
 *		Raul Xiong <xjian@marvell.com>
 *  Copyright:	(C) 2012 Marvell International Ltd.
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
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <mach/regs-apbc.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/cputype.h>
#include <mach/clock-pxa988.h>

/*
  * README:
  * 1. For clk which has fc_request bit, two step operation
  * is safer to enable clock with taget frequency
  * 1) set enable&rst bit
  * 2) set mux, div and fc to do FC, get target rate.
  */

struct periph_clk_tbl {
	unsigned long fclk;
	unsigned long aclk;  /* we haven't expose aclk as a separate clk node */
	struct clk	*fparent;
	unsigned long fsrc_val;
	unsigned long fdiv_val;
	unsigned long asrc_val;
	unsigned long adiv_val;
};

union pmum_pll2cr {
	struct {
		unsigned int reserved0:6;
		unsigned int reserved1:2;
		unsigned int en:1;
		unsigned int ctrl:1;
		unsigned int pll2fbd:9;
		unsigned int pll2refd:5;
		unsigned int reserved2:8;
	} b;
	unsigned int v;
};

union pmum_pll3cr {
	struct {
		unsigned int pll3refd:5;
		unsigned int pll3fbd:9;
		unsigned int reserved0:4;
		unsigned int pclk_1248_sel:1;
		unsigned int pll3_pu:1;
		unsigned int reserved1:12;
	} b;
	unsigned int v;
};

union apb_spare_pllswcr {
	struct {
		unsigned int lineupen:1;
		unsigned int gatectl:1;
		unsigned int bypassen:1;
		unsigned int diffclken:1;
		unsigned int divselse:4;
		unsigned int divseldiff:4;
		unsigned int ctune:2;
		unsigned int vcovnrg:3;
		unsigned int kvco:4;
		unsigned int icp:3;
		unsigned int vreg_ivreg:2;
		unsigned int vddl:4;
		unsigned int vddm:2;
	} b;
	unsigned int v;
};

/*
  * peripheral clock source:
  * 0x0 = PLL1 416 MHz
  * 0x1 = PLL1 624 MHz
  * 0x2 = PLL2_CLKOUT
  * 0x3 = PLL2_CLKOUTP
  */
enum periph_clk_src {
	CLK_PLL1_416 = 0x0,
	CLK_PLL1_624 = 0x1,
	CLK_PLL2 = 0x2,
	CLK_PLL2P = 0x3,
};

#define APB_SPARE_PLL2CR	(APB_VIRT_BASE + 0x90104)
#define APB_SPARE_PLL3CR	(APB_VIRT_BASE + 0x90108)


static DEFINE_SPINLOCK(ccic_lock);
static DEFINE_SPINLOCK(lcd_ci_share_lock);
static DEFINE_SPINLOCK(pll2_lock);
static DEFINE_SPINLOCK(pll3_lock);

#define MHZ	(1000000)
static unsigned long pll2_vco_default;
static unsigned long pll2_default;
static unsigned long pll2p_default;

static unsigned int pll_post_div_tbl[] = {
	1, 2, 3, 4, 6, 8,
};

#define CLK_SET_BITS(set, clear)	{	\
	unsigned long tmp;			\
	tmp = __raw_readl(clk->clk_rst);	\
	tmp &= ~clear;				\
	tmp |= set;				\
	__raw_writel(tmp, clk->clk_rst);	\
}						\

static struct clk ref_vco = {
	.name = "ref_vco",
	.rate = 26000000,
	.ops = NULL,
};

static struct clk pll1_416 = {
	.name = "pll1_416",
	.rate = 416000000,
	.ops = NULL,
};

static struct clk pll1_624 = {
	.name = "pll1_624",
	.rate = 624000000,
	.ops = NULL,
};

static struct clk pll1_1248 = {
	.name = "pll1_1248",
	.rate = 1248000000,
	.ops = NULL,
};

static int gate_clk_enable(struct clk *clk)
{
	CLK_SET_BITS(clk->enable_val, 0);
	return 0;
}

static void gate_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, clk->enable_val);
}

struct clkops gate_clk_ops = {
	.enable		= gate_clk_enable,
	.disable	= gate_clk_disable,
};

#define DEFINE_GATE_CLK(_name, _reg, _eval, _dev_id, _con_id)	\
	static struct clk _name = {				\
		.name = #_name,					\
		.clk_rst = (void __iomem *)_reg,		\
		.enable_val = _eval,				\
		.ops = &gate_clk_ops,				\
		.lookup = {					\
			.dev_id = _dev_id,			\
			.con_id = _con_id,			\
		},						\
	}							\

static int apbc_clk_enable(struct clk *clk)
{
	unsigned int data;

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
	unsigned int data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLK | APBC_FNCLKSEL(7));
	__raw_writel(data, clk->clk_rst);
	udelay(10);

	data &= ~APBC_APBCLK;
	__raw_writel(data, clk->clk_rst);
}

struct clkops apbc_clk_ops = {
	.enable		= apbc_clk_enable,
	.disable	= apbc_clk_disable,
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

/* PLL range 1.2G~2.5G, vco_vrng = kvco */
static void __clk_pll_rate2rng(unsigned long rate,
	unsigned int *kvco, unsigned int *vco_rng)
{
	if (rate >= 2400 && rate <= 2500)
		*kvco = 7;
	else if (rate >= 2150)
		*kvco = 6;
	else if (rate >= 1950)
		*kvco = 5;
	else if (rate >= 1750)
		*kvco = 4;
	else if (rate >= 1550)
		*kvco = 3;
	else if (rate >= 1350)
		*kvco = 2;
	else if (rate >= 1200)
		*kvco = 1;
	else
		pr_info("%s rate %lu out of range!\n",
			__func__, rate);

	*vco_rng = *kvco;
}

static unsigned int __clk_pll_calc_div(unsigned long rate,
	unsigned long parent_rate, unsigned int *div)
{
	unsigned int i;
	*div = 0;

	rate /= MHZ;
	parent_rate /= MHZ;

	for (i = 0; i < ARRAY_SIZE(pll_post_div_tbl); i++) {
		if (rate == (parent_rate / pll_post_div_tbl[i])) {
			*div = pll_post_div_tbl[i];
			return i;
		}
	}
	BUG_ON(i == ARRAY_SIZE(pll_post_div_tbl));
	return 0;
}

/*
 * 1. Whenever PLL2 is enabled, ensure it's set as HW activation.
 * 2. When PLL2 is disabled (no one uses PLL2 as source),
 * set it as SW activation.
 */
static void clk_pll2_vco_init(struct clk *clk)
{
	unsigned int kvco, vcovnrg;
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;

	BUG_ON(!pll2_vco_default);
	/* do nothing if pll2 is already enabled */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL2_VCO is already enabled @ %lu\n",
			clk->rate);
		BUG_ON(clk->rate != pll2_vco_default);
		return;
	}

	clk->rate = pll2_vco_default;
	__clk_pll_rate2rng(clk->rate/MHZ, &kvco, &vcovnrg);

	/* The default configuration of pll2 */
	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.vddm = 1;
	pll2_sw_ctrl.b.vddl = 9;
	pll2_sw_ctrl.b.vreg_ivreg = 2;
	pll2_sw_ctrl.b.icp = 4;
	pll2_sw_ctrl.b.ctune = 1;
	pll2_sw_ctrl.b.bypassen = 0;
	pll2_sw_ctrl.b.gatectl = 0;
	pll2_sw_ctrl.b.lineupen = 0;
	pll2_sw_ctrl.b.kvco = kvco;
	pll2_sw_ctrl.b.vcovnrg = vcovnrg;
	pll2_sw_ctrl.b.diffclken = 1;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);

	/* Refclk/Refdiv = pll2freq/Fbdiv, Refclk = 26M */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	pll2cr.b.pll2refd = 3;
	pll2cr.b.pll2fbd = (clk->rate/MHZ) * pll2cr.b.pll2refd / 26;
	__raw_writel(pll2cr.v, MPMU_PLL2CR);

	pr_info("PLL2 VCO default rate %lu\n", clk->rate);
}

static int clk_pll2_vco_enable(struct clk *clk)
{
	union pmum_pll2cr pll2cr;
	unsigned long flags;

	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl)
		return 0;

	spin_lock_irqsave(&pll2_lock, flags);

	/* we must lock refd/fbd first before enabling PLL2 */
	pll2cr.b.ctrl = 1;
	__raw_writel(pll2cr.v, MPMU_PLL2CR);
	pll2cr.b.ctrl = 0;	/* Let HW control PLL2 */
	__raw_writel(pll2cr.v, MPMU_PLL2CR);

	spin_unlock_irqrestore(&pll2_lock, flags);

	udelay(100);
	pr_debug("%s SWCR[%x] PLLCR[%x]\n", __func__, \
		__raw_readl(APB_SPARE_PLL2CR), pll2cr.v);
	return 0;
}

static void clk_pll2_vco_disable(struct clk *clk)
{
	union pmum_pll2cr pll2cr;

	pr_info("Disable pll2 as it is not used!\n");

	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	pll2cr.b.ctrl = 1;	/* Let SW control PLL2 */
	pll2cr.b.en = 0;	/* disable PLL2 by en bit */
	__raw_writel(pll2cr.v, MPMU_PLL2CR);
}

/*
  * pll2 rate change requires sequence:
  * clock off -> change rate setting -> clock on
  * This function doesn't really change rate, but cache the config
  */
static int clk_pll2_vco_setrate(struct clk *clk, unsigned long rate)
{
	unsigned int kvco, vcovnrg;
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;
	unsigned long flags;

	/* do nothing if pll2 is already enabled or no rate change */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl || rate == clk->rate)
		return -EPERM;

	rate = rate / MHZ;
	if (rate > 2500 || rate < 1200)	{
		pr_err("%lu rate out of range!\n", rate);
		return -EINVAL;
	}

	pr_debug("PLL2_VCO rate %lu -> %lu\n",
		clk->rate/MHZ, rate);

	spin_lock_irqsave(&pll2_lock, flags);

	__clk_pll_rate2rng(rate, &kvco, &vcovnrg);
	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.kvco = kvco;
	pll2_sw_ctrl.b.vcovnrg = vcovnrg;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);

	/* Refclk/Refdiv = pll2freq/Fbdiv Refclk = 26M */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	pll2cr.b.pll2refd = 3;
	pll2cr.b.pll2fbd = rate * pll2cr.b.pll2refd / 26;
	__raw_writel(pll2cr.v, MPMU_PLL2CR);

	clk->rate = 26 * pll2cr.b.pll2fbd / pll2cr.b.pll2refd;
	if (clk->rate != rate)
		pr_warning("Set PLL2 rate to %luMHZ\n", clk->rate);
	clk->rate *= MHZ;

	spin_unlock_irqrestore(&pll2_lock, flags);
	return 0;
}

static unsigned long clk_pll2_vco_getrate(struct clk *clk)
{
	union pmum_pll2cr pll2cr;
	unsigned long rate = 0, pll2refd;

	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	pll2refd = pll2cr.b.pll2refd;
	BUG_ON(pll2refd == 1);

	if (pll2refd == 0)
		pll2refd = 1;
	rate = 26 * pll2cr.b.pll2fbd / pll2refd;
	rate *= MHZ;
	return rate;
}

static struct clkops clk_pll2_vco_ops = {
	.init = clk_pll2_vco_init,
	.enable = clk_pll2_vco_enable,
	.disable = clk_pll2_vco_disable,
	.setrate = clk_pll2_vco_setrate,
	.getrate = clk_pll2_vco_getrate,
};

static struct clk pll2_vco = {
	.name = "pll2_vco",
	.parent = &ref_vco,
	.ops = &clk_pll2_vco_ops,
};

/* do nothing only used to adjust proper clk->refcnt */
static int clk_pll_dummy_enable(struct clk *clk)
{
	return 0;
}

static void clk_pll_dummy_disable(struct clk *clk)
{
}

static void clk_pll2_init(struct clk *clk)
{
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;
	unsigned int div_val;

	BUG_ON(!pll2_default);
	/* do nothing if pll2 is already enabled */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL2 is already enabled @ %lu\n", clk->rate);
		BUG_ON(clk->rate != pll2_default);
		return;
	}

	clk->rate = pll2_default;
	clk->mul = 1;
	div_val = __clk_pll_calc_div(clk->rate,
		clk->parent->rate, &clk->div);

	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.divselse = div_val;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);

	pr_info("PLL2 default rate %lu\n",
		clk->parent->rate * clk->mul / clk->div);
}

static int clk_pll2_setrate(struct clk *clk, unsigned long rate)
{
	unsigned int div_val;
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;
	unsigned long flags;

	/* do nothing if pll2 is already enabled */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl || rate == clk->rate)
		return -EPERM;

	pr_debug("PLL2 rate %lu -> %lu\n", clk->rate, rate);

	spin_lock_irqsave(&pll2_lock, flags);
	div_val = __clk_pll_calc_div(rate, clk->parent->rate, &clk->div);
	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.divselse = div_val;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);
	clk->rate = rate;
	spin_unlock_irqrestore(&pll2_lock, flags);
	return 0;
}

static unsigned long clk_pll2_getrate(struct clk *clk)
{
	unsigned int parent_rate = 0, rate;
	union apb_spare_pllswcr pll2_sw_ctl;

	parent_rate = clk_get_rate(clk_get_parent(clk));
	pll2_sw_ctl.v = __raw_readl(APB_SPARE_PLL2CR);
	BUG_ON(pll2_sw_ctl.b.divselse >= ARRAY_SIZE(pll_post_div_tbl));
	rate = parent_rate / pll_post_div_tbl[pll2_sw_ctl.b.divselse];
	return rate;
}

static struct clkops clk_pll2_ops = {
	.init = clk_pll2_init,
	.enable = clk_pll_dummy_enable,
	.disable = clk_pll_dummy_disable,
	.setrate = clk_pll2_setrate,
	.getrate = clk_pll2_getrate,
};

static struct clk pll2 = {
	.name = "pll2",
	.parent = &pll2_vco,
	.ops = &clk_pll2_ops,
};

static void clk_pll2p_init(struct clk *clk)
{
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;
	unsigned int div_val;

	BUG_ON(!pll2p_default);
	/* do nothing if pll2 is already enabled */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL2P is already enabled @ %lu\n", clk->rate);
		BUG_ON(clk->rate != pll2p_default);
		return;
	}

	clk->rate = pll2p_default;
	clk->mul = 1;
	div_val = __clk_pll_calc_div(clk->rate,
		clk->parent->rate, &clk->div);

	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.divseldiff = div_val;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);

	pr_info("PLL2P default rate %lu\n",
		clk->parent->rate * clk->mul / clk->div);
}

static int clk_pll2p_setrate(struct clk *clk, unsigned long rate)
{
	unsigned int div_val;
	union pmum_pll2cr pll2cr;
	union apb_spare_pllswcr pll2_sw_ctrl;
	unsigned long flags;

	/* do nothing if pll2 is already enabled */
	pll2cr.v = __raw_readl(MPMU_PLL2CR);
	if (!pll2cr.b.ctrl || rate == clk->rate)
		return -EPERM;

	pr_debug("PLL2P rate %lu -> %lu\n", clk->rate, rate);

	spin_lock_irqsave(&pll2_lock, flags);
	div_val = __clk_pll_calc_div(rate, clk->parent->rate, &clk->div);
	pll2_sw_ctrl.v = __raw_readl(APB_SPARE_PLL2CR);
	pll2_sw_ctrl.b.divseldiff = div_val;
	__raw_writel(pll2_sw_ctrl.v, APB_SPARE_PLL2CR);
	clk->rate = rate;
	spin_unlock_irqrestore(&pll2_lock, flags);
	return 0;
}

static unsigned long clk_pll2p_getrate(struct clk *clk)
{
	unsigned int parent_rate = 0, rate;
	union apb_spare_pllswcr pll2_sw_ctl;

	parent_rate = clk_get_rate(clk_get_parent(clk));
	pll2_sw_ctl.v = __raw_readl(APB_SPARE_PLL2CR);
	BUG_ON(pll2_sw_ctl.b.divseldiff >= ARRAY_SIZE(pll_post_div_tbl));
	rate = parent_rate / pll_post_div_tbl[pll2_sw_ctl.b.divseldiff];
	return rate;
}

static struct clkops clk_pll2p_ops = {
	.init = clk_pll2p_init,
	.enable = clk_pll_dummy_enable,
	.disable = clk_pll_dummy_disable,
	.setrate = clk_pll2p_setrate,
	.getrate = clk_pll2p_getrate,
};

static struct clk pll2p = {
	.name = "pll2p",
	.parent = &pll2_vco,
	.ops = &clk_pll2p_ops,
};

/* FIXME: default pll3_vco 2000M, pll3 500M(dsi), pll3p 1000M(cpu) */
static void clk_pll3_vco_init(struct clk *clk)
{
	unsigned int kvco, vcovnrg;
	union pmum_pll3cr pll3cr;
	union apb_spare_pllswcr pll3_sw_ctrl;
	unsigned long pll3_vco_default = 2000 * MHZ;

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	if (pll3cr.b.pll3_pu) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL3_VCO is already enabled @ %lu\n",
			clk->rate);
		BUG_ON(clk->rate != pll3_vco_default);
		return;
	}

	clk->rate = pll3_vco_default;
	__clk_pll_rate2rng(clk->rate/MHZ, &kvco, &vcovnrg);

	pll3_sw_ctrl.v = __raw_readl(APB_SPARE_PLL3CR);
	pll3_sw_ctrl.b.vddm = 1;
	pll3_sw_ctrl.b.vddl = 9;
	pll3_sw_ctrl.b.vreg_ivreg = 2;
	pll3_sw_ctrl.b.icp = 4;
	pll3_sw_ctrl.b.ctune = 1;
	pll3_sw_ctrl.b.bypassen = 0;
	pll3_sw_ctrl.b.gatectl = 0;
	pll3_sw_ctrl.b.lineupen = 0;
	pll3_sw_ctrl.b.kvco = kvco;
	pll3_sw_ctrl.b.vcovnrg = vcovnrg;
	pll3_sw_ctrl.b.diffclken = 1;
	__raw_writel(pll3_sw_ctrl.v, APB_SPARE_PLL3CR);

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	pll3cr.b.pll3refd = 3;
	pll3cr.b.pll3fbd = (clk->rate/MHZ) * pll3cr.b.pll3refd / 26;
	__raw_writel(pll3cr.v, MPMU_PLL3CR);

	pr_info("PLL3 VCO default rate %lu\n", clk->rate);
}

/* PLL3CR[19:18] = 0x0 shutdown */
/* PLL3CR[19:18] = 0x3 enable */
static int clk_pll3_vco_enable(struct clk *clk)
{
	union pmum_pll3cr pll3cr;
	unsigned long flags;

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	if (pll3cr.b.pll3_pu)
		return 0;

	spin_lock_irqsave(&pll3_lock, flags);
	pll3cr.b.pclk_1248_sel = 1;
	pll3cr.b.pll3_pu = 1;
	__raw_writel(pll3cr.v, MPMU_PLL3CR);
	spin_unlock_irqrestore(&pll2_lock, flags);

	udelay(100);
	pr_debug("%s SWCR3[%x] PLL3CR[%x]\n", __func__, \
		__raw_readl(APB_SPARE_PLL3CR), pll3cr.v);
	return 0;
}

static void clk_pll3_vco_disable(struct clk *clk)
{
	union pmum_pll3cr pll3cr;

	pr_info("Disable pll3 as it is not used!\n");

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	pll3cr.b.pclk_1248_sel = 0;
	pll3cr.b.pll3_pu = 0;
	__raw_writel(pll3cr.v, MPMU_PLL3CR);
}

static unsigned long clk_pll3_vco_getrate(struct clk *clk)
{
	union pmum_pll3cr pll3cr;
	unsigned long rate = 0, pll3refd;

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	pll3refd = pll3cr.b.pll3refd;
	BUG_ON(pll3refd == 1);

	if (pll3refd == 0)
		pll3refd = 1;
	rate = 26 * pll3cr.b.pll3fbd / pll3refd;
	rate *= MHZ;
	return rate;
}

static struct clkops clk_pll3_vco_ops = {
	.init = clk_pll3_vco_init,
	.enable = clk_pll3_vco_enable,
	.disable = clk_pll3_vco_disable,
	.getrate = clk_pll3_vco_getrate,
};

static struct clk pll3_vco = {
	.name = "pll3_vco",
	.parent = &ref_vco,
	.ops = &clk_pll3_vco_ops,
};

static void clk_pll3_init(struct clk *clk)
{
	union pmum_pll3cr pll3cr;
	union apb_spare_pllswcr pll3_sw_ctrl;
	unsigned int div_val;
	unsigned long pll3_default = 500 * MHZ;

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	if (pll3cr.b.pll3_pu) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL3 is already enabled @ %lu\n",
			clk->rate);
		BUG_ON(clk->rate != pll3_default);
		return;
	}

	clk->rate = pll3_default;
	clk->mul = 1;
	div_val = __clk_pll_calc_div(clk->rate,
		clk->parent->rate, &clk->div);

	pll3_sw_ctrl.v = __raw_readl(APB_SPARE_PLL3CR);
	pll3_sw_ctrl.b.divselse = div_val;
	__raw_writel(pll3_sw_ctrl.v, APB_SPARE_PLL3CR);

	pr_info("PLL3 default rate %lu\n",
		clk->parent->rate * clk->mul / clk->div);
}

static unsigned long clk_pll3_getrate(struct clk *clk)
{
	unsigned int parent_rate = 0, rate;
	union apb_spare_pllswcr pll3_sw_ctl;

	parent_rate = clk_get_rate(clk_get_parent(clk));
	pll3_sw_ctl.v = __raw_readl(APB_SPARE_PLL3CR);
	BUG_ON(pll3_sw_ctl.b.divselse >= ARRAY_SIZE(pll_post_div_tbl));
	rate = parent_rate / pll_post_div_tbl[pll3_sw_ctl.b.divselse];
	return rate;
}

static struct clkops clk_pll3_ops = {
	.init = clk_pll3_init,
	.enable = clk_pll_dummy_enable,
	.disable = clk_pll_dummy_disable,
	.getrate = clk_pll3_getrate,
};

static struct clk pll3 = {
	.name = "pll3",
	.parent = &pll3_vco,
	.ops = &clk_pll3_ops,
};

static void clk_pll3p_init(struct clk *clk)
{
	union pmum_pll3cr pll3cr;
	union apb_spare_pllswcr pll3_sw_ctrl;
	unsigned int div_val;
	unsigned long pll3p_default = 1000 * MHZ;

	pll3cr.v = __raw_readl(MPMU_PLL3CR);
	if (pll3cr.b.pll3_pu) {
		clk->rate = clk_get_rate(clk);
		pr_info("PLL3P is already enabled @ %lu\n",
			clk->rate);
		BUG_ON(clk->rate != pll3p_default);
		return;
	}

	clk->rate = pll3p_default;
	clk->mul = 1;
	div_val = __clk_pll_calc_div(clk->rate,
		clk->parent->rate, &clk->div);

	pll3_sw_ctrl.v = __raw_readl(APB_SPARE_PLL3CR);
	pll3_sw_ctrl.b.divseldiff = div_val;
	__raw_writel(pll3_sw_ctrl.v, APB_SPARE_PLL3CR);

	pr_info("PLL3P default rate %lu\n",
		clk->parent->rate * clk->mul / clk->div);
}

static unsigned long clk_pll3p_getrate(struct clk *clk)
{
	unsigned int parent_rate = 0, rate;
	union apb_spare_pllswcr pll3_sw_ctl;

	parent_rate = clk_get_rate(clk_get_parent(clk));
	pll3_sw_ctl.v = __raw_readl(APB_SPARE_PLL3CR);
	BUG_ON(pll3_sw_ctl.b.divseldiff >= ARRAY_SIZE(pll_post_div_tbl));
	rate = parent_rate / pll_post_div_tbl[pll3_sw_ctl.b.divseldiff];
	return rate;
}

static struct clkops clk_pll3p_ops = {
	.init = clk_pll3p_init,
	.enable = clk_pll_dummy_enable,
	.disable = clk_pll_dummy_disable,
	.getrate = clk_pll3p_getrate,
};

static struct clk pll3p = {
	.name = "pll3p",
	.parent = &pll3_vco,
	.ops = &clk_pll3p_ops,
};

#define SDH_ACLK_EN		((1 << 3) | (1 << 0))
#define SDH_FCLK_EN		((1 << 4) | (1 << 1))
#define SDH_FCLK_SEL(n)		((n & 0x1) << 6)
#define SDH_FCLK_SEL_MASK	SDH_FCLK_SEL(0x1)
#define SDH_FCLK_DIV(n)		((n & 0x7) << 8)
#define SDH_FCLK_DIV_MASK	SDH_FCLK_DIV(0x7)
#define SDH_FCLK_REQ		(1 << 11)
#define SDH_CLK_RATE_MASK	\
	(SDH_FCLK_SEL_MASK | SDH_FCLK_DIV_MASK)

/* dummy clock used for 3 SDH shared AXI bus enable */
DEFINE_GATE_CLK(sdh_shared_axi, APMU_SDH0, SDH_ACLK_EN, NULL, NULL);

static struct clk *sdhc_share_clk[] = {
	&sdh_shared_axi,
};

static struct clk_mux_sel sdhc_clk_mux[] = {
	{.input = &pll1_416, .value = CLK_PLL1_416},
	{.input = &pll1_624, .value = CLK_PLL1_624},
	{NULL, 0},
};

static void sdhc_clk_init(struct clk *clk)
{
	unsigned int mux, i;

	clk->mul = 1;
	clk->div = (clk->enable_val & SDH_FCLK_DIV_MASK) >> \
		(__ffs(SDH_FCLK_DIV_MASK));
	mux = (clk->enable_val & SDH_FCLK_SEL_MASK) >> \
		(__ffs(SDH_FCLK_SEL_MASK));

	i = 0;
	while ((clk->inputs[i].input) && (clk->inputs[i].value != mux))
		i++;
	BUG_ON(!clk->inputs[i].input);

	clk_reparent(clk, clk->inputs[i].input);
	clk->div += 1;
	clk->rate = clk_get_rate(clk->parent) * clk->mul / clk->div;
}

static int sdhc_clk_enable(struct clk *clk)
{
	CLK_SET_BITS(SDH_FCLK_EN, 0);
	CLK_SET_BITS(clk->enable_val, SDH_CLK_RATE_MASK);
	return 0;
}

static void sdhc_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, SDH_FCLK_EN);
}

struct clkops sdhc_clk_ops = {
	.init = sdhc_clk_init,
	.enable = sdhc_clk_enable,
	.disable = sdhc_clk_disable,
};

static struct clk pxa988_clk_sdh0 = {
	.name = "sdh0",
	.lookup = {
		.dev_id = "sdhci-pxa.0",
		.con_id = "PXA-SDHCLK",
	},
	.clk_rst = (void __iomem *)APMU_SDH0,
	/* SDcard 208M */
	.enable_val =
		SDH_FCLK_SEL(CLK_PLL1_416)|SDH_FCLK_DIV(1)|SDH_FCLK_REQ,
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.dependence = sdhc_share_clk,
	.dependence_count = 1,
};

static struct clk pxa988_clk_sdh1 = {
	.name = "sdh1",
	.lookup = {
		.dev_id = "sdhci-pxa.1",
		.con_id = "PXA-SDHCLK",
	},
	.clk_rst = (void __iomem *)APMU_SDH1,
	/* Wifi 208M */
	.enable_val =
		SDH_FCLK_SEL(CLK_PLL1_416)|SDH_FCLK_DIV(1)|SDH_FCLK_REQ,
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.dependence = sdhc_share_clk,
	.dependence_count = 1,
};

static struct clk pxa988_clk_sdh2 = {
	.name = "sdh2",
	.lookup = {
		.dev_id = "sdhci-pxa.2",
		.con_id = "PXA-SDHCLK",
	},
	.clk_rst = (void __iomem *)APMU_SDH2,
	/* Emmc 208M */
	.enable_val =
		SDH_FCLK_SEL(CLK_PLL1_416)|SDH_FCLK_DIV(1)|SDH_FCLK_REQ,
	.ops = &sdhc_clk_ops,
	.inputs = sdhc_clk_mux,
	.dependence = sdhc_share_clk,
	.dependence_count = 1,
};

static struct clk_mux_sel periph_mux_sel[] = {
	{.input = &pll1_416, .value = CLK_PLL1_416},
	{.input = &pll1_624, .value = CLK_PLL1_624},
	{.input = &pll2, .value = CLK_PLL2},
	{.input = &pll2p, .value = CLK_PLL2P},
	{NULL, 0},
};

static void __clk_fill_periph_tbl(struct clk *clk,
	struct periph_clk_tbl *clk_tbl, unsigned int clk_tbl_size)
{
	unsigned int i = 0;
	unsigned long prate = 0;
	const struct clk_mux_sel *sel;

	pr_info("************** clk_%s_tbl  **************\n", clk->name);

	/* Fill fclk setting */
	for (i = 0; i < clk_tbl_size; i++) {
		for (sel = clk->inputs; sel->input != NULL; sel++) {
			if (sel->input == clk_tbl[i].fparent) {
				prate = clk_get_rate(sel->input);
				clk_tbl[i].fsrc_val = sel->value;
				clk_tbl[i].fdiv_val =
					prate/clk_tbl[i].fclk - 1;
				pr_info("fclk[%lu] fsrc[%lu:%lu] fdiv[%lu]\n",
					clk_tbl[i].fclk, prate, \
					clk_tbl[i].fsrc_val, \
					clk_tbl[i].fdiv_val);
				break;
			}
		}
	}
	/* Fill aclk setting, aclk and fclk may from different parent */
	for (i = 0; i < clk_tbl_size; i++) {
		for (sel = clk->inputs; sel->input != NULL; sel++) {
			prate = clk_get_rate(sel->input);
			if ((prate % clk_tbl[i].aclk == 0) && prate) {
				clk_tbl[i].asrc_val = sel->value;
				clk_tbl[i].adiv_val =
					prate/clk_tbl[i].aclk - 1;
				pr_info("aclk[%lu] asrc[%lu:%lu] adiv[%lu]\n",
					clk_tbl[i].aclk, prate,
					clk_tbl[i].asrc_val,
					clk_tbl[i].adiv_val);
				break;
			}
		}
	}
}

static long __clk_round_rate_bytbl(struct clk *clk, unsigned long rate,
	struct periph_clk_tbl *clk_tbl, unsigned int clk_tbl_size)
{
	unsigned int i;

	if (unlikely(rate > clk_tbl[clk_tbl_size - 1].fclk))
		return clk_tbl[clk_tbl_size - 1].fclk;

	for (i = 0; i < clk_tbl_size; i++) {
		if (rate <= clk_tbl[i].fclk)
			return clk_tbl[i].fclk;
	}

	return clk->rate;
}

/*
  * This help function can get the rate close to the required rate,
  * we'd better not use it for clock which need to dynamic change
  * as efficiency consideration.
  */
static long __clk_sel_mux_div(struct clk *clk, unsigned long rate,
	unsigned int *mux, unsigned int *div, struct clk **best_parent)
{
	const struct clk_mux_sel *sel;
	struct clk *parent_sel = NULL;
	unsigned int i, bestdiv = 0, bestmux = 0;
	unsigned long parent_rate, best = 0, now, maxdiv;

	maxdiv = clk->reg_data[DIV][CONTROL].reg_mask + 1;
	if (rate < (clk_get_rate(clk->inputs[0].input) / maxdiv))
		rate = clk_get_rate(clk->inputs[0].input) / maxdiv;
	/*
	 * The maximum divider we can use without overflowing
	 * unsigned long in rate * i below
	 */
	maxdiv = min(ULONG_MAX / rate, maxdiv);
	for (sel = clk->inputs; sel->input != NULL; sel++) {
		parent_rate = clk_get_rate(sel->input);
		for (i = 1; i <= maxdiv; i++) {
			now = parent_rate / i;
			if (now <= rate && now > best) {
				bestdiv = i;
				best = now;
				parent_sel = sel->input;
				bestmux = sel->value;
			}
		}
	}
	BUG_ON(!bestdiv);

	*div = bestdiv;
	*mux = bestmux;
	*best_parent = parent_sel;

	pr_debug("%s clk:%s mux:%u div:%u, %lu\n", __func__, \
		clk->name, *mux, *div, clk_get_rate(parent_sel));

	return clk_get_rate(parent_sel)/bestdiv;
}

static long __clk_set_mux_div(struct clk *clk, struct clk *best_parent,
	unsigned int mux, unsigned int div)
{
	unsigned int muxmask, divmask;
	unsigned int muxval, divval;
	unsigned int regval;

	BUG_ON(!div);

	if (clk->refcnt)
		clk_enable(best_parent);

	clk->div = div;
	clk->mul = 1;

	div = div - 1;	/* rate = parent_rate / (div_regval + 1) */

	muxval = (mux > clk->reg_data[SOURCE][CONTROL].reg_mask) ? \
		clk->reg_data[SOURCE][CONTROL].reg_mask : mux;
	divval = (div > clk->reg_data[DIV][CONTROL].reg_mask) ? \
		clk->reg_data[DIV][CONTROL].reg_mask : div;

	muxmask = clk->reg_data[SOURCE][CONTROL].reg_mask << \
		clk->reg_data[SOURCE][CONTROL].reg_shift;
	divmask = clk->reg_data[DIV][CONTROL].reg_mask << \
		clk->reg_data[DIV][CONTROL].reg_shift;

	muxval = muxval << clk->reg_data[SOURCE][CONTROL].reg_shift;
	divval = divval << clk->reg_data[DIV][CONTROL].reg_shift;

	/*
	  * mux and div are from the same reg, if clk is enabled,
	  * set mux, div and trigger(clk->enable_val) at the same time
	  * if clock is not enabled, set mux, div here, set fc_request when
	  * enable it.
	  */
	regval = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);
	regval &= ~(muxmask | divmask);
	regval |= (muxval | divval);
	if (clk->refcnt)
		regval |= clk->enable_val;
	__raw_writel(regval, clk->reg_data[SOURCE][CONTROL].reg);

	if (clk->refcnt && clk->parent)
		clk_disable(clk->parent);
	clk_reparent(clk, best_parent);

	pr_debug("\n%s clk:%s [%x] = [%x]\n", __func__, clk->name, \
		clk->reg_data[SOURCE][CONTROL].reg, regval);

	return 0;
}

static void __clk_get_mux_div(struct clk *clk,
		unsigned int *mux, unsigned int *div)
{
	unsigned int muxmask, divmask;
	unsigned int muxval, divval;
	unsigned int regval;

	regval = __raw_readl(clk->reg_data[SOURCE][CONTROL].reg);
	muxmask = clk->reg_data[SOURCE][CONTROL].reg_mask << \
		clk->reg_data[SOURCE][CONTROL].reg_shift;
	divmask = clk->reg_data[DIV][CONTROL].reg_mask << \
		clk->reg_data[DIV][CONTROL].reg_shift;

	muxval = (regval & muxmask) >> __ffs(muxmask);
	divval = (regval & divmask) >> __ffs(divmask);

	pr_debug("\n%s clk:%s [%x]val%x mux[%d] div[%d]\n",
		__func__, clk->name, clk->reg_data[SOURCE][CONTROL].reg,
		regval, muxval, divval+1);

	*mux = muxval;
	*div = divval + 1;
}

static struct clk *__clk_mux_to_parent(struct clk *clk, unsigned int mux)
{
	unsigned int i;

	i = 0;
	while ((clk->inputs[i].input) && (clk->inputs[i].value != mux))
		i++;
	BUG_ON(!clk->inputs[i].input);

	return clk->inputs[i].input;
}

static unsigned int __clk_parent_to_mux(struct clk *clk, struct clk *parent)
{
	unsigned int i;

	i = 0;
	while ((clk->inputs[i].input) && (clk->inputs[i].input != parent))
		i++;
	BUG_ON(!clk->inputs[i].input);

	return clk->inputs[i].value;
}

static void __clk_periph_init(struct clk *clk,
	struct clk *parent, unsigned int div, bool dyn_chg)
{
	unsigned int mux = 0;

	clk->dynamic_change = dyn_chg;
	clk->mul = 1;
	clk->div = div;
	mux  = __clk_parent_to_mux(clk, parent);
	__clk_set_mux_div(clk, parent, mux, div);
	clk->rate = clk_get_rate(clk->parent) * clk->mul / clk->div;
}

static unsigned long __clk_periph_get_rate(struct clk *clk)
{
	struct clk *cur_parent;
	unsigned int mux, div = 1;

	__clk_get_mux_div(clk, &mux, &div);
	cur_parent = __clk_mux_to_parent(clk, mux);

	return clk_get_rate(cur_parent) / div;
}

static int __clk_periph_set_rate(struct clk *clk, unsigned long rate)
{
	unsigned long new_rate;
	unsigned int mux, div;
	struct clk *best_parent;

	new_rate = __clk_sel_mux_div(clk, rate, &mux, &div, &best_parent);
	if (rate != new_rate)
		pr_warning("clk[%s] required rate %lu, set as %lu\n", \
			clk->name, rate, new_rate);
	__clk_set_mux_div(clk, best_parent, mux, div);
	return 0;
}

#define GC_ACLK_EN	(0x1 << 3)
#define GC_FCLK_EN	(0x1 << 4)
#define GC_HCLK_EN	(0x1 << 5)

#define GC_FCLK_SEL(n)		((n & 0x3) << 6)
#define GC_FCLK_SEL_MASK	GC_FCLK_SEL(0x3)
#define GC_FCLK_DIV(n)		((n & 0x7) << 12)
#define GC_FCLK_DIV_MASK	GC_FCLK_DIV(0x7)
#define GC_FCLK_REQ		(0x1 << 15)

#define GC_ACLK_SEL(n)		((n & 0x3) << 20)
#define GC_ACLK_SEL_MASK	GC_ACLK_SEL(0x3)
#define GC_ACLK_DIV(n)		((n & 0x7) << 17)
#define GC_ACLK_DIV_MASK	GC_ACLK_DIV(0x7)
#define GC_ACLK_REQ		(0x1 << 16)

#define GC_FCLK_RATE(fsrc, fdiv)	\
	(GC_FCLK_SEL(fsrc) | GC_FCLK_DIV(fdiv))

#define GC_ACLK_RATE(asrc, adiv)	\
	(GC_ACLK_SEL(asrc) | GC_ACLK_DIV(adiv))

#define GC_CLK_RATE_MSK					\
	(GC_FCLK_SEL_MASK | GC_FCLK_DIV_MASK		\
	| GC_ACLK_SEL_MASK | GC_ACLK_DIV_MASK)		\


/*
  * 1. sort ascending
  * 2. Please don't select aclk from pll2 but fclk from pll1,
  * as aclk is not exposed as a clock node. Pll2 may be
  * shutdown if no clock node is using it.
  * TODO: GC 624M is unsafe PP in Z0, enable it after SV
  * confirm it is safe to be used.
  */
static struct periph_clk_tbl gc_clk_tbl[] = {
	{.fclk = 156000000, .aclk = 156000000, .fparent = &pll1_624},
	{.fclk = 312000000, .aclk = 208000000, .fparent = &pll1_624},
	{.fclk = 416000000, .aclk = 312000000, .fparent = &pll1_416},
#if 0
	{.fclk = 624000000, .aclk = 312000000, .fparent = &pll1_624},
#endif
};

static void gc_clk_init(struct clk *clk)
{
	clk->dynamic_change = 1;
	__clk_fill_periph_tbl(clk, gc_clk_tbl, ARRAY_SIZE(gc_clk_tbl));

	/* default GC fclk2x = 416M sel = pll1_416, div = 1 */
	clk_reparent(clk, &pll1_416);
	clk->mul = 1;
	clk->div = 1;
	clk->rate = clk_get_rate(clk_get_parent(clk)) * clk->mul / clk->div;
	/* default GC aclk = 312M sel = pll1_624, div = 2 */
	/* enable_val used to hack GC aclk value */
	clk->enable_val = GC_ACLK_RATE(CLK_PLL1_624, 1);
}

static int gc_clk_enable(struct clk *clk)
{
	unsigned int rate_cfg, en_cfg;
	unsigned int i;

	i = 0;
	while ((clk->inputs[i].input) && (clk->inputs[i].input != clk->parent))
		i++;
	BUG_ON(!clk->inputs[i].input);

	en_cfg = (GC_ACLK_EN | GC_FCLK_EN | GC_HCLK_EN);
	rate_cfg = GC_FCLK_RATE(clk->inputs[i].value, clk->div);
	rate_cfg |= clk->enable_val;
	rate_cfg |= (GC_FCLK_REQ | GC_ACLK_REQ);
	CLK_SET_BITS(en_cfg, 0);
	CLK_SET_BITS(rate_cfg, GC_CLK_RATE_MSK);
	return 0;
}

static void gc_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, GC_ACLK_EN | GC_FCLK_EN | GC_HCLK_EN);
}

static long gc_clk_round_rate(struct clk *clk, unsigned long rate)
{
	return __clk_round_rate_bytbl(clk, rate, \
		gc_clk_tbl, ARRAY_SIZE(gc_clk_tbl));
}

static int gc_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_cfg;
	unsigned int i;
	struct clk *new_parent;

	if (rate == clk->rate)
		return 0;

	i = 0;
	while (gc_clk_tbl[i].fclk != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(gc_clk_tbl));

	new_parent = gc_clk_tbl[i].fparent;
	if (clk->refcnt)
		clk_enable(new_parent);

	clk->mul = 1;
	clk->div = gc_clk_tbl[i].fdiv_val + 1;
	clk->enable_val =
		GC_ACLK_RATE(gc_clk_tbl[i].asrc_val, gc_clk_tbl[i].adiv_val);
	rate_cfg =
		GC_FCLK_RATE(gc_clk_tbl[i].fsrc_val, gc_clk_tbl[i].fdiv_val);
	/*
	  * FIXME: Aclk and fclk is changed together, need
	  * confirm whether aclk will change or not
	  */
	rate_cfg |= clk->enable_val;
	/* Do NOT trigger fc_request if clock is not enabled */
	if (clk->refcnt)
		rate_cfg |= (GC_FCLK_REQ | GC_ACLK_REQ);
	CLK_SET_BITS(rate_cfg, GC_CLK_RATE_MSK);

	if (clk->refcnt && clk->parent)
		clk_disable(clk->parent);
	clk_reparent(clk, new_parent);

	pr_debug("%s rate %lu->%lu\n", __func__, clk->rate, rate);
	return 0;
}

static unsigned long gc_clk_getrate(struct clk *clk)
{
	/* can only get fclk here */
	return __clk_periph_get_rate(clk);
}

struct clkops gc_clk_ops = {
	.init		= gc_clk_init,
	.enable		= gc_clk_enable,
	.disable	= gc_clk_disable,
	.round_rate	= gc_clk_round_rate,
	.setrate	= gc_clk_setrate,
	.getrate	= gc_clk_getrate,
};

static struct clk pxa988_clk_gc = {
	.name = "gc",
	.lookup = {
		.con_id = "GCCLK",
	},
	.clk_rst = (void __iomem *)APMU_GC,
	.inputs = periph_mux_sel,
	.ops = &gc_clk_ops,
	.reg_data = {
		     { {APMU_GC, 6, 0x3}, {APMU_GC, 6, 0x3} },
		     { {APMU_GC, 12, 0x7}, {APMU_GC, 12, 0x7} } }
};

#define VPU_ACLK_EN	(0x1 << 3)
#define VPU_FCLK_EN	(0x1 << 4)
#define	VPU_AHBCLK_EN	(0x1 << 5)
#define VPU_CLK_EN				\
	(VPU_ACLK_EN | VPU_FCLK_EN		\
	| VPU_AHBCLK_EN)			\

#define VPU_FCLK_SEL(n)		((n & 0x3) << 6)
#define	VPU_FCLK_SEL_MASK	VPU_FCLK_SEL(0x3)
#define VPU_FCLK_DIV(n)		((n & 0x7) << 8)
#define VPU_FCLK_DIV_MASK	VPU_FCLK_DIV(0x7)
#define VPU_FCLK_REQ		(0x1 << 20)

#define VPU_ACLK_SEL(n)		((n & 0x3) << 11)
#define VPU_ACLK_SEL_MASK	VPU_ACLK_SEL(0x3)
#define VPU_ACLK_DIV(n)		((n & 0x7) << 13)
#define VPU_ACLK_DIV_MASK	VPU_ACLK_DIV(0x7)
#define VPU_ACLK_REQ		(0x1 << 21)

#define VPU_FCLK_RATE(fsrc, fdiv) \
	(VPU_FCLK_SEL(fsrc) | VPU_FCLK_DIV(fdiv))

#define VPU_ACLK_RATE(asrc, adiv) \
	(VPU_ACLK_SEL(asrc) | VPU_ACLK_DIV(adiv))

#define VPU_CLK_RATE_MSK				\
	(VPU_FCLK_SEL_MASK | VPU_FCLK_DIV_MASK		\
	| VPU_ACLK_SEL_MASK | VPU_ACLK_DIV_MASK)	\


/*
  * 1. sort ascending
  * 2. Please do NOT select aclk from pll2 but fclk is from pll1,
  * as aclk is not exposed as a clock node. Pll2 may be shutdown
  * if no clock node is using it.
  * TODO: VPU 416M is unsafe PP in Z0, enable it after SV
  * confirm it is safe to be used according to performance
  * requirement
  */
static struct periph_clk_tbl vpu_clk_tbl[] = {
	{.fclk = 156000000, .aclk = 156000000, .fparent = &pll1_624},
	{.fclk = 208000000, .aclk = 208000000, .fparent = &pll1_416},
	{.fclk = 312000000, .aclk = 312000000, .fparent = &pll1_624},
#if 0
	{.fclk = 416000000, .aclk = 312000000, .fparent = &pll1_416},
#endif
};

static void vpu_clk_init(struct clk *clk)
{
	clk->dynamic_change = 1;
	__clk_fill_periph_tbl(clk, vpu_clk_tbl, ARRAY_SIZE(vpu_clk_tbl));

	/* default VPU fclk = 312M sel = pll1_624, div = 2 */
	clk_reparent(clk, &pll1_624);
	clk->mul = 1;
	clk->div = 2;
	clk->rate = clk_get_rate(clk_get_parent(clk)) * clk->mul / clk->div;
	/* default VPU aclk = 312M sel = pll1_624, div = 2 */
	/* enable_val used to hack VPU aclk value */
	clk->enable_val = VPU_ACLK_RATE(CLK_PLL1_624, 1);
}

static int vpu_clk_enable(struct clk *clk)
{
	unsigned int reg_cfg, en_cfg;
	unsigned int i;

	i = 0;
	while ((clk->inputs[i].input) && (clk->inputs[i].input != clk->parent))
		i++;
	BUG_ON(!clk->inputs[i].input);

	en_cfg = VPU_CLK_EN;
	reg_cfg = VPU_FCLK_RATE(clk->inputs[i].value, clk->div);
	reg_cfg |= clk->enable_val;
	reg_cfg |= (VPU_FCLK_REQ | VPU_ACLK_REQ);
	CLK_SET_BITS(en_cfg, 0);
	CLK_SET_BITS(reg_cfg, VPU_CLK_RATE_MSK);
	return 0;
}

static void vpu_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, VPU_CLK_EN);
}

static long vpu_clk_round_rate(struct clk *clk, unsigned long rate)
{
	return __clk_round_rate_bytbl(clk, rate, \
		vpu_clk_tbl, ARRAY_SIZE(vpu_clk_tbl));
}

static int vpu_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long rate_cfg;
	unsigned int i;
	struct clk *new_parent;

	if (rate == clk->rate)
		return 0;

	i = 0;
	while (vpu_clk_tbl[i].fclk != rate)
		i++;
	BUG_ON(i == ARRAY_SIZE(vpu_clk_tbl));

	new_parent = vpu_clk_tbl[i].fparent;
	if (clk->refcnt)
		clk_enable(new_parent);

	clk->mul = 1;
	clk->div = vpu_clk_tbl[i].fdiv_val + 1;
	clk->enable_val =
		VPU_ACLK_RATE(vpu_clk_tbl[i].asrc_val, vpu_clk_tbl[i].adiv_val);
	rate_cfg =
		VPU_FCLK_RATE(vpu_clk_tbl[i].fsrc_val, vpu_clk_tbl[i].fdiv_val);
	/*
	  * FIXME: Aclk and fclk is changed together, need
	  * confirm whether aclk will change or not
	  */
	rate_cfg |= clk->enable_val;
	/* Do NOT trigger fc_request if clock is not enabled */
	if (clk->refcnt)
		rate_cfg |= (VPU_FCLK_REQ | VPU_ACLK_REQ);
	CLK_SET_BITS(rate_cfg, VPU_CLK_RATE_MSK);

	if (clk->refcnt && clk->parent)
		clk_disable(clk->parent);
	clk_reparent(clk, new_parent);

	pr_debug("%s rate %lu->%lu\n", __func__, clk->rate, rate);
	return 0;
}

static unsigned long vpu_clk_getrate(struct clk *clk)
{
	/* can only get fclk here */
	return __clk_periph_get_rate(clk);
}

struct clkops vpu_clk_ops = {
	.init		= vpu_clk_init,
	.enable		= vpu_clk_enable,
	.disable	= vpu_clk_disable,
	.round_rate	= vpu_clk_round_rate,
	.setrate	= vpu_clk_setrate,
	.getrate	= vpu_clk_getrate,
};

static struct clk pxa988_clk_vpu = {
	.name = "vpu",
	.lookup = {
		.con_id = "VPUCLK",
	},
	.inputs = periph_mux_sel,
	.clk_rst = (void __iomem *)APMU_VPU_CLK_RES_CTRL,
	.ops = &vpu_clk_ops,
	.reg_data = {
		     { {APMU_VPU_CLK_RES_CTRL, 6, 0x3},
			{APMU_VPU_CLK_RES_CTRL, 6, 0x3} },
		     { {APMU_VPU_CLK_RES_CTRL, 8, 0x7},
			{APMU_VPU_CLK_RES_CTRL, 8, 0x7} } }
};

#define LCD_CI_ISP_ACLK_REQ		(1 << 22)
#define LCD_CI_ISP_ACLK_RST		(1 << 16)

/*
  * 1. This AXI clock is shared by LCD/CI/ISP
  * 2. Separate bit in LCD/CI/ISP_CLK_RES_REG is used
  * to enable its own bus clock
  * 3. This clock should NOT be dynamic changed
  * 4. Register setting is defined in LCD_CLK_RES_REG
  * 5. The safe maximum rate is 266M per DE's suggestion on Z0
  * 6. Use 208M for Z0 bringup at first
  */
static void lcd_ci_isp_axi_clk_init(struct clk *clk)
{
	__clk_periph_init(clk, &pll1_416, 2, 0);
}

static int lcd_ci_isp_axi_clk_enable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS(LCD_CI_ISP_ACLK_RST, 0);
	CLK_SET_BITS(LCD_CI_ISP_ACLK_REQ, 0);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
	return 0;
}

static void lcd_ci_isp_axi_clk_disable(struct clk *clk)
{
	unsigned long flags;

	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS(0, LCD_CI_ISP_ACLK_RST);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
}

static long lcd_ci_isp_axi_clk_roundrate(struct clk *clk, unsigned long rate)
{
	/*
	 * LCD_CI_ISP_AXI has four clock source:
	 * PLL1_416, PLL1_624, pll2 and pll2p.
	 * We don't limit the clock selection here as rate
	 * select will be handled in .setrate operation
	 */
	return rate;
}

static int lcd_ci_isp_axi_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;

	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	__clk_periph_set_rate(clk, rate);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
	return 0;
}

static unsigned long lcd_ci_isp_axi_clk_getrate(struct clk *clk)
{
	unsigned long rate, flags;

	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	rate = __clk_periph_get_rate(clk);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
	return rate;
}

struct clkops lcd_ci_isp_axi_clk_ops = {
	.init		= lcd_ci_isp_axi_clk_init,
	.enable		= lcd_ci_isp_axi_clk_enable,
	.disable	= lcd_ci_isp_axi_clk_disable,
	.round_rate	= lcd_ci_isp_axi_clk_roundrate,
	.setrate	= lcd_ci_isp_axi_clk_setrate,
	.getrate	= lcd_ci_isp_axi_clk_getrate,
};

/* bus clock shared by lcd, ci and isp */
static struct clk lcd_ci_isp_axi_clk = {
	.name = "lcd_ci_isp_axi",
	.lookup = {
		.con_id = "LCDCIISPAXI",
	},
	.clk_rst = (void __iomem *)APMU_LCD,
	.inputs = periph_mux_sel,
	.ops = &lcd_ci_isp_axi_clk_ops,
	.reg_data = {
		     { {APMU_LCD, 17, 0x3}, {APMU_LCD, 17, 0x3} },
		     { {APMU_LCD, 19, 0x7}, {APMU_LCD, 19, 0x7} } }
};

#define LCD_CI_HCLK_EN		(1 << 5)
#define LCD_CI_HCLK_RST		(1 << 2)

static int lcd_ci_hclk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS((LCD_CI_HCLK_EN | LCD_CI_HCLK_RST) , 0);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
	return 0;
}

static void lcd_ci_hclk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS(0, LCD_CI_HCLK_EN);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
}

struct clkops  lcd_ci_hclk_ops = {
	.enable		= lcd_ci_hclk_enable,
	.disable	= lcd_ci_hclk_disable,
};

static struct clk pxa988_lcd_ci_hclk = {
	.name = "lcd_ci_hclk",
	.lookup = {
		.con_id = "LCDCIHCLK",
	},
	.clk_rst = (void __iomem *)APMU_LCD,
	.ops = &lcd_ci_hclk_ops,
};

static inline int __ccic_clk_common_enable(struct clk *clk, unsigned int bits)
{
	unsigned long flags;
	spin_lock_irqsave(&ccic_lock, flags);
	CLK_SET_BITS(bits, 0);
	spin_unlock_irqrestore(&ccic_lock, flags);
	return 0;
}

static inline void __ccic_clk_common_disable(struct clk *clk, unsigned int bits)
{
	unsigned long flags;
	spin_lock_irqsave(&ccic_lock, flags);
	CLK_SET_BITS(0, bits);
	spin_unlock_irqrestore(&ccic_lock, flags);
}

#define CCIC_AXI_EN	((1 << 0) | (1 << 3))
#define CCIC_AXI_DIS	(1 << 3)

static int ccic_axi_clk_enable(struct clk *clk)
{
	return __ccic_clk_common_enable(clk, CCIC_AXI_EN);
}

static void ccic_axi_clk_disable(struct clk *clk)
{
	__ccic_clk_common_disable(clk, CCIC_AXI_DIS);
}

struct clkops ccic_axi_clk_ops = {
	.enable		= ccic_axi_clk_enable,
	.disable	= ccic_axi_clk_disable,
};

static struct clk pxa988_ccic_axi_clk = {
	.name = "ccic_axi",
	.lookup = {
		.con_id = "CCICAXICLK",
	},
	.clk_rst = (void __iomem *)APMU_CCIC_RST,
	.parent = &lcd_ci_isp_axi_clk,
	.ops = &ccic_axi_clk_ops,
};

#define CCIC_PHYSLOW_PRER	(0x1A << 10)
#define CCIC_PHYCLK_SEL		(0x1 << 7)
#define CCIC_PHYCLK_SELDIV	\
	(CCIC_PHYSLOW_PRER | CCIC_PHYCLK_SEL)
#define CCIC_PHYCLK_SELDIV_MSK	((1 << 7) | (0x1f << 10))
#define CCIC_PHY_EN	((1 << 5)|(1 << 8)|(1 << 9))
#define CCIC_PHY_DIS	((1 << 5)|(1 << 9))

static void ccic_phy_clk_init(struct clk *clk)
{
	/* default sel:52M  div : 0x1f */
	CLK_SET_BITS(CCIC_PHYCLK_SELDIV,
		CCIC_PHYCLK_SELDIV_MSK);
}

static int ccic_phy_clk_enable(struct clk *clk)
{
	__ccic_clk_common_enable(clk, CCIC_PHY_EN);
	__raw_writel(0x06000000 | __raw_readl(APMU_CCIC_DBG),
			APMU_CCIC_DBG);
	return 0;
}

static void ccic_phy_clk_disable(struct clk *clk)
{
	__ccic_clk_common_disable(clk, CCIC_PHY_DIS);
	__raw_writel((~0x06000000) & __raw_readl(APMU_CCIC_DBG),
		APMU_CCIC_DBG);
}

struct clkops ccic_phy_clk_ops = {
	.init		= ccic_phy_clk_init,
	.enable		= ccic_phy_clk_enable,
	.disable	= ccic_phy_clk_disable,
};

static struct clk pxa988_ccic_phy_clk = {
	.name = "ccic_phy",
	.lookup = {
		.con_id = "CCICPHYCLK",
	},
	.clk_rst = (void __iomem *)APMU_CCIC_RST,
	.ops = &ccic_phy_clk_ops,
};

#define CI_FUNC_CLK_REQ		(1 << 15)
#define CI_FUNC_CLK_EN		((1 << 1) | (1 << 4))
#define CI_FUNC_CLK_DIS		(1 << 4)

static void ccic_func_clk_init(struct clk *clk)
{
	/* default 312M pll1_624/2 */
	__clk_periph_init(clk, &pll1_624, 2, 0);
}

static int ccic_func_clk_enable(struct clk *clk)
{
	__ccic_clk_common_enable(clk, CI_FUNC_CLK_EN);
	return __ccic_clk_common_enable(clk, \
		CI_FUNC_CLK_REQ);
}

static void ccic_func_clk_disable(struct clk *clk)
{
	__ccic_clk_common_disable(clk, CI_FUNC_CLK_DIS);
}

static int ccic_func_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;

	spin_lock_irqsave(&ccic_lock, flags);
	__clk_periph_set_rate(clk, rate);
	spin_unlock_irqrestore(&ccic_lock, flags);
	return 0;
}

static unsigned long ccic_func_clk_getrate(struct clk *clk)
{
	unsigned long rate, flags;

	spin_lock_irqsave(&ccic_lock, flags);
	rate = __clk_periph_get_rate(clk);
	spin_unlock_irqrestore(&ccic_lock, flags);
	return rate;
}

struct clkops ccic_func_clk_ops = {
	.init		= ccic_func_clk_init,
	.enable		= ccic_func_clk_enable,
	.disable	= ccic_func_clk_disable,
	.setrate	= ccic_func_clk_setrate,
	.getrate	= ccic_func_clk_getrate,
};

static struct clk pxa988_ccic_func_clk = {
	.name = "ccic_func",
	.lookup = {
		.con_id = "CCICFUNCLK",
	},
	.clk_rst = (void __iomem *)APMU_CCIC_RST,
	.inputs = periph_mux_sel,
	.ops = &ccic_func_clk_ops,
	.reg_data = {
		     { {APMU_CCIC_RST, 16, 0x3}, {APMU_CCIC_RST, 16, 0x3} },
		     { {APMU_CCIC_RST, 18, 0x7}, {APMU_CCIC_RST, 18, 0x7} } }
};

#define DSI_PHYSLOW_PRER	(0x1A << 6)
#define DSI_ESC_SEL		(0x0)
#define DSI_PHYESC_SELDIV	\
	(DSI_PHYSLOW_PRER | DSI_ESC_SEL)
#define DSI_PHYESC_SELDIV_MSK	((0x1f << 6) | 0x3)
#define DSI_PHY_CLK_EN	((1 << 2) | (1 << 5))
#define DSI_PHY_CLK_RST	((1 << 3) | (1 << 4))

static void dsi_phy_clk_init(struct clk *clk)
{
	/* default sel 52M, div 0x1A */
	CLK_SET_BITS(DSI_PHYESC_SELDIV,
		DSI_PHYESC_SELDIV_MSK);
}

static int dsi_phy_clk_enable(struct clk *clk)
{
	CLK_SET_BITS(DSI_PHY_CLK_EN, 0);
	CLK_SET_BITS(DSI_PHY_CLK_RST, 0);
	return 0;
}

static void dsi_phy_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, DSI_PHY_CLK_EN);
}

struct clkops dsi_phy_clk_ops = {
	.init = dsi_phy_clk_init,
	.enable = dsi_phy_clk_enable,
	.disable = dsi_phy_clk_disable,
};

static struct clk lcd_dsi_phy_clk = {
	.name = "lcd_dsi_phy",
	.lookup = {
		.con_id = "DSIPHYCLK",
	},
	.clk_rst = (void __iomem *)APMU_DSI,
	.ops = &dsi_phy_clk_ops,
};

#define LCD_CLK_EN		((1 << 4) | (1 << 3))
#define LCD_CLK_RST		(1 << 1)
#define LCD_DEF_FCLK_SEL	(1 << 6)
#define LCD_FCLK_SEL_MASK	(1 << 6)

/* Actually this clock is the src of LCD controller and DSI */
/* Will be further divided in LCD */
static void lcd_func_clk_init(struct clk *clk)
{
	/* 1 --- 416M by default */
	CLK_SET_BITS(LCD_DEF_FCLK_SEL, LCD_FCLK_SEL_MASK);
	clk_reparent(clk, &pll1_416);
	clk->mul = clk->div = 1;
	clk->rate = clk_get_rate(clk->parent);
}

static int lcd_func_clk_enable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS((LCD_CLK_EN | LCD_CLK_RST), 0);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
	return 0;
}

static void lcd_func_clk_disable(struct clk *clk)
{
	unsigned long flags;
	spin_lock_irqsave(&lcd_ci_share_lock, flags);
	CLK_SET_BITS(0, LCD_CLK_EN);
	spin_unlock_irqrestore(&lcd_ci_share_lock, flags);
}

static long lcd_func_clk_round_rate(struct clk *clk, unsigned long rate)
{
	if (rate <= clk_get_rate(&pll1_416))
		return clk_get_rate(&pll1_416);
	else
		return clk_get_rate(&pll1_624);
}

static int lcd_func_clk_setrate(struct clk *clk, unsigned long rate)
{
	unsigned int mux = 0;

	if (rate == clk->rate)
		return 0;

	if (rate <= clk_get_rate(&pll1_416))
		clk_reparent(clk, &pll1_416);
	else
		clk_reparent(clk, &pll1_624);

	mux = __clk_parent_to_mux(clk, clk->parent);
	CLK_SET_BITS(mux << 6, LCD_FCLK_SEL_MASK);
	return 0;
}

static unsigned long lcd_func_clk_getrate(struct clk *clk)
{
	return clk->rate;
}

static struct clk *lcd_depend_clk[] = {
	&lcd_ci_isp_axi_clk,
	&lcd_dsi_phy_clk,
	&pxa988_lcd_ci_hclk,
};

static struct clk_mux_sel lcd_fclk_clk_mux[] = {
	{.input = &pll1_416, .value = 1},
	{.input = &pll1_624, .value = 0},
	{0, 0},
};

struct clkops lcd_fclk_clk_ops = {
	.init = lcd_func_clk_init,
	.enable = lcd_func_clk_enable,
	.disable = lcd_func_clk_disable,
	.round_rate = lcd_func_clk_round_rate,
	.setrate = lcd_func_clk_setrate,
	.getrate = lcd_func_clk_getrate,
};

static struct clk pxa988_lcd_clk = {
	.name = "lcd",
	.lookup = {
		.con_id = "LCDCLK",
	},
	.clk_rst = (void __iomem *)APMU_LCD,
	.dependence = lcd_depend_clk,
	.dependence_count = ARRAY_SIZE(lcd_depend_clk),
	.inputs = lcd_fclk_clk_mux,
	.ops = &lcd_fclk_clk_ops,
};

#define ISP_DXO_CLK_EN		\
	((1 << 1) | (1 << 9) | (1 << 11))
#define ISP_DXO_CLK_RST		\
	((1 << 0) | (1 << 8) | (1 << 10))
#define ISP_DXO_CLK_REQ		(1 << 7)

static void isp_dxo_clk_init(struct clk *clk)
{
	/* default 312M pll1_624/2 */
	__clk_periph_init(clk, &pll1_624, 2, 0);
}

static int isp_dxo_clk_enable(struct clk *clk)
{
	CLK_SET_BITS(ISP_DXO_CLK_EN | ISP_DXO_CLK_RST, 0);
	CLK_SET_BITS(ISP_DXO_CLK_REQ, 0);
	return 0;
}

static void isp_dxo_clk_disable(struct clk *clk)
{
	CLK_SET_BITS(0, ISP_DXO_CLK_EN);
}

static int isp_dxo_clk_setrate(struct clk *clk, unsigned long rate)
{
	__clk_periph_set_rate(clk, rate);
	return 0;
}

static unsigned long isp_dxo_clk_getrate(struct clk *clk)
{
	return __clk_periph_get_rate(clk);
}

struct clkops isp_dxo_clk_ops = {
	.init		= isp_dxo_clk_init,
	.enable		= isp_dxo_clk_enable,
	.disable	= isp_dxo_clk_disable,
	.setrate	= isp_dxo_clk_setrate,
	.getrate	= isp_dxo_clk_getrate,
};

static struct clk *isp_dxo_depend_clk[] = {
	&lcd_ci_isp_axi_clk,
};

static struct clk pxa988_isp_dxo_clk = {
	.name = "isp_dxo",
	.lookup = {
		.con_id = "ISP-CLK",
	},
	.dependence = isp_dxo_depend_clk,
	.dependence_count = ARRAY_SIZE(isp_dxo_depend_clk),
	.clk_rst = (void __iomem *)APMU_ISPDXO,
	.inputs = periph_mux_sel,
	.ops = &isp_dxo_clk_ops,
	.reg_data = {
		     { {APMU_ISPDXO, 2, 0x3}, {APMU_ISPDXO, 2, 0x3} },
		     { {APMU_ISPDXO, 4, 0x7}, {APMU_ISPDXO, 4, 0x7} } }
};

static int nand_clk_enable(struct clk *clk)
{
	__raw_writel(0x19b, clk->clk_rst);
	return 0;
}

static void nand_clk_disable(struct clk *clk)
{
	/* only disable peripheral clock */
	__raw_writel(0x18b, clk->clk_rst);
}

struct clkops nand_clk_ops = {
	.enable = nand_clk_enable,
	.disable = nand_clk_disable,
};

static int pwm_clk_enable(struct clk *clk)
{
	struct clk *clk_apb = NULL, *clk_share = NULL;
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLKSEL(7));
	data |= APBC_FNCLK | APBC_FNCLKSEL(clk->fnclksel);
	__raw_writel(data, clk->clk_rst);
	/*
	 * delay two cycles of the solwest clock between the APB bus clock
	 * and the functional module clock.
	 */
	udelay(10);

	if (!strcmp(clk->name, "pwm0")) {
		clk_share = clk_get_sys("pxa910-pwm.1", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm1")) {
		clk_share = clk_get_sys("pxa910-pwm.0", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	} else if (!strcmp(clk->name, "pwm2")) {
		clk_share = clk_get_sys("pxa910-pwm.3", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm3")) {
		clk_share = clk_get_sys("pxa910-pwm.2", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	}
	if ((clk->refcnt + clk_share->refcnt) == 1) {
		data = __raw_readl(clk_apb->clk_rst);
		data |= APBC_APBCLK;
		__raw_writel(data, clk_apb->clk_rst);
		udelay(10);
		if (!strcmp(clk->name, clk_apb->name)) {
			data = __raw_readl(clk->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk->clk_rst);
		} else {
			data = __raw_readl(clk->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk->clk_rst);
			data = __raw_readl(clk_apb->clk_rst);
			data &= ~APBC_RST;
			__raw_writel(data, clk_apb->clk_rst);
		}
	}

	return 0;
}

static void pwm_clk_disable(struct clk *clk)
{
	struct clk *clk_apb = NULL, *clk_share = NULL;
	unsigned long data;

	data = __raw_readl(clk->clk_rst) & ~(APBC_FNCLK | APBC_FNCLKSEL(7));
	__raw_writel(data, clk->clk_rst);
	udelay(10);

	if (!strcmp(clk->name, "pwm0")) {
		clk_share = clk_get_sys("pxa910-pwm.1", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm1")) {
		clk_share = clk_get_sys("pxa910-pwm.0", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	} else if (!strcmp(clk->name, "pwm2")) {
		clk_share = clk_get_sys("pxa910-pwm.3", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk;
	} else if (!strcmp(clk->name, "pwm3")) {
		clk_share = clk_get_sys("pxa910-pwm.2", NULL);
		BUG_ON(IS_ERR(clk_share));
		clk_apb = clk_share;
	}

	if ((clk->refcnt + clk_share->refcnt) == 0) {
		data = __raw_readl(clk_apb->clk_rst);
		data &= ~APBC_APBCLK;
		__raw_writel(data, clk_apb->clk_rst);
	}
}

struct clkops pwm_clk_ops = {
	.enable = pwm_clk_enable,
	.disable = pwm_clk_disable,
};

#define APBC_CLK(_name, _dev, _con, _reg, _fnclksel, _rate, _parent)\
{							\
	.name = _name,					\
	.lookup = {					\
		.dev_id = _dev,\
		.con_id = _con,\
	},						\
	.clk_rst = (void __iomem *)_reg,		\
	.fnclksel = _fnclksel,				\
	.rate = _rate,					\
	.ops = &apbc_clk_ops,				\
	.parent = _parent,				\
}

#define APBC_CLK_OPS(_name, _dev, _con, _reg, _fnclksel, _rate, _parent, _ops)\
{							\
	.name = _name,					\
	.lookup = {					\
		.dev_id = _dev,\
		.con_id = _con,\
	},						\
	.clk_rst = (void __iomem *)_reg,		\
	.fnclksel = _fnclksel,				\
	.rate = _rate,					\
	.ops = _ops,					\
	.parent = _parent,				\
}

#define APMU_CLK(_name, _dev, _con, _reg, _eval, _rate, _parent)\
{								\
	.name = _name,						\
	.lookup = {						\
		.dev_id = _dev,					\
		.con_id = _con,					\
	},							\
	.clk_rst = (void __iomem *)_reg,			\
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
	.clk_rst = (void __iomem *)_reg,			\
	.enable_val = _eval,					\
	.rate = _rate,						\
	.parent = _parent,					\
	.ops = _ops,						\
}

DEFINE_GATE_CLK(VCTCXO, MPMU_VRCR, 1, NULL, "VCTCXO");

/* all clk src on the board */
static struct clk *pxa988_clks_src[] = {
	&VCTCXO,
	&pll1_416,
	&pll1_624,
	&pll1_1248,
	&pll2_vco,
	&pll2,
	&pll2p,
	&pll3_vco,
	&pll3,
	&pll3p,
};

/* soc peripheral clk on the board */
static struct clk *pxa988_clks_peri[] = {
	&pxa988_clk_sdh0,
	&pxa988_clk_sdh1,
	&pxa988_clk_sdh2,
	&pxa988_clk_gc,
	&pxa988_clk_vpu,
	&lcd_ci_isp_axi_clk,
	&pxa988_lcd_ci_hclk,
	&pxa988_ccic_axi_clk,
	&pxa988_ccic_phy_clk,
	&pxa988_ccic_func_clk,
	&lcd_dsi_phy_clk,
	&pxa988_lcd_clk,
	&pxa988_isp_dxo_clk,
};

/* APB and some simple APMU clock */
static struct clk pxa988_list_clks[] = {
	/* APBC: _name, _dev, _con, _reg, _fnclksel, _rate, _parent*/
	APBC_CLK("uart0", "pxa2xx-uart.0", NULL,
		APBC_PXA988_UART0, 1, 14745600, NULL),/* CP uart */
	APBC_CLK("uart1", "pxa2xx-uart.1", NULL,
		APBC_PXA988_UART1, 1, 14745600, NULL),	/* AP uart0*/
	APBC_CLK("uart2", "pxa2xx-uart.2", NULL,
		APBC_PXA988_UART2, 1, 14745600, NULL),	/* AP uart1*/
	APBC_CLK("twsi0", "pxa910-i2c.0", NULL,
		APBC_PXA988_TWSI0, 0, 33000000, NULL),	/* ci2c */
	APBC_CLK("twsi1", "pxa910-i2c.1", NULL,
		APBC_PXA988_PWRTWSI, 0, 33000000, NULL),/* pwr_i2c */
	APBC_CLK("twsi2", "pxa910-i2c.2", NULL,
		APBC_PXA988_TWSI1, 0, 33000000, NULL),	/* ci2c1 */
	APBC_CLK("ssp0", "pxa910-ssp.0", NULL,
		APBC_PXA988_SSP0, 4, 3250000, NULL),
	APBC_CLK("ssp1", "pxa910-ssp.1", NULL,
		APBC_PXA988_SSP1, 0, 26000000, NULL),
	APBC_CLK("ssp2", "pxa910-ssp.2", NULL,
		APBC_PXA988_SSP2, 2, 26000000, NULL),
	APBC_CLK("keypad", "pxa27x-keypad", NULL,
		APBC_PXA988_KPC, 0, 32000, NULL),
	APBC_CLK("rtc", NULL, "MMP-RTC",
		APBC_PXA988_RTC, 0, 32000, NULL),
	APBC_CLK("1wire", NULL, "PXA-W1",
		APBC_PXA988_ONEWIRE, 0, 26000000, NULL),

	/* APBC_OPS: _name, _dev, _con, _reg, _fnclksel, _rate, _parent*/
	APBC_CLK_OPS("pwm0", "pxa910-pwm.0", NULL,
		APBC_PXA988_PWM0, 0, 13000000, NULL, &pwm_clk_ops),
	APBC_CLK_OPS("pwm1", "pxa910-pwm.1", NULL,
		APBC_PXA988_PWM1, 0, 13000000, NULL, &pwm_clk_ops),
	APBC_CLK_OPS("pwm2", "pxa910-pwm.2", NULL,
		APBC_PXA988_PWM2, 0, 13000000, NULL, &pwm_clk_ops),
	APBC_CLK_OPS("pwm3", "pxa910-pwm.3", NULL,
		APBC_PXA988_PWM3, 0, 13000000, NULL, &pwm_clk_ops),

	/* APMU: _name, _dev, _con, _reg, _eval, _rate, _parent */
	APMU_CLK("udc", NULL, "UDCCLK", APMU_USB,
			0x9, 480000000, NULL),
	APMU_CLK("ire", "pxa910-ire.0", NULL, APMU_IRE,
			0x9, 480000000, NULL),

	/* APMU: _name, _dev, _con, _reg, _eval, _rate, _parent , ops */
	APMU_CLK_OPS("nand", "pxa3xx-nand", NULL, APMU_NAND,
			0x19b, 156000000, NULL, &nand_clk_ops),
};

#define CIU_BASE (AXI_VIRT_BASE + 0x82c00)
#define MC_CONF	(CIU_BASE + 0x40)

static void __init clk_misc_init(void)
{
	/* pll2 default rate is different when using LPDDR400 and LPDDR533 */
	if (1) {
		pll2_vco_default = 1600 * MHZ;
		pll2_default = 800 * MHZ;
		pll2p_default = 533 * MHZ;
	} else {
		pll2_vco_default = 2132 * MHZ;
		pll2_default = 1066 * MHZ;
		pll2p_default = 533 * MHZ;
	}

	/* select i2s clock from VCTCXO , LP audio playback support */
	__raw_writel(__raw_readl(MPMU_FCCR) | (1 << 28), MPMU_FCCR);

	/* components' clock should always keep enabled */
	__raw_writel(0x3, APBC_PXA988_IPC);	/* ACIPC */
	__raw_writel(0x0, APBC_PXA988_RIPC);	/* RIPC */
	__raw_writel(0x3, APMU_MCK4_CTL);	/* MCK4 AHB */

	/* disable SOC and MC4 dynamic clk gating on Z0 */
	__raw_writel(0x00080008, MC_CONF);

}

void pxa988_init_one_clock(struct clk *c)
{
	clk_init(c);
	INIT_LIST_HEAD(&c->shared_bus_list);
	if (!c->lookup.dev_id && !c->lookup.con_id)
		c->lookup.con_id = c->name;
	c->lookup.clk = c;
	clkdev_add(&c->lookup);
}
EXPORT_SYMBOL(pxa988_init_one_clock);

static int __init pxa988_clk_init(void)
{
	int i;

	clk_misc_init();

	for (i = 0; i < ARRAY_SIZE(pxa988_clks_src); i++)
		pxa988_init_one_clock(pxa988_clks_src[i]);
	for (i = 0; i < ARRAY_SIZE(pxa988_clks_peri); i++)
		pxa988_init_one_clock(pxa988_clks_peri[i]);
	for (i = 0; i < ARRAY_SIZE(pxa988_list_clks); i++)
		pxa988_init_one_clock(&pxa988_list_clks[i]);

	return 0;
}
core_initcall(pxa988_clk_init);
