// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MMP PMU power island support
 *
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "clk.h"

#define to_mmp_pm_domain(genpd) container_of(genpd, struct mmp_pm_domain, genpd)

struct mmp_pm_domain {
	struct generic_pm_domain genpd;
	void __iomem *reg;
	spinlock_t *lock;
	u32 power_on;
	u32 reset;
	u32 clock_enable;
	unsigned int flags;
};




#include <asm/delay.h>
#define apmu_is_mmp3 1
static __iomem void *apmu_base;




#define GC2D_CLK_DIV(n)         ((n & 0xF) << 28)
#define GC2D_CLK_DIV_MSK        GC2D_CLK_DIV(0xF)
#define GC2D_CLK_SRC_SEL(n)     ((n & 3) << 12)
#define GC2D_CLK_SRC_SEL_MSK    GC2D_CLK_SRC_SEL(3)
#define GC2D_AXICLK_EN          (1u << 19)
#define GC2D_AXI_RST_N          (1u << 18)

#define GC3D_CLK_DIV(n)         ((n & 0xF) << 24)
#define GC3D_CLK_DIV_MSK        GC3D_CLK_DIV(0xF)
#define GC3D_CLK_SRC_SEL(n)     ((n & 3) << 6)
#define GC3D_CLK_SRC_SEL_MSK    GC3D_CLK_SRC_SEL(3)
#define GC3D_ACLK_SEL(n)        ((n & 3) << 4)
#define GC3D_ACLK_SEL_MSK       GC3D_ACLK_SEL(3)
#define GC3D_AXICLK_EN          (1u << 2)
#define GC3D_AXI_RST_N          (1u << 0)

#define GC2D3D_CLK_EN           (1u << 3)
#define GC2D_CLK_EN             (1u << 20)
#define GC2D3D_RST_N            (1u << 1)

#define GC_PWRUP(n)             ((n & 3) << 9)
#define GC_PWRUP_MSK            GC_PWRUP(3)
#define GC_ISB                  (1u << 8)

#define GC_CLK_RATE(div2d, div3d, src, aclk)                             \
        (GC2D_CLK_DIV(div2d) | GC2D_CLK_SRC_SEL(src)              \
        | GC3D_CLK_DIV(div3d) | GC3D_CLK_SRC_SEL(src)             \
        | GC3D_ACLK_SEL(aclk))

#define GC_CLK_RATE_MSK                                         \
        (GC2D_CLK_DIV_MSK | GC2D_CLK_SRC_SEL_MSK                \
        | GC3D_CLK_DIV_MSK | GC3D_CLK_SRC_SEL_MSK               \
        | GC3D_ACLK_SEL_MSK)

#define GC_SET_BITS(set, clear) {\
        unsigned long tmp;\
        tmp = __raw_readl(apmu_base + 0xcc);\
        tmp &= ~(clear);\
        tmp |= set;\
        __raw_writel(tmp, apmu_base + 0xcc);\
}

static void mmp3_gc_island_on(void)
{
	void __iomem *island_ctl = apmu_base + 0x1f4;
	void __iomem *clk_reg = apmu_base + 0x0cc;
	unsigned long gc_rate_cfg;

        unsigned long regval;
        regval = __raw_readl(clk_reg);

	if (regval & (GC_PWRUP_MSK | GC_ISB)) {
		printk("ALREADY UP 0x%08lx", regval);
		goto enable_clock; /*Pwr is already on*/
	}

        /* 0, set to boot default value, source on PLL1*/
        writel(GC_CLK_RATE(3, 3, 0, 0), clk_reg);

        /* 1. Turn on power switches */
        regval = __raw_readl(clk_reg);
	regval = readl(clk_reg);
	regval &= ~GC_PWRUP_MSK;
	regval |= GC_PWRUP(1);
	writel(regval, clk_reg);
	udelay(10);
	regval |= GC_PWRUP(3);
	writel(regval, clk_reg);
	udelay(10);

	/* 2. disable isolation*/
	regval = readl(clk_reg);
	regval |= GC_ISB;
	writel(regval, clk_reg);

	/* 3. enable SRAM dummy clock*/
	regval = readl(island_ctl);
	regval |= (1u << 4);
	writel(regval, island_ctl);
	udelay(250);
	regval = readl(island_ctl);
	regval &= ~(1u << 4);
	writel(regval, island_ctl);

	/* 4. enable AXI clocks*/
	regval = readl(clk_reg);
	regval |= (GC3D_AXICLK_EN | GC2D_AXICLK_EN);
	writel(regval, clk_reg);

	/* 5. change source if necessary*/

	/* 6. enable GC clock */
	regval = readl(clk_reg);
	regval |= GC2D3D_CLK_EN | GC2D_CLK_EN;
	writel(regval, clk_reg);

	/* 7. deassert resets*/
	/* GC300 requires AXI reset first, not align with B0 datasheet*/
	regval = readl(clk_reg);
	regval |= (1u << 1);
	writel(regval, clk_reg);
	udelay(100);
	regval = readl(clk_reg);
	regval |= ((1u << 18) | (1u << 0));
	writel(regval, clk_reg);

	/* 8 gate clock */
	regval = readl(clk_reg);
	regval &= ~(GC2D_AXICLK_EN | GC3D_AXICLK_EN | GC2D3D_CLK_EN
			| GC2D_CLK_EN);
	writel(regval, clk_reg);

enable_clock:
        gc_rate_cfg = GC_CLK_RATE(2, 8, 0, 0); //400Mhz
        gc_rate_cfg &= GC_CLK_RATE_MSK;

        GC_SET_BITS(gc_rate_cfg, GC_CLK_RATE_MSK);
        GC_SET_BITS(GC2D_CLK_EN | GC2D3D_CLK_EN | GC2D_AXICLK_EN\
                        | GC3D_AXICLK_EN, 0);

	printk("TURNED ON: 0x%08x\n", __raw_readl(apmu_base + 0xcc));
}

static void mmp3_gc_island_off(void)
{
	void __iomem *clk_reg = apmu_base + 0x0cc;

        unsigned long regval;
        regval = __raw_readl(clk_reg);

	if ((regval & (GC_PWRUP_MSK | GC_ISB)) == 0)
		goto disable_clock; /*Pwr is already off*/

	/* 1. isolation */
	regval = readl(clk_reg);
	regval &= ~GC_ISB;
	writel(regval, clk_reg);

	/* 2. reset*/
	regval = readl(clk_reg);
	regval &= ~(GC2D_AXI_RST_N | GC3D_AXI_RST_N | GC2D3D_RST_N);
	writel(regval, clk_reg);

	/* 3. make sure clock disabled*/
	regval = readl(clk_reg);
	regval &= ~(GC2D_AXICLK_EN | GC3D_AXICLK_EN | GC2D3D_CLK_EN | GC2D_CLK_EN);
	writel(regval, clk_reg);

	/* 4. turn off power */
	regval = readl(clk_reg);
	regval &= ~GC_PWRUP_MSK;
	writel(regval, clk_reg);

disable_clock:
        GC_SET_BITS(0, GC2D_AXICLK_EN | GC3D_AXICLK_EN | GC2D3D_CLK_EN\
                        | GC2D_CLK_EN);
}

#if 0
static void clk_mmp_enable_gc(void __iomem *clk_reg)
{
	if (apmu_is_mmp3) {
		mmp3_gc_island_on();
	}
}

static void clk_mmp_disable_gc(void __iomem *clk_reg)
{
	if (apmu_is_mmp3)
		mmp3_gc_island_off();
}
#endif







static int mmp_pm_domain_power_on(struct generic_pm_domain *genpd)
{
	struct mmp_pm_domain *pm_domain = to_mmp_pm_domain(genpd);
	unsigned long flags = 0;
	u32 val;

	if (strcmp(genpd->name, "gpu") == 0) {
		apmu_base = (__iomem void *)((u32)pm_domain->reg & 0xffffff00);
		//static __iomem void *apmu_base = (__iomem void *)0xd4282800;
		printk ("XXX POWERING ON {%s} 0x%08x\n", genpd->name, (u32)apmu_base);
		//mmp3_gc_island_on();
		//return 0;
	}

	if (pm_domain->lock)
		spin_lock_irqsave(pm_domain->lock, flags);

	val = readl(pm_domain->reg);

	/* Turn on the power island */
	val |= pm_domain->power_on;
	writel(val, pm_domain->reg);
	printk("W0 0x%08x\n", val);

	/* Disable isolation */
	val |= 0x100;
	writel(val, pm_domain->reg);
	printk("W1 0x%08x\n", val);

	/* Some blocks need to be reset after a power up */
	if (pm_domain->reset || pm_domain->clock_enable) {
		u32 after_power_on = val;

		val &= ~pm_domain->reset;
		writel(val, pm_domain->reg);
		printk("W2 0x%08x\n", val);

		val |= pm_domain->clock_enable;
		writel(val, pm_domain->reg);
		printk("W3 0x%08x\n", val);

		val |= pm_domain->reset;
		writel(val, pm_domain->reg);
		printk("W4 0x%08x\n", val);

		writel(after_power_on, pm_domain->reg);
		printk("W5 0x%08x\n", after_power_on);
	}

	if (pm_domain->lock)
		spin_unlock_irqrestore(pm_domain->lock, flags);

	return 0;
}

static int mmp_pm_domain_power_off(struct generic_pm_domain *genpd)
{
	struct mmp_pm_domain *pm_domain = to_mmp_pm_domain(genpd);
	unsigned long flags = 0;
	u32 val;

	printk ("XXX NOT POWERING OFF {%s}\n", genpd->name);
	return 0;

	if (pm_domain->flags & MMP_PM_DOMAIN_NO_DISABLE)
		return 0;

	if (pm_domain->lock)
		spin_lock_irqsave(pm_domain->lock, flags);

	/* Turn off and isolate the the power island. */
	val = readl(pm_domain->reg);
	val &= ~pm_domain->power_on;
	val &= ~0x100;
	writel(val, pm_domain->reg);

	if (pm_domain->lock)
		spin_unlock_irqrestore(pm_domain->lock, flags);

	return 0;
}

struct generic_pm_domain *mmp_pm_domain_register(const char *name,
		void __iomem *reg,
		u32 power_on, u32 reset, u32 clock_enable,
		unsigned int flags, spinlock_t *lock)
{
	struct mmp_pm_domain *pm_domain;

	pm_domain = kzalloc(sizeof(*pm_domain), GFP_KERNEL);
	if (!pm_domain)
		return ERR_PTR(-ENOMEM);

	pm_domain->reg = reg;
	pm_domain->power_on = power_on;
	pm_domain->reset = reset;
	pm_domain->clock_enable = clock_enable;
	pm_domain->flags = flags;
	pm_domain->lock = lock;

	pm_genpd_init(&pm_domain->genpd, NULL, true);
	pm_domain->genpd.name = name;
	pm_domain->genpd.power_on = mmp_pm_domain_power_on;
	pm_domain->genpd.power_off = mmp_pm_domain_power_off;

	return &pm_domain->genpd;
}
