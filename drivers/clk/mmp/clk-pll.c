// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MMP PLL clock rate calculation
 *
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/clk-provider.h>
#include <linux/slab.h>
#include <linux/io.h>

#include "clk.h"

#define to_clk_mmp_pll(hw)	container_of(hw, struct mmp_clk_pll, hw)

struct mmp_clk_pll {
	struct clk_hw hw;
	unsigned long default_rate;
	void __iomem *enable_reg;
	u32 enable;
	void __iomem *reg;
	u8 shift;
};

static unsigned long mmp_clk_pll_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct mmp_clk_pll *pll = to_clk_mmp_pll(hw);
	u64 rate;
	u32 val;
	u32 fbdiv, refdiv;

	val = readl_relaxed(pll->enable_reg);
	if ((val & pll->enable) != pll->enable)
		return pll->default_rate;

	val = readl_relaxed(pll->reg);
	fbdiv = (val >> pll->shift) & 0x1ff;
	refdiv = (val >> (pll->shift + 9)) & 0x1f;

	if (refdiv == 3) {
		rate = 19200000;
	} else if (refdiv == 4) {
		rate = 26000000;
	} else {
		pr_err("bad refdiv: %d (0x%08x)\n", refdiv, val);
		return 0;
	}

	rate *= fbdiv + 2;
	do_div(rate, refdiv + 2);
	return (unsigned long)rate;
}

static const struct clk_ops mmp_clk_pll_ops = {
	.recalc_rate = mmp_clk_pll_recalc_rate,
};

struct clk *mmp_clk_register_pll(char *name,
			unsigned long default_rate,
			void __iomem *enable_reg, u32 enable,
			void __iomem *reg, u8 shift)
{
	struct mmp_clk_pll *pll;
	struct clk *clk;
	struct clk_init_data init;

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.ops = &mmp_clk_pll_ops;
	init.flags = 0;
	init.parent_names = NULL;
	init.num_parents = 0;

	pll->default_rate = default_rate;
	pll->enable_reg = enable_reg;
	pll->enable = enable;
	pll->reg = reg;
	pll->shift = shift;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);

	if (IS_ERR(clk))
		kfree(pll);

	return clk;
}
