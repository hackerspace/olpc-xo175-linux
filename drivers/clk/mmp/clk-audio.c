// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MMP Audio Clock Controller driver
 *
 * Copyright (C) 2020 Lubomir Rintel <lkundrak@v3.sk>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <dt-bindings/clock/marvell,mmp2-audio.h>

enum {
	SSPA_AUD_CTRL		= 0x04,
	SSPA_AUD_PLL_CTRL0	= 0x08,
	SSPA_AUD_PLL_CTRL1	= 0x0c,
};

/* SSPA Audio Control Register */
#define SSPA_AUD_CTRL_SYSCLK_SHIFT		0
#define SSPA_AUD_CTRL_SSPA0_MUX_SHIFT		7
#define SSPA_AUD_CTRL_SSPA0_SHIFT		8
#define SSPA_AUD_CTRL_SSPA1_SHIFT		16
#define SSPA_AUD_CTRL_SSPA1_MUX_SHIFT		23
#define SSPA_AUD_CTRL_DIV_MASK			0x7e

/* SSPA Audio PLL Control 0 Register */
#define SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO_MASK (0x7 << 28)
#define SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO(x)	((x) << 28)
#define SSPA_AUD_PLL_CTRL0_FRACT_MASK		(0xfffff << 8)
#define SSPA_AUD_PLL_CTRL0_FRACT(x)		((x) << 8)
#define SSPA_AUD_PLL_CTRL0_ENA_DITHER		(1 << 7)
#define SSPA_AUD_PLL_CTRL0_ICP_2UA		(0 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_5UA		(1 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_7UA		(2 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_10UA		(3 << 5)
#define SSPA_AUD_PLL_CTRL0_DIV_FBCCLK_MASK	(0x3 << 3)
#define SSPA_AUD_PLL_CTRL0_DIV_FBCCLK(x)	((x) << 3)
#define SSPA_AUD_PLL_CTRL0_DIV_MCLK_MASK	(0x1 << 2)
#define SSPA_AUD_PLL_CTRL0_DIV_MCLK(x)		((x) << 2)
#define SSPA_AUD_PLL_CTRL0_PD_OVPROT_DIS	(1 << 1)
#define SSPA_AUD_PLL_CTRL0_PU			(1 << 0)

/* SSPA Audio PLL Control 1 Register */
#define SSPA_AUD_PLL_CTRL1_SEL_FAST_CLK		(1 << 24)
#define SSPA_AUD_PLL_CTRL1_CLK_SEL_MASK		(1 << 11)
#define SSPA_AUD_PLL_CTRL1_CLK_SEL_AUDIO_PLL	(1 << 11)
#define SSPA_AUD_PLL_CTRL1_CLK_SEL_VCXO		(0 << 11)
#define SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN_MASK (0x7ff << 0)
#define SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN(x)	((x) << 0)

struct mmp_audio_mux {
	struct clk_hw hw;
	struct mmp2_audio_clk *priv;
	u8 shift;
	u8 flags;
	int index;
};

#define to_mmp_audio_mux(_hw) container_of(_hw, struct mmp_audio_mux, hw)

struct mmp_audio_div {
	struct clk_hw hw;
	struct mmp2_audio_clk *priv;
	u8 shift;
	int value;
};

#define to_mmp_audio_div(_hw) container_of(_hw, struct mmp_audio_div, hw)

struct mmp_audio_pll {
	struct clk_hw hw;
	struct mmp2_audio_clk *priv;
	u32 ctrl0;
	u32 ctrl1;
};

#define to_mmp_audio_pll(_hw) container_of(_hw, struct mmp_audio_pll, hw)

struct mmp2_audio_clk {
	void __iomem *mmio_base;

	struct clk *audio_clk;
	struct clk *vctcxo_clk;
	struct clk *i2s0_clk;
	struct clk *i2s1_clk;

	struct mmp_audio_pll audio_pll;
	struct mmp_audio_mux sspa_mux;
	struct mmp_audio_mux sspa1_mux;
	struct mmp_audio_div sysclk;
	struct mmp_audio_div sspa0;
	struct mmp_audio_div sspa1;

	struct clk *clk_table[3];
	struct clk_onecell_data clk_data;

	spinlock_t lock;
};

static struct {
	unsigned long parent_rate;
	unsigned long freq_vco;
	unsigned char mclk;
	unsigned char fbcclk;
	unsigned short fract;
} predivs[] = {
	{ 26000000, 135475200, 0, 0, 0x8a18 },
	{ 26000000, 147456000, 0, 1, 0x0da1 },
	{ 38400000, 135475200, 1, 2, 0x8208 },
	{ 38400000, 147456000, 1, 3, 0xaaaa },
};

static struct {
	unsigned char divisor;
	unsigned char modulo;
	unsigned char pattern;
} postdivs[] = {
	{   1,	3,  0, },
	{   2,	5,  0, },
	{   4,	0,  0, },
	{   6,	1,  1, },
	{   8,	1,  0, },
	{   9,	1,  2, },
	{  12,	2,  1, },
	{  16,	2,  0, },
	{  18,	2,  2, },
	{  24,	4,  1, },
	{  36,	4,  2, },
	{  48,	6,  1, },
	{  72,	6,  2, },
};

static unsigned long mmp_audio_pll_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct mmp_audio_pll *pll = to_mmp_audio_pll(hw);
	unsigned int prediv;
	unsigned int postdiv;

	for (prediv = 0; prediv < ARRAY_SIZE(predivs); prediv++) {
		if (predivs[prediv].parent_rate != parent_rate)
			continue;
		for (postdiv = 0; postdiv < ARRAY_SIZE(postdivs); postdiv++) {
			unsigned long freq;
			u32 val;

			val = SSPA_AUD_PLL_CTRL0_ENA_DITHER;
			val |= SSPA_AUD_PLL_CTRL0_PU;
			val |= SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO(postdivs[postdiv].modulo);
			val |= SSPA_AUD_PLL_CTRL0_FRACT(predivs[prediv].fract);
			val |= SSPA_AUD_PLL_CTRL0_DIV_FBCCLK(predivs[prediv].fbcclk);
			val |= SSPA_AUD_PLL_CTRL0_DIV_MCLK(predivs[prediv].mclk);
			if (val != pll->ctrl0)
				continue;

			val = SSPA_AUD_PLL_CTRL1_CLK_SEL_AUDIO_PLL;
			val |= SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN(postdivs[postdiv].pattern);
			if (val != pll->ctrl1)
				continue;

			freq = predivs[prediv].freq_vco;
			freq /= postdivs[postdiv].divisor;
			return freq;
		}
	}

	return 0;
}

static long mmp_audio_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	unsigned int prediv;
	unsigned int postdiv;
	long rounded = 0;

	for (prediv = 0; prediv < ARRAY_SIZE(predivs); prediv++) {
		if (predivs[prediv].parent_rate != *parent_rate)
			continue;
		for (postdiv = 0; postdiv < ARRAY_SIZE(postdivs); postdiv++) {
			long freq = predivs[prediv].freq_vco;

			freq /= postdivs[postdiv].divisor;
			if (freq == rate)
				return rate;
			if (freq < rate)
				continue;
			if (rounded && freq > rounded)
				continue;
			rounded = freq;
		}
	}

	return rounded;
}

static void mmp_audio_pll_write_rate(struct mmp_audio_pll *pll)
{
	struct mmp2_audio_clk *priv = pll->priv;

	__raw_writel(pll->ctrl0, priv->mmio_base + SSPA_AUD_PLL_CTRL0);
	__raw_writel(pll->ctrl1, priv->mmio_base + SSPA_AUD_PLL_CTRL1);
}

static int mmp_audio_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct mmp_audio_pll *pll = to_mmp_audio_pll(hw);
	struct mmp2_audio_clk *priv = pll->priv;
	unsigned int prediv;
	unsigned int postdiv;

	for (prediv = 0; prediv < ARRAY_SIZE(predivs); prediv++) {
		if (predivs[prediv].parent_rate != parent_rate)
			continue;

		for (postdiv = 0; postdiv < ARRAY_SIZE(postdivs); postdiv++) {
			if (rate * postdivs[postdiv].divisor != predivs[prediv].freq_vco)
				continue;

			pll->ctrl0 = SSPA_AUD_PLL_CTRL0_ENA_DITHER;
			pll->ctrl0 |= SSPA_AUD_PLL_CTRL0_PU;
			pll->ctrl0 |= SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO(postdivs[postdiv].modulo);
			pll->ctrl0 |= SSPA_AUD_PLL_CTRL0_FRACT(predivs[prediv].fract);
			pll->ctrl0 |= SSPA_AUD_PLL_CTRL0_DIV_FBCCLK(predivs[prediv].fbcclk);
			pll->ctrl0 |= SSPA_AUD_PLL_CTRL0_DIV_MCLK(predivs[prediv].mclk);

			pll->ctrl1 = SSPA_AUD_PLL_CTRL1_CLK_SEL_AUDIO_PLL;
			pll->ctrl1 |= SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN(postdivs[postdiv].pattern);

			if (__clk_is_enabled(priv->audio_clk))
				mmp_audio_pll_write_rate(pll);

			return 0;
		}
	}

	return -ERANGE;
}

static int mmp_audio_pll_enable(struct clk_hw *hw)
{
	struct mmp_audio_pll *pll = to_mmp_audio_pll(hw);
	struct mmp2_audio_clk *priv = pll->priv;

	clk_prepare_enable(priv->audio_clk);
	mmp_audio_pll_write_rate(pll);
	return 0;
}

static void mmp_audio_pll_disable(struct clk_hw *hw)
{
	struct mmp_audio_pll *pll = to_mmp_audio_pll(hw);
	struct mmp2_audio_clk *priv = pll->priv;

	clk_disable_unprepare(priv->audio_clk);
}

const struct clk_ops mmp_audio_pll_ops = {
	.enable = mmp_audio_pll_enable,
	.disable = mmp_audio_pll_disable,
	.recalc_rate = mmp_audio_pll_recalc_rate,
	.round_rate = mmp_audio_pll_round_rate,
	.set_rate = mmp_audio_pll_set_rate,
};

static u8 mmp_audio_mux_get_parent(struct clk_hw *hw)
{
	struct mmp_audio_mux *mux = to_mmp_audio_mux(hw);
	struct mmp2_audio_clk *priv = mux->priv;
	u32 val;

	if (__clk_is_enabled(priv->audio_clk)) {
		val = __raw_readl(priv->mmio_base + SSPA_AUD_CTRL);
		val >>= mux->shift;
		val &= 1;
	} else {
		val = 0;
	}
	mux->index = val;

	return mux->index;
}

static void mmp_audio_mux_write_parent(struct mmp_audio_mux *mux)
{
	struct mmp2_audio_clk *priv = mux->priv;
	unsigned long flags = 0;
	u32 reg;

	spin_lock_irqsave(&priv->lock, flags);
	reg = __raw_readl(priv->mmio_base + SSPA_AUD_CTRL);
	reg &= ~(1 << mux->shift);
	reg |= mux->index << mux->shift;
	__raw_writel(reg, priv->mmio_base + SSPA_AUD_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static int mmp_audio_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct mmp_audio_mux *mux = to_mmp_audio_mux(hw);
	struct mmp2_audio_clk *priv = mux->priv;

	mux->index = index;
	if (__clk_is_enabled(priv->audio_clk))
		mmp_audio_mux_write_parent(mux);

	return 0;
}

static int mmp_audio_mux_determine_rate(struct clk_hw *hw,
					struct clk_rate_request *req)
{
	return clk_mux_determine_rate_flags(hw, req, 0);
}

static int mmp_audio_mux_enable(struct clk_hw *hw)
{
	struct mmp_audio_mux *mux = to_mmp_audio_mux(hw);
	struct mmp2_audio_clk *priv = mux->priv;

	clk_prepare_enable(priv->audio_clk);
	mmp_audio_mux_write_parent(mux);
	return 0;
}

static void mmp_audio_mux_disable(struct clk_hw *hw)
{
	struct mmp_audio_mux *mux = to_mmp_audio_mux(hw);
	struct mmp2_audio_clk *priv = mux->priv;

	clk_disable_unprepare(priv->audio_clk);
}

const struct clk_ops mmp_audio_mux_ops = {
	.enable = mmp_audio_mux_enable,
	.disable = mmp_audio_mux_disable,

	.get_parent = mmp_audio_mux_get_parent,
	.set_parent = mmp_audio_mux_set_parent,
	.determine_rate = mmp_audio_mux_determine_rate,
};

static unsigned long mmp_audio_div_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct mmp_audio_div *div = to_mmp_audio_div(hw);

	if (div->value == 0)
		return 0;

	return divider_recalc_rate(hw, parent_rate, div->value, NULL,
				   CLK_DIVIDER_ONE_BASED |
				   CLK_DIVIDER_ROUND_CLOSEST |
				   CLK_DIVIDER_ALLOW_ZERO, 6);
}

static long mmp_audio_div_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *prate)
{
	return divider_round_rate(hw, rate, prate, NULL, 6,
				  CLK_DIVIDER_ONE_BASED |
				  CLK_DIVIDER_ROUND_CLOSEST |
				  CLK_DIVIDER_ALLOW_ZERO);
}

static void mmp_audio_div_write_rate(struct mmp_audio_div *div, int enable)
{
	struct mmp2_audio_clk *priv = div->priv;
	u32 val;

	clk_prepare_enable(priv->audio_clk);
	val = __raw_readl(priv->mmio_base + SSPA_AUD_CTRL);
	val &= ~(SSPA_AUD_CTRL_DIV_MASK << div->shift);
	val |= (u32)div->value << (div->shift + 1);
	val |= enable << div->shift;
	__raw_writel(val, priv->mmio_base + SSPA_AUD_CTRL);
	clk_disable_unprepare(priv->audio_clk);
}

static int mmp_audio_div_enable(struct clk_hw *hw)
{
	struct mmp_audio_div *div = to_mmp_audio_div(hw);
	struct mmp2_audio_clk *priv = div->priv;

	clk_prepare_enable(priv->audio_clk);
	mmp_audio_div_write_rate(div, 1);
	return 0;
}

static void mmp_audio_div_disable(struct clk_hw *hw)
{
	struct mmp_audio_div *div = to_mmp_audio_div(hw);
	struct mmp2_audio_clk *priv = div->priv;
	unsigned long flags = 0;
	u32 val;

	spin_lock_irqsave(&priv->lock, flags);
	val = __raw_readl(priv->mmio_base + SSPA_AUD_CTRL);
	val &= ~(1 << div->shift);
	__raw_writel(val, priv->mmio_base + SSPA_AUD_CTRL);
	spin_unlock_irqrestore(&priv->lock, flags);
	clk_disable_unprepare(priv->audio_clk);
}

static int mmp_audio_div_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent_rate)
{
	struct mmp_audio_div *div = to_mmp_audio_div(hw);
	struct mmp2_audio_clk *priv = div->priv;
	int value;

	if (rate == 0) {
		value = 0;
	} else {
		value = divider_get_val(rate, parent_rate, NULL, 6,
					CLK_DIVIDER_ONE_BASED |
					CLK_DIVIDER_ROUND_CLOSEST |
					CLK_DIVIDER_ALLOW_ZERO);
		if (value < 0)
			return value;
	}

	div->value = value;
	if (__clk_is_enabled(priv->audio_clk))
		mmp_audio_div_write_rate(div, 0);

	return 0;
}

const struct clk_ops mmp_audio_div_ops = {
	.enable = mmp_audio_div_enable,
	.disable = mmp_audio_div_disable,

	.recalc_rate = mmp_audio_div_recalc_rate,
	.round_rate = mmp_audio_div_round_rate,
	.set_rate = mmp_audio_div_set_rate,
};

static int mmp2_audio_clk_probe(struct platform_device *pdev)
{
	const struct clk_hw *sspa_mux_parents[2];
	const struct clk_hw *sspa1_mux_parents[2];
	struct mmp2_audio_clk *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);

	priv->mmio_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->mmio_base))
		return PTR_ERR(priv->mmio_base);

	priv->audio_clk = devm_clk_get(&pdev->dev, "audio");
	if (IS_ERR(priv->audio_clk))
		return PTR_ERR(priv->audio_clk);

	priv->i2s0_clk = devm_clk_get(&pdev->dev, "i2s0");
	if (IS_ERR(priv->i2s0_clk))
		return PTR_ERR(priv->i2s0_clk);

	priv->i2s1_clk = devm_clk_get(&pdev->dev, "i2s1");
	if (IS_ERR(priv->i2s1_clk))
		return PTR_ERR(priv->i2s1_clk);

	priv->vctcxo_clk = devm_clk_get(&pdev->dev, "vctcxo");
	if (IS_ERR(priv->vctcxo_clk))
		return PTR_ERR(priv->vctcxo_clk);

	priv->audio_pll.hw.init =
		CLK_HW_INIT_HW("audio_pll", __clk_get_hw(priv->vctcxo_clk),
			       &mmp_audio_pll_ops, CLK_SET_RATE_PARENT);
	priv->audio_pll.priv = priv;
	ret = devm_clk_hw_register(&pdev->dev, &priv->audio_pll.hw);
	if (ret)
		return ret;

	sspa_mux_parents[0] = &priv->audio_pll.hw;
	sspa_mux_parents[1] = __clk_get_hw(priv->i2s0_clk);
	priv->sspa_mux.hw.init =
		CLK_HW_INIT_PARENTS_HW("sspa_mux", sspa_mux_parents,
				       &mmp_audio_mux_ops, CLK_SET_RATE_PARENT);
	priv->sspa_mux.priv = priv;
	priv->sspa_mux.shift = SSPA_AUD_CTRL_SSPA0_MUX_SHIFT;
	ret = devm_clk_hw_register(&pdev->dev, &priv->sspa_mux.hw);
	if (ret)
		return ret;

	priv->sysclk.hw.init =
		CLK_HW_INIT_HW("sysclk", &priv->sspa_mux.hw,
			       &mmp_audio_div_ops, CLK_SET_RATE_PARENT);
	priv->sysclk.priv = priv;
	priv->sysclk.shift = SSPA_AUD_CTRL_SYSCLK_SHIFT;
	ret = devm_clk_hw_register(&pdev->dev, &priv->sysclk.hw);
	if (ret)
		return ret;

	priv->sspa0.hw.init =
		CLK_HW_INIT_HW("sspa0", &priv->sspa_mux.hw,
			       &mmp_audio_div_ops, 0);
	priv->sspa0.priv = priv;
	priv->sspa0.shift = SSPA_AUD_CTRL_SSPA0_SHIFT;
	ret = devm_clk_hw_register(&pdev->dev, &priv->sspa0.hw);
	if (ret)
		return ret;

	sspa1_mux_parents[0] = &priv->audio_pll.hw;
	sspa1_mux_parents[1] = __clk_get_hw(priv->i2s1_clk);
	priv->sspa1_mux.hw.init =
		CLK_HW_INIT_PARENTS_HW("sspa1_mux", sspa1_mux_parents,
				       &mmp_audio_mux_ops, CLK_SET_RATE_PARENT);
	priv->sspa1_mux.priv = priv;
	priv->sspa1_mux.shift = SSPA_AUD_CTRL_SSPA1_MUX_SHIFT;
	ret = devm_clk_hw_register(&pdev->dev, &priv->sspa1_mux.hw);
	if (ret)
		return ret;

	priv->sspa1.hw.init =
		CLK_HW_INIT_HW("sspa1", &priv->sspa1_mux.hw,
			       &mmp_audio_div_ops, 0);
	priv->sspa1.priv = priv;
	priv->sspa1.shift = SSPA_AUD_CTRL_SSPA1_SHIFT;
	ret = devm_clk_hw_register(&pdev->dev, &priv->sspa1.hw);
	if (ret)
		return ret;

	priv->clk_table[MMP2_CLK_AUDIO_SYSCLK] = priv->sysclk.hw.clk;
	priv->clk_table[MMP2_CLK_AUDIO_SSPA0] = priv->sspa0.hw.clk;
	priv->clk_table[MMP2_CLK_AUDIO_SSPA1] = priv->sspa1.hw.clk;
	priv->clk_data.clks = priv->clk_table;
	priv->clk_data.clk_num = ARRAY_SIZE(priv->clk_table);

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get,
				   &priv->clk_data);
}

static const struct of_device_id mmp2_audio_clk_of_match[] = {
	{ .compatible = "marvell,mmp2-audio-clock" },
	{},
};

MODULE_DEVICE_TABLE(of, mmp2_audio_clk_of_match);

static struct platform_driver mmp2_audio_clk_driver = {
	.driver = {
		.name = "mmp2-audio-clock",
		.of_match_table = of_match_ptr(mmp2_audio_clk_of_match),
	},
	.probe = mmp2_audio_clk_probe,
};

module_platform_driver(mmp2_audio_clk_driver);

MODULE_AUTHOR("Lubomir Rintel <lkundrak@v3.sk>");
MODULE_DESCRIPTION("Clock driver for MMP2 Audio subsystem");
MODULE_LICENSE("GPL");
