// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Russell King
 * Copyright (C) 2018,2019,2020,2021 Lubomir Rintel
 *  Largely based on Armada 510 support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Support for Armada MMP2 and MMP3 variants
 */
#include <linux/clk.h>
#include <linux/rational.h>
#include <linux/delay.h>

#include <drm/drm_modes.h>
#include <drm/drm_crtc.h>

#include "armada_crtc.h"
#include "armada_drm.h"
#include "armada_hw.h"

struct armada6x0_variant_data {
	struct clk *clks[6];
	struct clk *sel_clk;
};


static int armada6x0_crtc_init(struct armada_crtc *dcrtc, struct device *dev)
{
	struct armada6x0_variant_data *v;
	struct property *prop;
	struct clk *clk;
	const char *s;
	int idx;

	v = devm_kzalloc(dev, sizeof(*v), GFP_KERNEL);
	if (!v)
		return -ENOMEM;

	dcrtc->variant_data = v;
	dcrtc->spu_iopad_ctrl = CFG_IOPAD_DUMB24;
	dcrtc->spu_dma_ctrl1 = 0x2000ff00;
	dcrtc->spu_adv_reg = NULL;

	of_property_for_each_string(dev->of_node, "clock-names", prop, s) {
		if (!strcmp(s, "disp0"))
			idx = 0;
		else if (!strcmp(s, "disp1"))
			idx = 1;
		else if (!strcmp(s, "hdmi"))
			idx = 2;
		else if (!strcmp(s, "axibus"))
			idx = 3;
		else if (!strcmp(s, "pll3"))
			idx = 4;
		else if (!strcmp(s, "vctcxo"))
			idx = 5;
		else
			continue;

		clk = devm_clk_get(dev, s);
		if (IS_ERR(clk))
			return PTR_ERR(clk) == -ENOENT ? -EPROBE_DEFER :
				PTR_ERR(clk);
		v->clks[idx] = clk;
	}

	writel_relaxed(0, dcrtc->disp_regs + LCD_CFG_VSYNC_CTRL);

	return 0;
}

static const struct armada_clocking_params armada6x0_clocking = {
	.permillage_min = 900,
	.permillage_max = 1100,
	.settable = BIT(0) | BIT(1) | BIT(2),
	.div_max = SCLK_610_INT_DIV_MASK,
};

/*
 * This gets called with sclk = NULL to test whether the mode is
 * supportable, and again with sclk != NULL to set the clocks up for
 * that.  The former can return an error, but the latter is expected
 * not to.
 */
static int armada6x0_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk,
	const u32 clk_sels[], size_t num_clk_sels)
{
	struct armada6x0_variant_data *v = dcrtc->variant_data;
	unsigned long desired_khz = mode->crtc_clock;
	struct armada_clk_result res;
	int ret, idx;

	if (dcrtc->hdmi_regs) {
		unsigned long refdiv;
		unsigned long fbdiv;
		u32 val;

		if (sclk)
			*sclk = 0x60010005;

		rational_best_approximation(clk_get_rate(v->clks[5]),
					    desired_khz * 5000,
					    REFDIV_MASK,
					    FBDIV_MASK,
					    &refdiv, &fbdiv);

		val = readl(dcrtc->hdmi_regs + HDMI_PLL_CFG3);
		val &= ~(REFDIV_MASK << REFDIV_SHIFT);
		val &= ~(FBDIV_MASK << FBDIV_SHIFT);
		val |= refdiv << REFDIV_SHIFT;
		val |= fbdiv << FBDIV_SHIFT;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG3);

		return 0;
	}

	idx = armada_crtc_select_clock(dcrtc, &res, &armada6x0_clocking,
				       v->clks, ARRAY_SIZE(v->clks),
				       desired_khz);
	if (idx < 0)
		return idx;
	if (idx >= num_clk_sels)
		return -EINVAL;

	ret = clk_prepare_enable(res.clk);
	if (ret)
		return ret;

	if (sclk) {
		clk_set_rate(res.clk, res.desired_clk_hz);

		*sclk = 0x00001000;	/* No idea */
		*sclk |= 1 << 8;	/* MIPI clock bypass */
		*sclk |= clk_sels[idx];
		*sclk |= res.div;

		/* We are now using this clock */
		v->sel_clk = res.clk;
		swap(dcrtc->clk, res.clk);
	}

	clk_disable_unprepare(res.clk);

	return 0;
}

static const u32 armada610_clk_sels[] = {
	SCLK_610_DISP0,
	SCLK_610_DISP1,
	SCLK_610_HDMI_PLL,
	SCLK_610_AXI,
};

static int armada610_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	return armada6x0_crtc_compute_clock(dcrtc, mode, sclk,
					    armada610_clk_sels,
					    ARRAY_SIZE(armada610_clk_sels));
}

static const u32 armada620_clk_sels[] = {
	SCLK_620_DISP0,
	SCLK_620_DISP1,
	SCLK_620_HDMI_PLL,
	SCLK_620_AXI,
	SCLK_620_PLL3,
};

static int armada620_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	return armada6x0_crtc_compute_clock(dcrtc, mode, sclk,
					    armada620_clk_sels,
					    ARRAY_SIZE(armada620_clk_sels));
}

static void armada6x0_crtc_disable(struct armada_crtc *dcrtc)
{
	if (dcrtc->hdmi_regs) {
		u32 val;

		val = readl(dcrtc->hdmi_regs + HDMI_PLL_CFG3);
		val &= ~PLL_RESET;
		val &= ~PLL_ON;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG3);

		writel(0, dcrtc->hdmi_regs + HDMI_PLL_CFG2);
		writel(0, dcrtc->hdmi_regs + HDMI_PLL_CFG1);
		writel(0, dcrtc->hdmi_regs + HDMI_PLL_CFG0);
	}


	if (dcrtc->clk) {
		clk_disable_unprepare(dcrtc->clk);
		dcrtc->clk = NULL;
	}

	writel_relaxed(0, dcrtc->disp_regs + LCD_CFG_VSYNC_CTRL);
}

static void armada6x0_crtc_enable(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode)
{
	struct armada6x0_variant_data *v = dcrtc->variant_data;

	if (dcrtc->hdmi_regs) {
		int retries = 10000;
		u32 val;

		// HDMI_CLOCK_CFG // 0x0000aa10 also works
		//                        5 5`--- 1 hd enable 
		writel(0x0000aad5, dcrtc->hdmi_regs + 0x1c);
		//writel(0x0000aa10, dcrtc->hdmi_regs + 0x1c);

		// HDMI_PLL_CFG0
		//writel(0x2c442651, dcrtc->hdmi_regs + HDMI_PLL_CFG0);
		val = CHICKEN;
		val |= 8 << VDDL_SHIFT;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG0);

		// 0x01800000 works
		// writel(0x418084b8, dcrtc->hdmi_regs + HDMI_PLL_CFG1);
		val = EN_HDMI;
		val |= EN_PANEL;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG1);

		writel(0, dcrtc->hdmi_regs + HDMI_PLL_CFG2);

		val = readl(dcrtc->hdmi_regs + HDMI_PLL_CFG3);
		val &= ~PLL_RESET;
		val |= PLL_ON;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG3);
		udelay(100);
		val |= PLL_RESET;
		writel(val, dcrtc->hdmi_regs + HDMI_PLL_CFG3);

		while (retries--) {
			val = readl(dcrtc->hdmi_regs + HDMI_PLL_CFG3);
			if (val & PLL_LOCK)
				break;
			udelay(1);
		}

		if (!retries)
			DRM_WARN("HDMI PLL wouldn't lock in time");

		writel(0x00001000, dcrtc->hdmi_regs + HDMI_PHY_CFG2);
	}


	if (!dcrtc->clk && v->sel_clk) {
		if (!WARN_ON(clk_prepare_enable(v->sel_clk)))
			dcrtc->clk = v->sel_clk;
	}

	writel_relaxed(mode->hsync_start << 16 | mode->hsync_start,
		       dcrtc->disp_regs + LCD_CFG_VSYNC_CTRL);
}

const struct armada_variant armada610_ops = {
	.init = armada6x0_crtc_init,
	.compute_clock = armada610_crtc_compute_clock,
	.disable = armada6x0_crtc_disable,
	.enable = armada6x0_crtc_enable,
};

const struct armada_variant armada620_ops = {
	.init = armada6x0_crtc_init,
	.compute_clock = armada620_crtc_compute_clock,
	.disable = armada6x0_crtc_disable,
	.enable = armada6x0_crtc_enable,
};
