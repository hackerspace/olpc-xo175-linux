/*
 * Copyright (C) 2012 Russell King
 * Copyright (C) 2018,2019 Lubomir Rintel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Armada MMP2 variant support
 */
#include <linux/clk.h>
#include <drm/drm_modes.h>
#include <drm/drm_crtc.h>
#include "armada_crtc.h"
#include "armada_drm.h"
#include "armada_hw.h"

static int armada610_crtc_init(struct armada_crtc *dcrtc, struct device *dev)
{
	struct clk *clk;

	clk = devm_clk_get(dev, "disp0");
	if (IS_ERR(clk))
		return PTR_ERR(clk) == -ENOENT ? -EPROBE_DEFER : PTR_ERR(clk);

	dcrtc->extclk[0] = clk;

	return 0;
}

/*
 * This gets called with sclk = NULL to test whether the mode is
 * supportable, and again with sclk != NULL to set the clocks up for
 * that.  The former can return an error, but the latter is expected
 * not to.
 */
static int armada610_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	struct clk *clk = dcrtc->extclk[0];
	uint32_t ret, rate, ref, div;

	if (IS_ERR(clk))
		return PTR_ERR(clk);

	rate = mode->clock * 1000;
	ref = clk_get_rate(clk);
	div = DIV_ROUND_UP(ref, rate);

	if (div < 2)
		return -EINVAL;

	if (dcrtc->clk != clk) {
		ret = clk_prepare_enable(clk);
		if (ret)
			return ret;
		dcrtc->clk = clk;
	}

	if (sclk) {
		*sclk = 0x00001000; /* No idea */
		*sclk |= 1 << 8; /* MIPI clock bypass */
		*sclk |= SCLK_610_DISP0;
		*sclk |= div;
	}

	return 0;
}

static void armada610_crtc_disable(struct armada_crtc *dcrtc)
{
	if (!IS_ERR(dcrtc->clk)) {
		clk_disable_unprepare(dcrtc->clk);
		dcrtc->clk = ERR_PTR(-EINVAL);
	}
}

static void armada610_crtc_enable(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode)
{
	if (IS_ERR(dcrtc->clk)) {
		dcrtc->clk = dcrtc->extclk[0];
		WARN_ON(clk_prepare_enable(dcrtc->clk));
	}
}

const struct armada_variant armada610_ops = {
	.init = armada610_crtc_init,
	.compute_clock = armada610_crtc_compute_clock,
	.disable = armada610_crtc_disable,
	.enable = armada610_crtc_enable,
};
