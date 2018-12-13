/*
 * Copyright (C) 2012 Russell King
 * Copyright (C) 2018 Lubomir Rintel
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

/*
 * This gets called with sclk = NULL to test whether the mode is
 * supportable, and again with sclk != NULL to set the clocks up for
 * that.  The former can return an error, but the latter is expected
 * not to.
 */
static int armada610_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	struct clk *clk = dcrtc->axiclk;
	uint32_t rate, ref, div;

	if (!clk)
		return -EINVAL;

	rate = mode->clock * 1000;
	ref = clk_get_rate(clk);
	div = DIV_ROUND_UP(ref, rate);

	if (div < 2)
		return -EINVAL;

	if (sclk) {
		*sclk = 0x00001100; /* No idea */
		*sclk |= (0x1 << 30); /* SCLK_SOURCE_SELECT = AXI bus clk */
		*sclk |= div;
	}

	return 0;
}

const struct armada_variant armada610_ops = {
	.compute_clock = armada610_crtc_compute_clock,
};
