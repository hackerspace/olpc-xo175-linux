/*
 * Copyright (C) 2012 Russell King
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Armada 610/MMP2/MMP3 variant support
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include "armada_crtc.h"
#include "armada_drm.h"
#include "armada_hw.h"

static int mmp23_crtc_init(struct armada_crtc *dcrtc, struct device *dev)
{
	dcrtc->extclk[0] = devm_clk_get(dev, NULL);

	if (IS_ERR(dcrtc->extclk[0]) && PTR_ERR(dcrtc->extclk[0]) == -ENOENT)
		dcrtc->extclk[0] = ERR_PTR(-EPROBE_DEFER);

	return PTR_RET(dcrtc->extclk[0]);
}

/*
 * This gets called with sclk = NULL to test whether the mode is
 * supportable, and again with sclk != NULL to set the clocks up for
 * that.  The former can return an error, but the latter is expected
 * not to.
 */
static int mmp23_crtc_compute_clock(struct armada_crtc *dcrtc,
	const struct drm_display_mode *mode, uint32_t *sclk)
{
	/*
	 * on MMP3 bits 31:29 select the clock, OLPC wants 0x1 here, LCD clock 1
	 * on MMP2 bits 31:30 select the clock, OLPC wants 0x1 here, LCD clock 1
	 */
	*sclk = 0x20001100; // FIXME hardcoded mmp3 value

	return 0;
}

const struct armada_variant mmp23_ops = {
	.init = mmp23_crtc_init,
	.compute_clock = mmp23_crtc_compute_clock,
};
