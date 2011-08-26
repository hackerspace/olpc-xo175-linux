/*
 *  linux/arch/arm/mach-mmp/dvfs-mmp3.c
 *
 *  based on arch/arm/mach-tegra/tegra2_dvfs.c
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <plat/clock.h>
#include <plat/dvfs.h>

#define VCC_MAIN_STEP	50

static struct dvfs_rail mmp3_dvfs_rail_vcc_main = {
	.reg_id = "vcc_main",
	.max_millivolts = 1300, /* FIXME: what's the real max voltage? */
	.min_millivolts = 1000, /* FIXME: what's the real min voltage? */
	.nominal_millivolts = 1200, /* FIXME: update me if ntim is updated */
	.step = VCC_MAIN_STEP,
};

static struct dvfs_relationship mmp3_dvfs_relationships[] = {
};

static struct dvfs_rail *mmp3_dvfs_rails[] = {
	&mmp3_dvfs_rail_vcc_main,
};

static struct dvfs *dvfs_init[] = {
};

int __init mmp3_init_dvfs(void)
{
	int i;
	struct dvfs *d;
	struct clk *c;
	int ret;

	dvfs_init_rails(mmp3_dvfs_rails, ARRAY_SIZE(mmp3_dvfs_rails));
	dvfs_add_relationships(mmp3_dvfs_relationships,
			ARRAY_SIZE(mmp3_dvfs_relationships));

	for (i = 0; i < ARRAY_SIZE(dvfs_init); i++) {
		d = dvfs_init[i];

		c = get_clock_by_name(d->clk_name);
		if (!c) {
			pr_debug("mmp3_dvfs: no clock found for %s\n",
					d->clk_name);
			continue;
		}

		ret = enable_dvfs_on_clk(c, d);
		if (ret)
			pr_err("mmp3_dvfs: failed to enable dvfs on %s\n",
					c->name);
	}

	return 0;
}
module_init(mmp3_init_dvfs);
