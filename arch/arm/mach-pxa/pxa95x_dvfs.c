/*
 *  linux/arch/arm/mach-pxa/pxa95x_dvfs.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <plat/clock.h>
#include <mach/dvfs.h>

#define DRIVER_NAME	"pxa95x_dvfs"
#define MHZ_TO_KHZ	1000

static int dvfs_notifier_freq(struct notifier_block *nb,
			      unsigned long val, void *data)
{
	struct dvfs_freqs *freqs = (struct dvfs_freqs *)data;
	unsigned int old_freq = freqs->old;
	unsigned int new_freq = freqs->new;

	switch (val) {
	case DVFS_FREQ_PRECHANGE:
		if (old_freq < new_freq)
			set_dvfs_rate(freqs->dvfs, freqs->new);
		break;
	case DVFS_FREQ_POSTCHANGE:
		if (old_freq > new_freq)
			set_dvfs_rate(freqs->dvfs, freqs->new);
		break;
	default:
		break;
	}
	return 0;
}

static struct notifier_block notifier_freq_block = {
	.notifier_call = dvfs_notifier_freq,
};

static struct dvfs_rail pxa95x_dvfs_rail_vcc_main = {
	.reg_id = "vcc_main",
	.max_millivolts = VOL_LEVL3_1,
	.min_millivolts = VOL_LEVL0,
	.nominal_millivolts = VOL_LEVL2,
};

static struct dvfs_relationship pxa95x_dvfs_relationships[] = {
};

static struct dvfs_rail *pxa95x_dvfs_rails[] = {
	&pxa95x_dvfs_rail_vcc_main,
};

static struct vol_table core_vol_table[] = {
	INIT_VOL_TABLE(156 * MHZ_TO_KHZ, VOL_LEVL0),
	INIT_VOL_TABLE(312 * MHZ_TO_KHZ, VOL_LEVL0),
	INIT_VOL_TABLE(624 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(806 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(1014 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(1196 * MHZ_TO_KHZ, VOL_LEVL3_0),
	INIT_VOL_TABLE(1404 * MHZ_TO_KHZ, VOL_LEVL3_1),
	INIT_VOL_TABLE(1508 * MHZ_TO_KHZ, VOL_LEVL3_1),
};

struct dvfs core_dvfs = {
	.clk_name = "pxa95x_core",
	.vol_freq_table = core_vol_table,
	.num_freqs = ARRAY_SIZE(core_vol_table),
	.dvfs_rail = &pxa95x_dvfs_rail_vcc_main,
	.millivolts = VOL_LEVL2,
};
EXPORT_SYMBOL(core_dvfs);

static struct vol_table display_vol_table[] = {
	INIT_VOL_TABLE(104 * MHZ_TO_KHZ, VOL_LEVL0),
	INIT_VOL_TABLE(156 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(312 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(416 * MHZ_TO_KHZ, VOL_LEVL3_0),
};

struct dvfs display_dvfs = {
	.clk_name = "PXA95x_LCDCLK",
	.vol_freq_table = display_vol_table,
	.num_freqs = ARRAY_SIZE(display_vol_table),
	.dvfs_rail = &pxa95x_dvfs_rail_vcc_main,
	.millivolts = VOL_LEVL2,
};
EXPORT_SYMBOL(display_dvfs);

static struct vol_table gc_vol_table[] = {
	INIT_VOL_TABLE(156 * MHZ_TO_KHZ, VOL_LEVL0),
	INIT_VOL_TABLE(208 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(312 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(416 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(481 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(498 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(600 * MHZ_TO_KHZ, VOL_LEVL3_0),
};

struct dvfs gc_dvfs = {
	.clk_name = "GCCLK",
	.vol_freq_table = gc_vol_table,
	.num_freqs = ARRAY_SIZE(gc_vol_table),
	.dvfs_rail = &pxa95x_dvfs_rail_vcc_main,
	.millivolts = VOL_LEVL2,
};
EXPORT_SYMBOL(gc_dvfs);

static struct vol_table vmeta_vol_table[] = {
	INIT_VOL_TABLE(156 * MHZ_TO_KHZ, VOL_LEVL0),
	INIT_VOL_TABLE(208 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(312 * MHZ_TO_KHZ, VOL_LEVL1),
	INIT_VOL_TABLE(416 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(481 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(498 * MHZ_TO_KHZ, VOL_LEVL2),
	INIT_VOL_TABLE(600 * MHZ_TO_KHZ, VOL_LEVL3_0),
};

struct dvfs vmeta_dvfs = {
	.clk_name = "VMETA_CLK",
	.vol_freq_table = vmeta_vol_table,
	.num_freqs = ARRAY_SIZE(vmeta_vol_table),
	.dvfs_rail = &pxa95x_dvfs_rail_vcc_main,
	.millivolts = VOL_LEVL2,
};
EXPORT_SYMBOL(vmeta_dvfs);

static struct dvfs *dvfs_init[] = {
	&core_dvfs,
	&display_dvfs,
	&gc_dvfs,
	&vmeta_dvfs,
};

int pxa95x_init_dvfs(void)
{
	int i;
	dvfs_init_rails(pxa95x_dvfs_rails, ARRAY_SIZE(pxa95x_dvfs_rails));
	dvfs_add_relationships(pxa95x_dvfs_relationships,
			       ARRAY_SIZE(pxa95x_dvfs_relationships));
	for (i = 0; i < ARRAY_SIZE(dvfs_init); i++)
		list_add_tail(&dvfs_init[i]->dvfs_node,
			      &pxa95x_dvfs_rail_vcc_main.dvfs);
	dvfs_register_notifier(&notifier_freq_block, DVFS_FREQUENCY_NOTIFIER);

	return 0;
}

MODULE_LICENSE("GPL");
MODULE_ALIAS(DRIVER_NAME);
MODULE_AUTHOR("Xiaoguang Chen <chenxg@marvell.com>");
MODULE_DESCRIPTION("DVFS driver on pxa95x platform");
