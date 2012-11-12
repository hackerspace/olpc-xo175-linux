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
#include <mach/cputype.h>

/*
 * VCC_MAIN
 */
static enum {
	MM = 0,
	MP1, MP2, FAB,
	DCLK1, DCLK2,
	ACLK1, ACLK2,
	GC,
	VMETA,
	MMP3_VM_ROW_FACTOR_NUM,
};

static enum {
	V_EN = 0,
	V0, V1, V2, V3, V4, V5,
	MMP3_VM_COL_VOL_NUM,
};
#define MMP3_VM_VOL_NUM (MMP3_VM_COL_VOL_NUM - 1)

/* mV */
#define VCC_MAIN_STEP		25
#define VCC_MAIN_DEFAULT	1200
#define VCC_MAIN_MAX		1325
#define VCC_MAIN_MIN		1150
/* default */
static const int vcc_main_vol_table_def[] = { [V0] = 1150, [V1] = 1150, [V2] = 1150, [V3] = 1200, [V4] = 1250, [V5] = 1300, };
/* B0P */
static const int vcc_main_vol_table_b0p[][MMP3_VM_COL_VOL_NUM] = {
	[0] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1325, [V5] = 1325, },
	[1] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1325, [V5] = 1325, },
	[2] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1325, [V5] = 1325, },
	[3] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1325, [V5] = 1325, },
	[4] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1300, [V5] = 1325, },
	[5] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1275, [V5] = 1300, },
	[6] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1250, [V5] = 1275, },
	[7] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1250, [V5] = 1250, },
	[8] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1250, [V5] = 1250, },
	[9] = { [V0] = 1150, [V1] = 1150, [V2] = 1250, [V3] = 1250, [V4] = 1250, [V5] = 1250, },
};

static const char *vcc_main_clk_name[] = {
	[MM]	= "",
	[MP1]	= "cpu",
	[MP2]	= "",
	[FAB]	= "",
	[DCLK1]	= "ddr",
	[DCLK2]	= "",
	[ACLK1]	= "",
	[ACLK2]	= "",
	[GC]	= "",
	[VMETA]	= "",
};

/* kHz */
static const int vcc_main_threshold[][MMP3_VM_COL_VOL_NUM] = {
	[MM]	= { 0, 0, 0, 0, 0, 0, 0 },
	[MP1]	= { 1, 100000, 200000, 400000, 800000, 1066666, 1200000 },
	[MP2]	= { 0, 0, 0, 0, 0, 0, 0 },
	[FAB]	= { 0, 100000, 200000, 400000, 400000, 533333, 600000 },
/*	[DCLK1]	= { 0, 400000, 400000, 400000, 400000, 400000, 400000 },*/
	[DCLK1]	= { 1, 266666000, 355555000, 533333000, 800000000, 1066666000, 1066666000 },
	[DCLK2]	= { 0, 0, 0, 0, 0, 0, 0 },
	[ACLK1]	= { 0, 100000, 200000, 400000, 400000, 400000, 400000 },
	[ACLK2]	= { 0, 100000, 100000, 200000, 200000, 200000, 200000 },
	[GC]	= { 0, 533333, 533333, 533333, 533333, 533333, 533333 },
	[VMETA]	= { 0, 400000, 400000, 400000, 400000, 400000, 400000 },
};

static struct dvfs_rail mmp3_dvfs_rail_vcc_main = {
	.reg_id = "vcc_main",
	.max_millivolts = VCC_MAIN_MAX,
	.min_millivolts = VCC_MAIN_MIN,
	.nominal_millivolts = VCC_MAIN_DEFAULT,
	.step = VCC_MAIN_STEP,
};

static struct dvfs *vcc_main_dvfs_init(int factor, const int (*vcc_main_vol_table)[])
{
	struct dvfs *vm_dvfs = 0;
	struct vol_table *vt = 0;
	int i;

	/* dvfs is not enabled for this factor in vcc_main_threshold */
	if (!vcc_main_threshold[factor][V_EN])
		goto err;
	if (!(vm_dvfs = kzalloc(sizeof(struct dvfs), GFP_KERNEL))) {
		pr_err("failed to request mem for vcc_main dvfs\n");
		goto err;
	}
	if (!(vt = kzalloc(sizeof(struct vol_table) * MMP3_VM_VOL_NUM, GFP_KERNEL))) {
		pr_err("failed to request mem for vcc_main vol table\n");
		goto err_vt;
	}

	for (i = V0; i < MMP3_VM_COL_VOL_NUM; i++) {
		vt[i - V0].freq = vcc_main_threshold[factor][i];
		vt[i - V0].millivolts = (*vcc_main_vol_table)[i];
	}
	vm_dvfs->vol_freq_table = vt;
	vm_dvfs->clk_name = vcc_main_clk_name[factor];
	vm_dvfs->num_freqs = MMP3_VM_VOL_NUM;
	vm_dvfs->dvfs_rail = &mmp3_dvfs_rail_vcc_main;

	return vm_dvfs;
err_vt:
	kzfree(vm_dvfs);
err:
	return vm_dvfs;
}

/* set regulator name of vcc_main */
static char vcc_main_reg_id[16];
void mmp3_set_vcc_main_reg_id(const char *name)
{
	if (name) {
		strcpy(vcc_main_reg_id, name);
		mmp3_dvfs_rail_vcc_main.reg_id = vcc_main_reg_id;
	}
}
EXPORT_SYMBOL(mmp3_set_vcc_main_reg_id);

/*
 * DVFS
 */
static struct dvfs_relationship mmp3_dvfs_relationships[] = {
};

static struct dvfs_rail *mmp3_dvfs_rails[] = {
	&mmp3_dvfs_rail_vcc_main,
};

int __init mmp3_init_dvfs(void)
{
	int i;
	struct dvfs *d;
	struct clk *c;
	int ret;
	const int (*vcc_main_vol_table)[];

	if (cpu_is_mmp3_b0p())
		vcc_main_vol_table = &vcc_main_vol_table_b0p[mmp_soc_profile];
	else
		vcc_main_vol_table = &vcc_main_vol_table_def;

	dvfs_init_rails(mmp3_dvfs_rails, ARRAY_SIZE(mmp3_dvfs_rails));
	dvfs_add_relationships(mmp3_dvfs_relationships,
			ARRAY_SIZE(mmp3_dvfs_relationships));

	for (i = 0; i < MMP3_VM_ROW_FACTOR_NUM; i++) {
		d = vcc_main_dvfs_init(i, vcc_main_vol_table);
		if (!d)
			continue;
		c = get_clock_by_name(d->clk_name);
		if (!c) {
			pr_debug("mmp3_dvfs: no clock found for %s\n",
					d->clk_name);
			kzfree(d->vol_freq_table);
			kzfree(d);
			continue;
		}
		ret = enable_dvfs_on_clk(c, d);
		if (ret) {
			pr_err("mmp3_dvfs: failed to enable dvfs on %s\n",
					c->name);
			kzfree(d->vol_freq_table);
			kzfree(d);
		}
	}

	return 0;
}
subsys_initcall(mmp3_init_dvfs);

