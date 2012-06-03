/*
 *  linux/arch/arm/mach-mmp/dvfs-pxa988.c
 *
 *  based on arch/arm/mach-tegra/tegra2_dvfs.c
 *	 Copyright (C) 2010 Google, Inc. by Colin Cross <ccross@google.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <plat/clock.h>
#include <plat/dvfs.h>


enum {
	CORE = 0,
	DDR_AXI,
	GC,
	VPU,
	VM_RAIL_MAX,
};

/*
 * NOTES: As dvc is used to change voltage, the max
 * voltage lvl should be 4
 */
enum {
	VL0 = 0,
	VL1,
	VL2,
	VL3,
	VL_MAX,
};

struct dvfs_rail_component {
	const char *clk_name;
	bool auto_dvfs;
	const int *millivolts;
	struct dvfs_rail *dvfs_rail;
	unsigned int freqs_mult;
	unsigned long freqs[VL_MAX];
	/* used to save related clk_node and dvfs ptr */
	struct clk *clk_node;
	struct dvfs *dvfs;
};

/* voltage tbl is sort ascending */
static const int vm_millivolts[VL_MAX] = {
	1200, 1250, 1300,
};

/*
 * NOTES: we set step to 500mV here as we don't want
 * voltage change step by step, as GPIO based DVC is
 * used. This can avoid tmp voltage which is not in saved
 * in 4 level regulator table.
 */
static struct dvfs_rail pxa988_dvfs_rail_vm = {
	.reg_id = "vcc_main",
	.max_millivolts = 1300,
	.min_millivolts = 1200,
	.nominal_millivolts = 1200,
	.step = 500,
};

#define CORE_DVFS(_clk_name, _auto, _millivolts, _freqs...)	\
	{							\
		.clk_name	= _clk_name,			\
		.auto_dvfs	= _auto,			\
		.millivolts	= _millivolts,			\
		.freqs_mult	= 1000,				\
		.dvfs_rail	= &pxa988_dvfs_rail_vm,		\
		.freqs		= {_freqs},			\
	}

static struct dvfs_rail_component vm_rail_comp_tbl[VM_RAIL_MAX] = {
	CORE_DVFS("cpu", true, vm_millivolts,
		624000, 800000, 1248000),
	CORE_DVFS("ddr", true, vm_millivolts,
		312000, 400000, 400000),
	CORE_DVFS("GCCLK", true, vm_millivolts,
		312000, 416000, 624000),
	CORE_DVFS("VPUCLK", false, vm_millivolts,
		312000, 416000, 416000),
};

unsigned int pxa988_get_vl_num(void)
{
	unsigned int i = 0;
	while (vm_millivolts[i])
		i++;
	return i;
};

int pxa988_get_vl(unsigned int vl)
{
	return vm_millivolts[vl];
};

static struct dvfs *vcc_main_dvfs_init(int factor)
{
	struct dvfs *vm_dvfs = NULL;
	struct vol_table *vt = NULL;
	int i;
	unsigned int vl_num = 0;
	const char *clk_name;

	/* dvfs is not enabled for this factor in vcc_main_threshold */
	if (!vm_rail_comp_tbl[factor].auto_dvfs)
		goto err;

	clk_name = vm_rail_comp_tbl[factor].clk_name;

	vm_dvfs = kzalloc(sizeof(struct dvfs), GFP_KERNEL);
	if (!vm_dvfs) {
		pr_err("failed to request mem for vcc_main dvfs\n");
		goto err;
	}

	vl_num = pxa988_get_vl_num();
	vt = kzalloc(sizeof(struct vol_table) * vl_num, GFP_KERNEL);
	if (!vt) {
		pr_err("failed to request mem for vcc_main vol table\n");
		goto err_vt;
	}

	for (i = 0; i < vl_num; i++) {
		vt[i].freq = vm_rail_comp_tbl[factor].freqs[i] * \
			vm_rail_comp_tbl[factor].freqs_mult;
		vt[i].millivolts = vm_rail_comp_tbl[factor].millivolts[i];
		pr_info("clk[%s] rate[%lu] volt[%d]\n", clk_name, vt[i].freq,
					vt[i].millivolts);
	}
	vm_dvfs->vol_freq_table = vt;
	vm_dvfs->clk_name = clk_name;
	vm_dvfs->num_freqs = vl_num;
	vm_dvfs->dvfs_rail = vm_rail_comp_tbl[factor].dvfs_rail;

	vm_rail_comp_tbl[factor].clk_node =
		clk_get_sys(NULL, clk_name);
	vm_rail_comp_tbl[factor].dvfs = vm_dvfs;

	return vm_dvfs;
err_vt:
	kzfree(vm_dvfs);
	vm_dvfs = NULL;
err:
	return vm_dvfs;
}

static struct dvfs_rail *pxa988_dvfs_rails[] = {
	&pxa988_dvfs_rail_vm,
};

static int __init pxa988_init_dvfs(void)
{
	int i;
	struct dvfs *d;
	struct clk *c;
	int ret;

	dvfs_init_rails(pxa988_dvfs_rails, ARRAY_SIZE(pxa988_dvfs_rails));

	for (i = 0; i < VM_RAIL_MAX; i++) {
		d = vcc_main_dvfs_init(i);
		if (!d)
			continue;
		c = clk_get_sys(NULL, d->clk_name);
		if (!c) {
			pr_err("pxa988_dvfs: no clock found for %s\n",
					d->clk_name);
			kzfree(d->vol_freq_table);
			kzfree(d);
			continue;
		}
		ret = enable_dvfs_on_clk(c, d);
		if (ret) {
			pr_err("pxa988_dvfs: failed to enable dvfs on %s\n",
					c->name);
			kzfree(d->vol_freq_table);
			kzfree(d);
		}
	}

	return 0;
}
subsys_initcall(pxa988_init_dvfs);
