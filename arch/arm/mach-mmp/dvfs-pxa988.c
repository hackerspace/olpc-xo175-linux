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
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <plat/clock.h>
#include <plat/debugfs.h>
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

#ifdef CONFIG_DEBUG_FS
static void attach_clk_auto_dvfs(const char *name, unsigned int endis)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vm_rail_comp_tbl); i++) {
		if ((vm_rail_comp_tbl[i].auto_dvfs) && \
			(!strcmp(vm_rail_comp_tbl[i].clk_name, name)))
			break;
	}
	if (i >= ARRAY_SIZE(vm_rail_comp_tbl)) {
		pr_err("clk %s doesn't support dvfs\n", name);
		return;
	}

	if (!endis)
		vm_rail_comp_tbl[i].clk_node->dvfs = NULL;
	else
		vm_rail_comp_tbl[i].clk_node->dvfs =
			vm_rail_comp_tbl[i].dvfs;
	pr_info("%s clk %s auto dvfs!\n",
		endis ? "Enable" : "Disable", name);
}

static ssize_t dc_clk_dvfs_write(struct file *filp,
	const char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[32] = {0};
	char name[10] = {0};
	unsigned int enable_dvfs = 0;

	if (copy_from_user(buf, buffer, count))
		return -EFAULT;

	if (0x2 != sscanf(buf, "%s %u", name, &enable_dvfs)) {
		pr_info("[cmd guide]echo clkname(cpu/ddr/GCCLK/VPUCLK) "
			"enable(0/1) > file node\n");
		return count;
	}
	attach_clk_auto_dvfs(name, enable_dvfs);
	return count;
}

static ssize_t dc_clk_dvfs_read(struct file *filp,
	char __user *buffer, size_t count, loff_t *ppos)
{
	char buf[156];
	int len = 0;
	size_t size = sizeof(buf) - 1;
	struct clk *clk_node;
	unsigned int i;
	const char *clk_name;

	len = snprintf(buf, size, "| name\t| auto_dvfs |\n");
	for (i = 0; i < ARRAY_SIZE(vm_rail_comp_tbl); i++) {
		if (vm_rail_comp_tbl[i].auto_dvfs) {
			clk_name = vm_rail_comp_tbl[i].clk_name;
			clk_node = vm_rail_comp_tbl[i].clk_node;
			len += snprintf(buf + len, size - len,
				"| %s\t| %d |\n", clk_name,
				clk_is_dvfs(clk_node));
		}
	}
	return simple_read_from_buffer(buffer, count, ppos, buf, len);
}

/*
 * Disable clk auto dvfs function, only avaiable when
 * has no corresponding clk FC
 */
const struct file_operations dc_clk_dvfs_fops = {
	.write = dc_clk_dvfs_write,
	.read = dc_clk_dvfs_read,
};

static int __init pxa988_dvfs_create_debug_node(void)
{
	struct dentry *dvfs_node;
	struct dentry *dc_dvfs;

	dvfs_node = debugfs_create_dir("dvfs", pxa);
	if (!dvfs_node)
		return -ENOENT;

	dc_dvfs = debugfs_create_file("dc_clk_dvfs", 0666,
		dvfs_node, NULL, &dc_clk_dvfs_fops);
	if (!dc_dvfs)
		goto err_dc_dvfs;

err_dc_dvfs:
	debugfs_remove(dvfs_node);
	return -ENOENT;
}
late_initcall(pxa988_dvfs_create_debug_node);
#endif
