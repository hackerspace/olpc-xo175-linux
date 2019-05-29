/*
 * MMP2 FC Driver
 *
 * Copyright (C) 2008 Marvell Corporation
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#undef DEBUG
//#define DEBUG
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/max8649.h>

#include <asm/io.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <mach/cputype.h>
#include <mach/hardware.h>
#include <mach/mmp2_fuse.h>
#include <mach/vmalloc.h>
#include <mach/board_fc_ops.h>
#include <mach/mfp-mmp2.h>
#include <mach/mmp2_plat_ver.h>

static int this_profile;
static int max_freq;

/**
 * bs_setup_fc_seq - setup the FC sequence in SRAM
 * @vaddr: the virtual address of SRAM
 */
static int bs_setup_fc_seq(void *vaddr)
{
	void *start_addr, *end_addr;

	start_addr = (void *)&bs_do_fcs;
	end_addr = (void *)&bs_do_fcs_end;

	copy_fcs_to_sram(vaddr, start_addr, end_addr);

	return 0;
}

/**
 * bs_exe_fc_seq - jump to SRAM and execute the FC seq.
 * @vaddr: the virtual address of SRAM
 * @info: the essential parameters for FC seq.
 */
static int bs_exe_fc_seq(void *vaddr, void *vstack, int vol, struct board_fc_info *info)
{
	int v;

	/* calc the value that should be writen to PMIC */
	v = max8649_calculate_voltage_reg(vol);
	info->priv = &v;

	v = jump_to_fcs_sram(vaddr, vstack, info);

	return 0;
}

/**
 * bs_exe_fc_seq - return the max freq
 */
static int bs_get_max_freq(void)
{
	int max;

	if (max_freq == MMP2_MAX_FREQ_988MHZ)
		max = 988;
	else
		max = 800;

	/* brownstone rev 1-4: for profile 0-5, max freq is 800MHz
	 * brownstone rev 5: no this limitation*/
	if ((this_profile < 6) && (board_is_mmp2_brownstone_rev5() != 1))
		max = 800;

	return max;
}

static int bs_get_voltage(unsigned int product_point)
{
	return mmp2_get_voltage(this_profile, product_point);
}

static int bs_get_profile(void)
{
	return this_profile;
}

static struct board_fc_ops brownstone_ops = {
	.setup_fc_seq		= bs_setup_fc_seq,
	.execute_fc_seq		= bs_exe_fc_seq,
	.get_profile		= bs_get_profile,
	.get_max_freq		= bs_get_max_freq,
	.get_voltage		= bs_get_voltage,
};

static int __init brownstone_fc_init(void)
{
	if (machine_is_brownstone()) {
		this_profile = mmp2_read_profile();
		max_freq = mmp2_read_max_freq();

		register_board_fc_ops(&brownstone_ops);
	}

	return 0;
}

static void __exit brownstone_fc_exit(void)
{
	unregister_board_fc_ops(&brownstone_ops);
}

arch_initcall(brownstone_fc_init);
__exitcall(brownstone_fc_exit);

