/*
 * Board Level freq change Driver
 *
 * Copyright (C) 2008 Marvell Corporation
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sysdev.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/setup.h>

#include <mach/mmp2_fuse.h>
#include <mach/board_fc_ops.h>
#include <mach/vmalloc.h>

/*
 * private data
 */
static struct board_fc_ops *this_ops = NULL;
static struct board_fc_info *board_info = NULL;

/*
 * public functions
 */


int register_board_fc_ops(struct board_fc_ops *ops)
{
	if (!ops)
		return -EINVAL;
	if (this_ops)
		return -EBUSY;

	this_ops = ops;
	board_info = (struct board_fc_info *)kzalloc(sizeof(struct board_fc_info), GFP_KERNEL);
	if (!board_info)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL(register_board_fc_ops);

int unregister_board_fc_ops(struct board_fc_ops *ops)
{
	this_ops = NULL;
	if (!board_info)
		kfree(board_info);

	return 0;
}
EXPORT_SYMBOL(unregister_board_fc_ops);

int board_get_profile(void)
{
	if (!this_ops)
		return -ENODEV;

	return this_ops->get_profile();
}

int board_get_max_freq(void)
{
	if (!this_ops)
		return -ENODEV;

	return this_ops->get_max_freq();
}

int board_get_voltage(unsigned int product_point)
{
	if (!this_ops)
		return -ENODEV;

	return this_ops->get_voltage(product_point);
}

int setup_fc_seq(void *vaddr)
{
	if (!this_ops)
		return -ENODEV;

	return this_ops->setup_fc_seq(vaddr);
}

int run_fc_seq(struct mmp2_fc_param *param, struct mmp2_pm_info *info)
{
	void *vaddr, *vstack;

	if (!this_ops)
		return -ENODEV;

	vaddr = info->fc_vaddr;
	vstack = info->fc_vstack;

	board_info->va_pmua = info->pmua_base;
	board_info->va_pmum = info->pmum_base;
	board_info->va_dmcu = info->dmcu_base;
	board_info->cc_ap = param->cc_ap;
	board_info->fccr = param->fccr;
	board_info->v_flag = param->v_flag;
	board_info->mem_config = param->mem_config;

	return this_ops->execute_fc_seq(vaddr, vstack, param->voltage, board_info);
}

