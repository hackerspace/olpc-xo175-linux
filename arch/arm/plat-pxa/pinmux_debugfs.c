/*
 * PXA9xx Pinmux print feature in debugfs
 *
 * Copyright (C) 2007 Marvell Corporation
 * Shay Pathov <shayp@marvell.com>
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */

/* Usage:
 *	mount -t debugfs none /sys/kernel/debug
 *	cat sys/kernel/debug/PINMUX/Pinmux
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/system.h>
#include <linux/types.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <linux/slab.h>

#if defined(CONFIG_CPU_PXA955)
#include <mach/pinmux_table_pxa955.h>
#else
#define BASE_ADDRESS 0
#endif

#define PXA9XX__PINMUX_NAME	"PINMUX"
#define USER_BUF_SIZE		50
#define BUFF_SIZE		4096

static ssize_t PXA9xx_PINMUX_read(struct file *file, char __user *userbuf,
				  size_t count, loff_t *ppos);

static struct dentry *dbgfs_root, *Pinmux_file;

static const struct file_operations PXA9xx_file_op = {
	.owner = THIS_MODULE,
	.read = PXA9xx_PINMUX_read
};

static ssize_t PXA9xx_PINMUX_read(struct file *file, char __user *userbuf,
				  size_t count, loff_t *ppos)
{
	char *buf = NULL;
	int sum = 0, i, pinmux_max;
	uint32_t address, value;
	const int *pinmux_arr = NULL;

#if defined(CONFIG_CPU_PXA955)
	if (cpu_is_pxa955() || cpu_is_pxa968()) {
		pinmux_arr = pinmux_offsets_pxa955;
		pinmux_max = PINMUX_TOTAL_PXA955;
	}
#endif

	if (pinmux_arr == NULL) {
		pr_err("%s: ERR: This platform is not supported!\n", __func__);
		return 0;
	}
	buf = kmalloc(BUFF_SIZE, GFP_THISNODE);
	if (buf == NULL) {
		pr_err("%s Failed to malloc\n", __func__);
		return 0;
	}
	sum += snprintf(buf, BUFF_SIZE - 1, "*** KERNEL MFPR ***\n");

	for (i = 0; i < pinmux_max; i++) {
		address = BASE_ADDRESS + pinmux_arr[i];
		value = __raw_readl((void *)&(__REG(address)));
		sum += snprintf(buf + sum, BUFF_SIZE - 1 - sum,
					"0x%x: 0x%x\n", address, value);
		if (sum > BUFF_SIZE - 1) {
			pr_err("%s Failed to write. Buffer too small\n",
			       __func__);
			kfree(buf);
			return 0;
		}
	}
	sum = simple_read_from_buffer(userbuf, count, ppos, buf, sum);
	kfree(buf);
	return sum;

}

int __init pxa9xx_pinmux_init_debugfs(void)
{
	dbgfs_root = debugfs_create_dir(PXA9XX__PINMUX_NAME, NULL);
	if (!(IS_ERR(dbgfs_root) || !dbgfs_root)) {
		Pinmux_file =
		    debugfs_create_file("Pinmux", 0600, dbgfs_root, NULL,
					&PXA9xx_file_op);
		if (Pinmux_file) {
			pr_warn("%s success\n", __func__);
		} else {
			debugfs_remove_recursive(dbgfs_root);
			pr_err("%s Failed\n", __func__);
		}
	} else
		pr_err("pinmux error: debugfs is not available\n");

	return 0;
}

void __exit pxa9xx_pinmux_cleanup_debugfs(void)
{
	debugfs_remove_recursive(dbgfs_root);
}

module_init(pxa9xx_pinmux_init_debugfs);
module_exit(pxa9xx_pinmux_cleanup_debugfs);
