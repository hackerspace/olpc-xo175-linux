/*
 *  driver/core_morph/cm_drv.c
 *
 *  Copyright (c) 2011 Marvell Semiconductors Inc.
 *  All Rights Reserved
 *
 *  core morphing's char driver used to call 'hypervisor'
 *  to switch cores.
 *
 *  author: marlon moncrieffe <mamoncri@marvell.com>
 *  Add sysfs support - Leo Yan <leoy@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */
#define DEBUG

#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/signal.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/cpufreq.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/mmp3_pm.h>
#include <mach/mmp_cm.h>

#define CMD_QUERY_CORE_ID	0
#define CMD_SWAP_CORES		1

#define MMP3_CM_THRESHOLD_FREQ	(400000)

#define endless_loop()			\
__asm__ __volatile__ (			\
	"1: b	1b\n\t"			\
	"1: b	1b\n\t"			\
)

/* core morphing api */
extern int cm_do_swap(void);
extern unsigned int get_reg(int);

/* driver data */
static dev_t cm_dev_no;			/* char device number */
static int cm_first_minor = 0;		/* starting minor number */
static int cm_count = 1;		/* number of devices */
static struct cdev *cm_cdev;		/* char device structure */

static struct class *cm_class;
static struct device *cm_dev;

static DEFINE_SPINLOCK(cm_lock);
static int cm_constraint = 0;
static int cm_on = 0;

#ifdef DEBUG
#define dump_regs()					\
do {							\
	int i;						\
	unsigned int r;					\
	printk("+-----------------+\n");		\
	for (i = 0; i < 15; i++) {			\
		r = get_reg(i);				\
		printk("| R%02d: 0x%08x |\n", i, r);	\
	}						\
	printk("+-----------------+\n");		\
} while(0)
#else
#define dump_regs() do {}  while(0)
#endif

/* ioctl handler */
static long cm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	volatile int core_id;
	unsigned long flag;

	local_irq_save(flag);

	switch (cmd) {
	case CMD_QUERY_CORE_ID:
		core_id = cm_get_active_core_id();
		if (core_id < 0) /* error */
			ret = core_id;
		printk(KERN_INFO "core id: %d\n", core_id);

		put_user(core_id, (int *)arg);
		break;

	case CMD_SWAP_CORES:

		printk(KERN_INFO "before swapping cores\n");

		dump_regs();

		__asm__ volatile (
			"bl	cm_do_swap\n\t"
		);

		dump_regs();
		break;

	default:
		printk(KERN_INFO "Invalid MSPM command.\n");
		ret = -EINVAL;
		break;
	}

	local_irq_restore(flag);
	printk(KERN_INFO "leaving core morphing driver.\n");

	return ret;
}

/* file operations */
static struct file_operations cm_fops =
{
	.owner = THIS_MODULE,
	.unlocked_ioctl = cm_ioctl,
};

void cm_enable(void)
{
	cm_on++;
	dsb();
}

int cm_vote_mp1(void)
{
	unsigned long flag;

	BUG_ON(cm_get_active_core_id() == MMP_CM_CPU_ID_MM &&
	       cm_constraint);

	spin_lock_irqsave(&cm_lock, flag);

	cm_constraint++;
	if (cm_on && cm_get_active_core_id() == MMP_CM_CPU_ID_MM) {
		dev_dbg(cm_dev,	"before swap.\n");
		__asm__ volatile (
			"bl	cm_do_swap\n\t"
		);
		dev_dbg(cm_dev,	"active core id is: sw%d:hw%d:cm%x\n",
			cm_get_active_core_id(), hard_smp_processor_id(),
			readl(MMP_CM_REG));
	}

	spin_unlock_irqrestore(&cm_lock, flag);
	return 0;
}

int cm_cancel_vote_mp1(void)
{
	unsigned long flag;

	BUG_ON(cm_get_active_core_id() == MMP_CM_CPU_ID_MM);

	spin_lock_irqsave(&cm_lock, flag);

	cm_constraint--;
	if (cm_on && !cm_constraint) {
		dev_dbg(cm_dev,	"before swap.\n");
		__asm__ volatile (
			"bl	cm_do_swap\n\t"
		);
		dev_dbg(cm_dev,	"active core id is: sw%d:hw%d:cm%x\n",
			cm_get_active_core_id(), hard_smp_processor_id(),
			readl(MMP_CM_REG));
	}

	spin_unlock_irqrestore(&cm_lock, flag);
	return 0;
}

static int swap_set(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t count)
{
	unsigned long flag;

	local_irq_save(flag);

	dev_dbg(cm_dev,	"before swap.\n");
	dump_regs();

	__asm__ volatile (
		"bl	cm_do_swap\n\t"
	);

	dev_dbg(cm_dev,	"after swap.\n");
	dump_regs();

	local_irq_restore(flag);

	return count;
}

static int swap_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int core_id, len;

	core_id = cm_get_active_core_id();
	if (core_id < 0)
		len = sprintf(buf, "get wrong core id %d\n", core_id);
	else
		len = sprintf(buf, "active core id is: sw%d:hw%d:cm%x\n",
			core_id, hard_smp_processor_id(), readl(MMP_CM_REG));

	return len;
}

static DEVICE_ATTR(swap, S_IRUGO|S_IWUSR|S_IWGRP, swap_show, swap_set);

static struct attribute *cm_attributes[] = {
	&dev_attr_swap.attr,
	NULL,
};

static struct attribute_group cm_attribute_group = {
	.attrs = cm_attributes,
};

int __init cm_init(void)
{
	int ret = 0;

	/* get the next free device number */
	ret = alloc_chrdev_region(&cm_dev_no, cm_first_minor, cm_count,
				  "core_morph");
	if (ret < 0) {
		printk(KERN_ERR "cm: unable to find free device numbers\n");
		goto alloc_cdev_fail;
	}

	/* init and load the device structure */
	cm_cdev = cdev_alloc();
	cm_cdev->ops = &cm_fops;
	cm_cdev->owner = THIS_MODULE;

	ret = cdev_add(cm_cdev, cm_dev_no, cm_count);
	if (ret < 0) {
		printk(KERN_ERR "cm: unable to add char device\n");
		goto add_cdev_fail;
	}

	cm_class = class_create(THIS_MODULE, "core_morph");
	cm_dev = device_create(cm_class, NULL, cm_dev_no, NULL, "core_morph");
	ret = sysfs_create_group(&cm_dev->kobj, &cm_attribute_group);
	if (ret) {
		printk(KERN_ERR "cm: create sysfs group failed\n");
		goto sysfs_fail;
	}

	printk(KERN_INFO "core morph has initialized succesfully.\n");
	return 0;

sysfs_fail:
	device_destroy(cm_class, cm_dev_no);
	class_destroy(cm_class);
	cdev_del(cm_cdev);
add_cdev_fail:
	unregister_chrdev_region(cm_dev_no, cm_count);
alloc_cdev_fail:
	return ret;
}

void __exit cm_exit(void)
{
	/* unregister module */
	sysfs_remove_group(&cm_dev->kobj, &cm_attribute_group);
	device_destroy(cm_class, cm_dev_no);
	class_destroy(cm_class);
	cdev_del(cm_cdev);
	unregister_chrdev_region(cm_dev_no, cm_count);
}

module_init(cm_init);
module_exit(cm_exit);

MODULE_LICENSE("GPL");
