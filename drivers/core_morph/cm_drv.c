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

#define CM_CPU_MP1	MMP_CM_CPU_ID_MP1
#define CM_CPU_MM	MMP_CM_CPU_ID_MM

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

static struct task_struct *cm_task;

/* driver data */
static dev_t cm_dev_no;			/* char device number */
static int cm_first_minor = 0;		/* starting minor number */
static int cm_count = 1;		/* number of devices */
static struct cdev *cm_cdev;		/* char device structure */

static struct class *cm_class;
static struct device *cm_dev;

#if defined(CONFIG_CPU_FREQ) || defined(CONFIG_HOTPLUG)
static DEFINE_MUTEX(cmtask_lock);
#endif

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

static int cm_trigger_swap(int target_core_id)
{
	int core_id;
	unsigned long flags;

	core_id = cm_get_active_core_id();
	if (core_id < 0)
		printk(KERN_ERR "get wrong core id %d\n", core_id);
	else
		printk(KERN_ERR  "current core id is %d\n", core_id);


	if (core_id == target_core_id) {
		printk(KERN_DEBUG "already at target core, quit\n");
		return 0;
	}

	local_irq_save(flags);

	__asm__ volatile (
		"bl	cm_do_swap\n\t"
	);

	local_irq_restore(flags);

	core_id = cm_get_active_core_id();
	if (core_id < 0)
		printk(KERN_ERR "get wrong core id %d\n", core_id);
	else
		printk(KERN_ERR "current core id is %d\n", core_id);

	return 0;
}

#ifdef CONFIG_CPU_FREQ
/* avoid from entering core_morph_task & mmp3_cpufreq_target
 * on two CPUs
 */
static struct cpufreq_freqs prev_freqs;
static int core_morph_task(void *data)
{
	unsigned long khz_curr;

	sched_setaffinity(0, cpumask_of(0));

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);

		schedule();
		if (kthread_should_stop())
			break;

		set_current_state(TASK_RUNNING);

		mutex_lock(&cmtask_lock);

		/* double check whether this swap is required */
		khz_curr = mmp3_getfreq(MMP3_CLK_MP1);

		if (((prev_freqs.old <= MMP3_CM_THRESHOLD_FREQ) && (khz_curr <= MMP3_CM_THRESHOLD_FREQ))
			|| ((prev_freqs.old > MMP3_CM_THRESHOLD_FREQ) && (khz_curr > MMP3_CM_THRESHOLD_FREQ))) {
			mutex_unlock(&cmtask_lock);
			continue;
		}

		cm_trigger_swap(CM_CPU_MM);

		mutex_unlock(&cmtask_lock);
	}

	return 0;
}

static int cm_task_triggered;
static int cm_cpufreq_notifier(struct notifier_block *nb,
				unsigned long val, void *data)
{
	struct cpufreq_freqs *freqs = data;

	switch (val) {
	case CPUFREQ_PRECHANGE:
		if (cm_task_triggered == 1)
			break;
		if (num_online_cpus() > 1)
			break;
		/* block cm task while cpufreq trys to switch pp */
		mutex_lock(&cmtask_lock);
		memcpy(&prev_freqs, freqs, sizeof(struct cpufreq_freqs));
#if 0
		if (((freqs->old <= MMP3_CM_THRESHOLD_FREQ) && (freqs->new > MMP3_CM_THRESHOLD_FREQ))
			|| ((freqs->old > MMP3_CM_THRESHOLD_FREQ) && (freqs->new <= MMP3_CM_THRESHOLD_FREQ)))
			wake_up_process(cm_task);
#endif
		if ((freqs->old <= MMP3_CM_THRESHOLD_FREQ) && (freqs->new > MMP3_CM_THRESHOLD_FREQ))
			cm_trigger_swap(CM_CPU_MP1);
		cm_task_triggered = 1;
		break;
	case CPUFREQ_POSTCHANGE:
		if (cm_task_triggered == 0)
			break;
		/* unblock cm task, cpufreq is done */
		mutex_unlock(&cmtask_lock);
		cm_task_triggered = 0;
		break;
	default:
		break;
	}

	return 0;
}

static struct notifier_block cm_cpufreq_notifier_block = {
	.notifier_call = cm_cpufreq_notifier
};
#endif

#ifdef CONFIG_HOTPLUG
static int __cpuinit cm_cpu_callback(struct notifier_block *nfb,
					unsigned long action, void *hcpu)
{
	unsigned long khz_curr;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		mutex_lock(&cmtask_lock);
		printk(KERN_DEBUG "MP2 is going to be plugged in\n");
		cm_trigger_swap(CM_CPU_MP1);
		mutex_unlock(&cmtask_lock);
		break;
	case CPU_DEAD:
		if (num_online_cpus() > 1) {
			printk(KERN_ERR "more than 1 CPU online, quit core morphing\n");
			break;
		}

		mutex_lock(&cmtask_lock);

		khz_curr = mmp3_getfreq(MMP3_CLK_MP1);
		printk(KERN_DEBUG "current freq: %lu\n", khz_curr);

		if (khz_curr <= MMP3_CM_THRESHOLD_FREQ) {
			cm_trigger_swap(CM_CPU_MM);
		} else {
			printk(KERN_DEBUG "use MP1!\n");
		}

		mutex_unlock(&cmtask_lock);
		break;
	default:
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block __refdata cm_cpu_notifier = {
	.notifier_call = cm_cpu_callback,
};
#endif

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

#ifdef CONFIG_CPU_FREQ
	/* reigster cpufreq notifier */
	if (cpufreq_register_notifier(&cm_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER)) {
		printk(KERN_ERR "cm: could not register cpufreq notifier\n");
		goto notifier_fail;
	}
	cm_task_triggered = 0;
	cm_task = kthread_create(core_morph_task, NULL, "core_morph");
	if (IS_ERR(cm_task)) {
		printk(KERN_ERR "cm: could not create cm task");
		/* FIXME */
		goto kthread_fail;
	}
	wake_up_process(cm_task);
#endif
#ifdef CONFIG_HOTPLUG
	register_hotcpu_notifier(&cm_cpu_notifier);
#endif

	printk(KERN_INFO "core morph has initialized succesfully.\n");
	return 0;

#ifdef CONFIG_CPU_FREQ
kthread_fail:
	cpufreq_unregister_notifier(&cm_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
notifier_fail:
	sysfs_remove_group(&cm_dev->kobj, &cm_attribute_group);
#endif
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
	kthread_stop(cm_task);
	sysfs_remove_group(&cm_dev->kobj, &cm_attribute_group);
	device_destroy(cm_class, cm_dev_no);
	class_destroy(cm_class);
	cdev_del(cm_cdev);
	unregister_chrdev_region(cm_dev_no, cm_count);
}

module_init(cm_init);
module_exit(cm_exit);

MODULE_LICENSE("GPL");
