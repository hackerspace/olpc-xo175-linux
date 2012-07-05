/*
 * linux/arch/arm/mach-mmp/pm-hotplug.c
 *
 * Based on S5PV310 - Dynamic CPU hotpluging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/percpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/cpufreq.h>
#include <linux/pm_qos_params.h>

#define CPUMON 1

#define CHECK_DELAY	(.5*HZ)
#define TRANS_LOAD_L	20
#define TRANS_LOAD_H	(TRANS_LOAD_L*3)

enum HP_STATE {
	HOTPLUG_PROFILING = 0,
	HOTPLUG_UP,
	HOTPLUG_DOWN,
};

static u32 hotplug_state = HOTPLUG_PROFILING;

static struct workqueue_struct *hotplug_wq;

static struct delayed_work hotplug_work;

static unsigned int hotpluging_rate = CHECK_DELAY;
static unsigned int hp_lock;
static unsigned int trans_load_l = TRANS_LOAD_L;
static unsigned int trans_load_h = TRANS_LOAD_H;
static unsigned int bound_freq = 400 * 1000;

struct cpu_time_info {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	unsigned int load;
};

static DEFINE_PER_CPU(struct cpu_time_info, hotplug_cpu_time);

/* mutex can be used since hotplug_timer does not run in
   timer(softirq) context but in process context */
static DEFINE_MUTEX(hotplug_lock);

static void hotplug_timer(struct work_struct *work)
{
	unsigned int i, avg_load = 0, load = 0;
	unsigned int cur_freq;

	mutex_lock(&hotplug_lock);

	/* force hotplug core 2 */
	if (hotplug_state == HOTPLUG_DOWN) {
		if (cpu_online(1) == 1)
			cpu_down(1);
		goto unlock;
	}

	/* force plug in core 2 */
	if (hotplug_state == HOTPLUG_UP) {
		if (cpu_online(1) == 0)
			cpu_up(1);
		goto unlock;
	}

	if (hp_lock == 1)
		goto unlock;

	for_each_online_cpu(i) {
		struct cpu_time_info *tmp_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		tmp_info = &per_cpu(hotplug_cpu_time, i);

		cur_idle_time = get_cpu_idle_time_us(i, &cur_wall_time);

		idle_time = (unsigned int)cputime64_sub(cur_idle_time,
							tmp_info->prev_cpu_idle);
		tmp_info->prev_cpu_idle = cur_idle_time;

		wall_time = (unsigned int)cputime64_sub(cur_wall_time,
							tmp_info->prev_cpu_wall);
		tmp_info->prev_cpu_wall = cur_wall_time;

		if (wall_time < idle_time)
			goto no_hotplug;

		tmp_info->load = 100 * (wall_time - idle_time) / wall_time;

		load += tmp_info->load;
	}

	avg_load = load / num_online_cpus();

	cur_freq = cpufreq_get(0);

	if (((avg_load < trans_load_l) && (cur_freq <= bound_freq)) &&
	    (cpu_online(1) == 1)) {
		printk(KERN_INFO "cpu1 turning off!\n");
		cpu_down(1);
#if CPUMON
		printk(KERN_INFO "CPUMON D %d\n", avg_load);
#endif
		pr_info("cpu1 off end!\n");
		hotpluging_rate = CHECK_DELAY;
	} else if (((avg_load > trans_load_h) && (cur_freq >= bound_freq)) &&
		   (cpu_online(1) == 0)) {
		pr_info("cpu1 turning on!\n");
		cpu_up(1);
#if CPUMON
		printk(KERN_INFO "CPUMON U %d\n", avg_load);
#endif
		printk(KERN_INFO "cpu1 on end!\n");
		hotpluging_rate = CHECK_DELAY * 4;
	}

no_hotplug:
	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, hotpluging_rate);

unlock:
	mutex_unlock(&hotplug_lock);
}

static int mmp_pm_hotplug_notifier_event(struct notifier_block *this,
					     unsigned long event, void *ptr)
{
	static unsigned hp_lock_saved;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&hotplug_lock);
		hp_lock_saved = hp_lock;
		hp_lock = 1;
		pr_info("%s: saving pm_hotplug lock %x\n",
			__func__, hp_lock_saved);
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		mutex_lock(&hotplug_lock);
		pr_info("%s: restoring pm_hotplug lock %x\n",
			__func__, hp_lock_saved);
		hp_lock = hp_lock_saved;
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block mmp_pm_hotplug_notifier = {
	.notifier_call = mmp_pm_hotplug_notifier_event,
};

static int hotplug_reboot_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	mutex_lock(&hotplug_lock);
	pr_err("%s: disabling pm hotplug\n", __func__);
	hp_lock = 1;
	mutex_unlock(&hotplug_lock);

	return NOTIFY_DONE;
}

static struct notifier_block hotplug_reboot_notifier = {
	.notifier_call = hotplug_reboot_notifier_call,
};

static int disable_hotplug_notify(struct notifier_block *nb,
				  unsigned long n, void *p)
{
	unsigned int disable;

	disable = pm_qos_request(PM_QOS_DISABLE_HP);

	mutex_lock(&hotplug_lock);
	hp_lock = disable ? 1 : 0;
	mutex_unlock(&hotplug_lock);
	return NOTIFY_OK;
}

static struct notifier_block disable_hotplug_notifier = {
	.notifier_call = disable_hotplug_notify,
};

static int hotplug_constraint_notify(struct notifier_block *nb,
				  unsigned long n, void *p)
{
	unsigned int min_cpus, max_cpus;

	min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);
	if (min_cpus < max_cpus)
		hotplug_state = HOTPLUG_PROFILING;
	else if ((min_cpus == max_cpus) && (max_cpus == 1))
		hotplug_state = HOTPLUG_DOWN;
	else if ((min_cpus == max_cpus) && (min_cpus == 2))
		hotplug_state = HOTPLUG_UP;
	else
		BUG();

	if (num_online_cpus() != n) {
		flush_delayed_work(&hotplug_work);
		queue_delayed_work_on(0, hotplug_wq, &hotplug_work, CHECK_DELAY);
	}

	return NOTIFY_OK;
}

static struct notifier_block hotplug_constraint_notifier = {
	.notifier_call = hotplug_constraint_notify,
};

static struct kobject hotplug_kobj;
struct pm_qos_request_list min_cpu_req;
struct pm_qos_request_list max_cpu_req;

static u32 parse_arg(const char *buf, size_t count)
{
	char *token, *s;
	char str[128];

	memcpy(str, buf, count);
	s = str;
	token = strsep(&s, " \t\n");
	return simple_strtol(token, NULL, 0);
}

static int loadh_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", trans_load_h);
}
static int loadh_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int loadh_tmp;
	if (!sscanf(buf, "%d", &loadh_tmp))
		return -EINVAL;
	trans_load_h = loadh_tmp;
	return count;

}
static DEVICE_ATTR(loadh, S_IRUGO | S_IWUSR, loadh_get, loadh_set);

static int loadl_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", trans_load_l);
}
static int loadl_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int loadl_tmp;
	if (!sscanf(buf, "%d", &loadl_tmp))
		return -EINVAL;
	trans_load_l = loadl_tmp;
	return count;
}
static DEVICE_ATTR(loadl, S_IRUGO | S_IWUSR, loadl_get, loadl_set);

static int bound_freq_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", bound_freq);
}
static int bound_freq_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	int freq_tmp;
	if (!sscanf(buf, "%d", &freq_tmp))
		return -EINVAL;
	bound_freq = freq_tmp;
	return count;
}
static DEVICE_ATTR(bound_freq, S_IRUGO | S_IWUSR, bound_freq_get,
		bound_freq_set);


static int lock_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", hp_lock);
}

static int lock_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	u32 val;
	int restart_hp = 0;

	val = parse_arg(buf, count);

	mutex_lock(&hotplug_lock);
	/* we want to re-enable governor */
	if ((1 == hp_lock) && (0 == val))
		restart_hp = 1;
	hp_lock = val ? 1 : 0;
	mutex_unlock(&hotplug_lock);

	if (restart_hp) {
		flush_delayed_work(&hotplug_work);
		queue_delayed_work_on(0, hotplug_wq, &hotplug_work,
				CHECK_DELAY);
	}
	return count;
}
static DEVICE_ATTR(lock, S_IRUGO | S_IWUSR, lock_get, lock_set);

static int min_cpus_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", pm_qos_request(PM_QOS_MIN_ONLINE_CPUS));
}

static int min_cpus_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	u32 max_cpus, val;

	val = parse_arg(buf, count);
	max_cpus = pm_qos_request(PM_QOS_MAX_ONLINE_CPUS);
	max_cpus = min(max_cpus, (u32)NR_CPUS);
	if (val < 1 || val > max_cpus) {
		printk(KERN_INFO "out of scope, will be 1 ~ %d\n", max_cpus);
		return -EINVAL;
	}

	pm_qos_update_request(&min_cpu_req, (s32)val);
	return count;
}
static DEVICE_ATTR(min_cpus, S_IRUGO | S_IWUSR, min_cpus_get, min_cpus_set);

static int max_cpus_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", pm_qos_request(PM_QOS_MAX_ONLINE_CPUS));
}

static int max_cpus_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count)
{
	u32 min_cpus, val;

	val = parse_arg(buf, count);
	min_cpus = pm_qos_request(PM_QOS_MIN_ONLINE_CPUS);
	if (val < min_cpus || val > NR_CPUS) {
		printk(KERN_INFO "out of scope, will be %d ~ %d\n",
			min_cpus, NR_CPUS);
		return -EINVAL;
	}

	pm_qos_update_request(&max_cpu_req, (s32)val);
	return count;
}

static DEVICE_ATTR(max_cpus, S_IRUGO | S_IWUSR, max_cpus_get, max_cpus_set);

static struct attribute *hotplug_attributes[] = {
	&dev_attr_lock.attr,
	&dev_attr_min_cpus.attr,
	&dev_attr_max_cpus.attr,
	&dev_attr_loadh.attr,
	&dev_attr_loadl.attr,
	&dev_attr_bound_freq.attr,
	NULL,
};

static struct kobj_type hotplug_dir_ktype = {
	.sysfs_ops	= &kobj_sysfs_ops,
	.default_attrs	= hotplug_attributes,
};

static int __init mmp_pm_hotplug_init(void)
{
	printk(KERN_INFO "mmp PM-hotplug init function\n");
	hotplug_wq = create_singlethread_workqueue("dynamic hotplug");
	if (!hotplug_wq) {
		printk(KERN_ERR "Creation of hotplug work failed\n");
		return -EFAULT;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_work, hotplug_timer);

	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, 60 * HZ);

	register_pm_notifier(&mmp_pm_hotplug_notifier);
	register_reboot_notifier(&hotplug_reboot_notifier);

	pm_qos_add_request(&min_cpu_req, PM_QOS_MIN_ONLINE_CPUS,
			   PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&max_cpu_req, PM_QOS_MAX_ONLINE_CPUS,
			   NR_CPUS);

	if (pm_qos_add_notifier(PM_QOS_MIN_ONLINE_CPUS,
				&hotplug_constraint_notifier))
		pr_err("%s: Failed to register min cpus PM QoS notifier\n",
			__func__);

	if (pm_qos_add_notifier(PM_QOS_MAX_ONLINE_CPUS,
				&hotplug_constraint_notifier))
		pr_err("%s: Failed to register max cpus PM QoS notifier\n",
			__func__);

	if (pm_qos_add_notifier(PM_QOS_DISABLE_HP,
				&disable_hotplug_notifier))
		pr_err("%s: Failed to register disable hp PM QoS notifier\n",
			__func__);

	if (kobject_init_and_add(&hotplug_kobj, &hotplug_dir_ktype,
				&cpu_sysdev_class.kset.kobj, "hotplug"))
		pr_err("%s: Failed to add kobject for hotplug\n", __func__);

	return 0;
}

late_initcall(mmp_pm_hotplug_init);
