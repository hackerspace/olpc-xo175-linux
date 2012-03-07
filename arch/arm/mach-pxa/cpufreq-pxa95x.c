/*
 * linux/arch/arm/mach-pxa/cpufreq-pxa95x.c
 *
 * Copyright (C) 2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/sysdev.h>
#include <linux/cpu.h>
#include <linux/slab.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#if defined(CONFIG_PXA95x_DVFM)
#include <mach/dvfm.h>
#include <mach/pxa95x_dvfm.h>
#endif

#include "generic.h"

#define MAX_CORE_FREQS	50
#define MHZ_TO_KHZ	1000

/* Core frequencies table according to PP,
 in MHz */
static int core_freqs_table[MAX_CORE_FREQS];
static int num_pp;

static unsigned int pxa95x_freqs_num;
static struct cpufreq_frequency_table *pxa95x_freqs_table;
static spinlock_t cpufreq_stats_lock;
static struct cpufreq_policy *pxa95x_cpufreq_policy;

static unsigned int pxa95x_cpufreq_get(unsigned int cpu);

struct cpufreq_freq_load_table {
	struct cpufreq_frequency_table *table_item;
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t idle_time;
	cputime64_t busy_time;
	cputime64_t total_idle_time;
	cputime64_t total_busy_time;
};

struct cpu_load_info {
	int opnum;
	int last_index;
	struct cpufreq_freq_load_table *pxa95x_freq_load_table;
};

static DEFINE_PER_CPU(struct cpu_load_info, pxa95x_cpu_load_info);

/* Interface under SYSFS */
/* Display duty cycles on all operating points */
static ssize_t all_supported_freqs_show(struct sys_device *sys_dev,
					struct sysdev_attribute *attr,
					char *buf)
{
	int len, i;

	len = sprintf(buf, "Supported CPU frequencies:\n");
	for (i = 0; i < num_pp; i++) {
		len += sprintf(&buf[len], "freq[%d] = %d MHz\n",
			       i, core_freqs_table[i]);
	}

	return len;
}

SYSDEV_ATTR(supported_freqs, 0444, all_supported_freqs_show, NULL);

/* Display duty cycles on all operating points */
static ssize_t current_freq_show(struct sys_device *sys_dev,
				 struct sysdev_attribute *attr, char *buf)
{
	int freq;

	freq = pxa95x_cpufreq_get(0);
	return sprintf(buf, "Current frequency = %d\n", freq);
}

SYSDEV_ATTR(current_freq, 0444, current_freq_show, NULL);

static struct attribute *cpufreq_stats_attr[] = {
	&attr_supported_freqs.attr,
	&attr_current_freq.attr,
};

static int stats_add(struct sys_device *sys_dev)
{
	int i, n, ret;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj),
					cpufreq_stats_attr[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int stats_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(cpufreq_stats_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(sys_dev->kobj), cpufreq_stats_attr[i]);

	return 0;
}

static struct sysdev_driver cpufreq_stats_driver = {
	.add = stats_add,
	.remove = stats_rm,
};

static int setup_freqs_table(struct cpufreq_policy *policy,
			     int *freqs_table, int num)
{
	struct cpufreq_frequency_table *table;
	struct cpu_load_info *load_info;
	struct cpufreq_freq_load_table *load_table;

	int i, ret;

	table = kzalloc((num + 1) * sizeof(*table), GFP_KERNEL);
	if (table == NULL)
		return -ENOMEM;
	load_info = &per_cpu(pxa95x_cpu_load_info, policy->cpu);
	load_info->last_index = -1;
	load_table = load_info->pxa95x_freq_load_table =
	    kzalloc((num + 1) * sizeof(*load_table), GFP_KERNEL);
	if (!load_table) {
		kfree(table);
		return -ENOMEM;
	}

	for (i = 0; i < num; i++) {
		table[i].index = i;
		table[i].frequency = freqs_table[i] * MHZ_TO_KHZ;
		load_table[i].table_item = &table[i];
	}
	table[num].index = i;
	table[num].frequency = CPUFREQ_TABLE_END;
	load_table[num].table_item = &table[num];

	pxa95x_freqs_num = load_info->opnum = num;
	pxa95x_freqs_table = table;

	ret = cpufreq_frequency_table_cpuinfo(policy, table);
	cpufreq_frequency_table_get_attr(table, policy->cpu);
	return ret;
}

static int pxa95x_cpufreq_verify(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, pxa95x_freqs_table);
}

static unsigned int pxa95x_cpufreq_get(unsigned int cpu)
{
	if (cpu != 0)
		return -EINVAL;

	return dvfm_current_core_freq_get() * MHZ_TO_KHZ;
}

static int pxa95x_cpufreq_target(struct cpufreq_policy *policy,
				 unsigned int target_freq,
				 unsigned int relation)
{
	int index;
	struct cpufreq_frequency_table *table =
	    cpufreq_frequency_get_table(policy->cpu);

	if (cpufreq_frequency_table_target(policy, table, target_freq, relation,
					   &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		return -EINVAL;
	}

	if (policy->cur == table[index].frequency)
		return 0;

#ifdef CONFIG_CPU_FREQ_DEBUG
	pr_info("target_freq is %d, index is %d\n", target_freq, index);
#endif

	/* Set the required constraints as derived from the */
	/* target freq */
	dvfm_freq_set(table[index].frequency / MHZ_TO_KHZ,
		      relation ==
		      CPUFREQ_RELATION_L ? RELATION_LOW : RELATION_HIGH);

	return 0;
}

static inline cputime64_t get_cpu_idle_time_jiffy(unsigned int cpu,
						  cputime64_t *wall)
{
	cputime64_t idle_time;
	cputime64_t cur_wall_time;
	cputime64_t busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());
	busy_time = cputime64_add(kstat_cpu(cpu).cpustat.user,
				  kstat_cpu(cpu).cpustat.system);

	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.irq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.softirq);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.steal);
	busy_time = cputime64_add(busy_time, kstat_cpu(cpu).cpustat.nice);

	idle_time = cputime64_sub(cur_wall_time, busy_time);
	if (wall)
		*wall = (cputime64_t) jiffies_to_usecs(cur_wall_time);
	return (cputime64_t) jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu,
					    cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, wall);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);

	return idle_time;
}

static int cpufreq_stats_update(int cpu)
{
	cputime64_t cur_wall_time, cur_idle_time;
	unsigned int idle_time, wall_time;
	struct cpu_load_info *load_info = &per_cpu(pxa95x_cpu_load_info, cpu);
	struct cpufreq_freq_load_table *load_table =
	    load_info->pxa95x_freq_load_table;
	struct cpufreq_freq_load_table *last_table =
	    &load_table[load_info->last_index];

	spin_lock(&cpufreq_stats_lock);

	cur_idle_time = get_cpu_idle_time(cpu, &cur_wall_time);
	wall_time =
	    (unsigned int)cputime64_sub(cur_wall_time,
					last_table->prev_cpu_wall);

	idle_time =
	    (unsigned int)cputime64_sub(cur_idle_time,
					last_table->prev_cpu_idle);

	last_table->idle_time += idle_time;
	last_table->busy_time += wall_time - idle_time;
	last_table->total_idle_time += idle_time;
	last_table->total_busy_time += wall_time - idle_time;
	last_table->prev_cpu_wall = cur_wall_time;
	last_table->prev_cpu_idle = cur_idle_time;
	spin_unlock(&cpufreq_stats_lock);
	return 0;
}

void clean_status(struct cpu_load_info *load_info)
{
	int i;
	struct cpufreq_freq_load_table *load_table;
	load_table = load_info->pxa95x_freq_load_table;

	for (i = 0; i < load_info->opnum; i++) {
		if (i != load_info->last_index) {
			load_table[i].prev_cpu_idle = 0;
			load_table[i].prev_cpu_wall = 0;
		}
		load_table[i].idle_time = 0;
		load_table[i].busy_time = 0;
	}
}

static int firstwindow = 1;
unsigned int pxa95x_cpufreq_getavg(struct cpufreq_policy *policy,
				   unsigned int cpu)
{
	unsigned long long freq = 0;
	cputime64_t total_time = 0;
	struct cpu_load_info *load_info;
	struct cpufreq_freq_load_table *load_table;
	int i;
	load_info = &per_cpu(pxa95x_cpu_load_info, cpu);
	load_table = load_info->pxa95x_freq_load_table;
	if (firstwindow) {
		firstwindow = 0;
		for (i = 0; i < load_info->opnum; i++) {
			if (load_table[i].table_item->frequency ==
			    policy->cur * MHZ_TO_KHZ) {
				load_info->last_index = i;
				load_table[i].prev_cpu_idle =
				    get_cpu_idle_time(cpu,
						      &load_table
						      [i].prev_cpu_wall);
				break;
			}
		}

		return 0;
	}
	cpufreq_stats_update(cpu);
	for (i = 0; i < load_info->opnum; i++) {
		freq +=
		    load_table[i].table_item->frequency *
		    load_table[i].busy_time;
		total_time += load_table[i].busy_time;
	}
	do_div(freq, total_time);
	clean_status(load_info);
	return freq;
}

static int cpufreq_stat_notifier_trans(struct notifier_block *nb,
				       unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_load_info *load_info;
	struct cpufreq_freq_load_table *load_table;
	int i;
	int cpu = smp_processor_id();
	load_info = &per_cpu(pxa95x_cpu_load_info, cpu);
	load_table = load_info->pxa95x_freq_load_table;
	if (val != CPUFREQ_POSTCHANGE || firstwindow)
		return 0;
	cpufreq_stats_update(cpu);
	if (freq->old != freq->new) {
		for (i = 0; i < load_info->opnum; i++) {
			if (load_table[i].table_item->frequency == freq->new) {
				load_info->last_index =
				    load_table[i].table_item->index;
				load_table[i].prev_cpu_idle =
				    get_cpu_idle_time(cpu,
						      &load_table
						      [i].prev_cpu_wall);
				break;
			}
		}
	}
	return 0;
}

static struct notifier_block notifier_trans_block = {
	.notifier_call = cpufreq_stat_notifier_trans
};

static int pxa95x_cpufreq_init(struct cpufreq_policy *policy)
{
	int current_freq_khz;
	int ret = 0;

	pxa95x_cpufreq_policy = policy;

#ifdef CONFIG_PXA95x_DVFM
	ret = dvfm_core_freqs_table_get(core_freqs_table,
					&num_pp, MAX_CORE_FREQS);
	if (ret)
		pr_err("failed to get cpu frequency table");

	/* set to 0 to indicate that no user configured */
	/* cpufreq until now */
	current_freq_khz = dvfm_current_core_freq_get() * MHZ_TO_KHZ;

	/* Set to 100us latency.
	 * This will cause the sampling_rate to 100ms.
	 * Need to tuning it later */
	policy->cpuinfo.transition_latency = 100 * 1000;

	policy->cur = current_freq_khz;

	/* setup cpuinfo frequency table */
	ret = setup_freqs_table(policy, core_freqs_table, num_pp);
	if (ret) {
		pr_err("failed to setup frequency table\n");
		return ret;
	}
	ret = cpufreq_register_notifier(&notifier_trans_block,
					CPUFREQ_TRANSITION_NOTIFIER);
	if (ret)
		pr_err("failed to register notifier!\n");

#endif /* CONFIG_PXA95x_DVFM */

	pr_info("CPUFREQ support for PXA95x initialized\n");
	return ret;
}

static struct freq_attr *pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver pxa95x_cpufreq_driver = {
	.verify = pxa95x_cpufreq_verify,
	.target = pxa95x_cpufreq_target,
	.getavg = pxa95x_cpufreq_getavg,
	.init = pxa95x_cpufreq_init,
	.get = pxa95x_cpufreq_get,
	.name = "pxa95x-cpufreq",
	.attr = pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	int ret;

	ret = sysdev_driver_register(&cpu_sysdev_class, &cpufreq_stats_driver);
	if (ret)
		printk(KERN_ERR "Can't register cpufreq STATS in sysfs\n");

	return cpufreq_register_driver(&pxa95x_cpufreq_driver);
}

module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	int j;
	struct cpu_load_info *load_info;
	sysdev_driver_unregister(&cpu_sysdev_class, &cpufreq_stats_driver);
	cpufreq_unregister_notifier(&notifier_trans_block,
				    CPUFREQ_TRANSITION_NOTIFIER);
	kfree(pxa95x_freqs_table);
	for_each_cpu(j, pxa95x_cpufreq_policy->cpus) {
		load_info = &per_cpu(pxa95x_cpu_load_info, j);
		kfree(load_info->pxa95x_freq_load_table);
	}
	cpufreq_unregister_driver(&pxa95x_cpufreq_driver);
}

module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for PXA95x");
MODULE_LICENSE("GPL");
