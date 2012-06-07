/*
 * linux/arch/arm/mach-mmp/cpufreq-mmp3.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
 * Copyright:	(C) 2010 Marvell International Ltd.
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
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <mach/mmp3_pm.h>
#include <mach/smp.h>
#include <plat/clock.h>
#include <linux/pm_qos_params.h>

static struct pm_qos_request_list cpufreq_qos_req_min;
static int cpufreq_disable = 0;
static int prof_freq_sav_khz = 0;
#define MHZ_TO_KHZ	1000
#define KHZ_TO_HZ	1000

/*
 * Frequency freq_table index must be sequential starting at 0
 * and frequencies must be ascending
 */
#define MAX_CPU_NUM	3
#define MAX_PP_NUM	8

enum {
	ONE_CORE_MM	= 0,
	ONE_CORE_MP1	= 1,
	TWO_CORES	= 2,
	THREE_CORES	= 3,
};
static int mmp3_core_num;

static struct cpufreq_frequency_table **freq_table;
static u32 num_cpus;
static struct clk *cpu_clk;

static int cpu_frequency_index_to_pp_index[MAX_CPU_NUM][MAX_PP_NUM];
static DEFINE_MUTEX(mmp3_cpu_lock);
static int freq_notify_and_change(unsigned int cpu_idx, unsigned int pp_index);

static int mmp3_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int mmp3_cpufreq_get(unsigned int cpu)
{
	int clk_pp_khz = 0;
	int clk_raw_khz;
	int delta, i;

	if (cpu >= num_cpus)
		return -EINVAL;
	clk_raw_khz = mmp3_get_core_clk(MMP3_CLK_MP1) / KHZ_TO_HZ;
	for (i = 0; i < mmp3_get_pp_number(); i++) {
		clk_pp_khz = mmp3_get_pp_freq(i, cpu);
		delta = clk_pp_khz / 10;
		if (clk_raw_khz > (clk_pp_khz - delta) &&
			clk_raw_khz < (clk_pp_khz + delta))
			break;
	}
	return clk_pp_khz;
}

static int mmp3_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int target_freq, unsigned int relation)
{
	int index, i;
	int prof_freq_khz, qos_freq_khz, freq_khz;
	int ret = 0;

	mutex_lock(&mmp3_cpu_lock);

	if (cpufreq_disable)
		goto out;
	if (cpufreq_frequency_table_target(policy, freq_table[policy->cpu],
					   target_freq, relation, &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto out;
	}

	if (policy->cur == freq_table[policy->cpu][index].frequency)
		goto out;

	/* FIXME: always use MP1 core as the index to do FC */
	prof_freq_khz = mmp3_get_pp_freq(index, MMP3_CLK_MP1);
	qos_freq_khz = pm_qos_request(PM_QOS_CPUFREQ_MIN) * MHZ_TO_KHZ;
	qos_freq_khz = min(qos_freq_khz, (int)mmp3_get_pp_freq(mmp3_get_pp_number() - 1, MMP3_CLK_MP1));
	freq_khz = max(prof_freq_khz, qos_freq_khz);
	prof_freq_sav_khz = prof_freq_khz;

	for (i = 0; i < mmp3_get_pp_number(); i++)
		if (mmp3_get_pp_freq(i, MMP3_CLK_MP1) >= freq_khz) {
			freq_notify_and_change(0, i);
			break;
		}
out:
	mutex_unlock(&mmp3_cpu_lock);
	return ret;
}

static unsigned int axi_freq[] = {
	100000,		/* core 100Mhz, axi 100Mhz */
	200000,		/* core 200Mhz, axi 200Mhz */
	266000,		/* core 400Mhz, axi 266Mhz */
	266000,		/* core 800Mhz, axi 266Mhz */
	266000,		/* core 1Ghz,   axi 266Mhz */
	266000,		/* core 1.2Ghz, axi 266Mhz */
};

static int freq_notify_and_change(unsigned int cpu_idx, unsigned int pp_index)
{
	int cpu;
	struct cpufreq_freqs freqs;
	unsigned int cfreq, afreq;
	int ret = 0;

	BUG_ON(ARRAY_SIZE(axi_freq) <= pp_index);

	/* FIXME: always use MP1 core as the index to do FC */
	cfreq = mmp3_get_pp_freq(pp_index, MMP3_CLK_MP1);
	afreq = axi_freq[pp_index];

	freqs.cpu = cpu_idx;
	freqs.old = mmp3_cpufreq_get(cpu_idx);
	freqs.new = cfreq;

	if (freqs.old == freqs.new)
		return ret;

	for_each_online_cpu(cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	clk_set_rate(cpu_clk, cfreq);
	mmp3_setfreq(MMP3_CLK_AXI_1, afreq);

	freqs.new = mmp3_cpufreq_get(cpu_idx);
	for_each_online_cpu(cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int cpufreq_min_notify(struct notifier_block *b,
				unsigned long min_mhz, void *v)
{
	int i, min_khz, freq_khz;
	mutex_lock(&mmp3_cpu_lock);

	/* FIXME: always use MP1 core as the index to do FC */
	min_khz = min_mhz * MHZ_TO_KHZ;
	min_khz = min(min_khz, (int)mmp3_get_pp_freq(mmp3_get_pp_number() - 1, MMP3_CLK_MP1));

	freq_khz = max(min_khz, prof_freq_sav_khz);
	for (i = 0; i < mmp3_get_pp_number(); i++) {
		if (mmp3_get_pp_freq(i, MMP3_CLK_MP1) >= freq_khz) {
			freq_notify_and_change(0, i);
			break;
		}
	}

	mutex_unlock(&mmp3_cpu_lock);
	return NOTIFY_OK;
}

static int cpufreq_disable_notify(struct notifier_block *b,
				unsigned long disable, void *v)
{
	int i;
	int qos_freq_khz, freq_khz;

	mutex_lock(&mmp3_cpu_lock);
	cpufreq_disable = disable;

	/* FIXME: always use MP1 core as the index to do FC */
	if (!cpufreq_disable) {
		qos_freq_khz = pm_qos_request(PM_QOS_CPUFREQ_MIN) * MHZ_TO_KHZ;
		qos_freq_khz = min(qos_freq_khz, (int)mmp3_get_pp_freq(mmp3_get_pp_number() - 1, MMP3_CLK_MP1));

		freq_khz = max(qos_freq_khz, prof_freq_sav_khz);
		for (i = 0; i < mmp3_get_pp_number(); i++) {
			if (mmp3_get_pp_freq(i, MMP3_CLK_MP1) >= freq_khz) {
				freq_notify_and_change(0, i);
				break;
			}
		}
	}

	mutex_unlock(&mmp3_cpu_lock);
	return NOTIFY_OK;
}

static struct notifier_block cpufreq_min_notifier = {
	.notifier_call = cpufreq_min_notify,
};

static struct notifier_block cpufreq_disable_notifier = {
	.notifier_call = cpufreq_disable_notify,
};

static int mmp3_cpufreq_init(struct cpufreq_policy *policy)
{
	int index;

	cpufreq_frequency_table_cpuinfo(policy, freq_table[policy->cpu]);
	cpufreq_frequency_table_get_attr(freq_table[policy->cpu], policy->cpu);

	if (!cpu_clk) {
		cpu_clk = clk_get(NULL, "cpu");
		if (IS_ERR(cpu_clk))
			return PTR_ERR(cpu_clk);
	}

	/* 100us, MMP3 FC takes 50 ~ 70 us*/
	policy->cpuinfo.transition_latency = 100 * 1000;

	policy->cur = mmp3_cpufreq_get(policy->cpu);
	if (cpufreq_frequency_table_target(policy, freq_table[policy->cpu],
				policy->cur, CPUFREQ_RELATION_L, &index)) {
		pr_err("mmp3_cpufreq_init: invalid freq: %d\n", policy->cur);
		return -EINVAL;
	}

        if (!cpu_online(1)) {
		cpumask_copy(policy->related_cpus, cpu_possible_mask);
		cpumask_copy(policy->cpus, cpu_online_mask);
	} else {
		cpumask_setall(policy->cpus);
	}

	pr_info("CPUFREQ cpu %d support for mmp3 initialized, cur %d\n",
		policy->cpu, policy->cur);
	return 0;
}

static struct freq_attr *mmp3_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver mmp3_cpufreq_driver = {
	.name	= "mmp3-cpufreq",
	.verify	= mmp3_cpufreq_verify,
	.target	= mmp3_cpufreq_target,
	.get	= mmp3_cpufreq_get,
	.init	= mmp3_cpufreq_init,
	.attr	= mmp3_cpufreq_attr,
};

static int __init cpufreq_init(void)
{
	int freq_table_item_count, i, j, k, core, freq;

	freq_table_item_count = mmp3_get_pp_number();
	/* FIXME: Here we assume all cores will be on before here! */
	num_cpus = num_online_cpus();

	freq_table =
		kzalloc(num_cpus * sizeof(struct cpufreq_frequency_table *),
			GFP_KERNEL);
	if (!freq_table)
		return -ENOMEM;

	for (i = 0; i < num_cpus; i++) {
		freq_table[i] =
			kzalloc((freq_table_item_count + 1) *
				sizeof(struct cpufreq_frequency_table),
				GFP_KERNEL);
		if (!freq_table[i])
			goto _exit;
	}

	/*
	 * FIXME:
	 * this need change when smp process id to real process id
	 * mapping is changed. We should do frequency table for each core.
	 * Currently we assume there are three cases: 1. only MP1/2 are active,
	 * the first core is MP1 and the second core is MP2. 2. Both MM and
	 * MP1/2 are active, the first core is MM, the second core is MP1 and
	 * the third core is MP2. MP1 and MP2 are always running at the same
	 * frequency. 3. UP case, it's MP1 or MM core.
	 */
	if (num_cpus == 2) {
		mmp3_core_num = TWO_CORES;
		for (i = 0; i < num_cpus; i++) {
			for (j = 0, k = 0; j < freq_table_item_count; j++) {
				freq = mmp3_get_pp_freq(j, MMP3_CLK_MP1);
				/* choose the lower pp */
				if (k > 0 &&
					freq == freq_table[i][k - 1].frequency)
					continue;
				freq_table[i][k].index = k;
				freq_table[i][k].frequency = freq;
				cpu_frequency_index_to_pp_index[i][k] = j;
				k++;
			}
			freq_table[i][k].index = k;
			freq_table[i][k].frequency = CPUFREQ_TABLE_END;
		}
	} else if (num_cpus == 3) {
		mmp3_core_num = THREE_CORES;
		for (i = 0; i < num_cpus; i++) {
			if (i == 0)
				core = MMP3_CLK_MM;
			else
				core = MMP3_CLK_MP1;
			for (j = 0, k = 0; j < freq_table_item_count; j++) {
				freq = mmp3_get_pp_freq(j, core);
				if (k > 0 &&
					freq == freq_table[i][k - 1].frequency)
					continue;
				freq_table[i][k].index = k;
				freq_table[i][k].frequency = freq;
				cpu_frequency_index_to_pp_index[i][k] = j;
				k++;
			}
			freq_table[i][k].index = k;
			freq_table[i][k].frequency = CPUFREQ_TABLE_END;
		}
	} else if (num_cpus == 1) {
#ifdef CONFIG_SMP
		pr_err("only one core detected in cpufreq driver!\n");
#endif
#ifndef CONFIG_CORE_MORPHING
		/* check if it's mm core */
		if (hard_smp_processor_id() == 2) {
			mmp3_core_num = ONE_CORE_MM;
			core = MMP3_CLK_MM;
		} else {
			mmp3_core_num = ONE_CORE_MP1;
			core = MMP3_CLK_MP1;
		}
#else
		mmp3_core_num = ONE_CORE_MP1;
		core = MMP3_CLK_MP1;
#endif
		for (j = 0, k = 0; j < freq_table_item_count; j++) {
			freq = mmp3_get_pp_freq(j, core);
			if (k > 0 &&
				freq == freq_table[0][k - 1].frequency)
				continue;
			freq_table[0][k].index = k;
			freq_table[0][k].frequency = freq;
			cpu_frequency_index_to_pp_index[0][k] = j;
			k++;
		}
		freq_table[0][k].index = k;
		freq_table[0][k].frequency = CPUFREQ_TABLE_END;
	}

	pm_qos_add_notifier(PM_QOS_CPUFREQ_MIN, &cpufreq_min_notifier);
	pm_qos_add_notifier(PM_QOS_CPUFREQ_DISABLE, &cpufreq_disable_notifier);

	pm_qos_add_request(&cpufreq_qos_req_min, PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);

	return cpufreq_register_driver(&mmp3_cpufreq_driver);

_exit:
	while (i > 0) {
		kfree(freq_table[i]);
		i--;
	}

	return -ENOMEM;
}

module_init(cpufreq_init);

static void __exit cpufreq_exit(void)
{
	int i;
	int num = num_possible_cpus();

	for (i = 0; i < num; i++) {
		if (!IS_ERR(freq_table[i]))
			kfree(freq_table[i]);
	}

	cpufreq_unregister_driver(&mmp3_cpufreq_driver);
}

module_exit(cpufreq_exit);

MODULE_DESCRIPTION("CPU frequency scaling driver for mmp3");
MODULE_LICENSE("GPL");
