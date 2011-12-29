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

static int *target_pp_index;
static int cpu_frequency_index_to_pp_index[MAX_CPU_NUM][MAX_PP_NUM];
static DEFINE_MUTEX(mmp3_cpu_lock);

static int mmp3_cpufreq_verify(struct cpufreq_policy *policy)
{
	cpufreq_verify_within_limits(policy, policy->cpuinfo.min_freq,
				     policy->cpuinfo.max_freq);
	return 0;
}

static unsigned int mmp3_cpufreq_get(unsigned int cpu)
{
	if (cpu >= num_cpus)
		return -EINVAL;
#ifndef CONFIG_CORE_MORPHING
	if (cpu == 0 && (mmp3_core_num == ONE_CORE_MM ||
			 mmp3_core_num == THREE_CORES))
		return mmp3_getfreq(MMP3_CLK_MM);
	else
#endif
		return mmp3_getfreq(MMP3_CLK_MP1);
}

static unsigned long mmp3_cpu_highest_pp(void)
{
	int i, index = 0;

	for_each_online_cpu(i)
		index = max(index, target_pp_index[i]);

	return index;
}

extern int hotplug_governor_cpufreq_action(unsigned long event, struct cpufreq_freqs *freqs);

static int mmp3_cpufreq_target(struct cpufreq_policy *policy,
			       unsigned int target_freq, unsigned int relation)
{
	int index, pp_index, core, cpu;
	struct cpufreq_freqs freqs[num_cpus];
	unsigned int highest_speed;
	int ret = 0;

	mutex_lock(&mmp3_cpu_lock);

	if (cpufreq_frequency_table_target(policy, freq_table[policy->cpu],
					   target_freq, relation, &index)) {
		pr_err("cpufreq: invalid target_freq: %d\n", target_freq);
		ret = -EINVAL;
		goto out;
	}

	if (policy->cur == freq_table[policy->cpu][index].frequency)
		goto out;

	target_pp_index[policy->cpu] =
		cpu_frequency_index_to_pp_index[policy->cpu][index];

	pp_index = mmp3_cpu_highest_pp();
	/* FIXME: always use MP1 core as the index to do FC */
	highest_speed = mmp3_get_pp_freq(pp_index, MMP3_CLK_MP1);

	for_each_online_cpu(cpu) {
		freqs[cpu].cpu = cpu;
#ifndef CONFIG_CORE_MORPHING
		if (cpu == 0 && (mmp3_core_num == ONE_CORE_MM ||
				mmp3_core_num == THREE_CORES))
			core = MMP3_CLK_MM;
		else
#endif
			core = MMP3_CLK_MP1;
		freqs[cpu].old = mmp3_cpufreq_get(cpu);
		freqs[cpu].new = mmp3_get_pp_freq(pp_index, core);
	}

	if (freqs[policy->cpu].old == freqs[policy->cpu].new)
		goto out;

#ifdef CONFIG_HOTPLUG_CPU
	ret = hotplug_governor_cpufreq_action(CPUFREQ_PRECHANGE, freqs);
#endif
	for_each_online_cpu(cpu)
		cpufreq_notify_transition(&freqs[cpu], CPUFREQ_PRECHANGE);

	/*
	 * FIXME:
	 * clk_set_rate(cpu_clk, highest_speed);
	 * clk_set_rate will do spinlock with irq off, which is not
	 * valid for cpu frequency change case, one reason is the DFC
	 * routine will do mutex lock since it can be called from cpufreq
	 * and other agents like DDR and AXI thing. To do spinlock with
	 * irq off at top level do no good to complex and long clock change
	 * sequence, especially the sequency requires some system call that
	 * cannot be called from this kind of context.
	 * We will do a clock tree solution later that will solve the problem.
	 * Now we just call the raw clock change pointer to bypass the lock.
	 */
	cpu_clk->ops->setrate(cpu_clk, highest_speed);

	for_each_online_cpu(cpu) {
		freqs[cpu].new = mmp3_cpufreq_get(cpu);
		cpufreq_notify_transition(&freqs[cpu], CPUFREQ_POSTCHANGE);
	}

#ifdef CONFIG_HOTPLUG_CPU
	ret = hotplug_governor_cpufreq_action(CPUFREQ_POSTCHANGE, freqs);
#endif

out:
	mutex_unlock(&mmp3_cpu_lock);
	return ret;
}

static int mmp3_cpufreq_init(struct cpufreq_policy *policy)
{
	int index;

	cpufreq_frequency_table_cpuinfo(policy, freq_table[policy->cpu]);
	cpufreq_frequency_table_get_attr(freq_table[policy->cpu], policy->cpu);

	if (policy->cpu != 0)
		policy->min = 400000;

	if (!cpu_clk) {
		cpu_clk = clk_get(NULL, "cpu");
		if (IS_ERR(cpu_clk))
			return PTR_ERR(cpu_clk);
	}

	/* 100us, MMP3 FC takes 50 ~ 70 us*/
	policy->cpuinfo.transition_latency = 100000;

	policy->cur = mmp3_cpufreq_get(policy->cpu);
	if (cpufreq_frequency_table_target(policy, freq_table[policy->cpu],
				policy->cur, CPUFREQ_RELATION_L, &index)) {
		pr_err("mmp3_cpufreq_init: invalid freq: %d\n", policy->cur);
		return -EINVAL;
	}
	target_pp_index[policy->cpu] =
		cpu_frequency_index_to_pp_index[policy->cpu][index];

	pr_info("CPUFREQ cpu %d support for mmp3 initialized, cur %d\n",
		policy->cpu, policy->cur);
	return 0;
}

static struct freq_attr *pxa_freq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver mmp3_cpufreq_driver = {
	.verify = mmp3_cpufreq_verify,
	.target = mmp3_cpufreq_target,
	.get = mmp3_cpufreq_get,
	.init = mmp3_cpufreq_init,
	.name = "mmp3-cpufreq",
	.attr = pxa_freq_attr,
};

static int __init cpufreq_init(void)
{
	int freq_table_item_count, i, j, k, core, freq;

	freq_table_item_count = mmp3_get_pp_number();
	/* FIXME: Here we assume all cores will be on before here! */
	num_cpus = num_online_cpus();
	target_pp_index = kzalloc(num_cpus * sizeof(int),
					GFP_KERNEL);
	if (!target_pp_index)
		return -ENOMEM;
	freq_table =
		kzalloc(num_cpus * sizeof(struct cpufreq_frequency_table *),
			GFP_KERNEL);
	if (!freq_table)
		goto cpufreq_exit1;

	for (i = 0; i < num_cpus; i++) {
		freq_table[i] =
			kzalloc((freq_table_item_count + 1) *
				sizeof(struct cpufreq_frequency_table),
				GFP_KERNEL);
		if (!freq_table[i])
			goto cpufreq_exit2;
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

	return cpufreq_register_driver(&mmp3_cpufreq_driver);

cpufreq_exit2:
	while (i > 0) {
		kfree(freq_table[i]);
		i--;
	}

cpufreq_exit1:
	kfree(target_pp_index);

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
