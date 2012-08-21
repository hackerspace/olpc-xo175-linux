/*
 * arch/arm/mach-mmp/cpufreq-pxa988.c
 *
 * Copyright (C) 2012 Marvell, Inc.
 *
 * Author:
 *	Zhoujie Wu <zjwu@marvell.com>
 *	Based on arch/arm/mach-tegra/cpu-tegra.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include <linux/debugfs.h>
#include <linux/cpu.h>

#include <asm/cpu.h>
#include <asm/system.h>

#include <mach/hardware.h>

#define NUM_CPUS	2

static struct clk *cpu_clk;
static struct clk *ddr_clk;

static DEFINE_MUTEX(pxa988_cpu_lock);
static bool is_suspended;
static struct cpufreq_frequency_table *freq_table;

#ifdef CONFIG_SMP
struct lpj_info {
	unsigned long	ref;
	unsigned int	freq;
};

static DEFINE_PER_CPU(struct lpj_info, lpj_ref);
#endif

int pxa988_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

unsigned int pxa988_getspeed(unsigned int cpu)
{
	unsigned long rate;

	if (cpu >= NUM_CPUS)
		return 0;

	rate = clk_get_rate(cpu_clk) / 1000;
	return rate;
}

static int pxa988_update_cpu_speed(unsigned long rate)
{
	int ret = 0;
	struct cpufreq_freqs freqs;
	unsigned int i;

	freqs.old = pxa988_getspeed(0);
	freqs.new = rate;

	if (freqs.old == freqs.new)
		return ret;
#ifdef CONFIG_DDR_DEVFREQ
	/*
	 * If DDR devfreq is enabled, CPUfreq will NOT trigger memory bus
	 * frequency change. Memory bus frequency will change according to
	 * ddr devfreq profiler. If we have to request bus rate here, qos
	 * can be used to support it
	 */
#else
	/*
	 * Vote on memory bus frequency based on cpu frequency
	 * This sets the minimum frequency, display or avp may request higher
	 * Bringup stage
	 * Core 156M -> DDR 156M,  Core 312M -> DDR 208M
	 * Core 624M -> DDR 312M,  Core 800M -> DDR 312M
	 */
	if (rate >= 624000)
		clk_set_rate(ddr_clk, 312000000);
	else if (rate >= 312000)
		clk_set_rate(ddr_clk, 208000000);
	else
		clk_set_rate(ddr_clk, 156000000);
#endif

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

#ifdef CONFIG_CPU_FREQ_DEBUG
	printk(KERN_DEBUG "cpufreq-pxa988: transition: %u --> %u\n",
	       freqs.old, freqs.new);
#endif

	ret = clk_set_rate(cpu_clk, freqs.new * 1000);
	if (ret) {
		pr_err("cpu-pxa988: Failed to set cpu frequency to %d kHz\n",
			freqs.new);
		freqs.new = freqs.old;
	}

#ifdef CONFIG_SMP
	/*
	 * Note that loops_per_jiffy is not updated on SMP systems in
	 * cpufreq driver. So, update the per-CPU loops_per_jiffy value
	 * on frequency transition. We need to update all dependent CPUs.
	 * But we don't adjust the global one as it will always
	 * kept as the value according to the highest core freq,
	 * and used for udelay.
	 */
	for_each_online_cpu(i) {
		struct lpj_info *lpj = &per_cpu(lpj_ref, i);
		if (!lpj->freq) {
			lpj->ref = per_cpu(cpu_data, i).loops_per_jiffy;
			lpj->freq = freqs.old;
		}

		per_cpu(cpu_data, i).loops_per_jiffy =
			cpufreq_scale(lpj->ref, lpj->freq, freqs.new);
	}
#endif

	for_each_online_cpu(freqs.cpu)
		cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int pxa988_target(struct cpufreq_policy *policy,
		       unsigned int target_freq,
		       unsigned int relation)
{
	int idx;
	unsigned int freq;
	int ret = 0;

	mutex_lock(&pxa988_cpu_lock);

	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}

	cpufreq_frequency_table_target(policy, freq_table, target_freq,
		relation, &idx);

	freq = freq_table[idx].frequency;

	ret = pxa988_update_cpu_speed(freq);
out:
	mutex_unlock(&pxa988_cpu_lock);
	return ret;
}


static int pxa988_pm_notify(struct notifier_block *nb, unsigned long event,
	void *dummy)
{
	mutex_lock(&pxa988_cpu_lock);
	if (event == PM_SUSPEND_PREPARE) {
		is_suspended = true;
		pr_info("%s: disable cpu freq-chg before suspend\n",
				__func__);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
		pr_info("%s: enable cpu freq-chg after resume\n",
				__func__);
	}
	mutex_unlock(&pxa988_cpu_lock);
	return NOTIFY_OK;
}

static struct notifier_block pxa988_cpu_pm_notifier = {
	.notifier_call = pxa988_pm_notify,
};

static int pxa988_cpufreq_init(struct cpufreq_policy *policy)
{
	if (policy->cpu >= NUM_CPUS)
		return -EINVAL;

	cpu_clk = clk_get_sys(NULL, "cpu");
	if (IS_ERR(cpu_clk))
		return PTR_ERR(cpu_clk);

	ddr_clk = clk_get_sys(NULL, "ddr");
	if (IS_ERR(ddr_clk)) {
		clk_put(cpu_clk);
		return PTR_ERR(ddr_clk);
	}

	freq_table = cpufreq_frequency_get_table(policy->cpu);
	BUG_ON(!freq_table);
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	policy->cur = pxa988_getspeed(policy->cpu);

	/*
	 * FIXME: what's the actual transition time?
	 * use 10ms as sampling rate for bring up
	 */
	policy->cpuinfo.transition_latency = 10 * 1000;

	cpumask_setall(policy->cpus);
	if (policy->cpu == 0)
		register_pm_notifier(&pxa988_cpu_pm_notifier);

	return 0;
}

static int pxa988_cpufreq_exit(struct cpufreq_policy *policy)
{
	cpufreq_frequency_table_cpuinfo(policy, freq_table);
	clk_put(ddr_clk);
	clk_put(cpu_clk);
	return 0;
}

static struct freq_attr *pxa988_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver pxa988_cpufreq_driver = {
	.verify		= pxa988_verify_speed,
	.target		= pxa988_target,
	.get		= pxa988_getspeed,
	.init		= pxa988_cpufreq_init,
	.exit		= pxa988_cpufreq_exit,
	.name		= "pxa988-cpufreq",
	.attr		= pxa988_cpufreq_attr,
};

static int __init cpufreq_init(void)
{
	return cpufreq_register_driver(&pxa988_cpufreq_driver);
}

static void __exit cpufreq_exit(void)
{
	cpufreq_unregister_driver(&pxa988_cpufreq_driver);
}


MODULE_DESCRIPTION("cpufreq driver for Marvell PXA988");
MODULE_LICENSE("GPL");
module_init(cpufreq_init);
module_exit(cpufreq_exit);
