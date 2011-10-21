/*
 * arch/arm/mach-mmp/cpuidle-mmp2.c
 *
 * CPU idle Marvell mmp2 SoCs
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * The cpu idle states -
 * #1 core_extidle: core clk is gated externally
 * #2 apps_idle: axi off, ddr off
 * #3 apps_sleep: axi off, ddr off, apb off
 * #4 chip_sleep: sleepen + apb clk off + axi off + ddr clk down & self refresh
 *			vcxo on + pll1 on + pll2 on
 *
 * constraint
 * #axi: lcd, usb, camera, sdh, etc.
 * #apb: uart, pwm, i2c, etc.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cpuidle.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <mach/mmp2_pm.h>
#include <mach/regs-mpmu.h>
#include <linux/pm_qos_params.h>
#include <linux/wakelock.h>

#define MMP2_MAX_STATES	4

static struct cpuidle_driver mmp2_idle_driver = {
	.name =         "mmp2_idle",
	.owner =        THIS_MODULE,
};

static DEFINE_PER_CPU(struct cpuidle_device, mmp2_cpuidle_device);

/* Actual code that puts the SoC in different idle states */
static int mmp2_enter_idle(struct cpuidle_device *dev,
			       struct cpuidle_state *state)
{
	struct timeval before, after;
	int idle_time;
	int mode = 0;
	uint32_t awucrm = 0;
	uint32_t apcr = 0;

	local_irq_disable();
	do_gettimeofday(&before);

	if (state == &dev->states[0])
		mode = POWER_MODE_CORE_EXTIDLE;
	else {
		if (state == &dev->states[1])
			mode = POWER_MODE_APPS_SLEEP;
		if (state == &dev->states[2])
			mode = POWER_MODE_APPS_IDLE;
		if (state == &dev->states[3])
			mode = POWER_MODE_CHIP_SLEEP;

		/*
		 * consider support wakelock
		 * if using qos to contrsaint, block in place where required
		 */
		if (has_wake_lock(WAKE_LOCK_IDLE))
			mode = POWER_MODE_CORE_EXTIDLE;
	}

	mmp2_pm_enter_lowpower_mode(mode);

	if ((mode == POWER_MODE_CHIP_SLEEP)
			|| (mode == POWER_MODE_APPS_SLEEP)) {

		awucrm = __raw_readl(MPMU_AWUCRM);
		apcr = __raw_readl(MPMU_APCR);

		/* enable all wake-up ports */
		__raw_writel(awucrm | PMUM_RTC_ALARM | PMUM_WAKEUP7
				| PMUM_WAKEUP4 | PMUM_WAKEUP2
				| PMUM_AP_ASYNC_INT | PMUM_AP_FULL_IDLE
				| PMUM_AP2_TIMER_3 | PMUM_AP2_TIMER_2
				| PMUM_AP2_TIMER_1 | PMUM_AP1_TIMER_3
				| PMUM_AP1_TIMER_2 | PMUM_AP1_TIMER_1,
				MPMU_AWUCRM);
		__raw_writel(apcr & 0xff087fff, MPMU_APCR);
	}

	cpu_do_idle();

	if ((mode == POWER_MODE_CHIP_SLEEP)
			|| (mode == POWER_MODE_APPS_SLEEP)) {
		/* restore wake up sources settings */
		__raw_writel(awucrm, MPMU_AWUCRM);
		__raw_writel(apcr, MPMU_APCR);
	}

	do_gettimeofday(&after);
	local_irq_enable();
	idle_time = (after.tv_sec - before.tv_sec) * USEC_PER_SEC +
			(after.tv_usec - before.tv_usec);
	return idle_time;
}

/* Initialize CPU idle by registering the idle states */
static int mmp2_init_cpuidle(void)
{
	struct cpuidle_device *device;

	cpuidle_register_driver(&mmp2_idle_driver);

	device = &per_cpu(mmp2_cpuidle_device, smp_processor_id());
	device->state_count = MMP2_MAX_STATES;

	/* core_extidle */
	device->states[0].enter = mmp2_enter_idle;
	device->states[0].exit_latency = EXIT_LATENCY_CORE_EXTIDLE;
	device->states[0].target_residency = 10000;
	device->states[0].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[0].name, "core_extidle");
	strcpy(device->states[0].desc,
			"core_extidle: core clk gated externally");

	/* apps_idle */
	device->states[1].enter = mmp2_enter_idle;
	device->states[1].exit_latency = EXIT_LATENCY_APPS_IDLE;
	device->states[1].target_residency = 15000;
	device->states[1].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[1].name, "apps_idle");
	strcpy(device->states[1].desc, "apps_idle: axi, ddr off");

	/* apps_sleep*/
	device->states[2].enter = mmp2_enter_idle;
	device->states[2].exit_latency = EXIT_LATENCY_APPS_SLEEP;
	device->states[2].target_residency = 20000;
	device->states[2].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[2].name, "apps_sleep");
	strcpy(device->states[2].desc, "apps_sleep: axi, apb, ddr off");

	/* chip_sleep */
	device->states[3].enter = mmp2_enter_idle;
	device->states[3].exit_latency = EXIT_LATENCY_CHIP_SLEEP;
	device->states[3].target_residency = 25000;
	device->states[3].flags = CPUIDLE_FLAG_TIME_VALID;
	strcpy(device->states[3].name, "chip_sleep");
	strcpy(device->states[3].desc,
			"chip_sleep: chip sleep, axi, apb, ddr off");

	if (cpuidle_register_device(device)) {
		printk(KERN_ERR "mmp2_init_cpuidle: Failed registering\n");
		return -EIO;
	}
	return 0;
}

device_initcall(mmp2_init_cpuidle);
