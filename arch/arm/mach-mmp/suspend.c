#undef DEBUG
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/sched.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/cpuidle.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/pm_qos_params.h>

#include <asm/proc-fns.h>
#include <mach/hardware.h>
#include <mach/addr-map.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/mmp3_pm.h>
#include <mach/system.h>

static unsigned long pm_state;
static struct pm_qos_request_list cpu_max_req;
static struct pm_qos_request_list cpu_dis_req;
static struct pm_qos_request_list cpu_max_num_req;
static struct pm_qos_request_list qos_ddrfreq_min;

int mmp3_set_wake(struct irq_data *data, unsigned int on)
{
	unsigned int irq = data->irq;
	struct irq_desc *desc = irq_to_desc(irq);

	if (unlikely(irq >= nr_irqs)) {
		pr_err("IRQ nubmers are out of boundary!\n");
		return -EINVAL;
	}

	if (on) {
		if (desc->action)
			desc->action->flags |= IRQF_NO_SUSPEND;
	} else {
		if (desc->action)
			desc->action->flags &= ~IRQF_NO_SUSPEND;
	}

	/*
	 * TODO: Setup wakeup source. Will do it when wakeup is
	 * ready in hardware side.
	 */

	return 0;
}
extern void mk2_suspend_enter(void);
extern void mk2_suspend_finish(void);

static int mmp3_pm_enter(suspend_state_t state)
{
	/*
	 * TODO: If any wakeup source uses thread to handle its interrupt,
	 * make sure the interrupt is enabled here.
	 */
	mk2_suspend_enter();	
	mmp3_pm_enter_d2();
	mk2_suspend_finish();
	return 0;
}

/*
 * Called after processes are frozen, but before we shut down devices.
 */
static int mmp3_pm_prepare(void)
{
	pm_qos_update_request(&cpu_max_req, 800);
	pm_qos_update_request(&cpu_dis_req, 1);
	pm_qos_update_request(&cpu_max_num_req, 1);
	pm_qos_update_request(&qos_ddrfreq_min, 800000);
	return 0;
}

/*
 * Called after devices are re-setup, but before processes are thawed.
 */
static void mmp3_pm_finish(void)
{
	pm_qos_update_request(&qos_ddrfreq_min, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&cpu_max_num_req, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&cpu_max_req, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&cpu_dis_req, PM_QOS_DEFAULT_VALUE);
	pm_state = PM_SUSPEND_ON;
}

static void mmp3_pm_wake(void)
{
	return;
}

static int mmp3_pm_valid(suspend_state_t state)
{
	int ret = 1;

	if (state == PM_SUSPEND_STANDBY) {
		pm_state = PM_SUSPEND_STANDBY;
	} else if (state == PM_SUSPEND_MEM) {
		pm_state = PM_SUSPEND_MEM;
	} else {
		ret = 0;
	}
	return ret;
}

/*
 * Set to PM_DISK_FIRMWARE so we can quickly veto suspend-to-disk.
 */
static struct platform_suspend_ops mmp3_pm_ops = {
	.valid		= mmp3_pm_valid,
	.prepare	= mmp3_pm_prepare,
	.enter		= mmp3_pm_enter,
	.finish		= mmp3_pm_finish,
	.wake		= mmp3_pm_wake,
};

static int __init mmp3_suspend_init(void)
{
	suspend_set_ops(&mmp3_pm_ops);
	pm_qos_add_request(&cpu_max_req, PM_QOS_CPUFREQ_MAX, PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&cpu_dis_req, PM_QOS_CPUFREQ_DISABLE, PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&cpu_max_num_req, PM_QOS_MAX_ONLINE_CPUS, PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&qos_ddrfreq_min, PM_QOS_DDR_DEVFREQ_MIN, PM_QOS_DEFAULT_VALUE);

	return 0;
}

late_initcall(mmp3_suspend_init);

