/*
 * linux/arch/arm/mach-mmp/pm-pxa988.c
 *
 * Author:      Hong Feng <hongfeng@marvell.com>
 * Copyright:   (C) 2012 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/cpu_pm.h>
#include <linux/suspend.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm80x.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/smp_scu.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/hardware/gic.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-icu.h>
#include <mach/pxa988_lowpower.h>
#include <mach/regs-ciu.h>
#include "common.h"

static struct clk *vctcxo_clk;
static struct pxa988_peripheral_config_ops *wakeup_ops;
static struct wake_lock system_wakeup;
static int pmic_wakeup_detect;

int pxa988_set_wake(struct irq_data *data, unsigned int on)
{
	int irq = data->irq;
	struct irq_desc *desc = irq_to_desc(data->irq);
	uint32_t awucrm = 0, apcr = 0;

	if (on) {
		if (desc->action)
			desc->action->flags |= IRQF_NO_SUSPEND;
	} else {
		if (desc->action)
			desc->action->flags &= ~IRQF_NO_SUSPEND;
	}

	/* setting wakeup sources */
	switch (irq) {
	/* wakeup line 2 */
	case IRQ_PXA988_GPIO_AP:
		awucrm = PMUM_WAKEUP2;
		apcr |= PMUM_SLPWP2;
		break;
	/* wakeup line 3 */
	case IRQ_PXA988_KEYPAD:
		awucrm = PMUM_WAKEUP3 | PMUM_KEYPRESS | PMUM_TRACKBALL |
				PMUM_NEWROTARY;
		apcr |= PMUM_SLPWP3;
		break;
	/* wakeup line 4 */
	case IRQ_PXA988_AP_TIMER1:
		awucrm = PMUM_WAKEUP4 | PMUM_AP1_TIMER_1;
		apcr |= PMUM_SLPWP4;
		break;
	case IRQ_PXA988_AP_TIMER2_3:
		awucrm = PMUM_WAKEUP4 | PMUM_AP1_TIMER_2 |
				PMUM_AP1_TIMER_3;
		apcr |= PMUM_SLPWP4;
		break;
	case IRQ_PXA988_AP2_TIMER1:
		awucrm = PMUM_WAKEUP4 | PMUM_AP2_TIMER_1;
		apcr |= PMUM_SLPWP4;
		break;
	case IRQ_PXA988_AP2_TIMER2_3:
		awucrm = PMUM_WAKEUP4 | PMUM_AP2_TIMER_2 |
				PMUM_AP2_TIMER_3;
		apcr |= PMUM_SLPWP4;
		break;
	case IRQ_PXA988_RTC_ALARM:
		awucrm = PMUM_WAKEUP4 | PMUM_RTC_ALARM;
		apcr |= PMUM_SLPWP4;
		break;
	/* wakeup line 5 */
	case IRQ_PXA988_USB1:
		awucrm = PMUM_WAKEUP5;
		apcr |= PMUM_SLPWP5;
		break;
	/* wakeup line 6 */
	case IRQ_PXA988_MMC:
		awucrm = PMUM_WAKEUP6 | PMUM_SDH;
		apcr |= PMUM_SLPWP6;
		break;
	case IRQ_PXA988_HIFI_DMA:
		awucrm = PMUM_WAKEUP6 | PMUM_SQU;
		apcr |= PMUM_SLPWP6;
		break;
	/* wakeup line 7 */
	case IRQ_PXA988_PMIC:
		awucrm = PMUM_WAKEUP7;
		apcr |= PMUM_SLPWP7;
		break;
	default:
		if (irq >= IRQ_GPIO_START && irq < IRQ_BOARD_START) {
			awucrm = PMUM_WAKEUP2;
			apcr |= PMUM_SLPWP2;
		} else if (irq < IRQ_BOARD_START + PM800_MAX_IRQ +
				PM805_MAX_IRQ) {
			awucrm = PMUM_WAKEUP7;
			apcr |= PMUM_SLPWP7;
		} else
			pr_err("Error: no defined wake up source irq: %d\n",
					irq);
	}
	/* add lock, MPMU_APCR may access through other cpu on SMP system */
	pmu_register_lock();
	if (on) {
		if (awucrm) {
			awucrm |= __raw_readl(MPMU_AWUCRM);
			__raw_writel(awucrm, MPMU_AWUCRM);
		}
		if (apcr) {
			apcr = ~apcr & __raw_readl(MPMU_APCR);
			__raw_writel(apcr, MPMU_APCR);
		}
	} else {
		if (awucrm) {
			awucrm = ~awucrm & __raw_readl(MPMU_AWUCRM);
			__raw_writel(awucrm, MPMU_AWUCRM);
		}
		if (apcr) {
			apcr |= __raw_readl(MPMU_APCR);
			__raw_writel(apcr, MPMU_APCR);
		}
	}
	pmu_register_unlock();
	return 0;
}


int pxa988_power_config_register(struct pxa988_peripheral_config_ops *ops)
{
	wakeup_ops = ops;
	return 0;
}

static void peripheral_suspend_config(void)
{
	if (wakeup_ops && wakeup_ops->pin_lpm_config)
		wakeup_ops->pin_lpm_config();
}

static void peripheral_resume_config(void)
{
	if (wakeup_ops && wakeup_ops->pin_lpm_restore)
		wakeup_ops->pin_lpm_restore();
}

static int pxa988_pm_valid(suspend_state_t state)
{
	return ((state == PM_SUSPEND_STANDBY) || (state == PM_SUSPEND_MEM));
}

/* Called after devices suspend, before noirq devices suspend */
static int pxa988_pm_prepare(void)
{
	/* TODO:set to the lowest PP */
	return 0;
}

static int pxa988_pm_enter(suspend_state_t state)
{
	uint32_t awucrs = 0;
	uint32_t awucrs_pmic_wakeup = PMUM_WAKEUP7;
	uint32_t reg = 0;
	/*
	 * pmic thread not completed, exit;
	 * otherwise system can't be waked up
	 */
	reg = __raw_readl(ICU_INT_CONF(IRQ_PXA988_PMIC - IRQ_PXA988_START));
	if ((reg & 0x3) == 0)
		return -EAGAIN;

	clk_disable(vctcxo_clk);
	peripheral_suspend_config();

	pxa988_pm_suspend(0, PXA988_LPM_D2_UDR);

	peripheral_resume_config();
	clk_enable(vctcxo_clk);

	awucrs = __raw_readl(MPMU_AWUCRS);
	if (awucrs & awucrs_pmic_wakeup) {
		pr_info(" [%s]AWUCRS:%x PMIC WAKEUP DETECT\n",
				__func__, awucrs);
		pmic_wakeup_detect = 1;
	}
	return 0;
}

/* Called after noirq devices resume, before devices resume */
static void pxa988_pm_finish(void)
{
	/*TODO: restore PP after resume */
}

static void pxa988_pm_wake(void)
{
	if (pmic_wakeup_detect)
		wake_lock_timeout(&system_wakeup, HZ * 5);
	pmic_wakeup_detect = 0;
}

static const struct platform_suspend_ops pxa988_pm_ops = {
	.valid          = pxa988_pm_valid,
	.prepare        = pxa988_pm_prepare,
	.enter          = pxa988_pm_enter,
	.finish         = pxa988_pm_finish,
	.wake           = pxa988_pm_wake,
};

static int __init pxa988_pm_init(void)
{
	u32 awucrm = 0;

	suspend_set_ops(&pxa988_pm_ops);

	/* DE suggest:enable SQU MP3 playback sleep mode*/
	__raw_writel(__raw_readl(APMU_SQU_CLK_GATE_CTRL) | (1 << 30),
			APMU_SQU_CLK_GATE_CTRL);
	/* Save power:I2S clk from VCTCXO, not from 312M*/
	__raw_writel(__raw_readl(MPMU_FCCR) | (1 << 28), MPMU_FCCR);

	wake_lock_init(&system_wakeup, WAKE_LOCK_SUSPEND,
			"system_wakeup_detect");

	vctcxo_clk = clk_get(NULL, "VCTCXO");
	if (IS_ERR(vctcxo_clk)) {
		pr_err("unable to get VCTCXO\n");
		return PTR_ERR(vctcxo_clk);
	}
	clk_enable(vctcxo_clk);
	/*
	 * These two bits are used to solve the corner case that
	 * 1.	APMU enters sleep mode.
	 * 2.	MPMU is activated but not finished.
	 * 3.	An interrupt comes
	 * 4.	The interrupt is not a wake up source
	 * Keeping these two bits can assure it aborts from
	 * the intermediate state.
	 *
	 * Actually for suspend it won't happen since we disable
	 * non-wake up interrupts before WFI.
	 * For idle replacement we will encounter such issue
	 * since not all interrupts are wake up source.
	 * */
	awucrm |= PMUM_AP_ASYNC_INT;
	awucrm |= PMUM_AP_FULL_IDLE;
	__raw_writel(awucrm, MPMU_AWUCRM);
	/* hook wakeup callback */
	gic_arch_extn.irq_set_wake = pxa988_set_wake;

	return 0;
}
late_initcall(pxa988_pm_init);
