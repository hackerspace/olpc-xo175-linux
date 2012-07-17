/*
 * linux/arch/arm/mach-mmp/pxa988_lowpower.c
 *
 * Author:	Raul Xiong <xjian@marvell.com>
 * Copyright:	(C) 2012 Marvell International Ltd.
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
#include <linux/spinlock.h>
#include <asm/mach/map.h>
#include <asm/suspend.h>
#include <asm/cacheflush.h>
#include <asm/io.h>
#include <asm/smp_scu.h>
#include <asm/hardware/cache-l2x0.h>
#include <mach/pxa988_lowpower.h>
#include <mach/regs-apmu.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-icu.h>
#include <mach/scu.h>
#include <mach/reset-pxa988.h>
#include <mach/gpio-edge.h>
#include "common.h"

/*
 * The topology of the reserved data is as following.
 * Each core will use 4 bytes to save the flags.
 * The base address is pointed by pm_reserve_pa
 *
 * Note: We can support more than 2 cores here.
 * current we define MAX_CPU_NUM as 2 for PXA988.
 *
 * +---------------------------------------------------------------------+
 * |CPU0:| hotplug | l2_shutdow | LPM[MAX_NUM_LPM - 1] | LPM[1] | LPM[0] |
 * +---------------------------------------------------------------------+
 * |CPU1:| hotplug | l2_shutdow | LPM[MAX_NUM_LPM - 1] | LPM[1] | LPM[0] |
 * +---------------------------------------------------------------------+
 * |     spin_lock                                                       |
 * +---------------------------------------------------------------------+
 *
 * There are totally seven low power modes defined for PXA988.
 * Please refer mach/pxa988_lowpower.h.
 *
 * 1. PXA988_LPM_C1: POWER_MODE_CORE_EXTIDLE
 * 2. PXA988_LPM_C2: POWER_MODE_CORE_POWERDOWN with L1 shutdown, L2 retentive
 * 3. PXA988_LPM_D1P: POWER_MODE_APPS_IDLE with L2 retentive
 * 4. PXA988_LPM_D1: POWER_MODE_SYS_SLEEP with L2 retentive
 * 5. PXA988_LPM_D2: POWER_MODE_UDR_VCTCXO with L2 retentive
 * 6. PXA988_LPM_D2_UDR: POWER_MODE_UDR with L2 shutdown
 */


#ifdef CONFIG_SMP
static char *coherent_buf;
static u32 num_cpus;
static u32 *enter_lpm_p;
static spinlock_t *lpm_lock_p;
#endif

static DEFINE_SPINLOCK(pmu_lock);
static unsigned long flags;

static const u32 APMU_CORE_IDLE_CFG[2] = {
	PMU_CA9_CORE0_IDLE_CFG, PMU_CA9_CORE1_IDLE_CFG };
static const u32 APMU_MP_IDLE_CFG[2] = {
	PMU_CA9MP_IDLE_CFG0, PMU_CA9MP_IDLE_CFG1 };
static const u32 ICU_A9_GBL_INT_MSK[2] = {
	PXA988_ICU_A9C0_GBL_INT_MSK, PXA988_ICU_A9C1_GBL_INT_MSK };

/*
 * To avoid multi-cores are accessing the same PMU register,
 * any functions MUST call pmu_register_lock before accessing
 * the PMU register and pmu_register_unlock after it.
 */
void pmu_register_lock()
{
	spin_lock_irqsave(&pmu_lock, flags);
}

void pmu_register_unlock()
{
	spin_unlock_irqrestore(&pmu_lock, flags);
}

static int pxa988_finish_suspend(unsigned long power_state)
{
	if (power_state > POWER_MODE_CORE_EXTIDLE) {
#ifdef CONFIG_SMP
		void __iomem *scu_addr;
		scu_addr = pxa_scu_base_addr();
		/*
		 * Set SCU CPU power status register to power off.
		 * This must be done after cache is flushed.
		 * Remember to set it back to normal mode in reset
		 * chunk before cache is re-enabled.
		 */
		scu_power_mode(scu_addr, SCU_PM_POWEROFF);
#endif
#ifdef CONFIG_CACHE_L2X0
		/* pl310_suspend includes flush_cache_all */
		pl310_suspend();
#endif
	}
	cpu_do_idle();

	panic("Core didn't get powered down! Should never reach here.\n");

	return 0;
}

static void pxa988_lowpower_config(u32 cpu,
			u32 power_state, u32 lowpower_enable)
{
	u32 core_idle_cfg, mp_idle_cfg, apcr, mc_slp_type;

	pmu_register_lock();
	core_idle_cfg = __raw_readl(APMU_CORE_IDLE_CFG[cpu]);
	mp_idle_cfg = __raw_readl(APMU_MP_IDLE_CFG[cpu]);
	apcr = __raw_readl(MPMU_APCR);
	mc_slp_type = __raw_readl(APMU_MC_HW_SLP_TYPE);

	if (lowpower_enable) {
		switch (power_state) {
		case POWER_MODE_UDR:
			mp_idle_cfg |= PMUA_MP_L2_SRAM_POWER_DOWN;
			apcr |= PMUM_VCTCXOSD;
			/* fall through */
		case POWER_MODE_UDR_VCTCXO:
			apcr |= PMUM_STBYEN;
			/* fall through */
		case POWER_MODE_SYS_SLEEP:
			apcr |= PMUM_APBSD;
			/* enable gpio edge for the modes need wakeup source */
			mmp_gpio_edge_enable();
			/* fall through */
		case POWER_MODE_APPS_SLEEP:
			apcr |= PMUM_SLPEN;
			/* fall through */
		case POWER_MODE_APPS_IDLE:
			/*
			 * FIXME: This is for PXA988 Z0, for A0 here we only
			 * need to vote PMUM_AXISD.
			 * Note that on Z0 we have to modify APMU_MC_HW_SLP_TYPE
			 * to change ddr sleep type from self-refresh to active
			 * power down. This makes ddr accessable in AP_IDLE.
			 * This is supposed to be fixed on A0.
			 */
			apcr |= PMUM_AXISD;
			apcr |= PMUM_DDRCORSD;
			/* fall through */
		case POWER_MODE_CORE_POWERDOWN:
			core_idle_cfg |= PMUA_CORE_POWER_DOWN;
			core_idle_cfg |= PMUA_CORE_L1_SRAM_POWER_DOWN;
			mp_idle_cfg |= PMUA_MP_SCU_SRAM_POWER_DOWN;
			mp_idle_cfg |= PMUA_MP_POWER_DOWN;
			/* fall through */
		case POWER_MODE_CORE_EXTIDLE:
			core_idle_cfg |= PMUA_CORE_IDLE;
			mp_idle_cfg |= PMUA_MP_IDLE;
			/* fall through */
		case POWER_MODE_CORE_INTIDLE:
			break;
		default:
			WARN(1, "Invalid power state!\n");
		}

		if (power_state == POWER_MODE_APPS_IDLE) {
			mc_slp_type &= ~0x7;
			mc_slp_type |= 0x1;
		}
	} else {
		core_idle_cfg &= ~(PMUA_CORE_IDLE | PMUA_CORE_POWER_DOWN |
				PMUA_CORE_L1_SRAM_POWER_DOWN);
		mp_idle_cfg &= ~(PMUA_MP_IDLE | PMUA_MP_POWER_DOWN |
				PMUA_MP_L2_SRAM_POWER_DOWN |
				PMUA_MP_SCU_SRAM_POWER_DOWN);
		apcr &= ~(PMUM_DDRCORSD | PMUM_APBSD | PMUM_AXISD |
			PMUM_VCTCXOSD | PMUM_STBYEN | PMUM_SLPEN);
		mc_slp_type &= ~0x7;
		/* disable the gpio edge for cpu active states */
		mmp_gpio_edge_disable();
	}

	/* set DSPSD, DTCMSD, BBSD, MSASLPEN */
	apcr |= PMUM_DSPSD | PMUM_DTCMSD | PMUM_BBSD | PMUM_MSASLPEN;

	/*
	 * FIXME: PXA920 was always setting SLEPEN bit but it seems no need
	 * to do that according to the power measurement.
	 */
	/* apcr |= PMUM_SLPEN; */

	__raw_writel(core_idle_cfg, APMU_CORE_IDLE_CFG[cpu]);
	__raw_writel(mp_idle_cfg, APMU_MP_IDLE_CFG[cpu]);
	__raw_writel(apcr, MPMU_APCR);
	__raw_writel(mc_slp_type, APMU_MC_HW_SLP_TYPE);
	pmu_register_unlock();
}

static void pxa988_gic_global_mask(u32 cpu, u32 mask)
{
	u32 core_idle_cfg;

	core_idle_cfg = __raw_readl(APMU_CORE_IDLE_CFG[cpu]);

	if (mask) {
		core_idle_cfg |= PMUA_GIC_IRQ_GLOBAL_MASK;
		core_idle_cfg |= PMUA_GIC_FIQ_GLOBAL_MASK;
	} else {
		core_idle_cfg &= ~(PMUA_GIC_IRQ_GLOBAL_MASK |
					PMUA_GIC_FIQ_GLOBAL_MASK);
	}
	__raw_writel(core_idle_cfg, APMU_CORE_IDLE_CFG[cpu]);
}

static void pxa988_icu_global_mask(u32 cpu, u32 mask)
{
	u32 icu_msk;

	icu_msk = __raw_readl(ICU_A9_GBL_INT_MSK[cpu]);

	if (mask) {
		icu_msk |= ICU_MASK_FIQ;
		icu_msk |= ICU_MASK_IRQ;
	} else {
		icu_msk &= ~(ICU_MASK_FIQ | ICU_MASK_IRQ);
	}
	__raw_writel(icu_msk, ICU_A9_GBL_INT_MSK[cpu]);
}

/* These states are used as idle replacement as well as suspend/hotplug */
struct pxa988_lowpower_data pxa988_lpm_data[] = {
	[PXA988_LPM_C1] = {
		.power_state = POWER_MODE_CORE_EXTIDLE,
		.l2_shutdown = 0,
		.valid = 1,
	},
	[PXA988_LPM_C2] = {
		.power_state = POWER_MODE_CORE_POWERDOWN,
		.l2_shutdown = 0,
		.valid = 1,
	},
	[PXA988_LPM_D1P] = {
		.power_state = POWER_MODE_APPS_IDLE,
		.l2_shutdown = 0,
		.valid = 1,
	},
	[PXA988_LPM_D1] = {
		.power_state = POWER_MODE_SYS_SLEEP,
		.l2_shutdown = 0,
		.valid = 1,
	},
	[PXA988_LPM_D2] = {
		.power_state = POWER_MODE_SYS_SLEEP,
		.l2_shutdown = 0,
		.valid = 1,
	},
	[PXA988_LPM_D2_UDR] = {
		.power_state = POWER_MODE_UDR,
		.l2_shutdown = 1,
		.valid = 1,
	},
	/* must always be the last one! */
	[PXA988_MAX_LPM_INDEX] = {
		.power_state = -1,
		.l2_shutdown = 0,
		.valid = 0,
	},
};

#ifdef CONFIG_PM
static void pxa988_enter_c1(u32 cpu)
{
	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[PXA988_LPM_C1].power_state, 1);
	cpu_do_idle();
	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[PXA988_LPM_C1].power_state, 0);
}

static void pxa988_pre_enter_lpm(u32 cpu, u32 power_mode)
{
	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[power_mode].power_state, 1);

	/* Mask GIC global interrupt */
	pxa988_gic_global_mask(cpu, 1);
	/* Mask ICU global interrupt */
	pxa988_icu_global_mask(cpu, 1);
}

static void pxa988_post_enter_lpm(u32 cpu, u32 power_mode)
{
	/* Unmask GIC interrtup */
	pxa988_gic_global_mask(cpu, 0);
	/*
	 * FIXME: Do we need to mask ICU before cpu_cluster_pm_exit
	 * to avoid GIC ID31 interrupt?
	 */
	/* Mask ICU global interrupt */
	pxa988_icu_global_mask(cpu, 1);

	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[power_mode].power_state, 0);
}
/*
 * pxa988_enter_lowpower - the entry function of pxa988 low power mode
 *
 * Here we are assuming there are maximum 16 low power modes,
 * and the first LPM is C1, the second LPM is C2 (cpu power down)
 * The following LPMs are D-stauts.
 *
 * @cpu: the cpu id of the cpu that calls this function.
 * @power_mode: then low power mode it will enter
 *
 */
int pxa988_enter_lowpower(u32 cpu, u32 power_mode)
{
#ifdef CONFIG_SMP
	int i;
	int cpus_enter_lpm = 0xffffffff;
	int mp_shutdown = 1;
	int mp_restore = 1;
	int l2_shutdown = 1;
	int lpm_index = 0;
#endif

	if (power_mode == PXA988_LPM_C1) {
		/* For C1 the core won't be reset */
		pxa988_enter_c1(cpu);
		return 0;
	}

	/* At least we can enter C2 here */
	pxa988_pre_enter_lpm(cpu, PXA988_LPM_C2);

	cpu_pm_enter();

#ifdef CONFIG_SMP
	arch_spin_lock(&(&lpm_lock_p->rlock)->raw_lock);

	/* mask the LPM states we can enter */
	enter_lpm_p[cpu] |= (1 << (power_mode + 1)) - 1;
	if (pxa988_lpm_data[power_mode].l2_shutdown)
		set_bit(L2_SHUTDOWN, (void *)&enter_lpm_p[cpu]);
	for (i = 0; i < num_cpus; i++)
		cpus_enter_lpm &= enter_lpm_p[i];
	mp_shutdown = test_bit(PXA988_LPM_C2, (void *)&cpus_enter_lpm);
	if (mp_shutdown) {
		l2_shutdown = test_bit(L2_SHUTDOWN,
					(void *)&cpus_enter_lpm);
		if (l2_shutdown) {
#ifdef CONFIG_CACHE_L2X0
			pl310_suspend();
#endif
			outer_disable();
		}

		cpu_cluster_pm_enter();

		/*
		 * Here we assume when one LPM state is disabled,
		 * all shallower states are disabled
		 */
		lpm_index = find_first_zero_bit((void *)&cpus_enter_lpm,
						PXA988_MAX_LPM_INDEX) - 1;
		/* check LPM constraints  */
		while (pxa988_lpm_data[lpm_index].valid != 1)
			lpm_index--;
		if (lpm_index > PXA988_LPM_C2)
			pxa988_lowpower_config(cpu,
				pxa988_lpm_data[lpm_index].power_state, 1);
	}

	arch_spin_unlock(&(&lpm_lock_p->rlock)->raw_lock);
	cpu_suspend(pxa988_lpm_data[lpm_index].power_state,
			pxa988_finish_suspend);
#else
	if (pxa988_lpm_data[power_mode].l2_shutdown) {
#ifdef CONFIG_CACHE_L2X0
		pl310_suspend();
#endif
		outer_disable();
	}

	cpu_cluster_pm_enter();

	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[power_mode].power_state, 1);
	cpu_suspend(pxa988_lpm_data[power_mode].power_state,
			pxa988_finish_suspend);
#endif /* CONFIG_SMP */

#ifdef CONFIG_SMP
	/* here we exit from LPM */
	arch_spin_lock(&(&lpm_lock_p->rlock)->raw_lock);

	/*
	 * clear all the software flag of LPM
	 * L2_shutdown flag is cleared by reset chunk
	 */
	enter_lpm_p[cpu] &= ~((1 << (power_mode + 1)) - 1);
	cpus_enter_lpm = 0xffffffff;
	for (i = 0; i < num_cpus; i++)
		if (i != cpu)
			cpus_enter_lpm &= enter_lpm_p[i];
	mp_restore = test_bit(PXA988_LPM_C2, (void *)&cpus_enter_lpm);
	if (mp_restore)
		cpu_cluster_pm_exit();

	arch_spin_unlock(&(&lpm_lock_p->rlock)->raw_lock);
#else
	cpu_cluster_pm_exit();
#endif /* CONFIG_SMP */

	pxa988_post_enter_lpm(cpu, power_mode);

	cpu_pm_exit();

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_HOTPLUG_CPU
/*
 * Allows POWER_MODE_UDR with L2 shutdown for CPU hotplug.
 * Actually the hogpluged CPU will enter C2 but it will allow
 * POWER_MODE_UDR with L2 shutdown since the hotpluged CPU
 * should never blocks other CPUs enter the deepest LPM.
 */
void pxa988_hotplug_enter(u32 cpu, u32 power_mode)
{
	pxa988_lowpower_config(cpu,
			pxa988_lpm_data[PXA988_LPM_C2].power_state, 1);

	/* Mask GIC global interrupt */
	pxa988_gic_global_mask(cpu, 1);
	/* Mask ICU global interrupt */
	pxa988_icu_global_mask(cpu, 1);

	/*
	 * For CPU hotplug, we don't need cpu_suspend help functions
	 * but still need to mask LPM bits as the deepest LPM.
	 * We are assuming the hotplug CPU will NEVER be the last CPU
	 * enter C2 since in platform_cpu_kill we ensure that.
	 */
	set_bit(LPM4HOTPLUG, (void *)&enter_lpm_p[cpu]);
	if (pxa988_lpm_data[power_mode].l2_shutdown)
		set_bit(L2_SHUTDOWN, (void *)&enter_lpm_p[cpu]);
	enter_lpm_p[cpu] |= (1 << (power_mode + 1)) - 1;

	pxa988_finish_suspend(pxa988_lpm_data[power_mode].power_state);
}
#endif

#ifdef CONFIG_SUSPEND
void pxa988_pm_suspend(u32 cpu, u32 power_mode)
{
	pxa988_pre_enter_lpm(cpu, power_mode);

	if (pxa988_lpm_data[power_mode].l2_shutdown) {
#ifdef CONFIG_CACHE_L2X0
		pl310_suspend();
#endif
		outer_disable();
	}

	cpu_suspend(pxa988_lpm_data[power_mode].power_state,
			pxa988_finish_suspend);

	pxa988_post_enter_lpm(cpu, power_mode);
}
#endif

static int __init pxa988_lowpower_init(void)
{
#ifdef CONFIG_SMP
	int i;
	void __iomem *scu_addr;
	num_cpus = num_online_cpus();
	coherent_buf = __arm_ioremap(pm_reserve_pa, PAGE_SIZE, MT_MEMORY_SO);
	if (coherent_buf == NULL)
		panic("%s: failed to remap memory for pm\n", __func__);
	memset(coherent_buf, 0x0, PAGE_SIZE);

	enter_lpm_p = (u32 *)coherent_buf;
	for (i = 0; i < num_cpus; i++)
		enter_lpm_p[i] = 0;

	lpm_lock_p = (spinlock_t *)(&enter_lpm_p[num_cpus]);
	spin_lock_init(lpm_lock_p);

	/*
	 * Set PL310 power ctrl register to set standby_mode_en bit
	 * and dynamic_clk_gating_en bit
	 * it is done in cache-l2x0.c : l2x0_init now
	 */
	scu_addr = pxa_scu_base_addr();
	/* Set SCU control register standby enable bit */
	__raw_writel(__raw_readl(scu_addr + SCU_CTRL) | (1 << 5),
			scu_addr + SCU_CTRL);
#else
	/* In SMP scenario, it will be called in platsmp.c */
	pxa_cpu_reset_handler_init();
#endif
	return 0;
}

postcore_initcall(pxa988_lowpower_init);
