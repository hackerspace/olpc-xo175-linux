/*
 * include/linux/platform_data/pxa_sdhci.h
 *
 * Copyright 2010 Marvell
 *	Zhangfei Gao <zhangfei.gao@marvell.com>
 *
 * PXA Platform - SDHCI platform data definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PXA_SDHCI_H_
#define _PXA_SDHCI_H_

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

/* pxa specific flag */
/* Require clock free running */
#define PXA_FLAG_ENABLE_CLOCK_GATING (1<<0)
/* card always wired to host, like on-chip emmc */
#define PXA_FLAG_CARD_PERMANENT	(1<<1)
/* Board design supports 8-bit data on SD/SDIO BUS */
#define PXA_FLAG_SD_8_BIT_CAPABLE_SLOT (1<<2)
/* controller always active during suspend */
#define PXA_FLAG_ACITVE_IN_SUSPEND (1<<4)

/*
 * struct pxa_sdhci_platdata() - Platform device data for PXA SDHCI
 * @flags: flags for platform requirement
 * @clk_delay_cycles:
 *	mmp2: each step is roughly 100ps, 5bits width
 *	pxa910: each step is 1ns, 4bits width
 * @clk_delay_sel: select clk_delay, used on pxa910
 *	0: choose feedback clk
 *	1: choose feedback clk + delay value
 *	2: choose internal clk
 * @clk_delay_enable: enable clk_delay or not, used on pxa910
 * @ext_cd_gpio: gpio pin used for external CD line
 * @ext_cd_gpio_invert: invert values for external CD gpio line
 * @max_speed: the maximum speed supported
 * @host_caps: Standard MMC host capabilities bit field.
 * @quirks: quirks of platfrom
 * @pm_caps: pm_caps of platfrom
 * @signal_1v8: signaling change to 1.8V
 * @lp_switch(): slot operations needed while going in/out low-power mode
 * @idle_lock: wake lock for idle
 */
struct sdhci_pxa_platdata {
	unsigned int	flags;
	unsigned int	clk_delay_cycles;
	unsigned int	clk_delay_sel;
	bool		clk_delay_enable;
	unsigned int	ext_cd_gpio;
	bool		ext_cd_gpio_invert;
	unsigned int	max_speed;
	unsigned int	host_caps;
	unsigned int	quirks;
	unsigned int	pm_caps;
	void	(*signal_1v8)(int set);
	int	(*lp_switch)(unsigned int on, int with_card);
#ifdef CONFIG_WAKELOCK
	struct wake_lock	idle_lock;
#endif
#ifdef CONFIG_SD8XXX_RFKILL
	/*for sd8688-rfkill device*/
	struct mmc_host **pmmc;
#endif
};

struct sdhci_pxa {
	u8	clk_enable;
	u8	power_mode;

	struct sdhci_pxa_platdata       *pdata;
};
#ifdef CONFIG_CPU_PXA910
int mmc1_idle_switch(u32 on);
#endif
#endif /* _PXA_SDHCI_H_ */
