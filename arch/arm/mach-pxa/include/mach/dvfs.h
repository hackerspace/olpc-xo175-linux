/*
 *  linux/arch/arm/mach-pxa/include/mach/dvfs.h
 *
 *  Author: Xiaoguang Chen chenxg@marvell.com
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#ifndef _MACH_PXA_DVFS_H_
#define _MACH_PXA_DVFS_H_
#include <plat/dvfs.h>

#define DVFS_FREQUENCY_NOTIFIER		0
#define DVFS_FREQ_PRECHANGE		0
#define DVFS_FREQ_POSTCHANGE		1

#define VOL_LEVL0	0
#define VOL_LEVL1	1
#define VOL_LEVL2	2
#define VOL_LEVL3_0	3
#define VOL_LEVL3_1	4

#define VOL_MEM_LOW	0x30
#define VOL_MEM_HIGH	0x34

#define INIT_VOL_TABLE(_freq, _volt)	\
	{				\
		.freq = _freq,		\
		.millivolts = _volt,	\
	}

struct dvfs_freqs {
	struct dvfs *dvfs;
	unsigned int old;	/*old frequency */
	unsigned int new;	/*new frequency */
};

extern int dvfs_notifier_frequency(struct dvfs_freqs *freqs,
				   unsigned int state);
extern int dvfs_register_notifier(struct notifier_block *nb, unsigned int list);
extern int dvfs_unregister_notifier(struct notifier_block *nb,
				    unsigned int list);
extern int set_dvfs_rate(struct dvfs *d, unsigned long rate);
extern void pxa978_set_voltage_level(unsigned int level);
extern struct sysdev_class cpu_sysdev_class;
extern int is_wkr_ddr533(void);

#endif
