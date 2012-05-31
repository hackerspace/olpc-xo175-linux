/*
 * devfreq: Generic Dynamic Voltage and Frequency Scaling (DVFS) Framework
 *	    for Non-CPU Devices.
 *
 * Copyright (C) 2012 Marvell
 *	Xiaoguang Chen <chenxg@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __PXA_DEVFREQ_H__
#define __PXA_DEVFREQ_H__

#include <linux/devfreq.h>

#define INIT_FREQ_TABLE(_index, _freq)	\
	{				\
		.index = _index,	\
		.frequency = _freq,	\
	}

struct devfreq_platform_data {
	const char *clk_name;
	struct devfreq_frequency_table *freq_table;
	int (*setup_freq_table) (struct devfreq *devfreq);
};

#endif
