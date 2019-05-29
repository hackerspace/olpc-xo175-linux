/*
 * Marvell Vmeta Video Encoder and Decoder Engine
 *
 * Copyright (C) 2011 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _VMETA_H_
#define _VMETA_H_

#include <linux/uio_driver.h>
#include <linux/pm_qos_params.h>

struct vmeta_instance {
	void *reg_base;
	struct uio_info uio_info;
	struct timer_list irq_poll_timer;
	spinlock_t lock;
	unsigned long flags;
	struct clk *clk;
	struct clk *axi_clk;
	struct mutex mutex;
	int power_constraint;
	struct timer_list power_timer;
	int power_status;	/* 0-off 1-on */
	int vop;		/* vmeta operating point 0-min, 15-max */
	int vop_real;		/* used in dvfm constraint only */
	int clk_status;
	struct semaphore *sema;
	struct semaphore *priv_sema;
	struct vmeta_plat_data *plat_data;
	struct delayed_work unset_op_work;
	struct mutex op_mutex;
	struct pm_qos_request_list qos_cpufreq_min;
	struct pm_qos_request_list qos_ddrfreq_min;
	struct pm_qos_request_list qos_idle;
};

extern int vmeta_runtime_constraint(struct vmeta_instance *vi, int on);
extern int vmeta_init_constraint(struct vmeta_instance *vi);
extern int vmeta_clean_constraint(struct vmeta_instance *vi);
extern int vmeta_freq_change(struct vmeta_instance *vi, int step);
extern void vmeta_power_switch(unsigned int enable);

#endif
