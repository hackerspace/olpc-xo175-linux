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
struct vmeta_plat_pdata;

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
	int power_down_ms;
	int power_status;	/* 0-off 1-on */
	int vop;		/* vmeta operating point 0-min, 15-max */
	int vop_real;		/* used in dvfm constraint only */
	int clk_status;
	struct semaphore *sema;
	struct semaphore *priv_sema;
	struct vmeta_plat_data *plat_data;
};

struct vmeta_plat_data {
	int (*set_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*unset_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*clean_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*init_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	irqreturn_t (*bus_irq_handler)(int irq, void *dev_id);
	int axi_clk_available;
	int (*decrease_core_freq)(const struct vmeta_instance *vi,
					const int step);
	int (*increase_core_freq)(const struct vmeta_instance *vi,
					const int step);
	void (*disable_lpm)(int idx);
	void (*enable_lpm)(int idx);
	int (*update_vmeta_clk)(struct vmeta_instance *vi);
};
#endif
