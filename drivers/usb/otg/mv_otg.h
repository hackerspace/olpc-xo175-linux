/*
 * Copyright (C) 2011 Marvell International Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef	__MV_USB_OTG_CONTROLLER__
#define	__MV_USB_OTG_CONTROLLER__

#include <linux/types.h>

/* Timer's interval, unit 10ms */
#define T_A_WAIT_VRISE		100
#define T_A_WAIT_BCON		2000
#define T_A_AIDL_BDIS		100
#define T_A_BIDL_ADIS		20
#define T_B_ASE0_BRST		400
#define T_B_SE0_SRP		300
#define T_B_SRP_FAIL		2000
#define T_B_DATA_PLS		10
#define T_B_SRP_INIT		100
#define T_A_SRP_RSPNS		10
#define T_A_DRV_RSM		5

enum otg_function {
	OTG_B_DEVICE = 0,
	OTG_A_DEVICE
};

enum mv_otg_timer {
	A_WAIT_BCON_TIMER = 0,
	OTG_TIMER_NUM
};

/* PXA OTG state machine */
struct mv_otg_ctrl {
	/* internal variables*/
	u8	a_set_b_hnp_en;	   /* A-Device set b_hnp_en */
	u8	b_srp_done;
	u8	b_hnp_en;

	/* OTG inputs*/
	u8	a_bus_drop;
	u8	a_bus_req;
	u8	a_clr_err;
	u8	a_bus_resume;
	u8	a_bus_suspend;
	u8	a_conn;
	u8	a_sess_vld;
	u8	a_srp_det;
	u8	a_vbus_vld;
	u8	b_bus_req;	/* B-Device Require Bus */
	u8	b_bus_resume;
	u8	b_bus_suspend;
	u8	b_conn;
	u8	b_se0_srp;
	u8	b_sess_end;
	u8	b_sess_vld;
	u8	id;
	u8	a_suspend_req;

	/*Timer event*/
	u8	a_aidl_bdis_timeout;
	u8	b_ase0_brst_timeout;
	u8	a_bidl_adis_timeout;
	u8	a_wait_bcon_timeout;

	struct timer_list	timer[OTG_TIMER_NUM];
};

struct mv_otg {
	struct otg_transceiver		otg;
	struct mv_otg_ctrl		otg_ctrl;

	/* base address */
	struct mv_cap_regs __iomem	*cap_regs;
	struct mv_op_regs __iomem	*op_regs;
	unsigned int			phy_regs;

	struct platform_device		*dev;
	int				irq;

	u32				irq_status;
	u32				irq_en;

	struct delayed_work		work;
	struct workqueue_struct		*qwork;

	spinlock_t			wq_lock;

	struct mv_usb_platform_data	*pdata;

	unsigned int			active;
	unsigned int			clock_gating;
	unsigned int			clknum;
	struct clk			*clk[0];
};

#endif
