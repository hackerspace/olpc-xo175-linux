/*
 * Copyright (C) 2011 Marvell International Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MV_PLATFORM_USB_H
#define __MV_PLATFORM_USB_H

#include <linux/usb/mv_usb.h>

enum pxa_ehci_type {
	EHCI_UNDEFINED = 0,
	PXA_U2OEHCI,  /* pxa 168, 9xx */
	PXA_SPH, /* pxa 168, 9xx SPH */
	MMP3_HSIC, /* mmp3 hsic */
	MMP3_FSIC, /* mmp3 fsic */
};

enum {
	MV_USB_MODE_OTG,
	MV_USB_MODE_HOST,
};

enum {
	VBUS_LOW	= 0,
	VBUS_HIGH	= 1 << 0,
};

enum charger_type {
	NULL_CHARGER            = 0,
	DEFAULT_CHARGER,
	VBUS_CHARGER,
	AC_CHARGER_STANDARD,
	AC_CHARGER_OTHER,
};

extern int mv_udc_register_client(struct notifier_block *nb);
extern int mv_udc_unregister_client(struct notifier_block *nb);

struct mv_usb_addon_irq {
	unsigned int	irq;
	int		(*poll)(void);
};

struct mv_usb_platform_data {
	unsigned int		clknum;
	char			**clkname;
	struct mv_usb_addon_irq	*id;	/* Only valid for OTG. ID pin change*/
	struct mv_usb_addon_irq	*vbus;	/* valid for OTG/UDC. VBUS change*/
	/* only valid for HCD. OTG or Host only*/
	unsigned int		mode;

	/* This flag is used for that needs id pin checked by otg */
	unsigned int	disable_otg_clock_gating:1;
	/* Force a_bus_req to be asserted */
	unsigned int	otg_force_a_bus_req:1;

	int     (*phy_init)(unsigned int regbase);
	void    (*phy_deinit)(unsigned int regbase);
	int	(*set_vbus)(unsigned int vbus);
	int	(*private_init)(struct mv_op_regs *opregs,
					unsigned int phyregs);
};

extern int pxa_usb_phy_init(unsigned int base);
extern void pxa_usb_phy_deinit(unsigned int base);

#endif
