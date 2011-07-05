/*
 * Copyright (C) 2011 Marvell International Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __MV_USB_H
#define __MV_USB_H

#define VUSBHS_MAX_PORTS	8
#define CAPLENGTH_MASK		(0xff)
#define DCCPARAMS_DEN_MASK	(0x1f)

#define HCSPARAMS_PPC		(0x10)

/* Frame Index Register Bit Masks */
#define USB_FRINDEX_MASKS	0x3fff

/* Command Register Bit Masks */
#define USBCMD_RUN_STOP				(0x00000001)
#define USBCMD_CTRL_RESET			(0x00000002)
#define USBCMD_SETUP_TRIPWIRE_SET		(0x00002000)
#define USBCMD_SETUP_TRIPWIRE_CLEAR		(~USBCMD_SETUP_TRIPWIRE_SET)

/* add dTD tripwire */
#define USBCMD_ATDTW_TRIPWIRE_SET		(0x00004000)
#define USBCMD_ATDTW_TRIPWIRE_CLEAR		(~USBCMD_ATDTW_TRIPWIRE_SET)

/* bit 15,3,2 are for frame list size */
#define USBCMD_FRAME_SIZE_1024			(0x00000000) /* 000 */
#define USBCMD_FRAME_SIZE_512			(0x00000004) /* 001 */
#define USBCMD_FRAME_SIZE_256			(0x00000008) /* 010 */
#define USBCMD_FRAME_SIZE_128			(0x0000000C) /* 011 */
#define USBCMD_FRAME_SIZE_64			(0x00008000) /* 100 */
#define USBCMD_FRAME_SIZE_32			(0x00008004) /* 101 */
#define USBCMD_FRAME_SIZE_16			(0x00008008) /* 110 */
#define USBCMD_FRAME_SIZE_8			(0x0000800C) /* 111 */

#define EPCTRL_TX_ALL_MASK			(0xFFFF0000)
#define EPCTRL_RX_ALL_MASK			(0x0000FFFF)

#define EPCTRL_TX_DATA_TOGGLE_RST		(0x00400000)
#define EPCTRL_TX_EP_STALL			(0x00010000)
#define EPCTRL_RX_EP_STALL			(0x00000001)
#define EPCTRL_RX_DATA_TOGGLE_RST		(0x00000040)
#define EPCTRL_RX_ENABLE			(0x00000080)
#define EPCTRL_TX_ENABLE			(0x00800000)
#define EPCTRL_CONTROL				(0x00000000)
#define EPCTRL_ISOCHRONOUS			(0x00040000)
#define EPCTRL_BULK				(0x00080000)
#define EPCTRL_INT				(0x000C0000)
#define EPCTRL_TX_TYPE				(0x000C0000)
#define EPCTRL_RX_TYPE				(0x0000000C)
#define EPCTRL_DATA_TOGGLE_INHIBIT		(0x00000020)
#define EPCTRL_TX_EP_TYPE_SHIFT			(18)
#define EPCTRL_RX_EP_TYPE_SHIFT			(2)

#define EPCOMPLETE_MAX_ENDPOINTS		(16)

/* endpoint list address bit masks */
#define USB_EP_LIST_ADDRESS_MASK              0xfffff800

#define PORTSCX_W1C_BITS			0x2a
#define PORTSCX_PORT_RESET			0x00000100
#define PORTSCX_PORT_POWER			0x00001000
#define PORTSCX_FORCE_FULL_SPEED_CONNECT	0x01000000
#define PORTSCX_PAR_XCVR_SELECT			0xC0000000
#define PORTSCX_PORT_FORCE_RESUME		0x00000040
#define PORTSCX_PORT_SUSPEND			0x00000080
#define PORTSCX_PORT_SPEED_FULL			0x00000000
#define PORTSCX_PORT_SPEED_LOW			0x04000000
#define PORTSCX_PORT_SPEED_HIGH			0x08000000
#define PORTSCX_PORT_SPEED_MASK			0x0C000000

/* otgsc Register Bit Masks */
#define OTGSC_CTRL_VUSB_DISCHARGE		0x00000001
#define OTGSC_CTRL_VUSB_CHARGE			0x00000002
#define OTGSC_CTRL_OTG_TERM			0x00000008
#define OTGSC_CTRL_DATA_PULSING			0x00000010
#define OTGSC_STS_USB_ID			0x00000100
#define OTGSC_STS_A_VBUS_VALID			0x00000200
#define OTGSC_STS_A_SESSION_VALID		0x00000400
#define OTGSC_STS_B_SESSION_VALID		0x00000800
#define OTGSC_STS_B_SESSION_END			0x00001000
#define OTGSC_STS_1MS_TOGGLE			0x00002000
#define OTGSC_STS_DATA_PULSING			0x00004000
#define OTGSC_INTSTS_USB_ID			0x00010000
#define OTGSC_INTSTS_A_VBUS_VALID		0x00020000
#define OTGSC_INTSTS_A_SESSION_VALID		0x00040000
#define OTGSC_INTSTS_B_SESSION_VALID		0x00080000
#define OTGSC_INTSTS_B_SESSION_END		0x00100000
#define OTGSC_INTSTS_1MS			0x00200000
#define OTGSC_INTSTS_DATA_PULSING		0x00400000
#define OTGSC_INTR_USB_ID			0x01000000
#define OTGSC_INTR_A_VBUS_VALID			0x02000000
#define OTGSC_INTR_A_SESSION_VALID		0x04000000
#define OTGSC_INTR_B_SESSION_VALID		0x08000000
#define OTGSC_INTR_B_SESSION_END		0x10000000
#define OTGSC_INTR_1MS_TIMER			0x20000000
#define OTGSC_INTR_DATA_PULSING			0x40000000

/* USB MODE Register Bit Masks */
#define USBMODE_CTRL_MODE_IDLE			0x00000000
#define USBMODE_CTRL_MODE_DEVICE		0x00000002
#define USBMODE_CTRL_MODE_HOST			0x00000003
#define USBMODE_CTRL_MODE_RSV			0x00000001
#define USBMODE_CTRL_MODE_MASK			0x00000003
#define USBMODE_SETUP_LOCK_OFF			0x00000008
#define USBMODE_STREAM_DISABLE			0x00000010

/* USB STS Register Bit Masks */
#define USBSTS_INT			0x00000001
#define USBSTS_ERR			0x00000002
#define USBSTS_PORT_CHANGE		0x00000004
#define USBSTS_FRM_LST_ROLL		0x00000008
#define USBSTS_SYS_ERR			0x00000010
#define USBSTS_IAA			0x00000020
#define USBSTS_RESET			0x00000040
#define USBSTS_SOF			0x00000080
#define USBSTS_SUSPEND			0x00000100
#define USBSTS_HC_HALTED		0x00001000
#define USBSTS_RCL			0x00002000
#define USBSTS_PERIODIC_SCHEDULE	0x00004000
#define USBSTS_ASYNC_SCHEDULE		0x00008000


/* Interrupt Enable Register Bit Masks */
#define USBINTR_INT_EN                          (0x00000001)
#define USBINTR_ERR_INT_EN                      (0x00000002)
#define USBINTR_PORT_CHANGE_DETECT_EN           (0x00000004)

#define USBINTR_ASYNC_ADV_AAE                   (0x00000020)
#define USBINTR_ASYNC_ADV_AAE_ENABLE            (0x00000020)
#define USBINTR_ASYNC_ADV_AAE_DISABLE           (0xFFFFFFDF)

#define USBINTR_RESET_EN                        (0x00000040)
#define USBINTR_SOF_UFRAME_EN                   (0x00000080)
#define USBINTR_DEVICE_SUSPEND                  (0x00000100)

#define USB_DEVICE_ADDRESS_MASK			(0xfe000000)
#define USB_DEVICE_ADDRESS_BIT_SHIFT		(25)

struct mv_cap_regs {
	u32	caplength_hciversion;
	u32	hcsparams;	/* HC structural parameters */
	u32	hccparams;	/* HC Capability Parameters*/
	u32	reserved[5];
	u32	dciversion;	/* DC version number and reserved 16 bits */
	u32	dccparams;	/* DC Capability Parameters */
};

struct mv_op_regs {
	u32	usbcmd;		/* Command register  */
	u32	usbsts;		/* Status register */
	u32	usbintr;	/* Interrupt enable */
	u32	frindex;	/* Frame index */
	u32	reserved1[1];
	u32	deviceaddr;	/* Device Address */
	u32	eplistaddr;	/* Endpoint List Address */
	u32	ttctrl;		/* HOST TT status and control */
	u32	burstsize;	/* Programmable Burst Size */
	u32	txfilltuning;	/* Host Transmit Pre-Buffer Packet Tuning */
	u32	reserved[4];	/*  0x016c, 0x0170, 0x0174) */
	u32	epnak;		/* Endpoint NAK */
	u32	epnaken;	/* Endpoint NAK Enable */
	u32	configflag;	/* Configured Flag register */
	u32	portsc[VUSBHS_MAX_PORTS]; /* Port Status/Control x, x = 1..8 */
	u32	otgsc;		/* OTG Status and Control Register */
	u32	usbmode;	/* USB Host/Device mode */
	u32	epsetupstat;	/* Endpoint Setup Status */
	u32	epprime;	/* Endpoint Initialize */
	u32	epflush;	/* Endpoint De-initialize */
	u32	epstatus;	/* Endpoint Status */
	u32	epcomplete;	/* Endpoint Interrupt On Complete */
	u32	epctrlx[16];	/* Endpoint Control, where x = 0.. 15 */
};

#endif
