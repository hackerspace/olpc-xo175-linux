/*
    camera.h - PXA camera driver header file

    Copyright (C) 2003, Intel Corporation
    Copyright (C) 2008, Guennadi Liakhovetski <kernel@pengutronix.de>

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

#ifndef __ASM_ARCH_CAMERA_H_
#define __ASM_ARCH_CAMERA_H_

#include <media/mrvl-camera.h>

#define PXA_CAMERA_MASTER	1
#define PXA_CAMERA_DATAWIDTH_4	2
#define PXA_CAMERA_DATAWIDTH_5	4
#define PXA_CAMERA_DATAWIDTH_8	8
#define PXA_CAMERA_DATAWIDTH_9	0x10
#define PXA_CAMERA_DATAWIDTH_10	0x20
#define PXA_CAMERA_PCLK_EN	0x40
#define PXA_CAMERA_MCLK_EN	0x80
#define PXA_CAMERA_PCP		0x100
#define PXA_CAMERA_HSP		0x200
#define PXA_CAMERA_VSP		0x400

/* for Marvell pxa955 camera driver */
#define SENSOR_CLOSE	0	/* Sensor clock disable */
#define SENSOR_OPEN	1	/* Sensor clock enable */
#define ISP_SENSOR_CLOSE	2
#define ISP_SENSOR_OPEN		3

struct mipi_phy {
	u16 cl_termen;
	u16 cl_settle;
	u16 cl_miss;
	u16 hs_termen;
	u16 hs_settle;
	u16 hs_rx_to;
	u16 lane;	/* When set to 0, S/W will try to figure out a value */
	u16 vc;		/* Virtual channel */
	u16 dt1;	/* Data type 1: For video or main data type */
	u16 dt2;	/* Data type 2: For thumbnail or auxiliry data type */
};

#ifdef CONFIG_SOC_CAMERA

struct pxa95x_csi_dev {
	u32 id;
	u32 irq_num;
	u32 reg_start;
	void __iomem *regs;
	spinlock_t dev_lock;
	struct clk *csi_tx_esc;
	struct mipi_phy *phy_cfg;
};

enum {
	CAMP_PWD_MAIN	= 0,
	CAMP_RST_MAIN,
	CAMP_PWD_SUB,
	CAMP_AVDD,
	CAMP_AFVCC,
	CAMP_END,
	CAMP_PWD	= 0,
	CAMP_RST,
};

enum {
	PIN_TYPE_NULL	= 0,
	PIN_TYPE_GPIO,
	PIN_TYPE_LDO,
	PIN_TYPE_END,
};

enum cam_pin_level {
	LEVEL_NEG	= 0,
	LEVEL_POS,
	LEVEL_LOW	= 0,
	LEVEL_HIGH,
};

struct sensor_power_pin {
	__u8	id;
	__u8	value;
	__u16	delay_ms;
};

struct layout_mapping {
	char	*name;
	int	type;
	union {
		char *ldo;
		int gpio;
	} layout;
	__u16	level_on;
	__u16	level_off;
	__u32	init_data;
	union {
		struct regulator *ldo;
		int gpio;
	} handle;
};

struct sensor_platform_data {
	int mount_pos;	/* bit 31: used/unused
			 * bit 0: resolution high/low
			 * bit 1:front/back
			 * bit 2: Left/Right */
	int interface;	/* MIPI or DVP flags*/
	struct pxa95x_csi_dev *csi_ctlr;
			/* Which controller is this sensor connected to? */
	int bridge;	/* Does this sensor needs or can act as a MIPI bridge
			 * if needs MIPI bridge, bridge = -x, x points to host
			 * if can be a bridge, bridge = x, x points to slave */
	int af_cap;	/* Auto focus capability */
	int pin_pwdn;	/* GPIO pin for power down */
	int pin_aux;	/* Auxiliary power down pin */
	int pin_irq;	/* Interrupt pin */
	int mclk_mhz;	/* Perfered MCLK for sensor in MHz */
	void *vendor_info;	/* Sensor vendor name or module name */
	char *board_name;	/* Board name */
	struct sensor_power_pin *power_on_seq;
	struct sensor_power_pin *power_off_seq;
	int (*v4l2_flash_if)(void *ctrl, bool op);
};

#endif /* CONFIG_SOC_CAMERA */

struct pxacamera_platform_data {
	unsigned long flags;
	unsigned long mclk_10khz;
};

extern void pxa_set_camera_info(struct pxacamera_platform_data *);

/* V4L2 driver specific controls */
/* #define V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD	(V4L2_CID_PRIVATE_BASE + 0) */
#define V4L2_CID_PRIVATE_GET_MIPI_PHY	(V4L2_CID_PRIVATE_BASE + 1)

#endif /* __ASM_ARCH_CAMERA_H_ */
