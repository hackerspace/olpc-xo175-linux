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
struct pxa95x_csi_dev {
	u32 irq_num;
	u32 reg_start;
	void __iomem *regs;
	spinlock_t dev_lock;
	struct clk *axi_clk;
	struct clk *csi_tx_esc;
};

struct pxa95x_cam_pdata {
	struct pxa95x_csi_dev *csidev;
};

enum {
	SENSOR_USED		= (1 << 31),
	SENSOR_UNUSED		= 0,
	SENSOR_POS_LEFT		= (1 << 2),
	SENSOR_POS_RIGHT	= 0,
	SENSOR_POS_FRONT	= (1 << 1),
	SENSOR_POS_BACK		= 0,
	SENSOR_RES_HIGH		= (1 << 0),
	SENSOR_RES_LOW		= 0,
};

struct sensor_platform_data {
	int mount_pos;	/* bit 31: used/unused
			 * bit 0: resolution high/low
			 * bit 1:front/back
			 * bit 2: Left/Right */
	int interface;	/* MIPI or DVP flags*/
	int intrfc_id;	/* To which controller is this sensor connected? */
	int bridge;	/* Does this sensor needs or can act as a MIPI bridge
			 * if needs MIPI bridge, bridge = -x, x points to host
			 * if can be a bridge, bridge = x, x points to slave */
	int torch;	/* strobe light */
	int af_cap;	/* Auto focus capability */
	int pin_pwdn;	/* GPIO pin for power down */
	int pin_aux;	/* Auxiliary power down pin */
	int mclk_mhz;	/* Perfered MCLK for sensor in MHz */
	void *vendor_info;	/* Sensor vendor name or module name */
	char *board_name;	/* Board name */
	char reserved[20];
};

struct pxacamera_platform_data {
	unsigned long flags;
	unsigned long mclk_10khz;
};

extern void pxa_set_camera_info(struct pxacamera_platform_data *);

#endif /* __ASM_ARCH_CAMERA_H_ */
