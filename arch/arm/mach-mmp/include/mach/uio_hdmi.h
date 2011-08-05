/******************************************************************************
 *
 * Name:        uio_hdmi.h
 * Project:     MMP3
 * Yifan Zhang
 *
 * Copyright (c) 2011, Marvell International Ltd (zhangyf@marvell.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * (C) Copyright 2011 Marvell International Ltd.
 * All Rights Reserved
 *****************************************************************************/

#ifndef __UIO_HDMI_H__
#define __UIO_HDMI_H__

#define SSPA1_GET_VALUE 0
#define HPD_PIN_READ 6

struct uio_hdmi_platform_data {
	u32 sspa_reg_base;
	u32 gpio;
};

#endif
