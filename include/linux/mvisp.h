/*
 * mvisp.h
 *
 * Marvell DxO ISP - DMA module
 *
 * Copyright:  (C) Copyright 2011 Marvell International Ltd.
 *              Henry Zhao <xzhao10@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */


#ifndef ISP_USER_H
#define ISP_USER_H

#include <linux/videodev2.h>
#include <linux/types.h>

struct v4l2_dxoipc_set_fb {
	void	*virAddr[4];
	int		phyAddr[4];
	int		size[4];
	int		fbcnt;
	int		burst_read;
	int		burst_write;
	int		flush_fifo_by_timer;
};

struct v4l2_dxoipc_ipcwait {
	int				timeout;
	unsigned int	tick;
};

struct v4l2_dxoipc_streaming_config {
	int		enable_in;
	int		enable_disp;
	int		enable_codec;
	int		enable_fbrx;
	int		enable_fbtx;
};

struct v4l2_dxoipc_config_codec {
	int		vbnum;
	int		vbsize;
	int		dma_burst_size;
};

struct v4l2_sensor_get_driver_name {
	char	driver[16];
};

struct v4l2_sensor_register_access {
	unsigned long	reg;
	unsigned long	value;
};


#define VIDIOC_PRIVATE_DXOIPC_SET_FB \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 1, struct v4l2_dxoipc_set_fb)
#define VIDIOC_PRIVATE_DXOIPC_WAIT_IPC \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 2, struct v4l2_dxoipc_ipcwait)
#define VIDIOC_PRIVATE_DXOIPC_SET_STREAM \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 3, struct v4l2_dxoipc_streaming_config)
#define VIDIOC_PRIVATE_SENSER_GET_DRIVER_NAME \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 4, struct v4l2_sensor_get_driver_name)
#define VIDIOC_PRIVATE_SENSER_REGISTER_SET \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 5, struct v4l2_sensor_register_access)
#define VIDIOC_PRIVATE_SENSER_REGISTER_GET \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 6, struct v4l2_sensor_register_access)
#define VIDIOC_PRIVATE_DXOIPC_CONFIG_CODEC \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 7, struct v4l2_dxoipc_config_codec)

#endif	/* ISP_USER_H */
