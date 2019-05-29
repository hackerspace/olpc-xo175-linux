/*
 * linux/drivers/media/video/icp-hd.c - Aptina ICP-HD Image Signal Processor driver
 *
 * Based on linux/drivers/media/video/ov7690.c
 *
 * Copyright:	(C) Copyright 2010, Marvell International Ltd.
 *		Jiaquan Su <jqsu@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>

#include "icp-hd.h"

#define	DRV_NAME	"icp-hd"

static const struct icphd_datafmt icphd_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8,	V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8,	V4L2_COLORSPACE_JPEG},
};

static struct icphd_format_struct {
	enum v4l2_mbus_pixelcode	code;
	struct regval_list	*regs;
} icphd_fmts[] = {
	{
		.code	=	V4L2_MBUS_FMT_UYVY8_2X8,
		.regs	=	NULL,
	},
	{
		.code	=	V4L2_MBUS_FMT_JPEG_1X8,
		.regs	=	NULL,
	},
};

struct icphd_win_size {
	int	width;
	int	height;
	struct regval_list *regs;
	int	timing_id;
};

static struct mipi_phy icphd_timings[] = {
	{ /* ICP-HD default timing */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x08,
		.hs_settle	= 0x20,
		.hs_rx_to	= 0xFFFF,
		.lane		= 2,
	},
	{ /* ICP-HD default timing */
		.cl_termen	= 0x00,
		.cl_settle	= 0x04,
		.hs_termen	= 0x02,
		.hs_settle	= 0x10,
		.hs_rx_to	= 0xFFFF,
		.lane		= 2,
	},
};
#define N_ICPHD_TIMINGS ARRAY_SIZE(icphd_timings)

static struct icphd_win_size icphd_sizes[] = {
	{
		.width	= 1920,
		.height	= 1080,
		.regs	= NULL,	/* 1080p */
	},

};
#define N_ICPHD_SIZES (ARRAY_SIZE(icphd_sizes))

static struct icphd_win_size icphd_jpg_sizes[] = {
	{
		.width	= 3264,
		.height	= 2448,
	},
};
#define N_ICPHD_JPG_SIZES (ARRAY_SIZE(icphd_jpg_sizes))

#define N_ICPHD_FMTS (ARRAY_SIZE(icphd_fmts))

static struct icphd_info *to_info(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), \
			struct icphd_info, subdev);
}

static inline int icphd_power(struct icphd_info *info, int onoff)
{
	struct i2c_client *client;
	struct soc_camera_device *icd;

	if (unlikely(info == NULL))
		goto err_pdata;
	client = info->i2c_client;
	if (unlikely(client == NULL))
		goto err_pdata;
	icd = client->dev.platform_data;
	if (unlikely(icd == NULL))
		goto err_pdata;

	return info->power(icd->pdev, onoff);

err_pdata:
	printk(KERN_ERR "cam: icphd: Can't find platform data\n");
	return -EINVAL;
}

/***************************** I2C I/O functions ******************************/
static inline int icphd_read_n(const struct i2c_client *i2c, u16 reg, \
				void *buffer, u8 len)
{
#define NR_TRANS 2
	unsigned char msgbuf0[2];
	unsigned char msgbuf1[BUFFER_DEPTH];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[NR_TRANS] = {
		/* Send address */
		{i2c->addr, i2c->flags, 2, msgbuf0},
		/* Receive data */
		{i2c->addr, i2c->flags | I2C_M_RD, len, msgbuf1},
	};
	int ret;

	if (buffer == NULL)
		return -EINVAL;

	msgbuf0[0] = (unsigned char)(reg>>8);
	msgbuf0[1] = (unsigned char)reg;

	ret = i2c_transfer(adap, msg, NR_TRANS);
	if (ret < 0)
		goto out;
	memcpy(buffer, msgbuf1, len);
out:
	return (ret < 0) ? ret : 0;
}

static inline u16 icphd_read_w(const struct i2c_client *i2c, u16 reg)
{
	u16 ret;
	icphd_read_n(i2c, reg, &ret, 2);
	return swab16(ret);
}

static inline u32 icphd_read_d(const struct i2c_client *i2c, u16 reg)
{
	u32 ret;
	icphd_read_n(i2c, reg, &ret, 4);
	return swab32(ret);
}

static int icphd_write_w(struct i2c_client *c, u16 reg, u16 value)
{
	u8 data[4];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value >> 8;
	data[3] = value & 0xFF;
	ret = i2c_master_send(c, data, 4);
	return (ret < 0) ? ret : 0;
}

#define HUNK_SIZE	0x400

static int icphd_write_wa(struct i2c_client *c, u16 reg, u16 *value, u16 cnt)
{
	static u8 i2c_buffer[2 + HUNK_SIZE * 2];
	int i, ret = 0;
	u8 *pdata = i2c_buffer;

	*pdata++ = reg >> 8;
	*pdata++ = reg;
	for (i = 0; i < cnt; i++) {
		*pdata++ = value[i] >> 8;
		*pdata++ = value[i] & 0xFF;
	}
	ret = i2c_master_send(c, i2c_buffer, 2 + cnt * 2);
	if (ret < 0)
		return ret;
	return 0;
}

static int icphd_write_d(struct i2c_client *c, u16 reg, u32 value)
{
	u8 data[6];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = (value >> 24) & 0xFF;
	data[3] = (value >> 16) & 0xFF;
	data[4] = (value >> 8) & 0xFF;
	data[5] = value & 0xFF;
	ret = i2c_master_send(c, data, 6);
	return (ret < 0) ? ret : 0;
}
#if 0
static int icphd_write(struct i2c_client *c, u16 reg, u8 value)
{
	return 0;
}

static int icphd_write_n(struct i2c_client *c, u16 reg, void *buffer, u8 len)
{
	return 0;
}
#endif
static inline int icphd_reset(struct i2c_client *client)
{

	return 0;
}

static int icphd_load_basic(struct i2c_client *client);

static int icphd_detect(struct i2c_client *client)
{
	int cnt = 0;
	u16 ret;

	for (cnt = 0; cnt < 3; cnt++) {
		ret = icphd_read_w(client, 0x0000);
		printk(KERN_INFO "icp-hd: dev_id = 0x%04X\n", ret);
		if (ret == 0x0EC1)
			goto read_fw;
		msleep(10);
	}
	return -ENXIO;
read_fw:
	ret = icphd_read_w(client, 0x004E);
	printk(KERN_INFO "icp-hd: firmware_ver = 0x%04X\n", ret);
	ret = icphd_read_w(client, 0x0050);
	printk(KERN_INFO "icp-hd: chip_ver = 0x%X, proto_rel = 0x%X " \
			"rel_rel = 0x%02X\n", \
			(ret>>12) & 0xF, (ret>>8) & 0xF, ret&0xFF);
	return 0;
}

static int icphd_load_basic(struct i2c_client *client)
{
#if 0
	/* VGA 30FPS */
	icphd_write_w(client, 0x604a, 0x0840);
	icphd_write_d(client, 0x6034, 0x00006666);
	icphd_write_d(client, 0x6038, 0x00240100);
	icphd_write_d(client, 0x603c, 0x00000000);
	icphd_write_w(client, 0x6040, 0x4000);
	icphd_write_w(client, 0x6042, 0x4000);
	icphd_write_w(client, 0x6044, 0x036c);
	icphd_write_d(client, 0x2058, 0x07037f26);
	icphd_write_d(client, 0x205c, 0x08030105);
	icphd_write_d(client, 0x2060, 0x40000000);
	icphd_write_d(client, 0x2064, 0x00ea0000);
	icphd_write_d(client, 0x304c, 0x07037f26);
	icphd_write_d(client, 0x3050, 0x08030105);
	icphd_write_d(client, 0x3054, 0x40000000);
	icphd_write_d(client, 0x3058, 0x00ea0000);
	icphd_write_d(client, 0xf03c, 0x00095004);
	icphd_write_d(client, 0xe000, 0x00008ff8);
	icphd_write_d(client, 0xf03c, 0x00095000);
	icphd_write_d(client, 0xe000, 0x000080f8);
	icphd_write_d(client, 0xf03c, 0x00001430);
	icphd_write_d(client, 0xe000, 0x000068d9);
	icphd_write_w(client, 0x2000, 640);
	icphd_write_w(client, 0x2002, 480);
	icphd_write_w(client, 0x201a, 0x0001);
	icphd_write_w(client, 0x201c, 0x1e1e);
	icphd_write_w(client, 0x3000, 0x0cc0);
	icphd_write_w(client, 0x3002, 0x0990);
	icphd_write_w(client, 0x60a0, 0x0000);
	icphd_write_w(client, 0x60a2, 0x0000);
	icphd_write_w(client, 0x60a4, 0x0000);
	icphd_write_w(client, 0x60a6, 0x0000);
	icphd_write_w(client, 0x60a8, 0x0000);
	icphd_write_w(client, 0x60aa, 0x0000);
	icphd_write_w(client, 0x60ac, 0x0000);
	icphd_write_w(client, 0x60ae, 0xa0b8);
	icphd_write_w(client, 0x600a, 0x0211);
	icphd_write_w(client, 0x2036, 0x0010);
	icphd_write_w(client, 0x302e, 0x0010);
	icphd_write_w(client, 0x3018, 0x0002);
	icphd_write_w(client, 0x301a, 0x0000);
	icphd_write_w(client, 0x301c, 0x1e1e);
	icphd_write_w(client, 0x301e, 0x001e);
	icphd_write_w(client, 0x4030, 0x0000);
#else
	/* 1080P 29FPS */
	icphd_write_w(client, 0x604a, 0x0840);
	icphd_write_d(client, 0x6034, 0x00006666);
	icphd_write_d(client, 0x6038, 0x00300100);
	icphd_write_d(client, 0x603c, 0x00000000);
	icphd_write_w(client, 0x6040, 0x4000);
	icphd_write_w(client, 0x6042, 0x4000);
	icphd_write_w(client, 0x6044, 0x036c);
	icphd_write_d(client, 0x2058, 0x0b077f33);
	icphd_write_d(client, 0x205c, 0x08070105);
	icphd_write_d(client, 0x2060, 0x40000000);
	icphd_write_d(client, 0x2064, 0x009c0000);
	icphd_write_d(client, 0x304c, 0x0b077f33);
	icphd_write_d(client, 0x3050, 0x08070105);
	icphd_write_d(client, 0x3054, 0x40000000);
	icphd_write_d(client, 0x3058, 0x009c0000);
	icphd_write_d(client, 0xf03c, 0x00095004);
	icphd_write_d(client, 0xe000, 0x00008ff8);
	icphd_write_d(client, 0xf03c, 0x00095000);
	icphd_write_d(client, 0xe000, 0x000080f8);
	icphd_write_d(client, 0xf03c, 0x00001430);
	icphd_write_d(client, 0xe000, 0x000068d9);
	icphd_write_w(client, 0x2000, 0x0780);
	icphd_write_w(client, 0x2002, 0x0438);
	icphd_write_w(client, 0x201a, 0x0000);
	icphd_write_w(client, 0x2008, 0x0180);
	icphd_write_w(client, 0x200a, 0x0b40);
	icphd_write_w(client, 0x200c, 0x020a);
	icphd_write_w(client, 0x200e, 0x0786);
	icphd_write_w(client, 0x201c, 0x1e1e);
	icphd_write_w(client, 0x3000, 0x0cc0);
	icphd_write_w(client, 0x3002, 0x0990);
	icphd_write_w(client, 0x60a0, 0x0000);
	icphd_write_w(client, 0x60a2, 0x0000);
	icphd_write_w(client, 0x60a4, 0x0000);
	icphd_write_w(client, 0x60a6, 0x0000);
	icphd_write_w(client, 0x60a8, 0x0000);
	icphd_write_w(client, 0x60aa, 0x0000);
	icphd_write_w(client, 0x60ac, 0x0000);
	icphd_write_w(client, 0x60ae, 0xa0b8);
	icphd_write_w(client, 0x600a, 0x0211);
	icphd_write_w(client, 0x2036, 0x0010);
	icphd_write_w(client, 0x302e, 0x0010);
	icphd_write_w(client, 0x3018, 0x0002);
	icphd_write_w(client, 0x301a, 0x0000);
	icphd_write_w(client, 0x301c, 0x1e1e);
	icphd_write_w(client, 0x301e, 0x001e);
	icphd_write_w(client, 0x4030, 0x0000);
#endif
	return 0;
}

static int icphd_load_bootdata(struct i2c_client *client)
{
	int hunk_size = HUNK_SIZE, idx = 0;
	int size = ARRAY_SIZE(mt9e013_bootdata_1080p);
	u16 addr = 0x8000;

	printk("icphd: loading bootdata ");
	while (size > 0) {
		if (size < hunk_size)
			hunk_size = size;

		icphd_write_wa(client, addr, mt9e013_bootdata_1080p + idx, \
				hunk_size);
		printk(".");
		size -= hunk_size;
		idx += hunk_size;
		addr += hunk_size * 2;
		if (addr >= 0xA000)
			addr = 0x8000;
	}
	printk("done\n");
	return 0;
}


/***************************** I2C I/O functions ******************************/
static int icphd_get_awb(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int icphd_set_awb(struct i2c_client *client, int value)
{
	return 0;
}

static int icphd_get_ae(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int icphd_set_ae(struct i2c_client *client, int value)
{
	return 0;
}

static int icphd_get_af(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int icphd_set_af(struct i2c_client *client, int value)
{
	return 0;
}

static int icphd_get_mipi_phy(struct i2c_client *client, __s32 *value)
{
	if (unlikely((void *)value == NULL))
		return -EPERM;
	/* Camera driver provide a address to fill in timing info */
	*value = (__s32)&(icphd_timings[1]);
	return 0;
}

static const struct v4l2_queryctrl icphd_controls[] = {
	{
		.id = V4L2_CID_FOCUS_AUTO,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto focus",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto white balance",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_EXPOSURE_AUTO,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.name = "auto exposure",
		.minimum = 0,
		.maximum = 1,
		.step = 1,
		.default_value = 0,
	},
	{
		.id = V4L2_CID_PRIVATE_GET_MIPI_PHY,
		.type = V4L2_CTRL_TYPE_CTRL_CLASS,
		.name = "get mipi timing"
	},
};

static unsigned long icphd_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_2LANE;

	/* If soc_camera_link::priv is pointing to sensor_platform_data */
	/* copy sensor_platform_data::interface to soc_camera_link::flags */
	if (icl->flags & 0x80000000) {
		struct sensor_platform_data *sensor;
		sensor = icl->priv;
		icl->flags |= sensor->interface;
	}
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int icphd_set_bus_param(struct soc_camera_device *icd, unsigned long f)
{
#if 0/*TODO: add mipi and parallel different setting*/
	if (f & SOCAM_MIPI) /* mipi setting*/
		icphd_write_array(client, icphd_mipi);
	else /* parallel setting*/
		icphd_write_array(client, icphd_mipi);
#endif
	return 0;
}

static struct soc_camera_ops icphd_ops = {
	.query_bus_param	= icphd_query_bus_param,
	.set_bus_param		= icphd_set_bus_param,
	.controls		= icphd_controls,
	.num_controls		= ARRAY_SIZE(icphd_controls),
};

static int icphd_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	int ret;

	qctrl = soc_camera_find_qctrl(&icphd_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = icphd_set_awb(client, ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = icphd_set_af(client, ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = icphd_set_ae(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int icphd_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = icphd_get_awb(client, &ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = icphd_get_af(client, &ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = icphd_get_ae(client, &ctrl->value);
		break;
	case V4L2_CID_PRIVATE_GET_MIPI_PHY:
		ret = icphd_get_mipi_phy(client, &ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int icphd_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct icphd_info *info = to_info(client);

	id->ident	= info->model;
	id->revision	= 0;

	return 0;
}

static int icphd_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	return 0;
}

static int icphd_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	return 0;
}

static int icphd_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int icphd_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct icphd_info *info = to_info(client);

	mf->width	= info->rect.width;
	mf->height	= info->rect.height;
	mf->code	= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	return 0;
}



static int icphd_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		icphd_write_w(client, 0x1000, 0x2000);
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		icphd_write_w(client, 0x1000, 0x2001);
		break;
	}
	return ret;
}

static int icphd_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct icphd_info *info = to_info(client);

	if (index >= info->num_fmts)
		return -EINVAL;

	*code = info->fmts[index].code;
	return 0;
}

static int icphd_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct icphd_info *info = to_info(client);

	/* enum the supported formats*/
	for (i = 0; i < N_ICPHD_FMTS; i++) {
		if (icphd_fmts[i].code == mf->code) {
			info->regs_fmt = icphd_fmts[i].regs;
			break;
		}
	}
	if (i >= N_ICPHD_FMTS) {
		printk(KERN_ERR "cam: icphd unsupported color format!\n");
		return -EINVAL;
	}

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		/* enum the supported sizes*/
		for (i = 0; i < N_ICPHD_SIZES; i++) {
			if (mf->width == icphd_sizes[i].width
				&& mf->height == icphd_sizes[i].height) {
				info->regs_size = icphd_sizes[i].regs;
				break;
			}
		}
		if (i >= N_ICPHD_SIZES) {
			printk(KERN_ERR "cam: icphd unsupported yuv size, " \
					"w%d, h%d!\n", mf->width, mf->height);
			return -EINVAL;
		}
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		/* enum the supported sizes*/
		for (i = 0; i < N_ICPHD_JPG_SIZES; i++) {
			if (mf->width == icphd_jpg_sizes[i].width
				&& mf->height == icphd_jpg_sizes[i].height) {
				info->regs_size = icphd_jpg_sizes[i].regs;
				break;
			}
		}
		if (i >= N_ICPHD_JPG_SIZES) {
			printk(KERN_ERR "cam: icphd unsupported jpeg size, " \
					"w%d, h%d!\n", mf->width, mf->height);
			return -EINVAL;
		}
		break;
	}
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static int icphd_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!fsize)
		return -EINVAL;

	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		if (fsize->index >= N_ICPHD_SIZES)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.height = icphd_sizes[fsize->index].height;
		fsize->discrete.width = icphd_sizes[fsize->index].width;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (fsize->index >= N_ICPHD_JPG_SIZES)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.height = icphd_jpg_sizes[fsize->index].height;
		fsize->discrete.width = icphd_jpg_sizes[fsize->index].width;
		break;
	default:
		dev_err(&client->dev, "ov5642 unsupported format!\n");
		return -EINVAL;
	}
	return 0;
}

static int icphd_init(struct v4l2_subdev *sd, u32 plat)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i;
	u16 val;

	msleep(10);
	icphd_load_basic(client);
	msleep(50);
	val = icphd_read_w(client, 0x601A);
	val |= 0x0001;
	icphd_write_w(client, 0x601A, val);
	msleep(1);

	for (i = 0; i < 30; i++) {
		val = icphd_read_w(client, 0x601A);
		if ((val & 0x0001) == 0)
			goto bootdata;
		msleep(1);
	}
	return -EAGAIN;
bootdata:
	mdelay(50);
	icphd_load_bootdata(client);
	mdelay(50);
	icphd_write_w(client, 0x6002, 0xFFFF);
	printk(KERN_INFO "icp-hd: ISP initialzed\n");
	return 0;
}

static struct v4l2_subdev_core_ops icphd_subdev_core_ops = {
	.g_ctrl		= icphd_g_ctrl,
	.s_ctrl		= icphd_s_ctrl,
	.g_chip_ident	= icphd_g_chip_ident,
	.init		= icphd_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= icphd_g_register,
	.s_register	= icphd_s_register,
#endif
};

static struct v4l2_subdev_video_ops icphd_subdev_video_ops = {
	.s_stream	= icphd_s_stream,
	.s_mbus_fmt	= icphd_s_fmt,
	.g_mbus_fmt	= icphd_g_fmt,
	.try_mbus_fmt	= icphd_try_fmt,
	.enum_mbus_fmt	= icphd_enum_fmt,
	.enum_mbus_fsizes = icphd_enum_fsizes,
};

static struct v4l2_subdev_ops icphd_subdev_ops = {
	.core	= &icphd_subdev_core_ops,
	.video	= &icphd_subdev_video_ops,
};

static int icphd_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct icphd_info *info;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "icphd missing soc-camera data!\n");
		return -EINVAL;
	}
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "icphd driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	info = kzalloc(sizeof(struct icphd_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "icphd failed to alloc struct!\n");
		return -ENOMEM;
	}

	info->i2c_client = client;
	info->power = icl->power;

	info->rect.left = 0;
	info->rect.top = 0;
	info->rect.width = icphd_sizes[0].width;
	info->rect.height = icphd_sizes[0].height;
	info->pixfmt = V4L2_PIX_FMT_UYVY;

	icd->ops = &icphd_ops;

	/*info->model = V4L2_IDENT_ICPHD;*/
	info->fmts = icphd_colour_fmts;
	info->num_fmts = ARRAY_SIZE(icphd_colour_fmts);

	v4l2_i2c_subdev_init(&info->subdev, client, &icphd_subdev_ops);

	ret = icphd_detect(client);
	if (!ret) {
		printk(KERN_NOTICE "cam: Aptina ICP-HD detected!\n");
		return 0;
	}
	printk(KERN_ERR "cam: failed to detect Aptina ICP-HD!\n");

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	if (info)
		kfree(info);

	return -ENODEV;;
}

static int icphd_remove(struct i2c_client *client)
{
	struct icphd_info *info = to_info(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	icd->ops = NULL;
	if (icl->free_bus)
		icl->free_bus(icl);
	icl->power(icd->pdev, 0);

	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(info);
	return 0;
}

static struct i2c_device_id icphd_idtable[] = {
	{ DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, icphd_idtable);

static struct i2c_driver icphd_driver = {
	.driver = {
		.name	= DRV_NAME,
	},
	.id_table	= icphd_idtable,
	.probe		= icphd_probe,
	.remove		= icphd_remove,
};

static int __init icphd_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&icphd_driver);
	return ret;
}

static void __exit icphd_mod_exit(void)
{
	i2c_del_driver(&icphd_driver);
}

module_init(icphd_mod_init);
module_exit(icphd_mod_exit);

MODULE_DESCRIPTION("Aptina ICP-HD Image Signal Processor");
MODULE_AUTHOR("Jiaquan Su");
MODULE_LICENSE("GPL");
