/*-------------------------------------------------------

* Driver for OmniVision CMOS Image Sensor
*
* Copyright (C) 2010, Marvell International Ltd.
*				Qing Xu <qingx@marvell.com>
*
* Based on linux/drivers/media/video/mt9m001.c
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.

-------------------------------------------------------*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include <linux/videodev2.h>
#include <media/v4l2-common.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

#include <mach/camera.h>

#define REG_SYS		0x12
#define SYS_RESET	0x80
struct regval_list {
	u8 reg_num;
	unsigned char value;
};

static struct regval_list ov7692_default[] = {
/* From OVT Ken Xu, 19th,JUL.2011 */
	{0x12, 0x80},
	{0x0e, 0x08},
	{0x69, 0x52},
	{0x1e, 0xb3},
	{0x48, 0x42},
	{0xff, 0x01},
	{0xae, 0xa0},
	{0xa8, 0x26},
	{0xb4, 0xc0},
	{0xb5, 0x40},
	{0xff, 0x00},
	{0x0c, 0x90},	/*Was 0x0. bit 4 - YU/YV swap, bit 7 - vertical flip*/
	{0x62, 0x10},
	{0x12, 0x00},
	{0x17, 0x65},
	{0x18, 0xa4},
	{0x19, 0x0a},
	{0x1a, 0xf6},
	{0x3e, 0x30},
	{0x64, 0x0a},
	{0xff, 0x01},
	{0xb4, 0xc0},
	{0xff, 0x00},
	{0x67, 0x20},
	{0x81, 0x3f},
	{0xcc, 0x02},
	{0xcd, 0x80},
	{0xce, 0x01},
	{0xcf, 0xe0},
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xd0, 0x48},
	{0x82, 0x03},
	{0x0e, 0x00},
	{0x70, 0x00},
	{0x71, 0x34},
	{0x74, 0x28},
	{0x75, 0x98},
	{0x76, 0x00},
	{0x77, 0x64},
	{0x78, 0x01},
	{0x79, 0xc2},
	{0x7a, 0x4e},
	{0x7b, 0x1f},
	{0x7c, 0x00},
	{0x11, 0x00},
	{0x20, 0x00},
	{0x21, 0x23},
	{0x50, 0x9a},
	{0x51, 0x80},
	{0x4c, 0x7d},
	{0x0e, 0x00},
	{0x85, 0x10},
	{0x86, 0x00},
	{0x87, 0x00},
	{0x88, 0x00},
	{0x89, 0x2a},
	{0x8a, 0x26},
	{0x8b, 0x22},
	{0xbb, 0x7a},
	{0xbc, 0x69},
	{0xbd, 0x11},
	{0xbe, 0x13},
	{0xbf, 0x81},
	{0xc0, 0x96},
	{0xc1, 0x1e},
	{0xb7, 0x05},
	{0xb8, 0x09},
	{0xb9, 0x00},
	{0xba, 0x18},
	{0x5a, 0x1f},
	{0x5b, 0x9f},
	{0x5c, 0x6a},
	{0x5d, 0x42},
	{0x24, 0x78},
	{0x25, 0x68},
	{0x26, 0xb3},
	{0xa3, 0x0b},
	{0xa4, 0x15},
	{0xa5, 0x2a},
	{0xa6, 0x51},
	{0xa7, 0x63},
	{0xa8, 0x74},
	{0xa9, 0x83},
	{0xaa, 0x91},
	{0xab, 0x9e},
	{0xac, 0xaa},
	{0xad, 0xbe},
	{0xae, 0xce},
	{0xaf, 0xe5},
	{0xb0, 0xf3},
	{0xb1, 0xfb},
	{0xb2, 0x06},
	{0x8c, 0x5c},
	{0x8d, 0x11},
	{0x8e, 0x12},	/*changed from 92 for VC=00*/
	{0x8f, 0x19},
	{0x90, 0x50},
	{0x91, 0x20},
	{0x92, 0x96},
	{0x93, 0x80},
	{0x94, 0x13},
	{0x95, 0x1b},
	{0x96, 0xff},
	{0x97, 0x00},
	{0x98, 0x3d},
	{0x99, 0x36},
	{0x9a, 0x51},
	{0x9b, 0x43},
	{0x9c, 0xf0},
	{0x9d, 0xf0},
	{0x9e, 0xf0},
	{0x9f, 0xff},
	{0xa0, 0x68},
	{0xa1, 0x62},
	{0xa2, 0x0e},
#if 0
	/* Turn on night mode, color is more bright, but lower FPS */
	{0x15, 0xB8},
#endif
	{0xFF, 0xFF},
};

static struct regval_list ov7692_fmt_yuv422[] = {

	{0xFF, 0xFF}
};

static struct regval_list ov7692_res_qcif[] = {
	/* 176 x 144 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x00},
	{0xcd, 0xB0},
	{0xce, 0x00},
	{0xcf, 0x90},

	{0xFF, 0xFF}
};
static struct regval_list ov7692_res_cif[] = {
	/* 352 x 288 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x60},
	{0xce, 0x01},
	{0xcf, 0x20},

	{0xFF, 0xFF}
};

static struct regval_list ov7692_res_qvga[] = {
	/* 320 x 240 */
	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x01},
	{0xcd, 0x40},
	{0xce, 0x00},
	{0xcf, 0xf0},

	{0xFF, 0xFF}
};

static struct regval_list ov7692_res_vga[] = {
	/* 640 x 480 */

	{0xc8, 0x02},
	{0xc9, 0x80},
	{0xca, 0x01},
	{0xcb, 0xe0},
	{0xcc, 0x02},
	{0xcd, 0x80},
	{0xce, 0x01},
	{0xcf, 0xe0},

	{0xFF, 0xFF}
};

struct ov7692_datafmt {
	enum v4l2_mbus_pixelcode	code;
	enum v4l2_colorspace		colorspace;
};

struct ov7692 {
	struct v4l2_subdev subdev;
	int model;	/* V4L2_IDENT_OV7692* codes from v4l2-chip-ident.h */
	struct v4l2_rect rect;
	u32 pixfmt;
	const struct ov7692_datafmt *curfmt;
	const struct ov7692_datafmt *fmts;
	int num_fmts;

	struct regval_list *regs_fmt;
	struct regval_list *regs_size;
};

static const struct ov7692_datafmt ov7692_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov7692_format_struct {
	enum v4l2_mbus_pixelcode	code;
	struct regval_list	*regs;
} ov7692_fmts[] = {
	{
		.code	=	V4L2_MBUS_FMT_UYVY8_2X8,
		.regs	=	ov7692_fmt_yuv422,
	},
};

static struct mipi_phy ov7692_timings[] = {
	{ /* ov7692 default mipi PHY config */
		.cl_termen      = 0x00,
		.cl_settle      = 0x0c,
		.hs_termen      = 0x08,
		.hs_settle      = 0x1A,
		.hs_rx_to       = 0xFFFF,
		.lane           = 1,
	}
};

static struct ov7692_win_size {
	int	width;
	int	height;
	struct regval_list *regs;
	int	phy_cfg_id;
} ov7692_sizes[] = {
	{
		.width	= 176,
		.height	= 144,
		.regs	= ov7692_res_qcif,/* QCIF */
	},
	{
		.width	= 352,
		.height	= 288,
		.regs	= ov7692_res_cif,/* CIF */
	},
	{
		.width	= 320,
		.height	= 240,
		.regs	= ov7692_res_qvga,/* QVGA */
	},
	{
		.width	= 640,
		.height	= 480,
		.regs	= ov7692_res_vga,/* VGA */
	},
};

#define N_OV7692_SIZES (ARRAY_SIZE(ov7692_sizes))
#define N_OV7692_FMTS (ARRAY_SIZE(ov7692_fmts))

static struct ov7692 *to_ov7692(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov7692, subdev);
}

static int ov7692_read(struct i2c_client *c, unsigned char reg,
		unsigned char *value)
{
	s32 data;

	data = i2c_smbus_read_byte_data(c, reg);
	if (data < 0)
		return data;
	*value = (u8)data;
	return 0;
}

static int ov7692_write(struct i2c_client *c, unsigned char reg,
		unsigned char value)
{
	int ret;
	u8 data[2];

	data[0] = reg;
	data[1] = value;
	ret = i2c_master_send(c, data, 2);
	if (reg == REG_SYS && (value & SYS_RESET)) {
		printk(KERN_WARNING "cam: a S/W reset triggered, may result in register value lost\n");
		msleep(5);
	}
	return 0;
}

static int ov7692_write_array(struct i2c_client *c, struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = ov7692_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}

static int ov7692_detect(struct i2c_client *client)
{
	u8 v = 0;
	int ret = 0;

	ret = ov7692_read(client, 0x0a, &v);
	printk(KERN_NOTICE "cam: ov7692_detect 0x%x\n", v);
	if (ret < 0)
		return ret;
	if (v != 0x76)
		return -ENODEV;

	ret = ov7692_read(client, 0x0b, &v);
	printk(KERN_NOTICE "cam: ov7692_detect 0x%x\n", v);
	if (ret < 0)
		return ret;
	if (v != 0x92)
		return -ENODEV;

	return 0;
}

static int ov7692_get_mipi_phy(struct i2c_client *client, __s32 *value)
{
	if (unlikely((void *)value == NULL))
		return -EPERM;

	*value = (__s32)&(ov7692_timings[0]);
	return 0;
}

static int ov7692_get_awb(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7692_set_awb(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7692_get_ae(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7692_set_ae(struct i2c_client *client, int value)
{
	return 0;
}

static int ov7692_get_af(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int ov7692_set_af(struct i2c_client *client, int value)
{
	return 0;
}

static const struct v4l2_queryctrl ov7692_controls[] = {
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
		.name = "dphy configure",
	},
};

static unsigned long ov7692_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov7692_set_bus_param(struct soc_camera_device *icd, unsigned long f)
{
	return 0;
}

static struct soc_camera_ops ov7692_ops = {
	.query_bus_param	= ov7692_query_bus_param,
	.set_bus_param		= ov7692_set_bus_param,
	.controls			= ov7692_controls,
	.num_controls		= ARRAY_SIZE(ov7692_controls),
};

static int ov7692_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	int ret;

	qctrl = soc_camera_find_qctrl(&ov7692_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov7692_set_awb(client, ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = ov7692_set_af(client, ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov7692_set_ae(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ov7692_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = ov7692_get_awb(client, &ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = ov7692_get_af(client, &ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = ov7692_get_ae(client, &ctrl->value);
		break;
	case V4L2_CID_PRIVATE_GET_MIPI_PHY:
		ret = ov7692_get_mipi_phy(client, &ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int ov7692_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(client);
	id->ident		= ov7692->model;
	id->revision	= 0;

	return 0;
}

static int ov7692_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov7692_read(client, reg->reg, (unsigned char *)&(reg->val));
}

static int ov7692_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov7692_write(client, reg->reg, (unsigned char)reg->val);
}

static int ov7692_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	/* hard coding, 640x480, fix me!!!*/
	ov7692_write_array(client, ov7692_default);
	return 0;
}

static int ov7692_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(client);

	mf->width	= ov7692->rect.width;
	mf->height	= ov7692->rect.height;
	mf->code	= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov7692_init(struct v4l2_subdev *sd, u32 unused)
{
	int ret;
	unsigned char val;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* S/W reset */
	dev_info(&client->dev, "cam: will reset ov7692\n");
	ret = ov7692_read(client, REG_SYS, &val);
	if (unlikely(ret < 0))
		return ret;
	ret = ov7692_write(client, REG_SYS, val|SYS_RESET);
	if (unlikely(ret < 0)) {
		printk(KERN_ERR "cam: sensor ov7692 S/W reset failed\n");
		return ret;
	}
	/* Wait till reset complete */
	msleep(2);
	ret = ov7692_write(client, REG_SYS, val);

	/* Initialize settings*/
	ret |= ov7692_write_array(client, ov7692_default);

	if (unlikely(ret)) {
		printk(KERN_ERR "cam: one or more error during ov7692 initialize\n");
		return ret;
	}
	return 0;
}

static int ov7692_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(client);

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		if (ov7692->regs_fmt)
			ov7692_write_array(client, ov7692->regs_fmt);

		if (ov7692->regs_size)
			ov7692_write_array(client, ov7692->regs_size);

		msleep(100);
		break;
	default:
		printk(KERN_ERR "cam: not supported fmt!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ov7692_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(client);

	if (index >= ov7692->num_fmts)
		return -EINVAL;

	*code = ov7692->fmts[index].code;
	return 0;
}

static int ov7692_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7692 *ov7692 = to_ov7692(client);

	/* enum the supported formats*/
	for (i = 0; i < N_OV7692_FMTS; i++) {
		if (ov7692_fmts[i].code == mf->code) {
			ov7692->regs_fmt = ov7692_fmts[i].regs;
			break;
		}
	}
	if (i >= N_OV7692_FMTS) {
		dev_err(&client->dev, "cam: ov7692 unsupported color format!\n");
		return -EINVAL;
	}

	/* enum the supported sizes*/
	for (i = 0; i < N_OV7692_SIZES; i++) {
		if (mf->width == ov7692_sizes[i].width
			&& mf->height == ov7692_sizes[i].height) {
			ov7692->regs_size = ov7692_sizes[i].regs;
			break;
		}
	}
	if (i >= N_OV7692_SIZES) {
		dev_err(&client->dev, "cam: ov7692 unsupported window size, w%d, h%d!\n",
				mf->width, mf->height);
		return -EINVAL;
	}

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov7692_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!fsize)
		return -EINVAL;

	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		if (fsize->index >= N_OV7692_SIZES)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.height = ov7692_sizes[fsize->index].height;
		fsize->discrete.width = ov7692_sizes[fsize->index].width;
		break;
	default:
		dev_err(&client->dev, "ov7692 unsupported format!\n");
		return -EINVAL;
	}
	return 0;
}

static struct v4l2_subdev_core_ops ov7692_subdev_core_ops = {
	.g_ctrl		= ov7692_g_ctrl,
	.s_ctrl		= ov7692_s_ctrl,
	.init		= ov7692_init,
	.g_chip_ident	= ov7692_g_chip_ident,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov7692_g_register,
	.s_register	= ov7692_s_register,
#endif
};

static struct v4l2_subdev_video_ops ov7692_subdev_video_ops = {
	.s_stream	= ov7692_s_stream,
	.s_mbus_fmt	= ov7692_s_fmt,
	.g_mbus_fmt	= ov7692_g_fmt,
	.try_mbus_fmt	= ov7692_try_fmt,
	.enum_mbus_fmt	= ov7692_enum_fmt,
	.enum_mbus_fsizes = ov7692_enum_fsizes,
};

static struct v4l2_subdev_ops ov7692_subdev_ops = {
	.core	= &ov7692_subdev_core_ops,
	.video	= &ov7692_subdev_video_ops,
};

static int ov7692_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct ov7692 *ov7692;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "ov7692 missing soc-camera data!\n");
		return -EINVAL;
	}
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov7692 driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov7692 = kzalloc(sizeof(struct ov7692), GFP_KERNEL);
	if (!ov7692) {
		dev_err(&client->dev, "ov7692 failed to alloc struct!\n");
		return -ENOMEM;
	}

	ov7692->rect.left = 0;
	ov7692->rect.top = 0;
	ov7692->rect.width = 320;
	ov7692->rect.height = 240;
	ov7692->pixfmt = V4L2_PIX_FMT_UYVY;

	icd->ops = &ov7692_ops;

	ov7692->model = V4L2_IDENT_OV7692;
	ov7692->fmts = ov7692_colour_fmts;
	ov7692->num_fmts = ARRAY_SIZE(ov7692_colour_fmts);

	client->addr = 0x3C;	/* W/R for turnkey */
	v4l2_i2c_subdev_init(&ov7692->subdev, client, &ov7692_subdev_ops);

	ret = ov7692_detect(client);
	if (!ret) {
		printk(KERN_NOTICE "cam: OmniVision ov7692 sensor detected!\n");
		return 0;
	}
	printk(KERN_ERR "cam: failed to detect OmniVision ov7692!\n");

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	kfree(ov7692);

	return -ENODEV;
}

static int ov7692_remove(struct i2c_client *client)
{
	struct ov7692 *ov7692 = to_ov7692(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	icd->ops = NULL;
	if (icl->free_bus)
		icl->free_bus(icl);
	icl->power(icd->pdev, 0);

	i2c_set_clientdata(client, NULL);
	client->driver = NULL;
	kfree(ov7692);
	return 0;
}

static struct i2c_device_id ov7692_idtable[] = {
	{ "ov7692", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ov7692_idtable);

static struct i2c_driver ov7692_driver = {
	.driver = {
		.name	= "ov7692",
	},
	.id_table	= ov7692_idtable,
	.probe		= ov7692_probe,
	.remove		= ov7692_remove,
};

static int __init ov7692_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov7692_driver);
	return ret;
}

static void __exit ov7692_mod_exit(void)
{
	i2c_del_driver(&ov7692_driver);
}

module_init(ov7692_mod_init);
module_exit(ov7692_mod_exit);

MODULE_DESCRIPTION("OmniVision OV7692 Camera Driver");
MODULE_AUTHOR("Qing Xu");
MODULE_LICENSE("GPL");
