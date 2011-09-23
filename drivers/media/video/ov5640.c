/*
 * ov5640 Camera Driver
 *
 * Copyright (c) 2010 Marvell Ltd.
 * Angela Wan <jwan@marvell.com>
 * Kassey Lee <ygli@marvell.com>
 *
 * Based on linux/drivers/media/video/mt9m001.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <mach/camera.h>

#include "ov5640.h"

MODULE_DESCRIPTION("OmniVision OV5640 Camera Driver");
MODULE_LICENSE("GPL");

#define REG_PIDH    0x300a
#define REG_PIDL    0x300b

struct i2c_client *g_i2c_client;

static const struct ov5640_datafmt ov5640_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG},
};

static struct ov5640_win_size {
	int width;
	int height;
} ov5640_win_sizes[] = {
	/* QCIF */
	{
		.width = 176,
		.height = 144,
	},
	/* QVGA */
	{
		.width = 320,
		.height = 240,
	},
	/* 480*320 */
	{
		.width = 480,
		.height = 320,
	},
	/* VGA */
	{
		.width = 640,
		.height = 480,
	},
	/* D1 */
	{
		.width = 720,
		.height = 480,
	},
	/* full */
	{
		.width = 2592,
		.height = 1944,
	},
};

/* capture jpeg size */
static struct ov5640_win_size ov5640_win_sizes_jpeg[] = {
	/* full */
	{
		.width = 2592,
		.height = 1944,
	},
	/* 3M */
	{
		.width = 2048,
		.height = 1536,
	},
};

/* Find a data format by a pixel code in an array */
static const struct ov5640_datafmt *ov5640_find_datafmt(
	enum v4l2_mbus_pixelcode code, const struct ov5640_datafmt *fmt,
	int n)
{
	int i;
	for (i = 0; i < n; i++)
		if (fmt[i].code == code)
			return fmt + i;

	return NULL;
}

static struct ov5640 *to_ov5640(const struct i2c_client
					     *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct ov5640, subdev);
}

int ov5640_read(struct i2c_client *i2c, u16 reg, unsigned char *value)
{
	unsigned char msgbuf0[2];
	unsigned char msgbuf1[1];
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[2] = {{i2c->addr, i2c->flags, 2, msgbuf0},
				 {i2c->addr, i2c->flags | I2C_M_RD, 1, msgbuf1},
				};
	int num = 2, ret;

	if (value == NULL)
		return -EINVAL;

	msgbuf0[0] = (unsigned char)(reg>>8);	/* command */
	msgbuf0[1] = (unsigned char)reg;	/* command */

	ret = i2c_transfer(adap, msg, num);
	if (ret < 0)
		goto out;
	memcpy(value, msgbuf1, 1);
out:
	return (ret < 0) ? ret : 0;
}

int ov5640_write(struct i2c_client *c, u16 reg, unsigned char value)
{
	u8 data[3];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value;
	ret = i2c_master_send(c, data, 3);
	return (ret < 0) ? ret : 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5640_write_array(struct i2c_client *c, struct regval_list * vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV5640_MIPI_END_ADDR
	       || vals->value != OV5640_MIPI_END_VAL) {
		ret = ov5640_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

static int ov5640_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov5640_read(client, REG_PIDH, &v);

	if (ret < 0)
		return ret;
	if (v != 0x56)
		return -ENODEV;

	ret = ov5640_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x40)
		return -ENODEV;
	dev_err(&client->dev, "ov5640 detected 0x%x\n", v);
	return 0;
}

static int ov5640_g_chip_ident(struct v4l2_subdev *sd,
				   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);

	id->ident = ov5640->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5640_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov5640_read(client, (u16) reg->reg,
			       (unsigned char *)&(reg->val));
}

static int ov5640_s_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov5640_write(client, (u16) reg->reg,
				(unsigned char)reg->val);
}
#endif

static int set_stream(struct i2c_client *client, int enable)
{
	int ret = 0;
	if (enable) {
		ret = ov5640_write(client, 0x4201, 0x00);
		if (ret < 0)
			goto out;
		ret = ov5640_write(client, 0x4202, 0x00);
		if (ret < 0)
			goto out;
	} else {
		ret = ov5640_write(client, 0x4201, 0x01);
		if (ret < 0)
			goto out;
		ret = ov5640_write(client, 0x4202, 0x00);
		if (ret < 0)
			goto out;
	}
out:
	return ret;
}

static int ov5640_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	ret = set_stream(client, enable);
	if (ret < 0)
		dev_err(&client->dev, "ov5640 set stream error\n");
	return ret;
}

static int ov5640_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5640_colour_fmts))
		return -EINVAL;
	*code = ov5640_colour_fmts[index].code;
	return 0;
}

static int ov5640_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct ov5640_datafmt *fmt;
	int i;

	fmt = ov5640_find_datafmt(mf->code, ov5640_colour_fmts,
				   ARRAY_SIZE(ov5640_colour_fmts));
	if (!fmt) {
		dev_err(&client->dev, "ov5640 unsupported color format!\n");
		return -EINVAL;
	}

	mf->field = V4L2_FIELD_NONE;

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		/* enum the supported sizes*/
		for (i = 0; i < ARRAY_SIZE(ov5640_win_sizes); i++)
			if (mf->width == ov5640_win_sizes[i].width
				&& mf->height == ov5640_win_sizes[i].height)
				break;

		if (i >= ARRAY_SIZE(ov5640_win_sizes)) {
			dev_err(&client->dev, "ov5640 unsupported window"
				"size, w%d, h%d!\n", mf->width, mf->height);
			return -EINVAL;
		}
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		/* enum the supported sizes for JPEG*/
		for (i = 0; i < ARRAY_SIZE(ov5640_win_sizes_jpeg); i++)
			if (mf->width == ov5640_win_sizes_jpeg[i].width &&
				mf->height == ov5640_win_sizes_jpeg[i].height)
				break;

		if (i >= ARRAY_SIZE(ov5640_win_sizes_jpeg)) {
			dev_err(&client->dev, "ov5640 unsupported jpeg size!\n");
			return -EINVAL;
		}
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		dev_err(&client->dev, "ov5640 doesn't support code"
				"%d\n", mf->code);
		break;
	}
	return 0;
}

static int ov5640_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct ov5640_datafmt *fmt;
	struct regval_list *pregs = NULL;
	struct regval_list *pregs_default = NULL;

	fmt = ov5640_find_datafmt(mf->code, ov5640_colour_fmts,
				   ARRAY_SIZE(ov5640_colour_fmts));
	if (!fmt) {
		dev_err(&client->dev, "ov5640 unsupported color format!\n");
		return -EINVAL;
	}

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		switch (mf->width) {
		case 176:
			pregs_default = init_global_tab;
			pregs = yuv_QCIF_tab;
			dev_info(&client->dev, "choose qcif setting!\n");
			break;
		case 320:
			pregs_default = init_global_tab;
			pregs = yuv_QVGA_tab;
			dev_info(&client->dev, "choose qvga setting!\n");
			break;
		case 480:
			pregs_default = init_global_tab;
			pregs = yuv_Wallpaper_tab;
			dev_info(&client->dev, "choose 480*320 setting!\n");
			break;
		case 640:
			pregs_default = init_global_tab;
			pregs = yuv_VGA_tab;
			dev_info(&client->dev, "choose vga setting!\n");
			break;
		case 720:
			pregs_default = init_global_tab;
			pregs = yuv_D1_tab;
			dev_info(&client->dev, "choose d1 setting!\n");
			break;
		case 2592:
			pregs = yuv_5M_tab;
			dev_info(&client->dev, "choose 5M setting!\n");
			break;
		default:
			dev_err(&client->dev, "unsupported YUV size"
					"%s %d!\n", __func__, __LINE__);
			ret = -EINVAL;
			goto out;
			break;
		}
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		switch (mf->width) {
		case 2592:
			pregs_default = jpg_default_tab;
			dev_info(&client->dev, "choose 5M jpeg setting!\n");
			break;
		case 2048:
			pregs_default = jpg_default_tab;
			pregs = jpg_QXGA_tab;
			dev_info(&client->dev, "choose 3M jpeg setting!\n");
			break;
		default:
			dev_info(&client->dev, "unsupported JPEG size!\n");
			ret = -EINVAL;
			goto out;
			break;
		}
		break;
	default:
		dev_err(&client->dev, "unsupported format"
				"%s %d!\n", __func__, __LINE__);
		ret = -EINVAL;
		goto out;
		break;
	}
	if (pregs_default) {
		ret = ov5640_write_array(client, pregs_default);
		if (ret < 0) {
			dev_err(&client->dev, "set default registers err\n");
			goto out;
		}
	}
	if (pregs)
		ret = ov5640_write_array(client, pregs);
out:
	return ret;
}

static int ov5640_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5640 *ov5640 = to_ov5640(client);

	mf->width		= ov5640->rect.width;
	mf->height		= ov5640->rect.height;
	mf->code		= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov5640_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!fsize)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	/* abuse pixel_format, in fact, it is xlate->code*/
	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		if (fsize->index >= ARRAY_SIZE(ov5640_win_sizes)) {
			dev_warn(&client->dev,
				"ov5640 unsupported size %d!\n", fsize->index);
			return -EINVAL;
		}
		fsize->discrete.height = ov5640_win_sizes[fsize->index].height;
		fsize->discrete.width = ov5640_win_sizes[fsize->index].width;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (fsize->index >= ARRAY_SIZE(ov5640_win_sizes_jpeg)) {
			dev_warn(&client->dev,
				"ov5640 unsupported jpeg size %d!\n",
				fsize->index);
			return -EINVAL;
		}
		fsize->discrete.height =
			ov5640_win_sizes_jpeg[fsize->index].height;
		fsize->discrete.width =
			ov5640_win_sizes_jpeg[fsize->index].width;
		break;
	default:
		dev_err(&client->dev, "ov5640 unsupported format!\n");
		return -EINVAL;
	}
	return 0;
}

static unsigned long ov5640_query_bus_param(struct soc_camera_device
						*icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	/* ov5640 mipi setting: support 1 lane or 2 lane */
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE \
			| SOCAM_HSYNC_ACTIVE_LOW | SOCAM_VSYNC_ACTIVE_LOW \
			| SOCAM_PCLK_SAMPLE_FALLING;
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov5640_set_bus_param(struct soc_camera_device *icd,
				    unsigned long flags)
{
	return 0;
}

static struct soc_camera_ops ov5640_ops = {
	.query_bus_param = ov5640_query_bus_param,
	.set_bus_param = ov5640_set_bus_param,
};

static int ov5640_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	ov5640_write(client, 0x3103, 0x11);
	ov5640_write(client, 0x3008, 0x82);
	mdelay(5);
	ret = ov5640_write_array(client, init_global_tab);
	return ret;
}

static int ov5640_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *inter)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int	pre_div, multiplier, vco_freq;
	int sys_div, pll_rdiv, bit_div, sclk_rdiv;
	int sys_clk, vts, hts, frame_rate;
	int mclk;
	u8 val;

	mclk = inter->pad;
	/* get sensor system clk */
	/* vco frequency */
	ov5640_read(client, 0x3037, &val);
	/* There should be a complicated algorithm
	   including double member.
	 */
	pre_div = val & 0xf;
	pll_rdiv = (val >> 4) & 0x1;
	ov5640_read(client, 0x3036, &val);
	multiplier = (val < 128) ? val : (val/2*2);

	vco_freq = mclk / pre_div * multiplier;
	dev_dbg(&client->dev, "vco_freq: %d, mclk:%d,pre_div:%d,"
			"multiplier:%d\n", vco_freq, mclk, pre_div, multiplier);

	ov5640_read(client, 0x3035, &val);
	val = (val >> 4) & 0xf;
	sys_div = (val > 0) ? val : 16;
	pll_rdiv = (pll_rdiv == 0) ? 1 : 2;
	ov5640_read(client, 0x3034, &val);
	val = val & 0xf;
	bit_div = (val / 4 == 2) ? 2 : 1;
	ov5640_read(client, 0x3108, &val);
	val = val & 0x3;
	sclk_rdiv = (val == 0) ? 1 : ((val == 1) ? 2
			: ((val == 2) ? 4 : 8));

	sys_clk = vco_freq / sys_div / pll_rdiv / bit_div
		/ sclk_rdiv;
	dev_dbg(&client->dev, "sys_clk: %d, sys_div:%d,pll_rdiv:%d,"
			"bit_div:%d,sclk_rdiv:%d\n", sys_clk, sys_div,
			pll_rdiv, bit_div, sclk_rdiv);

	/* get sensor hts & vts */
	ov5640_read(client, 0x380c, &val);
	hts = val & 0xf;
	hts <<= 8;
	ov5640_read(client, 0x380d, &val);
	hts += val & 0xff;
	ov5640_read(client, 0x380e, &val);
	vts = val & 0xf;
	vts <<= 8;
	ov5640_read(client, 0x380f, &val);
	vts += val & 0xff;

	if (!hts || !vts)
		return -EINVAL;
	frame_rate = sys_clk * 1000000 / (hts * vts);
	dev_dbg(&client->dev, "frame_rate: %d,"
			"hts:%x, vts:%x\n", frame_rate, hts, vts);

	inter->interval.numerator = frame_rate;
	inter->interval.denominator = 1;

	return 0;
}

static struct v4l2_subdev_core_ops ov5640_subdev_core_ops = {
	.g_chip_ident = ov5640_g_chip_ident,
	.load_fw = ov5640_load_fw,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5640_g_register,
	.s_register = ov5640_s_register,
#endif
};

static struct v4l2_subdev_video_ops ov5640_subdev_video_ops = {
	.s_stream = ov5640_s_stream,
	.g_mbus_fmt = ov5640_g_fmt,
	.s_mbus_fmt = ov5640_s_fmt,
	.try_mbus_fmt = ov5640_try_fmt,
	.enum_mbus_fsizes = ov5640_enum_fsizes,
	.enum_mbus_fmt = ov5640_enum_fmt,
	.g_frame_interval =  ov5640_g_frame_interval,
};

static struct v4l2_subdev_ops ov5640_subdev_ops = {
	.core = &ov5640_subdev_core_ops,
	.video = &ov5640_subdev_video_ops,
};

static int ov5640_video_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov5640 *ov5640 = to_ov5640(client);
	int ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = ov5640_detect(client);

	if (ret)
		goto out;
	dev_err(&client->dev, "OmniVision ov5640 sensor detected\n");

	ov5640->model = V4L2_IDENT_OV5640;

out:
	return ret;

}

static int ov5640_probe(struct i2c_client *client,
			    const struct i2c_device_id *did)
{
	struct ov5640 *ov5640;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	g_i2c_client = client;

	if (!icd) {
		dev_err(&client->dev, "missing soc-camera data!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov5640 = kzalloc(sizeof(struct ov5640), GFP_KERNEL);
	if (!ov5640)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ov5640->subdev, client, &ov5640_subdev_ops);
	icd->ops = &ov5640_ops;
	ov5640->rect.left = 0;
	ov5640->rect.top = 0;
	ov5640->rect.width = 640;
	ov5640->rect.height = 480;
	ov5640->pixfmt = V4L2_PIX_FMT_YUV420;

	ret = ov5640_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(ov5640);
	}

	return ret;
}

static int ov5640_remove(struct i2c_client *client)
{
	struct ov5640 *ov5640 = to_ov5640(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	kfree(ov5640);
	return 0;
}

static const struct i2c_device_id ov5640_idtable[] = {
	{"ov5640", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov5640_idtable);

static struct i2c_driver ov5640_driver = {
	.driver = {
		   .name = "ov5640",
		   },
	.probe = ov5640_probe,
	.remove = ov5640_remove,
	.id_table = ov5640_idtable,
};

static int __init ov5640_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov5640_driver);
	return ret;
}

static void __exit ov5640_mod_exit(void)
{
	i2c_del_driver(&ov5640_driver);
}

module_init(ov5640_mod_init);
module_exit(ov5640_mod_exit);
