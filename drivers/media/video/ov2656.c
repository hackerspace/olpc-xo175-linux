/*
 * ov2656 Camera Driver
 *
 * Copyright (c) 2010 Marvell Ltd.
 * Qing Xu <qingx@marvell.com>
 * Kassey Lee <ygli@marvell.com>
 * Angela Wan <jwan@marvell.com>
 * Jiaquan Su <jqsu@marvell.com>
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
#include "ov2656.h"

MODULE_DESCRIPTION("OmniVision OV2656 Camera Driver");
MODULE_LICENSE("GPL");

static struct i2c_client *g_i2c_client;
static int is_prev_res_high;

static const struct ov2656_datafmt ov2656_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB}
};

static const struct v4l2_queryctrl ov2656_controls[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "brightness",
	}
};

static struct ov2656 *to_ov2656(const struct i2c_client
					     *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct ov2656, subdev);
}

int ov2656_read(struct i2c_client *i2c, u16 reg, unsigned char *value)
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

int ov2656_write(struct i2c_client *c, u16 reg, unsigned char value)
{
	u8 data[3];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value;
	ret = i2c_master_send(c, data, 3);
	if (reg == REG_SYS && (value & SYS_RESET)) {
		printk(KERN_WARNING "ov2656: a S/W reset triggered, may result in register value lost\n");
		msleep(4);  /* Wait for reset to run */
	}
	return (ret < 0) ? ret : 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov2656_write_array(struct i2c_client *c, struct regval_list * vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV2656_END_ADDR
	       || vals->value != OV2656_END_VAL) {
		ret = ov2656_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

static int ov2656_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov2656_read(client, REG_PIDH, &v);

	if (ret < 0)
		return ret;
	if (v != 0x26)
		return -ENODEV;

	ret = ov2656_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x56)
		return -ENODEV;

	dev_err(&client->dev, "camera: ov2656 detected 0x%x\n", v);
	return 0;
}

static int ov2656_g_chip_ident(struct v4l2_subdev *sd,
				   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2656 *ov2656 = to_ov2656(client);

	id->ident = ov2656->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov2656_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov2656_read(client, (u16) reg->reg,
			       (unsigned char *)&(reg->val));
}

static int ov2656_s_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov2656_write(client, (u16) reg->reg,
				(unsigned char)reg->val);
}
#endif

static int ov2656_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ov2656_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov2656_colour_fmts))
		return -EINVAL;
	*code = ov2656_colour_fmts[index].code;
	return 0;
}

int ov2656_get_platform_name(struct v4l2_subdev *sd, char *name)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov2656 driver needs platform data\n");
		return -EINVAL;
	}
	strcpy(name, icl->priv);
	return 1;
}

static int ov2656_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2656 *ov2656 = to_ov2656(client);

	ov2656->regs_fmt = ov2656_get_fmt_regs(mf->code, mf->width, mf->height);
	ov2656->regs_default = ov2656_get_fmt_default_setting(mf->code);

	ov2656->regs_lane_set = NULL;

	mf->field = V4L2_FIELD_NONE;

	switch (mf->code) {
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		/* enum the supported sizes*/
		ov2656->regs_resolution = ov2656_get_yuv_resolution_regs(mf->width, mf->height);
		ov2656->regs_size = ov2656_get_yuv_size_regs(mf->width, mf->height);
		if (!ov2656->regs_size) {
			dev_err(&client->dev, "ov2656 unsupported yuv resolution!\n");
			return -EINVAL;
		}
		if (mf->code == V4L2_MBUS_FMT_RGB565_2X8_LE)
			mf->colorspace = V4L2_COLORSPACE_SRGB;
		else
			mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		dev_err(&client->dev, "ov2656 doesn't support code %d\n", \
					mf->code);
		break;
	}

	return 0;
}

static int ov2656_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2656 *ov2656 = to_ov2656(client);
	int is_curr_res_high;

	if((mf->width >=1280 && mf->height >= 720))
		is_curr_res_high = 1;
	else
		is_curr_res_high = 0; 
		

	if(is_curr_res_high != is_prev_res_high){
		if (ov2656->regs_default) {
			ret = ov2656_write_array(client, ov2656->regs_default);
			if (ret)
				return ret;
		}

		if (ov2656->regs_fmt) {
			ret = ov2656_write_array(client, ov2656->regs_fmt);
			if (ret)
				return ret;
		}
	
		if (ov2656->regs_resolution) {
			ret = ov2656_write_array(client, ov2656->regs_resolution);
			if (ret)
				return ret;
		}
	
		if (ov2656->regs_size) {
			ret = ov2656_write_array(client, ov2656->regs_size);
			if (ret)
				return ret;
		}
	
		if (ov2656->regs_lane_set) {
			ret = ov2656_write_array(client, ov2656->regs_lane_set);
			if (ret)
				return ret;
		}
	}
	else{
		if (ov2656->regs_size) {
			ret = ov2656_write_array(client, ov2656->regs_size);
			if (ret)
				return ret;
		}
	}

	if(mf->width >= 1280 && mf->height >= 720)
		is_prev_res_high = 1;
	else
		is_prev_res_high = 0;
			
	return ret;
}

static int ov2656_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2656 *ov2656 = to_ov2656(client);

	mf->width		= ov2656->rect.width;
	mf->height		= ov2656->rect.height;
	mf->code		= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov2656_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val[16];
	int i;
	for (i = 0; i < 4; ++i) {
		ov2656_read(client, SCAN_ARR_X_OFF_H+i, val+i);
		ov2656_read(client, ISP_INPUT_X_SZ_H+i, val+i+4);
	}

	/* Bit width and endian unknow here, so don't want to use any trick*/
	crop->c.left	= (val[0] << 8) + val[1];
	crop->c.top	= (val[2] << 8) + val[3];
	crop->c.width	= (val[4] << 8) + val[5];
	crop->c.height	= (val[6] << 8) + val[7];
#endif

	return 0;
}

static int ov2656_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Turn off sensor output first */
	ov2656_s_stream(sd, 0);
	ov2656_write(client, SCAN_ARR_X_OFF_H, (crop->c.left>>8) & 0xff);
	ov2656_write(client, SCAN_ARR_X_OFF_L, crop->c.left & 0xff);
	ov2656_write(client, SCAN_ARR_Y_OFF_H, (crop->c.top>>8) & 0xff);
	ov2656_write(client, SCAN_ARR_Y_OFF_L, crop->c.top & 0xff);
	ov2656_write(client, ISP_INPUT_X_SZ_H, (crop->c.width>>8) & 0xff);
	ov2656_write(client, ISP_INPUT_X_SZ_L, crop->c.width & 0xff);
	ov2656_write(client, ISP_INPUT_Y_SZ_H, (crop->c.height>>8) & 0xff);
	ov2656_write(client, ISP_INPUT_Y_SZ_L, crop->c.height & 0xff);
	printk(KERN_INFO "cam: ov2656: S_CROP " \
		"LTWH = [0x%04X, 0x%04X, 0x%04X, 0x%04X]\n", \
		crop->c.left, crop->c.top, crop->c.width, crop->c.height);
	/* Delay 2~3 frames before sensor output get stable */
#endif
	msleep(90);
	ov2656_s_stream(sd, 1);
	return 0;
}

static int ov2656_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!fsize)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	/* abuse pixel_format, in fact, it is xlate->code*/
	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		return ov2656_set_yuv_res_array(fsize);
	default:
		dev_err(&client->dev, "ov2656 unsupported format 0x%x!\n", fsize->pixel_format);
		return -EINVAL;
	}
}

static unsigned long ov2656_query_bus_param(struct soc_camera_device
						*icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	/* ov2656 mipi setting: support 1 lane or 2 lane */
	/* ov2656 dvp setting: default value = HSYNC_ACTIVE_LOW
	 *		| VSYNC_ACTIVE_LOW | PCLK_SAMPLE_FALLING */
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE \
			| SOCAM_HSYNC_ACTIVE_LOW | SOCAM_VSYNC_ACTIVE_LOW \
			| SOCAM_PCLK_SAMPLE_FALLING;
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov2656_set_bus_param(struct soc_camera_device *icd,
				    unsigned long flags)
{
	return 0;
}

static struct soc_camera_ops ov2656_ops = {
	.query_bus_param = ov2656_query_bus_param,
	.set_bus_param = ov2656_set_bus_param,
	.controls			= ov2656_controls,
	.num_controls		= ARRAY_SIZE(ov2656_controls),
};

static int ov2656_set_brightness(struct i2c_client *client, int value)
{
	return 0;
}

static int ov2656_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	int ret = 0;

	qctrl = soc_camera_find_qctrl(&ov2656_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = ov2656_set_brightness(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ov2656_init(struct v4l2_subdev *sd, u32 plat)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2656 *ov2656 = to_ov2656(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
	int ret = 0;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov2656 driver needs platform data\n");
		return -EINVAL;
	}

	if (icl->priv) {
		ret = ov2656_select_bus_type(icl->priv);
		if (ret) {
			dev_err(&client->dev, "need know interface type\n");
			return ret;
		}
	} else {
		dev_err(&client->dev, "ov5642 driver need know plat-sensor\n");
		return -EINVAL;
	}

	/* Initialize settings*/
	ov2656->init = ov2656_get_global_init_regs();
	if (ov2656->init)
		ret = ov2656_write_array(client, ov2656->init);

	ov2656->regs_color_effect = ov2656_get_global_color_effect();
	if (ov2656->regs_color_effect)
		ret = ov2656_write_array(client, ov2656->regs_color_effect);

	return ret;
}

static int ov2656_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *inter)
{
#if 0
	struct i2c_client *client = v4l2_get_subdevdata(sd);

//this function is used to get mipi clk, mipi_lane_clk and set dphy value.
//In mmp2/mmp3, it seems not to use this function and the relative value.(Marvell other platform uses the frame_rate).
//Maybe you can ignore it first.
#endif

	inter->pad = 0x68;
	inter->interval.numerator = 0x20;
	inter->interval.denominator = 0x1;
	return 0;
}

static int ov2656_s_power(struct v4l2_subdev *sd, int on){

	if(!on)
		is_prev_res_high = -1;

	return 0;
}

static struct v4l2_subdev_core_ops ov2656_subdev_core_ops = {
	.g_chip_ident = ov2656_g_chip_ident,
	.init = ov2656_init,
	.s_ctrl	= ov2656_s_ctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov2656_g_register,
	.s_register = ov2656_s_register,
#endif
	.s_power = ov2656_s_power,
};

static struct v4l2_subdev_video_ops ov2656_subdev_video_ops = {
	.s_stream = ov2656_s_stream,
	.g_mbus_fmt = ov2656_g_fmt,
	.s_mbus_fmt = ov2656_s_fmt,
	.try_mbus_fmt = ov2656_try_fmt,
	.enum_mbus_fsizes = ov2656_enum_fsizes,
	.enum_mbus_fmt = ov2656_enum_fmt,
	.g_frame_interval =  ov2656_g_frame_interval,
	.g_crop = ov2656_g_crop,
	.s_crop = ov2656_s_crop,
};

static struct v4l2_subdev_ops ov2656_subdev_ops = {
	.core = &ov2656_subdev_core_ops,
	.video = &ov2656_subdev_video_ops,
};

static int ov2656_video_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov2656 *ov2656 = to_ov2656(client);
	int ret = 0;
	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = ov2656_detect(client);

	if (ret)
		goto out;

	dev_err(&client->dev, "OmniVision ov2656 sensor detected\n");

#ifdef CONFIG_VIDEO_MV
	mv_set_sensor_attached(true);
#endif

	ov2656->model = V4L2_IDENT_OV2656;

out:
	return ret;

}

static int ov2656_probe(struct i2c_client *client,
			    const struct i2c_device_id *did)
{
	struct ov2656 *ov2656;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	g_i2c_client = client;

	if (!icd) {
		dev_err(&client->dev, "ov2656: missing soc-camera data!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov2656 = kzalloc(sizeof(struct ov2656), GFP_KERNEL);
	if (!ov2656)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ov2656->subdev, client, &ov2656_subdev_ops);
	icd->ops = &ov2656_ops;
	ov2656->rect.left = 0;
	ov2656->rect.top = 0;
	ov2656->rect.width = 800;
	ov2656->rect.height = 600;
	ov2656->pixfmt = V4L2_PIX_FMT_YUV422P;

	ret = ov2656_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(ov2656);
	}

	return ret;
}

static int ov2656_remove(struct i2c_client *client)
{
	struct ov2656 *ov2656 = to_ov2656(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	kfree(ov2656);
	return 0;
}

static const struct i2c_device_id ov2656_idtable[] = {
	{"ov2656", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov2656_idtable);

static struct i2c_driver ov2656_driver = {
	.driver = {
		   .name = "ov2656",
		   },
	.probe = ov2656_probe,
	.remove = ov2656_remove,
	.id_table = ov2656_idtable,
};

static int __init ov2656_mod_init(void)
{
	return i2c_add_driver(&ov2656_driver);
}

static void __exit ov2656_mod_exit(void)
{
	i2c_del_driver(&ov2656_driver);
}

module_init(ov2656_mod_init);
module_exit(ov2656_mod_exit);
