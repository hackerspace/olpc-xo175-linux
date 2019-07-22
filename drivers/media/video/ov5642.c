/*
 * ov5642 Camera Driver
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
#ifdef CONFIG_CPU_MMP2
#include <mach/mfp-mmp2.h>
#include <mach/mmp2_plat_ver.h>
#endif
#include "ov5642.h"

MODULE_DESCRIPTION("OmniVision OV5642 Camera Driver");
MODULE_LICENSE("GPL");

#define REG_PIDH    0x300a
#define REG_PIDL    0x300b

#define REG_SYS		0x3008
#define SYS_RESET	0x80
#define SYS_SWPD	0x40

#define REG_TIMINGCTRL	0x3818
#define REG_ARRAYCTRL	0x3621

struct i2c_client *g_i2c_client;

static const struct ov5642_datafmt ov5642_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB}
};

static const struct v4l2_queryctrl ov5642_controls[] = {
	{
		.id = V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "AF/FD etc. firmware download",
	}, {
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "brightness",
	}
};

static struct ov5642 *to_ov5642(const struct i2c_client
					     *client)
{
	return container_of(i2c_get_clientdata(client),
			    struct ov5642, subdev);
}

int ov5642_read(struct i2c_client *i2c, u16 reg, unsigned char *value)
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

int ov5642_write(struct i2c_client *c, u16 reg, unsigned char value)
{
	u8 data[3];
	int ret = 0;
	data[0] = reg >> 8;
	data[1] = reg;
	data[2] = value;
	ret = i2c_master_send(c, data, 3);
	if (reg == REG_SYS && (value & SYS_RESET)) {
		printk(KERN_WARNING "cam: a S/W reset triggered, may result in register value lost\n");
		msleep(4);  /* Wait for reset to run */
	}
	return (ret < 0) ? ret : 0;
}

/*
 * Write a list of register settings; ff/ff stops the process.
 */
static int ov5642_write_array(struct i2c_client *c, struct regval_list * vals)
{
	int i = 0;
	int ret = 0;
	while (vals->reg_num != OV5642_END_ADDR
	       || vals->value != OV5642_END_VAL) {
		ret = ov5642_write(c, vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
		i++;
	}
	return ret;
}

int to_mipi(u32 code, u32 width, u32 height)
{
struct regval_list *fmt_reg = NULL, *res_reg = NULL;
int ret;
	/* ov5642 works as a mipi converter, switch dvp signal to mipi signal*/
	ret = select_bus_type("saarb-mipi-bridge");
	if (ret) {
		dev_err(&g_i2c_client->dev, \
			"mipi bridge settings undefined\n");
		return ret;
	}

	fmt_reg = get_fmt_default_setting(code);
	res_reg = get_yuv_size_regs(width, height);

	if ((fmt_reg == NULL) || (res_reg == NULL)) {
		dev_err(&g_i2c_client->dev, \
			"mipi bridge format or resolution not supported\n");
		return -EINVAL;
	}

	ov5642_write_array(g_i2c_client, fmt_reg);
	ov5642_write_array(g_i2c_client, res_reg);

	msleep(1);
	return 0;
}
EXPORT_SYMBOL(to_mipi);

static int ov5642_detect(struct i2c_client *client)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov5642_read(client, REG_PIDH, &v);

	if (ret < 0)
		return ret;
	if (v != 0x56)
		return -ENODEV;

	ret = ov5642_read(client, REG_PIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0x42)
		return -ENODEV;
	dev_err(&client->dev, "camera: ov5642 detected 0x%x\n", v);
	return 0;
}

static int ov5642_g_chip_ident(struct v4l2_subdev *sd,
				   struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);

	id->ident = ov5642->model;
	id->revision = 0;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov5642_g_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov5642_read(client, (u16) reg->reg,
			       (unsigned char *)&(reg->val));
}

static int ov5642_s_register(struct v4l2_subdev *sd,
				 struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return ov5642_write(client, (u16) reg->reg,
				(unsigned char)reg->val);
}
#endif

static int ov5642_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	ret = set_stream(client, enable);
	if (ret < 0)
		dev_err(&client->dev, "ov5642 set stream error\n");
	return ret;
}

static int ov5642_enum_fmt(struct v4l2_subdev *sd,
		unsigned int index,
		enum v4l2_mbus_pixelcode *code)
{
	if (index >= ARRAY_SIZE(ov5642_colour_fmts))
		return -EINVAL;
	*code = ov5642_colour_fmts[index].code;
	return 0;
}

int ov5642_get_platform_name(struct v4l2_subdev *sd, char *name)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov5642 driver needs platform data\n");
		return -EINVAL;
	}
	strcpy(name, icl->priv);
	return 1;
}

static int ov5642_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);

	ov5642->regs_fmt = get_fmt_regs(mf->code, mf->width, mf->height);

	ov5642->regs_default = get_fmt_default_setting(mf->code);

	ov5642->regs_resolution = get_yuv_resolution_regs(mf->width, mf->height);

	ov5642->regs_lane_set = NULL;

	mf->field = V4L2_FIELD_NONE;

	switch (mf->code) {
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_UYVY8_2X8:
	case V4L2_MBUS_FMT_VYUY8_2X8:
		/* enum the supported sizes*/
		ov5642->regs_size = get_yuv_size_regs(mf->width, mf->height);
		if (!ov5642->regs_size) {
			dev_err(&client->dev, "ov5642 unsupported yuv resolution!\n");
			return -EINVAL;
		}
#ifdef CONFIG_CPU_MMP2
		if (board_is_mmp2_brownstone_rev5())
			ov5642->regs_lane_set = get_yuv_lane_set(mf->width, mf->height);
#endif
		if (mf->code == V4L2_MBUS_FMT_RGB565_2X8_LE)
			mf->colorspace = V4L2_COLORSPACE_SRGB;
		else
			mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		/* enum the supported sizes for JPEG*/
		ov5642->regs_size = get_jpg_size_regs(mf->width, mf->height);
		if (!ov5642->regs_size) {
			dev_err(&client->dev, "ov5642 unsupported yuv resolution!\n");
			return -EINVAL;
		}
#ifdef CONFIG_CPU_MMP2
		if (board_is_mmp2_brownstone_rev5())
			ov5642->regs_lane_set = get_jpg_lane_set(mf->width, mf->height);
#endif
		mf->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		dev_err(&client->dev, "ov5642 doesn't support code %d\n", \
					mf->code);
		break;
	}

	return 0;
}

static int ov5642_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);
#ifdef CONFIG_CPU_MMP2
	unsigned char val;
	char name[20] = "";

	ov5642_get_platform_name(sd, name);
	if (!strcmp(name, "pxa688-mipi")) {
		/* sensor resume from software power down mode */
		ov5642_read(client, REG_SYS, &val);
		val &= ~0x40;
		ov5642_write(client, REG_SYS, val);
	}
#endif

	if (ov5642->regs_default) {
		ret = ov5642_write_array(client, ov5642->regs_default);
		if (ret)
			return ret;
	}

	if (ov5642->regs_fmt) {
		ret = ov5642_write_array(client, ov5642->regs_fmt);
		if (ret)
			return ret;
	}

	if (ov5642->regs_resolution) {
		ret = ov5642_write_array(client, ov5642->regs_resolution);
		if (ret)
			return ret;
	}

	if (ov5642->regs_size) {
		ret = ov5642_write_array(client, ov5642->regs_size);
		if (ret)
			return ret;
	}

	if (ov5642->regs_lane_set) {
		ret = ov5642_write_array(client, ov5642->regs_lane_set);
		if (ret)
			return ret;
	}

#ifdef CONFIG_CPU_MMP2
	if (!strcmp(name, "pxa688-mipi")) {
	    /* Initialize MIPI settings */
		ov5642->regs_mipi_set = get_mipi_set_regs();
		if (ov5642->regs_mipi_set) {
			ret = ov5642_write_array(client, ov5642->regs_mipi_set);
			if (ret)
				return ret;
		}
		if (board_is_mmp2_brownstone_rev5())
			ov5642->regs_mipi_lane = get_mipi_lane_regs(1);
		else
			ov5642->regs_mipi_lane = get_mipi_lane_regs(2);
		if (ov5642->regs_mipi_lane) {
			ret = ov5642_write_array(client, ov5642->regs_mipi_lane);
			if (ret)
				return ret;
		}

		/* frontal camera sensor on brownstone board */
		/* need clear sensor mirror and flip */
		ov5642_read(client, REG_TIMINGCTRL, &val);
		val &= ~0x60;
		ov5642_write(client, REG_TIMINGCTRL, val);

		ov5642_read(client, REG_ARRAYCTRL, &val);
		val |= 0x20;
		ov5642_write(client, REG_ARRAYCTRL, val);

		/* sensor enter software power down mode */
		ov5642_read(client, REG_SYS, &val);
		val |= 0x40;
		ov5642_write(client, REG_SYS, val);
	}
#endif
	return ret;
}

static int ov5642_g_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);

	mf->width		= ov5642->rect.width;
	mf->height		= ov5642->rect.height;
	mf->code		= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field		= V4L2_FIELD_NONE;
	mf->colorspace		= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int ov5642_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val[16];
	int i;

	for (i = 0; i < 4; ++i) {
		ov5642_read(client, SCAN_ARR_X_OFF_H+i, val+i);
		ov5642_read(client, ISP_INPUT_X_SZ_H+i, val+i+4);
	}

	/* Bit width and endian unknow here, so don't want to use any trick*/
	crop->c.left	= (val[0] << 8) + val[1];
	crop->c.top	= (val[2] << 8) + val[3];
	crop->c.width	= (val[4] << 8) + val[5];
	crop->c.height	= (val[6] << 8) + val[7];

	return 0;
}

static int ov5642_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Turn off sensor output first */
	ov5642_s_stream(sd, 0);
	ov5642_write(client, SCAN_ARR_X_OFF_H, (crop->c.left>>8) & 0xff);
	ov5642_write(client, SCAN_ARR_X_OFF_L, crop->c.left & 0xff);
	ov5642_write(client, SCAN_ARR_Y_OFF_H, (crop->c.top>>8) & 0xff);
	ov5642_write(client, SCAN_ARR_Y_OFF_L, crop->c.top & 0xff);
	ov5642_write(client, ISP_INPUT_X_SZ_H, (crop->c.width>>8) & 0xff);
	ov5642_write(client, ISP_INPUT_X_SZ_L, crop->c.width & 0xff);
	ov5642_write(client, ISP_INPUT_Y_SZ_H, (crop->c.height>>8) & 0xff);
	ov5642_write(client, ISP_INPUT_Y_SZ_L, crop->c.height & 0xff);
	printk(KERN_INFO "cam: ov5642: S_CROP " \
		"LTWH = [0x%04X, 0x%04X, 0x%04X, 0x%04X]\n", \
		crop->c.left, crop->c.top, crop->c.width, crop->c.height);
	/* Delay 2~3 frames before sensor output get stable */
	msleep(90);
	ov5642_s_stream(sd, 1);
	return 0;
}

static int ov5642_enum_fsizes(struct v4l2_subdev *sd,
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
		return set_yuv_res_array(fsize);
	case V4L2_MBUS_FMT_JPEG_1X8:
		return set_jpg_res_array(fsize);
	default:
		dev_err(&client->dev, "ov5642 unsupported format!\n");
		return -EINVAL;
	}
}

static unsigned long ov5642_query_bus_param(struct soc_camera_device
						*icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	/* ov5642 mipi setting: support 1 lane or 2 lane */
	/* ov5642 dvp setting: default value = HSYNC_ACTIVE_LOW
	 *		| VSYNC_ACTIVE_LOW | PCLK_SAMPLE_FALLING */
	unsigned long flags = SOCAM_MIPI | SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE \
			| SOCAM_HSYNC_ACTIVE_LOW | SOCAM_VSYNC_ACTIVE_LOW \
			| SOCAM_PCLK_SAMPLE_FALLING;
	return soc_camera_apply_sensor_flags(icl, flags);
}

static int ov5642_set_bus_param(struct soc_camera_device *icd,
				    unsigned long flags)
{
	return 0;
}

static struct soc_camera_ops ov5642_ops = {
	.query_bus_param = ov5642_query_bus_param,
	.set_bus_param = ov5642_set_bus_param,
	.controls			= ov5642_controls,
	.num_controls		= ARRAY_SIZE(ov5642_controls),
};

static int ov5642_firmware_download(struct i2c_client *client)
{
	int ret = 0, i, j, size;
	unsigned char val, sysctl;
	char data[258];
	OV5642_FIRMWARE_ARRAY *firmware_regs = NULL;

	size = get_firmware(&firmware_regs);
	if (unlikely(size <= 0)) {
		dev_err(&client->dev, "No AF firmware available\n");
		return -ENODEV;
	}

	/* Check and clear software power down bit */
	ret = ov5642_read(client, REG_SYS, &sysctl);
	if (unlikely(ret < 0)) {
		dev_err(&client->dev, "cam: I2C failed to get 0x3008 value\n");
		return -EIO;
	}
	if (unlikely(sysctl & SYS_SWPD)) {
		ret = ov5642_write(client, REG_SYS, (sysctl & ~SYS_SWPD));
		if (unlikely(ret < 0))
			return -EIO;
	}

	/* Before start downloading firmware, check focus state, should be 0 */
	ret = ov5642_read(client, 0x3027, &val);
	if (unlikely(ret < 0)) {
		return -EIO;
	}
	if (unlikely(val != 0)) {
		dev_err(&client->dev, "Before download AF firmware " \
				"STA_FOCUS = 0x%02X, should be 0\n", val);
		return -EBUSY;
	}

	/* Actually start to download firmware */
	for (i = 0; i < size; i++) {
		data[0] = firmware_regs[i].reg_base >> 8;
		data[1] = firmware_regs[i].reg_base;
		for (j = 0; j < firmware_regs[i].len; j++)
			data[j+2] = firmware_regs[i].value[j];
		ret = i2c_master_send(client, data, firmware_regs[i].len+2);
		if (ret < 0) {
			dev_err(&client->dev, "i2c error %s %d\n",
				__func__, __LINE__);
			break;
		}
	}

	dev_info(&client->dev, "AF firmware downloaded\n");
	/* Recover ov5642 power state */
	ret = ov5642_write(client, REG_SYS, sysctl&(~SYS_RESET));
	if (unlikely(ret < 0))
		return -EIO;

	return (ret > 0) ? 0 : ret;
}

static int ov5642_set_brightness(struct i2c_client *client, int value)
{
	return 0;
}

static int ov5642_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	int ret = 0;

	qctrl = soc_camera_find_qctrl(&ov5642_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;
	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		ret = ov5642_set_brightness(client, ctrl->value);
		break;
	case V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD:
		ret = ov5642_firmware_download(client);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int ov5642_load_fw(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);
	struct soc_camera_device *icd = client->dev.platform_data;
	struct soc_camera_link *icl;
#ifdef CONFIG_PXA95x
	struct sensor_platform_data *pdata;
#endif
	int ret = 0;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "ov5642 driver needs platform data\n");
		return -EINVAL;
	}
#ifdef CONFIG_PXA95x
	pdata = icl->priv;
	if (pdata != NULL) {
		char name[32];
		strcpy(name, pdata->board_name);
		if (pdata->interface & SOCAM_MIPI)
			strcat(name, "-mipi");
		else
			strcat(name, "-dvp");
		ret = select_bus_type(name);
#else
	if (icl->priv) {
		ret = select_bus_type(icl->priv);
#endif
		if (ret) {
			dev_err(&client->dev, "need know interface type\n");
			return ret;
		}
	} else {
		dev_err(&client->dev, "ov5642 driver need know plat-sensor\n");
		return -EINVAL;
	}

	/* Initialize settings*/
	ov5642->init = get_global_init_regs();
	if (ov5642->init)
		ret = ov5642_write_array(client, ov5642->init);

	return ret;
}

static int ov5642_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *inter)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int mclk;

	/*Usd integer member*/
	/*
	 * For pre_div_map and div1to2p5_map it has float number in the original
	 * spec, we need to double its value to make it as interger number.
	 * When calculate them later we should remember to divide by two
	 */
	int	pre_div_map[8]   = { 2, 3, 4, 5, 6, 8, 12, 16 };
	int	div1to2p5_map[4] = { 2, 2, 4, 5 };

	int	m1_div_map[2]    = { 1, 2 };
	int	seld_map[4]      = { 1, 1, 4, 5 };
	int	divs_map[8]      = { 1, 2, 3, 4, 5, 6, 7, 8 };
	int	divm_map[8]      = { 1, 2, 3, 4, 5, 6, 7, 8 };
	int	divl_map[2]      = { 1, 1 };
	int	vco_freq;
	int	pre_div, div1to2p5;
	int	m1_div, divp, divs, seld, divm, divl;
	int sys_clk, vts, hts, frame_rate, mipi_clk, mipi_lane_clk;
	u8 val, v300f, v3010, v3011, v3012, v3029;

	ov5642_read(client, 0x300f, &v300f);
	ov5642_read(client, 0x3010, &v3010);
	ov5642_read(client, 0x3011, &v3011);
	ov5642_read(client, 0x3012, &v3012);
	ov5642_read(client, 0x3029, &v3029);

	/* get sensor system clk */
	/* vco frequency */
	val = v3012 & 0x7;
	pre_div = pre_div_map[val];
	val = v3011 & 0x3f;
	divp = val;
	val = v300f & 0x3;
	seld = seld_map[val];

	mclk = inter->pad;
	vco_freq = mclk * divp * seld * 2 / pre_div;
	dev_dbg(&client->dev, "vco_freq: %d\n", vco_freq);

	/* system clk */
	val = (v3010 & 0xf0) >> 4;
	divs = divs_map[val];
	val = v300f & 0x3;
	div1to2p5 = div1to2p5_map[val];
	val = v3029 & 0x1;
	m1_div = m1_div_map[val];

	sys_clk = vco_freq * 2 / div1to2p5 / divs / m1_div / 4;

	/* mipi clk */
	val = v3010 & 0xf;
	divm = divm_map[val];
	val = (v300f & 0x4) ? 1 : 0;
	divl = divl_map[val];

	mipi_clk = vco_freq / divs / divm / divl;

	/* mipi lane clk */
	mipi_lane_clk = mipi_clk / 2 / (divl + 1);
	inter->pad = mipi_lane_clk;
	dev_dbg(&client->dev, "MIPI clk: %d, SYS clk: %d, MIPI Clock Lane "
			"clk: %d\n", mipi_clk, sys_clk, mipi_lane_clk);

	/* get sensor hts & vts */
	ov5642_read(client, 0x380c, &val);
	hts = val & 0xf;
	hts <<= 8;
	ov5642_read(client, 0x380d, &val);
	hts += val & 0xff;
	ov5642_read(client, 0x380e, &val);
	vts = val & 0xf;
	vts <<= 8;
	ov5642_read(client, 0x380f, &val);
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

static struct v4l2_subdev_core_ops ov5642_subdev_core_ops = {
	.g_chip_ident = ov5642_g_chip_ident,
	.load_fw = ov5642_load_fw,
	.s_ctrl	= ov5642_s_ctrl,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ov5642_g_register,
	.s_register = ov5642_s_register,
#endif
};

static struct v4l2_subdev_video_ops ov5642_subdev_video_ops = {
	.s_stream = ov5642_s_stream,
	.g_mbus_fmt = ov5642_g_fmt,
	.s_mbus_fmt = ov5642_s_fmt,
	.try_mbus_fmt = ov5642_try_fmt,
	.enum_mbus_fsizes = ov5642_enum_fsizes,
	.enum_mbus_fmt = ov5642_enum_fmt,
	.g_frame_interval =  ov5642_g_frame_interval,
	.g_crop = ov5642_g_crop,
	.s_crop = ov5642_s_crop,
};

static struct v4l2_subdev_ops ov5642_subdev_ops = {
	.core = &ov5642_subdev_core_ops,
	.video = &ov5642_subdev_video_ops,
};

static int ov5642_video_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov5642 *ov5642 = to_ov5642(client);
	int ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = ov5642_detect(client);

	if (ret)
		goto out;
	dev_err(&client->dev, "OmniVision ov5642 sensor detected\n");

	ov5642->model = V4L2_IDENT_OV5642;

out:
	return ret;

}

static int ov5642_probe(struct i2c_client *client,
			    const struct i2c_device_id *did)
{
	struct ov5642 *ov5642;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int ret;

	g_i2c_client = client;

	if (!icd) {
		dev_err(&client->dev, "ov5642: missing soc-camera data!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	ov5642 = kzalloc(sizeof(struct ov5642), GFP_KERNEL);
	if (!ov5642)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&ov5642->subdev, client, &ov5642_subdev_ops);
	icd->ops = &ov5642_ops;
	ov5642->rect.left = 0;
	ov5642->rect.top = 0;
	ov5642->rect.width = 640;
	ov5642->rect.height = 480;
	ov5642->pixfmt = V4L2_PIX_FMT_YUV422P;

	ret = ov5642_video_probe(icd, client);
	if (ret) {
		icd->ops = NULL;
		kfree(ov5642);
	}

	return ret;
}

static int ov5642_remove(struct i2c_client *client)
{
	struct ov5642 *ov5642 = to_ov5642(client);
	struct soc_camera_device *icd = client->dev.platform_data;

	icd->ops = NULL;
	kfree(ov5642);
	return 0;
}

static const struct i2c_device_id ov5642_idtable[] = {
	{"ov5642", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ov5642_idtable);

static struct i2c_driver ov5642_driver = {
	.driver = {
		   .name = "ov5642",
		   },
	.probe = ov5642_probe,
	.remove = ov5642_remove,
	.id_table = ov5642_idtable,
};

static int __init ov5642_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&ov5642_driver);
	return ret;
}

static void __exit ov5642_mod_exit(void)
{
	i2c_del_driver(&ov5642_driver);
}

module_init(ov5642_mod_init);
module_exit(ov5642_mod_exit);
