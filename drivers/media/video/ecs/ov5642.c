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

static const struct v4l2_queryctrl ov5642_controls[] = {
	{
		.id = V4L2_CID_PRIVATE_FIRMWARE_DOWNLOAD,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "AF/FD etc. firmware download",
	}, {
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "brightness",
	},
};

static struct ov5642 *to_ov5642(const struct i2c_client
					     *client)
{
	struct x_subdev *xsd = container_of(i2c_get_clientdata(client), \
					struct x_subdev, subdev);
	return container_of(xsd, struct ov5642, xsd);
}
#if 0
static int ov5642_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char val[16];
	int i;

	for (i = 0; i < 4; ++i) {
		(*xic->read)(xic, SCAN_ARR_X_OFF_H+i, val+i);
		(*xic->read)(xic, ISP_INPUT_X_SZ_H+i, val+i+4);
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
	msleep(70);
	return 0;
}
#endif

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
	.query_bus_param	= ov5642_query_bus_param,
	.set_bus_param		= ov5642_set_bus_param,
	.controls			= ov5642_controls,
	.num_controls		= ARRAY_SIZE(ov5642_controls),
};

int ov5642_firmware_download(void *hw_ctx, const void *table, int size)
{
	int ret = 0, i;
	struct x_i2c *xic = hw_ctx;
	struct i2c_client *client = xic->client;
	const OV5642_FIRMWARE_ARRAY *firmware_regs = table;

	if (unlikely(size <= 0)) {
		dev_err(&client->dev, "No AF firmware available\n");
		return -ENODEV;
	}

	/* Actually start to download firmware */
	for (i = 0; i < size; i++) {
		ret = xic_write_burst_wb(xic, firmware_regs[i].reg_base, \
						firmware_regs[i].value, \
						firmware_regs[i].len);
		if (ret < 0) {
			dev_err(&client->dev, "i2c error %s %d\n",
				__func__, __LINE__);
			break;
		}
	}

	dev_info(&client->dev, "AF firmware downloaded\n");

	return size;
}
EXPORT_SYMBOL(ov5642_firmware_download);

static int select_bus_type(struct ov5642 *ov5642, const char *name)
{
	enum _ov5642_profile new_prof;
	int ret;
printk("ov5642: profile=%s\n", name);
	if (!strcmp(name, "generic")) {
		new_prof = GENERIC;
		goto exit;
	}
	if (!strcmp(name, "tavor-mipi")) {
		new_prof = PXA95X_SAARB;
		/* creat state, replace setting here */
		goto exit;
	}
	if (!strcmp(name, "DKB-mipi")) {
		new_prof = PXA97X_SAARC;
		/* creat state, replace setting here */
		goto exit;
	}
	if (!strcmp(name, "pxa2128-mipi")) {
		new_prof = PXA2128;
		/* creat state, replace setting here */
		goto exit;
	}
	return -EINVAL;
exit:
	if (new_prof == ov5642->profile) {
		printk(KERN_INFO "ov5642: profile name: %s\n", name);
		ecs_sensor_reset(ov5642->xsd.ecs);
		return 0;
	}

	strcpy(ov5642->name, name);
	strcat(ov5642->name, "-ov5642");
	ov5642->xsd.ecs = &generic_ov5642;

	/* Instantize to platform-specific driver */
	switch (new_prof) {
#ifdef CONFIG_PXA95x
	case PXA97X_SAARC:
		ret = ecs_sensor_merge(ov5642->xsd.ecs, &nevo_specific);
		if (ret < 0)
			return ret;
		/* Initialize ov5642 as a ecs subdev driver */
		ov5642->xsd.state_list = nevo_state_list;
		ov5642->xsd.state_cnt = ARRAY_SIZE(nevo_state_list);
		ecs_subdev_init(&ov5642->xsd);
		break;
#endif
	case PXA2128:
		ret = ecs_sensor_merge(ov5642->xsd.ecs, &mmp3_specific);
		if (ret < 0)
			return ret;
		/* Initialize ov5642 as a ecs subdev driver */
		ov5642->xsd.state_list = mmp3_state_list;
		ov5642->xsd.state_cnt = ARRAY_SIZE(mmp3_state_list);
		ecs_subdev_init(&ov5642->xsd);
		break;
	case PXA910_DVP:
		ret = ecs_sensor_merge(ov5642->xsd.ecs, &td_specific);
		if (ret < 0)
			return ret;
		/* Initialize ov5642 as a ecs subdev driver */
		ov5642->xsd.state_list = td_state_list;
		ov5642->xsd.state_cnt = ARRAY_SIZE(td_state_list);
		ecs_subdev_init(&ov5642->xsd);
		break;
	case GENERIC:
		ret = ecs_sensor_merge(ov5642->xsd.ecs, &mmp3_specific);
		if (ret < 0)
			return ret;
		/* Initialize ov5642 as a generic ecs driver */
		ecs_sensor_init(ov5642->xsd.ecs);
	default:
		return -EPERM;
	}

	ov5642->profile = new_prof;
	ecs_sensor_reset(ov5642->xsd.ecs);
	return 0;
}

static enum _ov5642_profile __attribute__((unused)) get_bus_type( \
	struct ov5642 *ov5642)
{
	return ov5642->profile;
}

static int __attribute__((unused)) ov5642_g_frame_interval( \
	struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *inter)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5642 *ov5642 = to_ov5642(client);
	struct x_i2c *xic = ov5642->xsd.ecs->hw_ctx;
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

	(*xic->read)(xic, 0x300f, &v300f);
	(*xic->read)(xic, 0x3010, &v3010);
	(*xic->read)(xic, 0x3011, &v3011);
	(*xic->read)(xic, 0x3012, &v3012);
	(*xic->read)(xic, 0x3029, &v3029);

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
	(*xic->read)(xic, 0x380c, &val);
	hts = val & 0xf;
	hts <<= 8;
	(*xic->read)(xic, 0x380d, &val);
	hts += val & 0xff;
	(*xic->read)(xic, 0x380e, &val);
	vts = val & 0xf;
	vts <<= 8;
	(*xic->read)(xic, 0x380f, &val);
	vts += val & 0xff;

	if (!hts || !vts)
		return -EINVAL;
	frame_rate = sys_clk * 1000000 / (hts * vts);
	ov5642->frame_rate = frame_rate;
	dev_dbg(&client->dev, "frame_rate: %d,"
			"hts:%x, vts:%x\n", frame_rate, hts, vts);

	inter->interval.numerator = frame_rate;
	inter->interval.denominator = 1;

	return 0;
}

static int ov5642_video_probe(struct soc_camera_device *icd,
				   struct i2c_client *client)
{
	struct ov5642 *ov5642 = to_ov5642(client);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	int ret = 0;

	/*
	 * We must have a parent by now. And it cannot be a wrong one.
	 * So this entire test is completely redundant.
	 */
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;
	ret = (*ov5642_xic.detect)(&ov5642_xic);

	if (ret)
		goto out;
	dev_err(&client->dev, "OmniVision ov5642 sensor detected\n");

#ifdef CONFIG_VIDEO_MV
	mv_set_sensor_attached(true);
#endif

	ov5642->xsd.model = V4L2_IDENT_OV5642;

	if (icl->priv == NULL) {
		dev_err(&client->dev, "ov5642 driver need know plat-sensor\n");
		return -EINVAL;
	}

	if (icl->flags & 0x80000000) {
#ifdef CONFIG_PXA95x
			/* priv is pointing to sensor_platform_data */
			struct sensor_platform_data *sensor = icl->priv;
			char name[32] = {0};

			strcpy(name, sensor->board_name);
			/* add postfix according to interface flag */
			icl->flags |= sensor->interface;
			if (icl->flags & SOCAM_MIPI)
				strcat(name, "-mipi");
			else
				strcat(name, "-dvp");

			ret = select_bus_type(ov5642, name);
#endif
	} else
		/* priv is pointing to ov5642 profile name */
		ret = select_bus_type(ov5642, icl->priv);

	if (ret) {
		dev_err(&client->dev, "need know interface type\n");
		return ret;
	}

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

	ov5642->xsd = ov5642_xsd;	/* setup xsd based on generic */
	v4l2_i2c_subdev_init(&ov5642->xsd.subdev, client, &ecs_subdev_ops);
	icd->ops = &ov5642_ops;
	ov5642->profile = PROFILE_END;
	ov5642_xic.client = v4l2_get_subdevdata(&ov5642->xsd.subdev);

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
