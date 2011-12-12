/*
 * linux/drivers/media/video/m6mo.c - Fujitsu M6MO Image Signal Processor driver
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

#include "m6mo.h"

#define	DRV_NAME	"m6mo"

static const struct m6mo_datafmt m6mo_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct m6mo_format_struct {
	enum v4l2_mbus_pixelcode	code;
	struct regval_list	*regs;
} m6mo_fmts[] = {
	{
		.code	=	V4L2_MBUS_FMT_UYVY8_2X8,
		.regs	=	NULL,
	},
};

struct m6mo_win_size {
	int	width;
	int	height;
	struct regval_list *regs;
	int	timing_id;
};

static struct mipi_phy m6mo_timings[] = {
	{ /* M6MO default timing */
		.cl_termen	= 0x00,
		.cl_settle	= 0x0C,
		.hs_termen	= 0x08,
		.hs_settle	= 0x20,
		.hs_rx_to	= 0xFFFF,
		.lane		= 2,
	},
};
#define N_M6MO_TIMINGS ARRAY_SIZE(m6mo_timings)

static struct m6mo_win_size m6mo_sizes[] = {
	{
		.width	= 640,
		.height	= 480,
		.regs	= NULL,	/* VGA */
	},
	{
		.width	= 1280,
		.height	= 720,
		.regs	= NULL,	/* 720p */
	},
#if 0	/* Not supportting 1080p now, needs follow up*/
	{
		.width	= 1920,
		.height	= 1080,
		.regs	= NULL,	/* 1080p */
	},
#endif
};

#define N_M6MO_SIZES (ARRAY_SIZE(m6mo_sizes))
#define N_M6MO_FMTS (ARRAY_SIZE(m6mo_fmts))

static struct m6mo_info *to_info(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), \
			struct m6mo_info, subdev);
}

static inline int m6mo_power(struct m6mo_info *info, int onoff)
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
	printk(KERN_ERR "cam: m6mo: Can't find platform data\n");
	return -EINVAL;
}

/***************************** I2C I/O functions ******************************/
static inline int m6mo_read_n(const struct i2c_client *c, \
			u16 reg, void *buffer, u8 len)
{
	u8 w_buf[5] = {5, ISP_REG_READ, WD_BYTES(reg), len};
	u8 r_buf[BUFFER_DEPTH+1];
	struct i2c_msg msg[2] = {
		{c->addr, c->flags, 5, w_buf},
		{c->addr, c->flags | I2C_M_RD, len+1, r_buf},
	};
	int ret = 0;

	if (unlikely((buffer == NULL) || (len > BUFFER_DEPTH)))
		return -EINVAL;

	ret = i2c_transfer(c->adapter, msg, ARRAY_SIZE(msg));
	if (unlikely((ret < 0) || (r_buf[0] != len+1))) {
		printk(KERN_ERR "cam: m6mo: read error code: 0x%02x\n ", \
			r_buf[0]);
		ret = -EIO;
		goto exit;
	}
	memcpy(buffer, r_buf+1, len);
exit:
	return ret;
}

static inline int m6mo_read(const struct i2c_client *c, u16 reg, void *value)
{
	return m6mo_read_n(c, reg, value, 1);
}

static int m6mo_write(struct i2c_client *c, u16 reg, u8 value)
{
	u8 cmd_buf[5] = {5, ISP_REG_WRITE, WD_BYTES(reg), value};

	return i2c_master_send(c, cmd_buf, 5);
}

static int m6mo_write_n(struct i2c_client *c, u16 reg, void *buffer, u8 len)
{
	/* FIXME: maybe better to change to block write */
	u8 w_buf[4+BUFFER_DEPTH] = {4+len, ISP_REG_WRITE, WD_BYTES(reg)};

	if (unlikely((buffer == NULL) || (len > BUFFER_DEPTH)))
		return -EINVAL;

	memcpy(w_buf+4, buffer, len);
	return i2c_master_send(c, w_buf, 4+len);
}

static inline int m6mo_reset(struct i2c_client *client)
{
	struct m6mo_info *info = to_info(client);

	printk(KERN_NOTICE "cam: m6mo: reset isp\n");
	m6mo_power(info, SENSOR_CLOSE);
	m6mo_power(info, SENSOR_OPEN);
	m6mo_write(client, CAM_START, 1);
	/* Wait here to let ISP start up */
	msleep(30);
	return 0;
}

static __attribute__ ((unused)) int m6mo_read_sensor(struct i2c_client *client, u16 addr, u8 *val)
{
	/* Send sensor 2-BYTE addr to ISP */
	u8 addr_buf[2] = {WD_BYTES(addr)};
	m6mo_write_n(client, SENSOR_REG_ADRH, &addr_buf, 2);
	m6mo_write(client, SENSOR_REG_RW, RW_READ);
	msleep(1);
	return m6mo_read(client, SENSOR_REG_DATA, val);
}

static __attribute__ ((unused)) int m6mo_write_sensor(struct i2c_client *client, u16 addr, u8 val)
{
	/* Send sensor 2-BYTE addr to ISP */
	u8 addr_buf[2] = {WD_BYTES(addr)};
	m6mo_write_n(client, SENSOR_REG_ADRH, &addr_buf, 2);
	m6mo_write(client, SENSOR_REG_DATA, val);
	return m6mo_write(client, SENSOR_REG_RW, RW_WRITE);
}

static int m6mo_detect(struct i2c_client *client)
{
	u8 v[6];

	m6mo_write(client, CAM_START, 1);
	msleep(10);
	m6mo_read_n(client, SYSP_CUSTOMER_CODE, v, 6);
	printk(KERN_NOTICE "cam: m6mo: magic_code: 0x%02X%02X, " \
		"firmware_ver: 0x%02X%02X, hardware_ver: 0x%02X%02X\n", \
		v[0], v[1], v[2], v[3], v[4], v[5]);

	m6mo_read(client, SYSP_STATUS, v);
	if (unlikely(v[0] != STAT_SETIN)) {
		printk(KERN_ERR "cam: m6mo: ISP status error: 0x%02X, " \
				"failed to initialize\n", v[0]);
		return -EBUSY;
	}
	return 0;
}

/* FIXME: only setup framework now, need convert power sequence to setting
 * in future */
static __attribute__ ((unused)) int m6mo_operation_list(struct i2c_client *c, \
				const struct isp_oper *list)
{
	const struct isp_oper *op = list;
	struct m6mo_info *info = to_info(c);
	int ret = 0, tmp;

	if (unlikely(list == NULL))
		return -EINVAL;
	while (1) {
		switch (op->oper) {
		case OPER_NOOP:
			break;
		case OPER_RESET:
			ret = m6mo_reset(c);
			break;
		case OPER_READ:
			ret = m6mo_read(c, op->addr, &tmp);
			break;
		case OPER_WRITE:
			ret = m6mo_write(c, op->addr, op->data);
			break;
		case OPER_SET:
			ret = m6mo_read(c, op->addr, &tmp);
			tmp |= op->data;
			ret |= m6mo_write(c, op->addr, tmp);
			break;
		case OPER_CLEAR:
			ret = m6mo_read(c, op->addr, &tmp);
			tmp &= (~op->data);
			ret |= m6mo_write(c, op->addr, tmp);
			break;
		case OPER_SLEEP:
			tmp = (op->addr << (sizeof(ISP_REG_ADDR_TYPE)*8)) \
				+ op->data;
			msleep(tmp);
			break;
		case OPER_WAIT_INT:
#ifdef ENABLE_ISP_IRQ
#else
			msleep(100);
#endif
			break;
		case OPER_TRIGGER:
			ret = m6mo_power(info, ISP_SENSOR_CLOSE);
			msleep(100);
			ret |= m6mo_power(info, ISP_SENSOR_OPEN);
			msleep(100);
			break;
		default:
			printk(KERN_INFO "cam: m6mo: end of list 0x%08X\n", \
				(__u32)list);
			return ret;
		}
		if (ret < 0)
			goto err;
		op++;
	}
err:
	return ret;
}


/***************************** I2C I/O functions ******************************/
static int m6mo_get_awb(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int m6mo_set_awb(struct i2c_client *client, int value)
{
	return 0;
}

static int m6mo_get_ae(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int m6mo_set_ae(struct i2c_client *client, int value)
{
	return 0;
}

static int m6mo_get_af(struct i2c_client *client, __s32 *value)
{
	return 0;
}

static int m6mo_set_af(struct i2c_client *client, int value)
{
	return 0;
}

static int m6mo_get_mipi_phy(struct i2c_client *client, __s32 *value)
{
	if (unlikely((void *)value == NULL))
		return -EPERM;
	/* Camera driver provide a address to fill in timing info */
	*value = (__s32)&(m6mo_timings[0]);
	return 0;
}

static const struct v4l2_queryctrl m6mo_controls[] = {
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

static unsigned long m6mo_query_bus_param(struct soc_camera_device *icd)
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

static int m6mo_set_bus_param(struct soc_camera_device *icd, unsigned long f)
{
#if 0/*TODO: add mipi and parallel different setting*/
	if (f & SOCAM_MIPI) /* mipi setting*/
		m6mo_write_array(client, m6mo_mipi);
	else /* parallel setting*/
		m6mo_write_array(client, m6mo_mipi);
#endif
	return 0;
}

static struct soc_camera_ops m6mo_ops = {
	.query_bus_param	= m6mo_query_bus_param,
	.set_bus_param		= m6mo_set_bus_param,
	.controls		= m6mo_controls,
	.num_controls		= ARRAY_SIZE(m6mo_controls),
};

static int m6mo_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct v4l2_queryctrl *qctrl;
	int ret;

	qctrl = soc_camera_find_qctrl(&m6mo_ops, ctrl->id);
	if (!qctrl)
		return -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = m6mo_set_awb(client, ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = m6mo_set_af(client, ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = m6mo_set_ae(client, ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int m6mo_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		ret = m6mo_get_awb(client, &ctrl->value);
		break;
	case V4L2_CID_FOCUS_AUTO:
		ret = m6mo_get_af(client, &ctrl->value);
		break;
	case V4L2_CID_EXPOSURE_AUTO:
		ret = m6mo_get_ae(client, &ctrl->value);
		break;
	case V4L2_CID_PRIVATE_GET_MIPI_PHY:
		ret = m6mo_get_mipi_phy(client, &ctrl->value);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int m6mo_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *id)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = to_info(client);

	id->ident	= info->model;
	id->revision	= 0;

	return 0;
}

static int m6mo_g_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	return 0;
}

static int m6mo_s_register(struct v4l2_subdev *sd,
			      struct v4l2_dbg_register *reg)
{
	return 0;
}

static int m6mo_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	if (enable)
		/* TODO: add streamon setting here */
		;
	else {
		/* FIXME: need to contact fujitsu for streamoff regs */
		m6mo_reset(client);
		msleep(50);
	}
	return 0;
}

static int m6mo_g_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = to_info(client);

	mf->width	= info->rect.width;
	mf->height	= info->rect.height;
	mf->code	= V4L2_MBUS_FMT_UYVY8_2X8;
	mf->field	= V4L2_FIELD_NONE;
	mf->colorspace	= V4L2_COLORSPACE_JPEG;
	return 0;
}

static int m6mo_s_fmt(struct v4l2_subdev *sd,
			 struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = to_info(client);
	int ret = 0;
	struct i2c_client *c = v4l2_get_subdevdata(sd);
	unsigned char res, fps;

	switch (mf->code) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		switch (mf->width) {
		case 1280:
			res = 0x21;
			fps = 0x02;
			break;
		case 1920:
			res = 0x28;
			fps = 0x02;
			break;
		default:
			/* Default value to be VGA @ 30FPS */
			res = 0x17;
			fps = 0x02;
		}
		m6mo_reset(c);
		m6mo_write(c, 0x0100, 0x02);	/* MIPI output */
		m6mo_write(c, 0x0101, res);	/* resolution */
		m6mo_write(c, 0x0102, fps);	/* FPS */

		m6mo_write(c, 0x0104, 0x01);
		m6mo_write(c, 0x0107, 0x00);

		m6mo_write(c, 0x0306, 0x01);
		m6mo_write(c, 0x0309, 0x1E);

		m6mo_write(c, 0x012D, 0x00);
		m6mo_write(c, 0x0010, 0x00);
		m6mo_write(c, 0x000B, 0x02);
		msleep(40);

		m6mo_power(info, ISP_SENSOR_CLOSE);
		m6mo_power(info, ISP_SENSOR_OPEN);

		m6mo_write(c, 0x0A00, 0x02);
		m6mo_write(c, 0x0A16, 0x01);
		m6mo_write(c, 0x0A02, 0x01);
		printk(KERN_INFO "cam: m6mo: set format YUV(%d, %d) done\n", \
			mf->width, mf->height);
		break;

	default:
		printk(KERN_ERR "cam: not supported fmt!\n");
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int m6mo_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			    enum v4l2_mbus_pixelcode *code)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = to_info(client);

	if (index >= info->num_fmts)
		return -EINVAL;

	*code = info->fmts[index].code;
	return 0;
}

static int m6mo_try_fmt(struct v4l2_subdev *sd,
			   struct v4l2_mbus_framefmt *mf)
{
	int i;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = to_info(client);

	/* enum the supported formats*/
	for (i = 0; i < N_M6MO_FMTS; i++) {
		if (m6mo_fmts[i].code == mf->code) {
			info->regs_fmt = m6mo_fmts[i].regs;
			break;
		}
	}
	if (i >= N_M6MO_FMTS) {
		printk(KERN_ERR "cam: m6mo unsupported color format!\n");
		return -EINVAL;
	}

	/* enum the supported sizes*/
	for (i = 0; i < N_M6MO_SIZES; i++) {
		if (mf->width == m6mo_sizes[i].width
			&& mf->height == m6mo_sizes[i].height) {
			info->regs_size = m6mo_sizes[i].regs;
			break;
		}
	}
	if (i >= N_M6MO_SIZES) {
		printk(KERN_ERR "cam: m6mo unsupported window size, " \
				"w%d, h%d!\n", mf->width, mf->height);
		return -EINVAL;
	}

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
	return 0;
}

static int m6mo_enum_fsizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!fsize)
		return -EINVAL;

	switch (fsize->pixel_format) {
	case V4L2_MBUS_FMT_UYVY8_2X8:
		if (fsize->index >= N_M6MO_SIZES)
			return -EINVAL;
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.height = m6mo_sizes[fsize->index].height;
		fsize->discrete.width = m6mo_sizes[fsize->index].width;
		break;
	default:
		dev_err(&client->dev, "ov5642 unsupported format!\n");
		return -EINVAL;
	}
	return 0;
}

static int sensor_init(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m6mo_info *info = i2c_get_clientdata(client);

	m6mo_reset(client);

	m6mo_write(client, 0x0100, 0x02);	/* MIPI output */
	m6mo_write(client, 0x0101, 0x17);	/* resolution */
	m6mo_write(client, 0x0102, 0x02);	/* 30FPS */
	m6mo_write(client, 0x0106, 0x01);
	m6mo_write(client, 0x0109, 0x00);

	m6mo_write(client, 0x0304, 0x01);
	m6mo_write(client, 0x0307, 0x01);

	m6mo_write(client, 0x012D, 0x00);
	m6mo_write(client, 0x0010, 0x00);
	m6mo_write(client, 0x000B, 0x02);
	msleep(100);

	m6mo_power(info, ISP_SENSOR_CLOSE);
	msleep(100);
	m6mo_power(info, ISP_SENSOR_OPEN);
	msleep(100);

	m6mo_write(client, 0x0A16, 0x01);
	m6mo_write(client, 0x0A02, 0x01);

	printk(KERN_NOTICE "cam: m6mo initialized for YUV\n");
	return 0;
}

static struct v4l2_subdev_core_ops m6mo_subdev_core_ops = {
	.g_ctrl		= m6mo_g_ctrl,
	.s_ctrl		= m6mo_s_ctrl,
	.g_chip_ident	= m6mo_g_chip_ident,
	.load_fw	= sensor_init,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= m6mo_g_register,
	.s_register	= m6mo_s_register,
#endif
};

static struct v4l2_subdev_video_ops m6mo_subdev_video_ops = {
	.s_stream	= m6mo_s_stream,
	.s_mbus_fmt	= m6mo_s_fmt,
	.g_mbus_fmt	= m6mo_g_fmt,
	.try_mbus_fmt	= m6mo_try_fmt,
	.enum_mbus_fmt	= m6mo_enum_fmt,
	.enum_mbus_fsizes = m6mo_enum_fsizes,
};

static struct v4l2_subdev_ops m6mo_subdev_ops = {
	.core	= &m6mo_subdev_core_ops,
	.video	= &m6mo_subdev_video_ops,
};

static int m6mo_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct m6mo_info *info;
	struct soc_camera_device *icd = client->dev.platform_data;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct soc_camera_link *icl;
	int ret;

	if (!icd) {
		dev_err(&client->dev, "m6mo missing soc-camera data!\n");
		return -EINVAL;
	}
	if (!icd->dev.parent ||
	    to_soc_camera_host(icd->dev.parent)->nr != icd->iface)
		return -ENODEV;

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "m6mo driver needs platform data\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_warn(&adapter->dev,
			 "I2C-Adapter doesn't support I2C_FUNC_SMBUS_WORD\n");
		return -EIO;
	}

	info = kzalloc(sizeof(struct m6mo_info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "m6mo failed to alloc struct!\n");
		return -ENOMEM;
	}

	info->i2c_client = client;
	info->power = icl->power;

	info->rect.left = 0;
	info->rect.top = 0;
	info->rect.width = 640;
	info->rect.height = 480;
	info->pixfmt = V4L2_PIX_FMT_UYVY;

	icd->ops = &m6mo_ops;

	info->model = V4L2_IDENT_M6MO;
	info->fmts = m6mo_colour_fmts;
	info->num_fmts = ARRAY_SIZE(m6mo_colour_fmts);

	v4l2_i2c_subdev_init(&info->subdev, client, &m6mo_subdev_ops);

	ret = m6mo_detect(client);
	if (!ret) {
		printk(KERN_NOTICE "cam: Fujitsu m6mo sensor detected!\n");
		return 0;
	}
	printk(KERN_ERR "cam: failed to detect Fujitsu m6mo!\n");

	icd->ops = NULL;
	i2c_set_clientdata(client, NULL);
	if (info)
		kfree(info);

	return -ENODEV;;
}

static int m6mo_remove(struct i2c_client *client)
{
	struct m6mo_info *info = to_info(client);
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

static struct i2c_device_id m6mo_idtable[] = {
	{ DRV_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, m6mo_idtable);

static struct i2c_driver m6mo_driver = {
	.driver = {
		.name	= DRV_NAME,
	},
	.id_table	= m6mo_idtable,
	.probe		= m6mo_probe,
	.remove		= m6mo_remove,
};

static int __init m6mo_mod_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&m6mo_driver);
	return ret;
}

static void __exit m6mo_mod_exit(void)
{
	i2c_del_driver(&m6mo_driver);
}

module_init(m6mo_mod_init);
module_exit(m6mo_mod_exit);

MODULE_DESCRIPTION("Fujitsu M6MO Image Signal Processor");
MODULE_AUTHOR("Jiaquan Su");
MODULE_LICENSE("GPL");
