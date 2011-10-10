/* OmniVision OV8820 Sensor Driver
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * Based on mt9v011 -Micron 1/4-Inch VGA Digital Image Sensor
 *
 * Copyright (c) 2009 Mauro Carvalho Chehab (mchehab@redhat.com)
 * This code is placed under the terms of the GNU General Public License v2
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/videodev2.h>
#include <linux/delay.h>
#include <linux/mvisp.h>

#include <asm/div64.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/ov8820.h>
#include <mach/camera.h>

MODULE_DESCRIPTION("ov8820 sensor driver");
MODULE_AUTHOR("Henry Zhao <xzhao10@marvell.com>");
MODULE_LICENSE("GPL");

#define MAX_DETECT_NUM			3
#define MAX_OV8820_PADS_NUM		1

#define OV8820_PAD_SOURCE		0
#define SENSOR_OPEN 1
#define SENSOR_CLOSE 0

static const struct ov8820_datafmt ov8820_colour_fmts[] = {
	{V4L2_MBUS_FMT_UYVY8_1X16, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_JPEG_1X8, V4L2_COLORSPACE_JPEG},
	{V4L2_MBUS_FMT_RGB565_2X8_LE, V4L2_COLORSPACE_SRGB}
};

enum ov8820_output_type {
	OV8820_OUTPUT_NONE = 0x0,
	OV8820_OUTPUT_CCIC = 0x1,
};

struct ov8820_core {
	struct v4l2_subdev			sd;
	struct sensor_platform_data	*plat_data;
	struct media_pad pads[MAX_OV8820_PADS_NUM];
	struct v4l2_mbus_framefmt formats[MAX_OV8820_PADS_NUM];
	u32 width;
	u32 height;
	enum ov8820_output_type	output;
};

enum ov8820_register_access_e {
	OV8820_REG_INVALID = 0,
	OV8820_REG_READ,
	OV8820_REG_WRITE,
};

/* supported controls */
static struct v4l2_queryctrl ov8820_qctrl[] = {
	{
		.id = V4L2_CID_GAIN,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Gain",
		.minimum = 0,
		.maximum = (1 << 10) - 1,
		.step = 1,
		.default_value = 0x0020,
		.flags = 0,
	}, {
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Red Balance",
		.minimum = -1 << 9,
		.maximum = (1 << 9) - 1,
		.step = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Blue Balance",
		.minimum = -1 << 9,
		.maximum = (1 << 9) - 1,
		.step = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id      = V4L2_CID_HFLIP,
		.type    = V4L2_CTRL_TYPE_BOOLEAN,
		.name    = "Mirror",
		.minimum = 0,
		.maximum = 1,
		.step    = 1,
		.default_value = 0,
		.flags = 0,
	}, {
		.id      = V4L2_CID_VFLIP,
		.type    = V4L2_CTRL_TYPE_BOOLEAN,
		.name    = "Vflip",
		.minimum = 0,
		.maximum = 1,
		.step    = 1,
		.default_value = 0,
		.flags = 0,
	}, {
	}
};



static inline struct ov8820_core *to_ov8820_core(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov8820_core, sd);
}

static int ov8820_read(struct v4l2_subdev *sd, u16 reg, unsigned char *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 data;
	u8 address[2];

	address[0] = reg>>8;
	address[1] = reg;
	ret = i2c_smbus_write_byte_data(client, address[0], address[1]);
	if (ret)
		return ret;
	data = i2c_smbus_read_byte(client);

	*value = data;

	return 0;
}

static int ov8820_write(struct v4l2_subdev *sd, u16 reg, unsigned char value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 data[3];

	data[0] = reg >> 8;
	data[1] = reg;
	data[2] =  value;
	ret = i2c_master_send(client, data, 3);
	if (ret < 0)
		return ret;
	if (reg == REG_RESET && (value == RESET_ACT))
		msleep(20); /* Wait for reset to run */

	return 0;
}

static int ov8820_detect(struct v4l2_subdev *sd)
{
	unsigned char v = 0;
	int ret = 0;

	ret = ov8820_read(sd, REG_PIDH, &v);
	if (ret < 0) {
		printk(KERN_NOTICE "cam: ov8820: Sensor not mounted / I2C address error\n");
		return -ENXIO;
	}
	if (v != 0x88) {
		printk(KERN_ERR "cam: ov8820: Not a OV8820 sensor: ID_HIGH = 0x%X\n"
			, (unsigned char)v);
		return -ENODEV;
	}
	ret = ov8820_read(sd, REG_PIDM, &v);
	if (ret < 0) {
		printk(KERN_NOTICE "cam: ov8820: Sensor not mounted / I2C address error\n");
		return -ENXIO;
	}
	if (v != 0x20) {
		printk(KERN_ERR "cam: ov8820: Not a OV8820 sensor: ID_LOW = 0x%X\n"
			, (unsigned char)v);
		return -ENODEV;
	} else
		printk(KERN_ERR "cam: ov8820: sensor detected!\n");

	return 0;
}

static int ov8820_reset(struct v4l2_subdev *sd, u32 val)
{
	int ret = 0;

	ret = ov8820_write(sd, REG_RESET, RESET_ACT);
	printk(KERN_NOTICE "cam: ov8820: Reset sensor\n");
	msleep(20);
	ret |= ov8820_write(sd, REG_RESET, RESET_DIS);
	return (ret < 0);
};

static int ov8820_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8820_core *core = to_ov8820_core(sd);

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		return 0;
	case V4L2_CID_RED_BALANCE:
		return 0;
	case V4L2_CID_BLUE_BALANCE:
		return 0;
	case V4L2_CID_HFLIP:
		return 0;
	case V4L2_CID_VFLIP:
		return 0;
	}
	return -EINVAL;
}

static int ov8820_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov8820_qctrl); i++)
		if (qc->id && qc->id == ov8820_qctrl[i].id) {
			memcpy(qc, &(ov8820_qctrl[i]),
			       sizeof(*qc));
			return 0;
		}

	return -EINVAL;
}


static int ov8820_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov8820_core *core = to_ov8820_core(sd);
	u8 i, n;
	n = ARRAY_SIZE(ov8820_qctrl);

	for (i = 0; i < n; i++) {
		if (ctrl->id != ov8820_qctrl[i].id)
			continue;
		if (ctrl->value < ov8820_qctrl[i].minimum ||
		    ctrl->value > ov8820_qctrl[i].maximum)
			return -ERANGE;
		break;
	}

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		break;
	case V4L2_CID_RED_BALANCE:
		break;
	case V4L2_CID_BLUE_BALANCE:
		break;
	case V4L2_CID_HFLIP:
		return 0;
	case V4L2_CID_VFLIP:
		return 0;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ov8820_try_format(struct v4l2_subdev *sd,
		struct v4l2_mbus_framefmt *fmt)
{
	struct ov8820_core *core = to_ov8820_core(sd);

	fmt->field = V4L2_FIELD_NONE;

	switch (fmt->code) {
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		fmt->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	case V4L2_MBUS_FMT_JPEG_1X8:
		fmt->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		printk(KERN_ERR "ov8820 doesn't support code 0x%08X\n"
			, fmt->code);
		break;
	}
	return 0;
}

static int ov8820_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad > 0)
		return -EINVAL;

	if (code->index >= ARRAY_SIZE(ov8820_colour_fmts))
		return -EINVAL;

	code->code = ov8820_colour_fmts[code->index].code;

	return 0;
}

static int ov8820_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ov8820_try_format(sd, &format);

	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;

	ov8820_try_format(sd, &format);
	fse->max_width = format.width;
	fse->max_height = format.height;
	return 0;
}

static int ov8820_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned index,
					enum v4l2_mbus_pixelcode *code)
{
	if (index > 0)
		return -EINVAL;

	*code = V4L2_MBUS_FMT_SGRBG8_1X8;
	return 0;
}

static int ov8820_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
#if 0
	if (!enable)
		ret = ov8820_write(sd, 0x0100, 0x00);
#endif
	printk(KERN_ERR "ov8820_s_stream %d\n", enable);
	return ret;
}

static int ov8820_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(struct v4l2_captureparm));
	cp->capability = V4L2_CAP_TIMEPERFRAME;

	return 0;
}

static int ov8820_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct v4l2_captureparm *cp = &parms->parm.capture;
	struct v4l2_fract *tpf = &cp->timeperframe;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (cp->extendedmode != 0)
		return -EINVAL;

	return 0;
}

static struct v4l2_mbus_framefmt *__ov8820_get_format(
			struct ov8820_core *core, struct v4l2_subdev_fh *fh,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &core->formats[pad];
}

static int ov8820_set_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov8820_core *core = to_ov8820_core(sd);
	struct v4l2_mbus_framefmt *format;
	int ret;

	format = __ov8820_get_format(core, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	if (fmt->pad > 0)
		return -EINVAL;

	ret = ov8820_try_format(sd, &fmt->format);
	if (ret < 0)
		return -EINVAL;

	*format = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY) {
		core->width = fmt->format.width;
		core->height = fmt->format.height;
	}

	return 0;
}

static int ov8820_get_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov8820_core *core = to_ov8820_core(sd);
	struct v4l2_mbus_framefmt *format;

	format = __ov8820_get_format(core, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int ov8820_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	u16 version = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return v4l2_chip_ident_i2c_client(client, chip,
					V4L2_IDENT_OV8820,
					version);
}

static long ov8820_get_driver_name(struct v4l2_subdev *sd,
		struct v4l2_sensor_get_driver_name *drv_name)
{
	if (!drv_name)
		return -EINVAL;

	strcpy(drv_name->driver, "ov8820");
	return 0;
}

static long ov8820_register_access(struct v4l2_subdev *sd,
			struct v4l2_sensor_register_access *param,
			enum ov8820_register_access_e access_type)
{
	int ret = -EINVAL;

	switch (access_type) {
	case OV8820_REG_READ:
		ret = ov8820_read(sd, param->reg,
			(unsigned char *)&param->value);
		break;
	case OV8820_REG_WRITE:
		ret = ov8820_write(sd, param->reg,
			(unsigned char) param->value);
		break;
	default:
		break;
	}

	return ret;
}

static long ov8820_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case VIDIOC_PRIVATE_SENSER_REGISTER_GET:
		ret = ov8820_register_access(sd,
			(struct v4l2_sensor_register_access *)arg,
			OV8820_REG_READ);
		break;
	case VIDIOC_PRIVATE_SENSER_REGISTER_SET:
		ret = ov8820_register_access(sd,
			(struct v4l2_sensor_register_access *)arg,
			OV8820_REG_WRITE);
		break;
	case VIDIOC_PRIVATE_SENSER_GET_DRIVER_NAME:
		ret = ov8820_get_driver_name(sd,
			(struct v4l2_sensor_get_driver_name *)arg);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return ret;
}

static int ov8820_subdev_open(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	return 0;
}

static int ov8820_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct ov8820_core *core = to_ov8820_core(sd);

	switch (local->index | media_entity_type(remote->entity)) {
	case OV8820_PAD_SOURCE | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			core->output |= OV8820_OUTPUT_CCIC;
		else
			core->output &= ~OV8820_OUTPUT_CCIC;
		break;
	default:
		/* Link from camera to CCIC is fixed... */
		return -EINVAL;
	}
	return 0;
}

static const struct v4l2_subdev_pad_ops ov8820_pad_ops = {
	.enum_mbus_code = ov8820_enum_mbus_code,
	.enum_frame_size = ov8820_enum_frame_size,
	.get_fmt = ov8820_get_format,
	.set_fmt = ov8820_set_format,
};

static const struct v4l2_subdev_core_ops ov8820_core_ops = {
	.ioctl = ov8820_subdev_ioctl,
	.queryctrl = ov8820_queryctrl,
	.g_ctrl = ov8820_g_ctrl,
	.s_ctrl = ov8820_s_ctrl,
	.reset = ov8820_reset,
	.g_chip_ident = ov8820_g_chip_ident,
};

static const struct v4l2_subdev_video_ops ov8820_video_ops = {
	.enum_mbus_fmt = ov8820_enum_mbus_fmt,
	.g_parm = ov8820_g_parm,
	.s_parm = ov8820_s_parm,
	.s_stream = ov8820_s_stream,
};

static const struct v4l2_subdev_ops ov8820_ops = {
	.core  = &ov8820_core_ops,
	.video = &ov8820_video_ops,
	.pad = &ov8820_pad_ops,
};

/* subdev internal operations */
static const struct v4l2_subdev_internal_ops ov8820_v4l2_internal_ops = {
	.open = ov8820_subdev_open,
};

/* media operations */
static const struct media_entity_operations ov8820_media_ops = {
	.link_setup = ov8820_link_setup,
};


/****************************************************************************
			I2C Client & Driver
 ****************************************************************************/

static int ov8820_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int i, ret;
	struct ov8820_core *core;
	struct v4l2_subdev *sd;
	struct sensor_platform_data *pdata = client->dev.platform_data;

	struct media_entity *me;
	struct media_pad *pads;

	if (!pdata)
		return -EINVAL;

	/* Check if the adapter supports the needed features */
	if (!i2c_check_functionality(client->adapter,
	     I2C_FUNC_SMBUS_READ_BYTE | I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
		return -EIO;

	core = kzalloc(sizeof(struct ov8820_core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	if (pdata->power_on) {
		ret = pdata->power_on(SENSOR_OPEN, pdata->id);
		msleep(20);
	}

	sd = &core->sd;
	v4l2_i2c_subdev_init(sd, client, &ov8820_ops);
	sd->internal_ops = &ov8820_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	pads = core->pads;

	me = &sd->entity;
	pads[OV8820_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	me->ops = &ov8820_media_ops;
	ret = media_entity_init(me, MAX_OV8820_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;
	/* Check if the sensor is really a OV8820 */
	for (i = MAX_DETECT_NUM; i > 0; --i) {
		ret = ov8820_detect(sd);
		if (!ret)
			break;

		if (ret == -ENXIO) {
			if (pdata->power_on)
				pdata->power_on(SENSOR_CLOSE, pdata->id);
			return ret;
		} else {
			printk(KERN_ERR \
				"camera: OV8820 sensor detect failure, will retry %d times!\n"
				, i);
		}
	}

	if (ret) {
		if (pdata->power_on)
			pdata->power_on(SENSOR_CLOSE, pdata->id);
		return ret;
	}

	core->plat_data = pdata;
	return 0;
}

static int ov8820_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct sensor_platform_data *pdata = client->dev.platform_data;

	if (pdata)
		if (pdata->power_on)
			pdata->power_on(SENSOR_CLOSE, pdata->id);

	v4l2_device_unregister_subdev(sd);
	kfree(to_ov8820_core(sd));
	return 0;
}

/* ----------------------------------------------------------------------- */

static const struct i2c_device_id ov8820_id[] = {
	{ "ov8820", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov8820_id);

static struct i2c_driver ov8820_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ov8820",
	},
	.probe		= ov8820_probe,
	.remove		= ov8820_remove,
	.id_table	= ov8820_id,
};

static __init int init_ov8820(void)
{
	return i2c_add_driver(&ov8820_driver);
}

static __exit void exit_ov8820(void)
{
	i2c_del_driver(&ov8820_driver);
}

module_init(init_ov8820);
module_exit(exit_ov8820);
