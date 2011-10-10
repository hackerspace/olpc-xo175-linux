/*
* ispccic.c
*
* Marvell DxO ISP - CCIC module
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

#include <linux/delay.h>
#include <media/v4l2-common.h>
#include <linux/v4l2-mediabus.h>
#include <linux/mm.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccic.h"

static const unsigned int ccic_input_fmts[] = {
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_YUYV8_2X8,
};

static const unsigned int ccic_output_fmts[] = {
	V4L2_MBUS_FMT_UYVY8_1X16,
};

static void ccic_dump_regs(struct mvisp_device *isp)
{
	unsigned long regval;

	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_Y0_BASE_ADDR);
	dev_warn(isp->dev, "ccic_set_stream Y0: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_Y1_BASE_ADDR);
	dev_warn(isp->dev, "ccic_set_stream Y1: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_Y2_BASE_ADDR);
	dev_warn(isp->dev, "ccic_set_stream Y2: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_RAW_STATUS);
	dev_warn(isp->dev, "ccic_set_stream IRQRAWSTATE: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_STATUS);
	dev_warn(isp->dev, "ccic_set_stream IRQSTATE: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_MASK);
	dev_warn(isp->dev, "ccic_set_stream IRQMASK: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
	dev_warn(isp->dev, "ccic_set_stream CTRL0: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_1);
	dev_warn(isp->dev, "ccic_set_stream CTRL1: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CLOCK_CTRL);
	dev_warn(isp->dev, "ccic_set_stream CLKCTRL: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_IRQ_RAW_STATUS);
	dev_warn(isp->dev, "ccic_set_stream MIPI STATUS: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY3);
	dev_warn(isp->dev, "ccic_set_stream MIPI DPHY3: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY5);
	dev_warn(isp->dev, "ccic_set_stream MIPI DPHY5: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY6);
	dev_warn(isp->dev, "ccic_set_stream MIPI DPHY6: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_IMG_SIZE);
	dev_warn(isp->dev, "ccic_set_stream SIZE: 0x%08lX\n", regval);
	regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_IMG_PITCH);
	dev_warn(isp->dev, "ccic_set_stream PITCH: 0x%08lX\n", regval);
}

static void ccic_set_dma_addr(struct isp_ccic_device *ccic,
	struct isp_video_buffer *buffer, enum isp_ccic_irq_type irqeof)
{
	struct mvisp_device *isp = ccic->isp;

	switch (irqeof) {
	case CCIC_EOF0:
		mvisp_reg_writel(isp, buffer->paddr,
			CCIC_ISP_IOMEM_1, CCIC_Y0_BASE_ADDR);
		break;
	case CCIC_EOF1:
		mvisp_reg_writel(isp, buffer->paddr,
			CCIC_ISP_IOMEM_1, CCIC_Y1_BASE_ADDR);
		break;
	case CCIC_EOF2:
		mvisp_reg_writel(isp, buffer->paddr,
			CCIC_ISP_IOMEM_1, CCIC_Y2_BASE_ADDR);
		break;
	default:
		break;
	}

	return;
}

static int ccic_clear_mipi(struct isp_ccic_device *ccic)
{
	struct mvisp_device *isp = ccic->isp;
	unsigned long mipi_lock_flags;

	spin_lock_irqsave(&ccic->mipi_flag_lock, mipi_lock_flags);
	if (ccic->mipi_config_flag == MIPI_NOT_SET) {
		spin_unlock_irqrestore(&ccic->mipi_flag_lock, mipi_lock_flags);
		return 0;
	}

	switch (ccic->ccic_id) {
	case CCIC_ID_1:
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY3);
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY5);
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY6);
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_CSI2_CTRL0);
		break;
	case CCIC_ID_2:
		break;
	default:
		break;
	}

	ccic->mipi_config_flag = MIPI_NOT_SET;
	spin_unlock_irqrestore(&ccic->mipi_flag_lock, mipi_lock_flags);
	return 0;
}

static int ccic_configure_mipi(struct isp_ccic_device *ccic)
{
	struct mvisp_device *isp = ccic->isp;
	unsigned long mipi_lock_flags;
	bool valid_sensor;

	spin_lock_irqsave(&ccic->mipi_flag_lock, mipi_lock_flags);
	if (ccic->mipi_config_flag != MIPI_NOT_SET) {
		spin_unlock_irqrestore(&ccic->mipi_flag_lock, mipi_lock_flags);
		return 0;
	}

	valid_sensor = false;
	switch (ccic->ccic_id) {
	case CCIC_ID_1:
		switch (ccic->sensor_type) {
		case SENSOR_OV5642:
			mvisp_reg_writel(isp, 0x1B0B,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY3);
			mvisp_reg_writel(isp, 0x33,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY5);
			mvisp_reg_writel(isp, 0x1A03,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY6);
			valid_sensor = true;
			break;
		case SENSOR_OV8820:
			mvisp_reg_writel(isp, 0x0A06,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY3);
			mvisp_reg_writel(isp, 0x33,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY5);
			mvisp_reg_writel(isp, 0x1A03,
				CCIC_ISP_IOMEM_1, CCIC_CSI2_DPHY6);
			valid_sensor = true;
			break;
		default:
			break;
		}
		if (valid_sensor  == true)
			mvisp_reg_writel(isp,
				0x43, CCIC_ISP_IOMEM_1, CCIC_CSI2_CTRL0);
		break;
	case CCIC_ID_2:
		break;
	default:
		break;
	}

	ccic->mipi_config_flag = MIPI_SET;
	spin_unlock_irqrestore(&ccic->mipi_flag_lock, mipi_lock_flags);
	return 0;
}

/* -----------------------------------------------------------------------------
 * ISP video operations */
static struct isp_ccic_device *find_ccic_from_video(struct isp_video *video)
{
	struct isp_ccic_device *ccic;
	struct mvisp_device *isp = video->isp;

	if (&(isp->mvisp_ccic1.video_out) == video)
		ccic = &isp->mvisp_ccic1;
	else if (&(isp->mvisp_ccic2.video_out) == video)
		ccic = &isp->mvisp_ccic2;
	else
		return NULL;

	return ccic;
}

static int ccic_video_stream_on_notify(struct isp_video *video)
{
	struct isp_ccic_device *ccic;
	struct mvisp_device *isp = video->isp;
	unsigned long regval;

	mutex_lock(&ccic->ccic_mutex);

	ccic = find_ccic_from_video(video);
	if (ccic == NULL) {
		mutex_unlock(&ccic->ccic_mutex);
		return -EINVAL;
	}

	if (ccic->state != ISP_PIPELINE_STREAM_STOPPED) {
		mutex_unlock(&ccic->ccic_mutex);
		return 0;
	}

	ccic_configure_mipi(ccic);

	if ((ccic->output&CCIC_OUTPUT_MEMORY) != 0) {
		mvisp_reg_writel(isp, 0x7,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_STATUS);
		mvisp_reg_writel(isp, 0x7,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_MASK);
		regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		regval |= 0x1;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		ccic_dump_regs(isp);
	}
	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

static int ccic_video_stream_off_notify(struct isp_video *video)
{
	struct isp_ccic_device *ccic;
	struct mvisp_device *isp = video->isp;
	unsigned long regval;

	mutex_lock(&ccic->ccic_mutex);
	ccic = find_ccic_from_video(video);
	if (ccic == NULL) {
		mutex_unlock(&ccic->ccic_mutex);
		return -EINVAL;
	}

	if (ccic->state == ISP_PIPELINE_STREAM_STOPPED) {
		mutex_unlock(&ccic->ccic_mutex);
		return 0;
	}

	if ((ccic->output&CCIC_OUTPUT_MEMORY) != 0) {
		regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		regval &= ~0x1;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_MASK);
		mvisp_reg_writel(isp, 0x7,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_STATUS);

		ccic_dump_regs(isp);
	}
	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

static int ccic_video_qbuf_notify(struct isp_video *video)
{
	return 0;
}

static const struct isp_video_operations ccic_ispvideo_ops = {
	.qbuf_notify = ccic_video_qbuf_notify,
	.stream_on_notify = ccic_video_stream_on_notify,
	.stream_off_notify = ccic_video_stream_off_notify,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

static struct v4l2_mbus_framefmt *
__ccic_get_format(struct isp_ccic_device *ccic, struct v4l2_subdev_fh *fh,
		  unsigned int pad, enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &ccic->formats[pad];
}

static void
ccic_try_format(struct isp_ccic_device *ccic, struct v4l2_subdev_fh *fh,
		unsigned int pad, struct v4l2_mbus_framefmt *fmt,
		enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case CCIC_PAD_SINK:
		for (i = 0; i < ARRAY_SIZE(ccic_input_fmts); i++) {
			if (fmt->code == ccic_input_fmts[i])
				break;
		}

		if (i >= ARRAY_SIZE(ccic_input_fmts))
			fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;

		break;

	case CCIC_PAD_SOURCE:
		for (i = 0; i < ARRAY_SIZE(ccic_output_fmts); i++) {
			if (fmt->code == ccic_output_fmts[i])
				break;
		}

		if (i >= ARRAY_SIZE(ccic_output_fmts))
			fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;
		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_JPEG;
	fmt->field = V4L2_FIELD_NONE;
}

static int ccic_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&ccic->ccic_mutex);
	if (code->pad == CCIC_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(ccic_input_fmts)) {
			ret = -EINVAL;
			goto error;
		}
		code->code = ccic_input_fmts[code->index];
	} else {
		if (code->index >= ARRAY_SIZE(ccic_output_fmts)) {
			ret = -EINVAL;
			goto error;
		}
		code->code = ccic_output_fmts[code->index];
	}

error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	if (fse->index != 0) {
		ret = -EINVAL;
		goto error;
	}

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ccic_try_format(ccic, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code) {
		ret = -EINVAL;
		goto error;
	}

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ccic_try_format(ccic, fh, fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->max_width = format.width;
	fse->max_height = format.height;

error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_get_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	format = __ccic_get_format(ccic, fh, fmt->pad, fmt->which);
	if (format == NULL) {
		ret = -EINVAL;
		goto error;
	}

	fmt->format = *format;
error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_config_format(struct isp_ccic_device *ccic, unsigned int pad)
{
	struct v4l2_mbus_framefmt *format = &ccic->formats[pad];
	struct mvisp_device *isp = ccic->isp;
	unsigned long width, height, regval, bytesperline;
	unsigned long ctrl0val = 0;
	unsigned long ypitch;
	int ret = 0;

	width = format->width;
	height = format->height;

	switch (format->code) {
	case V4L2_MBUS_FMT_YUYV8_2X8:
		ctrl0val = (0 << 16) | (0x4 << 13);
		bytesperline = width * 2;
		ypitch = bytesperline / 4;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		ctrl0val = (3 << 16) | (0x4 << 13);
		bytesperline = width * 2;
		ypitch = bytesperline / 4;
		break;
	default:
		return -EINVAL;
	}

	switch (ccic->ccic_id) {
	case CCIC_ID_1:
		regval = (height << 16) | bytesperline;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_IMG_SIZE);
		regval = (ypitch << 2);
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_IMG_PITCH);
		mvisp_reg_writel(isp, 0,
			CCIC_ISP_IOMEM_1, CCIC_IMG_OFFSET);
		mvisp_reg_writel(isp, ctrl0val,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		break;
	case CCIC_ID_2:
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int ccic_set_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;
	mutex_lock(&ccic->ccic_mutex);

	format = __ccic_get_format(ccic, fh, fmt->pad, fmt->which);
	if (format == NULL) {
		ret = -EINVAL;
		goto error;
	}

	ccic_try_format(ccic, fh, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY)
		ret = ccic_config_format(ccic, fmt->pad);

error:
	mutex_unlock(&ccic->ccic_mutex);
	return ret;
}

static int ccic_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	struct v4l2_mbus_framefmt *format_active, *format_try;
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (fh == NULL) {
		memset(&format, 0, sizeof(format));
		format.pad = CCIC_PAD_SINK;
		format.which =
			fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
		format.format.code = V4L2_MBUS_FMT_YUYV8_2X8;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_JPEG;
		format.format.field = V4L2_FIELD_NONE;

		format.pad = CCIC_PAD_SOURCE;
		format.which =
			fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
		format.format.code = V4L2_MBUS_FMT_UYVY8_1X16;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_JPEG;

		ret = ccic_set_format(sd, fh, &format);
	} else {
	/* Copy the active format to a newly opened fh structure */
		mutex_lock(&ccic->ccic_mutex);
		format_active = __ccic_get_format
			(ccic, fh, CCIC_PAD_SINK, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ccic_get_format
			(ccic, fh, CCIC_PAD_SINK, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

		format_active = __ccic_get_format
			(ccic, fh, CCIC_PAD_SOURCE, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ccic_get_format
			(ccic, fh, CCIC_PAD_SOURCE, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

error:
		mutex_unlock(&ccic->ccic_mutex);
	}

	return ret;
}

static int ccic_open(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	return ccic_init_formats(sd, fh);
}

static int ccic_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);
	struct mvisp_device *isp = ccic->isp;
	u32 regval;

	mutex_lock(&ccic->ccic_mutex);

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		ccic_configure_mipi(ccic);
		regval = (0x3 << 29) | (400 / 26);
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CLOCK_CTRL);
		regval = (1 << 25) | 0x800003C;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_1);
		break;
	case ISP_PIPELINE_STREAM_STOPPED:
		regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		regval &= ~0x1;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
		mvisp_reg_writel(isp, 0x0,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_MASK);
		mvisp_reg_writel(isp, 0x7,
			CCIC_ISP_IOMEM_1, CCIC_IRQ_STATUS);
		ccic_clear_mipi(ccic);
		regval = 0;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CLOCK_CTRL);
		regval = 0x800003C;
		mvisp_reg_writel(isp, regval,
			CCIC_ISP_IOMEM_1, CCIC_CTRL_1);
		break;
	default:
		break;
	}

	ccic->state = enable;
	mutex_unlock(&ccic->ccic_mutex);

	return 0;
}

/* subdev video operations */
static const struct v4l2_subdev_video_ops ccic_video_ops = {
	.s_stream = ccic_set_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops ccic_pad_ops = {
	.enum_mbus_code = ccic_enum_mbus_code,
	.enum_frame_size = ccic_enum_frame_size,
	.get_fmt = ccic_get_format,
	.set_fmt = ccic_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops ccic_ops = {
	.video = &ccic_video_ops,
	.pad = &ccic_pad_ops,
};

/* subdev internal operations */
static const struct v4l2_subdev_internal_ops ccic_internal_ops = {
	.open = ccic_open,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/*
 * ccic_link_setup - Setup CCIC connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
 */
static int ccic_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_ccic_device *ccic = v4l2_get_subdevdata(sd);

	switch (local->index | media_entity_type(remote->entity)) {
	case CCIC_PAD_SINK | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->input |= CCIC_INPUT_SENSOR;
		else
			ccic->output &= ~CCIC_INPUT_SENSOR;
		break;
	case CCIC_PAD_SOURCE | MEDIA_ENT_T_DEVNODE:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->output |= CCIC_OUTPUT_MEMORY;
		else
			ccic->output &= ~CCIC_OUTPUT_MEMORY;
		break;
	case CCIC_PAD_SOURCE | MEDIA_ENT_T_V4L2_SUBDEV:
		if (flags & MEDIA_LNK_FL_ENABLED)
			ccic->output |= CCIC_OUTPUT_ISP;
		else
			ccic->output &= ~CCIC_OUTPUT_ISP;
		break;
	default:
		/* Link from camera to CCIC is fixed... */
		return -EINVAL;
	}
	return 0;
}

/* media operations */
static const struct media_entity_operations ccic_media_ops = {
	.link_setup = ccic_link_setup,
};

/*
 * ccic_init_entities - Initialize subdev and media entity.
 * @ccic: Pointer to ccic structure.
 * return -ENOMEM or zero on success
 */
static int ccic_init_entities(struct isp_ccic_device *ccic,
		enum isp_ccic_identity ccic_id)
{
	struct v4l2_subdev *sd = &ccic->subdev;
	struct media_pad *pads = ccic->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	ccic->sensor_type = SENSOR_INVALID;
	ccic->state = ISP_PIPELINE_STREAM_STOPPED;
	spin_lock_init(&ccic->mipi_flag_lock);
	ccic->ccic_id = ccic_id;
	mutex_init(&ccic->ccic_mutex);

	v4l2_subdev_init(sd, &ccic_ops);
	sd->internal_ops = &ccic_internal_ops;
	strlcpy(sd->name, "pxaccic", sizeof(sd->name));

	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, ccic);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	pads[CCIC_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	pads[CCIC_PAD_SINK].flags = MEDIA_PAD_FL_SINK;

	me->ops = &ccic_media_ops;
	ret = media_entity_init(me, CCIC_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	ccic_init_formats(sd, NULL);

	/* Video device node */
	ccic->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ccic->video_out.ops = &ccic_ispvideo_ops;
	ccic->video_out.isp = ccic->isp;

	ret = mvisp_video_init(&ccic->video_out, "dma_ccic", 1);
	if (ret < 0)
		return ret;

	/* Connect the CCIC subdev to the video node. */
	ret = media_entity_create_link
				(&ccic->subdev.entity, CCIC_PAD_SOURCE,
				&ccic->video_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void pxa_ccic_unregister_entities(struct isp_ccic_device *ccic)
{
	media_entity_cleanup(&ccic->subdev.entity);

	v4l2_device_unregister_subdev(&ccic->subdev);
	mvisp_video_unregister(&ccic->video_out);
}

int pxa_ccic_register_entities(struct isp_ccic_device *ccic,
				    struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &ccic->subdev);
	if (ret < 0)
		goto error;

	ret = mvisp_video_register(&ccic->video_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	pxa_ccic_unregister_entities(ccic);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP CCIC initialisation and cleanup
 */

/*
 * pxa_ccic_cleanup - Routine for module driver cleanup
 */
void pxa_ccic_cleanup(struct mvisp_device *isp)
{
}

/*
 * pxa_ccic_init - Routine for module driver init
 */
int pxa_ccic_init(struct mvisp_device *isp)
{
	struct isp_ccic_device *ccic1 = &isp->mvisp_ccic1;
	struct isp_ccic_device *ccic2 = &isp->mvisp_ccic2;
	int ret;

	ccic1->isp = isp;
	ret = ccic_init_entities(ccic1, CCIC_ID_1);
	if (ret < 0)
		goto fail;

	ccic2->isp = isp;
	ret = ccic_init_entities(ccic2, CCIC_ID_2);
	if (ret < 0)
		goto fail;

	return 0;
fail:
	pxa_ccic_cleanup(isp);
	return ret;
}

void pxa_ccic_set_sensor_type(struct isp_ccic_device *ccic,
					enum mv_isp_sensor_type sensor_type)
{
	ccic->sensor_type = sensor_type;
	return;
}


static void ccic_isr_buffer
		(struct isp_ccic_device *ccic, enum isp_ccic_irq_type irqeof)
{
	struct isp_video_buffer *buffer;

	buffer = mvisp_video_buffer_next(&ccic->video_out, 0);
	if (buffer != NULL)
		ccic_set_dma_addr(ccic, buffer, irqeof);
	else {
		buffer = ccic->video_out.queue->dummy_buffers[0];
		if (buffer != NULL)
			ccic_set_dma_addr(ccic, buffer, irqeof);
	}
}

void pxa_ccic_dma_isr_handler(struct isp_ccic_device *ccic,
				unsigned long irq_status)
{
	unsigned long flags;
	struct mvisp_device *isp = ccic->isp;
	struct isp_video *video;
	u32 regval;
	struct isp_video_buffer *buffer;
	struct isp_video_queue *queue;

	if (irq_status & CCIC_IRQ_STATUS_EOF0) {
		regval = mvisp_reg_readl(isp,
				CCIC_ISP_IOMEM_1, CCIC_Y0_BASE_ADDR);
		if (regval != ccic->video_out.queue->dummy_buffers[0]->paddr)
			ccic_isr_buffer(ccic, CCIC_EOF0);
		else {
			video = &ccic->video_out;
			queue = video->queue;
			spin_lock_irqsave(&queue->irqlock, flags);
			if (list_empty(&video->dmaidlequeue)) {
				regval = mvisp_reg_readl(isp,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				regval &= ~0x1;
				mvisp_reg_writel(isp, regval,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				spin_unlock_irqrestore(&queue->irqlock, flags);
				return;
			}
			buffer = list_first_entry(&video->dmaidlequeue,
				struct isp_video_buffer, irqlist);
			list_del(&buffer->irqlist);
			list_add_tail(&buffer->irqlist, &video->dmabusyqueue);
			buffer->state = ISP_BUF_STATE_ACTIVE;
			ccic_set_dma_addr(ccic, buffer, CCIC_EOF0);
			spin_unlock_irqrestore(&queue->irqlock, flags);
		}
	}

	if (irq_status & CCIC_IRQ_STATUS_EOF1) {
		regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_Y1_BASE_ADDR);
		if (regval != ccic->video_out.queue->dummy_buffers[0]->paddr)
			ccic_isr_buffer(ccic, CCIC_EOF1);
		else {
			video = &ccic->video_out;
			queue = video->queue;
			spin_lock_irqsave(&queue->irqlock, flags);
			if (list_empty(&video->dmaidlequeue)) {
				regval = mvisp_reg_readl(isp,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				regval &= ~0x1;
				mvisp_reg_writel(isp, regval,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				spin_unlock_irqrestore(&queue->irqlock, flags);
				return;
			}
			buffer = list_first_entry(&video->dmaidlequeue,
				struct isp_video_buffer, irqlist);
			list_del(&buffer->irqlist);
			list_add_tail(&buffer->irqlist, &video->dmabusyqueue);
			buffer->state = ISP_BUF_STATE_ACTIVE;
			ccic_set_dma_addr(ccic, buffer, CCIC_EOF1);
			spin_unlock_irqrestore(&queue->irqlock, flags);
		}
	}

	if (irq_status & CCIC_IRQ_STATUS_EOF2) {
		regval = mvisp_reg_readl(isp,
			CCIC_ISP_IOMEM_1, CCIC_Y2_BASE_ADDR);
		if (regval != ccic->video_out.queue->dummy_buffers[0]->paddr)
			ccic_isr_buffer(ccic, CCIC_EOF2);
		else {
			video = &ccic->video_out;
			queue = video->queue;
			spin_lock_irqsave(&queue->irqlock, flags);
			if (list_empty(&video->dmaidlequeue)) {
				regval = mvisp_reg_readl(isp,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				regval &= ~0x1;
				mvisp_reg_writel(isp, regval,
					CCIC_ISP_IOMEM_1, CCIC_CTRL_0);
				spin_unlock_irqrestore(&queue->irqlock, flags);
				return;
			}
			buffer = list_first_entry(&video->dmaidlequeue,
				struct isp_video_buffer, irqlist);
			list_del(&buffer->irqlist);
			list_add_tail(&buffer->irqlist, &video->dmabusyqueue);
			buffer->state = ISP_BUF_STATE_ACTIVE;
			ccic_set_dma_addr(ccic, buffer, CCIC_EOF2);
			spin_unlock_irqrestore(&queue->irqlock, flags);
		}
	}

	return;
}
