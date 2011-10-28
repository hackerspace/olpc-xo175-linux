/*
* ispvideo.c
*
* Marvell DxO ISP - video node
*	Based on omap3isp
*
*
* Copyright:  (C) Copyright 2011 Marvell International Ltd.
*			   Henry Zhao <xzhao10@marvell.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*/


#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>

#include "isp.h"
#include "ispvideo.h"


static struct isp_format_convert_info formats[] = {
	{ V4L2_MBUS_FMT_YUYV8_2X8, V4L2_PIX_FMT_YUYV, 16},
	{ V4L2_MBUS_FMT_UYVY8_1X16, V4L2_PIX_FMT_UYVY, 16},
	{ V4L2_MBUS_FMT_Y12_1X12, V4L2_PIX_FMT_YVU420, 12},
	{ V4L2_MBUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, 8},
};

void set_vd_dmaqueue_flg(struct isp_video *video,
	enum isp_video_dmaqueue_flags flag_val)
{
	unsigned long flags;
	spin_lock_irqsave(&video->dmaflag_lock, flags);
	video->dmaqueue_flags = flag_val;
	spin_unlock_irqrestore(&video->dmaflag_lock, flags);
}

enum isp_video_dmaqueue_flags
	get_vd_dmaqueue_flg(struct isp_video *video)
{
	unsigned long flags;
	enum isp_video_dmaqueue_flags flag_val;

	spin_lock_irqsave(&video->dmaflag_lock, flags);
	flag_val = video->dmaqueue_flags;
	spin_unlock_irqrestore(&video->dmaflag_lock, flags);
	return flag_val;
}


const struct isp_format_convert_info *
mvisp_video_format_info(enum v4l2_mbus_pixelcode code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].code == code)
			return &formats[i];
	}

	return NULL;
}

static int isp_video_mbus_to_pix(const struct isp_video *video,
					  const struct v4l2_mbus_framefmt *mbus,
					  struct v4l2_pix_format *pix)
{
	unsigned int i;

	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].code == mbus->code) {
			pix->pixelformat = formats[i].pixelformat;
			pix->bytesperline = (formats[i].bpp * mbus->width) >> 3;
			break;
		}
	}

	if (WARN_ON(i == ARRAY_SIZE(formats)))
		return -EINVAL;

	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;

	return 0;
}

static int isp_video_pix_to_mbus(const struct v4l2_pix_format *pix,
				  struct v4l2_mbus_framefmt *mbus)
{
	unsigned int i;

	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].pixelformat == pix->pixelformat) {
			mbus->code = formats[i].code;
			break;
		}
	}

	if (WARN_ON(i == ARRAY_SIZE(formats)))
		return -EINVAL;

	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;

	return 0;
}

static struct v4l2_subdev *
isp_video_remote_subdev(struct isp_video *video, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_source(&video->pad);

	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int isp_video_far_end(struct isp_video **video_array
			, struct isp_video *video)
{
	struct media_entity_graph graph;
	struct media_entity *entity = &video->video.entity;
	struct media_device *mdev = entity->parent;
	struct isp_video *far_end_walk = NULL;
	struct isp_video *far_end[FAR_END_MAX_NUM];
	int cnt;

	mutex_lock(&mdev->graph_mutex);
	media_entity_graph_walk_start(&graph, entity);
	for (cnt = 0; cnt < FAR_END_MAX_NUM; cnt++)
		far_end[cnt] = NULL;
	cnt = 0;
	while ((entity = media_entity_graph_walk_next(&graph))) {
		if (entity == &video->video.entity)
			continue;

		if (media_entity_type(entity) != MEDIA_ENT_T_DEVNODE)
			continue;

		far_end_walk = to_isp_video
				(media_entity_to_video_device(entity));
		if ((far_end_walk->type != video->type)
			&& (cnt < FAR_END_MAX_NUM)) {
			switch (far_end_walk->video_type) {
			case ISP_VIDEO_DISPLAY:
				far_end[FAR_END_ISP_DISPLAY] = far_end_walk;
				cnt++;
				break;
			case ISP_VIDEO_CODEC:
				far_end[FAR_END_ISP_CODEC] = far_end_walk;
				cnt++;
				break;
			case ISP_VIDEO_INPUT:
				far_end[FAR_END_ISP_INPUT] = far_end_walk;
				cnt++;
				break;
			case ISP_VIDEO_CCIC:
				far_end[FAR_END_CCIC] = far_end_walk;
				cnt++;
				break;
			default:
				break;
			}
		}
		far_end_walk = NULL;
	}

	memcpy(video_array, far_end,
			sizeof(struct isp_video *) * FAR_END_MAX_NUM);
	mutex_unlock(&mdev->graph_mutex);
	return cnt;
}

static int isp_video_validate_pipeline(struct isp_pipeline *pipe, int index)
{
	struct v4l2_subdev_format fmt_source;
	struct v4l2_subdev_format fmt_sink;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret, max_loop;

	if (pipe->output[index] == NULL)
		return 0;

	subdev = isp_video_remote_subdev(pipe->output[index], NULL);
	if (subdev == NULL)
		return -EPIPE;

	max_loop = 100;
	while (max_loop > 0) {
		/* Retrieve the sink format */
		pad = &subdev->entity.pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		fmt_sink.pad = pad->index;
		fmt_sink.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt_sink);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return -EPIPE;

		/* Retrieve the source format */
		pad = media_entity_remote_source(pad);
		if (pad == NULL ||
		    media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		subdev = media_entity_to_v4l2_subdev(pad->entity);

		fmt_source.pad = pad->index;
		fmt_source.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt_source);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return -EPIPE;

		/* Check if the two ends match */
		if (fmt_source.format.code != fmt_sink.format.code ||
		    fmt_source.format.width != fmt_sink.format.width ||
		    fmt_source.format.height != fmt_sink.format.height)
			return -EPIPE;

		max_loop--;
	}

	if (max_loop == 0)
		return -EPIPE;

	return 0;
}

static int
isp_video_get_subdev_format(struct isp_video *video, struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret == -ENOIOCTLCMD)
		ret = -EINVAL;

	mutex_unlock(&video->mutex);

	if (ret)
		return ret;

	format->type = video->type;
	return isp_video_mbus_to_pix(video, &fmt.format, &format->fmt.pix);
}

static int
isp_video_check_format(struct isp_video *video, struct isp_video_fh *vfh)
{
	struct v4l2_format format;
	int ret;

	memcpy(&format, &vfh->format, sizeof(format));
	ret = isp_video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	if (vfh->format.fmt.pix.pixelformat != format.fmt.pix.pixelformat ||
	    vfh->format.fmt.pix.height != format.fmt.pix.height ||
	    vfh->format.fmt.pix.width != format.fmt.pix.width ||
	    vfh->format.fmt.pix.bytesperline != format.fmt.pix.bytesperline ||
	    vfh->format.fmt.pix.sizeimage != format.fmt.pix.sizeimage)
		return -EINVAL;

	return 0;
}


static void isp_video_queue_prepare(struct isp_video_queue *queue,
				    unsigned int *nbuffers, unsigned int *size)
{
	struct isp_video_fh *vfh =
		container_of(queue, struct isp_video_fh, queue);

	*size = vfh->format.fmt.pix.sizeimage;
	if (*size == 0)
		return;

	*nbuffers = min_t(unsigned int, *nbuffers, ISP_VIDEO_MAX_BUFFERS);
}

static void isp_video_buffer_cleanup(struct isp_video_buffer *buf)
{
}

static int isp_video_buffer_prepare(struct isp_video_buffer *buf)
{
	struct isp_video_fh *vfh = isp_video_queue_to_isp_video_fh(buf->queue);

	buf->vbuf.bytesused = vfh->format.fmt.pix.sizeimage;
	return 0;
}

static void isp_video_buffer_queue(struct isp_video_buffer *buf)
{
	struct isp_video_fh *vfh = isp_video_queue_to_isp_video_fh(buf->queue);
	struct isp_video *video = vfh->video;
	struct isp_pipeline *pipe = to_isp_pipeline(&video->video.entity);
	enum isp_pipeline_state state;
	unsigned long flags;
	unsigned int empty;

	empty = list_empty(&video->dmabusyqueue);
	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		switch (video->video_type) {
		case ISP_VIDEO_DISPLAY:
			state = ISP_PIPELINE_DISPLAY_QUEUED;
			break;
		case ISP_VIDEO_CODEC:
			state = ISP_PIPELINE_CODEC_QUEUED;
			break;
		default:
			state = 0;
			break;
		}
	} else
		state = ISP_PIPELINE_INPUT_QUEUED;

	spin_lock_irqsave(&pipe->lock, flags);
	pipe->state |= state;
	spin_unlock_irqrestore(&pipe->lock, flags);

	list_add_tail(&buf->irqlist, &video->dmaidlequeue);
	if (empty) {
		/* Notify the subdev of qbuf event */
		video->ops->qbuf_notify(video);
	}

	return;
}

struct isp_video_buffer *mvisp_video_get_next_work_buf(
	struct isp_video *video, int delay)
{
	struct isp_video_buffer *buf;
	unsigned long flags;
	struct isp_pipeline *pipe = to_isp_pipeline(&video->video.entity);
	enum isp_pipeline_state state;

	if (list_empty(&video->dmaidlequeue))
		return NULL;

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		switch (video->video_type) {
		case ISP_VIDEO_DISPLAY:
			state = ISP_PIPELINE_DISPLAY_QUEUED;
			break;
		case ISP_VIDEO_CODEC:
			state = ISP_PIPELINE_CODEC_QUEUED;
			break;
		default:
			state = 0;
			break;
		}
	} else
		state = ISP_PIPELINE_INPUT_QUEUED;

	buf = list_first_entry(&video->dmaidlequeue, struct isp_video_buffer,
				   irqlist);
	list_del(&buf->irqlist);

	switch (video->video_type) {
	case ISP_VIDEO_INPUT:
	case ISP_VIDEO_DISPLAY:
	case ISP_VIDEO_CODEC:
		buf->delay = delay;
		break;
	default:
		buf->delay = 0;
		break;
	}

	list_add_tail(&buf->irqlist, &video->dmabusyqueue);

	if (list_empty(&video->dmaidlequeue)) {
		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state &= ~state;
		spin_unlock_irqrestore(&pipe->lock, flags);
	}

	return buf;
}

enum isp_pipeline_start_condition isp_pipeline_ready(struct isp_pipeline *pipe)
{
	enum isp_pipeline_start_condition condition = ISP_CAN_NOT_START;
	unsigned long flags;

	spin_lock_irqsave(&pipe->lock, flags);
	if ((pipe->state & ISP_PIPELINE_INPUT_STREAM) &&
	(pipe->state & ISP_PIPELINE_INPUT_QUEUED) != 0) {
		condition |= ISP_INPUT_CAN_START;
	}

	if ((pipe->state & ISP_PIPELINE_DISPLAY_STREAM) &&
	(pipe->state & ISP_PIPELINE_DISPLAY_QUEUED) != 0) {
		condition |= ISP_DISPLAY_CAN_START;
	}

	if ((pipe->state & ISP_PIPELINE_CODEC_STREAM) &&
	(pipe->state & ISP_PIPELINE_CODEC_QUEUED) != 0) {
		condition |= ISP_CODEC_CAN_START;
	}
	spin_unlock_irqrestore(&pipe->lock, flags);

	return condition;
}


static const struct isp_video_queue_operations isp_video_queue_ops = {
	.queue_prepare = &isp_video_queue_prepare,
	.buffer_prepare = &isp_video_buffer_prepare,
	.buffer_queue = &isp_video_buffer_queue,
	.buffer_cleanup = &isp_video_buffer_cleanup,
};


struct isp_video_buffer *mvisp_video_load_userbuf(
	struct isp_video *video,
	struct isp_video_queue *queue)
{
	struct isp_video_buffer *buf, *last_buf;
	int num_ready_buf;
	bool buf_in_busy;

	num_ready_buf = 0;
	last_buf = NULL;
	list_for_each_entry(buf, &queue->queue, stream) {
		last_buf = buf;
		num_ready_buf++;
	}

	if (num_ready_buf > 2) {
		buf = list_first_entry(&queue->queue,
			struct isp_video_buffer, stream);
		list_del(&buf->stream);
		buf->state = ISP_BUF_STATE_ACTIVE;
		list_add_tail(&buf->stream, &queue->queue);
	} else {
		buf = last_buf;
		buf->state = ISP_BUF_STATE_ACTIVE;
		if (buf == NULL)
			return buf;
	}

	buf_in_busy = false;
	list_for_each_entry(last_buf, &video->dmabusyqueue, irqlist) {
		if (buf == last_buf)
			buf_in_busy = true;
	}

	if (buf_in_busy == false) {
		switch (video->video_type) {
		case ISP_VIDEO_INPUT:
		case ISP_VIDEO_DISPLAY:
		case ISP_VIDEO_CODEC:
			buf->delay = 1;
			break;
		default:
			buf->delay = 0;
			break;
		}
		list_add_tail(&buf->irqlist, &video->dmabusyqueue);
		set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_QUEUED);
	}

	return buf;
}

struct isp_video_buffer *mvisp_video_buffer_next(struct isp_video *video,
					      unsigned int error)
{
	struct isp_pipeline *pipe = to_isp_pipeline(&video->video.entity);
	struct isp_video_queue *queue = video->queue;
	enum isp_pipeline_state state;
	struct isp_video_buffer *buf;
	unsigned long flags, mode_flags;
	struct timespec ts;
	bool no_buf_found = false;
	enum ispvideo_capture_mode	capture_mode;

	if (WARN_ON(list_empty(&video->dmabusyqueue)))
		return NULL;

	buf = list_first_entry(&video->dmabusyqueue, struct isp_video_buffer,
			       irqlist);

	if (buf->delay == 0) {
		list_del(&buf->irqlist);

		ktime_get_ts(&ts);
		buf->vbuf.timestamp.tv_sec = ts.tv_sec;
		buf->vbuf.timestamp.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

		if (video == pipe->output[0] || video == pipe->output[1])
			buf->vbuf.sequence =
				atomic_inc_return(&pipe->frame_number);
		else
			buf->vbuf.sequence = atomic_read(&pipe->frame_number);

		spin_lock_irqsave(&queue->irq_queue_lock, flags);
		buf->state = error ? ISP_BUF_STATE_ERROR : ISP_BUF_STATE_DONE;
		spin_unlock_irqrestore(&queue->irq_queue_lock, flags);

		wake_up(&buf->wait);
	}

	list_for_each_entry(buf, &video->dmabusyqueue, irqlist) {
		if (buf->delay > 0)
			buf->delay--;
	}

	if (list_empty(&video->dmaidlequeue)) {
		spin_lock_irqsave(&video->cap_mode_lock, mode_flags);
		capture_mode = video->capture_mode;
		spin_unlock_irqrestore(&video->cap_mode_lock, mode_flags);

		if (capture_mode == ISPVIDEO_STILL_CAPTURE) {
			no_buf_found = true;
		} else {
			spin_lock_irqsave(&queue->irq_queue_lock, flags);
			if (list_empty(&queue->queue)) {
				no_buf_found = true;
			} else {
				buf =
					mvisp_video_load_userbuf(video, queue);
				spin_unlock_irqrestore(
					&queue->irq_queue_lock, flags);
				return buf;
			}
			spin_unlock_irqrestore(&queue->irq_queue_lock, flags);
		}
	} else {
		buf = list_first_entry(&video->dmaidlequeue,
				struct isp_video_buffer,
				irqlist);
		list_del(&buf->irqlist);
		spin_lock_irqsave(&queue->irq_queue_lock, flags);
		buf->state = ISP_BUF_STATE_ACTIVE;
		spin_unlock_irqrestore(&queue->irq_queue_lock, flags);
	}

	if (no_buf_found) {
		if (queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			switch (video->video_type) {
			case ISP_VIDEO_DISPLAY:
				state = ISP_PIPELINE_DISPLAY_QUEUED;
				break;
			case ISP_VIDEO_CODEC:
				state = ISP_PIPELINE_CODEC_QUEUED;
				break;
			default:
				state = 0;
				break;
			}
		} else
			state = ISP_PIPELINE_INPUT_QUEUED;

		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state &= ~state;
		spin_unlock_irqrestore(&pipe->lock, flags);
		set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_UNDERRUN);
		return NULL;
	}

	switch (video->video_type) {
	case ISP_VIDEO_INPUT:
	case ISP_VIDEO_DISPLAY:
	case ISP_VIDEO_CODEC:
		buf->delay = 1;
		break;
	default:
		buf->delay = 0;
		break;
	}
	list_add_tail(&buf->irqlist, &video->dmabusyqueue);
	set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_QUEUED);

	return buf;
}

void mvisp_video_resume(struct isp_video *video, int continuous)
{
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
isp_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct isp_video *video = video_drvdata(file);

	strlcpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));
	cap->version = ISP_VIDEO_DRIVER_VERSION;

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	else
		cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;

	return 0;
}

static int
isp_video_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);

	if (format->type != video->type)
		return -EINVAL;

	mutex_lock(&video->mutex);
	*format = vfh->format;
	mutex_unlock(&video->mutex);

	return 0;
}

static int
isp_video_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev_format subdev_fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (format->type != video->type)
		return -EINVAL;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	subdev_fmt.pad = pad;
	subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	isp_video_pix_to_mbus(&format->fmt.pix, &subdev_fmt.format);
	ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &subdev_fmt);
	if (ret) {
		mutex_unlock(&video->mutex);
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;
	}

	isp_video_mbus_to_pix(video, &subdev_fmt.format, &format->fmt.pix);
	vfh->format = *format;

	mutex_unlock(&video->mutex);
	return 0;
}

static int
isp_video_try_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev_format subdev_fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (format->type != video->type)
		return -EINVAL;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	isp_video_pix_to_mbus(&format->fmt.pix, &subdev_fmt.format);

	subdev_fmt.pad = pad;
	subdev_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &subdev_fmt);
	if (ret)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	mutex_lock(&video->mutex);
	isp_video_mbus_to_pix(video, &subdev_fmt.format, &format->fmt.pix);
	mutex_unlock(&video->mutex);

	return 0;
}

static int
isp_video_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = isp_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, cropcap, cropcap);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
isp_video_get_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev_format format;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	/* Try the get crop operation first and fallback to get format if not
	 * implemented.
	 */
	ret = v4l2_subdev_call(subdev, video, g_crop, crop);
	if (ret != -ENOIOCTLCMD)
		return ret;

	format.pad = pad;
	format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &format);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	mutex_lock(&video->mutex);
	crop->c.left = 0;
	crop->c.top = 0;
	crop->c.width = format.format.width;
	crop->c.height = format.format.height;
	mutex_unlock(&video->mutex);

	return 0;
}

static int
isp_video_set_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = isp_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, s_crop, crop);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
isp_video_get_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);

	if (video->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    video->type != a->type)
		return -EINVAL;

	memset(a, 0, sizeof(*a));
	a->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	a->parm.output.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.output.timeperframe = vfh->timeperframe;

	return 0;
}

static int
isp_video_set_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);

	if (video->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    video->type != a->type)
		return -EINVAL;

	if (a->parm.output.timeperframe.denominator == 0)
		a->parm.output.timeperframe.denominator = 1;

	vfh->timeperframe = a->parm.output.timeperframe;

	return 0;
}

static int
isp_video_reqbufs(struct file *file, void *fh
			, struct v4l2_requestbuffers *rb)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);

	return mvisp_video_queue_reqbufs(&vfh->queue
			, rb, video->isp->isp_dummy_vaddr);
}

static int
isp_video_querybuf(struct file *file, void *fh
			, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return mvisp_video_queue_querybuf(&vfh->queue, b);
}

static int
isp_video_qbuf(struct file *file, void *fh
			, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return mvisp_video_queue_qbuf(&vfh->queue, b);
}

static int
isp_video_dqbuf(struct file *file, void *fh
			, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return mvisp_video_queue_dqbuf(&vfh->queue, b,
					  file->f_flags & O_NONBLOCK, file);
}


static int
isp_video_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	enum isp_pipeline_state state;
	struct isp_pipeline *pipe;
	struct isp_video *far_end[FAR_END_MAX_NUM];
	int far_end_num;
	unsigned long flags;
	int ret, cnt;
	enum isp_pipeline_stream_state start_mode;
	enum isp_pipeline_stream_state stream_state;

	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	if (video->streaming) {
		mutex_unlock(&video->stream_lock);
		return -EBUSY;
	}

	pipe = video->video.entity.pipe
		? to_isp_pipeline(&video->video.entity) : &video->pipe;
	media_entity_pipeline_start(&video->video.entity, &pipe->pipe);


	ret = isp_video_check_format(video, vfh);
	if (ret < 0)
		goto error;

	far_end_num = isp_video_far_end(far_end, video);

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		switch (video->video_type) {
		case ISP_VIDEO_DISPLAY:
			state = ISP_PIPELINE_DISPLAY_STREAM;
			pipe->output[FAR_END_ISP_DISPLAY] = video;
			break;
		case ISP_VIDEO_CODEC:
			state = ISP_PIPELINE_CODEC_STREAM;
			pipe->output[FAR_END_ISP_DISPLAY] = video;
			break;
		default:
			state = 0;
			break;
		}

		if (far_end_num == 0) {
			pipe->input = NULL;
		} else {
			if (pipe->input == NULL)
				pipe->input = far_end[FAR_END_ISP_INPUT];
		}

	} else if (video->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		if (far_end_num == 0) {
			ret = -EPIPE;
			goto error;
		}

		state = ISP_PIPELINE_INPUT_STREAM;
		pipe->input = video;
		for (cnt = 0; cnt < far_end_num; cnt++) {
			if (pipe->output[cnt] != NULL)
				pipe->output[cnt] = far_end[cnt];
		}
	} else {
		ret = -EINVAL;
		goto error;
	}

	spin_lock_irqsave(&pipe->lock, flags);
	pipe->state |= state;
	spin_unlock_irqrestore(&pipe->lock, flags);

	switch (video->video_type) {
	case ISP_VIDEO_DISPLAY:
		ret = isp_video_validate_pipeline(pipe, 0);
		if (ret < 0)
			goto error;
		break;
	case ISP_VIDEO_CODEC:
		ret = isp_video_validate_pipeline(pipe, 1);
		if (ret < 0)
			goto error;
		break;
	case ISP_VIDEO_INPUT:
		for (cnt = 0; cnt < far_end_num; cnt++) {
			ret = isp_video_validate_pipeline(pipe, cnt);
			if (ret < 0)
				goto error;
		}
		break;
	default:
		goto error;
		break;
	}


	/* Set the maximum time per frame as the value requested by userspace.
	 * This is a soft limit that can be overridden if the hardware doesn't
	 * support the request limit.
	 */
	if (video->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		pipe->max_timeperframe = vfh->timeperframe;

	video->queue = &vfh->queue;
	INIT_LIST_HEAD(&video->dmabusyqueue);
	set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_UNDERRUN);

	if (((pipe->state & ISP_PIPELINE_DISPLAY_STREAM) == 0)
		&& ((pipe->state & ISP_PIPELINE_CODEC_STREAM) == 0))
		atomic_set(&pipe->frame_number, -1);

	ret = mvisp_video_queue_streamon(&vfh->queue);
	if (ret < 0)
		goto error;

	start_mode = ISP_PIPELINE_STREAM_CONTINUOUS;

	spin_lock_irqsave(&pipe->stream_lock, flags);
	stream_state = pipe->stream_state;
	spin_unlock_irqrestore(&pipe->stream_lock, flags);

	if (stream_state == ISP_PIPELINE_STREAM_STOPPED) {
		ret = mvisp_pipeline_set_stream(pipe, start_mode);
		if (ret < 0)
			goto error;
	}

	if (video->ops->stream_on_notify != NULL)
		video->ops->stream_on_notify(video);

error:

	if (ret < 0) {
		mvisp_video_queue_streamoff(&vfh->queue);
		media_entity_pipeline_stop(&video->video.entity);
		video->queue = NULL;
	}

	if (!ret)
		video->streaming = 1;

	mutex_unlock(&video->stream_lock);

	return ret;
}

static int
isp_video_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	struct isp_pipeline *pipe = to_isp_pipeline(&video->video.entity);
	enum isp_pipeline_state state;
	unsigned int streaming;
	unsigned long flags;


	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	/* Make sure we're not streaming yet. */
	mutex_lock(&vfh->queue.lock);
	streaming = vfh->queue.streaming;
	mutex_unlock(&vfh->queue.lock);

	if (!streaming)
		goto done;

	/* Update the pipeline state. */
	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		switch (video->video_type) {
		case ISP_VIDEO_DISPLAY:
			state = ISP_PIPELINE_DISPLAY_QUEUED
				| ISP_PIPELINE_DISPLAY_STREAM;
			break;
		case ISP_VIDEO_CODEC:
			state = ISP_PIPELINE_CODEC_QUEUED
				| ISP_PIPELINE_CODEC_STREAM;
			break;
		default:
			state = 0;
			break;
		}
	} else
		state = ISP_PIPELINE_INPUT_STREAM;

	spin_lock_irqsave(&pipe->lock, flags);
	pipe->state &= ~state;
	spin_unlock_irqrestore(&pipe->lock, flags);

	/* Stop the stream. */
	if (((pipe->state & ISP_PIPELINE_DISPLAY_STREAM) == 0)
		&& ((pipe->state & ISP_PIPELINE_CODEC_STREAM) == 0)
		&& ((pipe->state & ISP_PIPELINE_CCIC_STREAM) == 0)) {
		mvisp_pipeline_set_stream(pipe, ISP_PIPELINE_STREAM_STOPPED);
	}

	if (video->ops->stream_off_notify != NULL)
		video->ops->stream_off_notify(video);

	mvisp_video_queue_streamoff(&vfh->queue);
	video->queue = NULL;
	video->streaming = 0;
	INIT_LIST_HEAD(&video->dmaidlequeue);
	INIT_LIST_HEAD(&video->dmabusyqueue);
	set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_UNDERRUN);
	media_entity_pipeline_stop(&video->video.entity);
	if (video->video.entity.pipe == NULL) {
		video->pipe.input = NULL;
		video->pipe.output[0] = NULL;
		video->pipe.output[1] = NULL;
	}


done:
	mutex_unlock(&video->stream_lock);
	return 0;
}

static int
isp_video_enum_input(struct file *file, void *fh, struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int
isp_video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int
isp_video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap		= isp_video_querycap,
	.vidioc_g_fmt_vid_cap		= isp_video_get_format,
	.vidioc_s_fmt_vid_cap		= isp_video_set_format,
	.vidioc_try_fmt_vid_cap		= isp_video_try_format,
	.vidioc_g_fmt_vid_out		= isp_video_get_format,
	.vidioc_s_fmt_vid_out		= isp_video_set_format,
	.vidioc_try_fmt_vid_out		= isp_video_try_format,
	.vidioc_cropcap			= isp_video_cropcap,
	.vidioc_g_crop			= isp_video_get_crop,
	.vidioc_s_crop			= isp_video_set_crop,
	.vidioc_g_parm			= isp_video_get_param,
	.vidioc_s_parm			= isp_video_set_param,
	.vidioc_reqbufs			= isp_video_reqbufs,
	.vidioc_querybuf		= isp_video_querybuf,
	.vidioc_qbuf			= isp_video_qbuf,
	.vidioc_dqbuf			= isp_video_dqbuf,
	.vidioc_streamon		= isp_video_streamon,
	.vidioc_streamoff		= isp_video_streamoff,
	.vidioc_enum_input		= isp_video_enum_input,
	.vidioc_g_input			= isp_video_g_input,
	.vidioc_s_input			= isp_video_s_input,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int isp_video_open(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct isp_video_fh *handle;
	int ret = 0;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);

	if (mvisp_get(video->isp) == NULL) {
		ret = -EBUSY;
		goto err;
	}

	ret = mvisp_pipeline_pm_use(&video->video.entity, 1);
	if (ret < 0) {
		mvisp_put(video->isp);
		goto err;
	}

	mvisp_video_queue_init(&handle->queue, video->type,
				  &isp_video_queue_ops, video->isp->dev,
				  sizeof(struct isp_video_buffer),
				  video->enable_dummy);

	memset(&handle->format, 0, sizeof(handle->format));
	handle->format.type = video->type;
	handle->timeperframe.denominator = 1;

	handle->video = video;
	file->private_data = &handle->vfh;

	return 0;

err:
	v4l2_fh_del(&handle->vfh);
	kfree(handle);

	return ret;
}

static int isp_video_release(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct isp_video_fh *handle = to_isp_video_fh(vfh);

	/* Disable streaming and free the buffers queue resources. */
	isp_video_streamoff(file, vfh, video->type);

	mutex_lock(&handle->queue.lock);
	mvisp_video_queue_cleanup(&handle->queue);
	mutex_unlock(&handle->queue.lock);

	mvisp_pipeline_pm_use(&video->video.entity, 0);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	kfree(handle);
	file->private_data = NULL;

	mvisp_put(video->isp);

	return 0;
}

static unsigned int isp_video_poll(struct file *file, poll_table *wait)
{
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);
	struct isp_video_queue *queue = &vfh->queue;

	queue->poll_on_empty = false;
	return mvisp_video_queue_poll(queue, file, wait);
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);

	return mvisp_video_queue_mmap(&vfh->queue, vma);
}

static struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = isp_video_open,
	.release = isp_video_release,
	.poll = isp_video_poll,
	.mmap = isp_video_mmap,
};

/* -----------------------------------------------------------------------------
 * ISP video core
 */

static const struct isp_video_operations isp_video_dummy_ops = {
};

int mvisp_video_init(struct isp_video *video,
		const char *name, bool enable_dummy)
{
	const char *direction;
	int ret, cnt;

	switch (video->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		direction = "output";
		video->pad.flags = MEDIA_PAD_FL_SINK;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		direction = "input";
		video->pad.flags = MEDIA_PAD_FL_SOURCE;
		break;

	default:
		return -EINVAL;
	}

	ret = media_entity_init(&video->video.entity, 1, &video->pad, 0);
	if (ret < 0)
		return ret;

	mutex_init(&video->mutex);

	spin_lock_init(&video->pipe.lock);
	spin_lock_init(&video->pipe.stream_lock);
	spin_lock_init(&video->dmaflag_lock);
	spin_lock_init(&video->cap_mode_lock);
	mutex_init(&video->stream_lock);

	INIT_LIST_HEAD(&video->dmaidlequeue);
	INIT_LIST_HEAD(&video->dmabusyqueue);

	video->capture_mode = ISPVIDEO_NORMAL_CAPTURE;
	video->enable_dummy = enable_dummy;

	/* Initialize the video device. */
	if (video->ops == NULL)
		video->ops = &isp_video_dummy_ops;

	video->video.fops = &isp_video_fops;
	snprintf(video->video.name, sizeof(video->video.name),
		 "mvisp %s %s", name, direction);

	set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_UNDERRUN);
	video->video.vfl_type = VFL_TYPE_GRABBER;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &isp_video_ioctl_ops;
	video->pipe.stream_state = ISP_PIPELINE_STREAM_STOPPED;
	video->pipe.state = 0;

	video->pipe.input = NULL;
	for (cnt = 0; cnt < FAR_END_MAX_NUM; cnt++)
		video->pipe.output[cnt] = NULL;

	video_set_drvdata(&video->video, video);

	if (strcmp(name, ISP_VIDEO_INPUT_NAME) == 0)
		video->video_type = ISP_VIDEO_INPUT;
	else if (strcmp(name, ISP_VIDEO_DISPLAY_NAME) == 0)
		video->video_type = ISP_VIDEO_DISPLAY;
	else if (strcmp(name, ISP_VIDEO_CODEC_NAME) == 0)
		video->video_type = ISP_VIDEO_CODEC;
	else if (strcmp(name, ISP_VIDEO_CCIC1_NAME) == 0)
		video->video_type = ISP_VIDEO_CCIC;
	else
		video->video_type = ISP_VIDEO_UNKNOWN;

	return 0;
}

int mvisp_video_register(struct isp_video *video, struct v4l2_device *vdev)
{
	int nr;

	video->video.v4l2_dev = vdev;

	switch (video->video_type) {
	case ISP_VIDEO_DISPLAY:
		nr = ISP_VIDEO_NR_BASE;
		break;
	case ISP_VIDEO_CODEC:
		nr = ISP_VIDEO_NR_BASE + 1;
		break;
	case ISP_VIDEO_INPUT:
		nr = ISP_VIDEO_NR_BASE + 2;
		break;
	case ISP_VIDEO_CCIC:
		nr = ISP_VIDEO_NR_BASE + 3;
		break;
	default:
		nr = -1;
		break;
	}

	return video_register_device(&video->video, VFL_TYPE_GRABBER, nr);
}

void mvisp_video_unregister(struct isp_video *video)
{
	if (video_is_registered(&video->video)) {
		media_entity_cleanup(&video->video.entity);
		video_unregister_device(&video->video);
	}
}
