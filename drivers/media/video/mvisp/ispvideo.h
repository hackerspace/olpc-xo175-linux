/*
* ispvideo.h
*
* Marvell DxO ISP - video node module
*  Based on omap3isp
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


#ifndef ISP_VIDEO_H
#define ISP_VIDEO_H

#include <linux/v4l2-mediabus.h>
#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>

#include "ispqueue.h"


#define ISP_VIDEO_DRIVER_NAME		"mvsocisp"
#define ISP_VIDEO_DRIVER_VERSION	KERNEL_VERSION(0, 0, 1)
enum isp_video_pipe_far_end {
	FAR_END_ISP_DISPLAY = 0,
	FAR_END_ISP_CODEC,
	FAR_END_ISP_INPUT,
	FAR_END_CCIC,
	FAR_END_MAX_NUM,
};

struct mvisp_device;
struct isp_video;
struct v4l2_mbus_framefmt;
struct v4l2_pix_format;

#define ISP_VIDEO_INPUT_NAME	"dma_input"
#define ISP_VIDEO_CODEC_NAME	"dma_codec"
#define ISP_VIDEO_DISPLAY_NAME	"dma_display"
#define ISP_VIDEO_CCIC1_NAME	"dma_ccic1"



enum isp_video_type {
	ISP_VIDEO_UNKNOWN = 0,
	ISP_VIDEO_INPUT,
	ISP_VIDEO_DISPLAY,
	ISP_VIDEO_CODEC,
	ISP_VIDEO_CCIC
};

struct isp_format_convert_info {
	enum v4l2_mbus_pixelcode code;
	u32 pixelformat;
	unsigned int bpp;
};

enum isp_pipeline_stream_state {
	ISP_PIPELINE_STREAM_STOPPED = 0,
	ISP_PIPELINE_STREAM_CONTINUOUS = 1,
};

enum isp_pipeline_state {
	/* The stream has been started on the input video node. */
	ISP_PIPELINE_INPUT_STREAM = 1,
	/* The stream has been started on the output video node. */
	ISP_PIPELINE_DISPLAY_STREAM = 2,
	/* The stream has been started on the output video node. */
	ISP_PIPELINE_CODEC_STREAM = 4,
	/* At least one buffer is queued on the input video node. */
	ISP_PIPELINE_INPUT_QUEUED = 8,
	/* At least one buffer is queued on the output video node. */
	ISP_PIPELINE_DISPLAY_QUEUED = 16,
	/* At least one buffer is queued on the input video node. */
	ISP_PIPELINE_CODEC_QUEUED = 32,
	/* The stream has been started on the ccic video node. */
	ISP_PIPELINE_CCIC_STREAM = 64,
	/* At least one buffer is queued on the ccic video node. */
	ISP_PIPELINE_CCIC_QUEUED = 128,
};

enum isp_pipeline_start_condition {
	ISP_CAN_NOT_START = 0,
	ISP_INPUT_CAN_START = 1,
	ISP_DISPLAY_CAN_START = 2,
	ISP_CODEC_CAN_START = 4,
};

struct isp_pipeline {
	struct media_pipeline	pipe;
	spinlock_t				lock;
	unsigned int			state;
	struct isp_video		*input;
	struct isp_video		*output[FAR_END_MAX_NUM];
	atomic_t				frame_number;
	struct v4l2_fract		max_timeperframe;
	enum isp_pipeline_stream_state	stream_state;
	spinlock_t				stream_lock;
};

#define to_isp_pipeline(__e) \
	container_of((__e)->pipe, struct isp_pipeline, pipe)

enum isp_video_dmaqueue_flags {
	ISP_VIDEO_DMAQUEUE_UNDERRUN = 0,
	ISP_VIDEO_DMAQUEUE_QUEUED,
	ISP_VIDEO_DMAQUEUE_BUSY,
};


struct isp_video_operations {
	int(*qbuf_notify)(struct isp_video *video);
	int(*stream_on_notify)(struct isp_video *video);
	int(*stream_off_notify)(struct isp_video *video);
};

struct isp_video {
	struct video_device		video;
	enum v4l2_buf_type		type;
	struct media_pad		pad;
	struct mutex			mutex;
	enum isp_video_type		video_type;
	struct mvisp_device	*isp;

	/* Entity video node streaming */
	unsigned int streaming;

	/* Pipeline state */
	struct isp_pipeline		pipe;
	struct mutex			stream_lock;

	/* Video buffers queue */
	bool					enable_dummy;
	struct isp_video_queue	*queue;
	struct list_head		dmaidlequeue;
	struct list_head		dmabusyqueue;
	spinlock_t						dmaflag_lock;
	enum isp_video_dmaqueue_flags	dmaqueue_flags;

	const struct isp_video_operations	*ops;
};

void set_vd_dmaqueue_flg(struct isp_video *video,
	enum isp_video_dmaqueue_flags flag_val);

enum isp_video_dmaqueue_flags get_vd_dmaqueue_flg(struct isp_video *video);


#define to_isp_video(vdev)	container_of(vdev, struct isp_video, video)

struct isp_video_fh {
	struct v4l2_fh			vfh;
	struct isp_video		*video;
	struct isp_video_queue	queue;
	struct v4l2_format		format;
	struct v4l2_fract		timeperframe;
};

#define to_isp_video_fh(fh)	\
	container_of(fh, struct isp_video_fh, vfh)
#define isp_video_queue_to_isp_video_fh(q) \
	container_of(q, struct isp_video_fh, queue)

int mvisp_video_init(struct isp_video *video, const char *name,
			bool enable_dummy);
int mvisp_video_register(struct isp_video *video,
			struct v4l2_device *vdev);
void mvisp_video_unregister(struct isp_video *video);
struct isp_video_buffer *mvisp_video_buffer_next(struct isp_video *video,
					      unsigned int error);
struct isp_video_buffer
	*mvisp_video_get_next_work_buf(struct isp_video *video, int delay);

void mvisp_video_resume(struct isp_video *video, int continuous);
struct media_pad *mvisp_video_remote_pad(struct isp_video *video);

const struct isp_format_convert_info *
mvisp_video_format_info(enum v4l2_mbus_pixelcode code);

enum isp_pipeline_start_condition isp_pipeline_ready(struct isp_pipeline *pipe);

#endif /* ISP_VIDEO_H */
