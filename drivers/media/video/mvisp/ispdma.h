/*
 * ispdma.h
 *
 * Marvell DxO ISP - DMA module
 *	Based on omap3isp
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

#ifndef ISP_DMA_H
#define ISP_DMA_H

#include <linux/mvisp.h>
#include <linux/types.h>
#include <media/v4l2-ctrls.h>

#include "ispvideo.h"

enum ispdma_input_entity {
	ISPDMA_INPUT_NONE,
	ISPDMA_INPUT_CCIC_1,
	ISPDMA_INPUT_CCIC_2,
	ISPDMA_INPUT_MEMORY,
};

enum ispdma_output_entity {
	ISPDMA_OUTPUT_NONE,
	ISPDMA_OUTPUT_MEMORY,
};

enum ispdma_port {
	ISPDMA_PORT_CODEC,
	ISPDMA_PORT_DISPLAY,
	ISPDMA_PORT_FBRX,
	ISPDMA_PORT_FBTX,
	ISPDMA_PORT_INPUT,
};


#define ISPDMA_PAD_SINK					0
#define ISPDMA_PAD_CODEC_SOURCE			1
#define ISPDMA_PAD_DISPLAY_SOURCE		2
#define ISPDMA_PADS_NUM					3

#define DMA_NOT_WORKING				0x0
#define DMA_INPUT_WORKING			0x1
#define DMA_DISP_WORKING			0x2
#define DMA_CODEC_WORKING			0x4

struct isp_ispdma_device {
	struct v4l2_subdev			subdev;
	struct media_pad			pads[ISPDMA_PADS_NUM];
	struct v4l2_mbus_framefmt	formats[ISPDMA_PADS_NUM];

	struct v4l2_ctrl_handler	ctrls;

	enum ispdma_input_entity	input;
	enum ispdma_output_entity	disp_out;
	enum ispdma_output_entity	codec_out;
	struct isp_video			vd_in;
	struct isp_video			vd_disp_out;
	struct isp_video			vd_codec_out;

	enum isp_pipeline_stream_state	state;
	spinlock_t						ipc_irq_lock;
	spinlock_t						dma_irq_lock;
	struct mutex					ispdma_mutex;

	struct completion	ipc_event;
	wait_queue_head_t	ipc_irq_wait;

	unsigned int		ipc_event_cnt;
	unsigned int		dma_event_cnt;
	unsigned int		mipi_overrun_cnt;
	unsigned int		disp_eof_cnt;
	unsigned int		input_event_cnt;

	unsigned long		framebuf_count;
	spinlock_t			dmaflg_lock;
	unsigned long		dma_working_flag;

	unsigned int		codec_band_cnt;
	unsigned int		codec_band_max;
};

struct mvisp_device;


int mv_ispdma_init(struct mvisp_device *isp);
void mv_ispdma_cleanup(struct mvisp_device *isp);

int mv_ispdma_register_entities(struct isp_ispdma_device *wrp,
				       struct v4l2_device *vdev);
void mv_ispdma_unregister_entities(struct isp_ispdma_device *wrp);

void mv_ispdma_isr_frame_sync(struct isp_ispdma_device *wrp);

void mv_ispdma_ipc_isr_handler(struct isp_ispdma_device *wrp);

void mv_ispdma_dma_isr_handler(struct isp_ispdma_device *wrp,
	unsigned long irq_status);

void mv_ispdma_restore_context(struct mvisp_device *isp);

#endif	/* ISP_DMA_H */
