/*
 * ispqueue.h
 *
 * Marvell DxO ISP - buffer queue module
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

#ifndef ISP_QUEUE_H
#define ISP_QUEUE_H

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/wait.h>

struct isp_video_queue;

#define ISP_VIDEO_MAX_BUFFERS			16
#define ISP_VIDEO_MAX_DUMMY_BUFFERS		1

enum isp_video_buffer_state {
	ISP_BUF_STATE_IDLE = 0,
	ISP_BUF_STATE_QUEUED,
	ISP_BUF_STATE_ACTIVE,
	ISP_BUF_STATE_ERROR,
	ISP_BUF_STATE_DONE,
};

struct isp_video_buffer {
	unsigned long			vma_use_count;
	struct list_head		stream;
	struct isp_video_queue	*queue;
	unsigned int			prepared;

	/* For kernel buffers. */
	void *vaddr;

	/* For userspace buffers. */
	unsigned long		vm_flags;
	unsigned long		offset;
	unsigned int		npages;
	struct page			**pages;
	dma_addr_t			paddr;

	/* Touched by the interrupt handler. */
	struct v4l2_buffer			vbuf;
	struct list_head			irqlist;
	enum isp_video_buffer_state	state;
	wait_queue_head_t			wait;

	/* For dummy buffers */
	int				order;
	struct page		*dummy_pages;

	unsigned int delay;
};

struct isp_video_queue_operations {
	void (*queue_prepare)(struct isp_video_queue *queue,
			      unsigned int *nbuffers, unsigned int *size);
	int  (*buffer_prepare)(struct isp_video_buffer *buf);
	void (*buffer_queue)(struct isp_video_buffer *buf);
	void (*buffer_cleanup)(struct isp_video_buffer *buf);
};

struct isp_video_queue {
	enum v4l2_buf_type		type;
	const struct isp_video_queue_operations *ops;
	struct device			*dev;
	unsigned int			bufsize;

	unsigned int			count;
	struct isp_video_buffer	*buffers[ISP_VIDEO_MAX_BUFFERS];
	struct isp_video_buffer	*dummy_buffers[ISP_VIDEO_MAX_DUMMY_BUFFERS];
	bool					enable_dummy;
	struct mutex			lock;
	spinlock_t				irqlock;

	unsigned int			streaming;
	struct list_head		queue;
};

int mvisp_video_queue_cleanup(struct isp_video_queue *queue);
int mvisp_video_queue_init(struct isp_video_queue *queue,
			      enum v4l2_buf_type type,
			      const struct isp_video_queue_operations *ops,
			      struct device *dev, unsigned int bufsize,
			      bool enable_dummy);

int mvisp_video_queue_reqbufs(struct isp_video_queue *queue,
				 struct v4l2_requestbuffers *rb,
				 void *ext_dummy);
int mvisp_video_queue_querybuf(struct isp_video_queue *queue,
				  struct v4l2_buffer *vbuf);
int mvisp_video_queue_qbuf(struct isp_video_queue *queue,
			      struct v4l2_buffer *vbuf);
int mvisp_video_queue_dqbuf(struct isp_video_queue *queue,
			       struct v4l2_buffer *vbuf, int nonblocking);
int mvisp_video_queue_streamon(struct isp_video_queue *queue);
void mvisp_video_queue_streamoff(struct isp_video_queue *queue);
int mvisp_video_queue_mmap(struct isp_video_queue *queue,
			      struct vm_area_struct *vma);
unsigned int mvisp_video_queue_poll(struct isp_video_queue *queue,
				       struct file *file, poll_table *wait);

#endif /* ISP_QUEUE_H */
