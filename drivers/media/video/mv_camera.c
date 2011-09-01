/*
 * V4L2 Driver for Marvell Mobile SoC PXA910 CCIC
 * (CMOS Capture Interface Controller)
 *
 * This driver is based on soc_camera and videobuf2
 * framework, but part of the low level register function
 * is base on cafe_ccic.c
 *
 * Copyright (C) 2011, Marvell International Ltd.
 *	Kassey Lee <ygli@marvell.com>
 *	Angela Wan <jwan@marvell.com>
 *	Lei Wen <leiwen@marvell.com>
 *
 * Copyright 2006 One Laptop Per Child Association, Inc.
 * Copyright 2006-7 Jonathan Corbet <corbet@lwn.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <asm/highmem.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/videodev2.h>
#include <linux/wakelock.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-chip-ident.h>

#include <mach/camera.h>
#include "mv_camera.h"

static struct wake_lock idle_lock;
#define MV_CAM_DRV_NAME "mv-camera"

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff

#define MAX_DMA_BUFS 2

#define CF_DMA_ACTIVE	 3	/* A frame is incoming */
#define CF_SINGLE_BUFFER 5	/* Running with a single buffer */

/*
 * Basic frame stats
 */
static int frames;
static int singles;
static int delivered;

struct yuv_pointer_t {
	dma_addr_t y;
	dma_addr_t u;
	dma_addr_t v;
};

/* buffer for one video frame */
struct mv_buffer {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer vb_buf;
	struct yuv_pointer_t yuv_p;
	struct list_head queue;
	size_t bsize;
	char *vaddr;	/*kernel virtual addr for userptr in vb2_buf*/
	struct page *page;
	int list_init_flag;
};

enum mcam_state {
	S_IDLE,		/* Just hanging around */
	S_STREAMING,	/* Streaming data */
	S_BUFWAIT	/* streaming requested but no buffers yet */
};

struct mv_camera_dev {
	struct soc_camera_host soc_host;

	struct soc_camera_device *icd;
	unsigned int irq;
	void __iomem *base;

	struct platform_device *pdev;
	struct resource *res;

	struct list_head buffers; /*available frames*/
	spinlock_t list_lock;
	struct v4l2_pix_format pix_format;
	unsigned long platform_flags;
	unsigned long flags;
	enum mcam_state state;
	struct mv_buffer *vb_bufs[MAX_DMA_BUFS];
	unsigned int nbufs;		/* How many are used for ccic */
	struct vb2_alloc_ctx *vb_alloc_ctx;
};

/*
 * Device register I/O
 */
static void ccic_reg_write(struct mv_camera_dev *pcdev,
		 unsigned int reg, u32 val)
{
	iowrite32(val, pcdev->base + reg);
}

static u32 ccic_reg_read(struct mv_camera_dev *pcdev, unsigned int reg)
{
	return ioread32(pcdev->base + reg);
}

static void ccic_reg_write_mask(struct mv_camera_dev *pcdev,
		unsigned int reg, u32 val, u32 mask)
{
	u32 v = ccic_reg_read(pcdev, reg);

	v = (v & ~mask) | (val & mask);
	ccic_reg_write(pcdev, reg, v);
}

static void ccic_reg_clear_bit(struct mv_camera_dev *pcdev,
		unsigned int reg, u32 val)
{
	ccic_reg_write_mask(pcdev, reg, 0, val);
}

static void ccic_reg_set_bit(struct mv_camera_dev *pcdev,
		unsigned int reg, u32 val)
{
	ccic_reg_write_mask(pcdev, reg, val, val);
}

static void ccic_enable_clk(struct mv_camera_dev *pcdev)
{
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	int div, ctrl1;

	mcam->enable_clk(&pcdev->pdev->dev, 1);
	div = mcam->get_mclk_src(mcam->mclk_src) / mcam->mclk_min;
	ccic_reg_write(pcdev, REG_CLKCTRL, (mcam->mclk_src << 29) | div);
	ctrl1 = 0x800003c;
	switch (mcam->dma_burst) {
	case 128:
		ctrl1 |= 1 << 25;
		break;
	case 256:
		ctrl1 |= 2 << 25;
		break;
	}
	ccic_reg_write(pcdev, REG_CTRL1, ctrl1);
	if (mcam->bus_type != SOCAM_MIPI)
		ccic_reg_write(pcdev, REG_CTRL3, 0x00004);
}

static void ccic_disable_clk(struct mv_camera_dev *pcdev)
{
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	mcam->enable_clk(&pcdev->pdev->dev, 0);
	ccic_reg_write(pcdev, REG_CLKCTRL, 0x0);
	ccic_reg_write(pcdev, REG_CTRL1, 0x0);
}

static int ccic_config_image(struct mv_camera_dev *pcdev)
{
	int ret = 0;
	u32 imgsz_h;
	u32 imgsz_w;
	unsigned int temp;
	struct v4l2_pix_format *fmt = &pcdev->pix_format;
	u32 widthy, widthuv;
	struct device *dev = &pcdev->icd->dev;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	dev_dbg(dev, " %s %d bytesperline %d height %d\n", __func__, __LINE__,
		fmt->bytesperline, fmt->sizeimage / fmt->bytesperline);
	imgsz_h = (fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK;
	imgsz_w = fmt->bytesperline & IMGSZ_H_MASK;

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420)
		imgsz_w = (fmt->bytesperline * 4 / 3) & IMGSZ_H_MASK;
	else if (fmt->pixelformat == V4L2_PIX_FMT_JPEG)
		imgsz_h = (fmt->sizeimage / fmt->bytesperline) << IMGSZ_V_SHIFT;

	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		widthy = fmt->width * 2;
		widthuv = fmt->width * 2;
		break;
	case V4L2_PIX_FMT_RGB565:
		widthy = fmt->width * 2;
		widthuv = 0;
		break;
	case V4L2_PIX_FMT_JPEG:
		widthy = fmt->bytesperline;
		widthuv = fmt->bytesperline;
		break;
	case V4L2_PIX_FMT_YUV422P:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	case V4L2_PIX_FMT_YUV420:
		widthy = fmt->width;
		widthuv = fmt->width / 2;
		break;
	default:
		break;
	}

	ccic_reg_write(pcdev, REG_IMGPITCH, widthuv << 16 | widthy);
	ccic_reg_write(pcdev, REG_IMGSIZE, imgsz_h | imgsz_w);
	ccic_reg_write(pcdev, REG_IMGOFFSET, 0x0);
	/*
	 * Tell the controller about the image format we are using.
	 */
	switch (fmt->pixelformat) {
	case V4L2_PIX_FMT_YUV422P:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PLANAR |
			C0_YUVE_YVYU, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_YUV420:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_420PL |
			C0_YUVE_YVYU, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_YUYV:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PACKED |
			C0_YUVE_UYVY, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_UYVY:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PACKED |
			C0_YUVE_YUYV, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_JPEG:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_YUV | C0_YUV_PACKED |
			C0_YUVE_YUYV, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_RGB444:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_RGB | C0_RGBF_444 |
			C0_RGB4_XRGB, C0_DF_MASK);
		break;
	case V4L2_PIX_FMT_RGB565:
		ccic_reg_write_mask(pcdev, REG_CTRL0,
			C0_DF_RGB | C0_RGBF_565 |
			C0_RGB5_BGGR, C0_DF_MASK);
		break;
	default:
		dev_err(dev, "Unknown format %c%c%c%c\n",
			 pixfmtstr(fmt->pixelformat));
		break;
	}
	/*
	 * Make sure it knows we want to use hsync/vsync.
	 */
	ccic_reg_write_mask(pcdev, REG_CTRL0, C0_SIF_HVSYNC, C0_SIFM_MASK);
	/*
	 * This field controls the generation of EOF(DVP only)
	 */
	if (mcam->bus_type != SOCAM_MIPI) {
		temp = ccic_reg_read(pcdev, REG_CTRL0);
		temp |=  CO_EOF_VSYNC | C0_VEDGE_CTRL;
		ccic_reg_write(pcdev, REG_CTRL0, temp);
	}

	return ret;
}

static void ccic_irq_enable(struct mv_camera_dev *pcdev)
{
	ccic_reg_write(pcdev, REG_IRQSTAT, FRAMEIRQS_EOF);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, FRAMEIRQS_EOF);
}

static void ccic_irq_disable(struct mv_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, FRAMEIRQS_EOF);
}

static void ccic_start(struct mv_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

static void ccic_stop(struct mv_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_CTRL0, C0_ENABLE);
}

static void ccic_init(struct mv_camera_dev *pcdev)
{
	/*
	 * Make sure it's not powered down.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
	/*
	 * Turn off the enable bit.  It sure should be off anyway,
	 * but it's good to be sure.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL0, C0_ENABLE);
	/*
	 * Mask all interrupts.
	 */
	ccic_reg_write(pcdev, REG_IRQMASK, 0);
}

static void ccic_stop_dma(struct mv_camera_dev *pcdev)
{
	struct device *dev = &pcdev->icd->dev;
	ccic_stop(pcdev);
	/*
	 * CSI2/DPHY need to be cleared, or no EOF will be received
	 */
	ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);
	/*
	 * workaround when stop DMA controller!!!
	 * 1) ccic controller must be stopped first,
	 * and it shoud delay for one frame transfer time at least
	 * 2)and then stop the camera sensor's output
	 *
	 * FIXME! need sillcion to add DMA stop/start bit
	 */
	msleep(200);
	if (test_bit(CF_DMA_ACTIVE, &pcdev->flags))
		dev_err(dev, "Timeout waiting for DMA to end\n");
		/* This would be bad news - what now? */

	ccic_irq_disable(pcdev);
}

static void ccic_power_up(struct mv_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void ccic_power_down(struct mv_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void ccic_config_phy(struct mv_camera_dev *pcdev)
{
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	if (mcam->bus_type == SOCAM_MIPI) {
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, mcam->dphy[0]);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, mcam->dphy[2]);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, mcam->dphy[1]);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x43);
	} else {
		ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
		ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);
	}
}

/*
* Fetch buffer from list, if single mode, we reserve the last buffer
* until new buffer is got, or fetch directly
*/
static void mv_set_contig_buffer(struct mv_camera_dev *pcdev, int frame)
{
	struct mv_buffer *buf;
	struct v4l2_pix_format *fmt = &pcdev->pix_format;

	/*
	 * If there are no available buffers, go into single mode
	 */
	if (list_empty(&pcdev->buffers)) {
		buf = pcdev->vb_bufs[frame ^ 0x1];
		pcdev->vb_bufs[frame] = buf;
		ccic_reg_write(pcdev, frame == 0 ?
				REG_Y0BAR : REG_Y1BAR, buf->yuv_p.y);
		if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P ||
				fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
			ccic_reg_write(pcdev, frame == 0 ?
					REG_U0BAR : REG_U1BAR, buf->yuv_p.u);
			ccic_reg_write(pcdev, frame == 0 ?
					REG_V0BAR : REG_V1BAR, buf->yuv_p.v);
		}
		set_bit(CF_SINGLE_BUFFER, &pcdev->flags);
		singles++;
		return;
	}
	/*
	 * OK, we have a buffer we can use.
	 */
	buf = list_first_entry(&pcdev->buffers, struct mv_buffer, queue);
	list_del_init(&buf->queue);
	ccic_reg_write(pcdev, frame == 0 ?
			REG_Y0BAR : REG_Y1BAR, buf->yuv_p.y);
	if (fmt->pixelformat == V4L2_PIX_FMT_YUV422P ||
			fmt->pixelformat == V4L2_PIX_FMT_YUV420) {
		ccic_reg_write(pcdev, frame == 0 ?
				REG_U0BAR : REG_U1BAR, buf->yuv_p.u);
		ccic_reg_write(pcdev, frame == 0 ?
				REG_V0BAR : REG_V1BAR, buf->yuv_p.v);
	}
	pcdev->vb_bufs[frame] = buf;
	clear_bit(CF_SINGLE_BUFFER, &pcdev->flags);
}

static void mv_dma_setup(struct mv_camera_dev *pcdev)
{
	ccic_reg_write(pcdev, REG_CTRL1, C1_TWOBUFS);
	pcdev->nbufs = 2;
	mv_set_contig_buffer(pcdev, 0);
	mv_set_contig_buffer(pcdev, 1);
}

/*
 * Get everything ready, and start grabbing frames.
 */
static int mv_read_setup(struct mv_camera_dev *pcdev)
{
	ccic_config_phy(pcdev);
	ccic_irq_enable(pcdev);
	mv_dma_setup(pcdev);
	ccic_start(pcdev);
	pcdev->state = S_STREAMING;
	return 0;
}

static int mv_videobuf_setup(struct vb2_queue *vq,
			     u32 *count, u32 *num_planes,
			     unsigned long sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
		struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
		icd->current_fmt->host_fmt);

	int minbufs = 2;
	if (*count < minbufs)
		*count = minbufs;

	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;
	sizes[0] = pcdev->pix_format.sizeimage;
	alloc_ctxs[0] = pcdev->vb_alloc_ctx;
	dev_dbg(icd->dev.parent, "count=%d, size=%lu\n", *count, sizes[0]);
	return 0;
}

static int mv_videobuf_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
		struct soc_camera_device, vb2_vidq);
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb_buf);
	unsigned long size;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
		icd->current_fmt->host_fmt);
	if (bytes_per_line < 0)
		return bytes_per_line;

	dev_dbg(icd->dev.parent, "%s (vb=0x%p) 0x%p %lu\n", __func__, vb,
		vb2_plane_vaddr(vb, 0), vb2_get_plane_payload(vb, 0));

	/* Added list head initialization on alloc */
	WARN(!list_empty(&buf->queue), "Buffer %p on queue!\n", vb);

	BUG_ON(NULL == icd->current_fmt);
	size = vb2_plane_size(vb, 0);
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void mv_videobuf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
		struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->icd->dev;
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb_buf);
	unsigned long flags;
	int start;
	dma_addr_t dma_handle;
	u32 base_size = icd->user_width * icd->user_height;

	dma_handle = vb2_dma_contig_plane_paddr(vb, 0);
	BUG_ON(!dma_handle);
	/* Wait until two buffers already queued to the list, then start DMA*/
	start = (pcdev->state == S_BUFWAIT) && !list_empty(&pcdev->buffers);

	if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV422P) {
		buf->yuv_p.y = dma_handle;
		buf->yuv_p.u = buf->yuv_p.y + base_size;
		buf->yuv_p.v = buf->yuv_p.u + base_size / 2;
	} else if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV420) {
		buf->yuv_p.y = dma_handle;
		buf->yuv_p.u = buf->yuv_p.y + base_size;
		buf->yuv_p.v = buf->yuv_p.u + base_size / 4;
	} else {
		buf->yuv_p.y = dma_handle;
	}

	if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_JPEG) {
		if (dma_handle != PAGE_ALIGN(dma_handle)) {
			dev_dbg(dev, "[%s] Phy addr is not page"
					"aligned 0x%x\n", __func__, dma_handle);
			BUG_ON(1);
			return;
		}
		buf->page = pfn_to_page(dma_handle >> PAGE_SHIFT);
		if (PageHighMem(buf->page))
			buf->vaddr = kmap_high(buf->page);
		else
			buf->vaddr = page_address(buf->page);
		if (!buf->vaddr) {
			dev_err(dev, "Failed to get vaddr!\n");
			return;
		}
		dev_dbg(dev, "[%s],page paddr:0x%x, vaddr:0x%x\n",
				__func__, dma_handle, (unsigned int)buf->vaddr);
	}
	spin_lock_irqsave(&pcdev->list_lock, flags);
	list_add_tail(&buf->queue, &pcdev->buffers);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
	if (start)
		mv_read_setup(pcdev);
}

static void mv_videobuf_cleanup(struct vb2_buffer *vb)
{
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb_buf);
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
		struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	unsigned long flags;

	spin_lock_irqsave(&pcdev->list_lock, flags);
	/*queue list must be initialized before del*/
	if (buf->list_init_flag)
		list_del_init(&buf->queue);
	buf->vaddr = NULL;
	buf->list_init_flag = 0;
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

/*only the list that queued could be initialized*/
static int mv_videobuf_init(struct vb2_buffer *vb)
{
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb_buf);
	INIT_LIST_HEAD(&buf->queue);
	buf->list_init_flag = 1;
	return 0;
}

static int mv_start_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = container_of(vq,
		struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;

	if (pcdev->state != S_IDLE)
		return -EINVAL;

	/*
	 * Videobuf2 sneakily hoards all the buffers and won't
	 * give them to us until *after* streaming starts.  But
	 * we can't actually start streaming until we have a
	 * destination.  So go into a wait state and hope they
	 * give us buffers soon.
	 */
	if (list_empty(&pcdev->buffers)) {
		pcdev->state = S_BUFWAIT;
		return 0;
	}
	return mv_read_setup(pcdev);
}

static int mv_stop_streaming(struct vb2_queue *vq)
{
	struct soc_camera_device *icd = container_of(vq,
		struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	unsigned long flags;

	if (pcdev->state == S_BUFWAIT) {
		/* They never gave us buffers */
		pcdev->state = S_IDLE;
		return 0;
	}
	if (pcdev->state != S_STREAMING)
		return -EINVAL;

	ccic_stop_dma(pcdev);
	pcdev->state = S_IDLE;

	spin_lock_irqsave(&pcdev->list_lock, flags);
	INIT_LIST_HEAD(&pcdev->buffers);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
	return 0;
}

static struct vb2_ops mv_videobuf_ops = {
	.queue_setup = mv_videobuf_setup,
	.buf_prepare = mv_videobuf_prepare,
	.buf_queue = mv_videobuf_queue,
	.buf_cleanup = mv_videobuf_cleanup,
	.buf_init = mv_videobuf_init,
	.start_streaming = mv_start_streaming,
	.stop_streaming = mv_stop_streaming,
	.wait_prepare = soc_camera_unlock,
	.wait_finish = soc_camera_lock,
};

static int mv_camera_init_videobuf(struct vb2_queue *q,
				   struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;

	int ret = 0;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_USERPTR | VB2_MMAP;
	q->drv_priv = icd;
	q->ops = &mv_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mv_buffer);

	ret = v4l2_subdev_call(sd, core, load_fw);
	if (ret < 0)
		BUG_ON(1);

	pcdev->vb_alloc_ctx = vb2_dma_contig_init_ctx(ici->v4l2_dev.dev);

	return vb2_queue_init(q);
}

/*
 * Hand a completed buffer back to user space.
 */
static void mv_buffer_done(struct mv_camera_dev *pcdev,
		int frame, struct vb2_buffer *vbuf)
{
	vbuf->v4l2_buf.bytesused = pcdev->pix_format.sizeimage;
	vb2_set_plane_payload(vbuf, 0, pcdev->pix_format.sizeimage);
	vb2_buffer_done(vbuf, VB2_BUF_STATE_DONE);
}

/*
 * Interrupt handler stuff
 */
static void ccic_frame_complete(struct mv_camera_dev *pcdev, int frame)
{
	struct mv_buffer *buf = pcdev->vb_bufs[frame];
	struct device *dev = &pcdev->icd->dev;
	char *vaddr;

	clear_bit(CF_DMA_ACTIVE, &pcdev->flags);
	frames++;
	/*
	 * "This should never happen"
	 */
	if (pcdev->state != S_STREAMING)
		return;

	if (!test_bit(CF_SINGLE_BUFFER, &pcdev->flags)) {
		if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_JPEG
				&& buf->vb_buf.state == VB2_BUF_STATE_ACTIVE) {
			/* As mmap allocate uncached buffer, not
			 * necessary to invalid cache, but for userptr,
			 * we don't know buffer property, to make sure
			 * user could get the real data from DDR, invalid
			 * cache.
			 */
			if (buf->vb_buf.v4l2_buf.memory == V4L2_MEMORY_USERPTR)
				dma_map_page(dev, buf->page, 0,
					buf->vb_buf.v4l2_planes[0].length,
					DMA_FROM_DEVICE);

			vaddr = (char *)buf->vaddr;
			if (vaddr[0] != 0xff || vaddr[1] != 0xd8) {
				dev_err(dev, "cam: JPEG error 0x%x%x!\n",
						vaddr[0], vaddr[1]);
				return;
			}
		}
		delivered++;
		mv_buffer_done(pcdev, frame, &buf->vb_buf);
	}
	mv_set_contig_buffer(pcdev, frame);
}

static irqreturn_t mv_camera_irq(int irq, void *data)
{
	struct mv_camera_dev *pcdev = data;
	u32 irqs;
	u32 frame;

	irqs = ccic_reg_read(pcdev, REG_IRQSTAT);
	ccic_reg_write(pcdev, REG_IRQSTAT, irqs);

	for (frame = 0; frame < pcdev->nbufs; frame++)
		if (irqs & (IRQ_EOF0 << frame))
			ccic_frame_complete(pcdev, frame);

	if (irqs & (IRQ_SOF0 | IRQ_SOF1 | IRQ_SOF2))
		set_bit(CF_DMA_ACTIVE, &pcdev->flags);

	return IRQ_HANDLED;
}

static int mv_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	if (pcdev->icd)
		return -EBUSY;

	frames = singles = delivered = 0;

#ifdef CONFIG_PM
	mcam->controller_power(1);
#endif
	wake_lock(&idle_lock);
	pcdev->icd = icd;
	pcdev->state = S_IDLE;
	ccic_enable_clk(pcdev);
	ccic_init(pcdev);
	ccic_power_up(pcdev);
	return 0;
}

static void mv_camera_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	BUG_ON(icd != pcdev->icd);

	vb2_dma_contig_cleanup_ctx(pcdev->vb_alloc_ctx);
	dev_err(icd->dev.parent, "Release, %d frames, %d"
			"singles, %d delivered\n", frames, singles, delivered);
	ccic_disable_clk(pcdev);
	pcdev->icd = NULL;
	wake_unlock(&idle_lock);
#ifdef CONFIG_PM
	mcam->controller_power(0);
#endif
}

static int mv_camera_set_bus_param(struct soc_camera_device
				   *icd, __u32 pixfmt)
{
	struct device *dev = icd->dev.parent;
	int ret;
	unsigned long common_flags;

	common_flags = icd->ops->query_bus_param(icd);

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}
	return ret;
}

static int mv_camera_set_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_mbus_framefmt mf;
	int ret;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	dev_dbg(dev, "S_FMT %c%c%c%c, %ux%u\n",
		pixfmtstr(pix->pixelformat), pix->width, pix->height);
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(dev, "Format %c%c%c%c not found\n",
			pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = V4L2_FIELD_NONE;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0) {
		dev_err(dev, "s_mbus_fmt failed %s %d\n", __func__, __LINE__);
		return ret;
	}
	if (mf.code != xlate->code) {
		dev_err(dev, "wrong code %s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	pix->width = mf.width;
	pix->height = mf.height;
	pix->field = mf.field;
	pix->colorspace = mf.colorspace;
	pcdev->pix_format.sizeimage = pix->sizeimage;
	icd->current_fmt = xlate;

	memcpy(&(pcdev->pix_format), pix, sizeof(struct v4l2_pix_format));
	ret = ccic_config_image(pcdev);

	return ret;
}

static int mv_camera_try_fmt(struct soc_camera_device *icd,
		struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_err(dev, "Format %c%c%c%c not found\n",
			pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
							xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
	if (pix->pixelformat == V4L2_PIX_FMT_JPEG) {
		pix->bytesperline = 2048;
		/*Todo: soc_camera_try_fmt could clear
		 * sizeimage, we can't get the value from
		 * userspace, just hard coding
		 */
		/*pix->sizeimage = 2048*1000; */
	}
	else
		pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = V4L2_FIELD_NONE;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width = mf.width;
	pix->height = mf.height;
	pix->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field = V4L2_FIELD_NONE;
		break;
	default:
		dev_err(icd->dev.parent, "Field type %d unsupported.\n",
			mf.field);
		return -EINVAL;
	}

	return ret;
}

static unsigned int mv_camera_poll(struct file *file, poll_table * pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int mv_camera_querycap(struct soc_camera_host *ici,
		struct v4l2_capability *cap)
{
	struct v4l2_dbg_chip_ident id;
	struct mv_camera_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	int ret = 0;

	cap->version = KERNEL_VERSION(0, 0, 5);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	ret = v4l2_subdev_call(sd, core, g_chip_ident, &id);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	strcpy(cap->card, mcam->name);

	if (id.ident == V4L2_IDENT_OV5642)
		strcpy(cap->driver, "ov5642");
	else if (id.ident == V4L2_IDENT_OV5640)
		strcpy(cap->driver, "ov5640");
	else
		strcpy(cap->driver, "unknow sensor");
	return 0;
}

/*CameraEngine need set_parm to return 0*/
static int mv_camera_set_parm(struct soc_camera_device *icd,
		struct v4l2_streamparm *para)
{
	return 0;
}

static const struct soc_mbus_pixelfmt ccic_formats[] = {
	{
		.fourcc = V4L2_PIX_FMT_YUV422P,
		.name = "YUV422PLANAR",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_YUV420,
		.name = "YUV420PLANAR",
		.bits_per_sample = 12,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
	},
	{
		.fourcc = V4L2_PIX_FMT_UYVY,
		.name = "YUV422PACKED",
		.bits_per_sample = 8,
		.packing = SOC_MBUS_PACKING_2X8_PADLO,
		.order = SOC_MBUS_ORDER_LE,
	},
};

static int mv_camera_get_formats(struct soc_camera_device *icd, u32 idx,
		struct soc_camera_format_xlate  *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	int formats = 0, ret;
	enum v4l2_mbus_pixelcode code;
	const struct soc_mbus_pixelfmt *fmt;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fmt, idx, &code);
	if (ret < 0)
		/* No more formats */
		return 0;

	fmt = soc_mbus_get_fmtdesc(code);
	if (!fmt) {
		dev_err(dev, "Invalid format code #%u: %d\n", idx, code);
		return 0;
	}

	switch (code) {
		/* refer to mbus_fmt struct */
	case V4L2_MBUS_FMT_UYVY8_2X8:
		/* TODO: add support for YUV420 and YUV422P */
		formats = ARRAY_SIZE(ccic_formats);

		if (xlate) {
			int i;
			for (i = 0; i < ARRAY_SIZE(ccic_formats); i++) {
				xlate->host_fmt = &ccic_formats[i];
				xlate->code = code;
				xlate++;
			}
		}
		return formats;
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (xlate)
			dev_err(dev, "Providing format %s\n", fmt->name);
		break;
	default:
		/*
		 * camera controller can not support
		 * this format, which might supported by the sensor
		 */
		dev_warn(dev, "Not support fmt %s\n", fmt->name);
		return 0;
	}

	formats++;
	if (xlate) {
		xlate->host_fmt = fmt;
		xlate->code = code;
		xlate++;
	}

	return formats;
}

static struct soc_camera_host_ops mv_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = mv_camera_add_device,
	.remove = mv_camera_remove_device,
	.set_fmt = mv_camera_set_fmt,
	.try_fmt = mv_camera_try_fmt,
	.set_parm = mv_camera_set_parm,
	.init_videobuf2 = mv_camera_init_videobuf,
	.poll = mv_camera_poll,
	.querycap = mv_camera_querycap,
	.set_bus_param = mv_camera_set_bus_param,
	.get_formats = mv_camera_get_formats,
};

static int __devinit mv_camera_probe(struct platform_device *pdev)
{
	struct mv_camera_dev *pcdev;
	struct mv_cam_pdata *mcam;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err = 0;

	mcam = pdev->dev.platform_data;
	if (!mcam || !mcam->init_clk || !mcam->enable_clk
			|| !mcam->get_mclk_src)
		return -EINVAL;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0)
		return -ENODEV;

	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		return -ENOMEM;
	}

	pcdev->res = res;
	pcdev->pdev = pdev;

	err = mcam->init_clk(&pdev->dev, 1);
	if (err)
		goto exit_clk;

	INIT_LIST_HEAD(&pcdev->buffers);

	spin_lock_init(&pcdev->list_lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, resource_size(res),
				MV_CAM_DRV_NAME)) {
		err = -EBUSY;
		dev_err(&pdev->dev, "request_mem_region resource failed\n");
		goto exit_release;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "ioremap resource failed\n");
		goto exit_iounmap;
	}
	pcdev->irq = irq;
	pcdev->base = base;
	/* request irq */
	err = request_irq(pcdev->irq, mv_camera_irq, 0, MV_CAM_DRV_NAME, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Camera interrupt register failed\n");
		goto exit_free_irq;
	}

	ccic_enable_clk(pcdev);
#if defined(CONFIG_PM)
	mcam->controller_power(0);
#endif
	pcdev->soc_host.drv_name = MV_CAM_DRV_NAME;
	pcdev->soc_host.ops = &mv_soc_camera_host_ops;
	pcdev->soc_host.priv = pcdev;
	pcdev->soc_host.v4l2_dev.dev = &pdev->dev;
	pcdev->soc_host.nr = pdev->id;
	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto exit_free_irq;
	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
	ccic_power_down(pcdev);
exit_iounmap:
	iounmap(base);
exit_release:
	release_mem_region(res->start, resource_size(res));
exit_clk:
	mcam->init_clk(&pdev->dev, 0);

	return err;
}

static int __devexit mv_camera_remove(struct platform_device *pdev)
{

	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct mv_camera_dev *pcdev = container_of(soc_host,
		struct mv_camera_dev, soc_host);
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	struct resource *res;

	mcam->init_clk(&pdev->dev, 0);
	ccic_power_down(pcdev);
	free_irq(pcdev->irq, pcdev);

	soc_camera_host_unregister(soc_host);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	dev_info(&pdev->dev, "MV Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mv_camera_driver = {
	.driver = {
		.name = MV_CAM_DRV_NAME,
	},
	.probe = mv_camera_probe,
	.remove = __devexit_p(mv_camera_remove),
};

static int __init mv_camera_init(void)
{
	wake_lock_init(&idle_lock, WAKE_LOCK_IDLE, "mv_camera_idle");
	return platform_driver_register(&mv_camera_driver);
}

static void __exit mv_camera_exit(void)
{
	platform_driver_unregister(&mv_camera_driver);
	wake_lock_destroy(&idle_lock);
}

module_init(mv_camera_init);
module_exit(mv_camera_exit);

MODULE_DESCRIPTION("Marvell CCIC driver");
MODULE_AUTHOR("Kassey Lee <ygli@marvell.com>");
MODULE_LICENSE("GPL");
