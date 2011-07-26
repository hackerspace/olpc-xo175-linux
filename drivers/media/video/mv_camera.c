/*
 * V4L2 Driver for PXA168/910/668 CCIC
 *
 * Based on linux/drivers/media/video/pxa_camera.c
 *
 * Copyright (C) 2010, Marvell International Ltd.
 *              Kassey Lee <ygli@marvell.com>
 *		Angela Wan <jwan@marvell.com>
 *		Lei Wen <leiwen@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/version.h>
#include <linux/videodev2.h>

#include <linux/earlysuspend.h>
#include <linux/wakelock.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>
#include <media/videobuf-dma-contig.h>

#include <mach/camera.h>
#include <mach/dma.h>
#include "mv_camera.h"

static struct wake_lock idle_lock;
#define PXA_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)
#define CAM_DRV_NAME "mv-camera"
#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
			((x) >> 24) & 0xff

static int dma_buf_size = PAGE_ALIGN(2048 * 1536 * 2);	/* Worst case */

struct yuv_pointer_t {
	dma_addr_t y;
	dma_addr_t u;
	dma_addr_t v;
};

/* buffer for one video frame */
struct mv_buffer {
	/* common v4l buffer stuff -- must be first */
	struct videobuf_buffer vb;
	enum v4l2_mbus_pixelcode code;
	struct yuv_pointer_t yuv_p;
	struct page *page;
	char *buffer;		/* Where it lives in kernel space */
};

struct mv_camera_dev {
	struct soc_camera_host soc_host;
	struct soc_camera_device *icd;

	unsigned int irq;
	void __iomem *base;

	struct platform_device *pdev;
	struct resource *res;

	struct list_head capture;
	struct list_head sb_dma;	/* dma list (dev_lock) */

	spinlock_t list_lock;
	struct v4l2_pix_format pix_format;
	/*
	 * internal use only
	 * rubbisth_buf is used when available
	 * for DMA is less than 3
	 */
	void *dummy_buf_virt;
	dma_addr_t dummy_buf_phy;

	/* PM */
	struct early_suspend ccic_early_suspend;
	unsigned int state;
	unsigned int bus_type;
};

/*
 * Device register I/O
 */
static void ccic_reg_write(struct mv_camera_dev *pcdev,
				  unsigned int reg, u32 val)
{
	iowrite32(val, pcdev->base + reg);
}

static unsigned int ccic_reg_read(struct mv_camera_dev *pcdev,
					 u32 reg)
{
	return ioread32(pcdev->base + reg);
}

static void ccic_reg_write_mask(struct mv_camera_dev *pcdev,
				       unsigned int reg,  u32 val,
				       u32 mask)
{
	unsigned int v = ccic_reg_read(pcdev, reg);

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

static int ccic_enable_clk(struct mv_camera_dev *pcdev)
{
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	int div, ctrl1;
	mcam->enable_clk(&pcdev->pdev->dev, 1);

	div = mcam->get_mclk_src(mcam->mclk_src) / mcam->mclk_min;
	ccic_reg_write(pcdev, REG_CLKCTRL, (mcam->mclk_src << 29) | div);
	ctrl1 = 0x800003c;
	switch (mcam->dma_burst) {
	case 128:
		ctrl1 |= 1 << 25; break;
	case 256:
		ctrl1 |= 2 << 25; break;
	}
	ccic_reg_write(pcdev, REG_CTRL1, ctrl1);
	if (mcam->bus_type != SOCAM_MIPI)
		ccic_reg_write(pcdev, 0x1ec, 0x00004);

	return 0;
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
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	imgsz_h = (fmt->height << IMGSZ_V_SHIFT) & IMGSZ_V_MASK;
	imgsz_w = fmt->bytesperline & IMGSZ_H_MASK;

	dev_err(pcdev->soc_host.v4l2_dev.dev,
			KERN_ERR " %s %d bytesperline %d height %d\n", __func__,
			__LINE__, fmt->bytesperline,
			fmt->sizeimage / fmt->bytesperline);

	if (fmt->pixelformat == V4L2_PIX_FMT_YUV420)
		imgsz_w = (fmt->bytesperline * 4 / 3) & IMGSZ_H_MASK;
	else if (fmt->pixelformat == V4L2_PIX_FMT_JPEG)
		imgsz_h = (fmt->sizeimage / fmt->bytesperline) << IMGSZ_V_SHIFT;

	/* YPITCH just drops the last two bits */
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
		dev_err(pcdev->soc_host.v4l2_dev.dev,
				"Unknown format %c%c%c%c\n",
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
	/*
	 * Clear any pending interrupts, since we do not
	 * expect to have I/O active prior to enabling.
	 */
	ccic_reg_write(pcdev, REG_IRQSTAT, FRAMEIRQS);
	ccic_reg_set_bit(pcdev, REG_IRQMASK, FRAMEIRQS);
}

static void ccic_irq_disable(struct mv_camera_dev *pcdev)
{
	ccic_reg_clear_bit(pcdev, REG_IRQMASK, FRAMEIRQS);
}

/*
* Make the controller start grabbing images.  Everything must
* be set up before doing this.
*/
static void ccic_start(struct mv_camera_dev *pcdev)
{
	/* set_bit performs a read, so no other barrier should be
	   needed here */
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
	/*
	 * Clock the sensor appropriately. Controller clock should
	 * be 48MHz, sensor "typical" value is half that.
	 */
}

/*
* Stop the controller, and don't return
* until we're really sure that no further DMA is going on.
*/
static void ccic_stop_dma(struct mv_camera_dev *pcdev)
{
	/*
	 * Theory: stop the camera controller (whether it is operating
	 * or not).  Delay briefly just in case we race with the SOF
	 * interrupt, then wait until no DMA is active.
	 */
	ccic_stop(pcdev);
	/* CSI2/DPHY need to be cleared, or no EOF will be received */
	ccic_reg_write(pcdev, REG_CSI2_DPHY3, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY6, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_DPHY5, 0x0);
	ccic_reg_write(pcdev, REG_CSI2_CTRL0, 0x0);

	/* 166 = 1/6, jpeg fps is 8, that is why we set timeout as 200 */
	mdelay(200);
}

/*
* Power up and down.
*/
static void ccic_power_up(struct mv_camera_dev *pcdev)
{
	/*
	 * Part one of the sensor dance: turn the global
	 * GPIO signal on.
	 */
	ccic_reg_clear_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void ccic_power_down(struct mv_camera_dev *pcdev)
{
	ccic_reg_set_bit(pcdev, REG_CTRL1, C1_PWRDWN);
}

static void dma_attach_buf(struct mv_camera_dev *pcdev)
{
	struct mv_buffer *buf = NULL;
	struct videobuf_buffer *vb = NULL;
	int frame = 0;

	while (!list_empty(&pcdev->capture) && (frame < 2)) {

		buf = list_entry(pcdev->capture.next,
				 struct mv_buffer, vb.queue);

		if (NULL == buf)
			BUG_ON(1);
		if (NULL == (void *)(buf->yuv_p.y))
			BUG_ON(1);
		ccic_reg_write(pcdev, REG_Y0BAR + (frame << 2), buf->yuv_p.y);
		ccic_reg_write(pcdev, REG_U0BAR + (frame << 2), buf->yuv_p.u);
		ccic_reg_write(pcdev, REG_V0BAR + (frame << 2), buf->yuv_p.v);
		frame++;
		vb = &buf->vb;
		if (NULL == vb)
			BUG_ON(1);
		list_move_tail(&vb->queue, &pcdev->sb_dma);
		vb->state = VIDEOBUF_QUEUED;
		dev_err(pcdev->soc_host.v4l2_dev.dev, "%s %d\n",
				__func__, frame);
	}
}

static void mv_camera_start_capture(struct mv_camera_dev
					*pcdev)
{
	dev_dbg(pcdev->soc_host.v4l2_dev.dev, "%s\n", __func__);
	ccic_start(pcdev);
}

static void mv_camera_stop_capture(struct mv_camera_dev
				       *pcdev)
{
	ccic_stop_dma(pcdev);
	dev_dbg(pcdev->soc_host.v4l2_dev.dev, "%s\n", __func__);
}

/*
*  Videobuf operations
*/
static int mv_videobuf_setup(struct videobuf_queue *vq,
				 unsigned int *count, unsigned int *size)
{
	struct soc_camera_device *icd = vq->priv_data;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;

	if (bytes_per_line < 0)
		return bytes_per_line;

	dev_dbg(icd->dev.parent, "count=%d, size=%d\n", *count, *size);

	*size = bytes_per_line * icd->user_height;

	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG) {
		*size = pcdev->pix_format.sizeimage;
		bytes_per_line = pcdev->pix_format.bytesperline;
	}
	/*
	   if (0 == *count)
	   *count = 32;
	   if (*size * *count > vid_limit * 1024 * 1024)
	   *count = (vid_limit * 1024 * 1024) / *size;
	 */
	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct mv_buffer *buf)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct videobuf_buffer *vb = &buf->vb;

	BUG_ON(in_interrupt());

	dev_dbg(icd->dev.parent, "%s (vb=0x%p) 0x%08lx %d\n",
		__func__, vb, vb->baddr, vb->bsize);

	/*
	 * This waits until this buffer is out of danger, i.e., until it is no
	 * longer in STATE_QUEUED or STATE_ACTIVE
	 */
	videobuf_waiton(vq, vb, 0, 0);
	videobuf_dma_contig_free(vq, vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static int mv_videobuf_prepare(struct videobuf_queue *vq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->icd->dev;
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb);
	int ret;
	dma_addr_t dma_handle;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
						     icd->
						     current_fmt->host_fmt);
	unsigned long flags;

	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG)
		bytes_per_line = pcdev->pix_format.bytesperline;

	if (bytes_per_line < 0)
		return bytes_per_line;

	if (vb->baddr & 4095) {
		dev_err(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);
		dev_err(dev, "buffer base addr is not 4096 bytes aligned\n");
		BUG_ON(1);
	}

	if (vb->size & 31) {
		dev_err(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
			vb, vb->baddr, vb->bsize);
		dev_err(dev, "buffer size is not 32 bytes aligned\n");
		BUG_ON(1);
	}

	/* Added list head initialization on alloc */
	WARN_ON(!list_empty(&vb->queue));

	/*
	 * This can be useful if you want to see if we actually fill
	 * the buffer with something
	 */
	/*
	   memset((void *)vb->baddr, 0xaa, vb->bsize);
	 */
	BUG_ON(NULL == icd->current_fmt);

	/*
	 * I think, in buf_prepare you only have to protect global data,
	 * the actual buffer is yours
	 */

	if (buf->code != icd->current_fmt->code ||
	    vb->width != icd->user_width ||
	    vb->height != icd->user_height || vb->field != field) {
		buf->code = icd->current_fmt->code;
		vb->width = icd->user_width;
		vb->height = icd->user_height;
		vb->field = field;
		vb->state = VIDEOBUF_NEEDS_INIT;
	}
	if (icd->current_fmt->host_fmt->fourcc == V4L2_PIX_FMT_JPEG)
		vb->size = pcdev->pix_format.sizeimage;
	else
		vb->size = bytes_per_line * vb->height;
	if (0 != vb->baddr && vb->bsize < vb->size) {
		ret = -EINVAL;
		goto out;
	}

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret)
			goto fail;

		vb->state = VIDEOBUF_PREPARED;
	}

	dma_handle = videobuf_to_dma_contig(vb);
	if (!dma_handle) {
		dev_err(dev, "faied to get phy addr %s %d\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -ENOMEM;
	}

	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d dma_addr =  0x%p\n",
		__func__, vb, vb->baddr, vb->bsize, (void *)dma_handle);

	buf->page = phys_to_page(dma_handle);
	if (!buf->page) {
		dev_err(dev, "fail to get page %s %d info\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	if (PageHighMem(buf->page)) {
		dev_err(dev, "camera: HIGHMEM is not supported %s %d info\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	buf->buffer = page_address(buf->page);
	if (!buf->buffer) {
		dev_err(dev, "camera: fail to get buffer info %s %d\n",
			__func__, __LINE__);
		BUG_ON(1);
		return -EFAULT;
	}

	if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV422P) {
		buf->yuv_p.y = dma_handle;
		buf->yuv_p.u = buf->yuv_p.y + vb->width * vb->height;
		buf->yuv_p.v = buf->yuv_p.u + vb->width * vb->height / 2;
	} else if (pcdev->pix_format.pixelformat == V4L2_PIX_FMT_YUV420) {
		buf->yuv_p.y = dma_handle;
		buf->yuv_p.u = buf->yuv_p.y + vb->width * vb->height;
		buf->yuv_p.v = buf->yuv_p.u + vb->width * vb->height / 4;
	} else {
		buf->yuv_p.y = dma_handle;
		buf->yuv_p.u = pcdev->dummy_buf_phy;
		buf->yuv_p.v = pcdev->dummy_buf_phy;
	}

	spin_lock_irqsave(&pcdev->list_lock, flags);
	list_add_tail(&vb->queue, &pcdev->capture);
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
	return 0;

fail:
	free_buffer(vq, buf);
out:
	return ret;
}

/* Called under spinlock_irqsave(&pcdev->lock, ...) */
static void mv_videobuf_queue(struct videobuf_queue *vq,
				  struct videobuf_buffer *vb)
{
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct device *dev = &pcdev->icd->dev;

	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d dma_addr =  0x%p\n",
		__func__, vb, vb->baddr, vb->bsize,
		(void *)videobuf_to_dma_contig(vb));

	vb->state = VIDEOBUF_ACTIVE;
	if (vq->streaming == 1) {
		if (pcdev->state == 0) {
			dma_attach_buf(pcdev);
			ccic_irq_enable(pcdev);
			mv_camera_start_capture(pcdev);
			pcdev->state = 1;
		}
	}
}

static void mv_videobuf_release(struct videobuf_queue *vq,
				    struct videobuf_buffer *vb)
{
	struct mv_buffer *buf = container_of(vb, struct mv_buffer, vb);
	struct soc_camera_device *icd = vq->priv_data;
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	dev_dbg(dev, "%s (vb=0x%p) 0x%08lx %d\n", __func__,
		vb, vb->baddr, vb->bsize);
	/* fix me */
	vb->state = VIDEOBUF_DONE;
	switch (vb->state) {
	case VIDEOBUF_ACTIVE:
		dev_dbg(dev, "%s (active)\n", __func__);
		break;
	case VIDEOBUF_QUEUED:
		dev_dbg(dev, "%s (queued)\n", __func__);
		break;
	case VIDEOBUF_PREPARED:
		dev_dbg(dev, "%s (prepared)\n", __func__);
		break;
	default:
		dev_dbg(dev, "%s (unknown)\n", __func__);
		break;
	}
	if (vq->streaming == 0) {
		if (pcdev->state == 1) {
			ccic_irq_disable(pcdev);
			pcdev->state = 0;
		}
		INIT_LIST_HEAD(&pcdev->capture);
		INIT_LIST_HEAD(&pcdev->sb_dma);
	}

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops mv_videobuf_ops = {
	.buf_setup = mv_videobuf_setup,
	.buf_prepare = mv_videobuf_prepare,
	.buf_queue = mv_videobuf_queue,
	.buf_release = mv_videobuf_release,
};

static void mv_camera_init_videobuf(struct videobuf_queue *q,
					struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	int ret = 0;

	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	/*
	 * We must pass NULL as dev pointer, then all pci_* dma operations
	 * transform to normal dma_* ones.
	 */
	videobuf_queue_dma_contig_init(q, &mv_videobuf_ops, NULL,
				       &pcdev->list_lock,
				       V4L2_BUF_TYPE_VIDEO_CAPTURE,
				       V4L2_FIELD_NONE,
				       sizeof(struct mv_buffer), icd,
				       &icd->video_lock);

	ret = v4l2_subdev_call(sd, core, load_fw);
	if (ret < 0)
		BUG_ON(1);
}

static inline void ccic_dma_done(struct mv_camera_dev *pcdev, u32 frame)
{
	struct mv_buffer *buf = NULL;
	struct videobuf_buffer *vb = NULL;
	unsigned long flags = 0;
	dma_addr_t dma_base_reg = 0;
	struct device *dev = &pcdev->icd->dev;
	unsigned long y, u, v;

	spin_lock_irqsave(&pcdev->list_lock, flags);

	dma_base_reg = ccic_reg_read(pcdev, REG_Y0BAR + (frame << 2));

	if (pcdev->dummy_buf_phy == dma_base_reg)
		goto update_buf2dma;

	list_for_each_entry(buf, &pcdev->sb_dma, vb.queue) {

		if (dma_base_reg == buf->yuv_p.y) {
			vb = &buf->vb;
			if (pcdev->pix_format.pixelformat ==
					V4L2_PIX_FMT_JPEG) {
				dma_map_page(dev, buf->page, 0,
						64, DMA_FROM_DEVICE);

				if (!(buf->buffer[0] == 0xff \
						&& buf->buffer[1] == 0xd8)) {
					dev_err(pcdev->soc_host.v4l2_dev.dev,
						"DROP JPEG 0x%04x\n",
						(short)(buf->buffer[0]));
					/*
					 * this buffer is wrong, no need to
					 * update the DMA address
					 */
					goto out;
				}
			}
			/* delete it from captuer */
			list_del_init(&vb->queue);
			vb->state = VIDEOBUF_DONE;
			dma_map_page(dev, buf->page, 0,
					vb->bsize, DMA_FROM_DEVICE);
			wake_up(&vb->done);

			break;
		}
	}
update_buf2dma:
	/* update the DMA base address */
	if (list_empty(&pcdev->capture)) {
		dev_dbg(pcdev->soc_host.v4l2_dev.dev,
				"%s %d rubbish\n", __func__, __LINE__);
		/* use dummy buffer when no video buffer available */
		y = u = v = pcdev->dummy_buf_phy;
	} else {
		struct mv_buffer *newbuf = list_entry(pcdev->capture.next,
				struct mv_buffer, vb.queue);
		vb = &newbuf->vb;
		list_move_tail(&vb->queue, &pcdev->sb_dma);

		y = newbuf->yuv_p.y;
		u = newbuf->yuv_p.u;
		v = newbuf->yuv_p.v;

		vb->state = VIDEOBUF_QUEUED;
	}

	/* Setup DMA */
	ccic_reg_write(pcdev, REG_Y0BAR + (frame << 2), y);
	ccic_reg_write(pcdev, REG_U0BAR + (frame << 2), u);
	ccic_reg_write(pcdev, REG_V0BAR + (frame << 2), v);

out:
	spin_unlock_irqrestore(&pcdev->list_lock, flags);
}

/* #define IRQ_DEBUG */
#ifdef IRQ_DEBUG
static inline void ccic_irq_debug(struct mv_camera_dev *pcdev)
{
	int line_num = ccic_reg_read(pcdev, REG_LNNUM);
	dev_err(pcdev->soc_host.v4l2_dev.dev, " line_num = %d\n", line_num);
}
#endif

static irqreturn_t mv_camera_irq(int irq, void *data)
{
	struct mv_camera_dev *pcdev = data;
	/* do something here */

	unsigned int irqs;
	u32 frame;

	irqs = ccic_reg_read(pcdev, REG_IRQSTAT);
	ccic_reg_write(pcdev, REG_IRQSTAT, irqs);

#ifdef IRQ_DEBUG
	dev_err(pcdev->soc_host.v4l2_dev.dev, " irqs = 0x%08x\n", irqs);
#endif
	if (irqs & IRQ_EOF0)
		frame = 0;
	else if (irqs & IRQ_EOF1)
		frame = 1;
	else if (irqs & IRQ_EOF2)
		frame = 2;
	else
		frame = 0x0f;
	if (0x0f != frame) {

#ifdef IRQ_DEBUG
		ccic_irq_debug(pcdev);
#endif
		ccic_dma_done(pcdev, frame);
	}

	return IRQ_HANDLED;
}

/*
* Get everything ready, and start grabbing frames.
*/
static void ccic_config_phy(struct mv_camera_dev *pcdev)
{
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	if (SOCAM_MIPI & mcam->bus_type) {
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

static int mv_camera_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	if (pcdev->icd)
		return -EBUSY;

	ccic_config_phy(pcdev);
#ifdef CONFIG_PM
	mcam->controller_power(1);
#endif
	wake_lock(&idle_lock);

	ccic_power_up(pcdev);
	pcdev->icd = icd;
	ccic_enable_clk(pcdev);

	return 0;
}

static void mv_camera_remove_device(struct soc_camera_device
					*icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct mv_camera_dev *pcdev = ici->priv;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;

	BUG_ON(icd != pcdev->icd);

	ccic_power_down(pcdev);
	mv_camera_stop_capture(pcdev);
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
	int ret = 0;
	unsigned long common_flags = 0;

	common_flags = icd->ops->query_bus_param(icd);

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}
	return 0;
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
	int ret = 0;

	struct v4l2_pix_format *pix = &f->fmt.pix;
	dev_err(dev, "S_FMT %c%c%c%c, %ux%u\n",
		pixfmtstr(pix->pixelformat), pix->width, pix->height);
	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_err(dev, "Format %c%c%c%c not found\n",
			pixfmtstr(pix->pixelformat));
		return -EINVAL;
	}

	mf.width = pix->width;
	mf.height = pix->height;
	mf.field = pix->field;
	mf.colorspace = pix->colorspace;
	mf.code = xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}
	if (mf.code != xlate->code) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return -EINVAL;
	}

	pix->width = mf.width;
	pix->height = mf.height;
	pix->field = mf.field;
	pix->colorspace = mf.colorspace;
	icd->current_fmt = xlate;

	memcpy(&pcdev->pix_format, pix, sizeof(struct v4l2_pix_format));
	ret = ccic_config_image(pcdev);

	return ret;
}

static int mv_camera_try_fmt(struct soc_camera_device *icd,
				 struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = (struct device *)&icd->dev.parent;
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
	if (pix->pixelformat == V4L2_PIX_FMT_JPEG)
		pix->bytesperline = 2048;
	else
		pix->sizeimage = pix->height * pix->bytesperline;

	/* limit to sensor capabilities */
	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, try_mbus_fmt, &mf);
	if (ret < 0)
		return ret;

	pix->width	= mf.width;
	pix->height = mf.height;
	pix->colorspace = mf.colorspace;

	switch (mf.field) {
	case V4L2_FIELD_ANY:
	case V4L2_FIELD_NONE:
		pix->field	= V4L2_FIELD_NONE;
		break;
	default:
		dev_err(icd->dev.parent, "Field type %d unsupported.\n",
			mf.field);
		return -EINVAL;
	}

	return ret;

}

static int mv_camera_reqbufs(struct soc_camera_device *icd,
			     struct v4l2_requestbuffers *p)
{
	int i;
	for (i = 0; i < p->count; i++) {
		struct mv_buffer *buf = container_of(icd->vb_vidq.bufs[i],
							 struct mv_buffer,
							 vb);
		INIT_LIST_HEAD(&buf->vb.queue);
	}
	return 0;
}

static unsigned int mv_camera_poll(struct file *file, poll_table * pt)
{
	struct soc_camera_device *icd = file->private_data;

	return videobuf_poll_stream(file, &icd->vb_vidq, pt);
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
	cap->version = PXA_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	ret = v4l2_subdev_call(sd, core, g_chip_ident, &id);
	if (ret < 0) {
		dev_err(dev, "%s %d\n", __func__, __LINE__);
		return ret;
	}

	strcpy(cap->card, mcam->name);

	if (id.ident == V4L2_IDENT_OV5642)
		strcpy(cap->driver, "ov5642");
	else
		strcpy(cap->driver, "unknow sensor");
	return 0;
}

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

static int mv_camera_get_formats(struct soc_camera_device *icd,
				     unsigned int idx,
				     struct soc_camera_format_xlate
				     *xlate)
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
		/* camera controller can not support
		 * this format, which might supported by the sensor
		 */
		dev_err(dev, "Not support fmt %s\n", fmt->name);
		return 0;
	}

	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		xlate++;
	}

	return formats;
}

static struct soc_camera_host_ops pxa_soc_camera_host_ops = {
	.owner = THIS_MODULE,
	.add = mv_camera_add_device,
	.remove = mv_camera_remove_device,
	.set_fmt = mv_camera_set_fmt,
	.try_fmt = mv_camera_try_fmt,
	.init_videobuf = mv_camera_init_videobuf,
	.reqbufs = mv_camera_reqbufs,
	.poll = mv_camera_poll,
	.querycap = mv_camera_querycap,
	.set_bus_param = mv_camera_set_bus_param,
	.set_parm = mv_camera_set_parm,
	.get_formats = mv_camera_get_formats,
};

/* Power manament */
static void ccic_sleep_early_suspend(struct early_suspend *h)
{
#if 0
	struct mv_camera_dev *pcdev = container_of(h, \
			struct mv_camera_dev, ccic_early_suspend);
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	mcam->set_clock(&pcdev->pdev->dev, 0);
#endif
}

static void ccic_normal_late_resume(struct early_suspend *h)
{
#if 0
	struct mv_camera_dev *pcdev = container_of(h, \
			struct mv_camera_dev, ccic_early_suspend);
	struct device *dev = &pcdev->icd->dev;
	struct mv_cam_pdata *mcam = pcdev->pdev->dev.platform_data;
	mcam->set_clock(&pcdev->pdev->dev, 1);
#endif
}

static int __devinit mv_camera_probe(struct platform_device *pdev)
{
	struct mv_camera_dev *pcdev;
	struct mv_cam_pdata *mcam;
	struct resource *res;
	void __iomem *base;
	int irq;
	int err = 0;
	int i = 0;

	mcam = pdev->dev.platform_data;
	if (!mcam || !mcam->init_clk || !mcam->enable_clk
			|| !mcam->get_mclk_src)
		return -EINVAL;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq < 0) {
		err = -ENODEV;
		goto exit;
	}
	pcdev = kzalloc(sizeof(*pcdev), GFP_KERNEL);
	if (!pcdev) {
		dev_err(&pdev->dev, "Could not allocate pcdev\n");
		err = -ENOMEM;
		goto exit;
	}

	/* allocate rubbish buffer */
	pcdev->dummy_buf_virt =
	    (void *)__get_free_pages(GFP_KERNEL, get_order(dma_buf_size));
	if (!pcdev->dummy_buf_virt) {
		dev_err(&pdev->dev, "Can't get memory for rubbish buffer\n");
		err = -ENOMEM;
		goto exit_kfree;
	} else {
		pcdev->dummy_buf_phy = __pa(pcdev->dummy_buf_virt);
	}

	pcdev->res = res;
	pcdev->pdev = pdev;

	err = mcam->init_clk(&pdev->dev, 1);
	if (err)
		goto exit_clk;

	INIT_LIST_HEAD(&pcdev->capture);
	INIT_LIST_HEAD(&pcdev->sb_dma);

	spin_lock_init(&pcdev->list_lock);

	/*
	 * Request the regions.
	 */
	if (!request_mem_region(res->start, resource_size(res),
				CAM_DRV_NAME)) {
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
	err = request_irq(pcdev->irq, mv_camera_irq, 0,
			  CAM_DRV_NAME, pcdev);
	if (err) {
		dev_err(&pdev->dev, "Camera interrupt register failed\n");
		goto exit_free_irq;
	}

	/* setup dma with dummy_buf_phy firstly */
	for (i = 0; i < 3; i++) {
		ccic_reg_write(pcdev, REG_Y0BAR + (i << 2),
			       pcdev->dummy_buf_phy);
		ccic_reg_write(pcdev, REG_U0BAR + (i << 2),
			       pcdev->dummy_buf_phy);
		ccic_reg_write(pcdev, REG_V0BAR + (i << 2),
			       pcdev->dummy_buf_phy);
	}
	ccic_enable_clk(pcdev);
	/*
	 * Initialize the controller and leave it powered up.  It will
	 * stay that way until the sensor driver shows up.
	 */
	ccic_init(pcdev);
	ccic_power_up(pcdev);
#if defined(CONFIG_PM) && defined(CONFIG_HAS_EARLYSUSPEND)
	mcam->controller_power(0);

	pcdev->ccic_early_suspend.level =
	    EARLY_SUSPEND_LEVEL_STOP_DRAWING,
	    pcdev->ccic_early_suspend.suspend = ccic_sleep_early_suspend;
	pcdev->ccic_early_suspend.resume = ccic_normal_late_resume;
	register_early_suspend(&pcdev->ccic_early_suspend);
#endif

	pcdev->soc_host.drv_name = CAM_DRV_NAME;
	pcdev->soc_host.ops = &pxa_soc_camera_host_ops;
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
exit_kfree:
	/* free rubbish buffer */
	if (pcdev->dummy_buf_virt)
		free_pages((unsigned long)pcdev->dummy_buf_virt,
			   get_order(dma_buf_size));
	kfree(pcdev);
exit:
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
	free_irq(pcdev->irq, pcdev);

	soc_camera_host_unregister(soc_host);

	iounmap(pcdev->base);

	res = pcdev->res;
	release_mem_region(res->start, resource_size(res));

	kfree(pcdev);

	/* free rubbish buffer */
	if (pcdev->dummy_buf_virt)
		free_pages((unsigned long)pcdev->dummy_buf_virt,
			   get_order(dma_buf_size));

	dev_info(&pdev->dev, "PXA Camera driver unloaded\n");

	return 0;
}

static struct platform_driver mv_camera_driver = {
	.driver = {
		   .name = CAM_DRV_NAME,
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
