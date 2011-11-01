/*
 * ispdma.c
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


#include <linux/device.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/sched.h>


#include "isp.h"
#include "ispreg.h"
#include "ispdma.h"

#define MS_PER_JIFFIES  10

#define ISPDMA_MAX_IN_WIDTH			0x3FFF
#define ISPDMA_MAX_IN_HEIGHT		0x1FFF
#define ISPDMA_MAX_DISP_PITCH		0x3FFFFFF
#define ISPDMA_MAX_DISP_WIDTH		0x3FFFFFF
#define ISPDMA_MAX_DISP_HEIGHT		0x1FFF
#define ISPDMA_MAX_CODEC_PITCH		0x3FFFFFF
#define ISPDMA_MAX_CODEC_WIDTH		0x3FFFFFF
#define ISPDMA_MAX_CODEC_HEIGHT		0x1FFF

#define FBTX0_DMAENA		0x2
#define FBTX1_DMAENA		0x3
#define FBTX2_DMAENA		0x4
#define FBTX3_DMAENA		0x5
#define FBRX0_DMAENA		0x6
#define FBRX1_DMAENA		0x7
#define FBRX2_DMAENA		0x8
#define FBRX3_DMAENA		0x9
#define FBDMAENA_MAX		0xA

#define IS_CH_ENABLED(regval, ch) (regval & (0x1 << ch))
#define IS_TX_CH(ch) (ch >= FBTX0_DMAENA && ch <= FBTX3_DMAENA)
#define IS_RX_CH(ch) (ch >= FBRX0_DMAENA && ch <= FBRX3_DMAENA)
#define CLEAR_DMA_EN(reg, ch) (reg &= ~(0x1 << ch))
#define CLEAR_CLK_EN(reg, ch) do {\
	reg &= ~(0x3 << (ch - FBTX0_DMAENA));\
	reg |= (0x2 << (ch - FBTX0_DMAENA));\
	} while (0)

#define CLEAR_TX_IRQ_MASK(reg, ch) (reg &= ~((0x1 << ch)|(0x1 << (ch + 11))))
#define CLEAR_RX_IRQ_MASK(reg, ch) (reg &= ~(0x1 << ch))

#define SET_DMA_EN(reg, ch) (reg |= (0x1 << ch))
#define SET_CLK_EN(reg, ch) (reg |= (0x3 << (ch - FBTX0_DMAENA)))
#define SET_TX_IRQ_MASK(reg, ch) (reg |= ((0x1 << ch)|(0x1 << (ch + 11))))
#define SET_RX_IRQ_MASK(reg, ch) (reg |= (0x1 << ch))

struct isp_reg_context ispdma_reg_list[ISPDMA_INPSDMA_MAX_CTX] = {
	{ISPDMA_IRQMASK, 0},
	{ISPDMA_DMA_ENA, 0},
	{ISPDMA_INPSDMA_CTRL, 0},
	{ISPDMA_CLKENA, 0},
	{ISPDMA_MAINCTRL, 0},
	{ISPDMA_INSZ, 0},
	{ISPDMA_FBTX0_SDCA, 0},
	{ISPDMA_FBTX0_DCSZ, 0},
	{ISPDMA_FBTX0_CTRL, 0},
	{ISPDMA_FBTX0_DSTSZ, 0},
	{ISPDMA_FBTX0_DSTADDR, 0},
	{ISPDMA_FBTX0_TMR, 0},
	{ISPDMA_FBTX0_RAMCTRL, 0},
	{ISPDMA_FBRX0_SDCA, 0},
	{ISPDMA_FBRX0_DCSZ, 0},
	{ISPDMA_FBRX0_CTRL, 0},
	{ISPDMA_FBRX0_TMR, 0},
	{ISPDMA_FBRX0_RAMCTRL, 0},
	{ISPDMA_FBRX0_STAT, 0},
	{ISPDMA_FBTX1_SDCA, 0},
	{ISPDMA_FBTX1_DCSZ, 0},
	{ISPDMA_FBTX1_CTRL, 0},
	{ISPDMA_FBTX1_DSTSZ, 0},
	{ISPDMA_FBTX1_DSTADDR, 0},
	{ISPDMA_FBTX1_TMR, 0},
	{ISPDMA_FBTX1_RAMCTRL, 0},
	{ISPDMA_FBRX1_SDCA, 0},
	{ISPDMA_FBRX1_DCSZ, 0},
	{ISPDMA_FBRX1_CTRL, 0},
	{ISPDMA_FBRX1_TMR, 0},
	{ISPDMA_FBRX1_RAMCTRL, 0},
	{ISPDMA_FBRX1_STAT, 0},
	{ISPDMA_FBTX2_SDCA, 0},
	{ISPDMA_FBTX2_DCSZ, 0},
	{ISPDMA_FBTX2_CTRL, 0},
	{ISPDMA_FBTX2_DSTSZ, 0},
	{ISPDMA_FBTX2_DSTADDR, 0},
	{ISPDMA_FBTX2_TMR, 0},
	{ISPDMA_FBTX2_RAMCTRL, 0},
	{ISPDMA_FBRX2_SDCA, 0},
	{ISPDMA_FBRX2_DCSZ, 0},
	{ISPDMA_FBRX2_CTRL, 0},
	{ISPDMA_FBRX2_TMR, 0},
	{ISPDMA_FBRX2_RAMCTRL, 0},
	{ISPDMA_FBRX2_STAT, 0},
	{ISPDMA_FBTX3_SDCA, 0},
	{ISPDMA_FBTX3_DCSZ, 0},
	{ISPDMA_FBTX3_CTRL, 0},
	{ISPDMA_FBTX3_DSTSZ, 0},
	{ISPDMA_FBTX3_DSTADDR, 0},
	{ISPDMA_FBTX3_TMR, 0},
	{ISPDMA_FBTX3_RAMCTRL, 0},
	{ISPDMA_FBRX3_SDCA, 0},
	{ISPDMA_FBRX3_DCSZ, 0},
	{ISPDMA_FBRX3_CTRL, 0},
	{ISPDMA_FBRX3_TMR, 0},
	{ISPDMA_FBRX3_RAMCTRL, 0},
	{ISPDMA_FBRX3_STAT, 0},
	{ISPDMA_DISP_CTRL, 0},
	{ISPDMA_DISP_DSTSZ, 0},
	{ISPDMA_DISP_DSTADDR, 0},
	{ISPDMA_DISP_RAMCTRL, 0},
	{ISPDMA_DISP_PITCH, 0},
	{ISPDMA_CODEC_CTRL, 0},
	{ISPDMA_CODEC_DSTSZ, 0},
	{ISPDMA_CODEC_DSTADDR, 0},
	{ISPDMA_CODEC_RAMCTRL, 0},
	{ISPDMA_CODEC_STAT, 0},
	{ISPDMA_CODEC_PITCH, 0},
	{ISPDMA_CODEC_VBSZ, 0},
	{ISPDMA_INPSDMA_SRCADDR, 0},
	{ISPDMA_INPSDMA_SRCSZ, 0},
	{ISPDMA_INPSDMA_PIXSZ, 0},
	{ISP_IRQMASK, 0},
};

inline unsigned long get_dma_working_flag(struct isp_ispdma_device *ispdma)
{
	unsigned long dma_flags;
	unsigned long dma_working_flag;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	dma_working_flag = ispdma->dma_working_flag;
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return dma_working_flag;
}

static void ispdma_reg_dump(struct isp_ispdma_device *ispdma)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	int cnt;

	for (cnt = 0; cnt < ISPDMA_INPSDMA_MAX_CTX; cnt++) {
		ispdma_reg_list[cnt].val =
			mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);

		dev_warn(isp->dev, "REG[0x%08X]--->0x%08X\n",
			ispdma_reg_list[cnt].reg,
			ispdma_reg_list[cnt].val);
	}

	return;
}


static int ispdma_wait_ipc(struct isp_ispdma_device *ispdma,
		struct v4l2_dxoipc_ipcwait *ipc_wait)
{
	int ret = 0;
	unsigned long flags;
	long rc = 0;
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	rc = wait_for_completion_timeout(&ispdma->ipc_event,
		(ipc_wait->timeout + MS_PER_JIFFIES - 1) / MS_PER_JIFFIES);
	spin_lock_irqsave(&ispdma->ipc_irq_lock, flags);
	if (rc == 0) {
		dev_warn(isp->dev, "timeout mipi %d - %d, eof %d, ipc %d, dma %d\n",
			ispdma->disp_mipi_ovr_cnt,
			ispdma->codec_mipi_ovr_cnt,
			ispdma->disp_eof_cnt,
			ispdma->ipc_event_cnt,
			ispdma->dma_event_cnt);
		ret = -ETIMEDOUT;
	}

	INIT_COMPLETION(ispdma->ipc_event);
	ipc_wait->tick = 0;
	spin_unlock_irqrestore(&ispdma->ipc_irq_lock, flags);

	return ret;
}

void mv_ispdma_ipc_isr_handler(struct isp_ispdma_device *ispdma)
{
	unsigned long flags;

	spin_lock_irqsave(&ispdma->ipc_irq_lock, flags);
	ispdma->ipc_event_cnt++;
	complete_all(&ispdma->ipc_event);
	spin_unlock_irqrestore(&ispdma->ipc_irq_lock, flags);
}

static int ispdma_stream_cfg(struct isp_ispdma_device *ispdma,
		struct v4l2_dxoipc_streaming_config *stream_cfg)
{
	u32 reg_clk_en, reg_dma_en, reg_dma_en_old, reg_irq_mask;
	int fb_ch;
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	mutex_lock(&ispdma->ispdma_mutex);

	reg_dma_en = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
	reg_clk_en = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
	reg_irq_mask = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

	reg_dma_en_old = reg_dma_en;
	for (fb_ch = FBTX0_DMAENA; fb_ch < FBDMAENA_MAX; fb_ch++) {
		if (IS_CH_ENABLED(reg_dma_en, fb_ch)) {
			if (IS_TX_CH(fb_ch))
				if (stream_cfg->enable_fbtx == 0) {
					CLEAR_DMA_EN(reg_dma_en, fb_ch);
					CLEAR_CLK_EN(reg_clk_en, fb_ch);
					CLEAR_TX_IRQ_MASK(reg_irq_mask, fb_ch);
				}
			if (IS_RX_CH(fb_ch))
				if (stream_cfg->enable_fbrx == 0) {
					CLEAR_DMA_EN(reg_dma_en, fb_ch);
					CLEAR_CLK_EN(reg_clk_en, fb_ch);
					CLEAR_RX_IRQ_MASK(reg_irq_mask, fb_ch);
				}
		} else {
			if (IS_TX_CH(fb_ch))
				if ((stream_cfg->enable_fbtx != 0)
					&& (fb_ch - FBTX0_DMAENA <
						ispdma->framebuf_count)) {
					SET_DMA_EN(reg_dma_en, fb_ch);
					SET_CLK_EN(reg_clk_en, fb_ch);
					SET_TX_IRQ_MASK(reg_irq_mask, fb_ch);
				}
			if (IS_RX_CH(fb_ch))
				if ((stream_cfg->enable_fbrx != 0)
					&& (fb_ch - FBRX0_DMAENA <
						ispdma->framebuf_count)) {
					SET_DMA_EN(reg_dma_en, fb_ch);
					SET_CLK_EN(reg_clk_en, fb_ch);
					SET_RX_IRQ_MASK(reg_irq_mask, fb_ch);
				}
		}
	}

	if (reg_dma_en != reg_dma_en_old) {
		mvisp_reg_writel(isp, reg_clk_en,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
		mvisp_reg_writel(isp, reg_irq_mask,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		mvisp_reg_writel(isp, reg_dma_en,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
	}

	mutex_unlock(&ispdma->ispdma_mutex);
	return 0;
}

static int ispdma_set_fb_reg(struct mvisp_device *isp,
	int index, struct v4l2_dxoipc_set_fb *cfg_fb)
{
	u32 regoffset;
	u32 regval;

	regoffset = index * 0x100;

	/* Disable Descriptor */
	regval = mvisp_reg_readl(isp,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_CTRL + regoffset);
	regval &= ~FBTX_DC_ENA;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_CTRL + regoffset);
	regval = mvisp_reg_readl(isp,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_CTRL + regoffset);
	regval &= ~FBRX_DC_ENA;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_CTRL + regoffset);

	/* TX Settings */
	regval = 0;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_SDCA + regoffset);
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_DCSZ + regoffset);
	regval = 0x1 < 9;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_RAMCTRL + regoffset);

	regval = mvisp_reg_readl(isp,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_DSTSZ + regoffset);
	regval |= (ISPDMA_FBTXN_DSTSZ_MASK & cfg_fb->size[index]);
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_DSTSZ + regoffset);

	regval = cfg_fb->phyAddr[index];
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_DSTADDR + regoffset);

	regval = FBTX_DMA_TIMER_VAL;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_TMR + regoffset);

	regval = 0x1400 | ((cfg_fb->burst_write >> 7) & FBTX_DMABRSTSZ);
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBTX0_CTRL + regoffset);

	/* RX Settings */
	regval = 0;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_SDCA + regoffset);
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_DCSZ + regoffset);
	regval = 0x1 < 9;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_RAMCTRL + regoffset);

	regval = FBRX_DMA_TIMER_VAL;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_TMR + regoffset);

	regval = (cfg_fb->burst_read >> 3) & FBRX_DMABRSTSZ;
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_FBRX0_CTRL + regoffset);

	return 0;
}


static int ispdma_config_fb(struct isp_ispdma_device *ispdma,
			struct v4l2_dxoipc_set_fb *cfg_fb)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	int cnt;
	u32 regval;

	if (cfg_fb->burst_read != 64
		&& cfg_fb->burst_read != 128
		&& cfg_fb->burst_read != 256)
		return -EINVAL;

	if (cfg_fb->burst_write != 64
		&& cfg_fb->burst_write != 128
		&& cfg_fb->burst_write != 256)
		return -EINVAL;

	if (cfg_fb->fbcnt < 0 || cfg_fb->fbcnt > 4)
		return -EINVAL;

	for (cnt = 0; cnt < cfg_fb->fbcnt; cnt++) {
		if (cfg_fb->phyAddr[cnt] == 0 || cfg_fb->size[cnt] <= 0)
			return -EINVAL;
	}

	mutex_lock(&ispdma->ispdma_mutex);

	/* Disable all FB DMA transfer */
	regval = mvisp_reg_readl(isp,
		ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
	mvisp_reg_writel(isp, (regval & ~0x3FC),
		ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

	for (cnt = 0; cnt < cfg_fb->fbcnt; cnt++)
		ispdma_set_fb_reg(isp, cnt, cfg_fb);

	ispdma->framebuf_count = cfg_fb->fbcnt;

	/* Restart FB DMA transfer if any */
	mvisp_reg_writel(isp, regval,
		ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

	mutex_unlock(&ispdma->ispdma_mutex);
	return 0;
}

static void ispdma_set_inaddr(struct isp_ispdma_device *ispdma,
		struct isp_video_buffer *buffer)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	unsigned int bytesperpixel, width, height, size;
	unsigned int regval;

	if (buffer == NULL)
		return;

	width = ispdma->formats[ISPDMA_PAD_SINK].width;
	height = ispdma->formats[ISPDMA_PAD_SINK].height;

	switch (ispdma->formats[ISPDMA_PAD_SINK].code) {
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		bytesperpixel = 8;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		bytesperpixel = 16;
		break;
	case V4L2_MBUS_FMT_Y12_1X12:
		bytesperpixel = 12;
		regval |= 0x4;
		break;
	default:
		bytesperpixel = 0;
		break;
	}

	size = (bytesperpixel * width * height) >> 3;
	if (buffer->paddr != 0) {
		mvisp_reg_writel(isp, buffer->paddr,
			ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_SRCADDR);
		mvisp_reg_writel(isp, size,
			ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_SRCSZ);
	}

	return;
}

static void ispdma_set_disp_outaddr(struct isp_ispdma_device *ispdma,
		struct isp_video_buffer *buffer)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	unsigned int bytesperpixel, width, height, size;

	if (buffer == NULL)
		return;

	width = ispdma->formats[ISPDMA_PAD_DISPLAY_SOURCE].width;
	height = ispdma->formats[ISPDMA_PAD_DISPLAY_SOURCE].height;

	switch (ispdma->formats[ISPDMA_PAD_DISPLAY_SOURCE].code) {
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		bytesperpixel = 8;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		bytesperpixel = 16;
		break;
	case V4L2_MBUS_FMT_Y12_1X12:
		bytesperpixel = 12;
		break;
	default:
		bytesperpixel = 0;
		break;
	}

	size = (bytesperpixel * width * height) >> 3;
	if (buffer->paddr != 0) {
		mvisp_reg_writel(isp, size ,
			ISP_IOMEM_ISPDMA, ISPDMA_DISP_DSTSZ);
		mvisp_reg_writel(isp, buffer->paddr,
			ISP_IOMEM_ISPDMA, ISPDMA_DISP_DSTADDR);
	}

	return;
}

static void ispdma_set_codec_outaddr(struct isp_ispdma_device *ispdma,
		struct isp_video_buffer *buffer)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	unsigned int bytesperpixel, width, height, size;

	if (buffer == NULL)
		return;

	width = ispdma->formats[ISPDMA_PAD_CODEC_SOURCE].width;
	height = ispdma->formats[ISPDMA_PAD_CODEC_SOURCE].height;

	switch (ispdma->formats[ISPDMA_PAD_CODEC_SOURCE].code) {
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		bytesperpixel = 8;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		bytesperpixel = 16;
		break;
	case V4L2_MBUS_FMT_Y12_1X12:
		bytesperpixel = 12;
		break;
	default:
		bytesperpixel = 0;
		break;
	}

	size = (bytesperpixel * width * height) >> 3;
	if (buffer->paddr != 0) {
		mvisp_reg_writel(isp, size,
			ISP_IOMEM_ISPDMA, ISPDMA_CODEC_DSTSZ);
		mvisp_reg_writel(isp, buffer->paddr,
			ISP_IOMEM_ISPDMA, ISPDMA_CODEC_DSTADDR);
	}

	return;
}

static void ispdma_reset_counter(struct isp_ispdma_device *ispdma)
{
	ispdma->ipc_event_cnt = 0;
	ispdma->dma_event_cnt = 0;
	ispdma->disp_mipi_ovr_cnt = 0;
	ispdma->codec_mipi_ovr_cnt = 0;
	ispdma->disp_eof_cnt = 0;
	ispdma->input_event_cnt = 0;
	return;
}

static void ispdma_init_params(struct isp_ispdma_device *ispdma)
{
	ispdma_reset_counter(ispdma);
	ispdma->dma_working_flag = DMA_NOT_WORKING;
	ispdma->framebuf_count = 0;
	return;
}


static void ispdma_start_dma(struct isp_ispdma_device *ispdma
	, enum ispdma_port port)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	u32 regval;
	unsigned long dma_flags;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);

	switch (port) {
	case ISPDMA_PORT_DISPLAY:
	{
		if (ispdma->dma_working_flag & DMA_DISP_WORKING)
			break;

		/* Enable Output DMA here. */
		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_DISP_CTRL);
		regval &= ~0x2;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_DISP_CTRL);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
		regval |= 0x300;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval |= 0x20801;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		regval = (0x1 << 9);
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_DISP_RAMCTRL);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
		regval |= 0x1;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

		ispdma->dma_working_flag |= DMA_DISP_WORKING;
		set_vd_dmaqueue_flg(
			&ispdma->vd_disp_out, ISP_VIDEO_DMAQUEUE_BUSY);
		break;
	}
	case ISPDMA_PORT_CODEC:
	{
		if (ispdma->dma_working_flag & DMA_CODEC_WORKING)
			break;

		/* Enable Output DMA here. */
		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_CODEC_CTRL);
		regval &= ~0x2;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CODEC_CTRL);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
		regval |= 0xC00;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval |= 0x21002;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		regval = (0x1 << 9);
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CODEC_RAMCTRL);

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
		regval |= 0x2;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

		ispdma->dma_working_flag |= DMA_CODEC_WORKING;
		set_vd_dmaqueue_flg(
			&ispdma->vd_codec_out, ISP_VIDEO_DMAQUEUE_BUSY);
		break;
	}
	case ISPDMA_PORT_INPUT:
	{
		if (ispdma->dma_working_flag & DMA_INPUT_WORKING)
			break;

		if (ispdma->input == ISPDMA_INPUT_MEMORY) {
			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);
			regval |= 0x10;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);

			/* Enable Input DMA here. */
			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
			regval |= 0x3000;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
			regval |= 0x20400;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

			/* Select input as input DMA*/
			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_MAINCTRL);
			regval |= 0x4;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_MAINCTRL);

			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);
			regval |= 0x1;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);

			ispdma->dma_working_flag |= DMA_INPUT_WORKING;
			set_vd_dmaqueue_flg(
				&ispdma->vd_in, ISP_VIDEO_DMAQUEUE_BUSY);
		}
		break;
	}
	default:
		break;
	}

	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return;
}


static void ispdma_stop_dma(struct isp_ispdma_device *ispdma
		, enum ispdma_port port)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	u32 regval;
	unsigned long dma_flags;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);

	switch (port) {
	case ISPDMA_PORT_DISPLAY:
	{
		if ((ispdma->dma_working_flag & DMA_DISP_WORKING) == 0)
			break;

		/* Disable Output DMA here. */
		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
		regval &= ~0x1;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
		regval &= ~0x300;
		regval |= 0x200;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval &= ~0x801;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		ispdma->dma_working_flag &= ~DMA_DISP_WORKING;
		break;
	}
	case ISPDMA_PORT_CODEC:
	{
		if ((ispdma->dma_working_flag & DMA_CODEC_WORKING) == 0)
			break;

		/* Disable Output DMA here. */
		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
		regval &= ~0x2;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
		regval &= ~0xC00;
		regval |= 0x800;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval &= ~0x1002;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		ispdma->dma_working_flag &= ~DMA_CODEC_WORKING;
		break;
	}
	case ISPDMA_PORT_INPUT:
	{
		if ((ispdma->dma_working_flag & DMA_INPUT_WORKING) == 0)
			break;

		if (ispdma->input == ISPDMA_INPUT_MEMORY) {
			/* Disable Input DMA here. */
			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);
			regval &= ~0x1;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);

			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);
			regval &= ~0x3000;
			regval |= 0x2000;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CLKENA);

			regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
			regval &= ~0x400;
			mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		}

		ispdma->dma_working_flag &= ~DMA_INPUT_WORKING;
		break;
	}
	default:
		break;
	}

	if (ispdma->dma_working_flag == DMA_NOT_WORKING) {
		regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval &= ~0x20000;
		mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
	}

	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return;
}

static void ispdma_config_csi_input(struct isp_ispdma_device *ispdma)
{
	u32 regval;
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_MAINCTRL);
	regval |= 1 << 3;  /* Set input as CSI2 */
	mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_MAINCTRL);
	return;
}

static int load_dummy_buffer(struct isp_ispdma_device *ispdma,
			enum ispdma_port port)
{
	enum isp_pipeline_start_condition
			start_dma = ISP_CAN_NOT_START;
	struct isp_video_buffer *buffer;
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	switch (port) {
	case ISPDMA_PORT_DISPLAY:
		buffer =
			ispdma->vd_disp_out.queue->dummy_buffers[0];
		if (buffer == NULL) {
			ispdma_stop_dma(ispdma, ISPDMA_PORT_DISPLAY);
			dev_warn(isp->dev,
				"isp display dma stops [no dummy buffer]\n");
		} else {
			ispdma_set_disp_outaddr(ispdma, buffer);
			start_dma |= ISP_DISPLAY_CAN_START;
		}
		break;
	case ISPDMA_PORT_CODEC:
		buffer =
			ispdma->vd_codec_out.queue->dummy_buffers[0];
		if (buffer == NULL) {
			ispdma_stop_dma(ispdma, ISPDMA_PORT_CODEC);
			dev_warn(isp->dev,
				"isp codec dma stops [no dummy buffer]\n");
		} else {
			ispdma_set_codec_outaddr(ispdma, buffer);
			start_dma |= ISP_CODEC_CAN_START;
		}
		break;
	case ISPDMA_PORT_INPUT:
		ispdma_stop_dma(ispdma, ISPDMA_PORT_INPUT);
		break;
	default:
		break;
	}

	return start_dma;
}

static int ispdma_isr_load_buffer(struct isp_ispdma_device *ispdma
			, enum ispdma_port port)
{
	enum isp_pipeline_start_condition
			start_dma = ISP_CAN_NOT_START;
	struct isp_video_buffer *buffer;
	bool needdummy = false;

	switch (port) {
	case ISPDMA_PORT_DISPLAY:
		buffer = mvisp_video_buffer_next(
			&ispdma->vd_disp_out, ispdma->disp_mipi_ovr_cnt);
		if (buffer != NULL) {
			ispdma_set_disp_outaddr(ispdma, buffer);
			start_dma |= ISP_DISPLAY_CAN_START;
		} else {
			needdummy = true;
		}
		break;
	case ISPDMA_PORT_CODEC:
		buffer = mvisp_video_buffer_next(
			&ispdma->vd_codec_out, ispdma->codec_mipi_ovr_cnt);
		if (buffer != NULL) {
			ispdma_set_codec_outaddr(ispdma, buffer);
			start_dma |= ISP_CODEC_CAN_START;
		} else {
			needdummy = true;
		}
		break;
	case ISPDMA_PORT_INPUT:
		buffer = mvisp_video_buffer_next(&ispdma->vd_in, 0);
		if (buffer != NULL) {
			ispdma_set_inaddr(ispdma, buffer);
			start_dma |= ISP_INPUT_CAN_START;
		} else {
			needdummy = true;
		}
		break;
	default:
		break;
	}

	if (needdummy == true)
		start_dma |= load_dummy_buffer(ispdma, port);

	return start_dma;
}

static void ispdma_isr_buffer(struct isp_ispdma_device *ispdma
			, enum ispdma_port port)
{
	enum isp_pipeline_start_condition condition;
	unsigned long dma_working_flag;

	condition = ispdma_isr_load_buffer(ispdma, port);
	switch (ispdma->state) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		dma_working_flag = get_dma_working_flag(ispdma);

		if ((dma_working_flag & DMA_INPUT_WORKING) &&
			(condition & ISP_INPUT_CAN_START) == 0 &&
			port == ISPDMA_PORT_INPUT) {
			ispdma_stop_dma(ispdma, ISPDMA_PORT_INPUT);
		}
		if ((dma_working_flag & DMA_DISP_WORKING) &&
			(condition & ISP_DISPLAY_CAN_START) == 0 &&
			port == ISPDMA_PORT_DISPLAY)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_DISPLAY);

		if ((dma_working_flag & DMA_CODEC_WORKING) &&
			(condition & ISP_CODEC_CAN_START) == 0 &&
			port == ISPDMA_PORT_CODEC)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_CODEC);

		break;
	case ISP_PIPELINE_STREAM_STOPPED:
		dma_working_flag = get_dma_working_flag(ispdma);

		if (dma_working_flag & DMA_INPUT_WORKING)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_INPUT);
		if (dma_working_flag & DMA_DISP_WORKING)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_DISPLAY);
		if (dma_working_flag & DMA_CODEC_WORKING)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_CODEC);
		break;
	default:
		break;
	}

	return;
}

void mv_ispdma_dma_isr_handler(struct isp_ispdma_device *ispdma
		, unsigned long irq_status)
{
	unsigned long flags, dma_irq_flags;
	u32 regval;
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	struct isp_video *video;
	struct isp_video_queue *queue;
	struct isp_video_buffer *buffer = NULL;
	bool dummy_in_transfer;
	unsigned long dma_working_flag;

	spin_lock_irqsave(&ispdma->dma_irq_lock, dma_irq_flags);

	ispdma->dma_event_cnt++;
	if (irq_status & CSI2_BRIDEG) {
		if (ispdma->disp_mipi_ovr_cnt < 0xFFFF)
			ispdma->disp_mipi_ovr_cnt++;
		if (ispdma->codec_mipi_ovr_cnt < 0xFFFF)
			ispdma->codec_mipi_ovr_cnt++;

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval &= ~0x20000;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
	}

	if (irq_status & DISP_DMA_EOF) {
		dma_working_flag = get_dma_working_flag(ispdma);
		if ((dma_working_flag & DMA_DISP_WORKING) == 0)
			return;

		video = &ispdma->vd_disp_out;
		queue = video->queue;
		spin_lock_irqsave(&queue->irqlock, flags);
		ispdma->disp_eof_cnt++;
		if (queue->dummy_buffers[0] != NULL) {
			regval = mvisp_reg_readl(isp,
					ISP_IOMEM_ISPDMA, ISPDMA_DISP_DSTADDR);
			if (regval != video->queue->dummy_buffers[0]->paddr)
				dummy_in_transfer = false;
			else {
				dummy_in_transfer = true;
				buffer = queue->dummy_buffers[0];
			}
		} else
			dummy_in_transfer = false;

		if (dummy_in_transfer == false) {
			ispdma_isr_buffer(ispdma, ISPDMA_PORT_DISPLAY);
			ispdma->disp_mipi_ovr_cnt = 0;
		} else {
			ispdma->disp_mipi_ovr_cnt = 0;
			if (list_empty(&video->dmaidlequeue)) {
				ispdma_set_disp_outaddr(ispdma, buffer);
				spin_unlock_irqrestore(&queue->irqlock, flags);
				spin_unlock_irqrestore(&ispdma->dma_irq_lock
					, dma_irq_flags);
				return;
			}
			buffer =
				list_first_entry(&video->dmaidlequeue
				, struct isp_video_buffer, irqlist);
			list_del(&buffer->irqlist);
			list_add_tail(&buffer->irqlist, &video->dmabusyqueue);
			buffer->state = ISP_BUF_STATE_ACTIVE;
			ispdma_set_disp_outaddr(ispdma, buffer);
		}

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval |= 0x20000;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

	if (irq_status & CODEC_DMA_EOF) {
		dma_working_flag = get_dma_working_flag(ispdma);
		if ((dma_working_flag & DMA_CODEC_WORKING) == 0)
			return;

		if (ispdma->codec_band_max != 0) {
			ispdma->codec_band_cnt++;
			if (ispdma->codec_band_cnt < ispdma->codec_band_max) {
				spin_unlock_irqrestore(
					&ispdma->dma_irq_lock, dma_irq_flags);
				return;
			} else
				ispdma->codec_band_cnt = 0;
		}

		video = &ispdma->vd_codec_out;
		queue = video->queue;
		spin_lock_irqsave(&queue->irqlock, flags);
		if (queue->dummy_buffers[0] != NULL) {
			regval = mvisp_reg_readl(isp,
					ISP_IOMEM_ISPDMA, ISPDMA_CODEC_DSTADDR);
			if (regval != video->queue->dummy_buffers[0]->paddr)
				dummy_in_transfer = false;
			else {
				buffer = queue->dummy_buffers[0];
				dummy_in_transfer = true;
			}
		} else
			dummy_in_transfer = false;

		if (dummy_in_transfer == false) {
			ispdma_isr_buffer(ispdma, ISPDMA_PORT_CODEC);
			ispdma->codec_mipi_ovr_cnt = 0;
		} else {
			ispdma->codec_mipi_ovr_cnt = 0;
			if (list_empty(&video->dmaidlequeue)) {
				ispdma_set_codec_outaddr(ispdma, buffer);
				spin_unlock_irqrestore(&queue->irqlock
					, flags);
				spin_unlock_irqrestore(&ispdma->dma_irq_lock
					, dma_irq_flags);
				return;
			}
			buffer = list_first_entry(&video->dmaidlequeue
				, struct isp_video_buffer, irqlist);
			list_del(&buffer->irqlist);
			list_add_tail(&buffer->irqlist, &video->dmabusyqueue);
			buffer->state = ISP_BUF_STATE_ACTIVE;
			ispdma_set_codec_outaddr(ispdma, buffer);
		}

		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		regval |= 0x20000;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);

		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

	if (irq_status & INPUT_DMA_EOF) {
		dma_working_flag = get_dma_working_flag(ispdma);
		if ((dma_working_flag & DMA_INPUT_WORKING) == 0)
			return;

		video = &ispdma->vd_in;
		queue = video->queue;
		spin_lock_irqsave(&queue->irqlock, flags);
		ispdma_isr_buffer(ispdma, ISPDMA_PORT_INPUT);
		spin_unlock_irqrestore(&queue->irqlock, flags);
	}

	spin_unlock_irqrestore(&ispdma->dma_irq_lock,
		dma_irq_flags);
	return;
}

static int ispdma_reload_disp_buffer(
	struct isp_ispdma_device *ispdma, int delay)
{
	struct isp_video_buffer *buffer = NULL;
	unsigned long dma_flags;

	buffer = mvisp_video_get_next_work_buf(&ispdma->vd_disp_out, delay);
	if (buffer == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	if (buffer->vbuf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ispdma_set_disp_outaddr(ispdma, buffer);
		set_vd_dmaqueue_flg(&ispdma->vd_disp_out,
			ISP_VIDEO_DMAQUEUE_QUEUED);
	}
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return 0;
}

static int ispdma_reload_codec_buffer(
	struct isp_ispdma_device *ispdma, int delay)
{
	struct isp_video_buffer *buffer = NULL;
	unsigned long dma_flags;

	buffer = mvisp_video_get_next_work_buf(&ispdma->vd_codec_out, delay);
	if (buffer == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	if (buffer->vbuf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ispdma_set_codec_outaddr(ispdma, buffer);
		set_vd_dmaqueue_flg(&ispdma->vd_codec_out,
			ISP_VIDEO_DMAQUEUE_QUEUED);
	}
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return 0;
}

static int ispdma_reload_input_buffer(
	struct isp_ispdma_device *ispdma, int delay)
{
	struct isp_video_buffer *buffer = NULL;
	unsigned long dma_flags;

	buffer = mvisp_video_get_next_work_buf(&ispdma->vd_in, delay);
	if (buffer == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	if (buffer->vbuf.type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		ispdma_set_inaddr(ispdma, buffer);
		set_vd_dmaqueue_flg(&ispdma->vd_in,
			ISP_VIDEO_DMAQUEUE_QUEUED);
	}
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	return 0;
}

static int ispdma_try_restart_disp_dma(struct isp_ispdma_device *ispdma)
{
	enum isp_video_dmaqueue_flags dmaqueue_flags;
	unsigned long dma_flag;
	int ret;

	dmaqueue_flags = get_vd_dmaqueue_flg(&ispdma->vd_disp_out);
	switch (dmaqueue_flags) {
	case ISP_VIDEO_DMAQUEUE_UNDERRUN:
		ret = ispdma_reload_disp_buffer(ispdma, 0);
		if (ret != 0)
			return ret;
	case ISP_VIDEO_DMAQUEUE_QUEUED:
		dma_flag = get_dma_working_flag(ispdma);
		if ((dma_flag & DMA_DISP_WORKING) == 0) {
			ispdma_start_dma(ispdma, ISPDMA_PORT_DISPLAY);
			ispdma_reload_disp_buffer(ispdma, 1);
		}
		break;
	case ISP_VIDEO_DMAQUEUE_BUSY:
	default:
		break;
	}

	return 0;
}

static int ispdma_try_restart_codec_dma(struct isp_ispdma_device *ispdma)
{
	enum isp_video_dmaqueue_flags dmaqueue_flags;
	unsigned long dma_flag;
	int ret;

	dmaqueue_flags =
		get_vd_dmaqueue_flg(&ispdma->vd_codec_out);
	switch (dmaqueue_flags) {
	case ISP_VIDEO_DMAQUEUE_UNDERRUN:
		ret = ispdma_reload_codec_buffer(ispdma, 0);
		if (ret != 0)
			return ret;
	case ISP_VIDEO_DMAQUEUE_QUEUED:
		dma_flag = get_dma_working_flag(ispdma);
		if ((dma_flag & DMA_CODEC_WORKING) == 0) {
			ispdma_start_dma(ispdma, ISPDMA_PORT_CODEC);
			ispdma_reload_codec_buffer(ispdma, 1);
		}
		break;
	case ISP_VIDEO_DMAQUEUE_BUSY:
	default:
		break;
	}

	return 0;
}

static int ispdma_try_restart_input_dma(struct isp_ispdma_device *ispdma)
{
	enum isp_video_dmaqueue_flags dmaqueue_flags;
	unsigned long dma_flag;
	int ret;

	dmaqueue_flags =
		get_vd_dmaqueue_flg(&ispdma->vd_in);
	switch (dmaqueue_flags) {
	case ISP_VIDEO_DMAQUEUE_UNDERRUN:
		ret = ispdma_reload_input_buffer(ispdma, 0);
		if (ret != 0)
			return ret;
	case ISP_VIDEO_DMAQUEUE_QUEUED:
		dma_flag = get_dma_working_flag(ispdma);
		if ((dma_flag & DMA_INPUT_WORKING) == 0) {
			ispdma_start_dma(ispdma, ISPDMA_PORT_INPUT);
			ispdma_reload_input_buffer(ispdma, 1);
		}
		break;
	case ISP_VIDEO_DMAQUEUE_BUSY:
	default:
		break;
	}

	return 0;
}

static int ispdma_try_restart_dma(struct isp_ispdma_device *ispdma)
{
	struct isp_pipeline *pipe = to_isp_pipeline(&ispdma->subdev.entity);
	enum isp_pipeline_start_condition condition;
	int ret = 0;

	condition = isp_pipeline_ready(pipe);
	if (ispdma->state == ISP_PIPELINE_STREAM_CONTINUOUS) {
		if (condition & ISP_DISPLAY_CAN_START)
			ret = ispdma_try_restart_disp_dma(ispdma);
		if (condition & ISP_CODEC_CAN_START)
			ret = ispdma_try_restart_codec_dma(ispdma);
		if (condition & ISP_INPUT_CAN_START)
			ret = ispdma_try_restart_input_dma(ispdma);
	}

	return ret;
}

static void ctx_adjust_buffers(struct isp_video *video)
{
	struct isp_video_buffer *buf;
	struct isp_pipeline *pipe = to_isp_pipeline(&video->video.entity);
	enum isp_pipeline_state state;
	unsigned long flags;

	struct list_head temp_list;
	INIT_LIST_HEAD(&temp_list);

	while (!list_empty(&video->dmaidlequeue)) {
		buf = list_first_entry(&video->dmaidlequeue,
			struct isp_video_buffer, irqlist);
		list_del(&buf->irqlist);
		list_add_tail(&buf->irqlist, &temp_list);
	}

	while (!list_empty(&video->dmabusyqueue)) {
		buf = list_first_entry(&video->dmabusyqueue,
			struct isp_video_buffer, irqlist);
		list_del(&buf->irqlist);
		list_add_tail(&buf->irqlist, &video->dmaidlequeue);
	}

	while (!list_empty(&temp_list)) {
		buf = list_first_entry(&temp_list,
			struct isp_video_buffer, irqlist);
		list_del(&buf->irqlist);
		list_add_tail(&buf->irqlist, &video->dmaidlequeue);
	}

	if (list_empty(&video->dmaidlequeue) == 0) {
		switch (video->video_type) {
		case ISP_VIDEO_DISPLAY:
			state = ISP_PIPELINE_DISPLAY_QUEUED;
			break;
		case ISP_VIDEO_CODEC:
			state = ISP_PIPELINE_CODEC_QUEUED;
			break;
		case ISP_VIDEO_INPUT:
			state = ISP_PIPELINE_INPUT_QUEUED;
			break;
		default:
			state = 0;
			break;
		}

		spin_lock_irqsave(&pipe->lock, flags);
		pipe->state |= state;
		spin_unlock_irqrestore(&pipe->lock, flags);
	}


	set_vd_dmaqueue_flg(video, ISP_VIDEO_DMAQUEUE_UNDERRUN);

	return;
}

static void ispdma_context_save(struct isp_ispdma_device *ispdma)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	int cnt;

	for (cnt = 0; cnt < ISPDMA_INPSDMA_MAX_CTX; cnt++) {
		ispdma_reg_list[cnt].val =
			mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);

		if (cnt == ISPDMA_DMA_ENA_CTX) {
			mvisp_reg_writel(isp, 0,
				ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

			ctx_adjust_buffers(&ispdma->vd_disp_out);
			ctx_adjust_buffers(&ispdma->vd_codec_out);
		} else if (cnt == ISPDMA_INPSDMA_CTRL_CTX) {
			mvisp_reg_writel(isp, 0,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);

			ctx_adjust_buffers(&ispdma->vd_in);
		} else if (cnt == ISPDMA_IRQMASK_CTX) {
			mvisp_reg_writel(isp, 0,
				ISP_IOMEM_ISPDMA, ISPDMA_IRQMASK);
		}
	}

	return;
}

static void ispdma_context_restore(struct isp_ispdma_device *ispdma)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	int cnt;
	unsigned long dma_flags;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	ispdma->dma_working_flag = 0;
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	for (cnt = 0; cnt < ISPDMA_INPSDMA_MAX_CTX; cnt++) {
		if (cnt == ISPDMA_INPSDMA_CTRL_CTX) {
			/*Restore input settings*/
			mvisp_reg_writel(isp,
				(ispdma_reg_list[cnt].val & ~0x1),
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);

			/*Restart input DMA, IRQ mask*/
			ispdma_try_restart_dma(ispdma);
		} else if (cnt == ISPDMA_DMA_ENA_CTX) {
			/*Restart all the DMAs except disp/codec*/
			mvisp_reg_writel(isp,
				(ispdma_reg_list[cnt].val & ~0x3),
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);

			/*Restart all disp/codec DMAs, IRQ masks*/
			ispdma_try_restart_dma(ispdma);
		} else if (cnt == ISPDMA_IRQMASK_CTX) {
			/*Only restore FB IRQ masks here*/
			mvisp_reg_writel(isp,
				(ispdma_reg_list[cnt].val & ~0x21C03),
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);
		} else {
			mvisp_reg_writel(isp,
				ispdma_reg_list[cnt].val,
				ISP_IOMEM_ISPDMA,
				ispdma_reg_list[cnt].reg);
		}
	}

	return;
}



static int ispdma_video_qbuf_notify(struct isp_video *video)
{
	struct isp_ispdma_device *ispdma = &video->isp->mvisp_ispdma;
	int need_try_restart = 0;
	unsigned long dma_flags;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	switch (video->video_type) {
	case ISP_VIDEO_DISPLAY:
		if ((ispdma->dma_working_flag & DMA_DISP_WORKING) == 0)
			need_try_restart = 1;
		break;
	case ISP_VIDEO_CODEC:
		if ((ispdma->dma_working_flag & DMA_CODEC_WORKING) == 0)
			need_try_restart = 1;
		break;
	case ISP_VIDEO_INPUT:
		if ((ispdma->dma_working_flag & DMA_INPUT_WORKING) == 0)
			need_try_restart = 1;
		break;
	default:
		need_try_restart = 0;
		break;
	}
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	if (need_try_restart)
		ispdma_try_restart_dma(ispdma);

	return 0;
}

static int ispdma_video_stream_on_notify(struct isp_video *video)
{
	struct isp_ispdma_device *ispdma = &video->isp->mvisp_ispdma;
	int need_try_restart = 0;
	unsigned long dma_flags;

	spin_lock_irqsave(&ispdma->dmaflg_lock, dma_flags);
	switch (video->video_type) {
	case ISP_VIDEO_DISPLAY:
		if ((ispdma->dma_working_flag & DMA_DISP_WORKING) == 0)
			need_try_restart = 1;
		break;
	case ISP_VIDEO_CODEC:
		if ((ispdma->dma_working_flag & DMA_CODEC_WORKING) == 0)
			need_try_restart = 1;
		break;
	case ISP_VIDEO_INPUT:
		if ((ispdma->dma_working_flag & DMA_INPUT_WORKING) == 0)
			need_try_restart = 1;
		break;
	default:
		need_try_restart = 0;
		break;
	}
	spin_unlock_irqrestore(&ispdma->dmaflg_lock, dma_flags);

	if (need_try_restart)
		ispdma_try_restart_dma(ispdma);

	return 0;
}

static int ispdma_video_stream_off_notify(struct isp_video *video)
{
	struct isp_ispdma_device *ispdma = &video->isp->mvisp_ispdma;
	unsigned long dma_working_flag;

	dma_working_flag = get_dma_working_flag(ispdma);
	switch (video->video_type) {
	case ISP_VIDEO_DISPLAY:
		if (dma_working_flag & DMA_DISP_WORKING)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_DISPLAY);
		break;
	case ISP_VIDEO_CODEC:
		if (dma_working_flag & DMA_CODEC_WORKING)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_CODEC);
		break;
	case ISP_VIDEO_INPUT:
		if (dma_working_flag & ISP_VIDEO_INPUT)
			ispdma_stop_dma(ispdma, ISPDMA_PORT_INPUT);
		break;
	default:
		break;
	}

	return 0;
}

static const struct isp_video_operations ispdma_video_ops = {
	.qbuf_notify = ispdma_video_qbuf_notify,
	.stream_on_notify = ispdma_video_stream_on_notify,
	.stream_off_notify = ispdma_video_stream_off_notify,
};

static int ispdma_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isp_ispdma_device *ispdma =
		container_of(ctrl->handler
			, struct isp_ispdma_device, ctrls);

	mutex_lock(&ispdma->ispdma_mutex);

	ispdma = ispdma;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		break;
	case V4L2_CID_CONTRAST:
		break;
	}

	mutex_unlock(&ispdma->ispdma_mutex);
	return 0;
}

static const struct v4l2_ctrl_ops ispdma_ctrl_ops = {
	.s_ctrl = ispdma_s_ctrl,
};

static int ispdma_mmap(struct v4l2_subdev *sd, struct vm_area_struct *vma)
{
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	int ret = 0;

	mutex_lock(&ispdma->ispdma_mutex);

	vma->vm_pgoff =
		isp->mmio_base_phys[ISP_IOMEM_IPC] >> PAGE_SHIFT;
	vma->vm_page_prot =
		pgprot_noncached(vma->vm_page_prot);

	ret = remap_pfn_range(vma, vma->vm_start,
		vma->vm_pgoff,
		(vma->vm_end - vma->vm_start),
		vma->vm_page_prot);

	mutex_unlock(&ispdma->ispdma_mutex);
	return ret;
}

static int ispdma_config_capture_mode(
	struct isp_ispdma_device *ispdma,
	struct v4l2_ispdma_capture_mode *mode_cfg)
{
	unsigned long mode_flags;

	if (mode_cfg->mode > ISPVIDEO_STILL_CAPTURE)
		return -EINVAL;

	switch (mode_cfg->port) {
	case ISPDMA_PORT_CODEC:
		spin_lock_irqsave(
			&ispdma->vd_codec_out.cap_mode_lock,
			mode_flags);
		ispdma->vd_codec_out.capture_mode = mode_cfg->mode;
		spin_unlock_irqrestore(
			&ispdma->vd_codec_out.cap_mode_lock,
			mode_flags);
		break;
	case ISPDMA_PORT_DISPLAY:
		spin_lock_irqsave(
			&ispdma->vd_disp_out.cap_mode_lock,
			mode_flags);
		ispdma->vd_disp_out.capture_mode = mode_cfg->mode;
		spin_unlock_irqrestore(
			&ispdma->vd_disp_out.cap_mode_lock,
			mode_flags);
		break;
	default:
		break;
	}

	return 0;
}

static int ispdma_config_codec(struct isp_ispdma_device *ispdma,
		struct v4l2_dxoipc_config_codec *cfg_codec)
{
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	u32 regval;
	unsigned long dma_irq_flags;

	if (cfg_codec == NULL)
		return -EINVAL;

	if (cfg_codec->vbsize < 0)
		return -EINVAL;

	mutex_lock(&ispdma->ispdma_mutex);

	regval = mvisp_reg_readl(isp, ISP_IOMEM_ISPDMA, ISPDMA_CODEC_CTRL);
	if ((cfg_codec->vbnum > 1) && (cfg_codec->vbnum < 5)) {
		regval |=  (cfg_codec->vbnum - 1) << 0x6;
		regval |= 0x8;
		spin_lock_irqsave(&ispdma->dma_irq_lock, dma_irq_flags);
		ispdma->codec_band_max = cfg_codec->vbnum;
		ispdma->codec_band_cnt = 0;
		spin_unlock_irqrestore(&ispdma->dma_irq_lock, dma_irq_flags);
		mvisp_reg_writel(isp, cfg_codec->vbsize,
			ISP_IOMEM_ISPDMA, ISPDMA_CODEC_VBSZ);
	} else {
		regval &= ~(0xC8);
		spin_lock_irqsave(&ispdma->dma_irq_lock, dma_irq_flags);
		ispdma->codec_band_max = 0;
		ispdma->codec_band_cnt = 0;
		spin_unlock_irqrestore(&ispdma->dma_irq_lock, dma_irq_flags);
	}

	if (cfg_codec->dma_burst_size == 64
		|| cfg_codec->dma_burst_size == 128
		|| cfg_codec->dma_burst_size == 256) {
		regval |= ((cfg_codec->dma_burst_size >> 3) & 0x30);
	} else
		regval &= ~(0x30);
	mvisp_reg_writel(isp, regval, ISP_IOMEM_ISPDMA, ISPDMA_CODEC_CTRL);

	mutex_unlock(&ispdma->ispdma_mutex);
	return 0;
}

static long ispdma_ioctl(struct v4l2_subdev *sd
			, unsigned int cmd, void *arg)
{
	struct isp_ispdma_device *ispdma;
	int ret;

	ispdma = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_PRIVATE_DXOIPC_SET_FB:
		ret = ispdma_config_fb
			(ispdma, (struct v4l2_dxoipc_set_fb *)arg);
		break;
	case VIDIOC_PRIVATE_DXOIPC_WAIT_IPC:
		ret = ispdma_wait_ipc
			(ispdma,  (struct v4l2_dxoipc_ipcwait *)arg);
		break;
	case VIDIOC_PRIVATE_DXOIPC_SET_STREAM:
		ret = ispdma_stream_cfg
			(ispdma,
			(struct v4l2_dxoipc_streaming_config *)arg);
		break;
	case VIDIOC_PRIVATE_DXOIPC_CONFIG_CODEC:
		ret = ispdma_config_codec(ispdma,
			(struct v4l2_dxoipc_config_codec *) arg);
		break;
	case VIDIOC_PRIVATE_ISPDMA_CAPTURE_MODE:
		ret = ispdma_config_capture_mode(ispdma,
			(struct v4l2_ispdma_capture_mode *) arg);
		break;

	case VIDIOC_PRIVATE_ISPDMA_RESET:
		if (ispdma->mvisp_reset) {
			ispdma_context_save(ispdma);
			ret = ispdma->mvisp_reset(
				(struct v4l2_ispdma_reset *) arg);
			ispdma_context_restore(ispdma);
		} else
			ret = -EINVAL;

		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

static int ispdma_set_stream(struct v4l2_subdev *sd
			, int enable)
{
	struct isp_ispdma_device *ispdma
				= v4l2_get_subdevdata(sd);
	struct mvisp_device *isp
				= to_mvisp_device(ispdma);
	u32 regval;
	unsigned long dma_working_flag;

	mutex_lock(&ispdma->ispdma_mutex);

	ispdma->state = enable;
	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		if ((ispdma->input == ISPDMA_INPUT_CCIC_1)
			&& (isp->sensor_connected == true)) {
			regval = mvisp_reg_readl(isp, ISP_IOMEM_ISPDMA,
				ISPDMA_MAINCTRL);
			regval |= 1 << 24;
			mvisp_reg_writel(isp, regval, ISP_IOMEM_ISPDMA,
				ISPDMA_MAINCTRL);
		}
		break;
	case ISP_PIPELINE_STREAM_STOPPED:
		dma_working_flag = get_dma_working_flag(ispdma);

		if (ispdma->disp_out == ISPDMA_OUTPUT_MEMORY) {
			if (dma_working_flag & DMA_DISP_WORKING)
				ispdma_stop_dma(ispdma, ISPDMA_PORT_DISPLAY);
		}

		if (ispdma->codec_out == ISPDMA_OUTPUT_MEMORY) {
			if (dma_working_flag & DMA_CODEC_WORKING)
				ispdma_stop_dma(ispdma, ISPDMA_PORT_CODEC);
		}

		if (ispdma->input == ISPDMA_INPUT_MEMORY) {
			if (dma_working_flag & DMA_INPUT_WORKING)
				ispdma_stop_dma(ispdma, ISPDMA_PORT_INPUT);
		} else if ((ispdma->input == ISPDMA_INPUT_CCIC_1)
			&& (isp->sensor_connected == true)) {
			regval = mvisp_reg_readl(isp, ISP_IOMEM_ISPDMA,
				ISPDMA_MAINCTRL);
			regval &= ~(1 << 24);
			mvisp_reg_writel(isp, regval, ISP_IOMEM_ISPDMA,
				ISPDMA_MAINCTRL);
		}

		ispdma_reset_counter(ispdma);

		break;
	default:
		break;
	}

	mutex_unlock(&ispdma->ispdma_mutex);
	return 0;
}

static struct v4l2_mbus_framefmt *
__ispdma_get_format(struct isp_ispdma_device *ispdma,
			struct v4l2_subdev_fh *fh,
			unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);
	else
		return &ispdma->formats[pad];
}

/* ispdma format descriptions */
static const unsigned int ispdma_input_fmts[] = {
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_UYVY8_1X16,
	V4L2_MBUS_FMT_Y12_1X12,
};

static const unsigned int ispdma_disp_out_fmts[] = {
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_UYVY8_1X16,
};

static const unsigned int ispdma_codec_out_fmts[] = {
	V4L2_MBUS_FMT_SBGGR8_1X8,
	V4L2_MBUS_FMT_UYVY8_1X16,
};

static void ispdma_try_format(
				struct isp_ispdma_device *ispdma,
				struct v4l2_subdev_fh *fh, unsigned int pad,
				struct v4l2_mbus_framefmt *fmt,
				enum v4l2_subdev_format_whence which)
{
	switch (pad) {
	case ISPDMA_PAD_SINK:
		if (ispdma->input
				== ISPDMA_INPUT_MEMORY) {
			fmt->width =
				min_t(u32, fmt->width, ISPDMA_MAX_IN_WIDTH);
			fmt->height =
				min_t(u32, fmt->height, ISPDMA_MAX_IN_HEIGHT);
		}
		break;
	case ISPDMA_PAD_DISPLAY_SOURCE:
		fmt->width =
				min_t(u32, fmt->width,
					ISPDMA_MAX_DISP_WIDTH);
		fmt->height =
				min_t(u32, fmt->height,
					ISPDMA_MAX_DISP_HEIGHT);
		break;
	case ISPDMA_PAD_CODEC_SOURCE:
		fmt->width =
				min_t(u32, fmt->width,
					ISPDMA_MAX_CODEC_WIDTH);
		fmt->height =
				min_t(u32, fmt->height,
					ISPDMA_MAX_CODEC_HEIGHT);
		break;
	default:
		break;
	}

	if (fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8)
		fmt->colorspace = V4L2_COLORSPACE_SRGB;
	else
		fmt->colorspace = V4L2_COLORSPACE_JPEG;

	fmt->field = V4L2_FIELD_NONE;

	return;
}

static int ispdma_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	int ret = 0;
	struct isp_ispdma_device *ispdma
				= v4l2_get_subdevdata(sd);

	mutex_lock(&ispdma->ispdma_mutex);

	switch (code->pad) {
	case ISPDMA_PAD_SINK:
		if (code->index >=
				ARRAY_SIZE(ispdma_input_fmts))
			ret = -EINVAL;
		else
			code->code =
				ispdma_input_fmts[code->index];
		break;
	case ISPDMA_PAD_DISPLAY_SOURCE:
		if (code->index >=
				ARRAY_SIZE(ispdma_disp_out_fmts))
			ret = -EINVAL;
		else
			code->code =
				ispdma_disp_out_fmts[code->index];
		break;
	case ISPDMA_PAD_CODEC_SOURCE:
		if (code->index >=
				ARRAY_SIZE(ispdma_codec_out_fmts))
			ret = -EINVAL;
		else
			code->code =
				ispdma_codec_out_fmts[code->index];
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&ispdma->ispdma_mutex);
	return ret;
}

static int ispdma_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_fh *fh,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct isp_ispdma_device *ispdma
			= v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;

	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ispdma_try_format(ispdma, fh,
		fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->min_width = format.width;
	fse->min_height = format.height;
	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ispdma_try_format(ispdma, fh,
		fse->pad, &format, V4L2_SUBDEV_FORMAT_TRY);
	fse->max_width = format.width;
	fse->max_height = format.height;
	if (format.code != fse->code)
		return -EINVAL;

	return 0;
}

static int ispdma_get_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	int ret = 0;

	mutex_lock(&ispdma->ispdma_mutex);

	format = __ispdma_get_format(ispdma, fh, fmt->pad, fmt->which);
	if (format == NULL)
		ret = -EINVAL;
	else
		fmt->format = *format;

	mutex_unlock(&ispdma->ispdma_mutex);

	return 0;
}

static int ispdma_config_format(
			struct isp_ispdma_device *ispdma,
			unsigned int pad)
{
	struct v4l2_mbus_framefmt *format = &ispdma->formats[pad];
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	unsigned long width, height, pitch, in_bpp;
	u32 regval;
	int ret = 0;

	width = format->width;
	height = format->height;

	switch (format->code) {
	case V4L2_MBUS_FMT_SBGGR8_1X8:
		in_bpp = 0;
		pitch = width;
		break;
	case V4L2_MBUS_FMT_UYVY8_1X16:
		in_bpp = 0;
		pitch = width * 2;
		break;
	case V4L2_MBUS_FMT_Y12_1X12:
		in_bpp = 2;
		pitch = width * 16 / 12;
		break;
	default:
		in_bpp = 0;
		pitch  = 0;
		ret = -EINVAL;
		break;
	}

	if (ret != 0)
		return ret;

	switch (pad) {
	case ISPDMA_PAD_SINK:
		regval = ((height & ISPDMA_MAX_IN_HEIGHT) << 16)
				| (width & ISPDMA_MAX_IN_WIDTH);
		mvisp_reg_writel(isp, regval, ISP_IOMEM_ISPDMA,
				ISPDMA_INPSDMA_PIXSZ);
		/* Set input BPP */
		regval = mvisp_reg_readl(isp,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);
		regval |= in_bpp << 1;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_INPSDMA_CTRL);

		break;
	case ISPDMA_PAD_CODEC_SOURCE:
		regval = pitch & ISPDMA_MAX_CODEC_PITCH;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_CODEC_PITCH);

		break;
	case ISPDMA_PAD_DISPLAY_SOURCE:
		regval = pitch & ISPDMA_MAX_DISP_PITCH;
		mvisp_reg_writel(isp, regval,
				ISP_IOMEM_ISPDMA, ISPDMA_DISP_PITCH);

		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int ispdma_set_format(
					struct v4l2_subdev *sd,
					struct v4l2_subdev_fh *fh,
					struct v4l2_subdev_format *fmt)
{
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	int ret;

	mutex_lock(&ispdma->ispdma_mutex);

	format = __ispdma_get_format(ispdma, fh,
				fmt->pad, fmt->which);
	if (format == NULL) {
		mutex_unlock(&ispdma->ispdma_mutex);
		return -EINVAL;
	}

	ispdma_try_format(ispdma, fh, fmt->pad,
				&fmt->format, fmt->which);
	*format = fmt->format;

	if (fmt->which != V4L2_SUBDEV_FORMAT_TRY)
		ret = ispdma_config_format(ispdma, fmt->pad);
	else
		ret = 0;

	mutex_unlock(&ispdma->ispdma_mutex);

	return ret;
}

static int ispdma_init_formats(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	struct v4l2_mbus_framefmt *format_active, *format_try;
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (fh == NULL) {
		memset(&format, 0, sizeof(format));
		format.pad = ISPDMA_PAD_SINK;
		format.which =  V4L2_SUBDEV_FORMAT_ACTIVE;
		format.format.code = V4L2_MBUS_FMT_Y12_1X12;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_JPEG;
		format.format.field = V4L2_FIELD_NONE;
		ispdma_set_format(sd, fh, &format);

		format.format.code = V4L2_MBUS_FMT_UYVY8_1X16;
		format.pad = ISPDMA_PAD_CODEC_SOURCE;
		format.format.width = 640;
		format.format.height = 480;
		format.format.colorspace = V4L2_COLORSPACE_JPEG;
		format.format.field = V4L2_FIELD_NONE;
		ispdma_set_format(sd, fh, &format);

		format.pad = ISPDMA_PAD_DISPLAY_SOURCE;
		ret = ispdma_set_format(sd, fh, &format);
	} else {
		/* Copy the active format to a newly opened fh structure */
		mutex_lock(&ispdma->ispdma_mutex);
		format_active =
			__ispdma_get_format(ispdma, fh,
			ISPDMA_PAD_SINK, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try =
			__ispdma_get_format(ispdma, fh,
			ISPDMA_PAD_SINK, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
			sizeof(struct v4l2_subdev_format));

		format_active = __ispdma_get_format(ispdma,
			fh, ISPDMA_PAD_DISPLAY_SOURCE,
			V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ispdma_get_format(ispdma, fh,
			ISPDMA_PAD_DISPLAY_SOURCE, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
			sizeof(struct v4l2_subdev_format));

		format_active = __ispdma_get_format(ispdma, fh,
			ISPDMA_PAD_CODEC_SOURCE, V4L2_SUBDEV_FORMAT_ACTIVE);
		if (format_active == NULL) {
			ret = -EINVAL;
			goto error;
		}
		format_try = __ispdma_get_format(ispdma, fh,
			ISPDMA_PAD_CODEC_SOURCE, V4L2_SUBDEV_FORMAT_TRY);
		if (format_try == NULL) {
			ret = -EINVAL;
			goto error;
		}
		memcpy(format_try, format_active,
				sizeof(struct v4l2_subdev_format));

error:
		mutex_unlock(&ispdma->ispdma_mutex);
	}

	return ret;
}

static int ispdma_open(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	mvisp_get(isp);

	ispdma_init_params(ispdma);
	return ispdma_init_formats(sd, fh);
}

static int ispdma_close(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh)
{
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct mvisp_device *isp = to_mvisp_device(ispdma);
	u32 regval;

	regval = mvisp_reg_readl(isp,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);
	regval &= ~(FBRX3DMAENA | FBRX2DMAENA
				| FBRX1DMAENA | FBRX0DMAENA
				| FBTX3DMAENA | FBTX2DMAENA
				| FBTX1DMAENA | FBTX0DMAENA);
	mvisp_reg_writel(isp, regval,
			ISP_IOMEM_ISPDMA, ISPDMA_DMA_ENA);

	mvisp_put(isp);

	return 0;
}

/* subdev core operations */
static const struct v4l2_subdev_core_ops ispdma_v4l2_core_ops = {
	.ioctl = ispdma_ioctl,
	.mmap = ispdma_mmap,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops ispdma_v4l2_video_ops = {
	.s_stream = ispdma_set_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops ispdma_v4l2_pad_ops = {
	.enum_mbus_code = ispdma_enum_mbus_code,
	.enum_frame_size = ispdma_enum_frame_size,
	.get_fmt = ispdma_get_format,
	.set_fmt = ispdma_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops ispdma_v4l2_ops = {
	.core = &ispdma_v4l2_core_ops,
	.video = &ispdma_v4l2_video_ops,
	.pad = &ispdma_v4l2_pad_ops,
};

/* subdev internal operations */
static const struct v4l2_subdev_internal_ops
ispdma_v4l2_internal_ops = {
	.open = ispdma_open,
	.close = ispdma_close,
};

static int ispdma_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_ispdma_device *ispdma = v4l2_get_subdevdata(sd);
	struct mvisp_device *isp = to_mvisp_device(ispdma);

	switch (local->index | media_entity_type(remote->entity)) {
	case ISPDMA_PAD_SINK | MEDIA_ENT_T_DEVNODE:
		/* read from memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (ispdma->input != ISPDMA_INPUT_NONE)
				return -EBUSY;
			ispdma->input = ISPDMA_INPUT_MEMORY;
		} else {
			if (ispdma->input == ISPDMA_INPUT_MEMORY)
				ispdma->input = ISPDMA_INPUT_NONE;
		}
		break;
	case ISPDMA_PAD_SINK | MEDIA_ENT_T_V4L2_SUBDEV:
		/* read from ccic */
		if (isp->sensor_connected == false)
			return -EINVAL;
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (ispdma->input != ISPDMA_INPUT_NONE)
				return -EBUSY;
			ispdma->input = ISPDMA_INPUT_CCIC_1;
			ispdma_config_csi_input(ispdma);
		} else {
			if (ispdma->input == ISPDMA_INPUT_CCIC_1)
				ispdma->input = ISPDMA_INPUT_NONE;
		}
		break;
	case ISPDMA_PAD_CODEC_SOURCE | MEDIA_ENT_T_DEVNODE:
		/* write to memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (ispdma->codec_out == ISPDMA_OUTPUT_MEMORY)
				return -EBUSY;
			ispdma->codec_out = ISPDMA_OUTPUT_MEMORY;
		} else {
			ispdma->codec_out = ISPDMA_OUTPUT_NONE;
		}
		break;

	case ISPDMA_PAD_DISPLAY_SOURCE | MEDIA_ENT_T_DEVNODE:
		/* write to memory */
		if (flags & MEDIA_LNK_FL_ENABLED) {
			if (ispdma->disp_out == ISPDMA_OUTPUT_MEMORY)
				return -EBUSY;
			ispdma->disp_out = ISPDMA_OUTPUT_MEMORY;
		} else {
			ispdma->disp_out = ISPDMA_OUTPUT_NONE;
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct media_entity_operations ispdma_media_ops = {
	.link_setup = ispdma_link_setup,
};

static int ispdma_init_entities(struct isp_ispdma_device *ispdma)
{
	struct v4l2_subdev *sd = &ispdma->subdev;
	struct media_pad *pads = ispdma->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	spin_lock_init(&ispdma->ipc_irq_lock);
	spin_lock_init(&ispdma->dma_irq_lock);
	spin_lock_init(&ispdma->dmaflg_lock);
	init_completion(&ispdma->ipc_event);
	mutex_init(&ispdma->ispdma_mutex);

	ispdma->input = ISPDMA_INPUT_NONE;
	ispdma->disp_out = ISPDMA_OUTPUT_NONE;
	ispdma->codec_out = ISPDMA_OUTPUT_NONE;
	ispdma->state = ISP_PIPELINE_STREAM_STOPPED;

	v4l2_subdev_init(sd, &ispdma_v4l2_ops);
	sd->internal_ops = &ispdma_v4l2_internal_ops;
	strlcpy(sd->name, "mvisp_ispdma", sizeof(sd->name));
	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, ispdma);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

#if 0
	v4l2_ctrl_handler_init(&ispdma->ctrls, 2);
	v4l2_ctrl_new_std(&ispdma->ctrls, &ispdma_ctrl_ops,
				V4L2_CID_BRIGHTNESS,
				ISPPRV_BRIGHT_LOW, ISPPRV_BRIGHT_HIGH,
				ISPPRV_BRIGHT_STEP, ISPPRV_BRIGHT_DEF);
	v4l2_ctrl_new_std(&ispdma->ctrls, &ispdma_ctrl_ops,
				V4L2_CID_CONTRAST,
				ISPPRV_CONTRAST_LOW, ISPPRV_CONTRAST_HIGH,
				ISPPRV_CONTRAST_STEP, ISPPRV_CONTRAST_DEF);
	v4l2_ctrl_handler_setup(&ispdma->ctrls);
	sd->ctrl_handler = &ispdma->ctrls;
#endif

	pads[ISPDMA_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[ISPDMA_PAD_CODEC_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	pads[ISPDMA_PAD_DISPLAY_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	me->ops = &ispdma_media_ops;
	ret = media_entity_init(me, ISPDMA_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	ispdma_init_formats(sd, NULL);

	ispdma->vd_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ispdma->vd_in.ops = &ispdma_video_ops;
	ispdma->vd_in.isp = to_mvisp_device(ispdma);
	ispdma->vd_disp_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ispdma->vd_disp_out.ops = &ispdma_video_ops;
	ispdma->vd_disp_out.isp = to_mvisp_device(ispdma);
	ispdma->vd_codec_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ispdma->vd_codec_out.ops = &ispdma_video_ops;
	ispdma->vd_codec_out.isp = to_mvisp_device(ispdma);

	ret = mvisp_video_init
		(&ispdma->vd_in, ISP_VIDEO_INPUT_NAME, false);
	if (ret < 0)
		return ret;

	ret = mvisp_video_init
		(&ispdma->vd_codec_out, ISP_VIDEO_CODEC_NAME, false);
	if (ret < 0)
		return ret;
	ret = mvisp_video_init
		(&ispdma->vd_disp_out, ISP_VIDEO_DISPLAY_NAME, false);
	if (ret < 0)
		return ret;

	/* Connect the video nodes to the ispdma subdev. */
	ret = media_entity_create_link
			(&ispdma->vd_in.video.entity, 0,
			&ispdma->subdev.entity, ISPDMA_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link
			(&ispdma->subdev.entity, ISPDMA_PAD_CODEC_SOURCE,
			&ispdma->vd_codec_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link
			(&ispdma->subdev.entity, ISPDMA_PAD_DISPLAY_SOURCE,
			&ispdma->vd_disp_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void mv_ispdma_unregister_entities(struct isp_ispdma_device *ispdma)
{
	media_entity_cleanup(&ispdma->subdev.entity);

	v4l2_device_unregister_subdev(&ispdma->subdev);
/*	v4l2_ctrl_handler_free(&ispdma->ctrls); */
	mvisp_video_unregister(&ispdma->vd_in);
	mvisp_video_unregister(&ispdma->vd_disp_out);
	mvisp_video_unregister(&ispdma->vd_codec_out);
}

int mv_ispdma_register_entities(struct isp_ispdma_device *ispdma,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &ispdma->subdev);
	if (ret < 0)
		goto error;

	ret = mvisp_video_register(&ispdma->vd_in, vdev);
	if (ret < 0)
		goto error;

	ret = mvisp_video_register(&ispdma->vd_codec_out, vdev);
	if (ret < 0)
		goto error;

	ret = mvisp_video_register(&ispdma->vd_disp_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	mv_ispdma_unregister_entities(ispdma);
	return ret;
}

void mv_ispdma_cleanup(struct mvisp_device *isp)
{
}

int mv_ispdma_init(struct mvisp_device *isp)
{
	struct isp_ispdma_device *ispdma = &isp->mvisp_ispdma;
	struct mvisp_platform_data *pdata = isp->pdata;
	int ret;

	ispdma_init_params(ispdma);
	ispdma->mvisp_reset = pdata->mvisp_reset;

	ret = ispdma_init_entities(ispdma);
	if (ret < 0)
		goto out;

out:
	if (ret)
		mv_ispdma_cleanup(isp);

	return ret;
}
