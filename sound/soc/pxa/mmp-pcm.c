/*
 * linux/sound/soc/pxa/mmp2-squ.c
 *
 * Base on linux/sound/soc/pxa/pxa910-squ.c
 *
 * Copyright (C) 2011 Marvell International Ltd.
 * Author: Leo Yan <leoy@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>
#include <mach/mmp_dma.h>
#include <mach/sram.h>

#include "mmp-pcm.h"

static const struct snd_pcm_hardware mmp2_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE |
				  SNDRV_PCM_INFO_RESUME,
	.formats		= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min	= 1024,
	.period_bytes_max	= 2048,
	.periods_min		= 2,
	.periods_max		= MMP2_ADMA_DESC_SIZE / sizeof(mmp_tdma_desc),
	.buffer_bytes_max	= MMP2_DDR_BUF_SIZE,
	.fifo_size		= 32,
};

static DECLARE_WAIT_QUEUE_HEAD(dma_wq);

#ifdef DEBUG
static void mmp2_pcm_dump_adma_list(struct mmp2_runtime_data *prtd)
{
	mmp_tdma_desc *adma_desc;

	pr_debug("audio dma list description is:\n");
	adma_desc = prtd->adma_desc_array;
	do {
		pr_debug("---------------------\n");
		pr_debug("src_addr = 0x%08x\n", adma_desc->src_addr);
		pr_debug("dst_addr = 0x%08x\n", adma_desc->dst_addr);
		pr_debug("byte_cnt = 0x%08x\n", adma_desc->byte_cnt);
		pr_debug("nxt_desc = 0x%08x\n", adma_desc->nxt_desc);

		adma_desc = (mmp_tdma_desc *)(adma_desc->nxt_desc -
				(int)prtd->adma_desc_array_phys +
				(int)prtd->adma_desc_array);

	} while (adma_desc != prtd->adma_desc_array);

	return;
}
#else
#define mmp2_pcm_dump_adma_list(prtd) do { } while (0)
#endif

static void mmp2_pcm_copy_data(int *dst, int *src, int size)
{
	int i;

	BUG_ON(!IS_ALIGNED((int)dst, sizeof(int)) ||
	       !IS_ALIGNED((int)src, sizeof(int)) ||
	       !IS_ALIGNED((int)size, sizeof(int)));

	size = size / sizeof(int);
	for (i = 0; i < size; i++) {
		*dst = *src;
		dst++;
		src++;
	}
	return;
}

/*
 * check if the pointer is in the specific region,
 * for the buffer is the ring buffer, so the region
 * may have two seperate buffers at the bottom and
 * top of the ring buffer.
 */
static int mmp2_pcm_point_is_in_region(u32 total, u32 start,
				       u32 len, u32 point)
{
	u32 h1_start, h1_size;
	u32 h2_start, h2_size;

	/* split region into two halves */
	if (start + len > total) {
		h1_start = start;
		h1_size  = total - start;
		h2_start = 0;
		h2_size  = start + len - total;
	} else {
		h1_start = start;
		h1_size  = len;
		h2_start = 0;
		h2_size  = 0;
	}

	/* in first half */
	if ((point >= h1_start) && (point < h1_start + h1_size))
		return h1_start + h1_size - point + h2_size;

	/* in bottom half */
	if ((point >= h2_start) && (point < h2_start + h2_size))
		return h2_start + h2_size - point;

	return 0;
}

static void mmp2_pcm_sync_sram_with_ddr(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	char *src, *dst;
	u32 point, rest;
	int num, i;
	u32 base;
	pr_debug("%s: copy begin, sram_blk_idx = %d rbuf_blk_idx = %d "
		 "sync_blk = %d", __func__,
		 prtd->sram_blk_idx,  prtd->rbuf_blk_idx,
		 prtd->sync_blk);

	if (!prtd->sync_blk)
		return;

	base = mmp_get_dma_reg_base(prtd->adma_ch);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/*
		 * adjust sram index if dma has run ahead
		 * rather than memory copy speed
		 */
		point = TDSAR(base) - prtd->sram_phys;
		rest = mmp2_pcm_point_is_in_region(prtd->sram_size,
				prtd->sram_blk_idx * prtd->blk_size,
				prtd->sync_blk * prtd->blk_size,
				point);
		if (rest) {
			prtd->sram_blk_idx = point / prtd->blk_size + 1;
			if (prtd->sram_blk_idx >= prtd->sram_blk_num)
				prtd->sram_blk_idx = 0;
		}

		dst = (char *)(prtd->sram_virt +
			prtd->sram_blk_idx * prtd->blk_size);
		src = (char *)(prtd->rbuf_virt +
			prtd->rbuf_blk_idx * prtd->blk_size);
	} else {
		/*
		 * check if have periods has not been over-written
		 * by dma polluted, will copy these periods data
		 * as possible.
		 */
		point = TDSAR(base) - prtd->sram_phys;
		rest = mmp2_pcm_point_is_in_region(prtd->sram_size,
				prtd->sram_blk_idx * prtd->blk_size,
				prtd->sync_blk * prtd->blk_size,
				point);
		if (rest) {
			prtd->sram_blk_idx = point / prtd->blk_size + 1;
			if (prtd->sram_blk_idx >= prtd->sram_blk_num)
				prtd->sram_blk_idx = 0;
			prtd->sync_blk = rest / prtd->blk_size;
		}

		dst = (char *)(prtd->rbuf_virt +
			prtd->rbuf_blk_idx * prtd->blk_size);
		src = (char *)(prtd->sram_virt +
			prtd->sram_blk_idx * prtd->blk_size);
	}

	num = prtd->sync_blk;
	for (i = 0; i < num; i++) {

		mmp2_pcm_copy_data((int *)dst, (int *)src,
			prtd->blk_size);

		prtd->sram_blk_idx++;
		if (prtd->sram_blk_idx >= prtd->sram_blk_num)
			prtd->sram_blk_idx = 0;
		prtd->rbuf_blk_idx++;
		if (prtd->rbuf_blk_idx >= prtd->rbuf_blk_num)
			prtd->rbuf_blk_idx = 0;

		dst += prtd->blk_size;
		src += prtd->blk_size;
	}
	prtd->sync_blk = 0;

	pr_debug("%s: copy end, sram_blk_idx = %d rbuf_blk_idx = %d "
		 "sync_blk = %d\n", __func__,
		 prtd->sram_blk_idx, prtd->rbuf_blk_idx,
		 prtd->sync_blk);
	return;
}

static void mmp2_pcm_adma_irq(int dma_ch, void *data)
{
	struct snd_pcm_substream *substream = data;
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;
	u32 base = mmp_get_dma_reg_base(prtd->adma_ch);
	if (!base)
		return;

	if (!(TDISR(base) & 0x1))
		return;
	/* clear adma irq status */
	TDISR(base) = 0;

	/* sync sram with ring buf */
	prtd->sync_blk = min(prtd->sync_blk + 1, prtd->sram_blk_num);
	mmp2_pcm_sync_sram_with_ddr(substream);
	snd_pcm_period_elapsed(substream);

	return;
}

static int mmp2_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct mmp2_adma_params *dma;
	size_t totsize = params_buffer_bytes(params);
	size_t period = params_period_bytes(params);
	mmp_tdma_desc *adma_desc;
	dma_addr_t dma_buff_phys, next_desc_phys;
	int ret;

	dma = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
	    rtd->cpu_dai->playback_dma_data : rtd->cpu_dai->capture_dma_data;
	/* return if this is a bufferless transfer e.g.
	 * codec <--> BT codec or GSM modem -- lg FIXME */
	if (!dma)
		return 0;

	/*
	 * this may get called several times by oss
	 * emulation with different params
	 */
	if (prtd->params == NULL) {
		prtd->params = dma;
		ret = mmp_request_dma(prtd->params->name, dma->dma_ch,
					 mmp2_pcm_adma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->adma_ch = ret;
	} else if (prtd->params != dma) {
		mmp_free_dma(prtd->adma_ch);
		prtd->params = dma;
		ret = mmp_request_dma(prtd->params->name, dma->dma_ch,
					 mmp2_pcm_adma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->adma_ch = ret;
	}
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	prtd->blk_size     = period;
	prtd->rbuf_virt    = (u32)runtime->dma_area;
	prtd->rbuf_phys    = (u32)runtime->dma_addr;
	prtd->rbuf_blk_idx = 0;
	prtd->rbuf_blk_num = totsize / period;
	prtd->rbuf_size	   = prtd->rbuf_blk_num * period;
	prtd->sram_blk_idx = 0;
	prtd->sram_blk_num = min(totsize, (size_t)MMP2_ADMA_BUF_SIZE) / period;
	prtd->sram_size    = prtd->sram_blk_num * period;
	prtd->sync_blk     = 0;

	totsize        = prtd->sram_size;
	dma_buff_phys  = prtd->sram_phys;
	next_desc_phys = prtd->adma_desc_array_phys;
	adma_desc      = prtd->adma_desc_array;

	do {
		next_desc_phys += sizeof(mmp_tdma_desc);

		adma_desc->nxt_desc = next_desc_phys;
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			adma_desc->src_addr = dma_buff_phys;
			adma_desc->dst_addr = prtd->params->dev_addr;
		} else {
			adma_desc->src_addr = prtd->params->dev_addr;
			adma_desc->dst_addr = dma_buff_phys;
		}
		if (period > totsize)
			period = totsize;
		adma_desc->byte_cnt = period;
		adma_desc++;
		dma_buff_phys += period;

	} while (totsize -= period);
	adma_desc[-1].nxt_desc = prtd->adma_desc_array_phys;

	mmp2_pcm_dump_adma_list(prtd);

	return 0;
}

static int mmp2_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	if (prtd->adma_ch != -1) {
		snd_pcm_set_runtime_buffer(substream, NULL);
		mmp_free_dma(prtd->adma_ch);
		prtd->adma_ch = -1;
	}

	return 0;
}

static int mmp2_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;
	u32 base;

	base = mmp_get_dma_reg_base(prtd->adma_ch);
	if (!base)
		return -EINVAL;

	TDCR(base)  = (prtd->params->dcmd) & (~TDCR_CHANEN);
	TDIMR(base) = TDIMR_COMP;

	return 0;
}

static int mmp2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;
	unsigned long flags;
	int ret = 0;
	u32 base_register;

	base_register = mmp_get_dma_reg_base(prtd->adma_ch);
	if (!base_register)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		spin_lock_irqsave(&prtd->lock, flags);
		prtd->sram_blk_idx = 0;
		prtd->rbuf_blk_idx = 0;
		prtd->sync_blk = 0;

		/* clean sram buffer */
		memset((void *)prtd->sram_virt, 0, prtd->sram_size);

		pr_debug("%s: sram_blk_num = %d\n", __func__,
			 prtd->sram_blk_num);

		mmp2_pcm_sync_sram_with_ddr(substream);
		spin_unlock_irqrestore(&prtd->lock, flags);

		TDNDPR(base_register) = prtd->adma_desc_array_phys;
		TDCR(base_register) = prtd->params->dcmd | TDCR_CHANEN;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		TDCR(base_register) = prtd->params->dcmd;
		wake_up(&dma_wq);
		break;

	case SNDRV_PCM_TRIGGER_RESUME:
		TDCR(base_register) = prtd->params->dcmd | TDCR_CHANEN;
		break;

	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		TDNDPR(base_register) = prtd->adma_desc_array_phys;
		TDCR(base_register) = prtd->params->dcmd | TDCR_CHANEN;
		break;

	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static snd_pcm_uframes_t
mmp2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	snd_pcm_uframes_t x;

	pr_debug("%s enter\n", __func__);

	x = bytes_to_frames(runtime, prtd->rbuf_blk_idx * prtd->blk_size);
	if (x == runtime->buffer_size)
		x = 0;

	return x;
}

static int mmp2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd;
	int ret;

	pr_debug("%s enter\n", __func__);

	snd_soc_set_runtime_hwparams(substream, &mmp2_pcm_hardware);

	/*
	 * For mysterious reasons (and despite what the manual says)
	 * playback samples are lost if the DMA count is not a multiple
	 * of the DMA burst size.  Let's add a rule to enforce that.
	 */
	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_step(runtime, 0,
		SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 32);
	if (ret)
		goto out;

	ret = snd_pcm_hw_constraint_integer(runtime,
					SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(struct mmp2_runtime_data), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto alloc_sram_err;
	}

	prtd->substream = substream;
	runtime->private_data = prtd;
	prtd->adma_ch = -1;

	/*
	 * avoid sram fragment, allocate dma buffer and
	 * dma desc list at the same time.
	 */
	prtd->sram_virt = (unsigned int)sram_alloc(
			"audio sram",
			MMP2_ADMA_BUF_SIZE + MMP2_ADMA_DESC_SIZE,
			(dma_addr_t *)&prtd->sram_phys);

	prtd->adma_desc_array = (void *)(prtd->sram_virt + MMP2_ADMA_BUF_SIZE);
	prtd->adma_desc_array_phys = (dma_addr_t)(prtd->sram_phys
					+ MMP2_ADMA_BUF_SIZE);

	runtime->private_data = prtd;

	return 0;

alloc_sram_err:
	kfree(prtd);
out:
	return ret;
}

static int mmp2_pcm_suspend(struct snd_soc_dai *dai)
{
	return 0;
}

static int mmp2_pcm_resume(struct snd_soc_dai *dai)
{
	return 0;
}

static int mmp2_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;

	pr_debug("%s enter\n", __func__);

	sram_free("audio sram",
		(void *)prtd->adma_desc_array,
		MMP2_ADMA_BUF_SIZE + MMP2_ADMA_DESC_SIZE);
	sram_free("audio sram",
		(void *)prtd->sram_virt,
		MMP2_ADMA_BUF_SIZE + MMP2_ADMA_DESC_SIZE);
	kfree(prtd);
	runtime->private_data = NULL;
	return 0;
}

static int mmp2_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long off = vma->vm_pgoff;

	pr_debug("%s enter\n", __func__);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
				__phys_to_pfn(runtime->dma_addr) + off,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot);
}

struct snd_pcm_ops mmp2_pcm_ops = {
	.open		= mmp2_pcm_open,
	.close		= mmp2_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= mmp2_pcm_hw_params,
	.hw_free	= mmp2_pcm_hw_free,
	.prepare	= mmp2_pcm_prepare,
	.trigger	= mmp2_pcm_trigger,
	.pointer	= mmp2_pcm_pointer,
	.mmap		= mmp2_pcm_mmap,
};

static int mmp2_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size = mmp2_pcm_hardware.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = dma_alloc_coherent(pcm->card->dev, size,
			&buf->addr, GFP_KERNEL);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;

	printk(KERN_INFO "%s: pre-alloc dma buf (va : pa) 0x%x : 0x%x\n",
		__func__, (int)buf->area, (int)buf->addr);
	return 0;
}

static void mmp2_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {

		substream = pcm->streams[stream].substream;
		if (!substream)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				  buf->area, buf->addr);
		buf->area = NULL;
	}
	return;
}

static u64 mmp2_pcm_dmamask = DMA_BIT_MASK(64);

int mmp2_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
	struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mmp2_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	if (dai->driver->playback.channels_min) {
		ret = mmp2_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->driver->capture.channels_min) {
		ret = mmp2_pcm_preallocate_dma_buffer(pcm,
			SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}
 out:
	return ret;
}

struct snd_soc_platform_driver mmp2_soc_platform = {
	.ops		= &mmp2_pcm_ops,
	.pcm_new	= mmp2_pcm_new,
	.pcm_free	= mmp2_pcm_free_dma_buffers,
	.suspend	= mmp2_pcm_suspend,
	.resume		= mmp2_pcm_resume,
};
EXPORT_SYMBOL_GPL(mmp2_soc_platform);

static __devinit int mmp_pcm_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev,
			&mmp2_soc_platform);
}

static int __devexit mmp_pcm_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver mmp_pcm_driver = {
	.driver = {
		.name = "mmp-pcm-audio",
		.owner = THIS_MODULE,
	},

	.probe = mmp_pcm_probe,
	.remove = __devexit_p(mmp_pcm_remove),
};

static int __init mmp2_pcm_modinit(void)
{
	return platform_driver_register(&mmp_pcm_driver);
}
module_init(mmp2_pcm_modinit);

static void __exit mmp2_pcm_modexit(void)
{
	platform_driver_unregister(&mmp_pcm_driver);
}
module_exit(mmp2_pcm_modexit);

MODULE_AUTHOR("leoy@marvell.com");
MODULE_DESCRIPTION("MMP2 Audio DMA module");
MODULE_LICENSE("GPL");
