/*
 * linux/sound/soc/pxa/mmp2-squ.c
 *
 * Base on linux/sound/soc/pxa/pxa2xx-pcm.c
 *
 * Copyright (C) 2011 Marvell International Ltd.
 * Author: Bin Yang <bin.yang@marvell.com>
 *			 Yael Sheli Chemla<yael.s.shemla@marvell.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/dma.h>

#include <mach/mmp_dma.h>
#include <mach/sram.h>
#include <mach/cputype.h>
#include "mmp2-squ.h"

#include <linux/delay.h>

#ifdef CONFIG_CPU_MMP2

static const struct snd_pcm_hardware mmp2_pcm_hardware_playback = {
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
	.periods_max		= PAGE_SIZE / sizeof(mmp_tdma_desc),
	.buffer_bytes_max	= PAGE_SIZE,
	.fifo_size		= 32,
};

static const struct snd_pcm_hardware mmp2_pcm_hardware_capture = {
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
	.periods_max		= PAGE_SIZE / sizeof(mmp_tdma_desc),
	.buffer_bytes_max	= PAGE_SIZE,
	.fifo_size		= 32,
};

#elif defined(CONFIG_CPU_MMP3)

static const struct snd_pcm_hardware mmp2_pcm_hardware_playback = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
	    SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min = 32,
	.period_bytes_max = 20 * 1024,
	.periods_min = 1,
	.periods_max = PAGE_SIZE / sizeof(mmp_tdma_desc),
	.buffer_bytes_max = 60 * 1024,
	.fifo_size = 32,
};

static const struct snd_pcm_hardware mmp2_pcm_hardware_capture = {
	.info = SNDRV_PCM_INFO_MMAP |
	    SNDRV_PCM_INFO_MMAP_VALID |
	    SNDRV_PCM_INFO_INTERLEAVED |
	    SNDRV_PCM_INFO_PAUSE | SNDRV_PCM_INFO_RESUME,
	.formats = SNDRV_PCM_FMTBIT_S16_LE |
	    SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	.period_bytes_min = 32,
	.period_bytes_max = 20 * 1024,
	.periods_min = 1,
	.periods_max = PAGE_SIZE / sizeof(mmp_tdma_desc),
	.buffer_bytes_max = 56 * 1024,
	.fifo_size = 32,
};

#endif

static DECLARE_WAIT_QUEUE_HEAD(dma_wq);

static int mmp2_sram_mmap_noncached(struct vm_area_struct *vma,
				    void *cpu_addr, dma_addr_t dma_addr,
				    size_t size)
{
	unsigned long user_size;
	unsigned long off = vma->vm_pgoff;
	u32 ret;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	user_size = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	ret = remap_pfn_range(vma, vma->vm_start,
			      __phys_to_pfn(dma_addr) + off,
			      user_size << PAGE_SHIFT, vma->vm_page_prot);

	return ret;
}

static void mmp2_adma_irq(int dma_ch, void *dev_id)
{
	struct snd_pcm_substream *substream = dev_id;
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	u32 base_register = mmp_get_dma_reg_base(dma_ch);
	if (base_register) {
		if (TDISR(base_register) & 0x1)
			snd_pcm_period_elapsed(substream);
		else {
			printk(KERN_ERR "%s: SQU error on channel %d\n",
			       prtd->params->name, dma_ch);
		}
		TDISR(base_register) = 0;
	}
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

	/* this may get called several times by oss emulation
	 * with different params */
	if (prtd->params == NULL) {
		prtd->params = dma;
		ret = mmp_request_dma(prtd->params->name, dma->dma_ch,
					 mmp2_adma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->adma_ch = ret;
	} else if (prtd->params != dma) {
		mmp_free_dma(prtd->adma_ch);
		prtd->params = dma;
		ret = mmp_request_dma(prtd->params->name, dma->dma_ch,
					 mmp2_adma_irq, substream);
		if (ret < 0)
			return ret;

		prtd->adma_ch = ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = totsize;

	next_desc_phys = prtd->adma_desc_array_phys;
	dma_buff_phys = runtime->dma_addr;

	adma_desc = prtd->adma_desc_array;
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
	int ret = 0;
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;

	u32 base_register = mmp_get_dma_reg_base(prtd->adma_ch);
	if (base_register) {
		TDCR(base_register) = (prtd->params->dcmd)
		    & (~TDCR_CHANEN);
		TDIMR(base_register) = TDIMR_COMP;
	} else
		ret = -EINVAL;

	return ret;
}

static int mmp2_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct mmp2_runtime_data *prtd = substream->runtime->private_data;
	int ret = 0;
	u32 base_register;

	base_register = mmp_get_dma_reg_base(prtd->adma_ch);
	if (!base_register)
		return -EINVAL;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
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
	}

	return ret;
}

static snd_pcm_uframes_t mmp2_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;
	dma_addr_t ptr;
	snd_pcm_uframes_t x;

	u32 base_register = mmp_get_dma_reg_base(prtd->adma_ch);
	ptr = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
	    TDSAR(base_register) : TDDAR(base_register);

	x = bytes_to_frames(runtime, ptr - runtime->dma_addr);

	if (x == runtime->buffer_size)
		x = 0;
	return x;
}

static int mmp2_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd;
	int ret;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		snd_soc_set_runtime_hwparams(substream,
					     &mmp2_pcm_hardware_playback);
	else
		snd_soc_set_runtime_hwparams(substream,
					     &mmp2_pcm_hardware_capture);

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
		goto out;
	}

	/*
	 * avoid sram fragment, allocate dma buffer and
	 * dma desc list at the same time.
	 */
	prtd->adma_desc_array = (unsigned int)sram_alloc(
			"audio sram", PAGE_SIZE, (dma_addr_t *)&prtd->adma_desc_array_phys);
	if (!prtd->adma_desc_array) {
		ret = -ENOMEM;
		goto err1;
	}

	runtime->private_data = prtd;

	return 0;

err1:
	kfree(prtd);
out:
	return ret;
}

static int mmp2_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct mmp2_runtime_data *prtd = runtime->private_data;

	sram_free("audio sram", (void *)prtd->adma_desc_array, PAGE_SIZE);

	kfree(prtd);
	runtime->private_data = NULL;
	return 0;
}

static int mmp2_pcm_mmap(struct snd_pcm_substream *substream,
			 struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return mmp2_sram_mmap_noncached(vma,
					runtime->dma_area,
					runtime->dma_addr, runtime->dma_bytes);
}

struct snd_pcm_ops mmp2_pcm_ops = {
	.open = mmp2_pcm_open,
	.close = mmp2_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mmp2_pcm_hw_params,
	.hw_free = mmp2_pcm_hw_free,
	.prepare = mmp2_pcm_prepare,
	.trigger = mmp2_pcm_trigger,
	.pointer = mmp2_pcm_pointer,
	.mmap = mmp2_pcm_mmap,
};

static int mmp2_pcm_preallocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *buf = &substream->dma_buffer;
	size_t size;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		size = mmp2_pcm_hardware_playback.buffer_bytes_max;
	else
		size = mmp2_pcm_hardware_capture.buffer_bytes_max;

	buf->dev.type = SNDRV_DMA_TYPE_DEV;
	buf->dev.dev = pcm->card->dev;
	buf->private_data = NULL;
	buf->area = sram_alloc("audio sram", size, &buf->addr);
	if (!buf->area)
		return -ENOMEM;
	buf->bytes = size;
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
		sram_free("audio sram", (void *)buf->area, buf->bytes);
		buf->area = NULL;
	}
}

static u64 mmp2_pcm_dmamask = DMA_32BIT_MASK;

int mmp2_pcm_new(struct snd_card *card, struct snd_soc_dai *dai,
		 struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &mmp2_pcm_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = DMA_32BIT_MASK;

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
	.ops = &mmp2_pcm_ops,
	.pcm_new = mmp2_pcm_new,
	.pcm_free = mmp2_pcm_free_dma_buffers,
};
EXPORT_SYMBOL_GPL(mmp2_soc_platform);

static int __devinit mmp2_soc_platform_probe(struct platform_device *pdev)
{
	return snd_soc_register_platform(&pdev->dev, &mmp2_soc_platform);
}

static int __devexit mmp2_soc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

static struct platform_driver mmp2_pcm_driver = {
	.driver = {
		   .name = "mmp3-pcm-audio",
		   .owner = THIS_MODULE,
		   },

	.probe = mmp2_soc_platform_probe,
	.remove = __devexit_p(mmp2_soc_platform_remove),
};

static int __init snd_mmp2_pcm_init(void)
{
	return platform_driver_register(&mmp2_pcm_driver);
}

module_init(snd_mmp2_pcm_init);

static void __exit snd_mmp2_pcm_exit(void)
{
	platform_driver_unregister(&mmp2_pcm_driver);
}

module_exit(snd_mmp2_pcm_exit);

MODULE_AUTHOR("zhouqiao@marvell.com");
MODULE_DESCRIPTION("MMP3 SQU DMA module");
MODULE_LICENSE("GPL");
