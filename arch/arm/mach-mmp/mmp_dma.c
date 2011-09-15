/*
 *  linux/arch/arm/mach-pxa/mmp_dma.c
 *
 *  MMP DMA registration and IRQ dispatching
 *
 *  Mainly for Audio DMA, Memory DMA and Video DMA; Keep this driver
 *  and later should move to use dmaengine to manage dma channels.
 *
 *  Author:	Nicolas Pitre
 *  Created:	Nov 15, 2001
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/slab.h>

#include <asm/system.h>
#include <asm/irq.h>

#include <mach/hardware.h>
#include <mach/regs-icu.h>
#include <mach/mmp_dma.h>

struct mmp_dma_channel {
	char *name;
	void (*irq_handler)(int, void *);
	void *data;
};

static struct mmp_dma_channel *mmp_dma_channels;
static u32 mmp_reg_base_arr[DMA_CH_NUM] = {
	MDMA1_CH0_BASE, MDMA1_CH1_BASE,
	ADMA1_CH0_BASE, ADMA1_CH1_BASE,
	ADMA2_CH0_BASE, ADMA2_CH1_BASE,
	VDMA1_CH0_BASE, VDMA1_CH1_BASE,
};

u32 mmp_get_dma_reg_base(unsigned int dma_ch)
{
	if (dma_ch > DMA_CH_NUM)
		return 0;
	else
		return mmp_reg_base_arr[dma_ch];
}

unsigned int mmp_request_dma(char *name, unsigned int dma_ch,
		void (*irq_handler)(int, void *), void *data)
{
	unsigned long flags;
	int ret = 0;

	/* basic sanity checks */
	if (!name || !irq_handler || (dma_ch >= DMA_CH_NUM))
		return -EINVAL;

	local_irq_save(flags);

	if (!mmp_dma_channels[dma_ch].name) {
		u32 base, msk;

		base = mmp_get_dma_reg_base(dma_ch);

		if (dma_ch < VDMA1_CH0)
			TDCR(base) = 0;
		else
			VDCR(base) = 0;

		msk = __raw_readl(ICU_DMAIRQ_MASK);
		msk &= ~(1 << (16 + dma_ch));
		__raw_writel(msk, ICU_DMAIRQ_MASK);

		mmp_dma_channels[dma_ch].name = name;
		mmp_dma_channels[dma_ch].irq_handler = irq_handler;
		mmp_dma_channels[dma_ch].data = data;
		ret = dma_ch;
	} else {
		printk(KERN_WARNING "No more available MMP DMA "
		       "channels for %s\n", name);
		ret = -ENODEV;
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(mmp_request_dma);

void mmp_free_dma(unsigned int dma_ch)
{
	unsigned long flags;
	u32 base, msk;

	if (dma_ch >= DMA_CH_NUM) {
		printk(KERN_CRIT
		       "%s: trying to free an invalid channel %d\n",
		       __func__, dma_ch);
		return;
	}

	if (!mmp_dma_channels[dma_ch].name) {
		printk(KERN_CRIT "%s: trying to free channel %d "
		       "which is already freed\n", __func__, dma_ch);
		return;
	}

	local_irq_save(flags);

	base = mmp_get_dma_reg_base(dma_ch);

	if (dma_ch < VDMA1_CH0)
		TDCR(base) = 0;
	else
		VDCR(base) = 0;

	msk = __raw_readl(ICU_DMAIRQ_MASK);
	msk |= (1 << (16 + dma_ch));
	__raw_writel(msk, ICU_DMAIRQ_MASK);

	mmp_dma_channels[dma_ch].name = NULL;
	local_irq_restore(flags);
}
EXPORT_SYMBOL(mmp_free_dma);

static irqreturn_t mmp_dma_irq_handler(int irq, void *dev_id)
{
	int i;
	u32 base;
	int dint = __raw_readl(ICU_DMAIRQ_STATUS);

	if ((dint & 0xff0000) == 0)
		return IRQ_NONE;

	for (i = 0; i < DMA_CH_NUM; i++) {
		if (dint & (1 << (i + 16))) {
			struct mmp_dma_channel *channel = &mmp_dma_channels[i];

			base = mmp_get_dma_reg_base(i);
			if (channel->name && channel->irq_handler) {
				channel->irq_handler(i, channel->data);
				/*note: clear irq status in the handler */
			} else {
				/*
				 * IRQ for an unregistered DMA channel:
				 * let's clear the interrupts and disable it.
				 */
				printk(KERN_WARNING "spurious IRQ for DMA "
				       "channel %d\n", i);
				if (i < VDMA1_CH0)
					TDCR(base) = 0;
				else
					VDCR(base) = 0;
			}
		}
	}

	return IRQ_HANDLED;
}

int __init mmp_init_dma(unsigned int irq)
{
	int ret;

	mmp_dma_channels = kzalloc(sizeof(struct mmp_dma_channel) * DMA_CH_NUM,
				   GFP_KERNEL);
	if (!mmp_dma_channels)
		return -ENOMEM;

	ret = request_irq(irq, mmp_dma_irq_handler, IRQF_DISABLED | IRQF_SHARED,
			  "DMA", "MMP2_DMA");
	if (ret) {
		printk(KERN_CRIT "Wow! Can't register IRQ for MMP DMA\n");
		kfree(mmp_dma_channels);
	}

	return ret;
}
