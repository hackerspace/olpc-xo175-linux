/*
 *  linux/arch/arm/mach-mmp/include/mach/sram.h
 *
 *  PXA688 SRAM Memory Management
 *
 *  Copyright (c) 2011 Marvell Semiconductors Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 */

#ifndef __ARCH_SRAM_H
#define __ARCH_SRAM_H

/* ARBITRARY:  SRAM allocations are multiples of this 2^N size */
#define AUDIO_SRAM_GRANULARITY	1024
#define VIDEO_SRAM_GRANULARITY	64

/*
 * SRAM allocations return a CPU virtual address, or NULL on error.
 * If a DMA address is requested and the SRAM supports DMA, its
 * mapped address is also returned.
 *
 * Errors include SRAM memory not being available, and requesting
 * DMA mapped SRAM on systems which don't allow that.
 */

struct sram_bank {
	struct list_head sram_list;
	u32 step;
	u32 sram_phys;
	u32 sram_size;
	char *pool_name;
	void __iomem *sram_virt;
	struct gen_pool *pool;
};

extern void *sram_alloc(char *name, size_t len, dma_addr_t *dma);
extern void sram_free(char *name, void *addr, size_t len);

#endif /* __ARCH_SRAM_H */
