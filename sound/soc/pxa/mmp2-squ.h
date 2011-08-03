/*
 * linux/sound/soc/pxa/mmp2-squ.h
 *
 * Base on linux/sound/soc/pxa/pxa910-squ.h
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
#ifndef _MMP2_SQU_H
#define _MMP2_SQU_H

#define MMP2_ADMA_BUF_SIZE		(PAGE_SIZE)
#define MMP2_ADMA_DESC_SIZE		(1024)
#define MMP2_DDR_BUF_SIZE		(MMP2_ADMA_BUF_SIZE << 4)

/* adma buf and desc size */
#define MMP2_ADMA_TOTAL_SIZE		(PAGE_SIZE + 1024)

struct mmp2_adma_params {
	char *name;			/* stream identifier */
	u32 dma_ch;			/* the DMA request channel to use */
	u32 dcmd;			/* DMA descriptor dcmd field */
	u32 dev_addr;			/* device physical address for DMA */
};

struct mmp2_adma_registers {
	u32 byte_counter;
	u32 src_addr;
	u32 dest_addr;
	u32 next_desc_ptr;
	u32 ctrl;
	u32 chan_pri;			/* Only used in channel 0 */
	u32 curr_desc_ptr;
	u32 intr_mask;
	u32 intr_status;
};

struct mmp2_runtime_data {
	int adma_ch;
	struct mmp2_adma_params *params;
	void *adma_desc_array;
	dma_addr_t adma_desc_array_phys;

	spinlock_t lock;

	/* sram buf */
	unsigned int sram_virt;
	unsigned int sram_phys;
	unsigned int sram_size;
	unsigned int sram_blk_idx;
	unsigned int sram_blk_num;

	/* notes: below ddr info should be unneccessary for MMP3 */
	/* ddr buf */
	unsigned int rbuf_virt;
	unsigned int rbuf_phys;
	unsigned int rbuf_size;
	unsigned int rbuf_blk_idx;
	unsigned int rbuf_blk_num;

	unsigned int sync_blk;
	/* end notes */
	u32 blk_size;

	struct snd_pcm_substream *substream;

	struct mmp2_adma_registers adma_saved;
	char sram_saved[MMP2_ADMA_TOTAL_SIZE];
};

extern struct snd_soc_platform_driver mmp2_soc_platform;

#endif /* _MMP2_SQU_H */
