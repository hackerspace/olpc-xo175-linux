#ifndef __MACH_PXA688_DMA_H
#define __MACH_PXA688_DMA_H

#include <mach/addr-map.h>
#include <mach/mmp3_audisland.h>

#define __PXA688_DMA_REG(x, y)	(*((volatile u32 *)(AXI_VIRT_BASE + (x) + (y))))
#define __PXA688_ADMA_REG(x, y)	(*((volatile u32 *)(AUD_VIRT_BASE + (x) + (y))))

/* ADMA */
#define PXA688_ADSAR(base)			__PXA688_ADMA_REG(base, 0x10)
#define PXA688_ADDAR(base)			__PXA688_ADMA_REG(base, 0x20)
#define PXA688_ADNDPR(base)			__PXA688_ADMA_REG(base, 0x30)
#define PXA688_ADCR(base)			__PXA688_ADMA_REG(base, 0x40)
#define PXA688_ADCP(base)			__PXA688_ADMA_REG(base, 0x60)
#define PXA688_ADCDPR(base)			__PXA688_ADMA_REG(base, 0x70)
#define PXA688_ADIMR(base)			__PXA688_ADMA_REG(base, 0x80)
#define PXA688_ADISR(base)			__PXA688_ADMA_REG(base, 0xa0)

#define ADMA1_CH0_BASE		0x800
#define ADMA1_CH1_BASE		0x804
#define ADMA2_CH0_BASE		0x900
#define ADMA2_CH1_BASE		0x904
#define MDMA_CH0_BASE		0xA0a00
#define MDMA_CH1_BASE		0xA0a04
#define VDMA_CH0_BASE		0x0B300
#define VDMA_CH1_BASE		0x0B380

#define PXA688_DSAR(base)			__PXA688_DMA_REG(base, 0x10)
#define PXA688_DDAR(base)			__PXA688_DMA_REG(base, 0x20)
#define PXA688_DNDPR(base)			__PXA688_DMA_REG(base, 0x30)
#define PXA688_DCR(base)			__PXA688_DMA_REG(base, 0x40)
#define PXA688_DCP(base)			__PXA688_DMA_REG(base, 0x60)
#define PXA688_DCDPR(base)			__PXA688_DMA_REG(base, 0x70)
#define PXA688_DIMR(base)			__PXA688_DMA_REG(base, 0x80)
#define PXA688_DISR(base)			__PXA688_DMA_REG(base, 0xa0)
#define PXA688_VDCR(base)			__PXA688_DMA_REG(base, 0x28)

#define ADCR_SSZ_8_BITS		(0x0 << 22)	/* sample size */
#define ADCR_SSZ_12_BITS	(0x1 << 22)
#define ADCR_SSZ_16_BITS	(0x2 << 22)
#define ADCR_SSZ_20_BITS	(0x3 << 22)
#define ADCR_SSZ_24_BITS	(0x4 << 22)
#define ADCR_SSZ_32_BITS	(0x5 << 22)
#define ADCR_SSZ_SHIFT		(0x1 << 22)
#define ADCR_SSZ_MASK		(0x7 << 22)
#define ADCR_ABR		(0x1 << 20)	/* channel abort */
#define ADCR_CDE		(0x1 << 17)
#define ADCR_PACKMOD		(0x1 << 16)
#define ADCR_SDA		(0x1 << 15)
#define ADCR_CHANACT		(0x1 << 14)	/* dma channel active */
#define ADCR_FETCHND		(0x1 << 13)
#define ADCR_CHANEN		(0x1 << 12)	/* channel enable */
#define ADCR_TRANSMOD		(0x1 << 11)	/* transmod */
#define ADCR_INTMODE		(0x1 << 10)	/* interrupt mode */
#define ADCR_CHAINMOD		(0x1 << 9)	/* chain mode */
#define ADCR_BURSTLIMIT_MSK	(0x7 << 6)
#define ADCR_DESTDIR_MSK	(0x3 << 4)
#define ADCR_SRCDIR_MSK		(0x3 << 2) /* Source Direction */
#define ADCR_DESTDESCCONT	(0x1 << 1)
#define ADCR_SRCDESTCONT	(0x1 << 0)
#define ADCR_DST_ADDR_INC	(0x0 << 4)
#define ADCR_DST_ADDR_HOLD	(0x2 << 4)
#define ADCR_SRC_ADDR_INC	(0x0 << 2)
#define ADCR_SRC_ADDR_HOLD	(0x2 << 2)
#define ADCR_DMA_BURST_4B	(0x0 << 6)

#define ADIMR_COMP		(0x1 << 0)

/*mapping according to ICU_DMA_IRQ1[16:23]*/
typedef enum {
	MDMA_CH_0 = 0,
	MDMA_CH_1,
	ADMA1_CH_0,
	ADMA1_CH_1,
	ADMA2_CH_0,
	ADMA2_CH_1,
	VDMA_CH_0,
	VDMA_CH_1,
	DMA_CH_NUM,
} pxa688_dma_channel_mapping;

/*
 * Descriptor structure for PXA688 ADMA and MDMA engine
 * Note: this structure must always be aligned to a 16-byte boundary.
 */
typedef struct pxa688_dma_desc {
	volatile u32 byte_cnt;	/* byte count */
	volatile u32 src_addr;	/* source address */
	volatile u32 dst_addr;	/* target address */
	volatile u32 nxt_desc;	/* next descriptor dress */
} pxa688_dma_desc;

int __init pxa688_init_dma(unsigned int irq);

int pxa688_request_dma(char *name,
		       pxa688_dma_channel_mapping dma_ch,
		       void (*irq_handler) (int, void *), void *data);

void pxa688_free_dma(int dma_ch);

u32 pxa688_find_dma_register_base(int dma_ch);

#endif /* __MACH_PXA688_DMA_H */
