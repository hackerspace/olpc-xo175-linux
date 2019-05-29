#ifndef __MACH_MMP_DMA_H
#define __MACH_MMP_DMA_H

#include <mach/mmp_audisland.h>

#define __DMA_REG(x, y)		(*((volatile u32 *)(x + y)))

#define ADMA1_CH0_BASE		(AUD_VIRT_BASE + 0x800)
#define ADMA1_CH1_BASE		(AUD_VIRT_BASE + 0x804)
#define ADMA2_CH0_BASE		(AUD_VIRT_BASE + 0x900)
#define ADMA2_CH1_BASE		(AUD_VIRT_BASE + 0x904)
#define MDMA1_CH0_BASE		(AXI_VIRT_BASE + 0xA0a00)
#define MDMA1_CH1_BASE		(AXI_VIRT_BASE + 0xA0a04)
#define VDMA1_CH0_BASE		(AXI_VIRT_BASE + 0x0B300)
#define VDMA1_CH1_BASE		(AXI_VIRT_BASE + 0x0B380)

/* Two-Channel DMA registers */
#define TDBCR(base)		__DMA_REG(base, 0x0)	/* Byte Count Register */
#define TDSAR(base)		__DMA_REG(base, 0x10)	/* Src Addr Register */
#define TDDAR(base)		__DMA_REG(base, 0x20)   /* Dst Addr Register */
#define TDNDPR(base)		__DMA_REG(base, 0x30)   /* Next Desc Pointer Register */
#define TDCR(base)		__DMA_REG(base, 0x40)   /* Control Register */
#define TDCP(base)		__DMA_REG(base, 0x60)   /* Priority Register */
#define TDCDPR(base)		__DMA_REG(base, 0x70)   /* Current Desc Pointer Register */
#define TDIMR(base)		__DMA_REG(base, 0x80)   /* Int Mask Register */
#define TDISR(base)		__DMA_REG(base, 0xa0)   /* Int Status Register */
#define VDCR(base)		__DMA_REG(base, 0x28)	/* FIXME: Remove VDMA from this file */

/* Two-Channel DMA Control Register */
#define TDCR_SSZ_8_BITS		(0x0 << 22)	/* Sample Size */
#define TDCR_SSZ_12_BITS	(0x1 << 22)
#define TDCR_SSZ_16_BITS	(0x2 << 22)
#define TDCR_SSZ_20_BITS	(0x3 << 22)
#define TDCR_SSZ_24_BITS	(0x4 << 22)
#define TDCR_SSZ_32_BITS	(0x5 << 22)
#define TDCR_SSZ_SHIFT		(0x1 << 22)
#define TDCR_SSZ_MASK		(0x7 << 22)
#define TDCR_ABR		(0x1 << 20)	/* Channel Abort */
#define TDCR_CDE		(0x1 << 17)	/* Close Desc Enable */
#define TDCR_PACKMOD		(0x1 << 16)	/* Pack Mode (ADMA Only) */
#define TDCR_CLK_GATE_CTL	(0x1 << 16)	/* Clock Gate Conrol (MDMA Only) */
#define TDCR_CLK_GATE_ON	(0x1 << 15)	/* Clock Gate On (MDMA Only) */
#define TDCR_CHANACT		(0x1 << 14)	/* Channel Active */
#define TDCR_FETCHND		(0x1 << 13)	/* Fetch Next Desc */
#define TDCR_CHANEN		(0x1 << 12)	/* Channel Enable */
#define TDCR_TRANSMOD		(0x1 << 11)	/* Transfer Mode, 1: Normal mode (MDMA Only) */
#define TDCR_INTMODE		(0x1 << 10)	/* Interrupt Mode */
#define TDCR_CHAINMOD		(0x1 << 9)	/* Chain Mode */
#define TDCR_BURSTSZ_MSK	(0x7 << 6)	/* Burst Size */
#define TDCR_BURSTSZ_4B		(0x0 << 6)
#define TDCR_BURSTSZ_8B		(0x1 << 6)
#define TDCR_BURSTSZ_16B	(0x3 << 6)
#define TDCR_BURSTSZ_32B	(0x6 << 6)
#define TDCR_BURSTSZ_64B	(0x7 << 6)
#define TDCR_BURSTSZ_128B	(0x5 << 6)
#define TDCR_DSTDIR_MSK		(0x3 << 4)	/* Dst Direction */
#define TDCR_DSTDIR_ADDR_HOLD	(0x2 << 4)	/* Dst Addr Hold */
#define TDCR_DSTDIR_ADDR_INC	(0x0 << 4)	/* Dst Addr Increment */
#define TDCR_SRCDIR_MSK		(0x3 << 2)	/* Src Direction */
#define TDCR_SRCDIR_ADDR_HOLD	(0x2 << 2)	/* Src Addr Hold */
#define TDCR_SRCDIR_ADDR_INC	(0x0 << 2)	/* Src Addr Increment */
#define TDCR_DSTDESCCONT	(0x1 << 1)
#define TDCR_SRCDESTCONT	(0x1 << 0)

/* Two-Channel DMA Int Mask Register */
#define TDIMR_COMP		(0x1 << 0)

/* Two-Channel DMA Int Status Register */
#define TDISR_COMP		(0x1 << 0)

enum mmp_tdma_type {
	MDMA1_CH0 = 0,
	MDMA1_CH1,
	ADMA1_CH0,
	ADMA1_CH1,
	ADMA2_CH0,
	ADMA2_CH1,
	VDMA1_CH0,
	VDMA1_CH1,
	DMA_CH_NUM,
};

/*
 * Two-Channel DMA Descriptor Struct
 * NOTE: desc's buf must be aligned to 16 bytes.
 */
typedef struct mmp_tdma_desc {
	volatile u32 byte_cnt;	/* byte count */
	volatile u32 src_addr;	/* source address */
	volatile u32 dst_addr;	/* target address */
	volatile u32 nxt_desc;	/* next descriptor dress */
} mmp_tdma_desc;

int __init mmp_init_dma(unsigned int irq);
unsigned int mmp_request_dma(char *name, unsigned int dma_ch,
		void (*irq_handler)(int, void *), void *data);
void mmp_free_dma(unsigned int dma_ch);

u32 mmp_get_dma_reg_base(enum mmp_tdma_type dma_type);

#endif /* __MACH_MMP_DMA_H */
