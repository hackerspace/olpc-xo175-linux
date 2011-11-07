/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#ifndef __MDMA_H
#define __MDMA_H

#define MDCR_FILLMOD		(1 << 25) /* FILLMod */
#define MDCR_ABR		(1 << 20) /* Channel Abort */
#define MDCR_CDE		(1 << 17)
#define MDCR_CHANACT		(1 << 14) /* DMA Channel Active */
#define MDCR_FETCHND		(1 << 13)
#define MDCR_CHANEN		(1 << 12) /* Channel Enable */
#define MDCR_TRANSMOD		(1 << 11) /* TransMod */
#define MDCR_INTMODE		(1 << 10) /* Interrupt Mode */
#define MDCR_CHAINMOD		(1 << 9) /* Chain Mode */
#define MDCR_BURSTLIMIT_MSK	(0x7 << 6)
#define MDCR_DESTDIR_MSK	(0x3 << 4)
#define MDCR_SRCDIR_MSK		(0x3 << 2) /* Source Direction */
#define MDCR_DST_ADDR_INC	(0 << 4)
#define MDCR_DST_ADDR_HOLD	(0x2 << 4)
#define MDCR_SRC_ADDR_INC	(0 << 2)
#define MDCR_SRC_ADDR_HOLD	(0x2 << 2)
#define	MDCR_DMA_BURST_8B	(0x1 << 6)	
#define	MDCR_DMA_BURST_16B	(0x3 << 6)	
#define	MDCR_DMA_BURST_128B	(0x5 << 6)	
#define	MDCR_DMA_BURST_32B	(0x6 << 6)	
#define MDCR_DMA_BURST_64B	(0x7 << 6)
#define MDIMR_COMP		(1 << 0)

/* user level ioctl commands for accessing MDMA APIs */
#define MDMAIO_DMA_MEMCPY	0
#define MDMAIO_DMA_MEMSET	1

unsigned long mdma_pmemcpy(unsigned long pdst, unsigned long psrc, unsigned int len);
unsigned long mdma_pmemset(unsigned long paddr, unsigned long c, unsigned int len);

typedef struct {
	unsigned long srcaddr;	/* the source address of the block of memory */
	unsigned long dstaddr;	/* the destination address of the block of memory */
	unsigned int length;	/* the length of the block of memory */
} ioctl_mdma_memcpy;

typedef struct {
	unsigned long addr;		/* the address of the block of memory */
	unsigned long data;		/* the data filled */
	unsigned int length;	/* the length of the block of memory */
} ioctl_mdma_memset;


#endif /* __MDMA_H */
