/*----------------------------------------------------------

* V4L2 Driver for PXA95x camera host
* SCI -- Simple Camera Interface
* CSI -- CSI-2 controller, refer to MIPI CSI-2 protocol
*
* Based on linux/drivers/media/video/pxa_camera.c
*
* Copyright (C) 2010, Marvell International Ltd.
*		Qing Xu <qingx@marvell.com>
*		Jiaquan Su <jqsu@marvell.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.

----------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/list.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <mach/dma.h>
#include <mach/camera.h>
#include <mach/pxa3xx-regs.h>
#include <mach/regs-ost.h>

#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-chip-ident.h>

#include <linux/v4l2-mediabus.h>
#include <mach/dvfm.h>

MODULE_AUTHOR("Qing Xu <qingx@marvell.com>");
MODULE_DESCRIPTION("Marvell PXA955 Simple Capture Controller Driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("Video");

/*
 * CSI0 register base: 0x50020000
 * CSI1 register base: 0x50022000
 * Both CSI controller share the same CSSCR and CSGCR address
 */
/* Common register base */
#define REG_CSSCR_BASE		0x50020000
#define REG_CSGCR_BASE		0x5002000C
/* Register offset table */
#define REG_CSSCR		0x0000
#define REG_CSGCR		0x000C
#define REG_CSxCR0		0x0010
#define REG_CSxSR		0x0014
#define REG_CSxINEN		0x0018
#define REG_CSxINST		0x001C
#define REG_CSxTIM0		0x0020
#define REG_CSxTIM1		0x0024
#define REG_CSxGENDAT		0x0028
#define REG_CSxPHYCAL		0x002C
#define REG_CSxDTINTLV		0x0030

/* CSxCR0 */
#define CSxCR0_CSIEN	0x0001

/* CSxSR */
#define CSxSR_CSIEN	0x0001

/* CSxCR0 bits */
#define CSI_CONT_CSI_EN            (0x1u<<0)  /* CSI controller Enable/Disable*/
#define CSI_CONT_NOL(n)            ((n)<<2)   /* Number of Active Lanes (2 bits)*/
#define CSI_CONT_VC0_CFG(n)        ((n)<<5)   /* Chan 0 Addr Conf (2 bits)*/
#define CSI_CONT_VC1_CFG(n)        ((n)<<7)   /* Chan 1 Addr Conf (2 bits)*/
#define CSI_CONT_VC2_CFG(n)        ((n)<<9)   /* Chan 2 Addr Conf (2 bits)*/
#define CSI_CONT_VC3_CFG(n)        ((n)<<11)  /* Chan 3 Addr Conf (2 bits)*/

/* CSxTIM0 bits*/
#define CSI_CONT_CLTERMEN(n)       ((n)<<0)   /* Time to wait before enabling clock HS termination (8 bits)*/
#define CSI_CONT_CLSETTLE(n)       ((n)<<8)   /* Time to wait before HS clock is valid (8 bits)*/
#define CSI_CONT_CLMISS(n)         ((n)<<16)  /* Time to detect that the clock has stopped toggling (8 bits)*/
#define CSI_CONT_HSTERMEN(n)       ((n)<<24)  /* Time to wait before enabling data HS termination (8 bits)*/

/* CSxTIM1 bits */
#define CSI_CONT_HS_Rx_TO(n)       ((n)<<0)   /* Time to wait before declaring an error on a packet reception. This counter counts on the escape mode clock which is 52 MHz (16 bits)*/
#define CSI_CONT_HSTSETTLE(n)      ((n)<<16)  /* Timeout at RX end to neglect transition effects (8 bits)*/

/* CSxPHYCAL bits */
#define CSI_CONT_MIPI_BG_VREF_EN   (0x1u<<0)  /* See DPHY specifications for details*/
#define CSI_CONT_MIPI_RCOMP_CLKSEL (0x1u<<1)  /* Used to control the source of the PHY calibration clock (External/Internal).*/
#define CSI_CONT_MIPI_RCOMP_LOAD   (0x1u<<2)  /* Used to enable loading the MIPI_REN bypass value*/
#define CSI_CONT_MIPI_REN_BYPASS(n) ((n)<<3)  /* An 8-bit value to bypass the internal MIPI_REN value.*/
#define CSI_CONT_MIPI_RESET        (0x1u<<10) /* Mipi Reset Bit for pxa955 instead of 8th bit of REN_BYPASS*/
#define CSI_CONT_MIPI_RCOMP_CLK_EXT (0x1u<<31) /* Used tp provide a SW generated extternal clock to the PHY calibration block.*/

/* CSGCR bits */
#define CSI_CLK_DIV(n)             ((n)<<0)  /* Generated Clock Divisor. GCLK = 1/(DIV+2). 12 bits*/
#define CSI_CLK_GCLK_EN            (0x1u<<16)/* Generated Clock Enable*/

/* CSxINEN bits */
#define CSI_CONT_CSI_EN_INT        (0x1u<<0)  /* ENABLE Interrupt Enable*/
#define CSI_CONT_SoF_INT_EN        (0x1u<<1)  /* Start of Frame Interrupt Enable*/
#define CSI_CONT_EoF_INT_EN        (0x1u<<2)  /* End of Frame Interrupt Enable*/
#define CSI_CONT_OFLOW_INT_EN      (0x1u<<3)  /* Overflow Interrupt Enable*/
#define CSI_CONT_PHY_ERR_EN        (0x1u<<4)  /* D-PHY-related Error Interrupt Enable*/
#define CSI_CONT_GEN_PACK_INT_EN   (0x1u<<5)  /* Generic Packet Interrupt Enable*/
#define CSI_CONT_TIMEOUT_EN        (0x1u<<8)  /* Timeout Interrupt Enable*/
#define CSI_CONT_PROT_EN           (0x1u<<12) /* Protection Interupt Enable*/

/* CSxINEN bits */
#define CSI_CONT_SOF_INT_STS	(0x1u<<1)  /* Start of Frame Interrupt */
#define CSI_CONT_EOF_INT_STS	(0x1u<<2)  /* Start of Frame Interrupt */

#define ACSR_ALUF_MASK 0x00600000L
#define ACSR_ALUF_OFFSET 21

/* sci register, sci0 base: 0x50000000*/
#define REG_SCICR0	0x0000
#define REG_SCICR1	0x0004
#define REG_SCISR	0x0008
#define REG_SCIMASK	0x000C
#define REG_SCIFIFO	0x00F8
#define REG_SCIFIFOSR	0x00FC
#define REG_SCIDADDR0	0x0200
#define REG_SCISADDR0	0x0204
#define REG_SCITADDR0	0x0208
#define REG_SCIDCMD0	0x020C
#define REG_SCIDADDR1	0x0210
#define REG_SCISADDR1	0x0214
#define REG_SCITADDR1	0x0218
#define REG_SCIDCMD1	0x021C
#define REG_SCIDADDR2	0x0220
#define REG_SCISADDR2	0x0224
#define REG_SCITADDR2	0x0228
#define REG_SCIDCMD2	0x022C
#define REG_SCIDBR0	0x0300
#define REG_SCIDCSR0	0x0304
#define REG_SCIDBR1	0x0310
#define REG_SCIDCSR1	0x0314
#define REG_SCIDBR2	0x0320
#define REG_SCIDCSR2	0x0324

#define	IRQ_EOFX	0x00000001
#define	IRQ_EOF		0x00000002
#define	IRQ_DIS		0x00000004
#define	IRQ_OFO		0x00000008
#define	IRQ_SINT	0x00000100
#define	IRQ_DBS		0x00000200
#define	IRQ_SOFX	0x00000400
#define	IRQ_SOF		0x00000800

/* FMT_YUV420 & FMT_RGB666 only for input format without output format */
#define   FMT_RAW8	0x0000
#define   FMT_RAW10	0x0002
#define   FMT_YUV422	0x0003
#define   FMT_YUV420	0x0004
#define   FMT_RGB565	0x0005
#define   FMT_RGB666	0x0006
#define   FMT_RGB888	0x0007
#define   FMT_JPEG	0x0008
#define   FMT_YUV422PACKET	0x0009

#define SCICR1_FMT_OUT(n)	((n) << 12)	/* Output Pixel Format (4 bits) */
#define SCICR1_FMT_IN(n)	((n) << 28)	/* Input Pixel Format (4 bits) */

/* REG_SCIDCSR0 */
#define SCIDCSR_DMA_RUN	0x80000000	/* DMA Channel Enable */

/* REG_SCICR0 */
#define SCICR0_FSC(n)		(((n) & 0xF) << 2)	/* skip n frames before capture 1 frame */
#define SCICR0_CAP_EN	0x40000000	/* Capture Enable */
#define SCICR0_CI_EN	0x80000000	/* Camera Interface Enable */

/* REG_SCIFIFO */
#define SCIFIFO_TFS_64	0x0002
#define SCIFIFO_F0_EN	0x0010	/* FIFO 0 Enable */
#define SCIFIFO_F1_EN	0x0020	/* FIFO 1 Enable */
#define SCIFIFO_F2_EN	0x0040	/* FIFO 2 Enable */

/* REG_SCIDBRx*/
#define SCIDBR_EN	(0x1u<<1)	/*DMA Branch Enable*/

/* Define following macro to change arbitration register
 * for high camera priority on CI AXI arbitration */
#define _ARB_CHANGE_
#ifdef _ARB_CHANGE_
#define ARB_CNTRL_AXI   0x55D10000
#define ARB_CNTRL_CI1   0x55D10020
#define ARB_CNTRL_CI2   0x55D10040
#define ARB_CNTRL_GCU   0x55D10060
#define MC_WRR_NORMAL	0x7FF00280
#define MC_WRR_FAST	0x7FF007B0
#define MC_ARB_SCHEME	0x7FF00780
static unsigned int *pri_axi, *pri_ci1, *pri_ci2, *pri_gcu;
static unsigned int *wrr_nor, *wrr_fst, *arb_sch, wrr_nor_sv, wrr_fst_sv;
#endif

enum {
	SCI_PIXFMT_DEFAULT,
	SCI_PIXFMT_YVU,
};

/* Default output sequence is Y/U/V.
 * Currently we need switch U/V channels for YVU.
 * May add more for new requirements. */
static const int yuv_output_sequence[][3] = {
	[SCI_PIXFMT_DEFAULT] = {0, 1, 2},
	[SCI_PIXFMT_YVU] = {0, 2, 1},
};

/* During sensor format change, its output may get unstable, if controller
 * is turned on immediately, some unreasonable input may drive controller
 * into a deadloop reporting errors repeatedly. */
/* Define following macro to enable feature "controller reset when deadloop"
 * Add reset mechanism to recover controller from the deadloop*/
#define _CONTROLLER_DEADLOOP_RESET_

#define PXA955_CAM_VERSION_CODE KERNEL_VERSION(0, 0, 5)
#define PXA955_CAM_DRV_NAME "pxa95x-camera"

#define CCIC_0 0	/* for 2 camera controller*/
#define CCIC_1 1
#define CCIC_MAX 2

#define JPEG_COMPRESS_RATIO_HIGH 20
#define CHANNEL_NUM	3	/*YUV*/
#define MAX_DMA_BUFS	4
#define MIN_DMA_BUFS	2
#define MAX_DMA_SIZE	(1280 * 720 * 2)
#define JPEG_BUF_SIZE	(1024 * 1024)

#define ALIGN_SIZE	(cache_line_size())
#define ALIGN_MASK	(ALIGN_SIZE - 1)

static int dvfm_dev_idx;

static unsigned int skip_frame;

typedef enum {
	CAM_PCLK_104 = 0,
	CAM_PCLK_156,
	CAM_PCLK_208,
	CAM_PCLK_78,
} CAM_PCLK_t;

enum cam_state {
	CAM_STATE_UNKNOW	= 0,
	CAM_STATE_CLOSE,
	CAM_STATE_OPEN,
	CAM_STATE_FORMATED,
	CAM_STATE_STREAMING,
};

/* descriptor needed for the camera controller DMA engine */
struct pxa_cam_dma {
	dma_addr_t	sg_dma;
	struct pxa_dma_desc	*sg_cpu;
	size_t	sg_size;
	int	sglen;
};

struct pxa_buf_node {
	/* common v4l buffer stuff -- must be first */
	struct vb2_buffer vb2;
	struct list_head hook;	/* hook is mounted on dma_chain */
	enum v4l2_mbus_pixelcode	code;
	struct pxa_cam_dma dma_desc[CHANNEL_NUM];
};

struct pxa955_cam_dev {
	struct soc_camera_host	soc_host;
	struct soc_camera_device *icd;
	struct list_head dev_list;	/* link to other devices */

	/* SCI H/W facility */
	unsigned int		irq;
	void __iomem		*regs;
	struct clk *sci1_clk;
	struct clk *sci2_clk;

	struct resource		*res;
	unsigned long		platform_flags;
	enum cam_state		state;

	/* Associated CSI */
	struct pxa95x_csi_dev *csidev;

	/* DMA related */
	/* the chain for buffers that is actually been looped on by HW DMA */
	struct list_head dma_chain;
	unsigned int dma_bufs;
	/* the chain for buffers that will be added to DMA chain at next IRQ */
	struct list_head new_chain;
	unsigned int new_bufs;
	/* the spin lock to protect dma_chain and new_chain*/
	spinlock_t spin_lock;

	unsigned int channels;
	unsigned int channel_size[CHANNEL_NUM];
	unsigned int sci_pixfmt;

	/* vb2 facility */
	struct vb2_queue *vq;
	struct vb2_alloc_ctx *alloc_ctx;

#ifdef _CONTROLLER_DEADLOOP_RESET_
	/* A timer to detect controller error, on which controller will be reset */
	struct timer_list reset_timer;
	struct work_struct reset_wq;
	/* This var tells anyone who trys to modify reset timer
	 * it is being killed, don't modify it. otherwise, it'll restart again*/
	int killing_reset_timer;
#endif
};

#define MAGIC_DC_MEM 0x0733ac61
#define MAGIC_CHECK(is, should)					    \
	if (unlikely((is) != (should)))	{				    \
		pr_err("magic mismatch: %x expected %x\n", (is), (should)); \
		BUG();							    \
	}

#define pixfmtstr(x) (x) & 0xff, ((x) >> 8) & 0xff, ((x) >> 16) & 0xff, \
	((x) >> 24) & 0xff

/* Len should 8 bytes align, bit[2:0] should be 0 */
#define SINGLE_DESC_TRANS_MAX   (1 << 24)

#ifdef _CONTROLLER_DEADLOOP_RESET_
#define MIPI_RESET_TIMEOUT (msecs_to_jiffies(200))	/* For jpeg fps=10fps, so max polling time could be 200ms, */
static void ccic_timeout_handler(unsigned long);
#endif

/* VA for common register base, assgined value after 1st use */
static unsigned char __iomem *common_base;

static inline void csi_reg_write(struct pxa95x_csi_dev *csi, unsigned int reg,
		unsigned int val)
{
	/* If writing to common registers, override VA base */
	if (reg < REG_CSxCR0) {
		if (unlikely(common_base == NULL)) {
			common_base = ioremap(REG_CSSCR_BASE, REG_CSxCR0);
			BUG_ON(common_base == NULL);
		}
		__raw_writel(val, common_base + reg);
	} else
		__raw_writel(val, csi->regs + reg);
}

static inline unsigned int csi_reg_read(struct pxa95x_csi_dev *csi,
		unsigned int reg)
{
	/* If reading from common registers, override VA base */
	if (reg < REG_CSxCR0) {
		if (unlikely(common_base == NULL)) {
			common_base = ioremap(REG_CSSCR_BASE, REG_CSxCR0);
			BUG_ON(common_base == NULL);
			return -ENOMEM;
		}
		return __raw_readl(common_base + reg);
	} else
		return __raw_readl(csi->regs + reg);
}

void csi_reg_dump(struct pxa95x_csi_dev *csi)
{
	printk(KERN_ERR "CSSCR    |0x%08X\n", csi_reg_read(csi, REG_CSSCR));
	printk(KERN_ERR "CSGCR    |0x%08X\n", csi_reg_read(csi, REG_CSGCR));
	printk(KERN_ERR "CSxCR0   |0x%08X\n", csi_reg_read(csi, REG_CSxCR0));
	printk(KERN_ERR "CSxSR    |0x%08X\n", csi_reg_read(csi, REG_CSxSR));
	printk(KERN_ERR "CSxINEN  |0x%08X\n", csi_reg_read(csi, REG_CSxINEN));
	printk(KERN_ERR "CSxINST  |0x%08X\n", csi_reg_read(csi, REG_CSxINST));

	printk(KERN_ERR "CSxTIM0  |0x%08X\n", csi_reg_read(csi, REG_CSxTIM0));
	printk(KERN_ERR "CSxTIM1  |0x%08X\n", csi_reg_read(csi, REG_CSxTIM1));
	printk(KERN_ERR "CSxGENDAT|0x%08X\n", csi_reg_read(csi, REG_CSxGENDAT));
	printk(KERN_ERR "CSxPHYCAL|0x%08X\n", csi_reg_read(csi, REG_CSxPHYCAL));
}

void csi_cken(struct pxa95x_csi_dev *csi, int flag)
{
	if (flag) {
		clk_enable(csi->csi_tx_esc);
	} else {
		clk_disable(csi->csi_tx_esc);
	};
}

void csi_clkdiv(struct pxa95x_csi_dev *csi)
{
	unsigned int val = 0;
	unsigned int clk = 0;

	/* Before operating CSI, make sure CSI is NOT in the reset state */
	val = CSI_CONT_MIPI_RESET;
	csi_reg_write(csi, REG_CSxPHYCAL, val);

	/* using default sys video clock */
	clk = (ACSR & ACSR_ALUF_MASK) >> ACSR_ALUF_OFFSET;

	val = csi_reg_read(csi, REG_CSGCR);
	val &= 0xfffff000;/*bit 11:0 clk divider*/

	switch (clk) {
	case CAM_PCLK_104:
		/* divide clock by 4. 26Mhz.*/
		val = CSI_CLK_DIV(0x3) | CSI_CLK_GCLK_EN;
		break;

	case CAM_PCLK_156:
		/* divide clock by 6. 26Mhz.*/
		val = CSI_CLK_DIV(0x5) | CSI_CLK_GCLK_EN;
		break;

	case CAM_PCLK_208:
		/* divide clock by 8. 26Mhz.*/
		val = CSI_CLK_DIV(0x3) | CSI_CLK_GCLK_EN;/* in test result, the devider should be 3, no 7*/
		break;

	default:
		/* divide clock by 8. 26Mhz.*/
		val = CSI_CLK_DIV(0x7) | CSI_CLK_GCLK_EN;
		break;
	}
	csi_reg_write(csi, REG_CSGCR, val);
}

void csi_lane(struct pxa95x_csi_dev *csi, unsigned int lane)
{
	unsigned int val = 0;
	WARN_ON(lane>>2);
	if (lane != 0)
		lane--;	/* convert lane ammount to NOL */
	val = CSI_CONT_NOL(lane) |
		CSI_CONT_VC0_CFG(0x0) |
		CSI_CONT_VC1_CFG(0x1) |
		CSI_CONT_VC2_CFG(0x2) |
		CSI_CONT_VC3_CFG(0x3);
	csi_reg_write(csi, REG_CSxCR0, val);
}

/* This default phy setting is supposed to be workable with most sensors */
static struct mipi_phy default_phy_val = {
	.cl_termen	= 0x00,
	.cl_settle	= 0x0C,
	.cl_miss	= 0x00,
	.hs_termen	= 0x04,
	.hs_settle	= 0x48,
	.hs_rx_to	= 0xFFFF,
	.lane		= 1,
	.vc		= 0,
};

void csi_dphy(struct pxa95x_csi_dev *csi)
{
	unsigned int val = 0;
	unsigned int *calibration_p;

#if 0
	printk(KERN_INFO "cam: csi: configuring D-PHY parameter\n" \
		"------------------------------------------\n" \
		"\tCL_TERMEN = 0x%04X\n\tCL_SETTLE = 0x%04X\n" \
		"\tHS_TERMEN = 0x%04X\n\tHS_SETTLE = 0x%04X\n" \
		"------------------------------------------\n", \
		csi->phy_cfg->cl_termen, csi->phy_cfg->cl_settle, \
		csi->phy_cfg->hs_termen, csi->phy_cfg->hs_settle);
#endif
	csi_lane(csi, csi->phy_cfg->lane);

	val = 	CSI_CONT_CLTERMEN(csi->phy_cfg->cl_termen) |
		CSI_CONT_CLSETTLE(csi->phy_cfg->cl_settle) |
		CSI_CONT_CLMISS(0x00)   |
		CSI_CONT_HSTERMEN(csi->phy_cfg->hs_termen);
	csi_reg_write(csi, REG_CSxTIM0, val);

	val =	CSI_CONT_HS_Rx_TO(csi->phy_cfg->hs_rx_to) |
		CSI_CONT_HSTSETTLE(csi->phy_cfg->hs_settle);
	csi_reg_write(csi, REG_CSxTIM1, val);

	calibration_p = ioremap_nocache(0x42404078, 4);
	if (calibration_p != NULL) {
		val = *calibration_p | (5<<8);
		val &= ~(1<<9);
		*calibration_p = val;
		iounmap(calibration_p);
	}
	val = CSI_CONT_MIPI_RESET;
	val |= CSI_CONT_MIPI_REN_BYPASS(0x0) |
		CSI_CONT_MIPI_BG_VREF_EN;
	csi_reg_write(csi, REG_CSxPHYCAL, val);

}

void csi_enable(struct pxa95x_csi_dev *csi, int sci_idx)
{
	unsigned int vc, val = 0, csi_val;

	/* configure CSI source control register according to H/W data path */
	/* FIXME: virtual channel and data type interleaving is not supported
	 * yet, need more test and add code */
	vc = (csi->phy_cfg != NULL) ? csi->phy_cfg->vc : 0;
	csi_val = vc & 0x3;
	WARN_ON(csi_val != vc);
	/* 0~3 For CSI0, 4~7 for CSI1 */
	if (csi->id)
		csi_val += 4;
	/* only change the bit for attached SCI, do a read-change-write */
	val = csi_reg_read(csi, REG_CSSCR);
	switch (sci_idx) {
	case CCIC_0:
		if (cpu_is_pxa955_Ex() || cpu_is_pxa968())
			val = (val&0x0F) | (csi_val<<4);
		else
			val = (val&0xF0) | csi_val;	/* change bit0-3 */
		break;
	case CCIC_1:
		if (cpu_is_pxa955_Ex() || cpu_is_pxa968())
			val = (val&0xF0) | csi_val;
		else
			val = (val&0x0F) | (csi_val<<4);
		break;
	default:
		printk(KERN_ERR "cam: unknow SCI index: %d\n", sci_idx);
		WARN_ON(1);
	}
	csi_reg_write(csi, REG_CSSCR, val);

	/* Actually turn on the CSI to accept input */
	val = csi_reg_read(csi, REG_CSxCR0);
	val |= CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
}

void csi_disable(struct pxa95x_csi_dev *csi)
{
	unsigned int val = csi_reg_read(csi, REG_CSxCR0);

	val &= ~CSxCR0_CSIEN;
	csi_reg_write(csi, REG_CSxCR0, val);
}

/* csi_reset put CSI into reset state, must call csi_config
 * or manually set CSxPHYCAL[CAM_CSI_CONT_MIPI_RESET] later */
static void csi_reset(struct pxa95x_csi_dev *csi)
{
	int val;
	/* Clear CSxPHYCAL[CAM_CSI_CONT_MIPI_RESET] to put CSI to reset state*/
	val = csi_reg_read(csi, REG_CSxPHYCAL);
	csi_reg_write(csi, REG_CSxPHYCAL, val&(~CSI_CONT_MIPI_RESET));
}

static inline void sci_reg_write(struct pxa955_cam_dev *pcdev,
					unsigned int reg,
					unsigned int val)
{
	__raw_writel(val, pcdev->regs + reg);
}

static inline unsigned int sci_reg_read(struct pxa955_cam_dev *pcdev,
		unsigned int reg)
{
	return __raw_readl(pcdev->regs + reg);
}


static inline void sci_reg_write_mask(struct pxa955_cam_dev *pcdev,
					unsigned int reg,
					unsigned int val,
					unsigned int mask)
{
	unsigned int v = sci_reg_read(pcdev, reg);

	v = (v & ~mask) | (val & mask);
	sci_reg_write(pcdev, reg, v);
}

static inline void sci_reg_clear_bit(struct pxa955_cam_dev *pcdev,
		unsigned int reg, unsigned int val)
{
	sci_reg_write_mask(pcdev, reg, 0, val);
}

static inline void sci_reg_set_bit(struct pxa955_cam_dev *pcdev,
		unsigned int reg, unsigned int val)
{
	sci_reg_write_mask(pcdev, reg, val, val);
}

static void __attribute__((unused)) sci_dump_registers(struct pxa955_cam_dev *pcdev)
{
	printk(KERN_ERR "--------- SCI register dump --------\n");
	printk(KERN_ERR "SCICR0    | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCICR0));
	printk(KERN_ERR "SCICR1    | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCICR1));
	printk(KERN_ERR "SCISR     | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCISR));
	printk(KERN_ERR "SCIMASK   | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIMASK));
	printk(KERN_ERR "SCIFIFO   | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIFIFO));
	printk(KERN_ERR "SCIFIFOSR | 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIFIFOSR));

	printk(KERN_ERR "               CH_0       CH_1       CH_2   \n");
	printk(KERN_ERR "SCIDADDRx | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIDADDR0), \
		sci_reg_read(pcdev, REG_SCIDADDR1), \
		sci_reg_read(pcdev, REG_SCIDADDR2));
	printk(KERN_ERR "SCISADDRx | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCISADDR0), \
		sci_reg_read(pcdev, REG_SCISADDR1), \
		sci_reg_read(pcdev, REG_SCISADDR2));
	printk(KERN_ERR "SCITADDRx | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCITADDR0), \
		sci_reg_read(pcdev, REG_SCITADDR1), \
		sci_reg_read(pcdev, REG_SCITADDR2));
	printk(KERN_ERR "SCIDCMDx  | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIDCMD0), \
		sci_reg_read(pcdev, REG_SCIDCMD1), \
		sci_reg_read(pcdev, REG_SCIDCMD2));
	printk(KERN_ERR "SCIDBRx   | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIDBR0), \
		sci_reg_read(pcdev, REG_SCIDBR1), \
		sci_reg_read(pcdev, REG_SCIDBR2));
	printk(KERN_ERR "SCIDCSR0  | 0x%08X 0x%08X 0x%08X\n", \
		sci_reg_read(pcdev, REG_SCIDCSR0), \
		sci_reg_read(pcdev, REG_SCIDCSR1), \
		sci_reg_read(pcdev, REG_SCIDCSR2));
	printk(KERN_ERR "---------------- end ----------------\n");
}

static void __attribute__((unused)) dma_dump_desc(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node;
	int i, k;
	char VB_STATE[VB2_BUF_STATE_ERROR+1][16] = {
		[VB2_BUF_STATE_DEQUEUED]	= "VB2_OUT",
		[VB2_BUF_STATE_QUEUED]		= "VB2_VBQ",
		[VB2_BUF_STATE_ACTIVE]		= "VB2_ACT",
		[VB2_BUF_STATE_DONE]		= "VB2_DONE",
		[VB2_BUF_STATE_ERROR]		= "VB2_ERROR",
	};

	printk(KERN_ERR "********** cam: brief dma list dump **********\n");
	list_for_each_entry(buf_node, &pcdev->dma_chain, hook) {
		printk(KERN_ERR "buf:%08X, sg|%08X->DDADR|%08X, "\
				"DTADR[%08X-%08X] <%s>",\
			(__u32)buf_node, (__u32)buf_node->dma_desc[0].sg_dma,\
			(__u32)buf_node->dma_desc[0].sg_cpu[0].ddadr,
			(__u32)buf_node->dma_desc[0].sg_cpu[0].dtadr,\
			(__u32)buf_node->dma_desc[0].sg_cpu[0].dtadr + \
			buf_node->dma_desc[0].sg_cpu[0].dcmd,\
			VB_STATE[buf_node->vb2.state]);
		i = sci_reg_read(pcdev, REG_SCITADDR0);
		k = buf_node->dma_desc[0].sg_cpu[0].dtadr;
		if ((i >= k) && (i < k+pcdev->channel_size[0]))
			printk(" *\n");
		else
			printk("\n");
	}
	printk(KERN_ERR "********** cam: brief new list dump **********\n");
	list_for_each_entry(buf_node, &pcdev->new_chain, hook) {
		printk(KERN_ERR "buf:%08X, sg|%08X->DDADR|%08X, "\
				"DTADR[%08X-%08X] <%s>",\
			(__u32)buf_node, (__u32)buf_node->dma_desc[0].sg_dma,\
			(__u32)buf_node->dma_desc[0].sg_cpu[0].ddadr,
			(__u32)buf_node->dma_desc[0].sg_cpu[0].dtadr,\
			(__u32)buf_node->dma_desc[0].sg_cpu[0].dtadr + \
			buf_node->dma_desc[0].sg_cpu[0].dcmd,\
			VB_STATE[buf_node->vb2.state]);
		i = sci_reg_read(pcdev, REG_SCITADDR0);
		k = buf_node->dma_desc[0].sg_cpu[0].dtadr;
		if ((i>=k) && (i<k+pcdev->channel_size[0]))
			printk(" *\n");
		else
			printk("\n");
	}
	printk(KERN_ERR "*********************************************\n\n");
}

static void __attribute__((unused)) dma_dump_buf_list(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node;
	dma_addr_t dma_handles;

	printk(KERN_ERR "cam: dump_dma_chain ************+\n");
	list_for_each_entry(buf_node, &pcdev->dma_chain, hook) {
		dma_handles = vb2_dma_contig_plane_paddr(&buf_node->vb2, 0);
		printk(KERN_ERR "cam: buf_node 0x%08X, pa 0x%08X\n",
			(unsigned int)buf_node, dma_handles);
	}
	printk(KERN_ERR "cam: dump_dma_chain ************-\n\n");
}

static void dma_append_desc(struct pxa955_cam_dev *pcdev,
				struct pxa_buf_node *pre,
				struct pxa_buf_node *next)
{
	int i = 0;
	struct pxa_cam_dma *pre_dma = NULL, *next_dma = NULL;

	for (i = 0; i < pcdev->channels; i++) {
		pre_dma = &pre->dma_desc[i];
		next_dma = &next->dma_desc[i];
		pre_dma->sg_cpu[pre_dma->sglen-1].ddadr = (u32)next_dma->sg_dma;
	}
	printk(KERN_DEBUG "cam: append new dma 0x%08X to 0x%08X\n",
		next_dma->sg_dma, pre_dma->sg_dma);
}

/* only handle in irq context*/
static void dma_fetch_frame(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *buf_node = NULL;
	dma_addr_t dma_handles;

	spin_lock(&pcdev->spin_lock);

	if (pcdev->dma_bufs > 1) {
		/*
		* get the first node of dma_list, it must have been filled by dma, and
		* remove it from dma-buf-list.
		*/
		buf_node = list_entry(pcdev->dma_chain.next,
						struct pxa_buf_node, hook);
		/* detach from HW list */
		dma_append_desc(pcdev, buf_node, buf_node);
		pcdev->dma_bufs--;
		list_del_init(&buf_node->hook);
		spin_unlock(&pcdev->spin_lock);
		dma_handles = vb2_dma_contig_plane_paddr(&buf_node->vb2, 0);
		dma_unmap_page(pcdev->soc_host.v4l2_dev.dev,
				dma_handles,
				vb2_get_plane_payload(&buf_node->vb2, 0),
				DMA_FROM_DEVICE);
		vb2_buffer_done(&buf_node->vb2, VB2_BUF_STATE_DONE);
	} else {
		spin_unlock(&pcdev->spin_lock);
		/*if there is only one left in dma_list, drop it!*/
		printk(KERN_DEBUG "cam: drop a frame!\n");
	}
}

/* move all entry on list B to the tail of list A */
static inline void list_merge(struct list_head *a, struct list_head *b)
{
	if (!a || !b || list_empty(b))
		return;
	a->prev->next = b->next;
	b->next->prev = a->prev;
	b->prev->next = a;
	a->prev = b->prev;
	INIT_LIST_HEAD(b);
}

static void dma_attach_bufs(struct pxa955_cam_dev *pcdev)
{
	struct pxa_buf_node *dma_tail, *buf_node;
	unsigned int regval;

	spin_lock(&pcdev->spin_lock);

	buf_node = list_entry(pcdev->new_chain.next, struct pxa_buf_node, hook);
	dma_tail = list_entry(pcdev->dma_chain.prev,
						struct pxa_buf_node, hook);
	if (list_empty(&pcdev->new_chain)) {
		/* If no new buffer to add, right time to get out */
		spin_unlock(&pcdev->spin_lock);
		return;
	}

	list_merge(&pcdev->dma_chain, &pcdev->new_chain);
	pcdev->dma_bufs += pcdev->new_bufs;
	pcdev->new_bufs = 0;
	dma_append_desc(pcdev, dma_tail, buf_node);
	spin_unlock(&pcdev->spin_lock);

	/* DDADR is prefetched upon loading the containing descriptor, so the
	 * new buffer will not be run over until another IRQ triggers loading
	 * the DDADR again. So if expect the new buffer be DMAed on ASAP, DMA
	 * branch is necessary */
	/*
	* NOTE!!! only one desc for one frame buffer, if not in this way,
	* need change here, as phy addr might not located between the
	* begin and end, phy addr might not continuous between different
	* desc of one frame buffer.
	*/
	regval = sci_reg_read(pcdev, REG_SCITADDR0);
	if (((regval >= vb2_dma_contig_plane_paddr(&dma_tail->vb2, 0))
		&& (regval < (vb2_dma_contig_plane_paddr(&dma_tail->vb2, 0)
		+ pcdev->channel_size[0])))) {
		/*
		* if we find DMA is looping in the last buf, and there is new
		* coming buf, (DMA target address shows that DMA is working in
		* the tail buffer) we SHOULD set DMA branch reg, force DMA move
		* to the new buf descriptor in the next frame, so we can pick up
		* this buf when next irq comes.
		*/
		sci_reg_write(pcdev, REG_SCIDBR0, \
				(buf_node->dma_desc[0].sg_dma | SCIDBR_EN));
		if (pcdev->channels == 3) {
			sci_reg_write(pcdev, REG_SCIDBR1, \
				(buf_node->dma_desc[1].sg_dma | SCIDBR_EN));
			sci_reg_write(pcdev, REG_SCIDBR2, \
				(buf_node->dma_desc[2].sg_dma | SCIDBR_EN));
		}
	}
}

static int dma_alloc_desc(struct pxa_buf_node *buf_node,
					struct pxa955_cam_dev *pcdev)
{
	int i;
	unsigned int len = 0, len_tmp = 0;
	pxa_dma_desc *dma_desc_tmp;
	unsigned long dma_desc_phy_tmp;
	unsigned long srcphyaddr, dstphyaddr;
	struct pxa_cam_dma *desc;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;
	srcphyaddr = 0;	/* TBD */

	dstphyaddr = vb2_dma_contig_plane_paddr(&buf_node->vb2, 0);

	for (i = 0; i < pcdev->channels; i++) {
		printk(KERN_DEBUG "cam: index %d, channels %d\n", \
					buf_node->vb2.v4l2_buf.index, i);
		desc = &buf_node->dma_desc[yuv_output_sequence[pcdev->sci_pixfmt][i]];
		len = pcdev->channel_size[yuv_output_sequence[pcdev->sci_pixfmt][i]];

		desc->sglen = (len + SINGLE_DESC_TRANS_MAX - 1) / \
				SINGLE_DESC_TRANS_MAX;
		desc->sg_size = (desc->sglen) * sizeof(struct pxa_dma_desc);

		if (desc->sg_cpu == NULL) {
			desc->sg_cpu = dma_alloc_coherent(dev, desc->sg_size,
					     &desc->sg_dma, GFP_KERNEL);
		}
		printk(KERN_DEBUG "cam: sglen %d, size %d, sg_cpu 0x%08X\n",
			desc->sglen, desc->sg_size, (unsigned int)desc->sg_cpu);
		if (!desc->sg_cpu) {
			printk(KERN_ERR "cam: dma_alloc_coherent "\
					"failed at chnnl %d!\n", i);
			goto err;
		}

		dma_desc_tmp = desc->sg_cpu;
		dma_desc_phy_tmp = desc->sg_dma;

		while (len) {
			len_tmp = len > SINGLE_DESC_TRANS_MAX ? \
				SINGLE_DESC_TRANS_MAX : len;

			if ((dstphyaddr & 0xf) != 0) {
				printk(KERN_ERR "cam: error: at least " \
					"we need 16bytes align for DMA!\n");
				goto err;
			}
			dma_desc_tmp->ddadr = dma_desc_phy_tmp + \
						sizeof(pxa_dma_desc);
			dma_desc_tmp->dsadr = srcphyaddr; /* TBD */
			dma_desc_tmp->dtadr = dstphyaddr;
			dma_desc_tmp->dcmd = len_tmp;

			len -= len_tmp;
			dma_desc_tmp++;
			dma_desc_phy_tmp += sizeof(pxa_dma_desc);
			dstphyaddr += len_tmp;

		}
	}
	return 0;

err:
	for (i = 0; i < pcdev->channels; i++) {
		desc = &buf_node->dma_desc[i];
		if (desc->sg_cpu) {
			dma_free_coherent(dev, desc->sg_size,
				    desc->sg_cpu, desc->sg_dma);
			desc->sg_cpu = 0;
		}
	}
	return -ENOMEM;
}

static void dma_free_desc(struct pxa_buf_node *buf_node,
				struct pxa955_cam_dev *pcdev)
{
	int i;
	struct pxa_cam_dma *desc;
	struct device *dev = pcdev->soc_host.v4l2_dev.dev;

	for (i = 0; i < pcdev->channels; i++) {
		desc = &buf_node->dma_desc[i];
		if (desc->sg_cpu) {
			dma_free_coherent(dev, desc->sg_size,
				    desc->sg_cpu, desc->sg_dma);
			desc->sg_cpu = 0;
		}
	}
}

static void dma_free_bufs(struct pxa_buf_node *buf,
				struct pxa955_cam_dev *pcdev)
{
	dma_free_desc(buf, pcdev);
}

static int dma_chain_init(struct pxa955_cam_dev *pcdev)
{
	int i, ret = 0;
	struct pxa_cam_dma *desc;
	struct pxa_buf_node *buf_node, *dma_tail = NULL;

	spin_lock(&pcdev->spin_lock);
	if (!list_empty(&pcdev->dma_chain))
		dma_tail = list_entry(pcdev->dma_chain.prev,
						struct pxa_buf_node, hook);
	/* If at least one buffer to add */
	if (!list_empty(&pcdev->new_chain)) {
		buf_node = list_entry(pcdev->new_chain.next,
						struct pxa_buf_node, hook);
		if (dma_tail != NULL)
			dma_append_desc(pcdev, dma_tail, buf_node);
		list_merge(&pcdev->dma_chain, &pcdev->new_chain);
		pcdev->dma_bufs += pcdev->new_bufs;
		pcdev->new_bufs = 0;
	}
	/* If no buffer on DMA chain at all */
	if (unlikely(pcdev->dma_bufs == 0)) {
		printk(KERN_ERR "cam: dma chain is empty\n");
		ret = -EPERM;
		WARN_ON(1);
		goto unlock;
	}
	/* re-program DADDR with DMA chain head desc addr */
	buf_node = list_entry(pcdev->dma_chain.next, struct pxa_buf_node, hook);
	for (i = 0; i < pcdev->channels; i++) {
		desc = &(buf_node->dma_desc[i]);
		sci_reg_write(pcdev, REG_SCIDADDR0 + i*0x10, desc->sg_dma);
	}
unlock:
	spin_unlock(&pcdev->spin_lock);
	return ret;
}

static int sci_cken(struct pxa955_cam_dev *pcdev, int flag)
{
	if (flag) {
		clk_enable(pcdev->sci1_clk);
		clk_enable(pcdev->sci2_clk);
	} else {
		clk_disable(pcdev->sci2_clk);
		clk_disable(pcdev->sci1_clk);
	};

	return 0;
}

static void sci_s_fmt(struct pxa955_cam_dev *pcdev,
				struct v4l2_pix_format *fmt)
{
	unsigned int size = fmt->width*fmt->height;

	pcdev->sci_pixfmt = SCI_PIXFMT_DEFAULT;

	switch (fmt->pixelformat) {

	case V4L2_PIX_FMT_RGB565:
		pcdev->channels = 1;
		pcdev->channel_size[0] = size*2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_RGB565) | \
						SCICR1_FMT_OUT(FMT_RGB565));
	    break;

	case V4L2_PIX_FMT_JPEG:
		pcdev->channels = 1;
		/*
		* From the specification, the DMA transfer size in SCI DMA
		* command reg should not exceed 16MB.
		* Actually the size of compressed JPEG picture is variable.
		* And the DMA transfer size should be set as below:
		* width*height/JPEG_COMPRESS_RATIO_HIGH <= size <= 16MB
		* So we set it to width*height*2 since assume the JPEG size
		* for a resolution should be smaller than RGB/YUV size.
		*/
		pcdev->channel_size[0] = size*2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_JPEG) | \
						SCICR1_FMT_OUT(FMT_JPEG));
	    break;

	case V4L2_PIX_FMT_YUV422P:
		pcdev->channels = 3;
		pcdev->channel_size[0] = size;
		pcdev->channel_size[1] = pcdev->channel_size[2] = size/2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | \
						SCICR1_FMT_OUT(FMT_YUV422));
		break;

	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		pcdev->channels = 1;
		pcdev->channel_size[0] = size*2;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | \
					SCICR1_FMT_OUT(FMT_YUV422PACKET));
		break;

	case V4L2_PIX_FMT_YVU420:
		pcdev->sci_pixfmt = SCI_PIXFMT_YVU;
	case V4L2_PIX_FMT_YUV420:
		/* only accept YUV422 as input */
		pcdev->channels = 3;
		pcdev->channel_size[0] = size;
		pcdev->channel_size[1] = pcdev->channel_size[2] = size/4;
		sci_reg_write(pcdev, REG_SCICR1, SCICR1_FMT_IN(FMT_YUV422) | \
						SCICR1_FMT_OUT(FMT_YUV420));
		break;

	default:
		printk(KERN_ERR "cam: error can not support fmt!\n");
		break;
	}

}

static void sci_irq_enable(struct pxa955_cam_dev *pcdev, unsigned int val)
{
	sci_reg_write(pcdev, REG_SCISR, sci_reg_read(pcdev, REG_SCISR));
	sci_reg_clear_bit(pcdev, REG_SCIMASK, val);
}

static void sci_irq_disable(struct pxa955_cam_dev *pcdev, unsigned int val)
{
	sci_reg_set_bit(pcdev, REG_SCIMASK, val);
}

/*
 * Make the controller start grabbing images.  Everything must
 * be set up before doing this.
 */
static void sci_enable(struct pxa955_cam_dev *pcdev)
{
	int i = 0;
	unsigned int val = 0;

	/* start_fifo */
	for (i = 0; i < pcdev->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		sci_reg_set_bit(pcdev, REG_SCIFIFO, val);
	}
	if (cpu_is_pxa978())
		sci_reg_set_bit(pcdev, REG_SCIFIFO, SCIFIFO_TFS_64);

	/* start_dma */
	for (i = 0; i < pcdev->channels; i++)
		sci_reg_set_bit(pcdev, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);

	/* start sci */
	sci_reg_set_bit(pcdev, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);
}

static void sci_disable(struct pxa955_cam_dev *pcdev)
{
	int i = 0;
	unsigned int val = 0;

	/* stop_fifo */
	for (i = 0; i < pcdev->channels; i++) {
		val = SCIFIFO_F0_EN << i;
		sci_reg_clear_bit(pcdev, REG_SCIFIFO, val);
	}

	/* stop_dma */
	for (i = 0; i < pcdev->channels; i++) {
		sci_reg_clear_bit(pcdev, REG_SCIDCSR0 + i*0x10, SCIDCSR_DMA_RUN);
		sci_reg_clear_bit(pcdev, REG_SCIDBR0 + i*0x10, SCIDBR_EN);
	}
	/* stop sci */
	sci_reg_clear_bit(pcdev, REG_SCICR0, SCICR0_CAP_EN | SCICR0_CI_EN);
}

void sci_init(struct pxa955_cam_dev *pcdev)
{
	/*
	* Turn off the enable bit.  It sure should be off anyway,
	* but it's good to be sure.
	*/
	sci_reg_clear_bit(pcdev, REG_SCICR0, SCICR0_CI_EN);

	/* Mask all interrupts.*/
	sci_reg_write(pcdev, REG_SCIMASK, ~0);
}

/* The function will disable some PP to guarantee camera CSI/DMA works fine,
 * and ONLY guarantee SCI/DMA is good, if display/encode needs more constrain
 * it's not camera driver's concern */
void cam_set_constrain(struct pxa955_cam_dev *cam, int dev_idx)
{
	switch (cam->state) {
	case CAM_STATE_CLOSE:
		/* video device closed, allow all ops */
		dvfm_enable_op_name("D2", dev_idx);
		dvfm_enable_op_name("D1", dev_idx);
		dvfm_enable_op_name("CG", dev_idx);
		dvfm_enable_op_name("156M", dev_idx);
		dvfm_enable_op_name("312M", dev_idx);
		dvfm_enable_op_name("624M", dev_idx);
		dvfm_enable_op_name("806M", dev_idx);
		dvfm_enable_op_name("1014M", dev_idx);
		break;

	case CAM_STATE_OPEN:
		wrr_nor_sv = *wrr_nor;	/* backup original value */
		wrr_fst_sv = *wrr_fst;
	case CAM_STATE_FORMATED:
		/* video device opened, but not stream-on */
		dvfm_disable_op_name("CG", dev_idx);
		dvfm_disable_op_name("D1", dev_idx);
		dvfm_disable_op_name("D2", dev_idx);
		dvfm_enable_op_name("156M", dev_idx);
		dvfm_enable_op_name("312M", dev_idx);
		dvfm_enable_op_name("624M", dev_idx);
		dvfm_enable_op_name("806M", dev_idx);
		dvfm_enable_op_name("1014M", dev_idx);
#ifdef _ARB_CHANGE_
			*arb_sch |= 0x100;	/* set XPAGE_EN */
			*wrr_nor = wrr_nor_sv;
			*wrr_fst = wrr_fst_sv;
			printk(KERN_INFO "cam: MC_WRR recovered\n");
#endif
		break;

	case CAM_STATE_STREAMING:
		switch (cam->icd->user_width) {
		case 176:	/* QCIF */
		case 352:	/* CIF */
		case 320:	/* QVGA */
		case 640:	/* VGA */
		case 800:	/* WVGA */
			/* No constrain needed for cam driver */
			break;
		case 1280:
			/* 720P used at least 182MHz MIPI clock
			 * test shows 312M PP is OK with it */
			dvfm_disable_op_name("156M", dev_idx);
#ifdef _ARB_CHANGE_
			/* W/R for pxa978 D0 silicon issue */
			*arb_sch &= 0xFFFFFEFF;	/* clear XPAGE_EN */
			wrr_nor_sv = *wrr_nor;	/* backup original value */
			wrr_fst_sv = *wrr_fst;
			*wrr_nor = 0x050F0505;	/* CP:APP:GC:CORE */
			*wrr_fst = 0x050F0505;	/* CP:APP:GC:CORE */
			printk(KERN_INFO "cam: MC_WRR changed to %08X\n", \
				*wrr_nor);
#endif
			break;
		case 1920:
			/* 1080p used at least 300M MIPI clock, so 806Mhz is
			 * the minimum according to PP table. But actually
			 * 624Mhz also seems OK at 364M MIPI clock */
			dvfm_disable_op_name("312M", dev_idx);
			dvfm_disable_op_name("156M", dev_idx);
#ifdef _ARB_CHANGE_
			/* W/R for pxa978 D0 silicon issue */
			*arb_sch &= 0xFFFFFEFF;	/* clear XPAGE_EN */
			wrr_nor_sv = *wrr_nor;	/* backup original value */
			wrr_fst_sv = *wrr_fst;
			*wrr_nor = 0x050F0505;	/* CP:APP:GC:CORE */
			*wrr_fst = 0x050F0505;	/* CP:APP:GC:CORE */
			printk(KERN_INFO "cam: MC_WRR changed to %08X\n", \
				*wrr_nor);
#endif
			break;
		};
		break;
	default:
		BUG_ON(1);
	};
}

static unsigned long uva_to_pa(unsigned long addr, struct page **page)
{
	unsigned long ret = 0UL;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;

	pgd = pgd_offset(current->mm, addr);
	if (!pgd_none(*pgd)) {
		pud = pud_offset(pgd, addr);
		if (!pud_none(*pud)) {
			pmd = pmd_offset(pud, addr);
			if (!pmd_none(*pmd)) {
				pte = pte_offset_map(pmd, addr);
				if (!pte_none(*pte) && pte_present(*pte)) {
					(*page) = pte_page(*pte);
					ret = page_to_phys(*page);
					ret |= (addr & (PAGE_SIZE-1));
				}
			}
		}
	}
	return ret;
}

struct page *va_to_page(unsigned long user_addr)
{
	struct page *page = NULL;
	unsigned int vaddr = PAGE_ALIGN(user_addr);

	if (uva_to_pa(vaddr, &page) != 0)
		return page;

	return 0;
}
unsigned long va_to_pa(unsigned long user_addr, unsigned int size)
{
	unsigned long  paddr, paddr_tmp;
	unsigned long  size_tmp = 0;
	struct page *page = NULL;
	int page_num = PAGE_ALIGN(size) / PAGE_SIZE;
	unsigned int vaddr = PAGE_ALIGN(user_addr);
	int i = 0;

	if (vaddr == 0)
		return 0;

	paddr = uva_to_pa(vaddr, &page);

	for (i = 0; i < page_num; i++) {
		paddr_tmp = uva_to_pa(vaddr, &page);
		if ((paddr_tmp - paddr) != size_tmp)
			return 0;
		vaddr += PAGE_SIZE;
		size_tmp += PAGE_SIZE;
	}
	return paddr;
}

static int pxa97x_vb2_setup(struct vb2_queue *vq, unsigned int *count,
			   unsigned int *num_planes, unsigned long sizes[],
			   void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
					struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	int bytes_per_line = soc_mbus_bytes_per_line(icd->user_width,
		icd->current_fmt->host_fmt);

	int minbufs = 2;
	if (*count < minbufs)
		*count = minbufs;

	if (bytes_per_line < 0)
		return bytes_per_line;

	*num_planes = 1;
	sizes[0] = ALIGN(bytes_per_line * icd->user_height, ALIGN_SIZE);
	alloc_ctxs[0] = pcdev->alloc_ctx;

	if (!list_empty(&pcdev->dma_chain)) {
		printk(KERN_ERR "cam: dma list is not empty, forgot to clean it?\n");
		return -EPERM;
	}
	pcdev->dma_bufs = 0;
	INIT_LIST_HEAD(&pcdev->new_chain);
	pcdev->new_bufs = 0;
	pcdev->vq = vq;
	return 0;
}

static int pxa97x_vb2_prepare(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
					struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct pxa_buf_node *buf = container_of(vb, struct pxa_buf_node, vb2);
	unsigned long size;
	dma_addr_t paddr;
	struct pxa_cam_dma *desc;
	int ret = 0;

	size = vb2_plane_size(vb, 0);
	/* FIXME:	if YUV, must assert(W*H*bpp(plane)<=size)
	 *		if JPEG, assert size is not too small*/
	vb2_set_plane_payload(vb, 0, size);

	paddr = vb2_dma_contig_plane_paddr(vb, 0);
	if (paddr == (dma_addr_t)NULL)
		return -EPERM;

	if (unlikely(!IS_ALIGNED(size, ALIGN_MASK))) {
		printk(KERN_ERR "cam: buffer size 0x%08X is not 32 aligned\n",
				paddr);
		vb->state = VB2_BUF_STATE_ERROR;
		return -EPERM;
	}
	if (unlikely(!IS_ALIGNED((int)paddr, ALIGN_MASK))) {
		printk(KERN_ERR "cam: buffer addr 0x%08X is not 32 aligned\n",
				paddr);
		vb->state = VB2_BUF_STATE_ERROR;
		return -EPERM;
	}

	desc = &buf->dma_desc[0];
	if (desc->sg_cpu == NULL) {
		ret = dma_alloc_desc(buf, pcdev);
		if (ret < 0)
			goto exit;
	}
	INIT_LIST_HEAD(&buf->hook);
	dma_append_desc(pcdev, buf, buf);
exit:
	return ret;
}

static void pxa97x_vb2_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
					struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct pxa_buf_node *buf = container_of(vb, struct pxa_buf_node, vb2);
	struct pxa_buf_node *tail;
	unsigned long flag;

	/* Between two IRQ, more than one buffer may be pushed into driver,
	 * they are added to the new buffer list temporarily, and append to
	 * HW DMA list together when IRQ comes. Thus, qbuf thread will touch
	 * new_chain, DMA thread will touch both new_chain and dma_chain*/

	/* About to touch new buffer list, must prevent IRQ from touching it */
	spin_lock_irqsave(&pcdev->spin_lock, flag);
	if (pcdev->new_bufs) {
		tail = list_entry(pcdev->new_chain.prev, \
					struct pxa_buf_node, hook);
		dma_append_desc(pcdev, tail, buf);
	}
	list_add_tail(&buf->hook, &pcdev->new_chain);
	pcdev->new_bufs++;
	spin_unlock_irqrestore(&pcdev->spin_lock, flag);

	/* Invalidate the buffer before add to DMA chain */
	dma_map_page(pcdev->soc_host.v4l2_dev.dev,
			va_to_page(vb->v4l2_planes[0].m.userptr),
			0,
			vb2_get_plane_payload(vb, 0),
			DMA_FROM_DEVICE);
}

static void pxa97x_vb2_cleanup(struct vb2_buffer *vb)
{
	struct pxa_buf_node *buf = container_of(vb, struct pxa_buf_node, vb2);
	struct soc_camera_device *icd = vb2_get_drv_priv(vb->vb2_queue);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;

	/* By this time, reqbufs(0) is called, vb won't live for long*/
	INIT_LIST_HEAD(&pcdev->dma_chain);
	INIT_LIST_HEAD(&pcdev->new_chain);
	pcdev->dma_bufs = pcdev->new_bufs = 0;

	dma_free_bufs(buf, pcdev);
}

static int pxa97x_vb2_streamon(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd = vb2_get_drv_priv(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	int ret = 0;

	if (unlikely(count == 0)) {
		printk(KERN_ERR "cam: can't streamon without vb in driver\n");
		return -EPERM;
	}
	BUG_ON(count != pcdev->new_bufs + pcdev->dma_bufs);

	/* suppose IRQ is not enabled at this time, so actually it's safe to
	 * touch dma_chain without holding the lock */
	ret = dma_chain_init(pcdev);
	if (unlikely(ret < 0))
		return ret;

	pcdev->state = CAM_STATE_STREAMING;
	cam_set_constrain(pcdev, dvfm_dev_idx);

	sci_irq_enable(pcdev, IRQ_EOFX|IRQ_OFO);
	csi_dphy(pcdev->csidev);
	/* configure enable which camera interface controller*/
	sci_enable(pcdev);
	csi_enable(pcdev->csidev, icd->iface);

#ifdef _CONTROLLER_DEADLOOP_RESET_
	mod_timer(&pcdev->reset_timer, jiffies + MIPI_RESET_TIMEOUT);
	pcdev->killing_reset_timer = 0;
#endif
	return ret;
}

static int pxa97x_vb2_streamoff(struct vb2_queue *q)
{
	struct soc_camera_device *icd = vb2_get_drv_priv(q);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;

	csi_disable(pcdev->csidev);
	sci_irq_disable(pcdev, IRQ_EOFX|IRQ_OFO);
	sci_disable(pcdev);

#ifdef _CONTROLLER_DEADLOOP_RESET_
	/* Announce reset timer is being killed */
	pcdev->killing_reset_timer = 1;
	del_timer(&pcdev->reset_timer);
#endif
	pcdev->state = CAM_STATE_FORMATED;
	cam_set_constrain(pcdev, dvfm_dev_idx);
	return 0;
}

static struct vb2_ops pxa97x_vb2_ops = {
	.queue_setup		= pxa97x_vb2_setup,
	.buf_prepare		= pxa97x_vb2_prepare,
	.buf_queue		= pxa97x_vb2_queue,
	.buf_cleanup		= pxa97x_vb2_cleanup,
	.start_streaming	= pxa97x_vb2_streamon,
	.stop_streaming		= pxa97x_vb2_streamoff,
	.wait_prepare		= soc_camera_unlock,
	.wait_finish		= soc_camera_lock,
};

static int pxa97x_cam_init_videobuf2(struct vb2_queue *q,
				   struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_USERPTR;
	q->drv_priv = icd;
	q->ops = &pxa97x_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	/* vb2_buffer is enclosed inside pxa_buf_node */
	q->buf_struct_size = sizeof(struct pxa_buf_node);
	pcdev->alloc_ctx = vb2_dma_contig_init_ctx(ici->v4l2_dev.dev);

	return vb2_queue_init(q);
}

static irqreturn_t csi_irq(int irq, void *data);

static int pxa955_cam_add_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct sensor_platform_data *pdata = (void *)icl->priv;
	int ret = 0;

	if (pcdev->icd)
		return -EBUSY;

	pcdev->icd = icd;

	pcdev->state = CAM_STATE_OPEN;
	if (cpu_is_pxa978_Dx())
		cam_set_constrain(pcdev, dvfm_dev_idx);
	else {	/* Disable 624M PP for NEVO and 1Ghz for MGx */
		if (cpu_is_pxa978())
			dvfm_disable_op_name("624M", dvfm_dev_idx);
		else
			dvfm_disable_op_name("988M", dvfm_dev_idx);
		/* Disable OPs lower than 624 */
		dvfm_disable(dvfm_dev_idx);
#ifdef _ARB_CHANGE_
		*pri_axi = 1;
		*pri_gcu = 2;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk(KERN_NOTICE "cam: change AXI = 0x%X, GCU = 0x%X, "\
			"CI1 = 0x%X, CI2 = 0x%X\n", \
			*pri_axi, *pri_gcu, *pri_ci1, *pri_ci2);
#endif
	}

	/* Start initialize of CSI to which sensor is attached */
	/* Assume SCI can be connected to only one CSI at the same time*/
	if (unlikely(pcdev->csidev != NULL)) {
		printk(KERN_ERR "cam: sci: this device is already connected " \
				"to a CSI device\n");
		return -EBUSY;
	}
	/* Get CSI info from iclink */
	pcdev->csidev = kzalloc(sizeof(struct pxa95x_csi_dev), GFP_KERNEL);
	if (pcdev->csidev == NULL)
		return -ENOMEM;
	memcpy(pcdev->csidev, pdata->csi_ctlr, sizeof(struct pxa95x_csi_dev));
	/* Now handle the resource */
	/* 1st, registers' VA */
	pcdev->csidev->regs = ioremap(pcdev->csidev->reg_start, SZ_4K);
	if (pcdev->csidev->regs == NULL) {
		printk(KERN_ERR "cam: sci: unable to ioremap pxa95x-camera csi regs\n");
		ret = -ENOMEM;
		goto exit_csi;
	}
	/* 2nd, irq ISR */
	ret = request_irq(pcdev->csidev->irq_num, csi_irq, 0, \
				PXA955_CAM_DRV_NAME, pcdev);
	if (ret) {
		printk(KERN_ERR "cam: sci: unable to create csi irq\n");
		goto exit_csi_reg;
	}
	spin_lock_init(&pcdev->csidev->dev_lock);
	/* 3rd, clocks for controller*/
	pcdev->csidev->csi_tx_esc = clk_get(NULL, "CSI_TX_ESC");
	if (!pcdev->csidev->csi_tx_esc) {
		printk(KERN_ERR "cam: unable to get CSI_TX_ESC\n");
		goto exit_csi_irq;
	};
	/* Now all done! */

	csi_cken(pcdev->csidev, 1);
	sci_cken(pcdev, 1);
	sci_init(pcdev);
	csi_clkdiv(pcdev->csidev);

	ret = v4l2_subdev_call(sd, core, init, 0);
	if ((ret < 0) && (ret != -ENOIOCTLCMD))
		dev_info(icd->dev.parent, "cam: Failed to initialize subdev: "\
					"%d\n", ret);

	printk(KERN_INFO "cam: path ready %s => CSI#%d => SCI#%d => " \
			"/dev/video%d\n", icl->module_name, pcdev->csidev->id, \
			icd->iface, pcdev->soc_host.nr);
	return 0;

exit_csi_irq:
	free_irq(pcdev->csidev->irq_num, pcdev);
exit_csi_reg:
	iounmap(pcdev->csidev->regs);
exit_csi:
	kfree(pcdev->csidev);
	return ret;
}

static void pxa955_cam_remove_device(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct vb2_queue *vq = pcdev->vq;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_mbus_framefmt mf;
	int i;

	BUG_ON(icd != pcdev->icd);

#ifdef _CONTROLLER_DEADLOOP_RESET_
		/* Announce reset timer is being killed */
		pcdev->killing_reset_timer = 1;
		del_timer(&pcdev->reset_timer);
#endif

	/* actually, following code should never be taken, coz vb2 did it before
	 * ici::remove is called */
	if ((vq) && (vq->streaming != 0)) {
		struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
		printk(KERN_WARNING "cam: closeing device without " \
				"stream off\n");
		vb2_streamoff(vq, vq->type);
		v4l2_subdev_call(sd, video, s_stream, 0);
	}

	sci_cken(pcdev, 0);
	csi_cken(pcdev->csidev, 0);

	if (!v4l2_subdev_call(sd, video, g_mbus_fmt, &mf)) {
		for (i = 0; i < icd->num_user_formats; i++) {
			if (icd->user_formats[i].code == mf.code) {
				icd->current_fmt	= icd->user_formats + i;
				icd->user_width		= mf.width;
				icd->user_height	= mf.height;
				icd->colorspace		= mf.colorspace;
				icd->field		= mf.field;
			};
		}
	}

	pcdev->state = CAM_STATE_CLOSE;
	if (cpu_is_pxa978_Dx())
		cam_set_constrain(pcdev, dvfm_dev_idx);
	else {
#ifdef _ARB_CHANGE_
		*pri_axi = 0;
		*pri_gcu = 0;
		*pri_ci1 = 0;
		*pri_ci2 = 0;
		printk(KERN_NOTICE "CI AXI Fabric RR Arbitration recovered\n");
#endif
		dvfm_enable(dvfm_dev_idx);
		if (cpu_is_pxa978())
			dvfm_enable_op_name("624M", dvfm_dev_idx);
		else
			dvfm_enable_op_name("988M", dvfm_dev_idx);
	}
	pcdev->icd = NULL;

	/* Remove info for connected CSI */
	i = pcdev->csidev->id;
	clk_put(pcdev->csidev->csi_tx_esc);
	free_irq(pcdev->csidev->irq_num, pcdev);
	iounmap(pcdev->csidev->regs);
	kfree(pcdev->csidev);
	pcdev->csidev = NULL;
	/* Now CSI is totally dead..This is its last words */
	printk(KERN_INFO "cam: path free: %s => CSI#%d => SCI#%d\n", \
			icl->module_name, i, icd->iface);
}

static const struct soc_mbus_pixelfmt pxa955_camera_formats[] = {
	{
		.fourcc			= V4L2_PIX_FMT_YUV422P,
		.name			= "YUV422P",
		.bits_per_sample	= 8,
		.packing		= SOC_MBUS_PACKING_2X8_PADLO,
		.order			= SOC_MBUS_ORDER_LE,
	}, {
		.fourcc			= V4L2_PIX_FMT_YUV420,
		.name			= "YUV420P",
		.bits_per_sample	= 12,
		.packing		= SOC_MBUS_PACKING_1_5X8,
		.order			= SOC_MBUS_ORDER_LE,
	}, {
		.fourcc			= V4L2_PIX_FMT_YVU420,
		.name			= "YVU420P",
		.bits_per_sample	= 12,
		.packing		= SOC_MBUS_PACKING_1_5X8,
		.order			= SOC_MBUS_ORDER_LE,
	},
};

/* pxa955_cam_get_formats provide all fmts that camera controller support*/
static int pxa955_cam_get_formats(struct soc_camera_device *icd,
				  unsigned int idx,
				  struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->dev.parent;
	int formats = 0, ret, i;
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
	/* refer to mbus_fmt struct*/
	case V4L2_MBUS_FMT_UYVY8_2X8:
		formats = ARRAY_SIZE(pxa955_camera_formats);

		if (xlate) {
			for (i = 0; i < ARRAY_SIZE(pxa955_camera_formats); i++) {
				xlate->host_fmt = &pxa955_camera_formats[i];
				xlate->code	= code;
				xlate++;
				dev_err(dev, "Providing format %s\n",
					pxa955_camera_formats[i].name);
			}
			dev_err(dev, "Providing format %s\n", fmt->name);
		}
		break;

	case V4L2_MBUS_FMT_VYUY8_2X8:
	case V4L2_MBUS_FMT_RGB565_2X8_LE:
	case V4L2_MBUS_FMT_RGB565_2X8_BE:
	case V4L2_MBUS_FMT_JPEG_1X8:
		if (xlate)
			dev_err(dev, "Providing format %s\n", fmt->name);
		break;
	default:
		/* camera controller can not support this format, which might supported by the sensor*/
		dev_err(dev, "Not support fmt %s\n", fmt->name);
		return 0;
	}

	/* Generic pass-through */
	formats++;
	if (xlate) {
		xlate->host_fmt	= fmt;
		xlate->code	= code;
		xlate++;
	}

	return formats;
}

static void pxa955_cam_put_formats(struct soc_camera_device *icd)
{
	kfree(icd->host_priv);
	icd->host_priv = NULL;
}

static int pxa955_cam_try_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_mbus_framefmt mf;
	__u32 pixfmt = pix->pixelformat;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(icd->dev.parent, "Format %c%c%c%c not found\n", \
				pixfmtstr(pixfmt));
		return -EINVAL;
	}

	pix->bytesperline = soc_mbus_bytes_per_line(pix->width,
							xlate->host_fmt);
	if (pix->bytesperline < 0)
		return pix->bytesperline;
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

static int pxa955_cam_set_fmt(struct soc_camera_device *icd,
			      struct v4l2_format *f)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct device *dev = icd->dev.parent;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate = NULL;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	__u32 pixfmt = pix->pixelformat;
	struct v4l2_mbus_framefmt mf;
	struct v4l2_control ctrl;
	int ret;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate) {
		dev_warn(dev, "Format %x not found\n", pixfmt);
		return -EINVAL;
	}
	printk(KERN_NOTICE "cam: set fmt as %c%c%c%c, %ux%u\n", \
		pixfmtstr(f->fmt.pix.pixelformat), \
		f->fmt.pix.width, f->fmt.pix.height);

	mf.width	= pix->width;
	mf.height	= pix->height;
	mf.field	= pix->field;
	mf.colorspace	= pix->colorspace;
	mf.code		= xlate->code;

	ret = v4l2_subdev_call(sd, video, s_mbus_fmt, &mf);

	if (mf.code != xlate->code)
		return -EINVAL;

	icd->sense = NULL;
	pix->width		= mf.width;
	pix->height		= mf.height;
	pix->field		= mf.field;
	pix->colorspace		= mf.colorspace;
	icd->current_fmt	= xlate;

	sci_disable(pcdev);
	sci_s_fmt(pcdev, pix);

	/* Get phy timing parameter from sensor, if sensor donn't support */
	/* this behavior, will use default value */
	pcdev->csidev->phy_cfg = NULL;
	ctrl.id = V4L2_CID_PRIVATE_GET_MIPI_PHY;
	ret = v4l2_subdev_call(sd, core, g_ctrl, &ctrl);
	if ((ret < 0) || (ctrl.value == 0)) {
		printk(KERN_NOTICE "cam: use default D_PHY timing\n");
		pcdev->csidev->phy_cfg = &default_phy_val;
	} else
		pcdev->csidev->phy_cfg = (struct mipi_phy *)ctrl.value;
	/* Ignore return value from sensor, it's OK for sensor don't support */
	/* V4L2_CID_PRIVATE_GET_MIPI_PHY interface, will use default value */
	ret = 0;

	pcdev->state = CAM_STATE_FORMATED;
	v4l2_subdev_call(sd, sensor, g_skip_top_lines, &skip_frame);

	return ret;
}

static unsigned int pxa955_cam_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;
	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int pxa955_cam_querycap(struct soc_camera_host *ici,
			       struct v4l2_capability *cap)
{
	struct pxa955_cam_dev *pcdev = ici->priv;
	struct soc_camera_device *icd = pcdev->icd;
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct i2c_board_info *pci2c = icl->board_info;

	cap->version = PXA955_CAM_VERSION_CODE;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	strcpy(cap->card, PXA955_CAM_DRV_NAME);
	strcpy(cap->driver, pci2c->type);
	return 0;
}

static int pxa955_cam_set_bus_param(struct soc_camera_device *icd, __u32 pixfmt)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	unsigned long ctrller_flags, sensor_flags, common_flags;
	int ret;
	int lane = 0;

	/* Configure this flag according to controller ability: support 3 lanes */
	ctrller_flags = SOCAM_MIPI | SOCAM_MIPI_1LANE \
			| SOCAM_MIPI_2LANE | SOCAM_MIPI_3LANE,
	sensor_flags = icd->ops->query_bus_param(icd);

	common_flags = soc_camera_bus_param_compatible(sensor_flags, ctrller_flags);
	if (!common_flags) {
		return -EINVAL;
	}

	ret = icd->ops->set_bus_param(icd, common_flags);
	if (ret < 0)
		return ret;

	/* If sensor specified lane number, abort */
	if (pcdev->csidev->phy_cfg && pcdev->csidev->phy_cfg->lane)
		return 0;
	/* Otherwise, figure out lane number */
	if (common_flags & SOCAM_MIPI_1LANE) {
		lane = 1;
	} else if (common_flags & SOCAM_MIPI_2LANE) {
		lane = 2;
	} else if (common_flags & SOCAM_MIPI_3LANE) {
		lane = 3;
	} else if (common_flags & SOCAM_MIPI_4LANE) {
		lane = 4;
	}
	pcdev->csidev->phy_cfg->lane = lane;
	return 0;
}

static int pxa955_cam_get_param(struct soc_camera_device *icd,
				  struct v4l2_streamparm *parm)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_link *icl = to_soc_camera_link(icd);
	struct sensor_platform_data *pdata;
	struct v4l2_subdev_frame_interval inter;
	struct v4l2_captureparm *cp = &parm->parm.capture;
	int ret;

	if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	pdata = icl->priv;
	printk(KERN_INFO "cam: pdata->mclk_mhz=%u Mhz\n", pdata->mclk_mhz);

	/*To get frame_rate*/
	inter.pad = pdata->mclk_mhz;
	ret = v4l2_subdev_call(sd, video, g_frame_interval, &inter);
	if (ret < 0) {
		printk(KERN_ERR "cam: fail to get frame rate from sensor.\n");
		return ret;
	}

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = inter.interval.denominator;
	cp->timeperframe.denominator = inter.interval.numerator;

	return 0;
}

static int pxa955_cam_set_param(struct soc_camera_device *icd,
				  struct v4l2_streamparm *parm)
{
	return 0;
}

static int pxa955_cam_enum_fsizes(struct soc_camera_device *icd,
					struct v4l2_frmsizeenum *fsizes)
{
	int ret;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const struct soc_camera_format_xlate *xlate;
	__u32 pixfmt = fsizes->pixel_format;
	struct v4l2_frmsizeenum *fsize_mbus = fsizes;

	xlate = soc_camera_xlate_by_fourcc(icd, pixfmt);
	if (!xlate)
		return -EINVAL;

	/* map xlate-code to pixel_format, sensor only handle xlate-code*/
	fsize_mbus->pixel_format = xlate->code;

	ret = v4l2_subdev_call(sd, video, enum_mbus_fsizes, fsize_mbus);
	if (ret < 0)
		return ret;

	fsizes->pixel_format = pixfmt;

	return 0;
}

static int pxa955_cam_g_crop(struct soc_camera_device *icd,
				struct v4l2_crop *crop)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	return v4l2_subdev_call(sd, video, g_crop, crop);
}

static int pxa955_cam_s_crop(struct soc_camera_device *icd,
				struct v4l2_crop *crop)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->dev.parent);
	struct pxa955_cam_dev *pcdev = ici->priv;
	int ret;

	if (pcdev->state == CAM_STATE_STREAMING) {
#ifdef _CONTROLLER_DEADLOOP_RESET_
		/* Announce reset timer is being killed */
		pcdev->killing_reset_timer = 1;
		del_timer(&pcdev->reset_timer);
#endif
		ret = v4l2_subdev_call(sd, video, s_stream, 0);
		if (ret < 0) {
			printk(KERN_ERR "cam: failed to stream off for zoom\n");
			return -EAGAIN;
		}

		spin_lock(&pcdev->spin_lock);
		sci_irq_disable(pcdev, IRQ_EOFX|IRQ_OFO);
		sci_disable(pcdev);
		spin_unlock(&pcdev->spin_lock);
		pcdev->state = CAM_STATE_FORMATED;
	}

	ret = v4l2_subdev_call(sd, video, s_crop, crop);
	if (ret < 0) {
		printk(KERN_ERR "cam: failed to set sensor zoom\n");
		return -EAGAIN;
	}

	if (pcdev->state == CAM_STATE_STREAMING) {
		dma_chain_init(pcdev);
		pcdev->state = CAM_STATE_STREAMING;
		spin_lock(&pcdev->spin_lock);
		sci_irq_enable(pcdev, IRQ_EOFX|IRQ_OFO);
		sci_enable(pcdev);
		spin_unlock(&pcdev->spin_lock);

		ret = v4l2_subdev_call(sd, video, s_stream, 1);
		if (ret < 0) {
			printk(KERN_ERR "cam: failed to stream on for zoom\n");
			return -EAGAIN;
		}
#ifdef _CONTROLLER_DEADLOOP_RESET_
		/* Declear timer can be resumed, so after skipping 3 frames,
		 * on frame irq, timer will be started again */
		pcdev->killing_reset_timer = 0;
#endif
	}
	return ret;
}

static struct soc_camera_host_ops pxa955_soc_cam_host_ops = {
	.owner		= THIS_MODULE,
	.add		= pxa955_cam_add_device,
	.remove		= pxa955_cam_remove_device,
	.get_formats	= pxa955_cam_get_formats,
	.put_formats	= pxa955_cam_put_formats,
	.set_fmt	= pxa955_cam_set_fmt,
	.try_fmt	= pxa955_cam_try_fmt,
	.init_videobuf2	= pxa97x_cam_init_videobuf2,
	.poll		= pxa955_cam_poll,
	.querycap	= pxa955_cam_querycap,
	.set_bus_param	= pxa955_cam_set_bus_param,
	.get_parm 	= pxa955_cam_get_param,
	.set_parm 	= pxa955_cam_set_param,
	.enum_fsizes 	= pxa955_cam_enum_fsizes,
	.get_crop	= pxa955_cam_g_crop,
	.set_crop	= pxa955_cam_s_crop,
};

static irqreturn_t csi_irq(int irq, void *data)
{
	struct pxa955_cam_dev *pcdev = data;
	struct pxa95x_csi_dev *pcsi = pcdev->csidev;
	unsigned int irqs;

	spin_lock(&pcsi->dev_lock);
	irqs = csi_reg_read(pcsi, REG_CSxINST);
	csi_reg_write(pcsi, REG_CSxINST, irqs);
	spin_unlock(&pcsi->dev_lock);
	return IRQ_HANDLED;
}

static irqreturn_t cam_irq(int irq, void *data)
{
	struct pxa955_cam_dev *pcdev = data;
	unsigned int irqs = 0;

	spin_lock(&pcdev->spin_lock);
	irqs = sci_reg_read(pcdev, REG_SCISR);
	sci_reg_write(pcdev, REG_SCISR, irqs);		/*clear irqs here*/
	if (irqs & IRQ_OFO) {
		printk(KERN_ERR "cam: ccic over flow error!\n");
		csi_disable(pcdev->csidev);
		sci_disable(pcdev);
		/* After disable SCI, must clean SCISR again, because it's
		 * possible that more OFO comes between 1st OFO and disable
		 * SCI DMA */
		irqs = sci_reg_read(pcdev, REG_SCISR);
		sci_reg_write(pcdev, REG_SCISR, irqs);
		if (irqs & IRQ_OFO)
			printk(KERN_ERR "cam: ccic over flow error! (2nd)\n");
		/* The only possible sub-sequence irq at this time is OFO */
		if (irqs & IRQ_SINT)
			printk(KERN_ERR "cam: ccic over flow error! (pend)\n");
		dma_chain_init(pcdev);

		sci_enable(pcdev);
		csi_enable(pcdev->csidev, pcdev->icd->iface);
		spin_unlock(&pcdev->spin_lock);
		return IRQ_HANDLED;
	}

	if (irqs & IRQ_SOFX) {
		struct pxa_buf_node *buf_node;
		u64 now = OSCR4;

		/*
		 * get the first node of dma_list
		 * and fill the timestamp field of v4l2_buffer.
		 */
		buf_node = list_entry(pcdev->dma_chain.next,
						struct pxa_buf_node, hook);
		/*
		 * OSCR timer resolution is 1/32768 second.
		 * convert the OSCR counts to seconds and useconds.
		 */
		buf_node->vb2.v4l2_buf.timestamp.tv_sec = now >> 15;
		buf_node->vb2.v4l2_buf.timestamp.tv_usec =
			((now & 0x7FFF) * 1000000) >> 15;
	}

	if (irqs & IRQ_EOFX) {
		sci_reg_clear_bit(pcdev, REG_SCICR0, SCICR0_FSC(0xF));
#ifdef _CONTROLLER_DEADLOOP_RESET_
		if (!pcdev->killing_reset_timer)
			mod_timer(&pcdev->reset_timer, \
					jiffies + MIPI_RESET_TIMEOUT);
#endif
		if (skip_frame == 0) {
			dma_fetch_frame(pcdev);
			dma_attach_bufs(pcdev);
		} else {
			printk(KERN_NOTICE "cam: skip frame %d\n", skip_frame);
			skip_frame--;
		}
	}

	spin_unlock(&pcdev->spin_lock);
	return IRQ_HANDLED;
}

#ifdef _CONTROLLER_DEADLOOP_RESET_
static void ccic_timeout_handler(unsigned long data)
{
	static unsigned int reset_cnt = 1;
	struct pxa955_cam_dev *pcdev = (struct pxa955_cam_dev *)data;
	int csi_irqs, sci_irqs;
	int frame_flag = CSI_CONT_SOF_INT_STS | CSI_CONT_EOF_INT_STS;

	csi_irqs = csi_reg_read(pcdev->csidev, REG_CSxINST);
	csi_reg_write(pcdev->csidev, REG_CSxINST, csi_irqs);

	if (unlikely(csi_irqs & ~(frame_flag|0x01)))
		/* There are some error bit set */
		printk(KERN_INFO "cam: CSxINST = 0x%08X\n", csi_irqs);

	if ((csi_irqs & frame_flag) == frame_flag) {
		/* SOF and EOF is set, suppose CSI is normal */
		sci_irqs = sci_reg_read(pcdev, REG_SCISR);
		printk(KERN_INFO "cam: mipi normal, SCISR = 0x%08X-----------" \
			"-------------------------<%d>\n", sci_irqs, reset_cnt);
		/* not CSI fault, don't reset CSI here*/
#ifdef _CONTROLLER_DEADLOOP_RESET_
			mod_timer(&pcdev->reset_timer, \
					jiffies + MIPI_RESET_TIMEOUT);
#endif
	} else {
		printk(KERN_INFO "cam: CSxINST = 0x%08X, call reset workqueue" \
			"-----------------------<%d>\n", csi_irqs, reset_cnt++);
		schedule_work(&pcdev->reset_wq);
	}
}

static void ccic_reset_handler(struct work_struct *work)
{
#if 0 /* Don't turn off sensor now to save time */
#define _TURN_OFF_SENSOR_
	struct v4l2_subdev *sd;
#endif
#ifdef _TURN_OFF_SENSOR_
/*#define _RESET_SENSOR_*/
#endif
	struct pxa955_cam_dev *pcdev = container_of(work, struct pxa955_cam_dev,
							reset_wq);
	int val = 0;
	unsigned long flags;

	if (pcdev->killing_reset_timer)
		return;

	/* CSI erroneous state usually caused by abnormal sensor state
	 * If the CSI reset is commited frequently, must check sensor!*/

#ifdef _TURN_OFF_SENSOR_
	sd = soc_camera_to_subdev(pcdev->icd);
	/* Turn off sensor output, required by spec, but works well without it,
	 * disable it to save a lot of time in reset routing */
	v4l2_subdev_call(sd, video, s_stream, 0);
	msleep(2);
	/* Assume after 2 ms, sensor output is off, both data and clock lane
	 * is in LP11 state now, this is a pre-condition for CSI reset */
#endif
#ifdef _RESET_SENSOR_
	/* If sensor enters a unrecoverable error state, a sensor reset must
	 * be committed, which however, will take several millie-sec. */
	v4l2_subdev_call(sd, core, load_fw);
#endif
	spin_lock_irqsave(&pcdev->spin_lock, flags);
	csi_reset(pcdev->csidev);
	csi_disable(pcdev->csidev);
	sci_disable(pcdev);
	/* Wait until CSI is acutally turned off */
	while (val & CSxSR_CSIEN)
		val = csi_reg_read(pcdev->csidev, REG_CSxSR);

	csi_clkdiv(pcdev->csidev);
	csi_dphy(pcdev->csidev);
	sci_enable(pcdev);
	csi_enable(pcdev->csidev, pcdev->icd->iface);
	spin_unlock_irqrestore(&pcdev->spin_lock, flags);
#ifdef _TURN_OFF_SENSOR_
	v4l2_subdev_call(sd, video, s_stream, 1);
#endif
	/* After streamon sensor, controller should receive a valid frame
	 * before time out, otherwise handle mipi error again*/
	if (!pcdev->killing_reset_timer)
		mod_timer(&pcdev->reset_timer, jiffies + MIPI_RESET_TIMEOUT);
}
#endif

static int pxa955_camera_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err = -ENOMEM;
	struct pxa955_cam_dev *pcdev;

	pcdev = kzalloc(sizeof(struct pxa955_cam_dev), GFP_KERNEL);
	if (pcdev == NULL)
		goto exit;
	memset(pcdev, 0, sizeof(struct pxa955_cam_dev));

	spin_lock_init(&pcdev->spin_lock);
	INIT_LIST_HEAD(&pcdev->dev_list);
	INIT_LIST_HEAD(&pcdev->dma_chain);
	INIT_LIST_HEAD(&pcdev->new_chain);
	pcdev->dma_bufs = pcdev->new_bufs = 0;


	/* init camera controller resource*/
	pcdev->irq = platform_get_irq(pdev, 0);
	if (pcdev->irq < 0) {
		printk(KERN_ERR "cam: camera no irq\n");
		return -ENXIO;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "cam: no IO memory resource defined\n");
		return -ENODEV;
	}

	pcdev->regs = ioremap(res->start, SZ_4K);
	if (!pcdev->regs) {
		printk(KERN_ERR "cam: unable to ioremap pxa95x-camera regs\n");
		goto exit_free;
	}
	err = request_irq(pcdev->irq, cam_irq,
		0, PXA955_CAM_DRV_NAME, pcdev);
	if (err) {
		printk(KERN_ERR "cam: unable to create ist\n");
		goto exit_iounmap;
	}

	pcdev->sci1_clk = clk_get(NULL, "SCI1CLK");
	if (!pcdev->sci1_clk) {
		printk(KERN_ERR "cam: unable to get sci clk\n");
		goto exit_free_irq;
	}
	pcdev->sci2_clk = clk_get(NULL, "SCI2CLK");
	if (!pcdev->sci2_clk) {
		printk(KERN_ERR "cam: unable to get sci clk\n");
		goto exit_free_irq;
	}

#ifdef _CONTROLLER_DEADLOOP_RESET_
	init_timer(&pcdev->reset_timer);
	pcdev->reset_timer.function = ccic_timeout_handler;
	pcdev->reset_timer.data = (unsigned long)pcdev;
	INIT_WORK(&pcdev->reset_wq, ccic_reset_handler);
#endif
#ifdef _ARB_CHANGE_
	pri_axi = ioremap_nocache(ARB_CNTRL_AXI, 0x20);
	pri_ci1 = ioremap_nocache(ARB_CNTRL_CI1, 0x20);
	pri_ci2 = ioremap_nocache(ARB_CNTRL_CI2, 0x20);
	pri_gcu = ioremap_nocache(ARB_CNTRL_GCU, 0x20);
	wrr_nor = ioremap_nocache(MC_WRR_NORMAL, 0x20);
	wrr_fst = ioremap_nocache(MC_WRR_FAST, 0x20);
	arb_sch = ioremap_nocache(MC_ARB_SCHEME, 0x20);
#endif

	pcdev->soc_host.drv_name	= PXA955_CAM_DRV_NAME;
	pcdev->soc_host.ops		= &pxa955_soc_cam_host_ops;
	pcdev->soc_host.priv		= pcdev;
	pcdev->soc_host.v4l2_dev.dev	= &pdev->dev;
	pcdev->soc_host.nr		= pdev->id;

	err = soc_camera_host_register(&pcdev->soc_host);
	if (err)
		goto exit_free_irq;

	/* video dev initialized, but not opened */
	pcdev->state = CAM_STATE_CLOSE;
	return 0;

exit_free_irq:
	free_irq(pcdev->irq, pcdev);
exit_iounmap:
	iounmap(pcdev->regs);
	clk_put(pcdev->sci1_clk);
	clk_put(pcdev->sci2_clk);
	clk_put(pcdev->sci1_clk);
	clk_put(pcdev->sci2_clk);
exit_free:
	kfree(pcdev);
exit:
	return err;
}


static int pxa955_camera_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct pxa955_cam_dev *pcdev = container_of(soc_host,
					struct pxa955_cam_dev, soc_host);

	if (pcdev == NULL) {
		printk(KERN_WARNING "cam: remove on unknown pdev %p\n", pdev);
		return -EIO;
	}

	sci_disable(pcdev);
	sci_irq_disable(pcdev, IRQ_EOFX|IRQ_OFO);
	free_irq(pcdev->irq, pcdev);
	iounmap(common_base);
	return 0;
}

static struct platform_driver pxa955_camera_driver = {
	.driver = {
		.name = PXA955_CAM_DRV_NAME
	},
	.probe	= pxa955_camera_probe,
	.remove	= pxa955_camera_remove,

};

static int __devinit pxa955_camera_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&pxa955_camera_driver);
	dvfm_register(PXA955_CAM_DRV_NAME, &dvfm_dev_idx);
	return ret;
}

static void __exit pxa955_camera_exit(void)
{
	platform_driver_unregister(&pxa955_camera_driver);
	dvfm_unregister(PXA955_CAM_DRV_NAME, &dvfm_dev_idx);
}

module_init(pxa955_camera_init);
module_exit(pxa955_camera_exit);
