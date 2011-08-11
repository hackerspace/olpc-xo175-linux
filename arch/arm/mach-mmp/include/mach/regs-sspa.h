#ifndef __ASM_ARCH_REGS_SSPA_H
#define __ASM_ARCH_REGS_SSPA_H

#include <mach/mmp3_audisland.h>

#define SSPA1_VIRT_BASE			(AUD_VIRT_BASE + 0xc00)
#define SSPA2_VIRT_BASE			(AUD_VIRT_BASE + 0xd00)
#define DSA_SSP_CLK_RES_CTRL		(AUD_VIRT_BASE2 + 0x30)
#define DSA_ABU_CLK_RES_CTRL		(AUD_VIRT_BASE2 + 0x34)

#define SSPA1_REG(x)		(SSPA1_VIRT_BASE + (x))
#define SSPA2_REG(x)		(SSPA2_VIRT_BASE + (x))

#define SSPA_AUD_CTRL		SSPA1_REG(0x34)
#define SSPA_AUD_PLL_CTRL0	SSPA1_REG(0x38)
#define SSPA_AUD_PLL_CTRL1	SSPA1_REG(0x3c)

/*
 * PXA688 SSPA Serial Port Registers
 */

#define SSPA_RXD		(0x00)
#define SSPA_RXID		(0x04)
#define SSPA_RXCTL		(0x08)
#define SSPA_RXSP		(0x0c)
#define SSPA_RXFIFO_UL		(0x10)
#define SSPA_RXINT_MASK		(0x14)
#define SSPA_RXC		(0x18)
#define SSPA_RXFIFO_NOFS	(0x1c)
#define SSPA_RXFIFO_SIZE	(0x20)

#define SSPA_TXD		(0x80)
#define SSPA_TXID		(0x84)
#define SSPA_TXCTL		(0x88)
#define SSPA_TXSP		(0x8c)
#define SSPA_TXFIFO_LL		(0x90)
#define SSPA_TXINT_MASK		(0x94)
#define SSPA_TXC		(0x98)
#define SSPA_TXFIFO_NOFS	(0x9c)
#define SSPA_TXFIFO_SIZE	(0xa0)

/* SSPA Control Register */
#define	SSPA_CTL_XPH		(1 << 31)	/* Read Phase */
#define	SSPA_CTL_XFIG		(1 << 15)	/* Transmit Zeros when FIFO Empty */
#define	SSPA_CTL_JST		(1 << 3)	/* Audio Sample Justification */
#define	SSPA_CTL_XFRLEN2_MASK	(7 << 24)
#define	SSPA_CTL_XFRLEN2(x)	((x) << 24)	/* Transmit Frame Length in Phase 2 */
#define	SSPA_CTL_XWDLEN2_MASK	(7 << 21)
#define	SSPA_CTL_XWDLEN2(x)	((x) << 21)	/* Transmit Word Length in Phase 2 */
#define	SSPA_CTL_XDATDLY(x)	((x) << 19)	/* Tansmit Data Delay */
#define	SSPA_CTL_XSSZ2_MASK	(7 << 16)
#define	SSPA_CTL_XSSZ2(x)	((x) << 16)	/* Transmit Sample Audio Size */
#define	SSPA_CTL_XFRLEN1_MASK	(7 << 8)
#define	SSPA_CTL_XFRLEN1(x)	((x) << 8)	/* Transmit Frame Length in Phase 1 */
#define	SSPA_CTL_XWDLEN1_MASK	(7 << 5)
#define	SSPA_CTL_XWDLEN1(x)	((x) << 5)	/* Transmit Word Length in Phase 1 */
#define	SSPA_CTL_XSSZ1_MASK	(7 << 0)
#define	SSPA_CTL_XSSZ1(x)	((x) << 0)	/* XSSZ1 */

#define SSPA_CTL_8_BITS		(0x0)		/* sample size */
#define SSPA_CTL_12_BITS	(0x1)
#define SSPA_CTL_16_BITS	(0x2)
#define SSPA_CTL_20_BITS	(0x3)
#define SSPA_CTL_24_BITS	(0x4)
#define SSPA_CTL_32_BITS	(0x5)

/* SSPA Serial Port Register */
#define	SSPA_SP_WEN		(1 << 31)	/* Write Configuration Enable */
#define	SSPA_SP_MSL		(1 << 18)	/* Master Slave Configuration */
#define	SSPA_SP_CLKP		(1 << 17)	/* CLKP Polarity Clock Edge Select */
#define	SSPA_SP_FSP		(1 << 16)	/* FSP Polarity Clock Edge Select */
#define	SSPA_SP_FFLUSH		(1 << 2)	/* FIFO Flush */
#define	SSPA_SP_S_RST		(1 << 1)	/* Active High Reset Signal */
#define	SSPA_SP_S_EN		(1 << 0)	/* Serial Clock Domain Enable */
#define	SSPA_SP_FWID_MASK	(0xff << 20)
#define	SSPA_SP_FWID(x)		((x) << 20)	/* Frame-Sync Width */
#define	SSPA_SP_FPER_MASK	(0xfff << 4)
#define	SSPA_SP_FPER(x)		((x) << 4)	/* Frame-Sync Active */

/* SSPA Audio Control Register */
#define SSPA_AUD_CTRL_S2_CLK_SEL_MASK		(1 << 23)
#define SSPA_AUD_CTRL_S2_CLK_SEL_I2S		(1 << 23)
#define SSPA_AUD_CTRL_S2_CLK_SEL_AUDIO_PLL	(0 << 23)
#define SSPA_AUD_CTRL_S2_CLK_DIV_MASK		(0x3f << 17)
#define SSPA_AUD_CTRL_S2_CLK_DIV(x)		((x) << 17)
#define SSPA_AUD_CTRL_S2_ENA			(1 << 16)
#define SSPA_AUD_CTRL_S1_CLK_DIV_MASK		(0x3f << 9)
#define SSPA_AUD_CTRL_S1_CLK_DIV(x)		((x) << 9)
#define SSPA_AUD_CTRL_S1_ENA			(1 << 8)
#define SSPA_AUD_CTRL_S1_CLK_SEL_MASK		(1 << 7)
#define SSPA_AUD_CTRL_S1_CLK_SEL_I2S		(1 << 7)
#define SSPA_AUD_CTRL_S1_CLK_SEL_AUDIO_PLL	(0 << 7)
#define SSPA_AUD_CTRL_SYSCLK_DIV_MASK		(0x3f << 1)
#define SSPA_AUD_CTRL_SYSCLK_DIV(x)		((x) << 1)
#define SSPA_AUD_CTRL_SYSCLK_ENA		(1 << 0)

/* SSPA Audio PLL Control 0 Register */
#define SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO(x)	((x) << 28)
#define SSPA_AUD_PLL_CTRL0_FRACT(x)		((x) << 8)
#define SSPA_AUD_PLL_CTRL0_ENA_DITHER		(1 << 7)
#define SSPA_AUD_PLL_CTRL0_ICP_2UA		(0 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_5UA		(1 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_7UA		(2 << 5)
#define SSPA_AUD_PLL_CTRL0_ICP_10UA		(3 << 5)
#define SSPA_AUD_PLL_CTRL0_DIV_FBCCLK(x)	((x) << 3)
#define SSPA_AUD_PLL_CTRL0_DIV_MCLK(x)		((x) << 2)
#define SSPA_AUD_PLL_CTRL0_PD_OVPROT_DIS	(1 << 1)
#define SSPA_AUD_PLL_CTRL0_PU			(1 << 0)

/* SSPA Audio PLL Control 1 Register */
#define SSPA_AUD_PLL_CTRL1_EN_VCOX2		(1 << 17)
#define SSPA_AUD_PLL_CTRL1_PLL_LOCK		(1 << 16)
#define SSPA_AUD_PLL_CTRL1_CLK_SEL_AUDIO_PLL	(1 << 11)
#define SSPA_AUD_PLL_CTRL1_CLK_SEL_VCXO		(0 << 11)
#define SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN(x)	((x) << 0)

/* SSPA and sysclk pll sources */
#define SSPA_AUDIO_PLL				0
#define SSPA_I2S_PLL				1
#define SSPA_VCXO_PLL				2
#endif /* __ASM_ARCH_REGS_SSPA_H */
