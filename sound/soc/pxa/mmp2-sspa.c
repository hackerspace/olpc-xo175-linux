/*
 * linux/sound/soc/pxa/mmp2-sspa.c
 * Base on pxa2xx-ssp.c
 *
 * Copyright (C) 2007 Marvell International Ltd.
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
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <linux/io.h>
#include <mach/regs-sspa.h>
#include <mach/mmp_dma.h>
#include <plat/dma.h>
#include <plat/ssp.h>

#include "mmp2-squ.h"
#include "mmp2-sspa.h"

enum {
	MMP2_SSPA1,
	MMP2_SSPA2,
};

/*
 * SSPA audio private data
 */
struct sspa_priv {
	struct ssp_device *sspa;
	unsigned int freq;
	unsigned int sysclk;
	int dai_fmt;
	int suspend;
	int running_cnt;
#ifdef CONFIG_PM
	uint32_t rxctrl;
	uint32_t rxsp;
	uint32_t rxfifo_ul;
	uint32_t rxint_mask;
	uint32_t txctrl;
	uint32_t txsp;
	uint32_t txfifo_ll;
	uint32_t txint_mask;
#endif
};

struct pll_set {
	unsigned int freq_in;		/* vcxo freq */
	unsigned int freq_out;		/* audio pll */
	unsigned int mclk;
	unsigned int fbcclk;
	unsigned int fract;
	unsigned int postdiv;
	unsigned int oclk_modulo;
	unsigned int oclk_pattern;
};

static struct pll_set audio_pll_set[] = {
/* in out mclk fbcclk fract postdiv modulo pattern */
{26000000, 147456000, 0, 1, 0x00da1,  1, 0, 0},
{26000000,  36864000, 0, 1, 0x00da1,  4, 0, 0},
{26000000,  24576000, 0, 1, 0x00da1,  6, 1, 1},
{26000000,  18432000, 0, 1, 0x00da1,  8, 1, 0},
{26000000,  16384000, 0, 1, 0x00da1,  9, 1, 2},
{26000000,  12288000, 0, 1, 0x00da1, 12, 2, 1},
{26000000,   9216000, 0, 1, 0x00da1, 16, 2, 0},
{26000000,   8192000, 0, 1, 0x00da1, 18, 2, 2},
{26000000,   6144000, 0, 1, 0x00da1, 24, 4, 1},
{26000000,   4096000, 0, 1, 0x00da1, 36, 4, 2},
{26000000,   3072000, 0, 1, 0x00da1, 48, 6, 1},
{26000000,   2048000, 0, 1, 0x00da1, 72, 6, 2},
{26000000, 135475200, 0, 0, 0x08a18,  1, 0, 0},
{26000000,  33868800, 0, 0, 0x08a18,  4, 0, 0},
{26000000,  22579200, 0, 0, 0x08a18,  6, 1, 1},
{26000000,  16934400, 0, 0, 0x08a18,  8, 1, 0},
{26000000,  11289600, 0, 0, 0x08a18, 12, 2, 1},
{26000000,   8467200, 0, 0, 0x08a18, 16, 2, 0},
{26000000,   5644800, 0, 0, 0x08a18, 24, 4, 1},
};

static struct mmp2_adma_params mmp2_pcm_adma_params[] = {
	[0] = {
		.name = "SSPA1 PCM out",
	},
	[1] = {
		.name = "SSPA1 PCM in",
	},
	[2] = {
		.name = "SSPA2 PCM out",
	},
	[3] = {
		.name = "SSPA2 PCM in",
	},
};

static void mmp2_sspa_write_reg(struct ssp_device *sspa, u32 reg, u32 val)
{
	__raw_writel(val, sspa->mmio_base + reg);
}

static u32 mmp2_sspa_read_reg(struct ssp_device *sspa, u32 reg)
{
	return __raw_readl(sspa->mmio_base + reg);
}

static void mmp2_sspa_tx_enable(struct ssp_device *sspa)
{
	unsigned int sspa_sp;

	sspa_sp = mmp2_sspa_read_reg(sspa, SSPA_TXSP);
	sspa_sp |= SSPA_SP_S_EN;
	sspa_sp |= SSPA_SP_WEN;
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
}

static void mmp2_sspa_tx_disable(struct ssp_device *sspa)
{
	unsigned int sspa_sp;

	sspa_sp = mmp2_sspa_read_reg(sspa, SSPA_TXSP);
	sspa_sp &= ~SSPA_SP_S_EN;
	sspa_sp |= SSPA_SP_WEN;
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
}

static void mmp2_sspa_rx_enable(struct ssp_device *sspa)
{
	unsigned int sspa_sp;

	sspa_sp = mmp2_sspa_read_reg(sspa, SSPA_RXSP);
	sspa_sp |= SSPA_SP_S_EN;
	sspa_sp |= SSPA_SP_WEN;
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);
}

static void mmp2_sspa_rx_disable(struct ssp_device *sspa)
{
	unsigned int sspa_sp;

	sspa_sp = mmp2_sspa_read_reg(sspa, SSPA_RXSP);
	sspa_sp &= ~SSPA_SP_S_EN;
	sspa_sp |= SSPA_SP_WEN;
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);
}

static void mmp2_sspa_clk_enable(struct ssp_device *sspa, int id)
{
	unsigned int val;

	/* enable sspa clk */
	val = __raw_readl(SSPA_AUD_CTRL);
	if (id == MMP2_SSPA1)
		val |= SSPA_AUD_CTRL_S1_ENA;
	else
		val |= SSPA_AUD_CTRL_S2_ENA;
	__raw_writel(val, SSPA_AUD_CTRL);
}

static void mmp2_sspa_clk_disable(struct ssp_device *sspa, int id)
{
	unsigned int val;

	/* disable sspa clk */
	val = __raw_readl(SSPA_AUD_CTRL);
	if (id == MMP2_SSPA1)
		val &= ~SSPA_AUD_CTRL_S1_ENA;
	else
		val &= ~SSPA_AUD_CTRL_S2_ENA;
	__raw_writel(val, SSPA_AUD_CTRL);
}

static void mmp2_sspa_sysclk_enable(struct ssp_device *sspa)
{
	unsigned int val;

	/* enable sysclk clk */
	val = __raw_readl(SSPA_AUD_CTRL);
	val |= SSPA_AUD_CTRL_SYSCLK_ENA;
	__raw_writel(val, SSPA_AUD_CTRL);
}

static void mmp2_sspa_sysclk_disable(struct ssp_device *sspa)
{
	unsigned int val;

	/* disable sspa clk */
	val = __raw_readl(SSPA_AUD_CTRL);
	val &= ~SSPA_AUD_CTRL_SYSCLK_ENA;
	__raw_writel(val, SSPA_AUD_CTRL);
}

static void mmp2_sspa_dump_reg(struct ssp_device *sspa)
{
	dev_dbg(&sspa->pdev->dev, "RXD         %08x RXID        %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_RXD),
		mmp2_sspa_read_reg(sspa, SSPA_RXID));
	dev_dbg(&sspa->pdev->dev, "RXCTL       %08x RXSP        %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_RXCTL),
		mmp2_sspa_read_reg(sspa, SSPA_RXSP));
	dev_dbg(&sspa->pdev->dev, "RXFIFO_UL   %08x RXINT_MASK  %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_RXFIFO_UL),
		mmp2_sspa_read_reg(sspa, SSPA_RXINT_MASK));
	dev_dbg(&sspa->pdev->dev, "RXC         %08x RXFIFO_NOFS %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_RXC),
		mmp2_sspa_read_reg(sspa, SSPA_RXFIFO_NOFS));
	dev_dbg(&sspa->pdev->dev, "RXFIFO_SIZE %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_RXFIFO_SIZE));

	dev_dbg(&sspa->pdev->dev, "TXD         %08x TXID        %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_TXD),
		mmp2_sspa_read_reg(sspa, SSPA_TXID));
	dev_dbg(&sspa->pdev->dev, "TXCTL       %08x TXSP        %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_TXCTL),
		mmp2_sspa_read_reg(sspa, SSPA_TXSP));
	dev_dbg(&sspa->pdev->dev, "TXFIFO_LL   %08x TXINT_MASK  %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_TXFIFO_LL),
		mmp2_sspa_read_reg(sspa, SSPA_TXINT_MASK));
	dev_dbg(&sspa->pdev->dev, "TXC         %08x TXFIFO_NOFS %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_TXC),
		mmp2_sspa_read_reg(sspa, SSPA_TXFIFO_NOFS));
	dev_dbg(&sspa->pdev->dev, "TXFIFO_SIZE %08x\n",
		mmp2_sspa_read_reg(sspa, SSPA_TXFIFO_SIZE));

	return;
}

static struct mmp2_adma_params*
mmp2_sspa_get_dma_params(int id, struct ssp_device *sspa, int width, int out)
{
	struct mmp2_adma_params *dma;
	int index;

	index = (id << 1) + (!out);
	if (index >= ARRAY_SIZE(mmp2_pcm_adma_params))
		return NULL;

	dma = &mmp2_pcm_adma_params[index];
	dma->dcmd = (out ? (TDCR_DSTDIR_ADDR_HOLD | TDCR_SRCDIR_ADDR_INC) :
			   (TDCR_SRCDIR_ADDR_HOLD | TDCR_DSTDIR_ADDR_INC)) |
		    TDCR_PACKMOD | TDCR_BURSTSZ_4B | TDCR_FETCHND | width;
	dma->dma_ch = out ? sspa->drcmr_tx : sspa->drcmr_rx;
	dma->dev_addr = out ? (sspa->phys_base + SSPA_TXD) :
			      (sspa->phys_base + SSPA_RXD);
	return dma;
}

static int mmp3_sspa_startup(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	int ret = 0;

	/* enable sspa clk */
	if (!cpu_dai->active)
		mmp2_sspa_clk_enable(sspa, cpu_dai->id);

	return ret;
}

static void mmp3_sspa_shutdown(struct snd_pcm_substream *substream,
			       struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;

	/* disable sspa clk */
	if (!cpu_dai->active)
		mmp2_sspa_clk_enable(sspa, cpu_dai->id);

	return;
}

#ifdef CONFIG_PM

static uint32_t aud_ctrl;
static uint32_t aud_pll_ctrl0;
static uint32_t aud_pll_ctrl1;

static int mmp3_sspa_suspend(struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;

	pr_debug("%s: sspa id = %d\n", __func__, cpu_dai->id);

	/* other card's dai link has suspended already */
	if (priv->suspend)
		return 0;

	if (!cpu_dai->active)
		mmp2_sspa_clk_enable(sspa, cpu_dai->id);

	priv->rxctrl      = mmp2_sspa_read_reg(sspa, SSPA_RXCTL);
	priv->rxsp        = mmp2_sspa_read_reg(sspa, SSPA_RXSP);
	priv->rxfifo_ul   = mmp2_sspa_read_reg(sspa, SSPA_RXFIFO_UL);
	priv->rxint_mask  = mmp2_sspa_read_reg(sspa, SSPA_RXINT_MASK);

	priv->txctrl      = mmp2_sspa_read_reg(sspa, SSPA_TXCTL);
	priv->txsp        = mmp2_sspa_read_reg(sspa, SSPA_TXSP);
	priv->txfifo_ll   = mmp2_sspa_read_reg(sspa, SSPA_TXFIFO_LL);
	priv->txint_mask  = mmp2_sspa_read_reg(sspa, SSPA_TXINT_MASK);

	pr_debug("rxctrl %x rxsp %x rxfifo_ul %x rxint_mask %x\n",
		priv->rxctrl, priv->rxsp,
		priv->rxfifo_ul, priv->rxint_mask);
	pr_debug("txctrl %x txsp %x txfifo_ll %x txint_mask %x\n",
		priv->txctrl, priv->txsp,
		priv->txfifo_ll, priv->txint_mask);

	if (cpu_dai->id == MMP2_SSPA1) {
		aud_ctrl      = __raw_readl(SSPA_AUD_CTRL);
		aud_pll_ctrl0 = __raw_readl(SSPA_AUD_PLL_CTRL0);
		aud_pll_ctrl1 = __raw_readl(SSPA_AUD_PLL_CTRL1);
		mmp2_sspa_sysclk_disable(sspa);
	}

	mmp2_sspa_tx_disable(sspa);
	mmp2_sspa_rx_disable(sspa);
	mmp2_sspa_clk_disable(sspa, cpu_dai->id);
	priv->suspend = 1;
	return 0;
}

static int mmp3_sspa_resume(struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	u32 sspa_sp;

	pr_debug("%s: sspa id = %d\n", __func__, cpu_dai->id);

	if (!priv->suspend)
		return 0;

	if (cpu_dai->id == MMP2_SSPA1) {
		__raw_writel(aud_ctrl,      SSPA_AUD_CTRL);
		__raw_writel(aud_pll_ctrl0, SSPA_AUD_PLL_CTRL0);
		__raw_writel(aud_pll_ctrl1, SSPA_AUD_PLL_CTRL1);
		mmp2_sspa_sysclk_enable(sspa);
	}

	mmp2_sspa_clk_enable(sspa, cpu_dai->id);

	pr_debug("%s: aud_ctrl = %x, aud_pll_ctrl0 = %x, "
			"aud_pll_ctrl1 = %x\n",
			__func__,
			__raw_readl(SSPA_AUD_CTRL),
			__raw_readl(SSPA_AUD_PLL_CTRL0),
			__raw_readl(SSPA_AUD_PLL_CTRL1));

	pr_debug("rxctrl %x rxsp %x rxfifo_ul %x rxint_mask %x\n",
		priv->rxctrl, priv->rxsp,
		priv->rxfifo_ul, priv->rxint_mask);
	pr_debug("txctrl %x txsp %x txfifo_ll %x txint_mask %x\n",
		priv->txctrl, priv->txsp,
		priv->txfifo_ll, priv->txint_mask);

	sspa_sp = priv->rxsp | SSPA_SP_WEN |
		  SSPA_SP_FFLUSH | SSPA_SP_S_RST;
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_RXCTL, priv->rxctrl);
	mmp2_sspa_write_reg(sspa, SSPA_RXFIFO_UL, priv->rxfifo_ul);
	mmp2_sspa_write_reg(sspa, SSPA_RXINT_MASK, priv->rxint_mask);
	sspa_sp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);

	sspa_sp = priv->txsp | SSPA_SP_WEN |
		  SSPA_SP_FFLUSH | SSPA_SP_S_RST;
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_TXCTL, priv->txctrl);
	mmp2_sspa_write_reg(sspa, SSPA_TXFIFO_LL, priv->txfifo_ll);
	mmp2_sspa_write_reg(sspa, SSPA_TXINT_MASK, priv->txint_mask);
	sspa_sp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);

	if (!cpu_dai->active)
		mmp2_sspa_clk_disable(sspa, cpu_dai->id);

	mmp2_sspa_dump_reg(sspa);
	priv->suspend = 0;
	return 0;
}

#else
#define mmp3_sspa_suspend	NULL
#define mmp3_sspa_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int mmp3_sspa_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				    int clk_id, unsigned int freq, int dir)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	int val, div;

	div = priv->freq;
	do_div(div, freq);
	pr_debug("%s: devider = %d\n", __func__, div);

	val  = __raw_readl(SSPA_AUD_CTRL);
	val &= ~SSPA_AUD_CTRL_SYSCLK_DIV_MASK;
	val |= SSPA_AUD_CTRL_SYSCLK_DIV(div);
	__raw_writel(val, SSPA_AUD_CTRL);

	mmp2_sspa_sysclk_enable(sspa);

	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int mmp2_sspa_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				    int div_id, int div)
{
	int val;

	val = __raw_readl(SSPA_AUD_CTRL);
	if (cpu_dai->id == MMP2_SSPA1) {
		val &= ~SSPA_AUD_CTRL_S1_CLK_DIV_MASK;
		val |= SSPA_AUD_CTRL_S1_CLK_DIV(div);
	} else {
		val &= ~SSPA_AUD_CTRL_S2_CLK_DIV_MASK;
		val |= SSPA_AUD_CTRL_S2_CLK_DIV(div);
	}
	__raw_writel(val, SSPA_AUD_CTRL);

	return 0;
}

static int mmp2_sspa_set_dai_pll(struct snd_soc_dai *cpu_dai, int pll_id,
				 int source, unsigned int freq_in,
				 unsigned int freq_out)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct pll_set *set;
	u32 val, i;

	switch (pll_id) {
	case SSPA_AUDIO_PLL:
		set = audio_pll_set;
		for (i = 0; i < ARRAY_SIZE(audio_pll_set); i++) {
			if ((set->freq_in  == freq_in) &&
			    (set->freq_out == freq_out))
				break;
			set++;
		}

		if (i == ARRAY_SIZE(audio_pll_set))
			return -EINVAL;

		val = SSPA_AUD_PLL_CTRL1_CLK_SEL_AUDIO_PLL |
		      SSPA_AUD_PLL_CTRL1_PLL_LOCK |
		      SSPA_AUD_PLL_CTRL1_DIV_OCLK_PATTERN(set->oclk_pattern);
		__raw_writel(val, SSPA_AUD_PLL_CTRL1);

		val = SSPA_AUD_PLL_CTRL0_DIV_OCLK_MODULO(set->oclk_modulo) |
		      SSPA_AUD_PLL_CTRL0_FRACT(set->fract) |
		      SSPA_AUD_PLL_CTRL0_ENA_DITHER |
		      SSPA_AUD_PLL_CTRL0_DIV_FBCCLK(set->fbcclk) |
		      SSPA_AUD_PLL_CTRL0_DIV_MCLK(set->mclk) |
		      SSPA_AUD_PLL_CTRL0_PU;
		__raw_writel(val, SSPA_AUD_PLL_CTRL0);

		val = __raw_readl(SSPA_AUD_CTRL);
		if (cpu_dai->id == MMP2_SSPA1) {
			val &= ~SSPA_AUD_CTRL_S1_CLK_SEL_MASK;
			val |= SSPA_AUD_CTRL_S1_CLK_SEL_AUDIO_PLL;
		} else {
			val &= ~SSPA_AUD_CTRL_S2_CLK_SEL_MASK;
			val |= SSPA_AUD_CTRL_S2_CLK_SEL_AUDIO_PLL;
		}
		__raw_writel(val, SSPA_AUD_CTRL);

		pr_debug("aud_ctrl = %x, aud_pll_ctrl0 = %x, "
			"aud_pll_ctrl1 = %x\n",
			__raw_readl(SSPA_AUD_CTRL),
			__raw_readl(SSPA_AUD_PLL_CTRL0),
			__raw_readl(SSPA_AUD_PLL_CTRL1));
		break;
	case SSPA_I2S_PLL:
		break;
	default:
		return -ENODEV;
	}

	priv->freq = freq_out;
	return 0;
}

/*
 * Set up the sspa dai format. The sspa port must be inactive
 * before calling this function as the physical
 * interface format is changed.
 */
static int mmp2_sspa_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				 unsigned int fmt)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	u32 sspa_sp, sspa_ctrl;

	pr_debug("%s: enter\n", __func__);

	/* check if we need to change anything at all */
	if (priv->dai_fmt == fmt)
		return 0;

	/* we can only change the settings if the port is not in use */
	if ((mmp2_sspa_read_reg(sspa, SSPA_TXSP) & SSPA_SP_S_EN) ||
	    (mmp2_sspa_read_reg(sspa, SSPA_RXSP) & SSPA_SP_S_EN)) {
		dev_err(&sspa->pdev->dev,
			"can't change hardware dai format: stream is in use\n");
		return -EINVAL;
	}

	/* reset port settings */
	sspa_sp   = SSPA_SP_WEN | SSPA_SP_S_RST | SSPA_SP_FFLUSH;
	sspa_ctrl = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		sspa_sp |= SSPA_SP_MSL;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		sspa_sp |= SSPA_SP_FSP;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		sspa_ctrl |= SSPA_CTL_XDATDLY(1);
		break;
	default:
		return -EINVAL;
	}

	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);

	sspa_sp &= ~(SSPA_SP_S_RST | SSPA_SP_FFLUSH);
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);

	/*
	 * FIXME: hw issue, for the tx serial port,
	 * can not config the master/slave mode;
	 * so must clean this bit.
	 * The master/slave mode has been set in the
	 * rx port.
	 */

	mmp2_sspa_write_reg(sspa, SSPA_TXCTL, sspa_ctrl);
	mmp2_sspa_write_reg(sspa, SSPA_RXCTL, sspa_ctrl);

	mmp2_sspa_dump_reg(sspa);

	/* Since we are configuring the timings for the format by hand
	 * we have to defer some things until hw_params() where we
	 * know parameters like the sample size.
	 */
	priv->dai_fmt = fmt;
	return 0;
}

/*
 * Set the SSPA audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int mmp3_sspa_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	struct mmp2_adma_params *dma_data;
	u32 sspa_ctrl, sspa_sp, as_width;
	u32 bits_per_frame, word_size;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sspa_ctrl = mmp2_sspa_read_reg(sspa, SSPA_TXCTL);
		sspa_sp   = mmp2_sspa_read_reg(sspa, SSPA_TXSP);
	} else {
		sspa_ctrl = mmp2_sspa_read_reg(sspa, SSPA_RXCTL);
		sspa_sp   = mmp2_sspa_read_reg(sspa, SSPA_RXSP);
	}

	sspa_ctrl &= ~SSPA_CTL_XFRLEN1_MASK;
	sspa_ctrl |= SSPA_CTL_XFRLEN1(params_channels(params) - 1);
	sspa_ctrl &= ~SSPA_CTL_XWDLEN1_MASK;
	sspa_ctrl &= ~SSPA_CTL_XSSZ1_MASK;

	sspa_sp &= ~SSPA_SP_FPER_MASK;
	sspa_sp &= ~SSPA_SP_FWID_MASK;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		word_size = SSPA_CTL_8_BITS;
		bits_per_frame = 16;
		as_width = TDCR_SSZ_8_BITS;
	case SNDRV_PCM_FORMAT_S16_LE:
		word_size = SSPA_CTL_16_BITS;
		bits_per_frame = 32;
		as_width = TDCR_SSZ_16_BITS;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		word_size = SSPA_CTL_24_BITS;
		bits_per_frame = 48;
		as_width = TDCR_SSZ_24_BITS;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		word_size = SSPA_CTL_32_BITS;
		bits_per_frame = 64;
		as_width = TDCR_SSZ_32_BITS;
		break;
	default:
		return -EINVAL;
	}

	sspa_ctrl |= SSPA_CTL_XSSZ1(word_size) |
		     SSPA_CTL_XWDLEN1(word_size);
	sspa_sp |= SSPA_SP_FPER(bits_per_frame - 1) |
		     SSPA_SP_FWID((bits_per_frame >> 1) - 1);

	sspa_sp |= SSPA_SP_WEN;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mmp2_sspa_write_reg(sspa, SSPA_TXCTL, sspa_ctrl);
		mmp2_sspa_write_reg(sspa, SSPA_TXFIFO_LL, 0x1);
		mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
	} else {
		mmp2_sspa_write_reg(sspa, SSPA_RXCTL, sspa_ctrl);
		mmp2_sspa_write_reg(sspa, SSPA_RXFIFO_UL, 0x0);
		mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);
	}

	mmp2_sspa_dump_reg(sspa);

	dma_data = mmp2_sspa_get_dma_params(cpu_dai->id, sspa, as_width,
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);

	return 0;
}

static int mmp3_sspa_trigger(struct snd_pcm_substream *substream,
			     int cmd, struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = priv->sspa;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		/*
		 * whatever playback or capture, must enable rx.
		 * this is a hw issue, so need check if rx has been
		 * enabled or not; if has been enabled by another
		 * stream, do not enable again.
		 */
		if (!priv->running_cnt)
			mmp2_sspa_rx_enable(sspa);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mmp2_sspa_tx_enable(sspa);

		priv->running_cnt++;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		priv->running_cnt--;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mmp2_sspa_tx_disable(sspa);

		/* have no capture stream, disable rx port */
		if (!priv->running_cnt)
			mmp2_sspa_rx_disable(sspa);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

#define PXA688_SSPA_RATES 0xffffffff
#define PXA688_SSPA_FORMATS 0xffffffff

static struct snd_soc_dai_ops mmp3_sspa_dai_ops = {
	.startup = mmp3_sspa_startup,
	.shutdown = mmp3_sspa_shutdown,
	.trigger = mmp3_sspa_trigger,
	.hw_params = mmp3_sspa_hw_params,
	.set_sysclk = mmp3_sspa_set_dai_sysclk,
	.set_clkdiv	= mmp2_sspa_set_dai_clkdiv,
	.set_pll	= mmp2_sspa_set_dai_pll,
	.set_fmt	= mmp2_sspa_set_dai_fmt,
};

static int mmp3_sspa_probe(struct snd_soc_dai *dai)
{
	struct sspa_priv *priv;
	int ret;

	priv = kzalloc(sizeof(struct sspa_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->sspa = sspa_request(dai->id + 1, "SSPA");
	if (priv->sspa == NULL) {
		ret = -ENODEV;
		goto err_priv;
	}

	priv->dai_fmt = (unsigned int)-1;
	snd_soc_dai_set_drvdata(dai, priv);

	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static int mmp3_sspa_remove(struct snd_soc_dai *dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(dai);

	sspa_free(priv->sspa);
	kfree(priv);
	return 0;
}

struct snd_soc_dai_driver mmp3_sspa_dai = {
	.probe = mmp3_sspa_probe,
	.remove = mmp3_sspa_remove,
	.suspend = mmp3_sspa_suspend,
	.resume = mmp3_sspa_resume,
	.playback = {
		     .channels_min = 1,
		     .channels_max = 128,
		     .rates = PXA688_SSPA_RATES,
		     .formats = PXA688_SSPA_FORMATS,},
	.capture = {
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = PXA688_SSPA_RATES,
		    .formats = PXA688_SSPA_FORMATS,},
	.ops = &mmp3_sspa_dai_ops,
};
EXPORT_SYMBOL_GPL(mmp3_sspa_dai);

static __devinit int asoc_sspa_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &mmp3_sspa_dai);
}

static int __devexit asoc_sspa_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver asoc_sspa_driver = {
	.driver = {
		   .name = "mmp3-sspa-dai",
		   .owner = THIS_MODULE,
		   },

	.probe = asoc_sspa_probe,
	.remove = __devexit_p(asoc_sspa_remove),
};

static int __init mmp3_sspa_modinit(void)
{
	return platform_driver_register(&asoc_sspa_driver);
}

module_init(mmp3_sspa_modinit);

static void __exit mmp3_sspa_exit(void)
{
	platform_driver_unregister(&asoc_sspa_driver);
}

module_exit(mmp3_sspa_exit);
/* Module information */
MODULE_AUTHOR("zhouqiao@marvell.com");
MODULE_DESCRIPTION("MMP3 SSPA SoC Interface");
MODULE_LICENSE("GPL");
