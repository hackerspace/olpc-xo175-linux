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
#include <asm/io.h>
#include <mach/regs-audio.h>
#include <mach/mmp_dma.h>
#include <plat/dma.h>
#include <plat/ssp.h>

#include "mmp-pcm.h"
#include "mmp-sspa.h"

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

static struct clk *audio_clk;
static struct clk *sysclk;

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

static int mmp2_sspa_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	int ret = 0;

	pr_debug("%s: active = %d id = %d\n", __func__,
		cpu_dai->active, cpu_dai->id);

	/* enable sspa clk */
	if (!cpu_dai->active)
		clk_enable(sspa->clk);

	return ret;
}

static void mmp2_sspa_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = sspa_priv->sspa;

	pr_debug("%s: active = %d id = %d\n", __func__,
		cpu_dai->active, cpu_dai->id);

	/* disable sspa clk */
	if (!cpu_dai->active)
		clk_disable(sspa->clk);

	return;
}

#ifdef CONFIG_PM

static uint32_t aud_ctrl;
static uint32_t aud_pll_ctrl0;
static uint32_t aud_pll_ctrl1;

static int mmp2_sspa_suspend(struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = sspa_priv->sspa;

	pr_debug("%s: sspa id = %d\n", __func__, cpu_dai->id);

	/* other card's dai link has suspended already */
	if (sspa_priv->suspend)
		return 0;

	if (!cpu_dai->active)
		clk_enable(sspa->clk);

	sspa_priv->rxctrl      = mmp2_sspa_read_reg(sspa, SSPA_RXCTL);
	sspa_priv->rxsp        = mmp2_sspa_read_reg(sspa, SSPA_RXSP);
	sspa_priv->rxfifo_ul   = mmp2_sspa_read_reg(sspa, SSPA_RXFIFO_UL);
	sspa_priv->rxint_mask  = mmp2_sspa_read_reg(sspa, SSPA_RXINT_MASK);

	sspa_priv->txctrl      = mmp2_sspa_read_reg(sspa, SSPA_TXCTL);
	sspa_priv->txsp        = mmp2_sspa_read_reg(sspa, SSPA_TXSP);
	sspa_priv->txfifo_ll   = mmp2_sspa_read_reg(sspa, SSPA_TXFIFO_LL);
	sspa_priv->txint_mask  = mmp2_sspa_read_reg(sspa, SSPA_TXINT_MASK);

	pr_debug("rxctrl %x rxsp %x rxfifo_ul %x rxint_mask %x\n",
		sspa_priv->rxctrl, sspa_priv->rxsp,
		sspa_priv->rxfifo_ul, sspa_priv->rxint_mask);
	pr_debug("txctrl %x txsp %x txfifo_ll %x txint_mask %x\n",
		sspa_priv->txctrl, sspa_priv->txsp,
		sspa_priv->txfifo_ll, sspa_priv->txint_mask);

	mmp2_sspa_tx_disable(sspa);
	mmp2_sspa_rx_disable(sspa);
	clk_disable(sspa->clk);
	sspa_priv->suspend = 1;
	return 0;
}

static int mmp2_sspa_resume(struct snd_soc_dai *cpu_dai)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	u32 sspa_sp;

	pr_debug("%s: sspa id = %d\n", __func__, cpu_dai->id);

	if (!sspa_priv->suspend)
		return 0;

	clk_enable(sspa->clk);

	pr_debug("%s: aud_ctrl = %x, aud_pll_ctrl0 = %x, "
			"aud_pll_ctrl1 = %x\n",
			__func__,
			__raw_readl(AUD_CTL),
			__raw_readl(AUD_PLL_CTL0),
			__raw_readl(AUD_PLL_CTL1));

	pr_debug("rxctrl %x rxsp %x rxfifo_ul %x rxint_mask %x\n",
		sspa_priv->rxctrl, sspa_priv->rxsp,
		sspa_priv->rxfifo_ul, sspa_priv->rxint_mask);
	pr_debug("txctrl %x txsp %x txfifo_ll %x txint_mask %x\n",
		sspa_priv->txctrl, sspa_priv->txsp,
		sspa_priv->txfifo_ll, sspa_priv->txint_mask);

	sspa_sp = sspa_priv->rxsp | SSPA_SP_WEN |
		  SSPA_SP_FFLUSH | SSPA_SP_S_RST;
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_RXCTL, sspa_priv->rxctrl);
	mmp2_sspa_write_reg(sspa, SSPA_RXFIFO_UL, sspa_priv->rxfifo_ul);
	mmp2_sspa_write_reg(sspa, SSPA_RXINT_MASK, sspa_priv->rxint_mask);
	sspa_sp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
	mmp2_sspa_write_reg(sspa, SSPA_RXSP, sspa_sp);

	sspa_sp = sspa_priv->txsp | SSPA_SP_WEN |
		  SSPA_SP_FFLUSH | SSPA_SP_S_RST;
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);
	mmp2_sspa_write_reg(sspa, SSPA_TXCTL, sspa_priv->txctrl);
	mmp2_sspa_write_reg(sspa, SSPA_TXFIFO_LL, sspa_priv->txfifo_ll);
	mmp2_sspa_write_reg(sspa, SSPA_TXINT_MASK, sspa_priv->txint_mask);
	sspa_sp &= ~(SSPA_SP_FFLUSH | SSPA_SP_S_RST);
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);

	if (!cpu_dai->active)
		clk_disable(sspa->clk);

	mmp2_sspa_dump_reg(sspa);
	sspa_priv->suspend = 0;
	return 0;
}

#else
#define mmp2_sspa_suspend	NULL
#define mmp2_sspa_resume	NULL
#endif

/*
 * Set the SSP ports SYSCLK.
 */
static int mmp2_sspa_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				    int clk_id, unsigned int freq, int dir)
{
	switch (clk_id) {
	case MMP_SSPA_CLK_AUDIO:
		clk_set_rate(audio_clk, freq);
		clk_enable(audio_clk);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Set the SSP clock dividers.
 */
static int mmp2_sspa_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				    int div_id, int div)
{
	return 0;
}

static int mmp2_sspa_set_dai_pll(struct snd_soc_dai *cpu_dai, int pll_id,
				 int source, unsigned int freq_in,
				 unsigned int freq_out)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = sspa_priv->sspa;

	switch (pll_id) {
	case MMP_SYSCLK:
		clk_set_rate(sysclk, freq_out);
		clk_enable(sysclk);
		break;
	case MMP_SSPA_CLK:
		clk_set_rate(sspa->clk, freq_out);
		break;
	default:
		return -ENODEV;
	}

	pr_debug("%s: actrl %x ap_ctrl0 %x ap_ctrl1 %x\n",
		__func__, __raw_readl(AUD_CTL),
		__raw_readl(AUD_PLL_CTL0), __raw_readl(AUD_PLL_CTL1));

	sspa_priv->freq = freq_out;
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
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(cpu_dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	u32 sspa_sp, sspa_ctrl;

	pr_debug("%s: enter\n", __func__);

	/* check if we need to change anything at all */
	if (sspa_priv->dai_fmt == fmt)
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
	case SND_SOC_DAIFMT_CBM_CFS:
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
		sspa_sp |= SSPA_TXSP_FPER(63);
		sspa_sp |= SSPA_SP_FWID(31);
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
	sspa_sp &= ~SSPA_SP_MSL;
	mmp2_sspa_write_reg(sspa, SSPA_TXSP, sspa_sp);

	mmp2_sspa_write_reg(sspa, SSPA_TXCTL, sspa_ctrl);
	mmp2_sspa_write_reg(sspa, SSPA_RXCTL, sspa_ctrl);

	mmp2_sspa_dump_reg(sspa);

	/* Since we are configuring the timings for the format by hand
	 * we have to defer some things until hw_params() where we
	 * know parameters like the sample size.
	 */
	sspa_priv->dai_fmt = fmt;
	return 0;
}

/*
 * Set the SSPA audio DMA parameters and sample size.
 * Can be called multiple times by oss emulation.
 */
static int mmp2_sspa_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
	struct mmp2_adma_params *dma_data;
	u32 sspa_ctrl, as_width;

	pr_debug("%s: enter\n", __func__);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		sspa_ctrl = mmp2_sspa_read_reg(sspa, SSPA_TXCTL);
	else
		sspa_ctrl = mmp2_sspa_read_reg(sspa, SSPA_RXCTL);

	sspa_ctrl &= ~SSPA_CTL_XFRLEN1_MASK;
	sspa_ctrl |= SSPA_CTL_XFRLEN1(params_channels(params) - 1);
	sspa_ctrl &= ~SSPA_CTL_XWDLEN1_MASK;
	sspa_ctrl |= SSPA_CTL_XWDLEN1(SSPA_CTL_32_BITS);
	sspa_ctrl &= ~SSPA_CTL_XSSZ1_MASK;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		sspa_ctrl |= SSPA_CTL_XSSZ1(SSPA_CTL_8_BITS);
		as_width = TDCR_SSZ_8_BITS;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		sspa_ctrl |= SSPA_CTL_XSSZ1(SSPA_CTL_16_BITS);
		as_width = TDCR_SSZ_16_BITS;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		sspa_ctrl |= SSPA_CTL_XSSZ1(SSPA_CTL_20_BITS);
		as_width = TDCR_SSZ_20_BITS;
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
		sspa_ctrl |= SSPA_CTL_XSSZ1(SSPA_CTL_24_BITS);
		as_width = TDCR_SSZ_24_BITS;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		sspa_ctrl |= SSPA_CTL_XSSZ1(SSPA_CTL_32_BITS);
		as_width = TDCR_SSZ_32_BITS;
		break;
	default:
		return -EINVAL;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mmp2_sspa_write_reg(sspa, SSPA_TXCTL, sspa_ctrl);
		mmp2_sspa_write_reg(sspa, SSPA_TXFIFO_LL, 0x1);
	} else {
		mmp2_sspa_write_reg(sspa, SSPA_RXCTL, sspa_ctrl);
		mmp2_sspa_write_reg(sspa, SSPA_RXFIFO_UL, 0x0);
	}

	mmp2_sspa_dump_reg(sspa);

	dma_data = mmp2_sspa_get_dma_params(cpu_dai->id, sspa, as_width,
			substream->stream == SNDRV_PCM_STREAM_PLAYBACK);
	snd_soc_dai_set_dma_data(cpu_dai, substream, dma_data);
	return 0;
}

static int mmp2_sspa_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct sspa_priv *sspa_priv = snd_soc_dai_get_drvdata(dai);
	struct ssp_device *sspa = sspa_priv->sspa;
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
		if (!sspa_priv->running_cnt)
			mmp2_sspa_rx_enable(sspa);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mmp2_sspa_tx_enable(sspa);

		sspa_priv->running_cnt++;
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		sspa_priv->running_cnt--;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			mmp2_sspa_tx_disable(sspa);

		/* have no capture stream, disable rx port */
		if (!sspa_priv->running_cnt)
			mmp2_sspa_rx_disable(sspa);
		break;

	default:
		ret = -EINVAL;
	}

	return ret;
}

static int mmp2_sspa_probe(struct snd_soc_dai *dai)
{
	struct sspa_priv *priv;
	int ret;

	/*
	 * for sspa may be linked to different codecs;
	 * such as sspa1 linked to hdmi and codec.
	 */
	priv = snd_soc_dai_get_drvdata(dai);
	if (priv) {
		printk(KERN_WARNING "%s: this port has been linked\n",
			__func__);
		return 0;
	}

	priv = kzalloc(sizeof(struct sspa_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->sspa = sspa_request(dai->id + 1, "mmp-sspa");

	if (priv->sspa == NULL) {
		ret = -ENODEV;
		goto err_priv;
	}

	priv->dai_fmt = (unsigned int) -1;
	priv->suspend = 0;
	snd_soc_dai_set_drvdata(dai, priv);
	return 0;

err_priv:
	kfree(priv);
	return ret;
}

static int mmp2_sspa_remove(struct snd_soc_dai *dai)
{
	struct sspa_priv *priv = snd_soc_dai_get_drvdata(dai);

	sspa_free(priv->sspa);
	kfree(priv);
	return 0;
}

#define MMP2_SSPA_RATES SNDRV_PCM_RATE_8000_192000
#define MMP2_SSPA_FORMATS (SNDRV_PCM_FMTBIT_S8 | \
		SNDRV_PCM_FMTBIT_S16_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S24_LE | \
		SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_ops mmp2_sspa_dai_ops = {
	.startup	= mmp2_sspa_startup,
	.shutdown	= mmp2_sspa_shutdown,
	.trigger	= mmp2_sspa_trigger,
	.hw_params	= mmp2_sspa_hw_params,
	.set_sysclk	= mmp2_sspa_set_dai_sysclk,
	.set_clkdiv	= mmp2_sspa_set_dai_clkdiv,
	.set_pll	= mmp2_sspa_set_dai_pll,
	.set_fmt	= mmp2_sspa_set_dai_fmt,
};

struct snd_soc_dai_driver mmp2_sspa_dai = {
	.probe = mmp2_sspa_probe,
	.remove = mmp2_sspa_remove,
	.suspend = mmp2_sspa_suspend,
	.resume = mmp2_sspa_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 128,
		.rates = MMP2_SSPA_RATES,
		.formats = MMP2_SSPA_FORMATS,
	},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = MMP2_SSPA_RATES,
		.formats = MMP2_SSPA_FORMATS,
	},
	.ops = &mmp2_sspa_dai_ops,
};
EXPORT_SYMBOL_GPL(mmp2_sspa_dai);

static __devinit int asoc_mmp_sspa_probe(struct platform_device *pdev)
{
	return snd_soc_register_dai(&pdev->dev, &mmp2_sspa_dai);
}

static int __devexit asoc_mmp_sspa_remove(struct platform_device *pdev)
{
	snd_soc_unregister_dai(&pdev->dev);
	return 0;
}

static struct platform_driver asoc_mmp_sspa_driver = {
	.driver = {
			.name = "mmp-sspa-dai",
			.owner = THIS_MODULE,
	},

	.probe = asoc_mmp_sspa_probe,
	.remove = __devexit_p(asoc_mmp_sspa_remove),
};


static int __init mmp2_sspa_modinit(void)
{
	audio_clk = clk_get(NULL, "mmp-audio");
	if (IS_ERR(audio_clk))
		return PTR_ERR(audio_clk);

	sysclk = clk_get(NULL, "mmp-sysclk");
	if (IS_ERR(sysclk))
		return PTR_ERR(sysclk);

	return platform_driver_register(&asoc_mmp_sspa_driver);
}
module_init(mmp2_sspa_modinit);

static void __exit mmp2_sspa_exit(void)
{
	clk_put(sysclk);
	clk_put(audio_clk);
	platform_driver_unregister(&asoc_mmp_sspa_driver);
}
module_exit(mmp2_sspa_exit);

/* Module information */
MODULE_AUTHOR("leoy@marvell.com");
MODULE_DESCRIPTION("MMP2 SSPA SoC Interface");
MODULE_LICENSE("GPL");
