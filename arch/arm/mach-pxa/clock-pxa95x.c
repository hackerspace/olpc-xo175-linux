/*
 *  linux/arch/arm/mach-pxa/clock-pxa95x.c
 *
 *  Author:	Xiaoguang Chen <chenxg@marvell.com>
 *
 *  Copyright:	(C) 2011 Marvell International Ltd.
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <plat/clock.h>
#include <mach/pxa3xx-regs.h>
#include <mach/debug_pm.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/dvfs.h>
#include "dsi_hdmi_pll.h"
#include "clock.h"

#define HZ_TO_KHZ	1000

extern unsigned long max_gc, max_vmeta;
struct clk_table {
	unsigned long fclk;
};

#define GCU_CLK_NUM	4
static struct clk_table pxa978_gcu_clk_table[] = {
	{156000},
	{312000},
	{498000},
	{600000},
	{0},
};

static struct clk_table pxa978_dx_gcu_clk_table[] = {
	{156000},
	{312000},
	{481000},
	{600000},
	{0},
};

static void common_clk_init(struct clk *c)
{
	c->dynamic_change = 1;
}

int clk_pxa3xx_cken_nand_enable(struct clk *clk)
{
	CKENA |= (1 << CKEN_NAND);
	return 0;
}

void clk_pxa3xx_cken_nand_disable(struct clk *clk)
{
	CKENA &= ~(1 << CKEN_NAND);
}

/*
 * Return the NAND clock frequency
 * PXA320/PXA930:     104 MHz
 * PXA935/PXA95x:     156 MHz
 * PXA300/310: 208 MHz
 */
static unsigned long clk_pxa3xx_nand_getrate(struct clk *clk)
{
	if (cpu_is_pxa320() || cpu_is_pxa930())
		return 104 * 1000 * 1000;
	else if (cpu_is_pxa95x() && (!cpu_is_pxa930()))
		return 156 * 1000 * 1000;
	else
		return 208 * 1000 * 1000;
}

static const struct clkops clk_pxa3xx_nand_ops = {
	.enable = clk_pxa3xx_cken_nand_enable,
	.disable = clk_pxa3xx_cken_nand_disable,
	.getrate = clk_pxa3xx_nand_getrate,
};

static int clk_tout_s0_enable(struct clk *clk)
{
	OSCC |= OSCC_TENS0;
	return 0;
}

static void clk_tout_s0_disable(struct clk *clk)
{
	OSCC &= ~OSCC_TENS0;
}

const struct clkops clk_pxa95x_tout_s0_ops = {
	.enable = clk_tout_s0_enable,
	.disable = clk_tout_s0_disable,
};

#define MIPI_BANDGAP_CONTROL	(1 << 10)
#define USB_BANDGAP_CONTROL	(1 << 11)
static volatile u32 *gen_reg4;
#define GEN_REG4 0x42404078
u32 get_mipi_reference_control(void)
{
	if (!gen_reg4)
		gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
	return *gen_reg4;
}

void set_mipi_reference_control(void)
{
	if (!gen_reg4)
		gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
	*gen_reg4 |= (MIPI_BANDGAP_CONTROL);
	udelay(1);
	pr_info("set mipi band gap reference control bit\n");
}

static int dsi_enable_status;
static int csi_enable_status;
static int hdmi_enable_status;

void clear_mipi_reference_control(void)
{
	if ((dsi_enable_status == 0)
	    && (csi_enable_status == 0)
	    && (hdmi_enable_status == 0)) {
		if (!gen_reg4)
			gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
		*gen_reg4 &= ~(MIPI_BANDGAP_CONTROL);
		pr_info("clear mipi band gap reference control bit\n");
	}
}

static int clk_pxa95x_dsi_enable(struct clk *dsi_clk)
{
	set_mipi_reference_control();
	dsi_enable_status = 1;

	if (!cpu_is_pxa978()) {
		struct DSIRegisters *p_Regs =
		    get_dsi_pll(dsi_clk->enable_val == CKEN_DSI_TX2);
		/*Enable PLL with internal timer */
		p_Regs->DSI_REG3 = 0x05cc60a2;
		p_Regs->DSI_REG1 =
		    DSI_REG1_DSIx_PLL_LOCK_SEL | DSI_REG1_DSIx_PLL_ON;
	}

	pr_info("dsi clock enable\n");
	CKENC |= 1 << (dsi_clk->enable_val - 64);

	return 0;
}

static void clk_pxa95x_dsi_disable(struct clk *dsi_clk)
{
	CKENC &= ~(1 << (dsi_clk->enable_val - 64));

	if (!cpu_is_pxa978()) {
		struct DSIRegisters *p_Regs =
		    get_dsi_pll(dsi_clk->enable_val == CKEN_DSI_TX2);
		/*Disable PLL with internal timer */
		/* JIRA: MG1-1265 dis pll could only be disabled
		   by setting reg3 to this value */
		p_Regs->DSI_REG3 = 0x040c60a2;
		p_Regs->DSI_REG1 = DSI_REG1_DSIx_PLL_LOCK_SEL;
	}

	dsi_enable_status = 0;
	clear_mipi_reference_control();
	pr_info("dsi clock disable\n");

	return;
}

/* refdiv, fbdiv, vcodiv_sel_se, vco_vrng, kvco_range */
static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_202 = {
	3, 140, 7, 0, 1
};

static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_217 = {
	3, 150, 7, 0, 1
};

static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_594 = {
	3, 171, 3, 1, 2
};

static void dsi_set_clock_nevo(u32 dsi_clk_val)
{
	struct DSIRegisters_Nevo *p_Regs = get_dsi_pll_nevo();
	u32 fbdiv, refdiv, outdiv, vcodiv_sel_se, kvco_range, vco_vrng, kvco;

	/*
	 * This code only allow DSI clock frequencies from the
	 * values defined in LCD_Controller_DSI_Clock enum.
	 * It is possible to remove this limitation and work
	 * directly with continuous numerical values. HOWEVER,
	 * if doing so PHY timing configuration MUST ALSO
	 * BE MODIFIED with the needed calcultaion to comply
	 * with non-fixed values.
	 * */
	/*Make sure dsi_clk is within range */
	if (dsi_clk_val > 624) {
		pr_err("%s: BAD clock value used\n", __func__);
		return;
	}

	/*Disable PLL with internal timer */
	p_Regs->DSI_NEVO_PU = 0;

	/*Set default values */
	refdiv = 0x1;		/*Always Set to 00001 */
	if (dsi_clk_val < 150) {
		outdiv = 1;
		dsi_clk_val *= 2;
	} else {
		outdiv = 0;
	}
	/* For several frequencys, use optimal parameters from the table -
	   else, calculate according to spec */
	if (dsi_clk_val == 202) {
		refdiv = LCD_DSI_PLL_Freq_202.refdiv;
		fbdiv = LCD_DSI_PLL_Freq_202.fbdiv;
		vcodiv_sel_se = LCD_DSI_PLL_Freq_202.vcodiv_sel_se;
		vco_vrng = LCD_DSI_PLL_Freq_202.vco_vrng;
		kvco_range = LCD_DSI_PLL_Freq_202.kvco_range;
	} else if (dsi_clk_val == 217) {
		refdiv = LCD_DSI_PLL_Freq_217.refdiv;
		fbdiv = LCD_DSI_PLL_Freq_217.fbdiv;
		vcodiv_sel_se = LCD_DSI_PLL_Freq_217.vcodiv_sel_se;
		vco_vrng = LCD_DSI_PLL_Freq_217.vco_vrng;
		kvco_range = LCD_DSI_PLL_Freq_217.kvco_range;
	} else if (dsi_clk_val == 594) {
		refdiv = LCD_DSI_PLL_Freq_594.refdiv;
		fbdiv = LCD_DSI_PLL_Freq_594.fbdiv;
		vcodiv_sel_se = LCD_DSI_PLL_Freq_594.vcodiv_sel_se;
		vco_vrng = LCD_DSI_PLL_Freq_594.vco_vrng;
		kvco_range = LCD_DSI_PLL_Freq_594.kvco_range;
	} else {
		/* Step 1 - Choose REFDIV[4:0] for the reference clock divider
		   (M) */
		refdiv = 3;	/*default settings for all frequencies */

		/* Step 2 - Choose VCODIV_SEL_DIFF[3:0] for the differential
		   clock CLKOUTP/CLKOUTN */
		/* Step 3 - Choose KVCO[3:0] to get the right VCO frequency
		   This is only in the case of freq lower then 1200 */
		if (dsi_clk_val < 1200) {
			kvco = dsi_calc_kvco(dsi_clk_val, &kvco_range,
					     &vcodiv_sel_se);
		} else {
			kvco_range = dsi_get_kvco_range(dsi_clk_val);
			kvco = dsi_clk_val;
			vcodiv_sel_se = 0;
		}

		/* Step 4 - Choose KVCO[3:0] to get the right VCO frequency */
		vco_vrng = kvco_range - 1;

		/* Step 5 - Choose FBDIV[8:0] for feedback divider (N)
		   to satisfy FVCO = (REFCLK / M) * N */

		/*Calculate FBDIV according to dsi_clk_val */
		fbdiv = dsi_calc_fbdiv_Nevo(refdiv, kvco);
	}

	/*Assemble REG2 value */
	p_Regs->DSI_NEVO_CONFIG = DSI_NEVO_CONFIG_VCODIV_SEL_SE(vcodiv_sel_se)
	    | DSI_NEVO_CONFIG_VCO_VRNG(vco_vrng)
	    | DSI_NEVO_CONFIG_KVCO(kvco_range)
	    | DSI_NEVO_CONFIG_OUTDIV(outdiv)
	    | DSI_NEVO_CONFIG_FBDIV(fbdiv)
	    | DSI_NEVO_CONFIG_REFDIV(refdiv);

	/*Enable PLL with internal timer */
	p_Regs->DSI_NEVO_PU = DSI_NEVO_PU_PLL_ON;

	/*Need to at least wait 100us */
	mdelay(1);

	dsi_pll_locked_Nevo(10, p_Regs);
	return;
}

static void dsi_set_clock_pv2(u32 dsi_clk_val, u32 converter)
{
	struct DSIRegisters *p_Regs = get_dsi_pll(converter);
	u32 fbdiv, refdiv, outdiv, cksel;

	/*
	 * This code only allow DSI clock frequencies from the
	 * values defined in LCD_Controller_DSI_Clock enum.
	 * It is possible to remove this limitation and work
	 * directly with continuous numerical values. HOWEVER,
	 * if doing so PHY timing configuration MUST ALSO
	 * BE MODIFIED with the needed calcultaion to comply
	 * with non-fixed values.
	 * */
	/*Make sure dsi_clk is within range */
	if (dsi_clk_val > 624) {
		pr_err("%s: BAD clock value used\n", __func__);
		return;
	}

	/*Disable PLL with internal timer */
	p_Regs->DSI_REG1 = DSI_REG1_DSIx_PLL_LOCK_SEL;

	/*Set default values */
	refdiv = 0x1;		/*Always Set to 00001 */
	if (dsi_clk_val > 200) {
		outdiv = 0xF;
		if (dsi_clk_val < 400)
			cksel = 0x1;
		else
			cksel = 0x0;
	} else {
		outdiv = 16 - (200 / dsi_clk_val);
		cksel = 0x0;
	}

	/*Calculate FBDIV according to dsi_clk_val */
	fbdiv = dsi_calc_fbdiv(dsi_clk_val, outdiv, cksel);

	/*Assemble REG2 value */
	p_Regs->DSI_REG2 = DSI_REG2_DEFAULT | DSI_REG2_CK_SEL(cksel)
	    | DSI_REG2_OUTDIV(outdiv)
	    | DSI_REG2_FBDIV(fbdiv)
	    | DSI_REG2_REFDIV(refdiv);

	/*Enable PLL with internal timer */
	p_Regs->DSI_REG1 = DSI_REG1_DSIx_PLL_LOCK_SEL | DSI_REG1_DSIx_PLL_ON;

	/*Need to at least wait 100us */
	mdelay(1);
	return;
}

static int clk_pxa95x_dsi_setrate(struct clk *dsi_clk, unsigned long rate)
{
	int mrate = rate / 1000000;
	if (cpu_is_pxa978())
		dsi_set_clock_nevo(mrate);
	else
		dsi_set_clock_pv2(mrate, (dsi_clk->enable_val == CKEN_DSI_TX2));
	return 0;

}

static const struct clkops clk_pxa95x_dsi_ops = {
	.init = common_clk_init,
	.enable = clk_pxa95x_dsi_enable,
	.disable = clk_pxa95x_dsi_disable,
	.setrate = clk_pxa95x_dsi_setrate,
};

static void clk_pxa95x_ihdmi_init(struct clk *c)
{
	struct HDMIRegisters *p_Regs = get_ihdmi_pll();

	/* Power down hdmi reference current, it is useless */
	p_Regs->HDMI_PHY_CTL1 |= HDMI_PHY_CTL1_REG_PD_IREF(1);

	common_clk_init(c);
}
/* TODO: hdmi cken? */
static int clk_pxa95x_ihdmi_enable(struct clk *hdmi_clk)
{
	set_mipi_reference_control();
	hdmi_enable_status = 1;
	printk("hdmi: pll enable\n");
	return 0;
}

static void clk_pxa95x_ihdmi_disable(struct clk *hdmi_clk)
{
	struct HDMIRegisters *p_Regs = get_ihdmi_pll();
	/* Reset the phy */
	p_Regs->HDMI_PHY_CTL2 = 0x5;
	p_Regs->HDMI_PHY_CTL1 &= (0xFFF8007F);
	p_Regs->HDMI_PHY_CTL1 &= ~(HDMI_PHY_CTL1_REG_PD_IREF(0)
				   | HDMI_PHY_CTL1_REG_PD_TX(0xf));
	p_Regs->HDMI_PHY_CTL1 |= HDMI_PHY_CTL1_REG_RESET_TX_OFF;
	mdelay(1);
	p_Regs->HDMI_PHY_CTL1 &= HDMI_PHY_CTL1_REG_RESET_TX_ON;
	/* Reset the vpll HW state machine */
	p_Regs->HDMI_PLL_DEBUG2 &= ~HDMI_PLL_DEBUG2_PLL_CTRL_RSTN;
	mdelay(1);

	p_Regs->HDMI_PLL_PU = 0;
	mdelay(1);
	hdmi_enable_status = 0;
	clear_mipi_reference_control();
	printk("hdmi: pll disable\n");
}

/* place real turn on when setrate: enable -> setrate */
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_126 = {
	0x2, 0x4, 0x3, 77, 0x2, 0x0188D
};

static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_136 = {
	0x2, 0x4, 0x3, 83, 0x2, 0x003CC
};

static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_271 = {
	0x2, 0x4, 0x4, 83, 0x2, 0x003CC
};

static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_371 = {
	0x2, 0x4, 0x2, 114, 0x2, 0x084B
};

static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_742 = {
	0x2, 0x4, 0x1, 114, 0x2, 0x084B
};

static int clk_pxa95x_ihdmi_setrate(struct clk *clk, unsigned long rate)
{
	struct HDMIRegisters *p_Regs = get_ihdmi_pll();
	u32 hdmi_clk = rate / 1000000;

	struct HDMI_PLL_PARAMETERS *para;

	/* Make sure hdmi_clk is within range. */
	if (hdmi_clk > 800) {
		pr_err("%s HDMI clock not within range!!\n", __func__);
		return 0;
	}

	/* Initilize the phy */

	/* Reset the phy */
	p_Regs->HDMI_PHY_CTL2 = 0x5;
	p_Regs->HDMI_PHY_CTL1 &= (0xFFF8007F);
	p_Regs->HDMI_PHY_CTL1 &= ~(HDMI_PHY_CTL1_REG_PD_IREF(0)
				   | HDMI_PHY_CTL1_REG_PD_TX(0xf));
	p_Regs->HDMI_PHY_CTL1 |= HDMI_PHY_CTL1_REG_RESET_TX_OFF;
	mdelay(1);
	p_Regs->HDMI_PHY_CTL1 &= HDMI_PHY_CTL1_REG_RESET_TX_ON;
	/* Reset the vpll HW state machine */
	p_Regs->HDMI_PLL_DEBUG2 &= ~HDMI_PLL_DEBUG2_PLL_CTRL_RSTN;
	mdelay(1);
	/* Disable PLL */
	p_Regs->HDMI_PLL_PU = 0;
	mdelay(10);
	/* Release reset the vpll HW state machine */
	p_Regs->HDMI_PLL_DEBUG2 |= HDMI_PLL_DEBUG2_PLL_CTRL_RSTN;
	mdelay(1);
	/* Enable PLL */
	/* p_Regs->HDMI_PLL_PU = HDMI_PLL_PWR_PLL_ON; */
	p_Regs->HDMI_PHY_CTL1 = 0xC9200040;
	p_Regs->HDMI_PHY_CTL2 = 0x0000DDDD;
	p_Regs->HDMI_PLL_DEBUG1 |= 0x00024900;
	printk(KERN_INFO "hdmi: TMDS clk set to %dMhz\n", hdmi_clk / 5);
	/* For several frequencys, use optimal parameters from the table
	   - else, calculate according to spec */
	switch (hdmi_clk) {
	case 126:
		para = &hdmi_pll_freq_126;
		break;
	case 136:
		para = &hdmi_pll_freq_136;
		break;
	case 271:
		para = &hdmi_pll_freq_271;
		break;
	case 371:
		para = &hdmi_pll_freq_371;
		break;
	case 742:
		para = &hdmi_pll_freq_742;
		break;
	default:
		pr_err("%s Unknown freq value!\n", __func__);
		return 1;
		break;
	}

	/* Set HDMI PLL registers and enable panel and hdmi */
	p_Regs->HDMI_PLL_CONFIG1 = HDMI_PLL_CONFIG1_REFDIV(para->refdiv)
	    | HDMI_PLL_CONFIG1_FBDIV(para->fbdiv)
	    | HDMI_PLL_CONFIG1_POSTDIV_SEL(para->postdiv_sel)
	    | HDMI_PLL_CONFIG1_KVCO(para->kvco)
	    | HDMI_PLL_CONFIG1_INTPI(para->intpi)
	    | HDMI_PLL_CONFIG1_EN_HDMI | HDMI_PLL_CONFIG1_EN_PANEL;

	p_Regs->HDMI_PLL_CONFIG3 =
	    HDMI_PLL_CONFIG3_FREQ_OFFSET_INNER(para->freq_offset_inner);

	/* Enable PLL */
	p_Regs->HDMI_PLL_PU = HDMI_PLL_PWR_PLL_ON;
	/* p_Regs->HDMI_PLL_CONFIG1 |=
	   HDMI_PLL_CONFIG1_VPLL_CAL_START; */

	mdelay(10);

	/* Wait until PLL is locked */
	return !hdmi_pll_locked(10, p_Regs);

}

static const struct clkops clk_pxa95x_ihdmi_ops = {
	.init = clk_pxa95x_ihdmi_init,
	.enable = clk_pxa95x_ihdmi_enable,
	.disable = clk_pxa95x_ihdmi_disable,
	.setrate = clk_pxa95x_ihdmi_setrate,
};

static int clk_pxa95x_lcd_enable(struct clk *lcd_clk)
{
	CKENC |= (1 << (CKEN_DISPLAY - 64)) | (1 << (CKEN_PIXEL - 64));
	return 0;
}

static void clk_pxa95x_lcd_disable(struct clk *dsi_clk)
{
	CKENC &= ~((1 << (CKEN_DISPLAY - 64))
		   | (1 << (CKEN_PIXEL - 64)));
}

static long clk_pxa95x_lcd_round_rate(struct clk *lcd_clk, unsigned long rate)
{
	if (rate <= 104000000)
		rate = 104000000;
	else if (rate <= 156000000)
		rate = 156000000;
	else if (rate <= 208000000)
		rate = 208000000;
	else if (rate <= 312000000)
		rate = 312000000;
	else if (rate <= 416000000)
		rate = 416000000;
	else {
		printk(KERN_ERR "LCD don't support rate: %lu\n", rate);
		return -1;
	}
	return rate;
}

unsigned long clk_pxa95x_lcd_getrate(struct clk *lcd_clk)
{
	unsigned long rate;
	rate = ACCR0 & 0x7;
	switch (rate) {
	case 0:
		rate = 104;
		break;
	case 1:
		rate = 156;
		break;
	case 2:
		rate = 208;
		break;
	case 3:
		rate = 312;
		break;
	case 4:
		rate = 416;
		break;
	default:
		return -1;
	}
	return rate * 1000000;
}

static int clk_pxa95x_lcd_setrate(struct clk *lcd_clk, unsigned long rate)
{
	unsigned int value, mask = 0x7;
	switch (rate) {
	case 104000000:
		value = 0;
		break;
	case 156000000:
		value = 1;
		break;
	case 208000000:
		value = 2;
		break;
	case 312000000:
		value = 3;
		break;
	case 416000000:
		value = 4;
		break;
	default:
		printk(KERN_ERR "LCD don't suppport rate: %lu\n", rate);
		return -1;
	}
	write_accr0(value, mask);
	return 0;
}

static const struct clkops clk_pxa95x_lcd_ops = {
	.init = common_clk_init,
	.enable = clk_pxa95x_lcd_enable,
	.disable = clk_pxa95x_lcd_disable,
	.round_rate = clk_pxa95x_lcd_round_rate,
	.getrate = clk_pxa95x_lcd_getrate,
	.setrate = clk_pxa95x_lcd_setrate,
};

static int clk_axi_enable(struct clk *clk)
{
	CKENC |= (1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64));
	return 0;
}

static int is_wkr_dma_clock(void)
{
	if (cpu_is_pxa978())
		return 1;
	else
		return 0;
}

static void clk_axi_disable(struct clk *clk)
{
	if (is_wkr_dma_clock())
		CKENC &= ~(1 << (CKEN_AXI - 64));
	else
		CKENC &= ~((1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X - 64)));
}

static const struct clkops clk_axi_ops = {
	.enable = clk_axi_enable,
	.disable = clk_axi_disable,
};

static int clk_imu_axi_enable(struct clk *clk)
{
	CKENC |= (1 << (CKEN_IMU - 64));
	return 0;
}

static void clk_imu_axi_disable(struct clk *clk)
{
	CKENC &= ~(1 << (CKEN_IMU - 64));
}

static const struct clkops clk_imu_axi_ops = {
	.enable = clk_imu_axi_enable,
	.disable = clk_imu_axi_disable,
};

static int clk_pxa9xx_u2o_enable(struct clk *clk)
{
	local_irq_disable();

	if (!gen_reg4)
		gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
	*gen_reg4 |= USB_BANDGAP_CONTROL;

	CKENC |= (1 << (CKEN_USB_PRL - 64)) | (1 << (CKEN_USB_BUS - 64));
	/* ACCR1 */
	ACCR1 |= ACCR1_PU_OTG | ACCR1_PU_PLL | ACCR1_PU;

	local_irq_enable();
	return 0;
}

static void clk_pxa9xx_u2o_disable(struct clk *clk)
{
	local_irq_disable();

	CKENC &= ~((1 << (CKEN_USB_PRL - 64)) | (1 << (CKEN_USB_BUS - 64)));
	/* ACCR1 */
	ACCR1 &= ~(ACCR1_PU_OTG | ACCR1_PU_PLL | ACCR1_PU);

	if (!gen_reg4)
		gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
	*gen_reg4 &= ~USB_BANDGAP_CONTROL;

	local_irq_enable();
}

static const struct clkops clk_pxa9xx_u2o_ops = {
	.enable = clk_pxa9xx_u2o_enable,
	.disable = clk_pxa9xx_u2o_disable,
};

static void mm_pll_enable(unsigned int enable)
{
	if (enable) {
		MM_PLL_CTRL |= MMPLL_PWRON;
		while (!(MM_PLL_CTRL & MMPLL_PWR_ST))
			;
	} else
		MM_PLL_CTRL &= ~MMPLL_PWRON;
}

static inline unsigned int get_mm_pll_freq(void)
{
	unsigned int mmpllr, fbdiv, refdiv, vcodiv_sel, vcodiv, freq = 0;
	mmpllr = MM_PLL_PARAM;
	fbdiv = (mmpllr & MMPLL_FBDIV_MASK) >> 5;
	refdiv = mmpllr & MMPLL_REFDIV_MASK;
	vcodiv_sel = (mmpllr & MMPLL_VCODIV_SEL_MASK) >> 20;
	switch (vcodiv_sel) {
	case 0:
		vcodiv = 1;
		break;
	case 2:
		vcodiv = 2;
		break;
	case 5:
		vcodiv = 4;
		break;
	case 8:
		vcodiv = 8;
		break;
	default:
		pr_err("wrong vcodiv!\n");
		BUG_ON(1);
		break;
	}
	freq = (26 * fbdiv / refdiv) / vcodiv;
	return freq;
}

static inline unsigned int mm_pll_freq2reg(unsigned int x)
{
	switch (x) {
	case 481000000:
		/* VCODIV_SEL=5 KVCO=5 FBDIV=222(0xDE) REFDIV=3*/
		return 5 << 20 | 5 << 16 | 0xDE << 5 | 3 << 0;
	case 498000000:
		/* VCODIV_SEL=5 KVCO=5 FBDIV=230(0xE6) REFDIV=3 */
		return 5 << 20 | 5 << 16 | 0xE6 << 5 | 3 << 0;
	case 600000000:
		/* VCODIV_SEL=5 KVCO=7 FBDIV=277(0x115) REFDIV=3 */
		return 5 << 20 | 7 << 16 | 0x115 << 5 | 3 << 0;
	default:
		pr_err("Unsupported MM PLL frequency %d!\n", x);
		return 0;
	}
}

static inline void set_mmpll_freq(unsigned long rate)
{
	uint32_t mm_pll_param;
	if ((rate != 481000000) && (rate != 498000000) && (rate != 600000000))
		return;
	mm_pll_param = MM_PLL_PARAM;
	mm_pll_param &= ~(MMPLL_VCODIV_SEL_MASK
			  | MMPLL_KVCO_MASK
			  | MMPLL_FBDIV_MASK | MMPLL_REFDIV_MASK);
	mm_pll_param |= mm_pll_freq2reg(rate);

	MM_PLL_PARAM = mm_pll_param;
	while (!(MM_PLL_CTRL & MMPLL_PWR_ST))
		;
}

static int clk_mmpll_enable(struct clk *clk)
{
	mm_pll_enable(1);
	return 1;
}

static void clk_mmpll_disable(struct clk *clk)
{
	mm_pll_enable(0);
}

static unsigned long clk_mmpll_getrate(struct clk *mmpll)
{
	return get_mm_pll_freq() * 1000000;
}

static int clk_mmpll_setrate(struct clk *mmpll, unsigned long rate)
{
	set_mmpll_freq(rate);
	return 0;
}

static const struct clkops clk_mmpll_ops = {
	.enable = clk_mmpll_enable,
	.disable = clk_mmpll_disable,
	.getrate = clk_mmpll_getrate,
	.setrate = clk_mmpll_setrate,
};

extern struct dvfs gc_dvfs;
static int clk_gcu_enable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = 0;
	dvfs_freqs.new = clk->rate / HZ_TO_KHZ;
	dvfs_freqs.dvfs = &gc_dvfs;

	pr_debug("GC enable from 0 to %lu.\n", clk->rate);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);
	CKENC |= ((1 << (CKEN_GC_1X - 64)) | (1 << (CKEN_GC_2X - 64)));
	gc_vmeta_stats_clk_event(GC_CLK_ON);

	return 0;
}

static void clk_gcu_disable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = clk->rate / HZ_TO_KHZ;
	dvfs_freqs.new = 0;
	dvfs_freqs.dvfs = &gc_dvfs;

	pr_debug("GC disable from %lu to 0.\n", clk->rate);

	gc_vmeta_stats_clk_event(GC_CLK_OFF);
	CKENC &= ~((1 << (CKEN_GC_1X - 64)) | (1 << (CKEN_GC_2X - 64)));
	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
}

static long clk_pxa95x_gc_round_rate(struct clk *gc_clk, unsigned long rate)
{
	if (rate <= 156000000)
		rate = 156000000;
	else if (rate <= 208000000)
		rate = 208000000;
	else if (rate <= 312000000)
		rate = 312000000;
	else if (rate <= 416000000)
		rate = 416000000;
	else if ((rate <= 481000000) && cpu_is_pxa978_Dx())
		rate = 481000000;
	else if ((rate <= 498000000) && (!cpu_is_pxa978_Dx()))
		rate = 498000000;
	else
		rate = 600000000;

	return rate;
}

static unsigned long clk_pxa95x_gc_getrate(struct clk *gc_clk)
{
	unsigned long rate;
	rate = (ACCR0 >> 9) & 0x7;

	switch (rate) {
	case 0:
		rate = 208000000;
		break;
	case 1:
		rate = 156000000;
		break;
	case 2:
		rate = 312000000;
		break;
	case 3:
		rate = 416000000;
		break;
	case 5:
		rate = get_mm_pll_freq() * 1000000;
		break;
	default:
		return -1;
	}
	return rate;
}

#define GCVMETA_WR
/* flag = 1 means gc frequency change
 * flag = 0 means vmeta frequency change
 * current solution is change both GC and vMeta to 500/600
 * if GC or vMeta are at MMPLL and the other want to change
 * to MMPLL or change MMPLL's frequency.
 */
static struct clk clk_pxa95x_gcu, clk_pxa95x_vmeta;
static unsigned long clk_pxa95x_vmeta_getrate(struct clk *vmeta_clk);
static void mm_pll_setting(int flag, unsigned long rate, unsigned int value,
		unsigned int mask)
{
	unsigned int tmp, ori_gc, ori_vmeta;
	unsigned long gc_rate, vm_rate, flags;

	gc_rate = clk_pxa95x_gc_getrate(&clk_pxa95x_gcu);
	vm_rate = clk_pxa95x_vmeta_getrate(&clk_pxa95x_vmeta);

	pr_debug("mm_pll_setting: current gc is %lu, vmeta is %lu.\n"
		       "Set to %lu by %s.\n", gc_rate, vm_rate, rate,
		       flag ? "GC" : "vMeta");

	local_fiq_disable();
	local_irq_save(flags);
	if (rate < 481000000) {
		write_accr0(value, mask);
		if ((flag && (vm_rate < 481000000) && (gc_rate >= 481000000)) ||
		   (!flag && (gc_rate < 481000000) && (vm_rate >= 481000000)))
			mm_pll_enable(0);
		goto out;
	} else if (vm_rate < 481000000 && gc_rate < 481000000)
		mm_pll_enable(1);

	if (get_mm_pll_freq() * 1000000 == rate) {
		write_accr0(value, mask);
		goto out;
	}
	tmp = ACCR0;
	ori_gc = ACCR0 & (ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
	ori_vmeta = ACCR0 & ACCR0_VMFC_MASK;
	if (gc_rate >= 481000000) {
		tmp &= ~(ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
		tmp |= (0x3 << ACCR0_GCFS_OFFSET | 0x3 << ACCR0_GCAXIFS_OFFSET);
		if (!flag && (gc_rate != rate)) {
			value |= ori_gc;
			mask |= (ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
			clk_pxa95x_gcu.rate = rate;
		}
	}
	if (vm_rate >= 481000000) {
		tmp &= ~ACCR0_VMFC_MASK;
		tmp |= 0x3 << ACCR0_VMFC_OFFSET;
		if (flag && (vm_rate != rate)) {
			value |= ori_vmeta;
			mask |= ACCR0_VMFC_MASK;
			clk_pxa95x_vmeta.rate = rate;
		}
	}
	ACCR0 = tmp;
	set_mmpll_freq(rate);
	write_accr0(value, mask);
out:
	local_irq_restore(flags);
	local_fiq_enable();
}

int get_gcu_freqs_table(int *gcu_freqs_table, int *item_counts,
		int max_item_counts)
{
	int i;

	if (max_item_counts < GCU_CLK_NUM) {
		pr_err("Too many GC frequencies!\n");
		return -1;
	}

	*item_counts = GCU_CLK_NUM;

	for (i = 0; i < GCU_CLK_NUM; i++)
		if (cpu_is_pxa978() && pxa978_gcu_clk_table[i].fclk <= max_gc)
			gcu_freqs_table[i] = pxa978_gcu_clk_table[i].fclk;
		else if (pxa978_dx_gcu_clk_table[i].fclk <= max_gc)
			gcu_freqs_table[i] = pxa978_dx_gcu_clk_table[i].fclk;

	return 0;
}
EXPORT_SYMBOL(get_gcu_freqs_table);

static int clk_pxa95x_gcu_setrate(struct clk *gc_clk, unsigned long rate)
{
	unsigned int value, mask = 0x3F << 6;
	struct dvfs_freqs dvfs_freqs;

	pr_debug("gc setrate from %lu to %lu.\n", gc_clk->rate, rate);

	dvfs_freqs.old = gc_clk->rate / HZ_TO_KHZ;
	dvfs_freqs.new = rate / HZ_TO_KHZ;
	dvfs_freqs.dvfs = &gc_dvfs;
	if (dvfs_freqs.old < dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);

	switch (rate) {
	case 208000000:
		value = 0;
		break;
	case 156000000:
		value = (0x1 << 3) | 0x1;
		break;
	case 312000000:
		value = (0x2 << 3) | 0x2;
		break;
	case 416000000:
		value = (0x3 << 3) | 0x3;
		break;
	case 481000000:
	case 498000000:
	case 600000000:
		value = (0x5 << 3) | 0x5;
		break;
	default:
		printk(KERN_ERR "GC doesn't suppport rate: %lu\n", rate);
		return -1;
	}
#ifdef GCVMETA_WR
	mm_pll_setting(1, rate, value << 6, mask);
#endif

	if (dvfs_freqs.old > dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);

	return 0;
}

static const struct clkops clk_gcu_ops = {
	.init = common_clk_init,
	.enable = clk_gcu_enable,
	.disable = clk_gcu_disable,
	.round_rate = clk_pxa95x_gc_round_rate,
	.getrate = clk_pxa95x_gc_getrate,
	.setrate = clk_pxa95x_gcu_setrate,
};

static int clk_csi_tx_esc_enable(struct clk *csi_clk)
{
	set_mipi_reference_control();
	CKENC |= (1 << (CKEN_CSI_TX - 64));
	csi_enable_status = 1;
	pr_info("cam: csi clock enable\n");
	return 0;
}

static void clk_csi_tx_esc_disable(struct clk *csi_clk)
{
	CKENC &= ~(1 << (CKEN_CSI_TX - 64));
	csi_enable_status = 0;
	clear_mipi_reference_control();
	pr_info("cam: csi clock disable\n");
}

static const struct clkops clk_csi_tx_esc_ops = {
	.enable = clk_csi_tx_esc_enable,
	.disable = clk_csi_tx_esc_disable,
};

static int clk_pxa95x_sci_enable(struct clk *sci_clk)
{
	CKENC |= (1 << (CKEN_SCI1 - 64)) | (1 << (CKEN_SCI2 - 64));
	return 0;
}

static void clk_pxa95x_sci_disable(struct clk *sci_clk)
{
	CKENC &= ~(1 << (CKEN_SCI1 - 64) | (1 << (CKEN_SCI2 - 64)));
}

static const struct clkops clk_pxa95x_sci_ops = {
	.enable = clk_pxa95x_sci_enable,
	.disable = clk_pxa95x_sci_disable,
};

static int clk_mmc_enable(struct clk *clk)
{
	unsigned long mask;
	mask = 1 << (clk->enable_val - 64) | 1 << ((clk->enable_val - 64) + 13);
	CKENC |= mask;
	return 0;
}

static void clk_mmc_disable(struct clk *clk)
{
	unsigned long mask;
	mask =
	    ~(1 << (clk->enable_val - 64) | 1 << ((clk->enable_val - 64) + 13));
	CKENC &= mask;
}

static const struct clkops clk_mmc_ops = {
	.enable = clk_mmc_enable,
	.disable = clk_mmc_disable,
};

#define PWMCCR4 0x42404060
/* enable PWM SLOW clock */
int clk_pxa95x_pwm_slow_enable(struct clk *pwm_slow_clk)
{
	unsigned char __iomem *pwm_membase;
	u32 temp = 0;

	pwm_membase = ioremap(PWMCCR4 + pwm_slow_clk->enable_val, 4);
	temp = ioread32(pwm_membase);
	iowrite32(temp | PWMCLKEN_SLOW, pwm_membase);
	iounmap(pwm_membase);
	return 0;
}

/* disable PWM SLOW clock */
void clk_pxa95x_pwm_slow_disable(struct clk *pwm_slow_clk)
{
	unsigned char __iomem *pwm_membase;
	u32 temp = 0;

	pwm_membase = ioremap(PWMCCR4 + pwm_slow_clk->enable_val, 4);
	temp = ioread32(pwm_membase);
	iowrite32(temp & (~(PWMCLKEN_SLOW)), pwm_membase);
	iounmap(pwm_membase);
}

/*
 * Return the SLOW PWM Controller clock frequency
 */
static unsigned long clk_pxa95x_pwm_slow_getrate(struct clk *pwm_slow_clk)
{
	return 32768;
}

static const struct clkops clk_pxa95x_pwm_slow_ops = {
	.enable = clk_pxa95x_pwm_slow_enable,
	.disable = clk_pxa95x_pwm_slow_disable,
	.getrate = clk_pxa95x_pwm_slow_getrate,
};

int clk_pxa95x_smc_enable(struct clk *smc_clk)
{
	CKENA |= (1 << CKEN_SMC) | (1 << CKEN_NAND);
	return 0;
}

void clk_pxa95x_smc_disable(struct clk *smc_clk)
{
	CKENA &= ~((1 << CKEN_SMC) | (1 << CKEN_NAND));
}

/*
 * Return the Static Memory Controller clock frequency
 */
static unsigned long clk_pxa95x_smc_getrate(struct clk *clk)
{
	unsigned long acsr;
	unsigned int ro_s, smc_s, smc_clk = 0;

	acsr = ACSR;

	ro_s = (acsr >> 26) & 0x1;
	if (ro_s) {
		smc_clk = 15 * 1000 * 1000;
	} else {
		smc_s = (acsr >> 23) & 0x3;
		switch (smc_s) {
		case 0x0:
			smc_clk = 78 * 1000 * 1000;
			break;
		case 0x2:
			smc_clk = 104 * 1000 * 1000;
			break;
		case 0x5:
			smc_clk = 208 * 1000 * 1000;
			break;
		}
	}

	return smc_clk;
}

static const struct clkops clk_pxa95x_smc_ops = {
	.enable = clk_pxa95x_smc_enable,
	.disable = clk_pxa95x_smc_disable,
	.getrate = clk_pxa95x_smc_getrate,
};

static long clk_pxa95x_vmeta_round_rate(struct clk *vmeta_clk,
					unsigned long rate)
{
	if (rate <= 156000000)
		rate = 156000000;
	else if (rate <= 208000000)
		rate = 208000000;
	else if (rate <= 312000000)
		rate = 312000000;
	else if (rate <= 416000000)
		rate = 416000000;
	else if ((rate <= 481000000) && cpu_is_pxa978_Dx())
		rate = 481000000;
	else if ((rate <= 498000000) && (!cpu_is_pxa978_Dx()))
		rate = 498000000;
	else
		rate = 600000000;
	return rate;
}

static unsigned long clk_pxa95x_vmeta_getrate(struct clk *vmeta_clk)
{
	unsigned long rate;
	rate = (ACCR0 >> 3) & 0x7;
	switch (rate) {
	case 0:
		rate = 156000000;
		break;
	case 1:
		rate = 312000000;
		break;
	case 2:
		rate = 208000000;
		break;
	case 3:
		rate = 416000000;
		break;
	case 5:
		rate = get_mm_pll_freq() * 1000000;
		break;
	default:
		return -1;
	}
	return rate;
}

extern struct dvfs vmeta_dvfs;
static int clk_pxa95x_vmeta_setrate(struct clk *vmeta_clk, unsigned long rate)
{
	unsigned int value, mask = 0x7 << 3;
	struct dvfs_freqs dvfs_freqs;

	pr_debug("vmeta setrate from %lu to %lu.\n", vmeta_clk->rate, rate);

	dvfs_freqs.old = vmeta_clk->rate / HZ_TO_KHZ;
	dvfs_freqs.new = rate / HZ_TO_KHZ;
	dvfs_freqs.dvfs = &vmeta_dvfs;
	if (dvfs_freqs.old < dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);

	switch (rate) {
	case 156000000:
		value = 0;
		break;
	case 208000000:
		value = 2;
		break;
	case 312000000:
		value = 1;
		break;
	case 416000000:
		value = 3;
		break;
	case 481000000:
	case 498000000:
	case 600000000:
		value = 5;
		break;
	default:
		printk(KERN_ERR "VMeta don't suppport rate: %lu\n", rate);
		WARN_ON(1);
		return -1;
	}
#ifdef GCVMETA_WR
	mm_pll_setting(0, rate, value << 3, mask);
#endif

	if (dvfs_freqs.old > dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);

	return 0;
}

static int clk_pxa95x_vmeta_enable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = 0;
	dvfs_freqs.new = clk->rate / HZ_TO_KHZ;
	dvfs_freqs.dvfs = &vmeta_dvfs;

	pr_debug("vmeta enable from 0 to %lu.\n", clk->rate);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);
	CKENB |= (1 << (clk->enable_val - 32));
	gc_vmeta_stats_clk_event(VMETA_CLK_ON);

	return 0;
}

static void clk_pxa95x_vmeta_disable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = clk->rate / HZ_TO_KHZ;
	dvfs_freqs.new = 0;
	dvfs_freqs.dvfs = &vmeta_dvfs;

	pr_debug("vmeta disable from %lu to 0.\n", clk->rate);

	gc_vmeta_stats_clk_event(VMETA_CLK_OFF);
	CKENB &= ~(1 << (clk->enable_val - 32));
	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
}

static const struct clkops clk_pxa95x_vmeta_ops = {
	.init = common_clk_init,
	.disable = clk_pxa95x_vmeta_disable,
	.enable = clk_pxa95x_vmeta_enable,
	.round_rate = clk_pxa95x_vmeta_round_rate,
	.getrate = clk_pxa95x_vmeta_getrate,
	.setrate = clk_pxa95x_vmeta_setrate,
};

static DEFINE_CK(pxa95x_dsi0, DSI_TX1, &clk_pxa95x_dsi_ops);
static DEFINE_CK(pxa95x_dsi1, DSI_TX2, &clk_pxa95x_dsi_ops);
static DEFINE_CK(pxa95x_ihdmi, DISPLAY, &clk_pxa95x_ihdmi_ops);
static DEFINE_CK(pxa95x_axi, AXI, &clk_axi_ops);
static DEFINE_CK(pxa95x_smc, SMC, &clk_pxa95x_smc_ops);
static DEFINE_CK(pxa3xx_nand, NAND, &clk_pxa3xx_nand_ops);
static DEFINE_CK(pxa95x_pwm4, PWM4, &clk_pxa95x_pwm_slow_ops);
static DEFINE_CK(pxa95x_pwm5, PWM5, &clk_pxa95x_pwm_slow_ops);
static DEFINE_CK(pxa95x_pwm6, PWM6, &clk_pxa95x_pwm_slow_ops);
static DEFINE_CK(pxa95x_pwm7, PWM7, &clk_pxa95x_pwm_slow_ops);
static DEFINE_CLK(pxa95x_pout, &clk_pxa3xx_pout_ops, 13000000);
static DEFINE_CLK(pxa95x_tout_s0, &clk_pxa95x_tout_s0_ops, 13000000);

static DEFINE_PXA3_CKEN(pxa95x_ffuart, FFUART, 14857000);
static DEFINE_PXA3_CKEN(pxa95x_btuart, BTUART, 14857000);
static DEFINE_PXA3_CKEN(pxa95x_stuart, STUART, 14857000);
static DEFINE_PXA3_CKEN(pxa95x_i2c1, I2C, 32842000);
static DEFINE_PXA3_CKEN(pxa95x_i2c2, I2C2, 32842000);
static DEFINE_PXA3_CKEN(pxa95x_i2c3, I2C3, 32842000);
static DEFINE_PXA3_CKEN(pxa95x_keypad, KEYPAD, 32768);
static DEFINE_PXA3_CKEN(pxa95x_ssp1, SSP1, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_ssp2, SSP2, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_ssp3, SSP3, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_ssp4, SSP4, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_pwm0, PWM0, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_pwm1, PWM1, 13000000);
static DEFINE_PXA3_CKEN(pxa95x_abu, ABU, 20000000);
static DEFINE_PXA3_CKEN(pxa95x_gpio, GPIO, 0);
static DEFINE_PXA3_CKEN(pxa95x_bootrom, BOOT, 0);
static DEFINE_PXA3_CKEN(pxa95x_tpm, TPM, 0);

static struct clk *common_depend_clk[] = {
	&clk_pxa95x_axi,
};

static struct clk clk_pxa95x_imu = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_imu_axi_ops,
	.enable_val = CKEN_IMU,
};

static struct clk *mmc_usb_depend_clk[] = {
	&clk_pxa95x_imu,
};

static struct clk clk_pxa955_sdh0 = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_mmc_ops,
	.enable_val = CKEN_PXA95x_MMC1,
};

static struct clk clk_pxa955_sdh1 = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_mmc_ops,
	.enable_val = CKEN_PXA95x_MMC2,
};

static struct clk clk_pxa955_sdh2 = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_mmc_ops,
	.enable_val = CKEN_PXA95x_MMC3,
};

static struct clk clk_pxa955_sdh3 = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_mmc_ops,
	.enable_val = CKEN_PXA95x_MMC4,
};

static struct clk clk_mmc_bus = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_MMC_BUS,
};

static struct clk *mmc_depend_clk[] = {
	&clk_mmc_bus,
};

static struct clk clk_pxa978_sdh0 = {
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC1,
};

static struct clk clk_pxa978_sdh1 = {
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC2,
};

static struct clk clk_pxa978_sdh2 = {
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC3,
};

static struct clk clk_pxa978_sdh3 = {
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC4,
};

static struct clk clk_pxa95x_u2o = {
	.dependence = mmc_usb_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_usb_depend_clk),
	.ops = &clk_pxa9xx_u2o_ops,
	.enable_val = CKEN_USB_PRL,
};

static struct clk clk_pxa95x_gcu = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_gcu_ops,
	.enable_val = CKEN_GC_1X,
};

static struct clk clk_pxa978_gcu = {
	.ops = &clk_gcu_ops,
	.enable_val = CKEN_GC_1X,
};

static struct clk clk_pxa95x_csi_tx_esc = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_csi_tx_esc_ops,
	.enable_val = CKEN_CSI_TX,
};

static struct clk clk_pxa95x_sci1 = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_pxa95x_sci_ops,
	.enable_val = CKEN_SCI1,
};

static struct clk clk_pxa95x_sci2 = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_pxa95x_sci_ops,
	.enable_val = CKEN_SCI2,
};

static struct clk clk_pxa95x_vmeta = {
	.dependence = common_depend_clk,
	.dependence_count = ARRAY_SIZE(common_depend_clk),
	.ops = &clk_pxa95x_vmeta_ops,
	.enable_val = CKEN_VMETA,
};

static struct clk *lcd_depend_clk[] = {
	&clk_pxa95x_axi,
	&clk_pxa95x_tout_s0,
};

static struct clk clk_pxa95x_lcd = {
	.dependence = lcd_depend_clk,
	.dependence_count = ARRAY_SIZE(lcd_depend_clk),
	.ops = &clk_pxa95x_lcd_ops,
	.enable_val = CKEN_DISPLAY,
};

static struct clk clk_pxa978_mmpll = {
	.ops = &clk_mmpll_ops,
};

static struct clk_lookup common_clkregs[] = {

	INIT_CLKREG(&clk_pxa95x_pout, NULL, "CLK_POUT"),
	INIT_CLKREG(&clk_pxa95x_tout_s0, NULL, "CLK_TOUT_S0"),
	/* Power I2C clock is always on */
	INIT_CLKREG(&clk_pxa95x_ffuart, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_pxa95x_btuart, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_pxa95x_stuart, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_pxa95x_i2c1, "pxa95x-i2c.0", NULL),
	INIT_CLKREG(&clk_pxa95x_i2c2, "pxa95x-i2c.1", NULL),
	INIT_CLKREG(&clk_pxa95x_i2c3, "pxa95x-i2c.2", NULL),
	INIT_CLKREG(&clk_pxa95x_keypad, "pxa27x-keypad", NULL),
	INIT_CLKREG(&clk_pxa95x_ssp1, "pxa27x-ssp.0", NULL),
	INIT_CLKREG(&clk_pxa95x_ssp2, "pxa27x-ssp.1", NULL),
	INIT_CLKREG(&clk_pxa95x_ssp3, "pxa27x-ssp.2", NULL),
	INIT_CLKREG(&clk_pxa95x_ssp4, "pxa27x-ssp.3", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm0, "pxa27x-pwm.0", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm1, "pxa27x-pwm.1", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm4, "pxa95x-pwm.4", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm5, "pxa95x-pwm.5", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm6, "pxa95x-pwm.6", NULL),
	INIT_CLKREG(&clk_pxa95x_pwm7, "pxa95x-pwm.7", NULL),
	INIT_CLKREG(&clk_pxa95x_vmeta, NULL, "VMETA_CLK"),
	INIT_CLKREG(&clk_pxa95x_dsi0, NULL, "PXA95x_DSI0CLK"),
	INIT_CLKREG(&clk_pxa95x_dsi1, NULL, "PXA95x_DSI1CLK"),
	INIT_CLKREG(&clk_pxa95x_ihdmi, NULL, "HDMICLK"),
	INIT_CLKREG(&clk_pxa95x_lcd, "pxa95x-fb", "PXA95x_LCDCLK"),
	INIT_CLKREG(&clk_pxa95x_axi, NULL, "AXICLK"),
	INIT_CLKREG(&clk_pxa95x_imu, NULL, "IMUCLK"),
	INIT_CLKREG(&clk_pxa95x_u2o, NULL, "U2OCLK"),
	INIT_CLKREG(&clk_pxa95x_abu, NULL, "PXA95X_ABUCLK"),
	INIT_CLKREG(&clk_pxa95x_smc, NULL, "SMCCLK"),
	INIT_CLKREG(&clk_pxa3xx_nand, "pxa3xx-nand", NULL),
	INIT_CLKREG(&clk_pxa95x_gpio, NULL, "GPIOCLK"),
	INIT_CLKREG(&clk_pxa95x_bootrom, NULL, "BOOTCLK"),
	INIT_CLKREG(&clk_pxa95x_sci1, NULL, "SCI1CLK"),
	INIT_CLKREG(&clk_pxa95x_sci2, NULL, "SCI2CLK"),
	INIT_CLKREG(&clk_pxa95x_csi_tx_esc, NULL, "CSI_TX_ESC"),
	INIT_CLKREG(&clk_pxa95x_tpm, NULL, "TPM"),
};

static struct clk_lookup pxa955_specific_clkregs[] = {
	INIT_CLKREG(&clk_pxa955_sdh0, "sdhci-pxa.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa955_sdh1, "sdhci-pxa.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa955_sdh2, "sdhci-pxa.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa955_sdh3, "sdhci-pxa.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa95x_gcu, NULL, "GCCLK"),
};

static struct clk_lookup pxa978_specific_clkregs[] = {
	INIT_CLKREG(&clk_mmc_bus, NULL, "MMC_BUS"),
	INIT_CLKREG(&clk_pxa978_sdh0, "sdhci-pxa.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa978_sdh1, "sdhci-pxa.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa978_sdh2, "sdhci-pxa.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa978_sdh3, "sdhci-pxa.3", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa978_mmpll, NULL, "MM_PLL"),
	INIT_CLKREG(&clk_pxa978_gcu, NULL, "GCCLK"),
};

static inline void clock_lookup_init(struct clk_lookup *clk_lookup, int count)
{
	int i;
	for (i = 0; i < count; i++) {
		struct clk *c = clk_lookup[i].clk;
		c->lookup = clk_lookup[i];
		clk_init(c);
		INIT_LIST_HEAD(&c->shared_bus_list);
		if (!c->lookup.dev_id && !c->lookup.con_id)
			c->lookup.con_id = c->name;
		if (!c->name) {
			if (c->lookup.con_id)
				c->name = c->lookup.con_id;
			else if (c->lookup.dev_id)
				c->name = c->lookup.dev_id;
			else
				printk(KERN_ERR "clock has no name!\n");
		}
		clkdev_add(&clk_lookup[i]);
	}
}

/* setup cken base on the clear always and set always setting.
 * refer to related DM for more details.
 * And Base on the JIRA (MG1-1167), CKENB[10:9] is clear always.
 */
static void cken_clear_always_set_always_setup(void)
{
	unsigned int ckena_clear_always_bits_mask;
	unsigned int ckenb_clear_always_bits_mask;
	unsigned int ckenc_clear_always_bits_mask;
	unsigned int ckena_set_always_bits_mask;
	unsigned int ckenb_set_always_bits_mask;
	unsigned int ckenc_set_always_bits_mask;

	/* setup clear always bits */
	ckena_clear_always_bits_mask = 0x831730ee;
	if (cpu_is_pxa978())
		ckenb_clear_always_bits_mask = 0x00030e24;
	else
		ckenb_clear_always_bits_mask = 0x00030e20;
	ckenc_clear_always_bits_mask = 0x00303300;

	CKENA &= ~ckena_clear_always_bits_mask;
	CKENB &= ~ckenb_clear_always_bits_mask;
	CKENC &= ~ckenc_clear_always_bits_mask;

	/* setup set always bits */
	/* CKENA[19] is CKEN_TPM, not set it like spec define */
	ckena_set_always_bits_mask = 0x00000001;
	ckenb_set_always_bits_mask = 0xd7fcf040;
	if (cpu_is_pxa978())
		ckenc_set_always_bits_mask = 0x00038000;
	else
		ckenc_set_always_bits_mask = 0x00000000;

	CKENA |= ckena_set_always_bits_mask;
	CKENB |= ckenb_set_always_bits_mask;
	CKENC |= ckenc_set_always_bits_mask;

	/* Recheck the configuration. */
	if ((ckena_clear_always_bits_mask & ckena_set_always_bits_mask) ||
	    (ckenb_clear_always_bits_mask &
	     ckenb_set_always_bits_mask) ||
	    (ckenc_clear_always_bits_mask & ckenc_set_always_bits_mask)) {
		printk(KERN_ERR "The configuration of clear always and set"
		       "always is not right.\n");
		BUG();
	}
}

int pxa95x_clk_init(void)
{
	cken_clear_always_set_always_setup();

	CKENA &= ~((1 << CKEN_BOOT) | (1 << CKEN_CIR)
		   | (1 << CKEN_SSP1) | (1 << CKEN_SSP2)
		   | (1 << CKEN_SSP3) | (1 << CKEN_SSP4)
		   | (1 << CKEN_MSL0) | (1 << CKEN_BTUART)
#ifndef CONFIG_DEBUG_LL
		   /* Do not kill UART, otherwise nothing appears on the
		    * terminal until console becomes functional. */
		   | (1 << CKEN_FFUART)
#endif
		   | (1 << CKEN_STUART)
		   | (1 << CKEN_KEYPAD) | (1 << CKEN_TPM));

	if (cpu_is_pxa978())
		CKENA &= ~(1 << (CKEN_SMC) | 1 << (CKEN_NAND));

	CKENB &= ~((1 << (CKEN_I2C - 32))
		   | (1 << (CKEN_HSI - 32))
		   | (1 << (CKEN_VMETA - 32))
		   | (1 << (CKEN_1WIRE - 32))
		   | (1 << (CKEN_ABU - 32))
		   | (1 << (CKEN_PWM0 - 32))
		   | (1 << (CKEN_PWM1 - 32)));
	CKENC &= ~((1 << (CKEN_PXA95x_MMC1 - 64))
		   | (1 << (CKEN_PXA95x_MMC2 - 64))
		   | (1 << (CKEN_PXA95x_MMC3 - 64))
		   | (1 << (CKEN_PXA95x_MMC4 - 64))
		   | (1 << (CKEN_USB_PRL - 64))
		   | (1 << (CKEN_USBH_PRL - 64))
		   | (1 << (CKEN_USB_BUS - 64))
		   | (1 << (CKEN_USBH_BUS - 64))
		   | (1 << (CKEN_MMC1_BUS - 64))
		   | (1 << (CKEN_MMC2_BUS - 64))
		   | (1 << (CKEN_MMC3_BUS - 64))
		   | (1 << (CKEN_MMC4_BUS - 64))
		   | (1 << (CKEN_IMU - 64))
		   | (1 << (CKEN_I2C2 - 64))
		   | (1 << (CKEN_I2C3 - 64))
		   | (1 << (CKEN_SCI1 - 64))
		   | (1 << (CKEN_SCI2 - 64))
		   | (1 << (CKEN_CSI_TX - 64))
		   | (1 << (CKEN_GC_1X - 64))
		   | (1 << (CKEN_GC_2X - 64))
		   | (1 << (CKEN_DSI_TX1 - 64))
		   | (1 << (CKEN_DSI_TX2 - 64))
		   | (1 << (CKEN_DISPLAY - 64))
		   | (1 << (CKEN_PIXEL - 64))
		   | (1 << (CKEN_AXI - 64)));

	if (!is_wkr_dma_clock())
		CKENC &= ~(1 << (CKEN_AXI_2X - 64));

	clock_lookup_init(common_clkregs, ARRAY_SIZE(common_clkregs));
	if (cpu_is_pxa978()) {
		clock_lookup_init(pxa978_specific_clkregs,
				ARRAY_SIZE(pxa978_specific_clkregs));
		/* Make sure rate is always same as real setting */
		clk_pxa978_gcu.rate = clk_get_rate(&clk_pxa978_gcu);
		clk_pxa95x_vmeta.rate = clk_get_rate(&clk_pxa95x_vmeta);

		/* Initialize the clock of vMeta/GC to lowest */
		clk_set_rate(&clk_pxa95x_vmeta, 0);
		clk_set_rate(&clk_pxa978_gcu, 0);

		/* Don't use IRQ disable to protect the clock driver */
		clk_set_cansleep(&clk_pxa95x_vmeta);
		clk_set_cansleep(&clk_pxa978_gcu);
	} else
		clock_lookup_init(pxa955_specific_clkregs,
				  ARRAY_SIZE(pxa955_specific_clkregs));
	return 0;
}
