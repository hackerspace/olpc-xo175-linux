/*
 * linux/arch/arm/mach-pxa/pxa95x.c
 *
 * code specific to PXA95x aka MGx
 *
 * Copyright (C) 2009-2010 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/i2c/pxa-i2c.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/syscore_ops.h>
#include <linux/memblock.h>
#include <linux/sysdev.h>
#include <linux/delay.h>

#include <asm/hardware/cache-tauros2.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/pxa3xx-regs.h>
#include <mach/pxa930.h>
#include <mach/reset.h>
#include <mach/pm.h>
#include <mach/dma.h>
#include <mach/regs-intc.h>

#include "generic.h"
#include "devices.h"
#include "clock.h"
#include "dsi_hdmi_pll.h"

static struct mfp_addr_map pxa95x_mfp_addr_map[] __initdata = {

	MFP_ADDR(GPIO0, 0x02e0),
	MFP_ADDR(GPIO1, 0x02dc),
	MFP_ADDR(GPIO2, 0x02e8),
	MFP_ADDR(GPIO3, 0x02d8),
	MFP_ADDR(GPIO4, 0x02e4),
	MFP_ADDR(GPIO5, 0x02ec),
	MFP_ADDR(GPIO6, 0x02f8),
	MFP_ADDR(GPIO7, 0x02fc),
	MFP_ADDR(GPIO8, 0x0300),
	MFP_ADDR(GPIO9, 0x02d4),
	MFP_ADDR(GPIO10, 0x02f4),
	MFP_ADDR(GPIO11, 0x02f0),
	MFP_ADDR(GPIO12, 0x0304),
	MFP_ADDR(GPIO13, 0x0310),
	MFP_ADDR(GPIO14, 0x0308),
	MFP_ADDR(GPIO15, 0x030c),
	MFP_ADDR(GPIO16, 0x04e8),
	MFP_ADDR(GPIO17, 0x04f4),
	MFP_ADDR(GPIO18, 0x04f8),
	MFP_ADDR(GPIO19, 0x04fc),
	MFP_ADDR(GPIO20, 0x0518),
	MFP_ADDR(GPIO21, 0x051c),
	MFP_ADDR(GPIO22, 0x04ec),
	MFP_ADDR(GPIO23, 0x0500),
	MFP_ADDR(GPIO24, 0x04f0),
	MFP_ADDR(GPIO25, 0x0504),
	MFP_ADDR(GPIO26, 0x0510),
	MFP_ADDR(GPIO27, 0x0514),
	MFP_ADDR(GPIO28, 0x0520),
	MFP_ADDR(GPIO29, 0x0600),
	MFP_ADDR(GPIO30, 0x0618),
	MFP_ADDR(GPIO31, 0x0610),
	MFP_ADDR(GPIO32, 0x060c),
	MFP_ADDR(GPIO33, 0x061c),
	MFP_ADDR(GPIO34, 0x0620),
	MFP_ADDR(GPIO35, 0x0628),
	MFP_ADDR(GPIO36, 0x062c),
	MFP_ADDR(GPIO37, 0x0630),
	MFP_ADDR(GPIO38, 0x0634),
	MFP_ADDR(GPIO39, 0x0638),
	MFP_ADDR(GPIO40, 0x063c),
	MFP_ADDR(GPIO41, 0x0614),
	MFP_ADDR(GPIO42, 0x0624),
	MFP_ADDR(GPIO43, 0x0608),
	MFP_ADDR(GPIO44, 0x0604),
	MFP_ADDR(GPIO45, 0x050c),
	MFP_ADDR(GPIO46, 0x0508),
	MFP_ADDR(GPIO47, 0x02bc),
	MFP_ADDR(GPIO48, 0x02b4),
	MFP_ADDR(GPIO49, 0x02b8),
	MFP_ADDR(GPIO50, 0x02c8),
	MFP_ADDR(GPIO51, 0x02c0),
	MFP_ADDR(GPIO52, 0x02c4),
	MFP_ADDR(GPIO53, 0x02d0),
	MFP_ADDR(GPIO54, 0x02cc),
	MFP_ADDR(GPIO55, 0x029c),
	MFP_ADDR(GPIO56, 0x02a0),
	MFP_ADDR(GPIO57, 0x0294),
	MFP_ADDR(GPIO58, 0x0298),
	MFP_ADDR(GPIO59, 0x02a4),
	MFP_ADDR(GPIO60, 0x02a8),
	MFP_ADDR(GPIO61, 0x02b0),
	MFP_ADDR(GPIO62, 0x02ac),
	MFP_ADDR(GPIO63, 0x0640),
	MFP_ADDR(GPIO64, 0x065c),
	MFP_ADDR(GPIO65, 0x0648),
	MFP_ADDR(GPIO66, 0x0644),
	MFP_ADDR(GPIO67, 0x0674),
	MFP_ADDR(GPIO68, 0x0658),
	MFP_ADDR(GPIO69, 0x0654),
	MFP_ADDR(GPIO70, 0x0660),
	MFP_ADDR(GPIO71, 0x0668),
	MFP_ADDR(GPIO72, 0x0664),
	MFP_ADDR(GPIO73, 0x0650),
	MFP_ADDR(GPIO74, 0x066c),
	MFP_ADDR(GPIO75, 0x064c),
	MFP_ADDR(GPIO76, 0x0670),
	MFP_ADDR(GPIO77, 0x0678),
	MFP_ADDR(GPIO78, 0x067c),
	MFP_ADDR(GPIO79, 0x0694),
	MFP_ADDR(GPIO80, 0x069c),
	MFP_ADDR(GPIO81, 0x06a0),
	MFP_ADDR(GPIO82, 0x06a4),
	MFP_ADDR(GPIO83, 0x0698),
	MFP_ADDR(GPIO84, 0x06bc),
	MFP_ADDR(GPIO85, 0x06b4),
	MFP_ADDR(GPIO86, 0x06b0),
	MFP_ADDR(GPIO87, 0x06c0),
	MFP_ADDR(GPIO88, 0x06c4),
	MFP_ADDR(GPIO89, 0x06ac),
	MFP_ADDR(GPIO90, 0x0680),
	MFP_ADDR(GPIO91, 0x0684),
	MFP_ADDR(GPIO92, 0x0688),
	MFP_ADDR(GPIO93, 0x0690),
	MFP_ADDR(GPIO94, 0x068c),
	MFP_ADDR(GPIO95, 0x06a8),
	MFP_ADDR(GPIO96, 0x06b8),
	MFP_ADDR(GPIO97, 0x0410),
	MFP_ADDR(GPIO98, 0x0418),
	MFP_ADDR(GPIO99, 0x041c),
	MFP_ADDR(GPIO100, 0x0414),
	MFP_ADDR(GPIO101, 0x0408),
	MFP_ADDR(GPIO102, 0x0324),
	MFP_ADDR(GPIO103, 0x040c),
	MFP_ADDR(GPIO104, 0x0400),
	MFP_ADDR(GPIO105, 0x0328),
	MFP_ADDR(GPIO106, 0x0404),

	MFP_ADDR(GPIO159, 0x0524),
	MFP_ADDR(GPIO163, 0x0534),
	MFP_ADDR(GPIO167, 0x0544),
	MFP_ADDR(GPIO168, 0x0548),
	MFP_ADDR(GPIO169, 0x054c),
	MFP_ADDR(GPIO170, 0x0550),
	MFP_ADDR(GPIO171, 0x0554),
	MFP_ADDR(GPIO172, 0x0558),
	MFP_ADDR(GPIO173, 0x055c),

	MFP_ADDR(nXCVREN, 0x0204),
	MFP_ADDR(DF_CLE_nOE, 0x020c),
	MFP_ADDR(DF_nADV1_ALE, 0x0218),
	MFP_ADDR(DF_SCLK_E, 0x0214),
	MFP_ADDR(DF_SCLK_S, 0x0210),
	MFP_ADDR(nBE0, 0x021c),
	MFP_ADDR(nBE1, 0x0220),
	MFP_ADDR(DF_nADV2_ALE, 0x0224),
	MFP_ADDR(DF_INT_RnB, 0x0228),
	MFP_ADDR(DF_nCS0, 0x022c),
	MFP_ADDR(DF_nCS1, 0x0230),
	MFP_ADDR(nLUA, 0x0254),
	MFP_ADDR(nLLA, 0x0258),
	MFP_ADDR(DF_nWE, 0x0234),
	MFP_ADDR(DF_nRE_nOE, 0x0238),
	MFP_ADDR(DF_ADDR0, 0x024c),
	MFP_ADDR(DF_ADDR1, 0x0250),
	MFP_ADDR(DF_ADDR2, 0x025c),
	MFP_ADDR(DF_ADDR3, 0x0260),
	MFP_ADDR(DF_IO0, 0x023c),
	MFP_ADDR(DF_IO1, 0x0240),
	MFP_ADDR(DF_IO2, 0x0244),
	MFP_ADDR(DF_IO3, 0x0248),
	MFP_ADDR(DF_IO4, 0x0264),
	MFP_ADDR(DF_IO5, 0x0268),
	MFP_ADDR(DF_IO6, 0x026c),
	MFP_ADDR(DF_IO7, 0x0270),
	MFP_ADDR(DF_IO8, 0x0274),
	MFP_ADDR(DF_IO9, 0x0278),
	MFP_ADDR(DF_IO10, 0x027c),
	MFP_ADDR(DF_IO11, 0x0280),
	MFP_ADDR(DF_IO12, 0x0284),
	MFP_ADDR(DF_IO13, 0x0288),
	MFP_ADDR(DF_IO14, 0x028c),
	MFP_ADDR(DF_IO15, 0x0290),

	MFP_ADDR(GSIM_UIO, 0x0314),
	MFP_ADDR(GSIM_UCLK, 0x0318),
	MFP_ADDR(GSIM_UDET, 0x031c),
	MFP_ADDR(GSIM_nURST, 0x0320),

	MFP_ADDR(PMIC_INT, 0x06c8),

	MFP_ADDR(RDY, 0x0200),

	MFP_ADDR_END,
};

static struct mfp_addr_map pxa970_mfp_addr_map[] __initdata = {
	MFP_ADDR_X(GPIO0, GPIO191, 0x208),

	MFP_ADDR_END,
};

static void clk_tout_s0_enable(struct clk *clk)
{
	OSCC |= OSCC_TENS0;
}

static void clk_tout_s0_disable(struct clk *clk)
{
	OSCC &= ~OSCC_TENS0;
}

const struct clkops clk_pxa95x_tout_s0_ops = {
	.enable		= clk_tout_s0_enable,
	.disable	= clk_tout_s0_disable,
};
#define MIPI_BANDGAP_CONTROL	(1 << 10)
static volatile u32 *gen_reg4;
#define GEN_REG4 0x42404078
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


u32 get_mipi_reference_control(void)
{
	if (!gen_reg4)
		gen_reg4 = ioremap_nocache(GEN_REG4, 0x4);
	return *gen_reg4;
}


static void clk_pxa95x_dsi_enable(struct clk *dsi_clk)
{
	set_mipi_reference_control();
	dsi_enable_status = 1;

	if (!cpu_is_pxa970()) {
		struct DSIRegisters  *p_Regs =
			get_dsi_pll(dsi_clk->cken == CKEN_DSI_TX2);
		/*Enable PLL with internal timer*/
		p_Regs->DSI_REG3 = 0x05cc60a2;
		p_Regs->DSI_REG1 =
			DSI_REG1_DSIx_PLL_LOCK_SEL
			| DSI_REG1_DSIx_PLL_ON;
	}

	pr_info("dsi clock enable\n");
	CKENC |= 1 << (dsi_clk->cken - 64);

	return;
}

static void clk_pxa95x_dsi_disable(struct clk *dsi_clk)
{
	CKENC &= ~(1 << (dsi_clk->cken - 64));

	if (!cpu_is_pxa970()) {
		struct DSIRegisters  *p_Regs =
			get_dsi_pll(dsi_clk->cken == CKEN_DSI_TX2);
		/*Disable PLL with internal timer*/
		/* JIRA: MG1-1265 dis pll could only be disabled
		   by setting reg3 to this value */
		p_Regs->DSI_REG3 = 0x040c60a2;
		p_Regs->DSI_REG1 =
			DSI_REG1_DSIx_PLL_LOCK_SEL;
	}

	dsi_enable_status = 0;
	clear_mipi_reference_control();

	return;
}

/* refdiv, fbdiv, vcodiv_sel_se, vco_vrng, kvco_range */
static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_202 = {3, 140, 7, 0, 1};
static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_217 = {3, 150, 7, 0, 1};
static struct DSI_PLL_PARAMETERS_NEVO LCD_DSI_PLL_Freq_594 = {3, 171, 3, 1, 2};
static void dsi_set_clock_nevo(u32 dsi_clk_val)
{
	struct DSIRegisters_Nevo  *p_Regs = get_dsi_pll_nevo();
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
	/*Make sure dsi_clk is within range*/
	if (dsi_clk_val > 624) {
		pr_err("%s: BAD clock value used\n", __func__);
		return;
	}

	/*Disable PLL with internal timer*/
	p_Regs->DSI_NEVO_PU = 0;

	/*Set default values*/
	refdiv = 0x1;/*Always Set to 00001*/
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
		refdiv = 3; /*default settings for all frequencies */

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

		/*Calculate FBDIV according to dsi_clk_val*/
		fbdiv = dsi_calc_fbdiv_Nevo(refdiv, kvco);
	}

	/*Assemble REG2 value*/
	p_Regs->DSI_NEVO_CONFIG =
		DSI_NEVO_CONFIG_VCODIV_SEL_SE(vcodiv_sel_se)
		| DSI_NEVO_CONFIG_VCO_VRNG(vco_vrng)
		| DSI_NEVO_CONFIG_KVCO(kvco_range)
		| DSI_NEVO_CONFIG_OUTDIV(outdiv)
		| DSI_NEVO_CONFIG_FBDIV(fbdiv)
		| DSI_NEVO_CONFIG_REFDIV(refdiv);

	/*Enable PLL with internal timer*/
	p_Regs->DSI_NEVO_PU = DSI_NEVO_PU_PLL_ON;

	/*Need to at least wait 100us*/
	mdelay(1);

	dsi_pll_locked_Nevo(10, p_Regs);
	return;
}

static void dsi_set_clock_pv2(u32 dsi_clk_val, u32 converter)
{
	struct DSIRegisters  *p_Regs = get_dsi_pll(converter);
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
	/*Make sure dsi_clk is within range*/
	if (dsi_clk_val > 624) {
		pr_err("%s: BAD clock value used\n", __func__);
		return;
	}

	/*Disable PLL with internal timer*/
	p_Regs->DSI_REG1 =
		DSI_REG1_DSIx_PLL_LOCK_SEL;

	/*Set default values*/
	refdiv = 0x1;/*Always Set to 00001*/
	if (dsi_clk_val > 200) {
		outdiv = 0xF;
		if (dsi_clk_val < 400)
			cksel = 0x1;
		else
			cksel = 0x0;
	} else {
		outdiv = 16 - (200/dsi_clk_val);
		cksel = 0x0;
	}

	/*Calculate FBDIV according to dsi_clk_val*/
	fbdiv = dsi_calc_fbdiv(dsi_clk_val, outdiv, cksel);

	/*Assemble REG2 value*/
	p_Regs->DSI_REG2 = DSI_REG2_DEFAULT
		| DSI_REG2_CK_SEL(cksel)
		| DSI_REG2_OUTDIV(outdiv)
		| DSI_REG2_FBDIV(fbdiv)
		| DSI_REG2_REFDIV(refdiv);

	/*Enable PLL with internal timer*/
	p_Regs->DSI_REG1 =
		DSI_REG1_DSIx_PLL_LOCK_SEL
		| DSI_REG1_DSIx_PLL_ON;

	/*Need to at least wait 100us*/
	mdelay(1);
	return;
}


static int clk_pxa95x_dsi_setrate(struct clk *dsi_clk, unsigned long rate)
{
	int mrate = rate/1000000;
	if (cpu_is_pxa970())
		dsi_set_clock_nevo(mrate);
	else
		dsi_set_clock_pv2(mrate, (dsi_clk->cken == CKEN_DSI_TX2));
	return 0;

}

/* TODO: hdmi cken? */
static void clk_pxa95x_ihdmi_enable(struct clk *hdmi_clk)
{
	set_mipi_reference_control();
	dsi_enable_status = 1;
}

static void clk_pxa95x_ihdmi_disable(struct clk *hdmi_clk)
{
	struct HDMIRegisters *p_Regs = get_ihdmi_pll();

	p_Regs->HDMI_PLL_PU = 0;
	mdelay(1);
	hdmi_enable_status = 0;
	clear_mipi_reference_control();
}

/* place real turn on when setrate: enable -> setrate */
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_126 = {
	0x2, 0x4, 0x3, 77, 0x2, 0x0188D};
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_136 = {
	0x2, 0x4, 0x3, 83, 0x2, 0x003CC};
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_271 = {
	0x2, 0x4, 0x4, 83, 0x2, 0x003CC};
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_371 = {
	0x2, 0x4, 0x2, 114, 0x2, 0x084B};
static struct HDMI_PLL_PARAMETERS hdmi_pll_freq_742 = {
	0x2, 0x4, 0x1, 114, 0x2, 0x084B};
static int clk_pxa95x_ihdmi_setrate(struct clk *clk, unsigned long rate)
{
	struct HDMIRegisters *p_Regs = get_ihdmi_pll();
	u32 hdmi_clk = rate/1000000;

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
	p_Regs->HDMI_PHY_CTL1 &=
		~(HDMI_PHY_CTL1_REG_PD_IREF(0)
		| HDMI_PHY_CTL1_REG_PD_TX(0xf));
	p_Regs->HDMI_PHY_CTL1 |= HDMI_PHY_CTL1_REG_RESET_TX_OFF;
	mdelay(1);
	p_Regs->HDMI_PHY_CTL1 &= HDMI_PHY_CTL1_REG_RESET_TX_ON;
	/* Reset the vpll HW state machine */
	p_Regs->HDMI_PLL_DEBUG2 &= ~HDMI_PLL_DEBUG2_PLL_CTRL_RSTN;
	mdelay(1);
	/* Disable PLL */
	p_Regs->HDMI_PLL_PU = 0;
	mdelay(1);
	/* Release reset the vpll HW state machine */
	p_Regs->HDMI_PLL_DEBUG2 |= HDMI_PLL_DEBUG2_PLL_CTRL_RSTN;
	mdelay(1);
	/* Enable PLL */
	/* p_Regs->HDMI_PLL_PU = HDMI_PLL_PWR_PLL_ON; */

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
	p_Regs->HDMI_PLL_CONFIG1 =
			HDMI_PLL_CONFIG1_REFDIV(para->refdiv)
			|HDMI_PLL_CONFIG1_FBDIV(para->fbdiv)
			|HDMI_PLL_CONFIG1_POSTDIV_SEL(para->postdiv_sel)
			|HDMI_PLL_CONFIG1_KVCO(para->kvco)
			|HDMI_PLL_CONFIG1_INTPI(para->intpi)
			|HDMI_PLL_CONFIG1_EN_HDMI
			|HDMI_PLL_CONFIG1_EN_PANEL;

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

static void clk_pxa95x_lcd_enable(struct clk *lcd_clk)
{
	CKENC |= (1 << (CKEN_DISPLAY - 64)) | (1 << (CKEN_PIXEL - 64));
}

static void clk_pxa95x_lcd_disable(struct clk *dsi_clk)
{
	CKENC &= ~((1 << (CKEN_DISPLAY - 64))
		| (1 << (CKEN_PIXEL - 64)));
}

static void clk_axi_enable(struct clk *clk)
{
	CKENC |= (1 << (CKEN_AXI - 64)) | (1 << (CKEN_AXI_2X -64));
}

static void clk_axi_disable(struct clk *clk)
{
	/* we are checking that all clients of the AXI island have turned off
	*  their AXI clock enable. onlt if all clients are off we can turn off
	* the AXI clock.
	*/
	if (!(CKENC & ((1 << (CKEN_MMC4_BUS - 64))
		| (1 << (CKEN_MMC3_BUS - 64))
		| (1 << (CKEN_MMC2_BUS - 64))
		| (1 << (CKEN_MMC1_BUS - 64))
		| (1 << (CKEN_USB_BUS - 64))
		| (1 << (CKEN_USBH_BUS - 64))
		| (1 << (CKEN_GC_1X - 64))
		| (1 << (CKEN_GC_1X - 64))
		| (1 << (CKEN_DSI_TX1 - 64))
		| (1 << (CKEN_DSI_TX2 - 64))
		| (1 << (CKEN_DISPLAY - 64))
		| (1 << (CKEN_SCI1 - 64))
		| (1 << (CKEN_SCI2 - 64))
		| (1 << (CKEN_CSI_TX - 64))
		| (1 << (CKEN_PXA95x_MMC1 - 64))
		| (1 << (CKEN_PXA95x_MMC2 - 64))
		| (1 << (CKEN_PXA95x_MMC3 - 64))
		| (1 << (CKEN_PXA95x_MMC4 - 64))
		))
		&& !(CKENB & ((1 << (CKEN_HSI - 32))
			| (1 << (CKEN_VMETA - 32)))))
		/* turning off AXI clock. */
		CKENC &= ~(1 << (CKEN_AXI - 64) | (1 << (CKEN_AXI_2X -64)));
}

static const struct clkops clk_pxa95x_dsi_ops = {
	.enable		= clk_pxa95x_dsi_enable,
	.disable	= clk_pxa95x_dsi_disable,
	.setrate	= clk_pxa95x_dsi_setrate,
};

static const struct clkops clk_pxa95x_ihdmi_ops = {
	.enable		= clk_pxa95x_ihdmi_enable,
	.disable	= clk_pxa95x_ihdmi_disable,
	.setrate	= clk_pxa95x_ihdmi_setrate,
};

static const struct clkops clk_pxa95x_lcd_ops = {
	.enable		= clk_pxa95x_lcd_enable,
	.disable	= clk_pxa95x_lcd_disable,
};

static const struct clkops clk_axi_ops = {
	.enable		= clk_axi_enable,
	.disable	= clk_axi_disable,
};

static DEFINE_CK(pxa95x_dsi0, DSI_TX1, &clk_pxa95x_dsi_ops);
static DEFINE_CK(pxa95x_dsi1, DSI_TX2, &clk_pxa95x_dsi_ops);
static DEFINE_CK(pxa95x_ihdmi, DISPLAY, &clk_pxa95x_ihdmi_ops);
static DEFINE_CK(pxa95x_lcd, DISPLAY, &clk_pxa95x_lcd_ops);
static DEFINE_CK(pxa95x_axi, AXI, &clk_axi_ops);
static DEFINE_CLK(pxa95x_pout, &clk_pxa3xx_pout_ops, 13000000, 70);
static DEFINE_CLK(pxa95x_tout_s0, &clk_pxa95x_tout_s0_ops, 13000000, 70);
static DEFINE_PXA3_CKEN(pxa95x_ffuart, FFUART, 14857000, 1);
static DEFINE_PXA3_CKEN(pxa95x_btuart, BTUART, 14857000, 1);
static DEFINE_PXA3_CKEN(pxa95x_stuart, STUART, 14857000, 1);
static DEFINE_PXA3_CKEN(pxa95x_i2c1, I2C, 32842000, 0);
static DEFINE_PXA3_CKEN(pxa95x_i2c2, I2C2, 32842000, 0);
static DEFINE_PXA3_CKEN(pxa95x_i2c3, I2C3, 32842000, 0);
static DEFINE_PXA3_CKEN(pxa95x_keypad, KEYPAD, 32768, 0);
static DEFINE_PXA3_CKEN(pxa95x_ssp1, SSP1, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_ssp2, SSP2, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_ssp3, SSP3, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_ssp4, SSP4, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_pwm0, PWM0, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_pwm1, PWM1, 13000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_sdh0, PXA95x_MMC1, 200000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_sdh1, PXA95x_MMC2, 200000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_sdh2, PXA95x_MMC3, 200000000, 0);
static DEFINE_PXA3_CKEN(pxa95x_sdh3, PXA95x_MMC4, 200000000, 0);

static struct clk_lookup pxa95x_clkregs[] = {
	INIT_CLKREG(&clk_pxa95x_pout, NULL, "CLK_POUT"),
	INIT_CLKREG(&clk_pxa95x_tout_s0, NULL, "CLK_TOUT_S0"),
	/* Power I2C clock is always on */
	INIT_CLKREG(&clk_dummy, "pxa3xx-pwri2c.1", NULL),
	INIT_CLKREG(&clk_pxa95x_ffuart, "pxa2xx-uart.0", NULL),
	INIT_CLKREG(&clk_pxa95x_btuart, "pxa2xx-uart.1", NULL),
	INIT_CLKREG(&clk_pxa95x_stuart, "pxa2xx-uart.2", NULL),
	INIT_CLKREG(&clk_pxa95x_stuart, "pxa2xx-ir", "UARTCLK"),
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
	INIT_CLKREG(&clk_pxa95x_dsi0, NULL, "PXA95x_DSI0CLK"),
	INIT_CLKREG(&clk_pxa95x_dsi1, NULL, "PXA95x_DSI1CLK"),
	INIT_CLKREG(&clk_pxa95x_ihdmi, NULL, "PXA95x_iHDMICLK"),
	INIT_CLKREG(&clk_pxa95x_lcd, "pxa95x-fb", "PXA95x_LCDCLK"),
	INIT_CLKREG(&clk_pxa95x_axi, NULL, "AXICLK"),
	INIT_CLKREG(&clk_pxa95x_sdh0, "sdhci-pxav2.0", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa95x_sdh1, "sdhci-pxav2.1", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa95x_sdh2, "sdhci-pxav2.2", "PXA-SDHCLK"),
	INIT_CLKREG(&clk_pxa95x_sdh3, "sdhci-pxav2.3", "PXA-SDHCLK"),
};

void __init pxa95x_init_irq(void)
{
	pxa_init_irq(96, NULL);
	pxa_init_gpio(IRQ_GPIO_2_x, 2, 191, NULL);
}

/*
 * device registration specific to PXA93x.
 */

void __init pxa95x_set_i2c_power_info(struct i2c_pxa_platform_data *info)
{
	pxa_register_device(&pxa3xx_device_i2c_power, info);
}

static struct platform_device *devices[] __initdata = {
	&sa1100_device_rtc,
	&pxa_device_rtc,
	&pxa27x_device_ssp1,
	&pxa27x_device_ssp2,
	&pxa27x_device_ssp3,
	&pxa3xx_device_ssp4,
	&pxa27x_device_pwm0,
	&pxa27x_device_pwm1,
};

static int __init pxa95x_init(void)
{
	int ret = 0, i;

	if (cpu_is_pxa95x()) {
#ifdef CONFIG_CACHE_TAUROS2
		tauros2_init();
#endif
		mfp_init_base(io_p2v(MFPR_BASE));
		if (cpu_is_pxa970())
			mfp_init_addr(pxa970_mfp_addr_map);
		else
			mfp_init_addr(pxa95x_mfp_addr_map);

		reset_status = ARSR;

		/*
		 * clear RDH bit every time after reset
		 *
		 * Note: the last 3 bits DxS are write-1-to-clear so carefully
		 * preserve them here in case they will be referenced later
		 */
		ASCR &= ~(ASCR_RDH | ASCR_D1S | ASCR_D2S | ASCR_D3S);

		clkdev_add_table(pxa95x_clkregs, ARRAY_SIZE(pxa95x_clkregs));

		if ((ret = pxa_init_dma(IRQ_DMA, 32)))
			return ret;

		register_syscore_ops(&pxa_irq_syscore_ops);
		register_syscore_ops(&pxa_gpio_syscore_ops);
		register_syscore_ops(&pxa3xx_clock_syscore_ops);

		ret = platform_add_devices(devices, ARRAY_SIZE(devices));
	}

	return ret;
}

postcore_initcall(pxa95x_init);

#define CP_MEM_MAX_SEGMENTS 2
unsigned _cp_area_addr[CP_MEM_MAX_SEGMENTS];
unsigned _cp_area_size[CP_MEM_MAX_SEGMENTS+1]; /* last entry 0 */
static int __init setup_cpmem(char *p)
{
	unsigned long size, start = 0xa7000000;
	int seg;

	size  = memparse(p, &p);
	if (*p == '@')
		start = memparse(p + 1, &p);

	for (seg = 0; seg < CP_MEM_MAX_SEGMENTS; seg++)
		if (!_cp_area_size[seg])
			break;
	BUG_ON(seg == CP_MEM_MAX_SEGMENTS);
	_cp_area_addr[seg] = (unsigned)start;
	_cp_area_size[seg] = (unsigned)size;
	return 0;
}
early_param("cpmem", setup_cpmem);

unsigned cp_area_addr(void)
{
	/* _cp_area_addr[] contain actual CP region addresses for reservation.
	This function returns the address of the first region, which is
	the main one used for AP-CP interface, aligned to 16MB.
	The AP-CP interface code takes care of the offsets inside the region,
	including the non-CP area at the beginning of the 16MB aligned range. */
	return _cp_area_addr[0]&0xFF000000;
}

void pxa95x_cpmem_reserve(void)
{
	int seg;

	for (seg = 0; seg < CP_MEM_MAX_SEGMENTS; seg++) {
		if (_cp_area_size[seg] != 0) {
			BUG_ON(memblock_reserve(_cp_area_addr[seg], _cp_area_size[seg]));
			printk(KERN_INFO "Reserving CP memory: %dM at %.8x\n",
				(unsigned)_cp_area_size[seg]/0x100000,
				(unsigned)_cp_area_addr[seg]);
		}
	}
}

void pxa95x_mem_reserve(void)
{
	pxa95x_cpmem_reserve();
}
