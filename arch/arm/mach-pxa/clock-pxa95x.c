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
#include <asm/mach/map.h>
#include <plat/clock.h>
#include <plat/devfreq.h>
#include <plat/reg_rw.h>
#include <mach/pxa3xx-regs.h>
#include <mach/debug_pm.h>
#include <mach/pxa95x_dvfm.h>
#include <mach/dvfs.h>
#include "dsi_hdmi_pll.h"
#include "clock.h"
#include <mach/pxa9xx_pm_logger.h>	/* for pm debug tracing */
#include <mach/regs-ost.h>

#define KHZ_TO_HZ	1000
#define MHZ_TO_HZ	1000000

static u32 sram_map, clkmgr_base, dmc_base;

struct reg_table_profile reg_table_profiles[DDR_FROFILE_MAX_NUM];
static unsigned int used_pro_count;
static unsigned int entry_count[DDR_FROFILE_MAX_NUM];
static unsigned long ddr_prof_data_base_init;
static u16 ddr_reg_inited;

static void ddr_reg_table_init(unsigned int ddr_prof_data_base)
{
	void __iomem *ddr_prof_data_base_Addr;

	pr_debug("%s, DDR_Reg Base: 0x%X\n", __func__, ddr_prof_data_base);

	if (ddr_prof_data_base) {
		u32 base = 0;
		ddr_prof_data_base_Addr =
		    ioremap(ddr_prof_data_base, sizeof(reg_table_profiles));

		memset(entry_count, 0, sizeof(entry_count));
		memset(reg_table_profiles, 0, sizeof(reg_table_profiles));

		for (used_pro_count = 0; used_pro_count < DDR_FROFILE_MAX_NUM;
		     used_pro_count++) {
			reg_table_profiles[used_pro_count].freq =
			    readl(ddr_prof_data_base_Addr + base);
			base += sizeof(reg_table_profiles[used_pro_count].freq);

			if (!reg_table_profiles[used_pro_count].freq)
				break;

			/* Workaround: OBM freq is not aligned to kernel's */
			if (reg_table_profiles[used_pro_count].freq == 475)
				reg_table_profiles[used_pro_count].freq = 472;
			if (reg_table_profiles[used_pro_count].freq == 400)
				reg_table_profiles[used_pro_count].freq = 398;
			/* Workaround end */

			pr_debug("Freq: %d MHz\n",
				 reg_table_profiles[used_pro_count].freq);
			pr_debug("Profile: %d\n", used_pro_count);

			while (1) {
				u32 data, offset;

				offset = readl(ddr_prof_data_base_Addr + base);
				base += (DDR_ENTRY_SIZE / 2);
				reg_table_profiles[used_pro_count].
				    offset[entry_count[used_pro_count]] =
				    offset;

				data = readl(ddr_prof_data_base_Addr + base);
				base += (DDR_ENTRY_SIZE / 2);
				reg_table_profiles[used_pro_count].
				    data[entry_count[used_pro_count]] = data;

				pr_debug("OFFSET: 0x%X,  Data: 0x%X\n", offset,
					 data);

				entry_count[used_pro_count]++;
				if ((!offset && !data) ||
				    (!(entry_count[used_pro_count] <
				       DDR_ENTRY_MAX_NUM)))
					break;
			}
			if (entry_count[used_pro_count] < DDR_ENTRY_MAX_NUM)
				base +=
				    ((DDR_ENTRY_MAX_NUM -
				      entry_count[used_pro_count])
				     * DDR_ENTRY_SIZE);
		}
		ddr_reg_inited = 1;
		pr_info("%d Profiles are initated\n", used_pro_count);
		iounmap(ddr_prof_data_base_Addr);
	} else
		pr_err("DDR base Address is not valid");
}

static int reg_base_init(char *buf)
{
	int res = 0;
	res = strict_strtoul(buf, 16, &ddr_prof_data_base_init);
	return res;
}

__setup("MCPD=", reg_base_init);

struct devfreq_frequency_table pxa978_ddr_clk_table[] = {
	INIT_FREQ_TABLE(1, 208000),
	INIT_FREQ_TABLE(2, 312000),
	INIT_FREQ_TABLE(3, 416000),
	INIT_FREQ_TABLE(3, 797000),
	INIT_FREQ_TABLE(4, 944000),
	INIT_FREQ_TABLE(5, 1066000),
	INIT_FREQ_TABLE(6, DEVFREQ_TABLE_END),
};

/* The last frequency requested by Gc or Vmeta */
static unsigned long last_gc, last_vmeta;
static DEFINE_SPINLOCK(mmpll_lock);
static DEFINE_MUTEX(gc_vmeta_mutex);

static struct devfreq_frequency_table pxa978_gcvmeta_clk_table[] = {
	INIT_FREQ_TABLE(1, 156000000),
	INIT_FREQ_TABLE(2, 208000000),
	INIT_FREQ_TABLE(3, 312000000),
	INIT_FREQ_TABLE(4, 416000000),
	INIT_FREQ_TABLE(5, 498000000),
	INIT_FREQ_TABLE(6, 600000000),
	INIT_FREQ_TABLE(7, DEVFREQ_TABLE_END),
};

static struct devfreq_frequency_table pxa978_dx_gcvmeta_clk_table[] = {
	INIT_FREQ_TABLE(1, 156000000),
	INIT_FREQ_TABLE(2, 208000000),
	INIT_FREQ_TABLE(3, 312000000),
	INIT_FREQ_TABLE(4, 416000000),
	INIT_FREQ_TABLE(5, 481000000),
	INIT_FREQ_TABLE(6, 600000000),
	INIT_FREQ_TABLE(7, DEVFREQ_TABLE_END),
};

unsigned long gc_cur_freqs_table[GC_VM_OP_NUM_MAX];
unsigned int gc_freq_counts;

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

/*
 * If use CKRQxP/CKREQxE to control CLK26MOUTDMD/CLK13MOUTDMD, the
 * MFP for hardware request signal must be configured as AF sysclk_req.
 * If control the clock via software(bit 16 of GEN_REG3), hardware
 * request signal is not necessary.
 */
/* GEN_REG3 system clock request line */
enum sys_clk_req_line {
	SYS_CLK_REQ_LINE_0_SHIFT = 14,
	/* not used currently */
	SYS_CLK_REQ_LINE_1_SHIFT = 12,
	/* not used currently */
	SYS_CLK_REQ_LINE_2_SHIFT = 10,
	/* not used currently */
	SYS_CLK_REQ_LINE_3_SHIFT = 8,
	/* not used currently */
};

/* request line active config, by default: low for request active */
enum sys_clk_req_line_active {
	SYS_CLK_REQ_LINE_ACTIVE_HIGH = 0,
	SYS_CLK_REQ_LINE_ACTIVE_LOW,
	SYS_CLK_REQ_LINE_ACTIVE_DEFAULT = SYS_CLK_REQ_LINE_ACTIVE_LOW,
};

/* request line enable or disable */
enum sys_clk_req_line_control {
	SYS_CLK_REQ_LINE_DISABLE = 0,
	SYS_CLK_REQ_LINE_ENABLE,
};

#define SYS_CLK_ON ((SYS_CLK_REQ_LINE_ACTIVE_DEFAULT << 1) | SYS_CLK_REQ_LINE_ENABLE)
#define SYS_CLK_OFF ((SYS_CLK_REQ_LINE_ACTIVE_DEFAULT << 1) | SYS_CLK_REQ_LINE_DISABLE)
#define SYS_CLK_REQ_LINE_MASK (3)

static int clk_pxa95x_26MOUTDMD_enable(struct clk *clk)
{
	/* GEN_REG3_CKRSW1 is used to control CLK26MOUTDMD */
	if (pxa_reg_write(GEN_REG3, GEN_REG3_CKRSW1, GEN_REG3_CKRSW1_MASK) < 0)
		return -EINVAL;

	pr_info("enable CLK26MOUTDMD\n");
	return 0;
}

static void clk_pxa95x_26MOUTDMD_disable(struct clk *clk)
{
	/* GEN_REG3_CKRSW1 is used to control CLK26MOUTDMD */
	if (pxa_reg_write(GEN_REG3, 0, GEN_REG3_CKRSW1_MASK) < 0)
		return;

	pr_info("disable CLK26MOUTDMD\n");
}

static const struct clkops clk_pxa95x_26MOUTDMD_ops = {
	.enable = clk_pxa95x_26MOUTDMD_enable,
	.disable = clk_pxa95x_26MOUTDMD_disable,
};

static int clk_pxa95x_26MOUT_enable(struct clk *clk)
{
	/* GEN_REG3_CKRSW2 is used to control CLK26MOUT */
	if (pxa_reg_write(GEN_REG3, GEN_REG3_CKRSW2, GEN_REG3_CKRSW2_MASK) < 0)
		return -EINVAL;

	pr_info("enable CLK26MOUT\n");
	return 0;
}

static void clk_pxa95x_26MOUT_disable(struct clk *clk)
{
	/* GEN_REG3_CKRSW2 is used to control CLK26MOUT */
	if (pxa_reg_write(GEN_REG3, 0, GEN_REG3_CKRSW2_MASK) < 0)
		return;

	pr_info("disable CLK26MOUT\n");
}

static const struct clkops clk_pxa95x_26MOUT_ops = {
	.enable = clk_pxa95x_26MOUT_enable,
	.disable = clk_pxa95x_26MOUT_disable,
};

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

extern struct dvfs display_dvfs;
static struct clk clk_pxa978_syspll_416;
static int clk_pxa95x_lcd_enable(struct clk *lcd_clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = 0;
	dvfs_freqs.new = lcd_clk->rate / KHZ_TO_HZ;
	dvfs_freqs.dvfs = &display_dvfs;

	pr_debug("Display enable from 0 to %lu.\n", lcd_clk->rate);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);

	if (lcd_clk->rate == 416000000)
		clk_enable(&clk_pxa978_syspll_416);

	CKENC |= (1 << (CKEN_DISPLAY - 64)) | (1 << (CKEN_PIXEL - 64));
	return 0;
}

static void clk_pxa95x_lcd_disable(struct clk *lcd_clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = lcd_clk->rate / KHZ_TO_HZ;
	dvfs_freqs.new = 0;
	dvfs_freqs.dvfs = &display_dvfs;

	pr_debug("Display disable from %lu to 0.\n", lcd_clk->rate);

	CKENC &= ~((1 << (CKEN_DISPLAY - 64))
		   | (1 << (CKEN_PIXEL - 64)));

	if (lcd_clk->rate == 416000000)
		clk_disable(&clk_pxa978_syspll_416);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
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
	struct dvfs_freqs dvfs_freqs;

	/* if dvfm is disabled, do not change lcd rate */
	if (DvfmDisabled)
		return 0;

	if (lcd_clk->refcnt > 0) {
		dvfs_freqs.old = lcd_clk->rate / KHZ_TO_HZ;
		dvfs_freqs.new = rate / KHZ_TO_HZ;
		dvfs_freqs.dvfs = &display_dvfs;

		if (dvfs_freqs.old < dvfs_freqs.new)
			dvfs_notifier_frequency(&dvfs_freqs,
						DVFS_FREQ_PRECHANGE);
	}

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

	if ((lcd_clk->rate != 416000000) && (rate == 416000000)
	    && (lcd_clk->refcnt > 0))
		clk_enable(&clk_pxa978_syspll_416);

	write_accr0(value, mask);

	if ((lcd_clk->rate == 416000000) && (rate != 416000000)
	    && (lcd_clk->refcnt > 0))
		clk_disable(&clk_pxa978_syspll_416);

	if (lcd_clk->refcnt > 0 && dvfs_freqs.old > dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);

	pm_logger_app_add_trace(2, PM_DISPLAY_FREQ_CHANGE, OSCR4,
				dvfs_freqs.old, dvfs_freqs.new);

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
		/* VCODIV_SEL=5 KVCO=5 FBDIV=222(0xDE) REFDIV=3 */
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

static int clk_mmpll_enable(struct clk *clk)
{
	mm_pll_enable(1);
	return 0;
}

static void clk_mmpll_disable(struct clk *clk)
{
	mm_pll_enable(0);
}

static struct clk clk_pxa978_mmpll;
static struct clk clk_pxa978_gcu, clk_pxa95x_vmeta;
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
	/* Only polling status when Gc or Vmeta is at MM PLL */
	if (((clk_pxa978_gcu.rate >= 481000000) && (clk_pxa978_gcu.refcnt > 0))
	    || ((clk_pxa95x_vmeta.rate >= 481000000) && (clk_pxa95x_vmeta.refcnt > 0)))
		while (!(MM_PLL_CTRL & MMPLL_PWR_ST))
			;
}

static unsigned long clk_mmpll_getrate(struct clk *mmpll)
{
	return get_mm_pll_freq() * 1000000;
}

static int clk_mmpll_setrate(struct clk *mmpll, unsigned long rate)
{
	if (mmpll->rate == rate)
		return 0;

	set_mmpll_freq(rate);
	return 0;
}

static const struct clkops clk_mmpll_ops = {
	.enable = clk_mmpll_enable,
	.disable = clk_mmpll_disable,
	.getrate = clk_mmpll_getrate,
	.setrate = clk_mmpll_setrate,
};

static int clk_pxa978_syspll_416_enable(struct clk *clk)
{
	SYS_PLL_416M_CTRL = CLK_EN;
	return 0;
}

static void clk_pxa978_syspll_416_disable(struct clk *clk)
{
	SYS_PLL_416M_CTRL = 0;
}

static const struct clkops clk_pxa978_syspll416_ops = {
	.enable = clk_pxa978_syspll_416_enable,
	.disable = clk_pxa978_syspll_416_disable,
};

extern struct dvfs gc_dvfs, vmeta_dvfs;
/*These function and Variables are used for GC&VMETA stats in debugfs*/
static void gcu_vmeta_stats(struct clk *clk, unsigned long rate);
extern struct gc_vmeta_ticks gc_vmeta_ticks_info;

static int clk_gcu_enable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = 0;
	dvfs_freqs.new = clk->rate / KHZ_TO_HZ;
	dvfs_freqs.dvfs = &gc_dvfs;

	pr_debug("GC enable from 0 to %lu.\n", clk->rate);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);

	if (cpu_is_pxa978_Dx())
		GC_switch_cg_constraint(GET_CG_CONSTRAINT);

	if (clk->rate == 416000000)
		clk_enable(&clk_pxa978_syspll_416);
	else if (clk->rate >= 481000000)
		clk_enable(&clk_pxa978_mmpll);

	CKENC |= ((1 << (CKEN_GC_1X - 64)) | (1 << (CKEN_GC_2X - 64)));
	gc_vmeta_stats_clk_event(GC_CLK_ON);
	if (gc_vmeta_ticks_info.gc_stats_start
	    && !gc_vmeta_ticks_info.gc_stats_stop)
		gcu_vmeta_stats(clk, clk->rate);
	gc_vmeta_ticks_info.gc_state = GC_CLK_ON;

	return 0;
}

static void clk_gcu_disable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = clk->rate / KHZ_TO_HZ;
	dvfs_freqs.new = 0;
	dvfs_freqs.dvfs = &gc_dvfs;

	pr_debug("GC disable from %lu to 0.\n", clk->rate);

	gc_vmeta_stats_clk_event(GC_CLK_OFF);
	CKENC &= ~((1 << (CKEN_GC_1X - 64)) | (1 << (CKEN_GC_2X - 64)));
	if (clk->rate == 416000000)
		clk_disable(&clk_pxa978_syspll_416);
	else if (clk->rate >= 481000000)
		clk_disable(&clk_pxa978_mmpll);

	if (cpu_is_pxa978_Dx())
		GC_switch_cg_constraint(RELEASE_CG_CONSTRAINT);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
	if (gc_vmeta_ticks_info.gc_stats_start
	    && !gc_vmeta_ticks_info.gc_stats_stop)
		gcu_vmeta_stats(clk, clk->rate);
	gc_vmeta_ticks_info.gc_state = GC_CLK_OFF;
}

static long clk_pxa95x_gc_round_rate(struct clk *gc_clk, unsigned long rate)
{
	int i, temp;
	if (cpu_is_pxa978_Dx()) {
		temp = dvfm_current_axi_freq_get();
		min_gc = temp > 0 ? (temp * 2 * MHZ_TO_HZ) : 0;
	}
	for (i = 0; i < gc_freq_counts; i++) {
		if (cpu_is_pxa978_Dx() && (gc_cur_freqs_table[i] < min_gc))
			continue;
		if (rate <= gc_cur_freqs_table[i]) {
			rate = gc_cur_freqs_table[i];
			break;
		}
		if (i == gc_freq_counts - 1)
			rate = gc_cur_freqs_table[gc_freq_counts - 1];
	}

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

/* flag = 1 means gc frequency change
 * flag = 0 means vmeta frequency change
 */

static unsigned long clk_pxa95x_vmeta_getrate(struct clk *vmeta_clk);
static void mm_pll_setting(int flag, unsigned long rate, unsigned int value,
			   unsigned int mask)
{
	unsigned int tmp, ori_gc, ori_vmeta;
	unsigned long gc_rate, vm_rate, flags;

	spin_lock_irqsave(&mmpll_lock, flags);

	gc_rate = clk_pxa978_gcu.rate;
	vm_rate = clk_pxa95x_vmeta.rate;

	pr_debug("mm_pll_setting: current gc is %lu, vmeta is %lu.\n"
		 "Set to %lu by %s.\n", gc_rate, vm_rate, rate,
		 flag ? "GC" : "vMeta");

	tmp = ACCR0;
	ori_gc = ACCR0 & (ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
	ori_vmeta = ACCR0 & ACCR0_VMFC_MASK;
	if (gc_rate >= 481000000) {
		/* Switch to 416Mhz */
		tmp &= ~(ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
		tmp |= (0x3 << ACCR0_GCFS_OFFSET | 0x3 << ACCR0_GCAXIFS_OFFSET);
		if (!flag && (gc_rate != rate)) {
			value |= ori_gc;
			mask |= (ACCR0_GCFS_MASK | ACCR0_GCAXIFS_MASK);
			clk_pxa978_gcu.rate = rate;
		}
	}
	if (vm_rate >= 481000000) {
		/* Switch to 416Mhz */
		tmp &= ~ACCR0_VMFC_MASK;
		tmp |= 0x3 << ACCR0_VMFC_OFFSET;
		if (flag && (vm_rate != rate)) {
			value |= ori_vmeta;
			mask |= ACCR0_VMFC_MASK;
			clk_pxa95x_vmeta.rate = rate;
		}
	}

	clk_enable(&clk_pxa978_syspll_416);

	ACCR0 = tmp;
	set_mmpll_freq(rate);
	write_accr0(value, mask);

	clk_disable(&clk_pxa978_syspll_416);

	spin_unlock_irqrestore(&mmpll_lock, flags);
}

#define GC_FC 1
#define VMETA_FC 0
void update_GC_VMETA_op_cycle(int gvsel, unsigned long new_rate,
			      unsigned int runtime, unsigned int idletime)
{
	int op_idx;

	if (gvsel == GC_FC) {
		if (!gc_vmeta_ticks_info.gc_cur_freq)
			gc_vmeta_ticks_info.gc_cur_freq = new_rate;
		for (op_idx = 0; op_idx < gc_freq_counts; op_idx++)
			if (gc_vmeta_ticks_info.gc_cur_freq ==
			    gc_cur_freqs_table[op_idx])
				break;
		gc_vmeta_ticks_info.GC_op_ticks_array[op_idx].runtime +=
		    runtime;
		gc_vmeta_ticks_info.GC_op_ticks_array[op_idx].idletime +=
		    idletime;
		if (gc_vmeta_ticks_info.gc_cur_freq != new_rate) {
			gc_vmeta_ticks_info.GC_op_ticks_array[op_idx].count++;
			gc_vmeta_ticks_info.gc_cur_freq = new_rate;
		}
		gc_vmeta_ticks_info.gc_total_ticks += (runtime + idletime);
	} else {
		if (!gc_vmeta_ticks_info.vm_cur_freq)
			gc_vmeta_ticks_info.vm_cur_freq = new_rate;
		for (op_idx = 0; op_idx < gc_freq_counts; op_idx++)
			if (gc_vmeta_ticks_info.vm_cur_freq ==
			    gc_cur_freqs_table[op_idx])
				break;
		gc_vmeta_ticks_info.VM_op_ticks_array[op_idx].runtime +=
		    runtime;
		gc_vmeta_ticks_info.VM_op_ticks_array[op_idx].idletime +=
		    idletime;
		if (gc_vmeta_ticks_info.vm_cur_freq != new_rate) {
			gc_vmeta_ticks_info.VM_op_ticks_array[op_idx].count++;
			gc_vmeta_ticks_info.vm_cur_freq = new_rate;
		}
		gc_vmeta_ticks_info.vm_total_ticks += (runtime + idletime);
	}
}

static void gcu_vmeta_stats(struct clk *clk, unsigned long rate)
{
	if (!strcmp(clk->name, "GCCLK")) {
		unsigned int timestamp, time;

		timestamp = read_curtime();
		time =
		    (timestamp >=
		     gc_vmeta_ticks_info.gc_prev_timestamp) ? timestamp -
		    gc_vmeta_ticks_info.gc_prev_timestamp : 0xFFFFFFFF -
		    gc_vmeta_ticks_info.gc_prev_timestamp + timestamp;

		gc_vmeta_ticks_info.gc_prev_timestamp = timestamp;

		if (gc_vmeta_ticks_info.gc_state == GC_CLK_ON)
			update_GC_VMETA_op_cycle(GC_FC, rate, time, 0);
		else
			update_GC_VMETA_op_cycle(GC_FC, rate, 0, time);

		if (rate >= 481000000) {
			if (gc_vmeta_ticks_info.vm_stats_start
			    && (gc_vmeta_ticks_info.vm_cur_freq >= 481000000)) {
				gc_vmeta_ticks_info.vm_prev_timestamp =
				    timestamp;
				if (gc_vmeta_ticks_info.vmeta_state ==
				    VMETA_CLK_ON)
					update_GC_VMETA_op_cycle(VMETA_FC, rate,
								 time, 0);
				else
					update_GC_VMETA_op_cycle(VMETA_FC, rate,
								 0, time);
			}
		}
	} else if (!strcmp(clk->name, "VMETA_CLK")) {
		unsigned int timestamp, time;

		timestamp = read_curtime();
		time =
		    (timestamp >=
		     gc_vmeta_ticks_info.vm_prev_timestamp) ? timestamp -
		    gc_vmeta_ticks_info.vm_prev_timestamp : 0xFFFFFFFF -
		    gc_vmeta_ticks_info.vm_prev_timestamp + timestamp;

		gc_vmeta_ticks_info.vm_prev_timestamp = timestamp;

		if (gc_vmeta_ticks_info.vmeta_state == VMETA_CLK_ON)
			update_GC_VMETA_op_cycle(VMETA_FC, rate, time, 0);
		else
			update_GC_VMETA_op_cycle(VMETA_FC, rate, 0, time);
		if (rate >= 481000000) {
			if (gc_vmeta_ticks_info.gc_stats_start
			    && (gc_vmeta_ticks_info.gc_cur_freq >= 481000000)) {
				gc_vmeta_ticks_info.gc_prev_timestamp =
				    timestamp;
				if (gc_vmeta_ticks_info.gc_state == GC_CLK_ON)
					update_GC_VMETA_op_cycle(GC_FC, rate,
								 time, 0);
				else
					update_GC_VMETA_op_cycle(GC_FC, rate, 0,
								 time);
			}
		}
	}
}

int get_gcu_freqs_table(unsigned long *gcu_freqs_table,
			unsigned int *item_counts, unsigned int max_item_counts)
{
	int i;

	if (!cpu_is_pxa978_Dx()) {
		if (max_item_counts < ARRAY_SIZE(pxa978_gcvmeta_clk_table)) {
			pr_err("Too many GC frequencies!\n");
			return -1;
		}
		for (i = 0;
		     (pxa978_gcvmeta_clk_table[i].frequency !=
		      DEVFREQ_TABLE_END)
		     && (pxa978_gcvmeta_clk_table[i].frequency <= max_gc); i++)
			gcu_freqs_table[i] =
			    pxa978_gcvmeta_clk_table[i].frequency;
	} else {
		if (max_item_counts < ARRAY_SIZE(pxa978_dx_gcvmeta_clk_table)) {
			pr_err("Too many GC frequencies!\n");
			return -1;
		}
		for (i = 0;
		     (pxa978_dx_gcvmeta_clk_table[i].frequency !=
		      DEVFREQ_TABLE_END)
		     && (pxa978_dx_gcvmeta_clk_table[i].frequency <= max_gc);
		     i++)
			gcu_freqs_table[i] =
			    pxa978_dx_gcvmeta_clk_table[i].frequency;
	}
	*item_counts = i;

	return 0;
}

EXPORT_SYMBOL(get_gcu_freqs_table);

/* Frequency is in unit of Khz*/
static struct devfreq_frequency_table pxa978_vmeta_clk_table[] = {
	INIT_FREQ_TABLE(1, 156000),
	INIT_FREQ_TABLE(2, 208000),
	INIT_FREQ_TABLE(3, 312000),
	INIT_FREQ_TABLE(4, 416000),
	INIT_FREQ_TABLE(5, 498000),
	INIT_FREQ_TABLE(6, 600000),
	INIT_FREQ_TABLE(7, DEVFREQ_TABLE_END),
};

static struct devfreq_frequency_table pxa978_dx_vmeta_clk_table[] = {
	INIT_FREQ_TABLE(1, 156000),
	INIT_FREQ_TABLE(2, 208000),
	INIT_FREQ_TABLE(3, 312000),
	INIT_FREQ_TABLE(4, 416000),
	INIT_FREQ_TABLE(5, 481000),
	INIT_FREQ_TABLE(6, 600000),
	INIT_FREQ_TABLE(7, DEVFREQ_TABLE_END),
};

int set_vmeta_freqs_table(struct devfreq *devfreq)
{
	if (cpu_is_pxa978_Dx())
		devfreq_set_freq_table(devfreq, pxa978_dx_vmeta_clk_table);
	else
		devfreq_set_freq_table(devfreq, pxa978_vmeta_clk_table);

	if (max_vmeta)
		devfreq->max_freq = max_vmeta / KHZ_TO_HZ;
	return 0;
}

static int clk_pxa95x_gcu_setrate(struct clk *gc_clk, unsigned long rate)
{
	unsigned int value, mask = 0x3F << 6;
	struct dvfs_freqs dvfs_freqs;

	/* if dvfm is disabled, do not change gc rate */
	if (DvfmDisabled)
		return 0;

	if (gc_clk->rate == rate)
		return 0;
	pr_debug("gc setrate from %lu to %lu.\n", gc_clk->rate, rate);

	mutex_lock(&gc_vmeta_mutex);

	last_gc = rate;
	if ((rate >= 481000000) && (rate < last_vmeta))
		rate = last_vmeta;

	if (gc_clk->refcnt > 0) {
		dvfs_freqs.old = gc_clk->rate / KHZ_TO_HZ;
		dvfs_freqs.new = rate / KHZ_TO_HZ;
		dvfs_freqs.dvfs = &gc_dvfs;
		if (dvfs_freqs.old < dvfs_freqs.new)
			dvfs_notifier_frequency(&dvfs_freqs,
						DVFS_FREQ_PRECHANGE);
	}

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

	if ((gc_clk->rate != 416000000) && (rate == 416000000)
	    && (gc_clk->refcnt > 0))
		clk_enable(&clk_pxa978_syspll_416);
	else if ((gc_clk->rate < 481000000) && (rate >= 481000000)
		 && (gc_clk->refcnt > 0))
		clk_enable(&clk_pxa978_mmpll);

	if (rate < 481000000)
		write_accr0(value << 6, mask);
	else if (get_mm_pll_freq() * 1000000 == rate)
		write_accr0(value << 6, mask);
	else {
		if ((clk_pxa95x_vmeta.refcnt > 0) && (clk_pxa95x_vmeta.rate >= 481000000)
		    && (clk_pxa95x_vmeta.rate < rate)) {
			dvfs_freqs.old = clk_pxa95x_vmeta.rate / KHZ_TO_HZ;
			dvfs_freqs.new = rate / KHZ_TO_HZ;
			dvfs_freqs.dvfs = &vmeta_dvfs;
			dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);
		}

		mm_pll_setting(1, rate, value << 6, mask);

		if ((clk_pxa95x_vmeta.refcnt > 0) && (clk_pxa95x_vmeta.rate >= 481000000)
		    && (clk_pxa95x_vmeta.rate > rate)) {
			dvfs_freqs.old = clk_pxa95x_vmeta.rate / KHZ_TO_HZ;
			dvfs_freqs.new = rate / KHZ_TO_HZ;
			dvfs_freqs.dvfs = &vmeta_dvfs;
			dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
		}
	}

	if ((gc_clk->rate == 416000000) && (rate != 416000000)
	    && (gc_clk->refcnt > 0))
		clk_disable(&clk_pxa978_syspll_416);
	else if ((gc_clk->rate >= 481000000) && (rate < 481000000)
		 && (gc_clk->refcnt > 0))
		clk_disable(&clk_pxa978_mmpll);

	if (gc_vmeta_ticks_info.gc_stats_start
	    && !gc_vmeta_ticks_info.gc_stats_stop)
		gcu_vmeta_stats(gc_clk, rate);
	else
		gc_vmeta_ticks_info.gc_cur_freq = rate;

	if (gc_clk->refcnt > 0 && dvfs_freqs.old > dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);

	pm_logger_app_add_trace(2, PM_GC_FREQ_CHANGE, OSCR4,
				dvfs_freqs.old, dvfs_freqs.new);

	mutex_unlock(&gc_vmeta_mutex);

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

static int clk_pxa95x_vmeta_setrate(struct clk *vmeta_clk, unsigned long rate)
{
	unsigned int value, mask = 0x7 << 3;
	struct dvfs_freqs dvfs_freqs;

	/* if dvfm is disabled, do not change vmeta rate */
	if (DvfmDisabled)
		return 0;

	pr_debug("vmeta setrate from %lu to %lu.\n", vmeta_clk->rate, rate);

	mutex_lock(&gc_vmeta_mutex);

	last_vmeta = rate;
	if ((rate >= 481000000) && rate < last_gc)
		rate = last_gc;


	if (vmeta_clk->refcnt > 0) {
		dvfs_freqs.old = vmeta_clk->rate / KHZ_TO_HZ;
		dvfs_freqs.new = rate / KHZ_TO_HZ;
		dvfs_freqs.dvfs = &vmeta_dvfs;
		if (dvfs_freqs.old < dvfs_freqs.new)
			dvfs_notifier_frequency(&dvfs_freqs,
						DVFS_FREQ_PRECHANGE);
	}

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

	if ((vmeta_clk->rate != 416000000) && (rate == 416000000)
	    && (vmeta_clk->refcnt > 0))
		clk_enable(&clk_pxa978_syspll_416);
	else if ((vmeta_clk->rate < 481000000) && (rate >= 481000000)
		 && (vmeta_clk->refcnt > 0))
		clk_enable(&clk_pxa978_mmpll);

	if (rate < 481000000)
		write_accr0(value << 3, mask);
	else if (get_mm_pll_freq() * 1000000 == rate)
		write_accr0(value << 3, mask);
	else {
		if ((clk_pxa978_gcu.refcnt > 0) && (clk_pxa978_gcu.rate >= 481000000)
		    && (clk_pxa978_gcu.rate < rate)) {
			dvfs_freqs.old = clk_pxa978_gcu.rate / KHZ_TO_HZ;
			dvfs_freqs.new = rate / KHZ_TO_HZ;
			dvfs_freqs.dvfs = &gc_dvfs;
			dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);
		}

		mm_pll_setting(0, rate, value << 3, mask);

		if ((clk_pxa978_gcu.refcnt > 0) && (clk_pxa978_gcu.rate >= 481000000)
		    && (clk_pxa978_gcu.rate > rate)) {
			dvfs_freqs.old = clk_pxa978_gcu.rate / KHZ_TO_HZ;
			dvfs_freqs.new = rate / KHZ_TO_HZ;
			dvfs_freqs.dvfs = &gc_dvfs;
			dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
		}
	}

	if ((vmeta_clk->rate == 416000000) && (rate != 416000000)
	    && (vmeta_clk->refcnt > 0))
		clk_disable(&clk_pxa978_syspll_416);
	else if ((vmeta_clk->rate >= 481000000) && (rate < 481000000)
		 && (vmeta_clk->refcnt > 0))
		clk_disable(&clk_pxa978_mmpll);

	if (gc_vmeta_ticks_info.vm_stats_start
	    && !gc_vmeta_ticks_info.vm_stats_stop)
		gcu_vmeta_stats(vmeta_clk, rate);
	else
		gc_vmeta_ticks_info.vm_cur_freq = rate;

	if (vmeta_clk->refcnt > 0 && dvfs_freqs.old > dvfs_freqs.new)
		dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);

	pm_logger_app_add_trace(2, PM_VM_FREQ_CHANGE, OSCR4,
				dvfs_freqs.old, dvfs_freqs.new);

	mutex_unlock(&gc_vmeta_mutex);

	return 0;
}

static int clk_pxa95x_vmeta_enable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	if (cpu_is_pxa978_Dx())
		switch_vmeta_cg_constraint(GET_CG_CONSTRAINT);

	dvfs_freqs.old = 0;
	dvfs_freqs.new = clk->rate / KHZ_TO_HZ;
	dvfs_freqs.dvfs = &vmeta_dvfs;

	pr_debug("vmeta enable from 0 to %lu.\n", clk->rate);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_PRECHANGE);
	if (clk->rate == 416000000)
		clk_enable(&clk_pxa978_syspll_416);
	else if (clk->rate >= 481000000)
		clk_enable(&clk_pxa978_mmpll);

	CKENB |= (1 << (clk->enable_val - 32));
	gc_vmeta_stats_clk_event(VMETA_CLK_ON);
	if (gc_vmeta_ticks_info.vm_stats_start
	    && !gc_vmeta_ticks_info.vm_stats_stop)
		gcu_vmeta_stats(clk, clk->rate);
	gc_vmeta_ticks_info.vmeta_state = VMETA_CLK_ON;

	return 0;
}

static void clk_pxa95x_vmeta_disable(struct clk *clk)
{
	struct dvfs_freqs dvfs_freqs;

	dvfs_freqs.old = clk->rate / KHZ_TO_HZ;
	dvfs_freqs.new = 0;
	dvfs_freqs.dvfs = &vmeta_dvfs;

	pr_debug("vmeta disable from %lu to 0.\n", clk->rate);

	gc_vmeta_stats_clk_event(VMETA_CLK_OFF);
	CKENB &= ~(1 << (clk->enable_val - 32));
	if (clk->rate == 416000000)
		clk_disable(&clk_pxa978_syspll_416);
	else if (clk->rate >= 481000000)
		clk_disable(&clk_pxa978_mmpll);

	dvfs_notifier_frequency(&dvfs_freqs, DVFS_FREQ_POSTCHANGE);
	if (gc_vmeta_ticks_info.vm_stats_start
	    && !gc_vmeta_ticks_info.vm_stats_stop)
		gcu_vmeta_stats(clk, clk->rate);
	gc_vmeta_ticks_info.vmeta_state = VMETA_CLK_OFF;

	if (cpu_is_pxa978_Dx())
		switch_vmeta_cg_constraint(RELEASE_CG_CONSTRAINT);
}

static const struct clkops clk_pxa95x_vmeta_ops = {
	.init = common_clk_init,
	.disable = clk_pxa95x_vmeta_disable,
	.enable = clk_pxa95x_vmeta_enable,
	.round_rate = clk_pxa95x_vmeta_round_rate,
	.getrate = clk_pxa95x_vmeta_getrate,
	.setrate = clk_pxa95x_vmeta_setrate,
};

static unsigned long clk_pxa978_peri_pll_getrate(struct clk *peri_pll_clk)
{
	unsigned long rate;
	unsigned int fbdiv, refdiv, vcodiv_sel;
	u32 peri_pll_param;

	peri_pll_param = PERI_PLL_PARAM;

	fbdiv = (peri_pll_param >> 5) & 0x1ff;
	refdiv = peri_pll_param & 0x1f;

	switch ((peri_pll_param >> 20) & 0xf) {
	case 2:
		vcodiv_sel = 2;
		break;
	case 4:
		vcodiv_sel = 3;
		break;
	case 8:
		vcodiv_sel = 8;
		break;
	default:
		printk(KERN_ERR
		       "VCODIV_SEL = 0x1 is not a recommended value\n");
		return -EINVAL;
		break;
	}

	rate = (26 * fbdiv / refdiv) / vcodiv_sel;
	rate *= MHZ_TO_HZ;

	return rate;
}

static int clk_pxa978_peri_pll_enable(struct clk *clk)
{
	PERI_PLL_CTRL |= PERIPLL_PWRON;
	while (!(PERI_PLL_CTRL & PERIPLL_PWR_ST))
		;
	return 0;
}

static void clk_pxa978_peri_pll_disable(struct clk *clk)
{
	PERI_PLL_CTRL &= ~PERIPLL_PWRON;
}

static const struct clkops clk_pxa978_peri_pll_ops = {
	.enable = clk_pxa978_peri_pll_enable,
	.disable = clk_pxa978_peri_pll_disable,
	.getrate = clk_pxa978_peri_pll_getrate,
};

static int clk_ddr_pll_enable(struct clk *clk)
{
	OSCC &= ~OSCC_DPRM;
	while (OSCC & OSCC_DPRM)
		;
	/* turn on DDR PLL by AGENP[DDRPLL_CTRL] and AGENP[DDRPLL_DATA] */
	AGENP |= (AGENP_DDRPLL_CTRL | AGENP_DDRPLL_DATA);

	/* wait until DDR PLL is ready for use as DMC clock source */
	while (!(OSCC & OSCC_DPLS))
		;
	return 0;
}

static void clk_ddr_pll_disable(struct clk *clk)
{
	unsigned int agenp;
	agenp = AGENP;
	/* turn off DDR PLL, only set AGENP[DDRPLL_CTRL] */
	agenp &= ~(AGENP_DDRPLL_CTRL | AGENP_DDRPLL_DATA);
	agenp |= AGENP_DDRPLL_CTRL;
	AGENP = agenp;
	while (OSCC & OSCC_DPLS)
		;
	/*
	 * mask enable/disable DDRPLL commands initiated
	 * by AGENP[DDRPLL_CTRL]
	 */
	OSCC |= OSCC_DPRM;
	while (!(OSCC & OSCC_DPRM))
		;
}

static const struct clkops clk_ddr_pll_ops = {
	.enable = clk_ddr_pll_enable,
	.disable = clk_ddr_pll_disable,
};

static inline unsigned int get_ddr_pll_freq(void)
{
	unsigned int ddrpll, m, n, vcodiv_sel, vcodiv, freq;
	ddrpll = DDRPLLR;
	m = ddrpll & REFDIV_MASK;
	n = (ddrpll & FBDIV_MASK) >> 5;
	vcodiv_sel = (ddrpll & VCODIV_SEL_MASK) >> 17;
	switch (vcodiv_sel) {
	case 0:
		vcodiv = 1;
		break;
	case 2:
		vcodiv = 2;
		break;
	case 4:
		vcodiv = 3;
		break;
	case 5:
		vcodiv = 4;
		break;
	case 6:
		vcodiv = 5;
		break;
	case 7:
		vcodiv = 6;
		break;
	case 8:
		vcodiv = 8;
		break;
	default:
		pr_err("wrong ddr pll vcodiv!\n");
		BUG_ON(1);
		break;
	}
	freq = 26 * n / m / vcodiv;
	return freq;
}

static inline void choose_regtable(unsigned int table_index, u16 tab_inited)
{
	if (tab_inited) {
		if ((table_index < REG_TABLE_MAX)) {
			pr_debug("%d Regtable is selected\n", table_index);
			DDR_FC_REG_TBL = table_index;
		} else {
			pr_err("%d Regtable is not existed\n", table_index);
			BUG_ON(1);
		}
	} else {
		pr_debug("old mechanism to choose DDR Regtable, index is %d\n",
			 table_index);
		switch (table_index) {
		case 7:
		case 6:
		case 5:
			DDR_FC_REG_TBL = 3;
			break;
		case 4:
			DDR_FC_REG_TBL = 2;
			break;
		case 3:
		case 2:
		case 1:
			DDR_FC_REG_TBL = 1;
			break;
		case 0:
			DDR_FC_REG_TBL = 0;
			break;
		default:
			BUG_ON(1);
			break;
		}
	}
}

static void regtable_init(void)
{
	/* This function is used init Reg table 0,1, and 3
	 * Reg table 2 can be used dynamicly
	 */
	u16 tab_cnt, en_count, sel_tab_cnt = 0;

	if (!dmc_base)
		return;
	for (tab_cnt = 0; tab_cnt < used_pro_count; tab_cnt++) {
		if ((reg_table_profiles[tab_cnt].freq == 104)
		    || (reg_table_profiles[tab_cnt].freq == 208)
		    || (reg_table_profiles[tab_cnt].freq == 533)) {
			if (reg_table_profiles[tab_cnt].freq == 104)
				sel_tab_cnt = 0;
			else if (reg_table_profiles[tab_cnt].freq == 208)
				sel_tab_cnt = 1;
			else if (reg_table_profiles[tab_cnt].freq == 533)
				sel_tab_cnt = 3;

			pr_debug("Table : %d\n", sel_tab_cnt);
			pr_debug("Freq: %dMHz\n",
				 reg_table_profiles[tab_cnt].freq);
			pr_debug("Profile : %d\n", tab_cnt);
			for (en_count = 0; en_count <= entry_count[tab_cnt];
			     en_count++) {
				unsigned int tmp_data_1, tmp_table_ctrl;

				tmp_table_ctrl =
				    (sel_tab_cnt << 5) | en_count | (1 << 31);

				if (!reg_table_profiles[tab_cnt].
				    offset[en_count]
				    && !reg_table_profiles[tab_cnt].
				    data[en_count])
					tmp_data_1 = 0 | (1 << 17);
				else
					tmp_data_1 =
					    (reg_table_profiles[tab_cnt].
					     offset[en_count] & 0xFFF);

				writel(reg_table_profiles[tab_cnt].
				       data[en_count],
				       dmc_base + REG_TABLE_DATA_0);
				writel(tmp_data_1, dmc_base + REG_TABLE_DATA_1);

				writel(tmp_table_ctrl,
				       dmc_base + REG_TABLE_CONTROL_0);
			}
		}
	}
}

static int regtable_program(unsigned int ddr_new_freq)
{
	u16 tab_cnt, en_count;
	static unsigned int pre_freq;
	unsigned int dclk = ddr_new_freq >> 1;
	int ret = 4;
	if (!dmc_base)
		return 0;
	/*Workaround for SaarC: OBM freq is not aligned to kernel's */
	if (dclk == 450) {
		pr_err
		    ("SaarC Freq workaround, Configure 450MHz"
		     "with 472MHz setting\n");
		dclk = 472;
	}
	if (dclk == 225) {
		pr_err
		    ("SaarC Freq workaround, Configure 225MHz"
		     "with 208MHz setting\n");
		dclk = 208;
	}
	/*Workaround end */
	pr_debug("dclk is %d\n", dclk);
	switch (dclk) {
	case 104:
		ret = 0;
		break;
	case 208:
		ret = 1;
		break;
	case 533:
		ret = 3;
		break;
	default:
		if (pre_freq != dclk)
			pre_freq = dclk;
		else {
			pr_debug("Reg table 2 has been configured\n");
			ret = 2;
			break;
		}
		for (tab_cnt = 0; tab_cnt < DDR_FROFILE_MAX_NUM; tab_cnt++) {
			pr_debug("Searching... Current Profile's Freq : %d\n",
				 reg_table_profiles[tab_cnt].freq);
			if (reg_table_profiles[tab_cnt].freq == dclk) {
				for (en_count = 0;
				     en_count <= entry_count[tab_cnt];
				     en_count++) {
					unsigned int tmp_data_1, tmp_table_ctrl;

					tmp_table_ctrl =
					    (DYNAMIC_TABLE_INDEX << 5) |
					    en_count | (1 << 31);

					if (!reg_table_profiles[tab_cnt].
					    offset[en_count]
					    && !reg_table_profiles[tab_cnt].
					    data[en_count])
						tmp_data_1 = 0 | (1 << 17);
					else
						tmp_data_1 =
						    (reg_table_profiles
						     [tab_cnt].
						     offset[en_count] & 0xFFF);

					writel(reg_table_profiles[tab_cnt].
					       data[en_count],
					       dmc_base + REG_TABLE_DATA_0);
					writel(tmp_data_1,
					       dmc_base + REG_TABLE_DATA_1);

					writel(tmp_table_ctrl,
					       dmc_base + REG_TABLE_CONTROL_0);
				}
				ret = DYNAMIC_TABLE_INDEX;
				break;
			}
		}
		break;
	}
	return ret;
}

static inline unsigned int ddr_pll_freq2reg(unsigned long x)
{
	switch (x) {
	case 797000000:
		/* KVCO=3 VCODIV_SEL=2 FBDIV=184(0xB8) REFDIV=3 */
		return 3 << 21 | 2 << 17 | 0xB8 << 5 | 3 << 0;
	case 901000000:
		/* KVCO=4 VCODIV_SEL=2 FBDIV=208(0xD0) REFDIV=3 */
		return 4 << 21 | 2 << 17 | 0xD0 << 5 | 3 << 0;
	case 944000000:
		/* KVCO=4 VCODIV_SEL=2 FBDIV=218(0xDA) REFDIV=3 */
		return 4 << 21 | 2 << 17 | 0xDA << 5 | 3 << 0;
	case 1066000000:
		/* KVCO=6 VCODIV_SEL=2 FBDIV=246(0xF6) REFDIV=3 */
		return 6 << 21 | 2 << 17 | 0xF6 << 5 | 3 << 0;
	default:
		pr_err("Unsupported DDR PLL frequency %ld!\n", x);
		return 0;
	}
}

static unsigned long get_ddr_pll_freq_from_dmcfs(unsigned long dmcfs)
{
	unsigned int freq;
	if (dmcfs == 398000000)
		freq = 797000000;
	else if (dmcfs == 450000000)
		freq = 901000000;
	else if (dmcfs == 472000000)
		freq = 944000000;
	else if (dmcfs == 533000000)
		freq = 1066000000;
	else if ((dmcfs == 797000000) || (dmcfs == 901000000) ||
		 (dmcfs == 944000000) || (dmcfs == 1066000000))
		freq = dmcfs;
	else
		freq = 0;

	return freq;
}

/*
 *Return value indicates whether switches to DDR 416Mhz
 * 1 means yes, 0 means no
 */

static inline int set_ddr_pll_freq(unsigned long old,
				   unsigned long new, uint32_t accr)
{
	unsigned long ddrpll, oldfreq, newfreq, data, mask;
	int ret = 0;
	oldfreq = get_ddr_pll_freq_from_dmcfs(old);
	newfreq = get_ddr_pll_freq_from_dmcfs(new);
	if (newfreq && newfreq != (get_ddr_pll_freq() * 1000000)) {
		ddrpll = DDRPLLR;
		ddrpll &= ~(KVCO_MASK | VCODIV_SEL_MASK |
			    FBDIV_MASK | REFDIV_MASK | DDRPLL_FC_MASK);
		ddrpll |= ddr_pll_freq2reg(newfreq);
		ddrpll |= 1 << DDRPLL_FC_OFFSET;

		if (oldfreq) {
			/*Already on DDR PLL, need to switch to 416Mhz
			   before change DDR PLL */
			clk_enable(&clk_pxa978_syspll_416);
			data = 0x3 << 11;
			mask = ACCR_DMCFS_MASK_978;
			accr &= ~mask;
			accr |= data;
			pr_debug("Switch to 208MHz\n");
			if (ddr_reg_inited)
				choose_regtable(1, ddr_reg_inited);
			else
				choose_regtable(3, ddr_reg_inited);
			write_accr_in_sram((u32) sram_map + 0x9000,
					   (u32) sram_map + 0xa000 - 4, accr,
					   data, mask, clkmgr_base, dmc_base);
			ret = 1;
		}
		DDRPLLR = ddrpll;
		do {
			ddrpll = DDRPLLR;
		} while ((ddrpll & DDRPLL_FC_MASK) >> DDRPLL_FC_OFFSET);
		while (!(OSCC & OSCC_DPLS))
			;
	}
	return ret;
}

unsigned long clk_ddr_getrate(struct clk *clk)
{
	unsigned long dmcfs = (ACSR >> ACCR_DMCFS_OFFSET_978) & 0x07;
	switch (dmcfs) {
	case 0:
		dmcfs = 52000000;
		break;
	case 1:
		dmcfs = 208000000;
		break;
	case 2:
		dmcfs = 312000000;
		break;
	case 3:
		dmcfs = 416000000;
		break;
	case 4:
		dmcfs = (get_ddr_pll_freq() >> 1) * 1000000;
		break;
	case 5:
		dmcfs = (get_core_pll() >> 1) * 1000000;
		break;
	case 6:
		dmcfs = get_ddr_pll_freq() * 1000000;
		break;
	case 7:
		dmcfs = get_core_pll() * 1000000;
		break;
	default:
		return -1;
	}
	return dmcfs;
}

long clk_ddr_round_rate(struct clk *clk, unsigned long rate)
{
	if (rate <= 208000000)
		rate = 208000000;
	else if (rate <= 312000000)
		rate = 312000000;
	else if (rate <= 416000000)
		rate = 416000000;
	else if (rate <= 797000000)
		rate = 797000000;
	else if (rate <= 944000000)
		rate = 944000000;
	else
		rate = 1066000000;
	return rate;
}

int ddr_freq_to_table_idx(unsigned long ddr_freq)
{
	int i;
	for (i = 0; pxa978_ddr_clk_table[i].frequency != DEVFREQ_TABLE_END;
									i++)
		if (pxa978_ddr_clk_table[i].frequency == ddr_freq / KHZ_TO_HZ)
			return i;
	return -1;
}
EXPORT_SYMBOL(ddr_freq_to_table_idx);

unsigned long ddr_table_idx_to_freq(int idx)
{
	if ((idx >= 0) && (idx < ARRAY_SIZE(pxa978_ddr_clk_table)))
		return pxa978_ddr_clk_table[idx].frequency;
	return -1;
}
EXPORT_SYMBOL(ddr_table_idx_to_freq);

int ddr_get_table_size(void)
{
	return ARRAY_SIZE(pxa978_ddr_clk_table) - 1;
}
EXPORT_SYMBOL(ddr_get_table_size);

static struct clk clk_pxa978_ddr_pll;
extern struct dvfs ddr_main_dvfs, ddr_mem_dvfs;
int clk_ddr_setrate(struct clk *clk, unsigned long rate)
{
	unsigned long flags;
	unsigned int value, accr, data, old, new;
	int ddr416;
	struct dvfs_freqs ddr_main_dvfs_freqs, ddr_mem_dvfs_freqs;
	/* if dvfm is disabled, do not change DDR rate */
	if (DvfmDisabled)
		return 0;
	pr_debug("ddr setrate from %lu to %lu.\n", clk->rate, rate);

	ddr_main_dvfs_freqs.old = ((clk->rate / MHZ_TO_HZ) >> 1) * KHZ_TO_HZ;
	ddr_main_dvfs_freqs.new = ((rate / MHZ_TO_HZ) >> 1) * KHZ_TO_HZ;
	ddr_main_dvfs_freqs.dvfs = &ddr_main_dvfs;
	dvfs_notifier_frequency(&ddr_main_dvfs_freqs, DVFS_FREQ_PRECHANGE);

	ddr_mem_dvfs_freqs.old = ((clk->rate / MHZ_TO_HZ) >> 1) * KHZ_TO_HZ;
	ddr_mem_dvfs_freqs.new = ((rate / MHZ_TO_HZ) >> 1) * KHZ_TO_HZ;
	ddr_mem_dvfs_freqs.dvfs = &ddr_mem_dvfs;
	dvfs_notifier_frequency(&ddr_mem_dvfs_freqs, DVFS_FREQ_PRECHANGE);

	update_ddr_performance_data();
	stop_ddr_performance_counter();

	local_fiq_disable();
	local_irq_save(flags);

	old = clk->rate;
	new = rate;

	if (rate == 52000000)
		value = 0;
	else if (rate == 208000000)
		value = 1;
	else if (rate == 312000000)
		value = 2;
	else if (rate == 416000000)
		value = 3;
	else if ((rate == 398000000) || (rate == 450000000) ||
		 (rate == 472000000) || (rate == 533000000))
		value = 4;
	else if ((rate == 797000000) || (rate == 901000000) ||
		 (rate == 944000000) || (rate == 1066000000))
		value = 6;
	else {
		printk(KERN_ERR "DDR don't suppport rate: %lu\n", rate);
		return -1;
	}

	if ((rate != 52000000) && (rate != 208000000)
	    && (rate != 312000000) && (rate != 416000000)
	    && clk_pxa978_ddr_pll.refcnt == 0)
		clk_enable(&clk_pxa978_ddr_pll);

	data = ACCR_DMCFS_978(value);
	accr = ACCR;
	accr &= ~ACCR_DMCFS_MASK_978;
	accr |= data;

	if ((416000000 != old) && (416000000 == new))
		clk_enable(&clk_pxa978_syspll_416);
	ddr416 = set_ddr_pll_freq(old, new, accr);

	if (ddr_reg_inited) {
		int table_index;
		table_index = regtable_program(rate / MHZ_TO_HZ);
		choose_regtable(table_index, ddr_reg_inited);
	} else
		choose_regtable(value, ddr_reg_inited);
	write_accr_in_sram(sram_map + 0x9000, sram_map + 0xa000 - 4, accr,
			   data, ACCR_DMCFS_MASK_978, clkmgr_base, dmc_base);

	if ((416000000 == old) && (416000000 != new))
		clk_disable(&clk_pxa978_syspll_416);
	/*
	 * DDR PLL frequency change may lead DDR to 416Mhz
	 * Need to turn it off after the change.
	 */
	if (ddr416)
		clk_disable(&clk_pxa978_syspll_416);
	if ((rate == 52000000) || (rate == 208000000) ||
	    (rate == 312000000) || (rate == 416000000))
		clk_disable(&clk_pxa978_ddr_pll);

	local_irq_restore(flags);
	local_fiq_enable();

	dvfs_notifier_frequency(&ddr_main_dvfs_freqs, DVFS_FREQ_POSTCHANGE);
	dvfs_notifier_frequency(&ddr_mem_dvfs_freqs, DVFS_FREQ_POSTCHANGE);
	init_ddr_performance_counter();

	return 0;
}

static const struct clkops clk_ddr_ops = {
	.init = common_clk_init,
	.round_rate = clk_ddr_round_rate,
	.getrate = clk_ddr_getrate,
	.setrate = clk_ddr_setrate,
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

static struct clk clk_pxa978_peri_pll = {
	.ops = &clk_pxa978_peri_pll_ops,
};

static struct clk *mmc_depend_clk[] = {
	&clk_mmc_bus,
};

static struct clk clk_pxa978_sdh0 = {
	.parent = &clk_pxa978_peri_pll,
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC1,
};

static struct clk clk_pxa978_sdh1 = {
	.parent = &clk_pxa978_peri_pll,
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC2,
};

static struct clk clk_pxa978_sdh2 = {
	.parent = &clk_pxa978_peri_pll,
	.dependence = mmc_depend_clk,
	.dependence_count = ARRAY_SIZE(mmc_depend_clk),
	.ops = &clk_pxa3xx_cken_ops,
	.enable_val = CKEN_PXA95x_MMC3,
};

static struct clk clk_pxa978_sdh3 = {
	.parent = &clk_pxa978_peri_pll,
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

static struct clk clk_pxa978_syspll_416 = {
	.ops = &clk_pxa978_syspll416_ops,
};

static struct clk clk_pxa95x_26MOUTDMD = {
	.ops = &clk_pxa95x_26MOUTDMD_ops,
};

static struct clk clk_pxa95x_26MOUT = {
	.ops = &clk_pxa95x_26MOUT_ops,
};

static struct clk clk_pxa978_ddr_pll = {
	.ops = &clk_ddr_pll_ops,
};

static struct clk clk_pxa978_ddr = {
	.ops = &clk_ddr_ops,
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
	INIT_CLKREG(&clk_pxa978_syspll_416, NULL, "SYS_PLL_416"),
	INIT_CLKREG(&clk_pxa978_peri_pll, NULL, "PERI_PLL"),
	INIT_CLKREG(&clk_pxa978_gcu, NULL, "GCCLK"),
	INIT_CLKREG(&clk_pxa95x_26MOUT, NULL, "CLK26MOUT"),
	INIT_CLKREG(&clk_pxa95x_26MOUTDMD, NULL, "CLK26MOUTDMD"),
	INIT_CLKREG(&clk_pxa978_ddr_pll, NULL, "DDRPLL"),
	INIT_CLKREG(&clk_pxa978_ddr, NULL, "DDR"),
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

static void setup_max_pp(void)
{
	/* Initialize the maximum frequencies  */
	if (cpu_is_pxa978_Dx()) {
		if (max_pp > 6)
			max_pp = 6;

		switch (max_pp) {
		case 1:
			max_core = 156;
			max_gc = 156 * MHZ_TO_HZ;
			max_vmeta = 156 * MHZ_TO_HZ;
			break;
		case 2:
			max_core = 312;
			max_gc = 156 * MHZ_TO_HZ;
			max_vmeta = 156 * MHZ_TO_HZ;
			break;
		case 3:
			max_core = 416;
			max_gc = 312 * MHZ_TO_HZ;
			max_vmeta = 312 * MHZ_TO_HZ;
			break;
		case 4:
			max_core = 728;
			max_gc = 481 * MHZ_TO_HZ;
			max_vmeta = 481 * MHZ_TO_HZ;
			break;
		case 5:
			max_core = 1196;
			max_gc = 600 * MHZ_TO_HZ;
			max_vmeta = 600 * MHZ_TO_HZ;
			break;
		case 6:
			max_core = 1404;
			max_gc = 600 * MHZ_TO_HZ;
			max_vmeta = 600 * MHZ_TO_HZ;
			break;
		default:
			printk(KERN_ERR "Not supported max pp!\n");
			break;
		}
		min_gc = 0;
	} else {
		if (max_pp > 7)
			max_pp = 7;
		switch (max_pp) {
		case 1:
			max_core = 156;
			max_gc = 156 * MHZ_TO_HZ;
			max_vmeta = 156 * MHZ_TO_HZ;
			break;
		case 2:
			max_core = 312;
			max_gc = 156 * MHZ_TO_HZ;
			max_vmeta = 156 * MHZ_TO_HZ;
			break;
		case 3:
			max_core = 624;
			max_gc = 312 * MHZ_TO_HZ;
			max_vmeta = 312 * MHZ_TO_HZ;
			break;
		case 4:
			max_core = 806;
			max_gc = 498 * MHZ_TO_HZ;
			max_vmeta = 498 * MHZ_TO_HZ;
			break;
		case 5:
			max_core = 1014;
			max_gc = 498 * MHZ_TO_HZ;
			max_vmeta = 498 * MHZ_TO_HZ;
			break;
		case 6:
			max_core = 1196;
			max_gc = 600 * MHZ_TO_HZ;
			max_vmeta = 600 * MHZ_TO_HZ;
			break;
		case 7:
			max_core = 1404;
			max_gc = 600 * MHZ_TO_HZ;
			max_vmeta = 600 * MHZ_TO_HZ;
			break;
		default:
			printk(KERN_ERR "Not supported max pp!\n");
			break;
		}
		min_gc = 0;
	}
}

int pxa95x_clk_init(void)
{
	clkmgr_base = (u32) ioremap(ACCU_PHY_BASE, 0x10000);
	sram_map = (u32) __arm_ioremap(SRAM_PHY_BASE, 0x20000,
				       MT_MEMORY_NONCACHED);
	dmc_base = (u32) ioremap(DMC_PHY_BASE, 0x1000);

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
		   | (1 << (CKEN_DSI_TX2 - 64)));

	if (!is_wkr_dma_clock())
		CKENC &= ~(1 << (CKEN_AXI_2X - 64));

	clock_lookup_init(common_clkregs, ARRAY_SIZE(common_clkregs));
	if (cpu_is_pxa978()) {
		setup_max_pp();
		if (!gc_freq_counts)
			get_gcu_freqs_table(gc_cur_freqs_table, &gc_freq_counts,
					    ARRAY_SIZE(gc_cur_freqs_table));
		clock_lookup_init(pxa978_specific_clkregs,
				  ARRAY_SIZE(pxa978_specific_clkregs));

		ddr_reg_table_init(ddr_prof_data_base_init);
		if (ddr_reg_inited)
			regtable_init();

		/* Make sure rate is always same as real setting */
		clk_pxa978_gcu.rate = clk_get_rate(&clk_pxa978_gcu);
		clk_pxa95x_vmeta.rate = clk_get_rate(&clk_pxa95x_vmeta);
		clk_pxa95x_lcd.rate = clk_get_rate(&clk_pxa95x_lcd);
		clk_pxa978_ddr.rate = clk_get_rate(&clk_pxa978_ddr);

		/* Initialize the clock of vMeta/GC to lowest */
		clk_set_rate(&clk_pxa95x_vmeta, 0);
		if (cpu_is_pxa978_Dx())
			clk_set_rate(&clk_pxa978_gcu, 312000000);
		else
			clk_set_rate(&clk_pxa978_gcu, 0);

		/* Don't use IRQ disable to protect the clock driver */
		clk_set_cansleep(&clk_pxa95x_vmeta);
		clk_set_cansleep(&clk_pxa978_gcu);
		clk_set_cansleep(&clk_pxa95x_lcd);
		clk_set_cansleep(&clk_pxa978_ddr);
	} else
		clock_lookup_init(pxa955_specific_clkregs,
				  ARRAY_SIZE(pxa955_specific_clkregs));
	return 0;
}
