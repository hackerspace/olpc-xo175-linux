/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * 	Yifan Zhang <zhangyf@marvell.com>
 *
 * (C) Copyright 2012 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <mach/mmp3_pm.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/regs-ciu.h>

#define VCXO 26000000

static u32 pd[9] = {2, 3, 4, 5, 6, 8, 10, 12, 16};
static char *ddr_inter[] = {"4k", "16k", "64k", "256k", "1024k", "512m",
	"1g", "none"};

static u32 pll_clk_calculate(u32 refdiv, u32 fbdiv, u32 postdiv)
{
	u32 input_clk;
	u32 output_clk;
	u32 M, N;

	switch (refdiv) {
	case 3:
		M = 3;
		input_clk = VCXO;
		break;
	default:
		pr_warn("The PLL REFDIV should be 0x03\n");
		return 0;
	}

	N = fbdiv;

	/* multiplied by 2 since pd is multiplied by 2 */
	output_clk = input_clk / 10 * N / M * 10 * 2 / postdiv;

	return output_clk;
}

static u32 clk_selection(u32 sel, u32 pll1_clk, u32 pll2_clk, u32 pll1p_clk)
{
	u32 sel_clk;

	switch (sel) {
	case 0:
		sel_clk = pll1_clk >> 1;
		break;
	case 1:
		sel_clk = pll1_clk;
		break;
	case 2:
		sel_clk = pll2_clk;
		break;
	case 3:
		sel_clk = pll1p_clk;
		break;
	case 4:
		sel_clk = VCXO;
		break;
	default:
		pr_warn("The clock selection is reserved\n");
		sel_clk = 0;
		break;
	}

	return sel_clk;
}

static u32 pll1_get_clk(void)
{
	u32 mpmu_fccr;
	u32 mpmu_pll1_ctrl;

	u32 refdiv;
	u32 fbdiv;
	u32 postdiv;
	int pll1_en;

	mpmu_fccr = readl(MPMU_FCCR);
	pr_debug("mpmu is 0x%x\n", mpmu_fccr);
	pll1_en   = (mpmu_fccr >> 14) & 0x1;
	mpmu_pll1_ctrl = readl(MPMU_PLL1_CTRL);

	if (unlikely(pll1_en)) {
		refdiv    = (mpmu_fccr >> 9) & 0x1f;
		fbdiv     = mpmu_fccr & 0x1ff;
		/* FIXME, offsets are wrong in spec*/
		postdiv   = (mpmu_pll1_ctrl >> 25) & 0x7;
		return pll_clk_calculate(refdiv, fbdiv, pd[postdiv]);
	} else {
		return 797330000;
	}
}

static u32 pll2_get_clk(void)
{
	u32 mpmu_pll2cr;
	u32 mpmu_pll2_ctrl1;

	u32 refdiv;
	u32 fbdiv;
	u32 postdiv;
	int pll2_en;

	mpmu_pll2cr = readl(MPMU_PLL2CR);
	mpmu_pll2_ctrl1 = readl(MPMU_PLL2_CTRL1);

	pll2_en = (mpmu_pll2cr >> 9) & 0x1;
	pr_debug("MPMU_PLL2CR is 0x%x\n", mpmu_pll2cr);
	/* Suppose no HW control */
	if (likely(pll2_en)) {
		refdiv      = (mpmu_pll2cr >> 19) & 0x1f;
		fbdiv       = (mpmu_pll2cr >> 10) & 0x1ff;
		/* FIXME, offset are wrong in spec */
		postdiv = (mpmu_pll2_ctrl1 >> 25) & 0x7;

		return pll_clk_calculate(refdiv, fbdiv, pd[postdiv]);
	} else {
		return 0;
	}
}

static u32 pll3_get_clk(void)
{
	u32 mpmu_pll3_cr;
	u32 mpmu_pll3_ctrl1;

	u32 refdiv;
	u32 fbdiv;
	u32 postdiv;
	int pll3_en;

	mpmu_pll3_cr = readl(PMUM_PLL3_CR);
	pr_debug("mpmu_pll3_cr is %d\n", mpmu_pll3_cr);
	mpmu_pll3_ctrl1 = readl(PMUM_PLL3_CTRL1);
	pll3_en = (mpmu_pll3_cr >> 8) & 0x1;

	if (pll3_en) {
		refdiv = (mpmu_pll3_cr >> 19) & 0x1f;
		fbdiv = (mpmu_pll3_cr >> 10) & 0x1ff;
		postdiv = (mpmu_pll3_ctrl1 >> 25) & 0x7;

		return pll_clk_calculate(refdiv, fbdiv, pd[postdiv]);
	} else {
		return 0;
	}
}

static u32 pll1_p_get_clk(void)
{
	u32 mpmu_fccr;
	u32 mpmu_pll1_ctrl;
	u32 pllp_ctrl;

	u32 postdiv;
	int pllp_en;

	mpmu_fccr = readl(MPMU_FCCR);
	pllp_ctrl = readl(PMUM_PLL_DIFF_CTRL);
	pr_debug("pllp_ctrl is 0x%x\n", pllp_ctrl);
	mpmu_pll1_ctrl = readl(MPMU_PLL1_CTRL);

	pllp_en = (pllp_ctrl >> 4) & 0x1;
	pr_debug("mpmu is 0x%x\n", mpmu_fccr);

	if (pllp_en) {
		/* hard code since pll1_p is made by
		 * HW fuse */
		postdiv = pllp_ctrl & 0xf;
		return 797330000 * 2 / pd[postdiv] * 2;
	} else {
		return 0;
	}
}

static u32 pll2_p_get_clk(void)
{
	u32 mpmu_pll2cr;
	u32 pllp_ctrl;

	u32 refdiv;
	u32 fbdiv;
	u32 postdiv;
	int pllp_en;

	mpmu_pll2cr = __raw_readl(MPMU_PLL2CR);
	pllp_ctrl   = readl(PMUM_PLL_DIFF_CTRL);

	pr_debug("MPMU_PLL2CR is 0x%x\n", mpmu_pll2cr);
	pllp_en    = (pllp_ctrl >> 9) & 1;

	if (pllp_en) {
		refdiv      = (mpmu_pll2cr >> 19) & 0x1f;
		fbdiv       = (mpmu_pll2cr >> 10) & 0x1ff;
		postdiv     = (pllp_ctrl >> 5) & 0xf;

		return pll_clk_calculate(refdiv, fbdiv, pd[postdiv]);
	} else {
		return 0;
	}
}

static u32 get_sdh_clk(u32 index, u32 pll1_clk, u32 pll2_clk)
{
	u32 sdh_clk;
	u32 res_clk;
	u32 tmp;
	u32 addr;

	switch (index) {
	case 0:
		addr = APMU_SDH0;
		break;
	case 1:
		addr = APMU_SDH1;
		break;
	case 2:
		addr = APMU_SDH2;
		break;
	case 3:
		addr = APMU_SDH3;
		break;
	case 4:
		addr = APMU_SDH4;
		break;
	}

	sdh_clk = readl(APMU_SDH0);

	switch ((sdh_clk >> 8) & 0x3) {
	case 0:
		res_clk = pll1_clk / 4;
		break;
	case 1:
		res_clk = pll2_clk;
		break;
	case 2:
		res_clk = pll1_clk / 2;
		break;
	case 3:
		res_clk = pll1_clk;
		break;
	}

	tmp = (readl(APMU_SDH0) >> 10) & 0xf;
	if (tmp && (readl(addr) >> 4 & 0x1))
		res_clk /= tmp;
	else
		res_clk = 0;

	return res_clk;
}

static int ddr_interleave_get(void)
{
	int i;
	u32 ddr_interleave = readl(CIU_DDR_ILV_CTRL);

	for (i = 0; i < 7; i++)
		if (ddr_interleave >> i & 1) return i;

	/* if no interleave, return overflow value */
	return 7;
}

static ssize_t mmp3_sysset_read(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 pll1;
	u32 pll2;
	u32 pll3;
	u32 pll1_p;
	u32 pll2_p;
	u32 mpmu_fccr;
	u32 mpmu_pll2cr;
	u32 mpmu_pdc;
	u32 apmu_cc_pj;
	u32 apmu_cc2_pj;
	u32 apmu_cc3_pj;
	u32 apmu_gc;
	u32 apmu_vmeta;
	u32 apmu_isp;
	u32 apmu_ccic2;
	u32 apmu_ccic;
	u32 pll_sel_status;
	u32 ddr1_clk;
	u32 ddr2_clk;
	u32 axi1_clk;
	u32 axi2_clk;
	u32 gc2000_clk;
	u32 gc300_clk;
	u32 gc_bus;
	u32 vmeta_clk;
	u32 vmeta_bus_clk;
	u32 isp_clk;
	u32 mp1_clk;
	u32 mp2_clk;
	u32 mm_clk;
	u32 ccic_clk;
	u32 ccic2_clk;
	u32 sdh_clk[5];

	int i;
	int tmp;
	int len;
	int inter;

	pll1 = pll1_get_clk();
	pll2 = pll2_get_clk();
	pll3 = pll3_get_clk();
	pll1_p = pll1_p_get_clk();
	pll2_p = pll2_p_get_clk();
	mpmu_fccr = readl(MPMU_FCCR);
	mpmu_pdc = readl(PMUM_PLL_DIFF_CTRL);
	mpmu_pll2cr = readl(MPMU_PLL2CR);
	apmu_cc_pj = readl(APMU_CC_PJ);
	apmu_cc2_pj = readl(APMU_CC2_PJ);
	apmu_cc3_pj = readl(APMU_CC3_PJ);
	pll_sel_status = readl(APMU_PLL_SEL_STATUS);

	/* mp1 */
	tmp = (pll_sel_status >> 3) & 0x7;
	mp1_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = apmu_cc_pj & 0x7;
	mp1_clk /= tmp + 1;
	tmp = (apmu_cc2_pj >> 9) & 0xf;
	mp1_clk /= tmp + 1;

	/* mp2 */
	tmp = (pll_sel_status >> 3) & 0x7;
	mp2_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = apmu_cc_pj & 0x7;
	mp2_clk /= tmp + 1;
	tmp = (apmu_cc2_pj >> 13) & 0xf;
	mp2_clk /= tmp + 1;

	/* mm */
	tmp = (pll_sel_status >> 3) & 0x7;
	mm_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = apmu_cc_pj & 0x7;
	mm_clk /= tmp + 1;
	tmp = (apmu_cc2_pj >> 17) & 0xf;
	mm_clk /= tmp + 1;

	/* ddr1 */
	tmp = (pll_sel_status >> 6) & 0x7;
	ddr1_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = (apmu_cc_pj >> 12) & 0x7;
	ddr1_clk /= tmp + 1;
	/* it is 2x value in spec */
	ddr1_clk /= 2;

	/* ddr2 */
	tmp = (pll_sel_status >> 6) & 0x7;
	ddr2_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = (apmu_cc3_pj >> 17) & 0x7;
	ddr2_clk /= tmp + 1;
	/* it is 2x value in spec */
	ddr2_clk /= 2;

	/* axi1 */
	tmp = (pll_sel_status >> 9) & 0x7;
	axi1_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = (apmu_cc_pj >> 15) & 0x7;
	axi1_clk /= tmp + 1;

	/* axi2 */
	tmp = (pll_sel_status >> 9) & 0x7;
	axi2_clk = clk_selection(tmp, pll1, pll2, pll1_p);
	tmp = apmu_cc2_pj & 0x7;
	axi2_clk /= tmp + 1;

	/* GC2000, GC300, GC bus */
	gc2000_clk = 0;
	gc300_clk = 0;
	gc_bus = 0;

	apmu_gc = readl(APMU_GC_CLK_RES_CTRL);
	pr_debug("apmu_gc is %d\n", apmu_gc);
	if (((apmu_gc >> 9) & 0x3) == 0x3) {
		tmp = (apmu_gc >> 6) & 0x3;
		switch (tmp) {
		case 0:
			gc2000_clk = pll1;
			break;
		case 1:
			gc2000_clk = pll2;
			break;
		case 2:
			gc2000_clk = pll1_p;
			break;
		case 3:
			gc2000_clk = pll2_p;
			break;
		default:
			break;
		}

		tmp = (apmu_gc >> 24) & 0xf;
		if (tmp)
			gc2000_clk /= tmp;
		else
			gc2000_clk = 0;

		/* gc300 clk */
		tmp = (apmu_gc >> 12) & 0x3;
		switch (tmp) {
		case 0:
			gc300_clk = pll1;
			break;
		case 1:
			gc300_clk = pll2;
			break;
		case 2:
			gc300_clk = pll1_p;
			break;
		case 3:
			gc300_clk = pll2_p;
			break;
		default:
			break;
		}

		tmp = (apmu_gc >> 28) & 0xf;
		if (tmp)
			gc300_clk /= tmp;
		else
			gc300_clk = 0;

		tmp = (apmu_gc >> 4) & 0x2;
		switch (tmp) {
		case 0:
			gc_bus = pll1 / 4;
			break;
		case 1:
			gc_bus = pll1 / 6;
			break;
		case 2:
			gc_bus = pll1 / 2;
			break;
		case 3:
			gc_bus = pll2 / 2;
			break;
		default:
			break;
		}

	} else {
		gc2000_clk = 0;
		gc300_clk = 0;
		gc_bus = 0;
	}

	/* vMeta Tech */
	vmeta_clk = 0;
	vmeta_bus_clk = 0;
	apmu_vmeta = readl(APMU_VMETA);
	if (((apmu_vmeta >> 9) & 0x3) == 3) {
		tmp = (apmu_vmeta >> 6) & 0x3;
		switch (tmp) {
		case 0:
			vmeta_clk = pll1;
			break;
		case 1:
			vmeta_clk = pll2;
			break;
		case 2:
			vmeta_clk = pll1_p;
			break;
		case 3:
			vmeta_clk = pll2_p;
			break;
		default:
			break;
		}

		tmp = (apmu_vmeta >> 16) & 0xf;
		if (tmp)
			vmeta_clk /= tmp;
		else
			vmeta_clk = 0;

		tmp = (apmu_vmeta >> 11) & 0x3;
		switch (tmp) {
		case 0:
			vmeta_bus_clk = pll1 / 4;
			break;
		case 1:
			vmeta_bus_clk = pll1 / 6;
			break;
		case 2:
			vmeta_bus_clk = pll1 / 2;
			break;
		case 3:
			vmeta_bus_clk = pll2 / 2;
			break;
		default:
			break;
		}
	} else {
		vmeta_clk = 0;
		vmeta_bus_clk = 0;
	}

	/* ISP */
	apmu_isp = readl(APMU_ISPCLK);
	pr_debug("apmu_isp is 0x%x\n", apmu_isp);
	if (((readl(APMU_ISPPWR) >> 9) & 0x3) == 3) {
		tmp = (apmu_isp >> 6) & 0x3;
		switch (tmp) {
		case 0:
			isp_clk = pll1 / 2;
			break;
		case 1:
			isp_clk = pll1;
			break;
		case 2:
			isp_clk = pll2;
			break;
		case 3:
			isp_clk = VCXO;
			break;
		default:
			break;
		}

		tmp = (apmu_isp >> 8) & 0xf;
		if (tmp)
			isp_clk /= tmp;
		else
			isp_clk = 0;
	} else {
		isp_clk = 0;
	}

	/* SDH */
	memset(sdh_clk, 0, 5 * sizeof(u32));
	for (i = 0; i < 5; i++)
		sdh_clk[i] = get_sdh_clk(i, pll1, pll2);

	/* CCIC */
	apmu_ccic = readl(APMU_CCIC_RST);
	pr_debug("apmu_ccic is %x\n", apmu_ccic);
	if ((apmu_ccic >> 3) & (apmu_ccic >> 4)) {
		tmp = (apmu_ccic >> 6) & 0x3;
		switch (tmp) {
		case 0:
			ccic_clk = pll1 / 2;
			break;
		case 1:
			ccic_clk = pll1 / 16;
			break;
		case 2:
			ccic_clk = pll2;
			break;
		case 3:
			ccic_clk = VCXO;
			break;
		default:
			break;
		}

		tmp = (apmu_ccic >> 17) & 0xf;
		if (tmp)
			ccic_clk /= tmp;
		else
			ccic_clk = 0;
	} else {
		ccic_clk = 0;
	}

	/* CCIC2 */
	apmu_ccic2 = readl(APMU_CCIC2_RST);
	pr_debug("apmu_ccic2 is %x\n", apmu_ccic2);
	if ((apmu_ccic2 >> 3) & (apmu_ccic2 >> 4)) {
		tmp = (apmu_ccic2 >> 6) & 0x3;
		switch (tmp) {
		case 0:
			ccic2_clk = pll1 / 2;
			break;
		case 1:
			ccic2_clk = pll1 / 16;
			break;
		case 2:
			ccic2_clk = pll2;
			break;
		case 3:
			ccic2_clk = VCXO;
			break;
		default:
			break;
		}

		tmp = (apmu_ccic2 >> 16) & 0xf;
		if (tmp)
			ccic2_clk /= tmp;
		else
			ccic2_clk = 0;
	} else {
		ccic2_clk = 0;
	}

	/* ddr interleave */
	inter = ddr_interleave_get();

	len = sprintf(buf, "\nPLL1[%d], PLL2[%d], PLL3[%d], PLL1_P[%d], "
			"PLL2_P[%d]\n\nMP1[%d], MP2[%d], MM[%d]\n\nDDR1[%d], "
			"DDR2[%d]\n\nAXI1[%d], AXI2[%d]\n\nGC2000[%d], "
			"GC300[%d], GC_BUS[%d]\n\nVMETA[%d], VMETA_BUS[%d]\n\n"
			"ISP[%d]\n\nSDH1[%d], SDH2[%d], SDH3[%d], SDH4[%d], "
			"SDH5[%d]\n\nCCIC[%d], CCIC2[%d]\n\nDDR_INTERLEAVE[%s]\n\n",
			pll1, pll2, pll3, pll1_p, pll2_p,
			mp1_clk, mp2_clk, mm_clk, ddr1_clk, ddr2_clk,
			axi1_clk, axi2_clk, gc2000_clk, gc300_clk, gc_bus,
			vmeta_clk, vmeta_bus_clk, isp_clk,
			sdh_clk[0], sdh_clk[1], sdh_clk[2], sdh_clk[3],
			sdh_clk[4], ccic_clk, ccic2_clk, ddr_inter[inter]);

	return len;
}

static DEVICE_ATTR(mmp3_sysset, 0444, mmp3_sysset_read, NULL);

static struct attribute *mmp3_sysset_sysfs_entries[] = {
	&dev_attr_mmp3_sysset.attr,
	NULL
};

static struct attribute_group mmp3_sysset_attr_group = {
	.name   = NULL,
	.attrs  = mmp3_sysset_sysfs_entries,
};

static int __devinit mmp3_sysset_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_INFO "mmp3 system setting module was loaded\n");

	ret = sysfs_create_group(&pdev->dev.kobj, &mmp3_sysset_attr_group);
	if (ret)
		printk(KERN_ALERT "Error: unable to create sysfs for mmp3 system setting\n");

	return ret;
}
static int __devexit mmp3_sysset_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &mmp3_sysset_attr_group);
	return 0;
}

static struct platform_driver mmp3_sysset_driver = {
	.driver		= {
		.name	= "mmp3-sysset",
		.owner	= THIS_MODULE,
	},
	.probe		= mmp3_sysset_probe,
	.remove		= mmp3_sysset_remove,
	.suspend	= NULL,
	.resume		= NULL,
};

static struct platform_device mmp3_sysset_device = {
	.name		= "mmp3-sysset",
	.id		= -1,
};

static int __devinit mmp3_sysset_init(void)
{
	int ret = platform_device_register(&mmp3_sysset_device);
	if (ret)
		return ret;
	return platform_driver_register(&mmp3_sysset_driver);
}

static void mmp3_sysset_exit(void)
{
	platform_device_unregister(&mmp3_sysset_device);
	platform_driver_unregister(&mmp3_sysset_driver);
}

module_init(mmp3_sysset_init);
module_exit(mmp3_sysset_exit);

MODULE_DESCRIPTION("Get system setting for mmp3");
MODULE_LICENSE("GPL");
