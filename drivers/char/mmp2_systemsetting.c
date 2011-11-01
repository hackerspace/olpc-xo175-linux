/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <mach/mmp2_pm.h>
#include <mach/regs-mpmu.h>
#include <mach/regs-apmu.h>
#include <mach/cputype.h>
#include <mach/addr-map.h>
#include <asm/io.h>

#define CPU_CONF        (AXI_VIRT_BASE + 0x82c08)
static unsigned long pll_clk_calculate(u32 refdiv, u32 fbdiv)
{
	u32 input_clk;
	u32 output_clk;
	u32 M, N;

	switch (refdiv) {
	case 3:
		M = 5;
		input_clk = 19200000;
		break;
	case 4:
		M = 6;
		input_clk = 26000000;
		break;
	default:
		printk(KERN_WARNING "The PLL REFDIV should be 0x03 or 0x04\n");
		return 0;
	}

	N = fbdiv + 2;
	output_clk = input_clk /10 * N / M *10;
	return output_clk;
}

static unsigned long get_pll1_clk(void)
{
	u32 mpmu_fccr;
	u32 refdiv;
	u32 fbdiv;
	u32 pll1_en;

	mpmu_fccr = __raw_readl(MPMU_FCCR);
	pll1_en = (mpmu_fccr >> 14) & 0x1;
	if (pll1_en) {
		refdiv = (mpmu_fccr >> 9) & 0x1f;
		fbdiv = mpmu_fccr & 0x1ff;
		return pll_clk_calculate(refdiv, fbdiv);
	} else {
		return 797330000;
	}
}

static unsigned long get_pll2_clk(void)
{
	u32 mpmu_pll2cr;
	u32 refdiv;
	u32 fbdiv;

	mpmu_pll2cr = __raw_readl(MPMU_PLL2CR);
	refdiv = (mpmu_pll2cr >> 19) & 0x1f;
	fbdiv = (mpmu_pll2cr >> 10) & 0x1ff;
	return pll_clk_calculate(refdiv, fbdiv);
}

static unsigned long clk_selection(u32 sel, u32 pll1_clk, u32 pll2_clk)
{
	u32 sel_clk;
	switch (sel) {
	case 0x0: /* PLL1/2 */
		sel_clk = pll1_clk >> 1;
		break;
	case 0x1: /* PLL1 */
		sel_clk = pll1_clk;
		break;
	case 0x2: /* PLL2 */
		sel_clk = pll2_clk;
		break;
	case 0x3: /* VCXO */
		sel_clk = 26000000;
		break;
	default:
		printk(KERN_WARNING "The clock selection is reserved\n");
		sel_clk = 0;
		break;

	}
	return sel_clk;
}

static ssize_t mmp2_sysset_read(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 mpmu_fccr;
	u32 pll_set_status;
	u32 pll1clk;
	u32 pll2clk;
	u32 apmu_dm_cc_pj;
	u32 apmu_cc_sp;
	u32 apmu_cc_pj;
	u32 pj4_core_clk;
	u32 secure_core_clk;
	u32 ddr_clk;
	u32 axi_clk;
	u32 apmu_vmeta;
	u32 vmeta_clk = 0;
	u32 vmeta_bus_clk = 0;
	u32 tmp;
	u32 apmu_gc;
	u32 gc_clk;
	u32 gc_bus_clk;

	size_t len = 0;

	/*
	 * CPU core, ddr, axi, vmeta and gc frequency
	 */
	/* Get PLL1 and PLL2 output clock */
	pll1clk = get_pll1_clk();
	pll2clk = get_pll2_clk();
	pll_set_status = readl(APMU_PLL_SEL_STATUS);
	mpmu_fccr = readl(MPMU_FCCR);
	apmu_dm_cc_pj = readl(APMU_DM_CC_PJ);

	/* PJ4 core */
	tmp = (pll_set_status >> 2) & 0x3;
	pj4_core_clk = clk_selection(tmp, pll1clk, pll2clk);
	tmp = (apmu_dm_cc_pj & 0x7);
	pj4_core_clk /= (tmp + 1);

	/* DDR */
	tmp = (pll_set_status >> 4) & 0x3;
	ddr_clk = clk_selection(tmp, pll1clk, pll2clk);
	tmp = (apmu_dm_cc_pj >> 12) & 0x7;
	ddr_clk /= (tmp + 1);

	/* AXI */
	tmp = (pll_set_status >> 6) & 0x3;
	axi_clk = clk_selection(tmp, pll1clk, pll2clk);
	tmp = (apmu_dm_cc_pj >> 15) & 0x7;
	axi_clk /= (tmp + 1);

	/* Secure processor core */
	tmp = (mpmu_fccr >> 26) & 0x7;
	secure_core_clk = clk_selection(tmp, pll1clk, pll2clk);
	tmp = readl(APMU_DM_CC_SP);
	tmp &= 0x7;
	secure_core_clk /= (tmp + 1);

	/* VMETA */
	apmu_vmeta = readl(APMU_VMETA);
	if ((apmu_vmeta >> 10) & 0x1) {
		tmp = (apmu_vmeta >> 5) & 0x7;
		switch (tmp) {
			case 0: /* PLL1/2 */
				vmeta_clk = pll1clk/2;
				break;
			case 1: /* PLL2/3 */
				vmeta_clk = pll2clk/3;
				break;
			case 2: /* PLL2 */
				vmeta_clk = pll2clk;
				break;
			case 3: /* PLL2/2 */
				vmeta_clk = pll2clk/2;
				break;
			case 4: /* PLL2/4 */
				vmeta_clk = pll2clk/4;
				break;
			case 5: /* USB PLL */
				vmeta_clk = 480000000;
				break;
			default:
				break;
		}

		tmp = (apmu_vmeta >> 12) & 0x7;
		switch (tmp) {
			case 0: /* PLL1/4 */
				vmeta_bus_clk = pll1clk/4;
				break;
			case 1: /* PLL1/3 */
				vmeta_bus_clk = pll1clk/3;
				break;
			case 2: /* PLL2/3 */
				vmeta_bus_clk = pll2clk/3;
				break;
			case 3: /* PLL2 */
				vmeta_bus_clk = pll2clk;
				break;
			case 4: /* PLL2/2 */
				vmeta_bus_clk = pll2clk/2;
				break;
			case 5: /* PLL1/2 */
				vmeta_bus_clk = pll1clk/2;
				break;
			case 6: /* USB PLL */
				vmeta_bus_clk = 480000000;
				break;
			default:
				break;
		}
	} else {
		printk(KERN_INFO "VMeta Technology module power down/power off\n");
		vmeta_clk = vmeta_bus_clk = 0;
	}

	/* GC800 */
	apmu_gc = readl(APMU_GC);
	if (((apmu_gc >> 9) & 0x3) == 0x3) {
		gc_clk = gc_bus_clk = 0;
		if((apmu_gc >> 12) & 0x1){
			tmp = (apmu_gc >> 6) & 0x3;
			switch (tmp) {
				case 0: /* PLL2/4 */
					gc_clk = pll2clk/4;
					break;
				case 1: /* USB PLL */
					gc_clk = 480000000;
					break;
				default:
					break;
			}

			tmp = (apmu_gc >> 4) & 0x3;
			switch (tmp) {
				case 0: /* PLL2/2 */
					gc_bus_clk = pll2clk/2;
					break;
				case 1: /* PLL1/2 */
					gc_bus_clk = pll1clk/2;
					break;
				case 2: /* USB PLL */
					gc_bus_clk = 480000000;
					break;
				default:
					break;
			}
		}else{
			tmp = (apmu_gc >> 6) & 0x3;
			switch (tmp) {
				case 0: /* PLL1/2 */
					gc_clk = pll1clk/2;
					break;
				case 1: /* PLL2/3 */
					gc_clk = pll2clk/3;
					break;
				case 2: /* PLL2 */
					gc_clk = pll2clk;
					break;
				case 3: /* PLL2/2 */
					gc_clk = pll2clk/2;
					break;
				default:
					break;
			}

			tmp = (apmu_gc >> 4) & 0x3;
			switch (tmp) {
				case 0: /* PLL1/4 */
					gc_bus_clk = pll1clk/4;
					break;
				case 1: /* PLL1/3 */
					gc_bus_clk = pll1clk/3;
					break;
				case 2: /* PLL2/3 */
					gc_bus_clk = pll2clk/3;
					break;
				case 3: /* PLL2 */
					gc_bus_clk = pll2clk;
					break;
				default:
					break;
			}
		}
	} else {
		printk(KERN_INFO "2D/3D Graphics Controller module power down/power off\n");
		gc_clk = 0;
		gc_bus_clk = 0;
	}
	/*it only supports MMP2 A2 on kernel3.0 now*/
	tmp = readl(CPU_CONF) & 0x2;
	if (tmp == 0x2)
		printk("MMP2 A2 ARM v7 mode\n");
	else
		printk("MMP2 A2 ARM v6 mode\n");

	tmp = readl(APMU_CC_SP);
	len = sprintf(buf, "PJ4 Core[%d], Secure Core[%d], "
			"DDR[%d], AXI[%d], Vmeta[%d], Vmeta Bus[%d], "
			"GC[%d], GC BUS[%d]\n", pj4_core_clk, secure_core_clk,
			ddr_clk/2, axi_clk, vmeta_clk, vmeta_bus_clk, gc_clk, gc_bus_clk);

	tmp = readl(APMU_CC_SP);
	apmu_cc_sp = (tmp | PMUA_CC_SEA_SEA_RD_ST_CLEAR);
	writel(apmu_cc_sp, APMU_CC_SP);

	tmp = readl(APMU_CC_PJ);
	apmu_cc_pj = (tmp | PMUA_CC_MOH_MOH_RD_ST_CLEAR);
	writel(apmu_cc_pj, APMU_CC_PJ);

	return len;
}

static DEVICE_ATTR(mmp2_sysset, 0444, mmp2_sysset_read, NULL);

static struct attribute *mmp2_sysset_sysfs_entries[] = {
	&dev_attr_mmp2_sysset.attr,
	NULL
};

static struct attribute_group mmp2_sysset_attr_group = {
	.name   = NULL,
	.attrs  = mmp2_sysset_sysfs_entries,
};

static int __devinit mmp2_sysset_probe(struct platform_device *pdev)
{
	int ret;
	printk(KERN_INFO "MMP2 system setting module was loaded\n");

	ret = sysfs_create_group(&pdev->dev.kobj, &mmp2_sysset_attr_group);
	if (ret)
		printk(KERN_ALERT "Error: unable to create sysfs for mmp2 system setting\n");

	return ret;
}
static int __devexit mmp2_sysset_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &mmp2_sysset_attr_group);
	return 0;
}

static struct platform_driver mmp2_sysset_driver = {
	.driver		= {
		.name	= "mmp2-sysset",
		.owner	= THIS_MODULE,
	},
	.probe		= mmp2_sysset_probe,
	.remove		= mmp2_sysset_remove,
	.suspend	= NULL,
	.resume		= NULL,
};

static struct platform_device mmp2_sysset_device = {
	.name		= "mmp2-sysset",
	.id		= -1,
};

static int __devinit mmp2_sysset_init(void)
{
	int ret = platform_device_register(&mmp2_sysset_device);
	if (ret)
		return ret;
	return platform_driver_register(&mmp2_sysset_driver);
}

static void mmp2_sysset_exit(void)
{
	platform_device_unregister(&mmp2_sysset_device);
	platform_driver_unregister(&mmp2_sysset_driver);
}

module_init(mmp2_sysset_init);
module_exit(mmp2_sysset_exit);

MODULE_DESCRIPTION("Get system setting for MMP2");
MODULE_LICENSE("GPL");
