/*
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2010 Marvell International Ltd.
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


static int ddr_interleave_get(void)
{
	int i;
	u32 ddr_interleave = __raw_readl(CIU_DDR_ILV_CTRL);

	for (i = 0; i < 7; i++)
		if (ddr_interleave >> i & 1) return i;

	/* if no interleave, return overflow value */
	return 7;
}

static ssize_t mmp3_sysset_read(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	u32 pj4b1_core_clk = mmp3_getfreq(MMP3_CLK_MP1);
	u32 pj4b2_core_clk = mmp3_getfreq(MMP3_CLK_MP2);
	u32 pj4bmm_core_clk = mmp3_getfreq(MMP3_CLK_MM);
	u32 ddr1_clk = mmp3_getfreq(MMP3_CLK_DDR_1);
	u32 ddr2_clk = mmp3_getfreq(MMP3_CLK_DDR_2);
	u32 axi1_clk = mmp3_getfreq(MMP3_CLK_AXI_1);
	u32 axi2_clk = mmp3_getfreq(MMP3_CLK_AXI_2);
	u32 pll1 = clk_get_rate(clk_get(NULL, "pll1"));
	u32 pll2 = clk_get_rate(clk_get(NULL, "pll2"));
	u32 pll1_clkoutp = clk_get_rate(clk_get(NULL, "pll1_clkoutp"));
	u32 vctcxo = clk_get_rate(clk_get(NULL, "vctcxo"));
	u32 gc = clk_get_rate(clk_get(NULL, "GCCLK"));
	u32 vmeta = clk_get_rate(clk_get(NULL, "VMETA_CLK"));

	int inter = ddr_interleave_get();
	char *ddr_inter[] = {"4k", "16k", "64k", "256k", "1024k", "512m", "1g", "none"};

	ssize_t len = sprintf(buf, "PJ4B1_Core[%d], PJ4B2_Core[%d], PJ4BMM_Core[%d], "
			"DDR1[%d], DDR2[%d], AXI1[%d], AXI2[%d], PLL1[%d], PLL2[%d], "
			"PLL1_CLKOUTP[%d]\nVCTCXO[%d], GC[%d], VMETA[%d], "
			"ddr_interleave[%s]\n",
			pj4b1_core_clk, pj4b2_core_clk, pj4bmm_core_clk, ddr1_clk,
			ddr2_clk, axi1_clk, axi2_clk, pll1, pll2, pll1_clkoutp,
			vctcxo, gc, vmeta, ddr_inter[inter]);

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
