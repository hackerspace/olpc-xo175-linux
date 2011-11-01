/*
 * MMP2 Fuse Management Routines
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html
 *
 * (C) Copyright 2011 Marvell International Ltd.
 * All Rights Reserved
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/sysdev.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <asm/setup.h>
#include <mach/regs-fuse.h>
#include <mach/regs-apmu.h>
#include <mach/mmp2_fuse.h>
#include <mach/cputype.h>

#define MMP2_FUSE_PROC_ENTRY		"mmp2_fuse"
#define MMP2_PROFILE_PROC_ENTRY		"mmp2_profile"

struct mmp2_fuse {
	char __iomem	*reg_base;
	unsigned long	reg_size;

	struct clk	*clk;

	struct proc_dir_entry *fuse_entry;
	struct proc_dir_entry *profile_entry;
};

static struct mmp2_fuse *fuse;

static unsigned int mmp2_has_init_tag;
static unsigned int mmp2_profile_adjust;
static unsigned int mmp2_profile;
static unsigned int mmp2_max_freq;
static unsigned int mmp2_ts_calibration;

static int __init parse_tag_profile(const struct tag *tag)
{
	/*
	 * if uboot can not get ack from wtm,
	 * the kernel will later try to read the
	 * fuse value, otherwise use the uboot
	 * passed data.
	 */
	if (!tag->u.profile.ack_from_wtm) {
		mmp2_has_init_tag = 0;
		return 0;
	}

	printk(KERN_INFO "%s: use uboot's fuse value\n", __func__);

	mmp2_profile_adjust = tag->u.profile.profile_adjust;
	mmp2_profile        = tag->u.profile.profile;
	mmp2_max_freq	    = tag->u.profile.max_freq;
	mmp2_ts_calibration = tag->u.profile.ts_calibration;

	mmp2_has_init_tag = 1;
	return 0;
}

__tagtable(ATAG_PROFILE, parse_tag_profile);

static void show_register_group(struct seq_file *s, char *str,
				int group, int num)
{
	int i, offset;
	char buf[32];

	seq_printf(s, "------  --------------  ----------\n");

	for (i = 0; i < num; i++) {
		offset = group + (i * 4);
		sprintf(buf, "%s-%02d", str, i + 1);
		seq_printf(s, "0x%04x  %s  0x%08x\n", offset, buf,
			   readl(fuse->reg_base + offset));
	}

	return;
}

static int mmp2_fuse_proc_show(struct seq_file *s, void *v)
{
	seq_printf(s, "offset  description     value\n");

	show_register_group(s, "fuse block0", MMP2_FUSE_BLK0_CFG1, 8);
	show_register_group(s, "fuse block3", MMP2_FUSE_BLK3_CFG1, 8);
	show_register_group(s, "fuse block8", MMP2_FUSE_BLK8_CFG1, 8);
	show_register_group(s, "fuse block9", MMP2_FUSE_BLK9_CFG1, 4);

	return 0;
}

static int mmp2_fuse_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmp2_fuse_proc_show, NULL);
}

static const struct file_operations mmp2_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= mmp2_fuse_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

unsigned int mmp2_read_profile(void)
{
	unsigned int i, temp = 0;
	unsigned int fuse_val;
	unsigned int profile = 0, adjust;

	if (mmp2_has_init_tag) {
		fuse_val = mmp2_profile;
		adjust   = mmp2_profile_adjust;
	} else {
		fuse_val = readl(fuse->reg_base + MMP2_FUSE_BLK3_CFG7);
		adjust   = (fuse_val >> 16) & MMP2_PROFILE_ADJUST_MASK;
	}

	for (i = 0; i < MMP2_PROFILE_NUM; i++) {
		temp |= MMP2_PROFILE_SHIFT_VAL << (i * 2);
		if (temp == (fuse_val & 0xffff)) {
			profile = MMP2_PROFILE_NUM - i;
			break;
		}
	}

	/*
	 * bits[210..208] are for the voltage adjustment,
	 * b000: do not change profile,
	 * b011: increase 1 to profile,
	 * b111: decrease 1 to profile.
	 */
	switch (adjust) {
	case MMP2_PROFILE_ADJUST_INC_1:
		if (profile != MMP2_PROFILE_NUM)
			profile += 1;
		break;
	case MMP2_PROFILE_ADJUST_DEC_1:
		if (profile != 0)
			profile -= 1;
		break;
	default:
		break;
	}

	return profile;
}
EXPORT_SYMBOL(mmp2_read_profile);

unsigned int mmp2_read_max_freq(void)
{
	unsigned int max_freq;
	unsigned int fuse_val;

	if (mmp2_has_init_tag)
		max_freq = mmp2_max_freq;
	else {
		fuse_val = readl(fuse->reg_base + MMP2_FUSE_BLK3_CFG8);
		max_freq = fuse_val >> MMP2_MAX_FREQ_SHIFT;
		max_freq = max_freq & MMP2_MAX_FREQ_MASK;
	}

	if (max_freq != MMP2_MAX_FREQ_800MHZ &&
	    max_freq != MMP2_MAX_FREQ_988MHZ) {
		printk(KERN_ERR "%s: get the invalid max freq value %d\n",
		       __func__, max_freq);
		return MMP2_MAX_FREQ_800MHZ;
	}

	return max_freq;

}
EXPORT_SYMBOL(mmp2_read_max_freq);

static ssize_t mmp2_profile_read_proc(char *page, char **start, off_t off,
				      int count, int *eof, void *data)
{
	int len;

	len  = sprintf(page, "profile  = 0x%x\n", mmp2_read_profile());
	len += sprintf(page + len, "max freq = 0x%x\n", mmp2_read_max_freq());
	return len;
}

static int mmp2_profile_table[MMP2_PROFILE_NUM + 1][MMP2_PRODUCT_POINT_NUM] = {
	{1325, 1325, 1325, 1325, 1425},		/* p0 */
	{1325, 1325, 1325, 1325, 1425},		/* p1 */
	{1300, 1300, 1300, 1300, 1425},		/* p2 */
	{1275, 1275, 1275, 1275, 1425},		/* p3 */
	{1250, 1250, 1250, 1275, 1425},		/* p4 */
	{1250, 1250, 1250, 1250, 1400},		/* p5 */
	{1225, 1225, 1225, 1225, 1375},		/* p6 */
	{1200, 1200, 1200, 1200, 1350},		/* p7 */
	{1175, 1175, 1175, 1175, 1325}		/* p8 */
};

static const int mmp2_dummy_profile_table[MMP2_PRODUCT_POINT_NUM] = {
	1230, 1230, 1280, 1300, 1350
};

int mmp2_get_voltage(unsigned int profile, unsigned int product_point)
{
	if (profile > MMP2_PROFILE_NUM)
		return -1;
	if (product_point >= MMP2_PRODUCT_POINT_NUM)
		return -1;

	/* use the dummy table for the old chips
	 * and from A2 takeout, use the profile to
	 * set the voltage value
	 */
	if (cpu_is_mmp2)
		return mmp2_profile_table[profile][product_point];
	else
		/* return a safe value anyway */
		return mmp2_dummy_profile_table[product_point];
}
EXPORT_SYMBOL(mmp2_get_voltage);

static int tbl_set(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t count)
{
	int i = 0;
	char *token, *s;
	unsigned int argv[3];
	char str[128];

	if (count > sizeof(str) - 1)
		return -EINVAL;

	memcpy(str, buf, count);
	s = str;

	while ((token = strsep(&s, " \t\n")) != NULL) {
		if (!*token)
			continue;
		/*
		 * argv[0] is profile, argv[1] is ppt,
		 * and argv[2] is voltage value.
		 */
		argv[i++] = simple_strtol(token, NULL, 0);

		if (i >= ARRAY_SIZE(argv))
			break;
	}

	if (i != ARRAY_SIZE(argv)) {
		printk(KERN_ERR "usage: echo profile ppt vol > vtbl\n");
		return -EINVAL;
	}

	if (argv[0] > MMP2_PROFILE_NUM) {
		printk(KERN_ERR "profile is out scope (0 ~ %d)\n",
			MMP2_PROFILE_NUM);
		return -EINVAL;
	}
	if (argv[1] >= MMP2_PRODUCT_POINT_NUM) {
		printk(KERN_ERR "ppt is out scope (0 ~ %d)\n",
			MMP2_PRODUCT_POINT_NUM - 1);
		return -EINVAL;
	}

	mmp2_profile_table[argv[0]][argv[1]] = argv[2];

	return count;
}

static int tbl_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	int i, j, len;

	len = sprintf(buf, "profile  ppt0  ppt1  ppt2  ppt3  ppt4\n");
	for (i = 0; i <= MMP2_PROFILE_NUM; i++) {
		len += sprintf(buf + len, "   p%d    ", i);

		for (j = 0; j < MMP2_PRODUCT_POINT_NUM; j++)
			len += sprintf(buf + len, "%d  ",
					mmp2_profile_table[i][j]);

		len += sprintf(buf + len, "\n");
	}

	return len;
}

static DEVICE_ATTR(vtbl, S_IRUGO | S_IWUSR, tbl_show, tbl_set);

static struct attribute *mmp2_fuse_attributes[] = {
	&dev_attr_vtbl.attr,
	NULL,
};

static struct attribute_group mmp2_fuse_attribute_group = {
	.attrs = mmp2_fuse_attributes,
};

static int mmp2_fuse_probe(struct platform_device *dev)
{
	struct resource  *res;
	int ret;

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL)
		return -ENODEV;

	fuse = kzalloc(sizeof(struct mmp2_fuse), GFP_KERNEL);
	if (!fuse) {
		ret = -ENOMEM;
		goto exit;
	}

	fuse->reg_base = ioremap(res->start, resource_size(res));
	if (!fuse->reg_base) {
		ret = -EIO;
		goto clean_priv_data;
	}
	fuse->reg_size = resource_size(res);

	fuse->clk = clk_get(NULL, "mmp2-wtm");
	if (IS_ERR(fuse->clk)) {
		ret = PTR_ERR(fuse->clk);
		goto clean_iomap;
	}
	clk_enable(fuse->clk);

	fuse->fuse_entry = create_proc_entry(MMP2_FUSE_PROC_ENTRY, 0644, NULL);
	if (!fuse->fuse_entry) {
		ret = -ENOMEM;
		goto clean_clk;
	}
	fuse->fuse_entry->proc_fops = &mmp2_proc_fops;

	fuse->profile_entry = 
		create_proc_entry(MMP2_PROFILE_PROC_ENTRY, 0644, NULL);
	if (!fuse->profile_entry) {
		ret = -ENOMEM;
		goto clean_fuse_proc;
	}
	fuse->profile_entry->read_proc = &mmp2_profile_read_proc;

	platform_set_drvdata(dev, fuse);

	ret = sysfs_create_group(&(dev->dev.kobj), &mmp2_fuse_attribute_group);
	if (ret)
		goto clean_fuse_proc;

	printk(KERN_INFO "%s: probe mmp2 fuse device\n", dev_name(&(dev->dev)));

	return 0;

clean_fuse_proc:
	remove_proc_entry(MMP2_FUSE_PROC_ENTRY, NULL);
clean_clk:
	clk_disable(fuse->clk);
	clk_put(fuse->clk);
clean_iomap:
	iounmap(fuse->reg_base);
clean_priv_data:
	kfree(fuse);
exit:
	return ret;
}

static int mmp2_fuse_remove(struct platform_device *dev)
{
	struct mmp2_fuse *fuse = platform_get_drvdata(dev);

	sysfs_remove_group(&(dev->dev.kobj), &mmp2_fuse_attribute_group);
	platform_set_drvdata(dev, NULL);
	remove_proc_entry(MMP2_PROFILE_PROC_ENTRY, NULL);
	remove_proc_entry(MMP2_FUSE_PROC_ENTRY, NULL);
	clk_disable(fuse->clk);
	clk_put(fuse->clk);
	iounmap(fuse->reg_base);
	kfree(fuse);

	return 0;
}

static struct platform_driver mmp2_fuse_driver = {
	.driver = {
		.name	= "mmp2-fuse",
	},
	.probe		= mmp2_fuse_probe,
	.remove		= mmp2_fuse_remove,
};

static int __init mmp2_fuse_init(void)
{
	return platform_driver_register(&mmp2_fuse_driver);
}

static void __exit mmp2_fuse_exit(void)
{
	platform_driver_unregister(&mmp2_fuse_driver);
}

arch_initcall(mmp2_fuse_init);
module_exit(mmp2_fuse_exit);
