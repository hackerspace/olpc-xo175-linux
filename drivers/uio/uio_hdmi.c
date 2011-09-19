/*
 * Marvell HDMI UIO driver
 *
 * Yifan Zhang <zhangyf@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 * (c) 2011
 *
 */

#include <linux/uio_driver.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <mach/uio_hdmi.h>
#include <linux/earlysuspend.h>
#include <mach/addr-map.h>
#include <linux/clk.h>

static atomic_t hdmi_state = ATOMIC_INIT(0);
static int early_suspend_flag = 0;
static atomic_t edge_lock = ATOMIC_INIT(0);

struct hdmi_instance {
	struct clk *clk;
	void *reg_base;
	void *sspa1_reg_base;
	unsigned int gpio;
	struct work_struct work;
	struct uio_info uio_info;
	struct early_suspend    early_suspend;
};
static unsigned gsspa1_reg_base;

int hdmi_open(struct uio_info *info, struct inode *inode, void *file_priv)
{
	return 0;
}

int hdmi_release(struct uio_info *info, struct inode *indoe, void *file_priv)
{
	return 0;
}

static int hdmi_ioctl(struct uio_info *info, unsigned cmd, unsigned long arg,
		void *file_priv)
{
	unsigned offset, tmp;
	void *argp = (void *)arg;
	struct hdmi_instance *hi =
		container_of(info, struct hdmi_instance, uio_info);
	int hpd = -1;

	switch (cmd) {
	case SSPA1_GET_VALUE:
		if (copy_from_user(&offset, argp, sizeof(offset)))
			return -EFAULT;
		tmp = readl(gsspa1_reg_base + offset);
		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			return -EFAULT;
		break;
	case HPD_PIN_READ:
		if (!early_suspend_flag)
			hpd = gpio_get_value(hi->gpio);
		pr_debug("early_suspend_flag %d Kernel space: hpd is %d\n",
			early_suspend_flag, hpd);
		if (copy_to_user(argp, &hpd, sizeof(int))) {
			pr_err("copy_to_user error !~!\n");
			return -EFAULT;
		}
	}
	return 0;
}

static int hdmi_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static unsigned int hdmi_phy_cfg_2;
static void hdmi_early_suspend(struct early_suspend *h)
{
	struct hdmi_instance *hi =
	container_of(h, struct hdmi_instance, early_suspend);
	unsigned int addr;

	printk(KERN_DEBUG "----%s: %d\n", __func__, atomic_read(&hdmi_state));
	addr = AXI_VIRT_BASE + 0xbc10;
	hdmi_phy_cfg_2 = __raw_readl(addr);
	__raw_writel(hdmi_phy_cfg_2 | 0xf<<19, addr);
	early_suspend_flag = 1;

	if (atomic_read(&hdmi_state) == 1) {
		/* send disconnect event to upper layer */
		uio_event_notify(&hi->uio_info);

	}

	return;
}
static void hdmi_late_resume(struct early_suspend *h)
{
	struct hdmi_instance *hi =
	container_of(h, struct hdmi_instance, early_suspend);
	unsigned int addr;

	printk(KERN_DEBUG "-----%s: %d\n", __func__, atomic_read(&hdmi_state));
	addr = AXI_VIRT_BASE + 0xbc10;
	__raw_writel(hdmi_phy_cfg_2, addr);
	early_suspend_flag = 0;

	if (atomic_read(&hdmi_state) == 1) {
		/* send connect event to upper layer */
		uio_event_notify(&hi->uio_info);
	}

	return;
}
#endif

static int hdmi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);

	clk_disable(hi->clk);
	pdev->dev.power.power_state = mesg;

	return 0;
}

static int hdmi_resume(struct platform_device *pdev)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);

	clk_enable(hi->clk);
	return 0;
}

#if 0
static atomic_t edge_lock = ATOMIC_INIT(0);
static void hdmi_switch_work(struct work_struct *work)
{
	struct hdmi_instance *hi =
		container_of(dev_info, struct hdmi_instance, uio_info);
	int state = gpio_get_value(data->gpio);

	if (state != 0) {
		/* cable disconnected */
		if (atomic_read(&edge_lock) == 1) {
			atomic_set(&hdmi_state, 0);
			atomic_set(&edge_lock, 0);
			if (early_suspend_flag == 0)
				unset_dvfm_constraint();
			switch_set_state(&data->sdev, state);
		}
	} else {
		/* cable connected */
		if (atomic_read(&edge_lock) == 0) {
			atomic_set(&hdmi_state, 1);
			atomic_set(&edge_lock, 1);
			if (early_suspend_flag == 0)
				set_dvfm_constraint();
			switch_set_state(&data->sdev, state);
		}
	}
}
#else
static void hdmi_switch_work(struct work_struct *work)
{
	struct hdmi_instance *hi =
		container_of(work, struct hdmi_instance, work);
	int state = gpio_get_value(hi->gpio);

	if (state != 0) {
		atomic_set(&hdmi_state, 0);
		state = 0;
	} else {
		atomic_set(&hdmi_state, 1);
		state = 1;
	}
	pr_debug("++++++++++++ %s state %x hdmi_state %d\n", __func__,
		state, atomic_read(&hdmi_state));
}
#endif

static irqreturn_t hpd_handler(int irq, struct uio_info *dev_info)
{
	struct hdmi_instance *hi =
		container_of(dev_info, struct hdmi_instance, uio_info);

	pr_debug("----- %s\n", __func__);
	schedule_work(&hi->work);
	return IRQ_HANDLED;
}

static int hdmi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct hdmi_instance *hi;
	int ret;
	struct clk *clk;
	struct uio_hdmi_platform_data *pdata;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata = pdev->dev.platform_data;
	if (res == NULL) {
		printk(KERN_ERR "hdmi_probe: no memory resources given");
		return -ENODEV;
	}

	clk = clk_get(NULL, "HDMICLK");
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get HDMICLK");
		return PTR_ERR(clk);
	}

	hi = kzalloc(sizeof(*hi), GFP_KERNEL);
	if (hi == NULL) {
		printk(KERN_ERR "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	hi->reg_base = ioremap(res->start, res->end - res->start + 1);
	if (hi->reg_base == NULL) {
		printk(KERN_ERR "%s: can't remap resgister area", __func__);
		return -ENOMEM;
	}
	hi->sspa1_reg_base = ioremap_nocache(pdata->sspa_reg_base, 0xff);
	if (hi->sspa1_reg_base == NULL) {
		printk(KERN_WARNING "failed to request register memory\n");
		ret = -EBUSY;
		goto out_free;
	}
	gsspa1_reg_base = (unsigned)hi->sspa1_reg_base;

	platform_set_drvdata(pdev, hi);

	hi->uio_info.name = "mmp-hdmi";
	hi->uio_info.version = "build1";
	hi->uio_info.irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
	hi->uio_info.handler = hpd_handler;
	hi->gpio = pdata->gpio;
	hi->clk = clk;

	ret = gpio_request(pdata->gpio, pdev->name);
	if (ret < 0)
		goto out_free;

	ret = gpio_direction_input(pdata->gpio);
	if (ret < 0)
		goto out_free;

	hi->uio_info.irq = gpio_to_irq(pdata->gpio);
	if (hi->uio_info.irq < 0) {
		ret = hi->uio_info.irq;
		goto out_free;
	}

	hi->uio_info.mem[0].internal_addr = hi->reg_base;
	hi->uio_info.mem[0].addr = res->start;
	hi->uio_info.mem[0].memtype = UIO_MEM_PHYS;
	hi->uio_info.mem[0].size = res->end - res->start + 1;
	hi->uio_info.mem[0].name = "hdmi-iomap";
	hi->uio_info.priv = hi;

	hi->uio_info.open = hdmi_open;
	hi->uio_info.release = hdmi_release;
	hi->uio_info.ioctl = hdmi_ioctl;
	ret = uio_register_device(&pdev->dev, &hi->uio_info);
	if (ret) {
		printk(KERN_ERR"%s: register device fails !!!\n", __func__);
		goto out_free;
	}

	clk_enable(hi->clk);

	/* Check HDMI cable when boot up */
	ret = gpio_get_value(hi->gpio);
	pr_info("%s hpd 0x%x------------\n", __func__, ret);
	if (ret == 0) {
		atomic_set(&hdmi_state, 1);
		atomic_set(&edge_lock, 1);
	}

	INIT_WORK(&hi->work, hdmi_switch_work);

#ifdef CONFIG_HAS_EARLYSUSPEND
	hi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	hi->early_suspend.suspend = hdmi_early_suspend;
	hi->early_suspend.resume = hdmi_late_resume;
	register_early_suspend(&hi->early_suspend);
#endif

	return 0;

out_free:
	clk_disable(hi->clk);
	clk_put(hi->clk);
	kfree(hi);

	return ret;
}

static struct platform_driver hdmi_driver = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver = {
		.name	= "mmp-hdmi",
		.owner	= THIS_MODULE,
	},
#ifdef CONFIG_PM
	.suspend = hdmi_suspend,
	.resume  = hdmi_resume,
#endif
};

static void __init hdmi_exit(void)
{
	platform_driver_unregister(&hdmi_driver);
}

static int __init hdmi_init(void)
{
	return platform_driver_register(&hdmi_driver);
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_DESCRIPTION("UIO driver for Marvell hdmi");
MODULE_LICENSE("GPL");
