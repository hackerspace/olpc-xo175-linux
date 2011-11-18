/*
 * Marvell HDMI UIO driver
 *
 * Yifan Zhang <zhangyf@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 * (c) 2010
 *
 */

#include <linux/uio_driver.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <mach/uio_hdmi.h>
#include <linux/earlysuspend.h>
#include <mach/addr-map.h>
#include <mach/cputype.h>

static atomic_t hdmi_state = ATOMIC_INIT(0);
static int early_suspend_flag;
static int late_disable_flag;
enum connect_lock con_lock;

struct hdmi_instance {
	struct clk *clk;
	void *reg_base;
	void *sspa1_reg_base;
	unsigned int gpio;
	unsigned int edid_bus_num;
	struct work_struct work;
	struct delayed_work hpd_work;
	struct uio_info uio_info;
	struct early_suspend    early_suspend;
};
static unsigned gsspa1_reg_base;

static u32 hdmi_direct_read(unsigned addr)
{
       u32 hdmi_addr = AXI_VIRT_BASE + 0xbc00;

	return __raw_readl(hdmi_addr + addr);
}

static void hdmi_direct_write(unsigned addr, unsigned data)
{
       u32 hdmi_addr = AXI_VIRT_BASE + 0xbc00;

	__raw_writel(data, hdmi_addr + addr);
}

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
		hpd = gpio_get_value(hi->gpio);
		/*if disconnected HDMI , late_disable_flag is 1, 300 ms is
		 * the time wait for HDMI is disabled, then disp1_axi_bus
		 * can be disabled. disp1_axi_bus clear will cause HDMI
		 * clock disbaled and any operation not takes effects*/
		if (late_disable_flag && hpd)
			schedule_delayed_work(&hi->hpd_work,
					msecs_to_jiffies(300));
		/* when resume, force disconnect/connect HDMI */
		if (con_lock == FIRST_ACCESS_LOCK) {
			hpd = -1;
			con_lock = SECOND_ACCESS_LOCK;
		} else if (con_lock == SECOND_ACCESS_LOCK) {
			hpd = 0;
			con_lock = UNLOCK;
		}
		pr_debug("early_suspend_flag %d Kernel space: hpd is %d\n",
			early_suspend_flag, hpd);
		if (copy_to_user(argp, &hpd, sizeof(int))) {
			pr_err("copy_to_user error !~!\n");
			return -EFAULT;
		}
		break;
	case EDID_NUM:
		if (copy_to_user(argp, &hi->edid_bus_num,
					sizeof(unsigned int))) {
			pr_err("copy to user error !\n");
			return -EFAULT;
		}
	}
	return 0;
}

static int hdmi_remove(struct platform_device *pdev)
{
	return 0;
}

void hdmi_3d_sync_view(void)
{
	u32 reg;

	reg = hdmi_direct_read(0x30);
	reg &= ~ ((1 << 1) | (1 << 2) | (1 << 3));
	reg |= 1 << 2;
	hdmi_direct_write(0x30, 0);
	hdmi_direct_write(0x30, reg);
}
EXPORT_SYMBOL(hdmi_3d_sync_view);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmi_early_suspend(struct early_suspend *h)
{
	early_suspend_flag = 1;

	return;
}
static void hdmi_late_resume(struct early_suspend *h)
{
	early_suspend_flag = 0;

	return;
}
#endif
static int hdmi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);

	if (atomic_read(&hdmi_state) == 1)
		clk_disable(hi->clk);
	pdev->dev.power.power_state = mesg;

	return 0;
}

static int hdmi_resume(struct platform_device *pdev)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);

	if (gpio_get_value(hi->gpio) == 0) {
		/*if connected, reset HDMI*/
		atomic_set(&hdmi_state, 1);
		clk_enable(hi->clk);
		con_lock = FIRST_ACCESS_LOCK;
		/* send disconnect event to upper layer */
		uio_event_notify(&hi->uio_info);
		/*if uio_event_notify both directly, 1 event will be
		 * missed, so delayed_work*/
		schedule_delayed_work(&hi->hpd_work, msecs_to_jiffies(1500));
	} else if (atomic_read(&hdmi_state) == 1) {
		atomic_set(&hdmi_state, 0);
		uio_event_notify(&hi->uio_info);
	}

	return 0;
}

static void hdmi_delayed_func(struct work_struct *work)
{
	struct hdmi_instance *hi = container_of((struct delayed_work *)work,
			struct hdmi_instance, hpd_work);
	if (late_disable_flag) {
		if (cpu_is_mmp2())
			clk_disable(hi->clk);
		late_disable_flag = 0;
	} else {
		/* send connect event to upper layer */
		uio_event_notify(&hi->uio_info);
	}
}

static void hdmi_switch_work(struct work_struct *work)
{
	struct hdmi_instance *hi =
		container_of(work, struct hdmi_instance, work);
	int state = gpio_get_value(hi->gpio);

	if (state != 0) {
		state = 0;
		late_disable_flag = 1; /*wait for hdmi disbaled*/
		atomic_set(&hdmi_state, 0);
	} else {
		if (cpu_is_mmp2())
			clk_enable(hi->clk);
		state = 1;
		atomic_set(&hdmi_state, 1);
	}
	pr_debug("++++++++++++ %s state %x hdmi_state %d\n", __func__,
		state, atomic_read(&hdmi_state));
}

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
	struct uio_hdmi_platform_data *pdata;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pdata = pdev->dev.platform_data;
	if (res == NULL) {
		printk(KERN_ERR "hdmi_probe: no memory resources given");
		return -ENODEV;
	}

	hi = kzalloc(sizeof(*hi), GFP_KERNEL);
	if (hi == NULL) {
		printk(KERN_ERR "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	hi->clk = clk_get(NULL, "HDMICLK");
	if (IS_ERR(hi->clk)) {
		pr_err("%s: can't get HDMICLK\n", __func__);
		kfree(hi);
		return  -EIO;
	}

	hi->reg_base = ioremap(res->start, res->end - res->start + 1);
	if (hi->reg_base == NULL) {
		printk(KERN_ERR "%s: can't remap resgister area", __func__);
		ret =  -ENOMEM;
		goto out_free;
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
	hi->edid_bus_num = pdata->edid_bus_num;
	if (hi->edid_bus_num == 0)
		hi->edid_bus_num = 6;

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

	/* Check HDMI cable when boot up */
	ret = gpio_get_value(hi->gpio);
	printk(KERN_INFO"%s hpd 0x%x------------\n", __func__, ret);
	if (ret == 0) {
		clk_enable(hi->clk);
		atomic_set(&hdmi_state, 1);
	}else if (cpu_is_mmp3()) {
		clk_enable(hi->clk);
	}

	INIT_WORK(&hi->work, hdmi_switch_work);
	INIT_DELAYED_WORK(&hi->hpd_work, hdmi_delayed_func);

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
