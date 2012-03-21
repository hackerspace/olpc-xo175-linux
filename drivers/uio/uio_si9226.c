/******************************************************
 * Marvell HDMI UIO driver
 *
 * Yifan Zhang <zhangyf@marvell.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 * (c) 2010
 *
*******************************************************/

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
#include <mach/dvfm.h>
#include <linux/earlysuspend.h>
#include <plat/pm.h>


static atomic_t hdmi_state = ATOMIC_INIT(0);
static int early_suspend_flag;
static int late_disable_flag;
enum connect_lock con_lock;
static bool timer_inited = 0;
static int si9226_suspend = 0;

struct hdmi_instance {
	struct timer_list jitter_timer;
	struct work_struct work;
	struct delayed_work hpd_work;
	struct uio_info uio_info;
	struct early_suspend    early_suspend;
	int (*hdmi_power)(int on);
};
int hdmi_open(struct uio_info *info, struct inode *inode, void *file_priv)
{
	return 0;
}

int hdmi_release(struct uio_info *info, struct inode *indoe, void *file_priv)
{
	return 0;
}


extern int sii9226_enable(bool en);
extern int sii9226_detect(void);
static int dvfm_dev_idx;

static int hdmi_ioctl(struct uio_info *info, unsigned cmd, unsigned long arg,
		void *file_priv)
{
	void *argp = (void *)arg;
	int val;
	switch (cmd) {
	case HPD_PIN_READ:
		val = sii9226_detect();
		/* if enter suspend, always report as disconnected*/
		if (si9226_suspend == 1)
			val = 0;
		if (copy_to_user(argp, &val, sizeof(int))) {
			pr_err("%s: copy_to_user error !~!\n", __func__);
			return -EFAULT;
		}
		break;
	case HDMI_PLL_ENABLE:
		/* Disable Lowpower mode */
		dvfm_disable_op_name("156M", dvfm_dev_idx);
		dvfm_disable_op_name("156M_HF", dvfm_dev_idx);
		dvfm_disable_op_name("416M", dvfm_dev_idx);
		sii9226_enable(1);
		printk("%s: enabled\n", __func__);
		break;
	case HDMI_PLL_DISABLE:
		/* Enable Lowpower mode */
		dvfm_enable_op_name("156M", dvfm_dev_idx);
		dvfm_enable_op_name("156M_HF", dvfm_dev_idx);
		dvfm_enable_op_name("416M", dvfm_dev_idx);
		sii9226_enable(0);
		printk("%s: disabled\n", __func__);
		break;
	default:
		pr_err("hdmi_ioctl: no suppprt ioctl!\n");
		return -EFAULT;
	}
	return 0;
}

static int hdmi_remove(struct platform_device *pdev)
{
	return 0;
}

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
	sii9226_enable(0);
	if (hi->hdmi_power)
		hi->hdmi_power(0);
	pdev->dev.power.power_state = mesg;
	si9226_suspend = 1;

	return 0;
}

static int hdmi_resume(struct platform_device *pdev)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);
	if (hi->hdmi_power)
		hi->hdmi_power(1);
	sii9226_enable(1);
	si9226_suspend = 0;
#if 0/*TODO*/

	struct hdmi_instance *hi = platform_get_drvdata(pdev);
	if (gpio_get_value(hi->gpio) == hi->hpd_in) {
		/*if connected, reset HDMI*/
		atomic_set(&hdmi_state, 1);
		if (hi->hdmi_power)
			hi->hdmi_power(1);
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
#endif
	return 0;
}

static void hdmi_delayed_func(struct work_struct *work)
{
	struct hdmi_instance *hi = container_of((struct delayed_work *)work,
			struct hdmi_instance, hpd_work);
	if (late_disable_flag) {
		late_disable_flag = 0;
	} else {
		/* send connect event to upper layer */
		uio_event_notify(&hi->uio_info);
	}
}

static void hdmi_switch_work(struct work_struct *work)
{
#if 0/*TODO*/
	struct hdmi_instance *hi =
		container_of(work, struct hdmi_instance, work);
	int state = gpio_get_value(hi->gpio);
	if(state != hi->hpd_in) {
		if (atomic_cmpxchg(&hdmi_state, 1, 0) == 1) {
			printk("uio_hdmi: cable pull out\n\n");
			late_disable_flag = 1; /*wait for hdmi disbaled*/
			/*if hdmi_state change, report hpd*/
			uio_event_notify(&hi->uio_info);
		}
	} else {
		if (atomic_cmpxchg(&hdmi_state, 0, 1) == 0) {
			printk("uio_hdmi: cable plug in\n\n");
			/*if hdmi_state change, report hpd*/
			uio_event_notify(&hi->uio_info);
		}
	}
	pr_debug("++++++++++++ %s state %x hdmi_state %d\n", __func__,
		state, atomic_read(&hdmi_state));
#endif
}

/* set_power_constraint can't be called in interrupt context, use timer to
 * workaround this issue.
 */
void work_launch(unsigned long data)
{
	struct hdmi_instance *hi = (struct hdmi_instance *)data;

	pr_debug("%s\n", __func__);
	schedule_work(&hi->work);
}

/* use timer to remove jitter
 */
static irqreturn_t hpd_handler(int irq, struct uio_info *dev_info)
{
	struct hdmi_instance *hi =
		container_of(dev_info, struct hdmi_instance, uio_info);

	pr_debug("%s\n", __func__);
	if (timer_inited)
		mod_timer(&hi->jitter_timer, jiffies + HZ);

	/*Don't report hpd in top half, wait for jitter is gone.*/
	return IRQ_NONE;
}

static int hdmi_probe(struct platform_device *pdev)
{
	struct hdmi_instance *hi;
	int ret;
	printk("hdmi uio_si9226 ==========================+\n");
	hi = kzalloc(sizeof(*hi), GFP_KERNEL);
	if (hi == NULL) {
		printk(KERN_ERR "%s: out of memory\n", __func__);
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, hi);

	hi->uio_info.name = "uio-si9226";
	hi->uio_info.version = "build1";
	hi->uio_info.irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
	hi->uio_info.handler = hpd_handler;

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
	setup_timer(&hi->jitter_timer, work_launch, (unsigned long)hi);
	INIT_WORK(&hi->work, hdmi_switch_work);
	INIT_DELAYED_WORK(&hi->hpd_work, hdmi_delayed_func);
	timer_inited = 1;

#ifdef CONFIG_HAS_EARLYSUSPEND
	hi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	hi->early_suspend.suspend = hdmi_early_suspend;
	hi->early_suspend.resume = hdmi_late_resume;
	register_early_suspend(&hi->early_suspend);
#endif
	platform_set_drvdata(pdev, hi);
	printk("hdmi uio_si9226 ==========================-\n");
	return 0;

out_free:
	kfree(hi);
	return ret;
}

static struct platform_driver hdmi_driver = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver = {
		.name	= "uio-si9226",
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
	dvfm_register("uio-si9226", &dvfm_dev_idx);
	return platform_driver_register(&hdmi_driver);
}

late_initcall(hdmi_init);
module_exit(hdmi_exit);

MODULE_DESCRIPTION("UIO driver for SI9226");
MODULE_LICENSE("GPL");
