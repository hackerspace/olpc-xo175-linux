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
#include <linux/pm_qos_params.h>
#include <mach/uio_hdmi.h>
#include <linux/earlysuspend.h>
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
#include <mach/addr-map.h>
#include <mach/cputype.h>
#endif
#ifdef CONFIG_CPU_PXA978
#include <mach/dvfm.h>
#endif
#include <plat/pm.h>

static atomic_t hdmi_state = ATOMIC_INIT(0);
static int early_suspend_flag;
static int late_disable_flag;
enum connect_lock con_lock;
static bool timer_inited = 0;
static int suspend_flag;
#ifdef CONFIG_CPU_PXA978
static int dvfm_dev_idx;
static unsigned int *arb_f_mc, *arb_n_mc;
static unsigned int val_f_mc, val_n_mc;
#endif

struct hdmi_instance {
	struct clk *clk;
	void *reg_base;
	void *sspa1_reg_base;
	unsigned int hpd_in; /* if cable plug in, this is the gpio value*/
	unsigned int gpio;
	unsigned int edid_bus_num;
	struct timer_list jitter_timer;
	struct work_struct work;
	struct delayed_work delay_resumed;
	struct delayed_work delay_disable;
	struct uio_info uio_info;
	struct early_suspend    early_suspend;
	int (*hdmi_power)(int on);
	struct pm_qos_request_list qos_cpufreq_min;
	struct pm_qos_request_list qos_cpufreq_disable;
	struct pm_qos_request_list qos_idle;
};

static void set_power_constraint(struct hdmi_instance *hi, int min)
{
#ifdef CONFIG_CPU_MMP2
	pm_qos_update_request(&hi->qos_idle, PM_QOS_CONSTRAINT);
	pm_qos_update_request(&hi->qos_cpufreq_min, min);
	pm_qos_update_request(&hi->qos_cpufreq_disable, 1);
#endif

#ifdef CONFIG_CPU_MMP3
	pm_qos_update_request(&hi->qos_cpufreq_min, min);
#endif

#ifdef CONFIG_CPU_PXA978
	printk("hdmi: set_power_constraint\n");
	dvfm_disable_op_name("156M", dvfm_dev_idx);
	dvfm_disable_op_name("312M", dvfm_dev_idx);
	dvfm_disable_op_name("624M", dvfm_dev_idx);
	/* Disable Lowpower mode */
	dvfm_disable_op_name("D2", dvfm_dev_idx);
	dvfm_disable_op_name("CG", dvfm_dev_idx);
#endif
}

static void unset_power_constraint(struct hdmi_instance *hi)
{
#ifdef CONFIG_CPU_MMP2
	pm_qos_update_request(&hi->qos_idle, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&hi->qos_cpufreq_min, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_request(&hi->qos_cpufreq_disable, PM_QOS_DEFAULT_VALUE);
#endif

#ifdef CONFIG_CPU_MMP3
	pm_qos_update_request(&hi->qos_cpufreq_min, PM_QOS_DEFAULT_VALUE);
#endif

#ifdef CONFIG_CPU_PXA978
	printk("hdmi: unset_power_constraint\n");
	dvfm_enable_op_name("156M", dvfm_dev_idx);
	dvfm_enable_op_name("312M", dvfm_dev_idx);
	dvfm_enable_op_name("624M", dvfm_dev_idx);
	/* Enable Lowpower mode */
	dvfm_enable_op_name("D2", dvfm_dev_idx);
	dvfm_enable_op_name("CG", dvfm_dev_idx);
#endif
}

#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
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
#endif

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
	int hpd = -1, status;
#ifdef CONFIG_CPU_PXA978
	int hdmi_freq = 0;
#endif
	switch (cmd) {
	case SSPA1_GET_VALUE:
		if (copy_from_user(&offset, argp, sizeof(offset)))
			return -EFAULT;
		tmp = readl(hi->sspa1_reg_base + offset);
		if (copy_to_user(argp, &tmp, sizeof(tmp)))
			return -EFAULT;
		break;
	case HPD_PIN_READ:
		/* when resume, force disconnect/connect HDMI */
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
		if (con_lock == FIRST_ACCESS_LOCK) {
			hpd = -1;
			con_lock = SECOND_ACCESS_LOCK;
		} else if (con_lock == SECOND_ACCESS_LOCK) {
			hpd = 0;
			con_lock = UNLOCK;
		}
#else
		if (con_lock == FIRST_ACCESS_LOCK) {
			hpd = 1;
			con_lock = UNLOCK;
		}
#endif
		else {
			status = gpio_get_value(hi->gpio);
			/* hdmi stack recognize 0 as connected, 1 as disconnected*/
			if (status == hi->hpd_in)
				hpd = 0;
			else
				hpd = 1;
			/*if disconnected HDMI , late_disable_flag is
			 * 1, 300 ms is the time wait for HDMI is
			 * disabled, then disp1_axi_bus can be disabled.
			 * disp1_axi_bus clear will cause HDMI clock
			 * disbaled and any operation not takes effects*/
			if (late_disable_flag && hpd)
				schedule_delayed_work(&hi->delay_disable,
					msecs_to_jiffies(300));
		}

		pr_debug("early_suspend_flag %d Kernel space: hpd is %d\n",
			early_suspend_flag, hpd);
		if (copy_to_user(argp, &hpd, sizeof(int))) {
			pr_err("copy_to_user error !~!\n");
			return -EFAULT;
		}
		printk("uio_hdmi: report cable %s to hdmi-service\n",
			(hpd==0)?"pulg in":"pull out");
		break;
	case EDID_NUM:
		if (copy_to_user(argp, &hi->edid_bus_num,
					sizeof(unsigned int))) {
			pr_err("copy to user error !\n");
			return -EFAULT;
		}
		break;
#ifdef CONFIG_CPU_PXA978
	case HDMI_PLL_ENABLE:
		clk_enable(hi->clk);
		break;
	case HDMI_PLL_DISABLE:
		clk_disable(hi->clk);
		break;
	case HDMI_PLL_SETRATE:
		if (copy_from_user(&hdmi_freq, argp, sizeof(hdmi_freq)))
			return -EFAULT;
		printk("uio_hdmi: set TMDS clk freq = %dMhz\n", hdmi_freq/5);
		if (clk_set_rate(hi->clk, hdmi_freq * 1000000)) {
			pr_err(KERN_ERR "uio_hdmi: HDMI PLL set failed!\n");
			return -EFAULT;
		}
		break;
#endif
	default:
		pr_err("uio_hdmi: no suppprt ioctl!\n");
		return -EFAULT;
	}
	return 0;
}

static int hdmi_remove(struct platform_device *pdev)
{
	return 0;
}
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
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
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hdmi_early_suspend(struct early_suspend *h)
{
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
	struct hdmi_instance *hi =
		container_of(h, struct hdmi_instance, early_suspend);
	if (atomic_read(&hdmi_state) == 1)
		unset_power_constraint(hi);
#endif
	early_suspend_flag = 1;
	return;
}
static void hdmi_late_resume(struct early_suspend *h)
{
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
	struct hdmi_instance *hi =
		container_of(h, struct hdmi_instance, early_suspend);
	if (atomic_read(&hdmi_state) == 1)
		set_power_constraint(hi, HDMI_FREQ_CONSTRAINT);
#endif
	early_suspend_flag = 0;
	suspend_flag = 0;
	return;
}
#endif
static int hdmi_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);
	suspend_flag = 1;
	if (atomic_read(&hdmi_state) == 1) {
		clk_disable(hi->clk);
		if (hi->hdmi_power)
			hi->hdmi_power(0);
#ifdef CONFIG_CPU_PXA978
			unset_power_constraint(hi);
#endif
		printk("uio_hdmi: suspend done!\n");
	}
#ifdef CONFIG_CPU_PXA978
	/* always turn off 5v power*/
	if (hi->hdmi_power)
		hi->hdmi_power(0);
#endif
	pdev->dev.power.power_state = mesg;
	return 0;
}

static int hdmi_resume(struct platform_device *pdev)
{
	struct hdmi_instance *hi = platform_get_drvdata(pdev);
#if defined(CONFIG_CPU_PXA978) || defined(CONFIG_CPU_MMP3)
	/* always turn on 5v power*/
	if (hi->hdmi_power)
		hi->hdmi_power(1);
#endif
	if (gpio_get_value(hi->gpio) == hi->hpd_in) {
#ifdef CONFIG_CPU_PXA978
		set_power_constraint(hi, -1);
#endif
		/*if connected, reset HDMI*/
		atomic_set(&hdmi_state, 1);
		clk_enable(hi->clk);
#ifdef CONFIG_CPU_MMP2
		if (hi->hdmi_power)
			hi->hdmi_power(1);
#endif
		con_lock = FIRST_ACCESS_LOCK;
		/* send disconnect event to upper layer */
		uio_event_notify(&hi->uio_info);
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
		/*if uio_event_notify both directly, 1 event will be
		 * missed, so delayed_work*/
		schedule_delayed_work(&hi->delay_resumed,
			msecs_to_jiffies(1500));
#else
		atomic_set(&hdmi_state, 0);
		mod_timer(&hi->jitter_timer, jiffies + HZ);
#endif
	} else if (atomic_read(&hdmi_state) == 1) {
		atomic_set(&hdmi_state, 0);
		late_disable_flag = 1; /*wait for hdmi disbaled*/
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
		if (cpu_is_mmp2() && hi->hdmi_power)
			hi->hdmi_power(0);
		if (early_suspend_flag == 0)
			unset_power_constraint(hi);
#else
		unset_power_constraint(hi);
#endif
		uio_event_notify(&hi->uio_info);
	}
	printk("uio_hdmi: resume done!\n");
	return 0;
}

static void delayed_disable(struct work_struct *work)
{
	struct hdmi_instance *hi = container_of((struct delayed_work *)work,
			struct hdmi_instance, delay_disable);
	if (late_disable_flag) {
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
		if (cpu_is_mmp2())
#endif
		clk_disable(hi->clk);
		late_disable_flag = 0;
	}
}

static void delayed_resume(struct work_struct *work)
{
	struct hdmi_instance *hi = container_of((struct delayed_work *)work,
			struct hdmi_instance, delay_resumed);
	/* send connect event to upper layer */
	uio_event_notify(&hi->uio_info);
}

static void hdmi_switch_work(struct work_struct *work)
{
	struct hdmi_instance *hi =
		container_of(work, struct hdmi_instance, work);
	int state = gpio_get_value(hi->gpio);
	if(state != hi->hpd_in) {
		if (atomic_cmpxchg(&hdmi_state, 1, 0) == 1) {
			late_disable_flag = 1; /*wait for hdmi disbaled*/
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
			if (cpu_is_mmp2() && hi->hdmi_power)
				hi->hdmi_power(0);
			if (early_suspend_flag == 0)
				unset_power_constraint(hi);
#else
			unset_power_constraint(hi);
			*arb_f_mc = val_f_mc;
			*arb_n_mc = val_n_mc;
#endif
			/*if hdmi_state change, report hpd*/
			uio_event_notify(&hi->uio_info);
		}
	} else {
		if (atomic_cmpxchg(&hdmi_state, 0, 1) == 0) {
#if defined(CONFIG_CPU_MMP2) || defined(CONFIG_CPU_MMP3)
			if (cpu_is_mmp2()) {
				if (hi->hdmi_power)
					hi->hdmi_power(1);
				clk_enable(hi->clk);
			}
			if (early_suspend_flag == 0)
				set_power_constraint(hi, HDMI_FREQ_CONSTRAINT);
#else
			/* raise priority of display controller mc arbiter*/
			val_f_mc = *arb_f_mc;
			val_n_mc = *arb_n_mc;
			*arb_f_mc = 0x010f0101;
			*arb_n_mc = 0x010f0101;
			set_power_constraint(hi, -1);
			clk_enable(hi->clk);
#endif
			/*if hdmi_state change, report hpd*/
			uio_event_notify(&hi->uio_info);
		}
	}
	pr_debug("++++++++++++ %s state %x hdmi_state %d\n", __func__,
		state, atomic_read(&hdmi_state));
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
#ifdef CONFIG_CPU_PXA978
	/* during suspend, need not response to any irq*/
	if (suspend_flag) {
		printk("uio_hdmi: ~ignore all hpd during suspend~\n");
		return IRQ_NONE;
	}
#endif

	if (timer_inited)
		mod_timer(&hi->jitter_timer, jiffies + HZ);
	/*printk("uio_hdmi: irq HDMI cable is %s\n",
		(hi->hpd_in == gpio_get_value(hi->gpio))?"plug in":"pull out");*/
	/*Don't report hpd in top half, wait for jitter is gone.*/
	return IRQ_NONE;
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

	if (pdata->sspa_reg_base) {
		hi->sspa1_reg_base = ioremap_nocache(pdata->sspa_reg_base, 0xff);
		if (hi->sspa1_reg_base == NULL) {
			printk(KERN_WARNING "failed to request register memory\n");
			ret = -EBUSY;
			goto out_free;
		}
	}
	pm_qos_add_request(&hi->qos_cpufreq_min, PM_QOS_CPUFREQ_MIN,
			PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&hi->qos_cpufreq_disable, PM_QOS_CPUFREQ_DISABLE,
			PM_QOS_DEFAULT_VALUE);
	pm_qos_add_request(&hi->qos_idle, PM_QOS_CPU_DMA_LATENCY,
			PM_QOS_DEFAULT_VALUE);

	platform_set_drvdata(pdev, hi);

	hi->uio_info.name = "uio-hdmi";
	hi->uio_info.version = "build1";
	hi->uio_info.irq_flags = IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING;
	hi->uio_info.handler = hpd_handler;
	hi->gpio = pdata->gpio;
	hi->hpd_in = pdata->hpd_val;
	hi->edid_bus_num = pdata->edid_bus_num;
	if (hi->edid_bus_num == 0)
		hi->edid_bus_num = 6;

	ret = gpio_request(pdata->gpio, pdev->name);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed in gpio_request\n", __func__);
		goto out_free;
	}
	ret = gpio_direction_input(pdata->gpio);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed in gpio_direction_input\n", __func__);
		goto out_free;
	}
	hi->uio_info.irq = gpio_to_irq(pdata->gpio);
	if (hi->uio_info.irq < 0) {
		printk(KERN_ERR "%s: failed in gpio_to_irq\n", __func__);
		ret = hi->uio_info.irq;
		goto out_free;
	}

	hi->uio_info.mem[0].internal_addr = hi->reg_base;
	hi->uio_info.mem[0].addr = res->start;
	hi->uio_info.mem[0].memtype = UIO_MEM_PHYS;
	hi->uio_info.mem[0].size = res->end - res->start + 1;
	hi->uio_info.mem[0].name = "hdmi-iomap";
	hi->uio_info.priv = hi;

	if (pdata->itlc_reg_base) {
		hi->uio_info.mem[1].internal_addr =
			ioremap_nocache(pdata->itlc_reg_base, 0xff);
		hi->uio_info.mem[1].addr = pdata->itlc_reg_base;
		hi->uio_info.mem[1].memtype = UIO_MEM_PHYS;
		hi->uio_info.mem[1].size = 0xff;
	}

	hi->uio_info.open = hdmi_open;
	hi->uio_info.release = hdmi_release;
	hi->uio_info.ioctl = hdmi_ioctl;
	if (pdata->hdmi_v5p_power)
		hi->hdmi_power = pdata->hdmi_v5p_power;

#ifdef CONFIG_CPU_PXA978
	dvfm_register("uio-hdmi", &dvfm_dev_idx);
	/* mc arbiter*/
	arb_f_mc = ioremap_nocache(0x7ff007b0, 4);
	arb_n_mc = ioremap_nocache(0x7ff00280, 4);
#endif

#if defined(CONFIG_CPU_PXA978) || defined(CONFIG_CPU_MMP3)
	if (hi->hdmi_power)
		hi->hdmi_power(1); /* mmp3 and nevo need 5v to detect hpd*/
#endif

	/* Check HDMI cable when boot up */
	ret = gpio_get_value(hi->gpio);
	printk(KERN_INFO"%s hpd %s\n",
		__func__, (ret == hi->hpd_in)?"plug in":"pull out");
#ifdef CONFIG_CPU_PXA978 /* nevo*/
	if (ret == hi->hpd_in)
		atomic_set(&hdmi_state, 1);
#else /* mmp2 & mmp3*/
	if (ret == hi->hpd_in) {
		atomic_set(&hdmi_state, 1);
		set_power_constraint(hi, HDMI_FREQ_CONSTRAINT);
		if (cpu_is_mmp2() && hi->hdmi_power)
			hi->hdmi_power(1);
		clk_enable(hi->clk);
	} else if (cpu_is_mmp3()) {
		clk_enable(hi->clk);
	}
#endif

	ret = uio_register_device(&pdev->dev, &hi->uio_info);
	if (ret) {
		printk(KERN_ERR"%s: register device fails !!!\n", __func__);
		goto out_free;
	}

	/* avoid cable hot plug/pull out jitter within 1s*/
	setup_timer(&hi->jitter_timer, work_launch, (unsigned long)hi);
	INIT_WORK(&hi->work, hdmi_switch_work);
	timer_inited = 1;

	/* silicon issue on MMP: delayed 300ms to disable clk when cable pull out*/
	INIT_DELAYED_WORK(&hi->delay_disable, delayed_disable);

	/* during resume: simulate cable pull out->wait 1.5s->plug in*/
	INIT_DELAYED_WORK(&hi->delay_resumed, delayed_resume);

#ifdef CONFIG_HAS_EARLYSUSPEND
	hi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	hi->early_suspend.suspend = hdmi_early_suspend;
	hi->early_suspend.resume = hdmi_late_resume;
	register_early_suspend(&hi->early_suspend);
#endif
	platform_set_drvdata(pdev, hi);
	return 0;

out_free:
	clk_disable(hi->clk);
	clk_put(hi->clk);
#if defined(CONFIG_CPU_PXA978) || defined(CONFIG_CPU_MMP3)
	if (hi->hdmi_power)
		hi->hdmi_power(0);
#else
	if (atomic_read(&hdmi_state) && (hi->hdmi_power))
		hi->hdmi_power(0);
#endif
	kfree(hi);

	return ret;
}

static struct platform_driver hdmi_driver = {
	.probe	= hdmi_probe,
	.remove	= hdmi_remove,
	.driver = {
		.name	= "uio-hdmi",
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

late_initcall(hdmi_init);
module_exit(hdmi_exit);

MODULE_DESCRIPTION("UIO driver for Marvell hdmi");
MODULE_LICENSE("GPL");
