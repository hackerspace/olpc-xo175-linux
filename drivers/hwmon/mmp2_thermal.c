/*
 * drivers/hwmon/mmp2_thermal.c
 *
 * Marvell MMP2 thermal sensor driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/timer.h>

#define LO_THRESH		(0*196/100 + 5294/10)	/*default thermal low threshold is 0 degree Celsius*/
#define HI_THRESH		(70*196/100 + 5294/10)	/*default thermal high threshold is 70 degree Clesius*/
#define ROOM_TEMP_OFFSET	0x0000
#define LO_THRESH_OFFSET	0x0008
#define HI_THRESH_OFFSET	0x000c
#define TS_CTRL_OFFSET		0x0010
#define TS_EN			(1 << 16)
#define ADC_RDY			(1 << 10)
#define HI_INT_STS		(1 << 6)
#define LO_INT_STS		(1 << 5)
#define HI_INT_EN		(1 << 2)
#define LO_INT_EN		(1 << 1)
#define ALARM_DELAY		(10*HZ)		/*alarm cycle time if temperature exceed
						*the threshold, default setting is 10 seconds*/

struct mmp2_thermal_sensor_device {
	struct device	*hwmon_dev;
	struct clk		*clk;
	int irq;
	int low_thresh;
	int high_thresh;
	unsigned int high_temp_flag;
	unsigned int low_temp_flag;
	struct timer_list	thermal_timer;
	struct delayed_work delayed_work;
	unsigned char __iomem	*mmp2_thermal_regs_base;
};

static struct mmp2_thermal_sensor_device g_mmp2_thermal_dev;

static unsigned int read_thermal_reg(unsigned int offset)
{
	struct mmp2_thermal_sensor_device *dev = &g_mmp2_thermal_dev;
	unsigned int ret;

	ret = readl(dev->mmp2_thermal_regs_base + offset);
	return ret;
}

static void write_thermal_reg(unsigned int value, unsigned int offset)
{
	struct mmp2_thermal_sensor_device *dev = &g_mmp2_thermal_dev;
	writel(value, dev->mmp2_thermal_regs_base + offset);
}

static void enable_high_threshold_int(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) | HI_INT_EN;
	write_thermal_reg(value, TS_CTRL_OFFSET);
}

static void disable_high_threshold_int(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) & ~HI_INT_EN;
	write_thermal_reg(value, TS_CTRL_OFFSET);
}

static void enable_low_threshold_int(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) | LO_INT_EN;
	write_thermal_reg(value, TS_CTRL_OFFSET);
}

static void disable_low_threshold_int(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) & ~LO_INT_EN;
	write_thermal_reg(value, TS_CTRL_OFFSET);
}

static void clear_high_int_status(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) | HI_INT_STS;
	write_thermal_reg(value, TS_CTRL_OFFSET);

}

static void clear_low_int_status(void)
{
	unsigned int value;

	value = read_thermal_reg(TS_CTRL_OFFSET) | LO_INT_STS;
	write_thermal_reg(value, TS_CTRL_OFFSET);
}

static void thermal_timer_handler(unsigned long arg)
{
	struct mmp2_thermal_sensor_device *dev = &g_mmp2_thermal_dev;

	if (dev->high_temp_flag) {
		dev->high_temp_flag = 0;
		clear_high_int_status();
		enable_high_threshold_int();
	} else {
		if (dev->low_temp_flag) {
			dev->low_temp_flag = 0;
			clear_low_int_status();
			enable_low_threshold_int();
		}
	}
}

/*get thermal raw temperature*/
static void get_thermal_raw_temp(struct work_struct *work)
{
	unsigned int i, value = 0;
	unsigned int loop = 100;
	ssize_t cur_temp;

	for (i = 0; i < loop; i++)
		value += read_thermal_reg(ROOM_TEMP_OFFSET) & 0x000003FF;

	cur_temp = (value/loop - 5294/10) * 100/196;
	printk(KERN_WARNING "Current raw temperature is %d (%d degrees Celsius)\n",
			value/loop, cur_temp);
}

static irqreturn_t mmp2_thermal_handler(int irq, void *data)
{
	struct mmp2_thermal_sensor_device *thermal_dev = &g_mmp2_thermal_dev;
	unsigned int value;
	ssize_t thres_temp;

	value = read_thermal_reg(TS_CTRL_OFFSET) & HI_INT_STS;
	if (value) {
		thermal_dev->high_temp_flag = 1;
		thres_temp = (thermal_dev->high_thresh - 5294/10) * 100/196;
		printk(KERN_WARNING "High threshold is %d (%d degrees Celsius), temperature exceeds the high threshold\n",
				thermal_dev->high_thresh, thres_temp);
		schedule_delayed_work(&thermal_dev->delayed_work, 0);
		disable_high_threshold_int();

		mod_timer(&thermal_dev->thermal_timer, jiffies + ALARM_DELAY);
	} else {
		value = read_thermal_reg(TS_CTRL_OFFSET) & LO_INT_STS;
		if (value) {
			thermal_dev->low_temp_flag = 1;
			thres_temp = (thermal_dev->low_thresh - 5294/10) * 100/196;
			printk(KERN_WARNING "Low threshold is %d (%d degrees Celsius), temperature below the low threshold\n",
					thermal_dev->low_thresh, thres_temp);
			schedule_delayed_work(&thermal_dev->delayed_work, 0);
			disable_low_threshold_int();

			mod_timer(&thermal_dev->thermal_timer, jiffies + ALARM_DELAY);
		}
	}

	return IRQ_HANDLED;
}


/*T (degrees Celcius) = ((Dout - 529.4)/1.96)*/
static ssize_t get_thermal_raw_data(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	unsigned int i, value = 0;
	unsigned int loop = 100;

	for (i = 0; i < loop; i++)
		value += read_thermal_reg(ROOM_TEMP_OFFSET) & 0x000003FF;

	return sprintf(buf, "raw temperature is %d\n", value/loop);
}

static ssize_t thsens_show_high_threshold(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct mmp2_thermal_sensor_device *thermal_dev = dev_get_drvdata(dev);

	return sprintf(buf, "high threshold is %d\n", thermal_dev->high_thresh);
}

static ssize_t thsens_store_high_threshold(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct mmp2_thermal_sensor_device *thermal_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &thermal_dev->high_thresh);
	printk("Set thermal sensor high threshold to: %d\n", thermal_dev->high_thresh);
	write_thermal_reg(thermal_dev->high_thresh & 0x000003FF, HI_THRESH_OFFSET);

	return size;
}

static ssize_t thsens_show_low_threshold(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct mmp2_thermal_sensor_device *thermal_dev = dev_get_drvdata(dev);

	return sprintf(buf, "low threshold is %d\n", thermal_dev->low_thresh);
}

static ssize_t thsens_store_low_threshold(struct device *dev,
			struct device_attribute *attr, const char *buf, size_t size)
{
	struct mmp2_thermal_sensor_device *thermal_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &thermal_dev->low_thresh);
	printk("Set thermal sensor low threshold to: %d\n", thermal_dev->low_thresh);
	write_thermal_reg(thermal_dev->low_thresh & 0x000003FF, LO_THRESH_OFFSET);

	return size;
}

static DEVICE_ATTR(thermal_raw_data, S_IRUGO, get_thermal_raw_data, NULL);
static DEVICE_ATTR(thermal_high_threshold, 0644,
		thsens_show_high_threshold, thsens_store_high_threshold);
static DEVICE_ATTR(thermal_low_threshold, 0644,
		thsens_show_low_threshold, thsens_store_low_threshold);

static struct attribute *mmp2_thermal_attrs[] = {
	&dev_attr_thermal_raw_data.attr,
	&dev_attr_thermal_high_threshold.attr,
	&dev_attr_thermal_low_threshold.attr,
	NULL
};

static struct attribute_group mmp2_thermal_attr_grp = {
	.attrs = mmp2_thermal_attrs,
};

static int __devinit mmp2_thermal_sensor_probe(struct platform_device *pdev)
{
	struct mmp2_thermal_sensor_device *dev = &g_mmp2_thermal_dev;
	struct resource *res;
	struct clk *clk;
	int ret = 0, size;
	unsigned int value;

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "unable to get thermal clock");
		return PTR_ERR(clk);
	}

	INIT_DELAYED_WORK(&dev->delayed_work, get_thermal_raw_temp);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = res->end - res->start + 1;
	dev->mmp2_thermal_regs_base = ioremap_nocache(res->start, size);
	if (dev->mmp2_thermal_regs_base == NULL) {
		dev_err(&pdev->dev, "failed to request register memory\n");
		return -ENOENT;
	}

	dev->clk = clk;
	clk_enable(dev->clk);

	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "no IRQ defined\n");
		goto err_get_plat_irq;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &mmp2_thermal_attr_grp);
	if (ret)
		goto err_free;

	dev->hwmon_dev = hwmon_device_register(&pdev->dev);
	if (!IS_ERR(dev->hwmon_dev)) {
		platform_set_drvdata(pdev, dev);

		init_timer(&dev->thermal_timer);
		dev->thermal_timer.function = &thermal_timer_handler;
		dev->high_temp_flag = 0;
		dev->low_temp_flag = 0;
		value = read_thermal_reg(ROOM_TEMP_OFFSET) | TS_EN;
		write_thermal_reg(value, ROOM_TEMP_OFFSET);
		dev->low_thresh = LO_THRESH;
		dev->high_thresh = HI_THRESH;
		write_thermal_reg(dev->low_thresh & 0x000003FF, LO_THRESH_OFFSET);
		write_thermal_reg(dev->high_thresh & 0x000003FF, HI_THRESH_OFFSET);

		ret = request_irq(dev->irq, mmp2_thermal_handler,
					IRQF_SHARED, "mmp2-thermal", dev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to request IRQ\n");
			ret = -EINVAL;
			goto err_request_irq;
		}
		enable_high_threshold_int();
		enable_low_threshold_int();

		return 0;
	}

	ret = PTR_ERR(dev->hwmon_dev);
	dev_err(&pdev->dev, "error registering hwmon device.\n");
	sysfs_remove_group(&pdev->dev.kobj, &mmp2_thermal_attr_grp);
err_request_irq:
	hwmon_device_unregister(dev->hwmon_dev);
err_free:
	iounmap(dev->mmp2_thermal_regs_base);
err_get_plat_irq:
	clk_disable(dev->clk);
	clk_put(dev->clk);
	return ret;
}

static int __devexit mmp2_thermal_sensor_remove(struct platform_device *pdev)
{
	struct mmp2_thermal_sensor_device *dev = platform_get_drvdata(pdev);

	free_irq(dev->irq, dev);
	del_timer(&dev->thermal_timer);
	cancel_delayed_work_sync(&dev->delayed_work);
	iounmap(dev->mmp2_thermal_regs_base);
	sysfs_remove_group(&pdev->dev.kobj, &mmp2_thermal_attr_grp);
	hwmon_device_unregister(dev->hwmon_dev);
	clk_disable(dev->clk);

	return 0;
}

static struct platform_driver mmp2_thermal_sensor_driver = {
	.driver		= {
		.name	= "mmp2-thermal",
	},
	.probe		= mmp2_thermal_sensor_probe,
	.remove		= __devexit_p(mmp2_thermal_sensor_remove)
};

static int __init mmp2_thermal_sensor_init(void)
{
	return platform_driver_register(&mmp2_thermal_sensor_driver);
}

static void __exit mmp2_thermal_sensor_exit(void)
{
	platform_driver_unregister(&mmp2_thermal_sensor_driver);
}

MODULE_AUTHOR("Mingliang Hu");
MODULE_DESCRIPTION("MMP2 thermal sensor driver");
MODULE_LICENSE("GPL");

module_init(mmp2_thermal_sensor_init);
module_exit(mmp2_thermal_sensor_exit);
