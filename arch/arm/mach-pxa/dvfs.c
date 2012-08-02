/*
 *  linux/arch/arm/mach-pxa/dvfs.c
 *
 *  Author: Xiaoguang Chen chenxg@marvell.com
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/sysdev.h>
#include <linux/mfd/88pm80x.h>
#include <asm/delay.h>
#include <plat/clock.h>
#include <mach/pxa3xx-regs.h>
#include <mach/dvfs.h>
#include <mach/pxa9xx_pm_logger.h>	/* for pm debug tracing */
#include <mach/regs-ost.h>

static LIST_HEAD(dvfs_rail_list);
static DEFINE_MUTEX(dvfs_lock);
static ATOMIC_NOTIFIER_HEAD(dvfs_freq_notifier_list);
static struct i2c_client *i2c;
static int cur_volt3, cur_mem_volt;
static unsigned int last_level;

static int dvfs_rail_update(struct dvfs_rail *rail);

static unsigned long volt3_low, volt3_high;
static int buck1_voltage_get(char *s)
{
	char **str;
	if (!s)
		return 1;
	str = &s;
	volt3_low = simple_strtoul(s, str, 16);

	printk(KERN_INFO "Buck1 level3's low voltage "
	       "is set to 0x%lx\n", volt3_low);

	if (**str == ',')
		(*str)++;

	volt3_high = simple_strtoul(*str, str, 16);

	printk(KERN_INFO "Buck1 level3's high voltage "
	       "is set to 0x%lx\n", volt3_high);
	return 1;
}

__setup("MFVP=", buck1_voltage_get);

static inline int dvfs_solve_relationship(struct dvfs_relationship *rel)
{
	return rel->solve(rel->from, rel->to);
}

static inline int volt_to_reg(int millivolts)
{
	return (millivolts - 600) * 10 / 125;
}

static inline int reg_to_volt(int value)
{
	return  (value * 125 + 6000) / 10;
}

static int vcc_main_set_voltage(struct dvfs_rail *rail)
{
	unsigned int level = 0;
	int volts, newvolts;
	volts = rail->millivolts;
	newvolts = rail->new_millivolts;
	if (newvolts <= VOL_LEVL0)
		level = 0;
	else if (newvolts <= VOL_LEVL1)
		level = 1;
	else if (newvolts <= VOL_LEVL2)
		level = 2;
	else if (newvolts <= VOL_LEVL3_0) {
		if (volt3_low && (cur_volt3 != volt3_low)) {
			pm80x_reg_write(i2c, PM800_BUCK1_3, volt3_low);
			cur_volt3 = volt3_low;
		}
		level = 3;
	} else if (newvolts <= VOL_LEVL3_1) {
		if (volt3_high && (cur_volt3 != volt3_high)) {
			pm80x_reg_write(i2c, PM800_BUCK1_3, volt3_high);
			cur_volt3 = volt3_high;
		}
		level = 3;
	} else
		printk(KERN_ERR "Wrong voltage setting!\n");

	if (((volts <= VOL_LEVL1) && (level == 3)) ||
	    ((volts >= VOL_LEVL3_0) && (level <= 1))) {
		pxa978_set_voltage_level(2);
	}
	/*
	 * 1.2G/1.4G switch will change level 3's voltage
	 * but the level is not changed, we have to delay
	 * a while to make sure voltage value stable.
	 * In pmic spec, voltage change speed is 12.5mV/us
	 */
	if (level == 3 && last_level == 3)
		udelay(3);

	pxa978_set_voltage_level(level);
	last_level = level;
	pm_logger_app_add_trace(2, PM_VCC_MAIN_SET_VOL, OSCR4,
				volts, newvolts);

	return 0;
}

static int vcc_io_mem_set_voltage(struct dvfs_rail *rail)
{
	int newvolts = rail->new_millivolts;
	if (is_wkr_ddr533()) {
		if (newvolts <= VOL_MEM_LOW)
			newvolts = VOL_MEM_LOW;
		else if (newvolts <= VOL_MEM_HIGH)
			newvolts = VOL_MEM_HIGH;
		else
			printk(KERN_ERR "Invalid voltage for VCC_IO_MEM !\n");
		if (cur_mem_volt != newvolts) {
			pm80x_reg_write(i2c, PM800_BUCK3, newvolts);
			/*
			 * 12.5mV/us ramp rate, there are 4 steps
			 * between 1.25v and 1.2v(each step is 12.5mV)
			*/
			udelay(4);
		}
		cur_mem_volt = newvolts;
	}
	return 0;
}

/*
 * Sets the voltage on a dvfs rail to a specific value, and updates any
 * rails that depend on this rail.
 */
static int dvfs_rail_set_voltage(struct dvfs_rail *rail, int millivolts)
{
	int ret = 0;
	struct dvfs_relationship *rel;
	if (millivolts == rail->millivolts)
		return 0;
	rail->new_millivolts = millivolts;

	/*
	 * Before changing the voltage, tell each rail that depends
	 * on this rail that the voltage will change.
	 * This rail will be the "from" rail in the relationship,
	 * the rail that depends on this rail will be the "to" rail.
	 * from->millivolts will be the old voltage
	 * from->new_millivolts will be the new voltage
	 */
	list_for_each_entry(rel, &rail->relationships_to, to_node) {
		ret = dvfs_rail_update(rel->to);
		if (ret)
			return ret;
	}
	/* Currently we only have vcc main rail, if other rails are added
	 * need to add other rails' set voltage function here.
	 */
	if (!strcmp(rail->reg_id, "vcc_main"))
		ret = vcc_main_set_voltage(rail);
	else if (!strcmp(rail->reg_id, "vcc_io_mem"))
		ret = vcc_io_mem_set_voltage(rail);

	rail->millivolts = rail->new_millivolts;

	/*
	 * After changing the voltage, tell each rail that depends
	 * on this rail that the voltage has changed.
	 * from->millivolts and from->new_millivolts will be the
	 * new voltage
	 */
	list_for_each_entry(rel, &rail->relationships_to, to_node) {
		ret = dvfs_rail_update(rel->to);
		if (ret)
			return ret;
	}

	if (unlikely(rail->millivolts != millivolts)) {
		pr_err("%s: rail didn't reach target %d  (%d)\n",
		       __func__, millivolts, rail->millivolts);
		return -EINVAL;
	}

	return ret;
}

/*
 * Determine the minimum valid voltage for a rail, taking into account
 * the dvfs clocks and any rails that this rail depends on.  Calls
 * dvfs_rail_set_voltage with the new voltage, which will call
 * dvfs_rail_update on any rails that depend on this rail.
 */
static int dvfs_rail_update(struct dvfs_rail *rail)
{
	int millivolts = 0;
	struct dvfs *d;
	struct dvfs_relationship *rel;
	int ret = 0;

	/* Find the maximum voltage requested by any clock */
	list_for_each_entry(d, &rail->dvfs, dvfs_node)
	    millivolts = max(d->millivolts, millivolts);

	if (millivolts != 0)
		rail->new_millivolts = millivolts;

	/* Check any rails that this rail depends on */
	list_for_each_entry(rel, &rail->relationships_from, from_node)
	    rail->new_millivolts = dvfs_solve_relationship(rel);

	if (rail->new_millivolts != rail->millivolts)
		ret = dvfs_rail_set_voltage(rail, rail->new_millivolts);

	return ret;
}

int set_dvfs_rate(struct dvfs *d, unsigned long rate)
{
	int i = 0;
	int ret;

	if (d->vol_freq_table == NULL)
		return -ENODEV;

	if (rate > d->vol_freq_table[d->num_freqs - 1].freq) {
		pr_warn("dvfs: rate %lu too high for dvfs\n", rate);
		return -EINVAL;
	}

	mutex_lock(&dvfs_lock);
	if (rate == 0)
		d->millivolts = 0;
	else {
		while (i < d->num_freqs && rate > d->vol_freq_table[i].freq)
			i++;

		d->millivolts = d->vol_freq_table[i].millivolts;
	}

	pr_debug("set voltage to %d from %s for rate %lu.\n",
			d->millivolts, d->clk_name, rate);

	ret = dvfs_rail_update(d->dvfs_rail);
	if (ret)
		pr_err("Failed to set voltage to %d mV\n", d->millivolts);

	mutex_unlock(&dvfs_lock);

	return ret;
}
EXPORT_SYMBOL(set_dvfs_rate);

int dvfs_notifier_frequency(struct dvfs_freqs *freqs, unsigned int state)
{
	int ret;

	switch (state) {
	case DVFS_FREQ_PRECHANGE:
		ret = atomic_notifier_call_chain(&dvfs_freq_notifier_list,
						 DVFS_FREQ_PRECHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver before "
				 "switching frequency\n");
		break;
	case DVFS_FREQ_POSTCHANGE:
		ret = atomic_notifier_call_chain(&dvfs_freq_notifier_list,
						 DVFS_FREQ_POSTCHANGE, freqs);
		if (ret != NOTIFY_DONE)
			pr_debug("Failure in device driver after "
				 "switching frequency\n");
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfs_notifier_frequency);

int dvfs_register_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFS_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_register
		    (&dvfs_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfs_register_notifier);

int dvfs_unregister_notifier(struct notifier_block *nb, unsigned int list)
{
	int ret;

	switch (list) {
	case DVFS_FREQUENCY_NOTIFIER:
		ret = atomic_notifier_chain_unregister
		    (&dvfs_freq_notifier_list, nb);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(dvfs_unregister_notifier);

static inline ssize_t voltage_show(struct sys_device *sys_dev,
				   struct sysdev_attribute *attr,
				   char *buf)
{
	int level, volt, len = 0;

	level = (AVLSR >> 1) & 0x3;
	volt = reg_to_volt(pm80x_reg_read(i2c, PM800_BUCK1 + level));
	len += sprintf(buf + len, "VCC_MAIN:\tLevel %d (%d mV)\n", level, volt);
	volt = reg_to_volt(pm80x_reg_read(i2c, PM800_BUCK3));
	len += sprintf(buf + len, "VCC_IO_MEM:\t%d mV\n", volt);

	return len;
}

static inline ssize_t voltage_store(struct sys_device *sys_dev,
				    struct sysdev_attribute *attr,
				    const char *buf, size_t len)
{
	int new_vol;

	sscanf(buf, "%u", &new_vol);
	if ((new_vol > 4) || (new_vol < 0)) {
		printk(KERN_ERR "Wrong voltage level! "
		       "Must between 0 ~ 4\n(3 means level 3 low,"
		       " 4 means level3 high)\n");
		return len;
	}

	if ((new_vol == 3) && volt3_low && (cur_volt3 != volt3_low)) {
		pm80x_reg_write(i2c, PM800_BUCK1_3, volt3_low);
		cur_volt3 = volt3_low;
	}

	if (new_vol == 4) {
		if (volt3_high && (cur_volt3 != volt3_high)) {
			pm80x_reg_write(i2c, PM800_BUCK1_3, volt3_high);
			cur_volt3 = volt3_high;
		}
		new_vol = 3;
	}
	pxa978_set_voltage_level(new_vol);
	return len;
}

SYSDEV_ATTR(voltage, 0644, voltage_show, voltage_store);

static struct attribute *dvfs_attr[] = {
	&attr_voltage.attr,
};

static int dvfs_add(struct sys_device *sys_dev)
{
	int i, n;
	int ret;

	n = ARRAY_SIZE(dvfs_attr);
	for (i = 0; i < n; i++) {
		ret = sysfs_create_file(&(sys_dev->kobj), dvfs_attr[i]);
		if (ret)
			return -EIO;
	}
	return 0;
}

static int dvfs_rm(struct sys_device *sys_dev)
{
	int i, n;
	n = ARRAY_SIZE(dvfs_attr);
	for (i = 0; i < n; i++)
		sysfs_remove_file(&(sys_dev->kobj), dvfs_attr[i]);
	return 0;
}

static struct sysdev_driver dvfs_sysdev_driver = {
	.add = dvfs_add,
	.remove = dvfs_rm,
};

static int __devinit dvfs_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct pm80x_chip *chip = dev_get_drvdata(pdev->dev.parent);
	i2c = chip->power_page;
	ret = sysdev_driver_register(&cpu_sysdev_class, &dvfs_sysdev_driver);

	return 0;
}

static struct platform_driver dvfs_driver = {
	.driver = {
		   .name = "dvc",
		   .owner = THIS_MODULE,
		   },
	.probe = dvfs_probe,
};

int dvfs_init(void)
{
	struct dvfs_rail *rail;

	mutex_lock(&dvfs_lock);

	list_for_each_entry(rail, &dvfs_rail_list, node)
	    dvfs_rail_update(rail);

	last_level = (AVLSR >> 1) & 0x3;

	mutex_unlock(&dvfs_lock);
	return platform_driver_register(&dvfs_driver);

	return 0;
}
