/*
 * A dummy driver for only control the GPIO when suspend and resume!
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <mach/mmp3_pm.h>
#include <plat/pm.h>

extern int wwan_enable(int);

unsigned long long cpu_clock_when_cut_modem_power;



/* Dummy device to control modem power before suspend and after resume */
//static struct platform_device *tlite2_modem_power_devices;


static struct platform_device tlite2_modem_power_devices = {
	.name = "modem_power_dev",
	.id = 0,
	.dev = {
		.platform_data = &wwan_enable,
	},
};


#ifdef CONFIG_PM
static struct workqueue_struct *my_wq;
static struct delayed_work mp_ws;
int additional_sleep_ms;

static int modem_power_pm_suspend(struct device *dev)
{
	unsigned long nanosec_rem;
	int this_cpu = smp_processor_id();
	
	/* 
	 * Don't know if it is possible that a delayed work is still waiting but we enter suspend again.
	 * But, it is always safe to cancel a work and flush the queue!
	 */
	cancel_delayed_work(&mp_ws);
	flush_workqueue(my_wq);
	
	wwan_enable(0);
	
	cpu_clock_when_cut_modem_power = cpu_clock(this_cpu);
	nanosec_rem = do_div(cpu_clock_when_cut_modem_power, 1000000000);
	printk("%s, current ticks = [%5lu.%06lu]\n", __func__, (unsigned long) cpu_clock_when_cut_modem_power,
						nanosec_rem / 1000);
	return 0;
}

static void my_wq_function( struct work_struct *work)
{
  //msleep(additional_sleep_ms);
  wwan_enable(1);
  return;
}

static int modem_power_pm_resume(struct device *dev)
{
	unsigned long nanosec_rem_after;
	unsigned long long cpu_clock_before_power_on_modem;
	int this_cpu = smp_processor_id();
	
	cpu_clock_before_power_on_modem = cpu_clock(this_cpu);
	nanosec_rem_after = do_div(cpu_clock_before_power_on_modem, 1000000000);
	printk("%s, current ticks = [%5lu.%06lu]\n", __func__, (unsigned long) cpu_clock_before_power_on_modem,
						nanosec_rem_after / 1000);
	/* The magic number 5 is 5 seconds for capastor discharging */
	if((additional_sleep_ms = (cpu_clock_before_power_on_modem - cpu_clock_when_cut_modem_power)) <= (unsigned long long)5) {
		// corner case, if device alive longer then 131072 seconds (1 day = 86500 seconds), the tick may wrap back to 0!!
		if (additional_sleep_ms < 0) {
			printk("Hit time wrap, force the delay to 5000 ms!");
			additional_sleep_ms = 5000; 
		}else{
			additional_sleep_ms = (5-additional_sleep_ms)*1000;
		}
		printk("Short sleep detected, additional sleep required is %d ms!", additional_sleep_ms);
		queue_delayed_work( my_wq, &mp_ws, msecs_to_jiffies(additional_sleep_ms)/*jiffies*/);
		
 	}else{
		wwan_enable(1);
	}
	return 0;
}
#if 0
static int modem_power_pm_poweroff(struct device *dev)
{
	printk("%s\r\n", __func__);
	return 0;
}

static int modem_power_pm_restore(struct device *dev)
{
	printk("%s\r\n", __func__);
	return 0;
}
#endif

static const struct dev_pm_ops modem_power_pm_ops = {
	.suspend	= modem_power_pm_suspend,
	.resume		= modem_power_pm_resume,
//	.poweroff	= modem_power_pm_poweroff,
//	.restore	= modem_power_pm_restore,
};
#endif

static int __devexit modem_power_dev_remove(struct platform_device *pdev)
{
	return 0;
}
static struct platform_driver modem_power_driver = {
	.driver		= {
		.name = "modem_power_dev",
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &modem_power_pm_ops,
#endif
	},
	.remove = __devexit_p(modem_power_dev_remove),
};

static int __init modem_power_probe(struct platform_device *dev)
{
	return 0;
}

static int __init toughpad_modem_power_init(void)
{	
	int ret = 0;
	printk(".....%s .....\n", __func__);

	ret = platform_device_register(&tlite2_modem_power_devices);
	if (ret)
		goto err;

	ret = platform_driver_probe(&modem_power_driver, modem_power_probe);
	if (ret)
		goto err;
	
	my_wq = create_workqueue("my_queue");
	if (my_wq) {
    /* Queue some work (item 1) */
    INIT_DELAYED_WORK(&mp_ws, my_wq_function );
    }
	
	return 0;
	
err:
	return -1;
}

module_init(toughpad_modem_power_init);
