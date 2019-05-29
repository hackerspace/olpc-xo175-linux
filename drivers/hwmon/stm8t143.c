#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/mutex.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <mach/mfp-mmp2.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <linux/delay.h>
#include <linux/proc_fs.h> // for proc_dir_entry 
#include <asm/uaccess.h> //for copy_from_user
#include <linux/timer.h> //for struct timer_list
#include <linux/random.h> //for get_random_bytes
#include <linux/earlysuspend.h> //for early suspend function
#include <linux/regulator/machine.h>

struct stm8t143_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	int irq;
	struct work_struct work;
        struct input_dev *g_ps; 
        struct timer_list g_timer_ps;
        unsigned char g_ps_user_count;
        unsigned char g_ps_state;
        unsigned char g_irq_pout_triggered;
        unsigned char g_irq_tout_triggered;
        unsigned int g_ps_sample_interval;
        //struct proc_dir_entry *atte_proc_file;
        int g_atte_status;
        unsigned int g_irq_rf_psen_pout;
        unsigned int g_irq_rf_psen_tout;
};
static struct stm8t143_data *gp_stm8t143_dev = NULL;
static struct work_struct g_stm8t143_ps_work;
static struct work_struct g_stm8t143_int_work;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend g_stm8t143_early_suspend;
#endif //CONFIG_HAS_EARLYSUSPEND

// for RF ATTE
static int g_atte1_status = 0,
           g_atte2_status = 0,
           g_atte3_status = 0;

//No need any more
#if 0           
static void atte1_set_command(int cmd);
static void atte2_set_command(int cmd);
#endif
#if 1 //T-Lite22, 
// Declarations

static void stm8t143_3v3_power(unsigned int on);
static void stm8t143_power_enable(int enable);

static void 
stm8t143_get_ps(unsigned char *data)
{
        unsigned char /*val = *data,*/
        #if 1
                     val_new = 0;
           if (gp_stm8t143_dev->g_irq_pout_triggered || gp_stm8t143_dev->g_irq_tout_triggered)
                     val_new = 0;
           else
                     val_new = 205;
        #else
                     val_random_raw = 0,
                     val_random = 0,
                     val_new = 0;              

        get_random_bytes(&val_random_raw, sizeof(val_random_raw));
        val_random = val_random_raw % 50;
        val_new = (gp_stm8t143_dev->g_irq_pout_triggered || gp_stm8t143_dev->g_irq_tout_triggered)? val_random : val_random + 205;
        #endif
        /*
        printk("%s: val_new = %d , pout_triggered = %d, tout_triggered = %d.\r\n", __func__, \
                                                                                                 val_new,\
                                                                                                 gp_stm8t143_dev->g_irq_pout_triggered, \
                                                                                                 gp_stm8t143_dev->g_irq_tout_triggered);
        */
        
        *data = val_new;
}

static void
stm8t143_report_ps_value(struct work_struct *work)
{
        unsigned char tmp = 0;
        //printk("%s: work = %p.\r\n", __func__, work);
        
        stm8t143_get_ps(&tmp);
        //printk("%s: report data = %d.\r\n", __func__, tmp);
        //printk("%s: input_report_abs(0x%p, 0x%x, %d).\r\n", __func__, gp_stm8t143_dev->g_ps, ABS_DISTANCE, tmp);
        input_report_abs(gp_stm8t143_dev->g_ps, ABS_DISTANCE, tmp);
        input_sync(gp_stm8t143_dev->g_ps);
}

static int 
stm8t143_ps_register_device(struct switch_dev *sdev)
{
        int err = 0;
           
        printk("%s: sdev = %p.\r\n", __func__, sdev);
        gp_stm8t143_dev->g_ps = input_allocate_device();
        gp_stm8t143_dev->g_ps->name       = "STM8T143 Proximity Sensor";
        gp_stm8t143_dev->g_ps->phys       = "Proximity-sensor/input0";
        gp_stm8t143_dev->g_ps->id.bustype = BUS_VIRTUAL; // STM8T143 has no bus, so asign bustype as "Virtual"
        gp_stm8t143_dev->g_ps->id.vendor  = 0;
        gp_stm8t143_dev->g_ps->dev.parent = sdev->dev;

        __set_bit(EV_ABS, gp_stm8t143_dev->g_ps->evbit);
        __set_bit(ABS_DISTANCE, gp_stm8t143_dev->g_ps->absbit);
        __set_bit(ABS_MISC, gp_stm8t143_dev->g_ps->absbit);   
        __set_bit(EV_SYN, gp_stm8t143_dev->g_ps->evbit);      
        input_set_abs_params(gp_stm8t143_dev->g_ps, ABS_DISTANCE, 0, 255, 0, 0);
        input_set_abs_params(gp_stm8t143_dev->g_ps, ABS_MISC, -100, 100, 0, 0);
        
        printk("%s: [1] gp_stm8t143_dev->g_ps = %p\r\n", __func__, gp_stm8t143_dev->g_ps);                                                                                                   
        err = input_register_device(gp_stm8t143_dev->g_ps);
        if (err) 
        {
                dev_err(sdev->dev, "regist input driver error\n");
                input_free_device(gp_stm8t143_dev->g_ps);
                gp_stm8t143_dev->g_ps = NULL;
        }
        printk("%s: [2] gp_stm8t143_dev->g_ps = %p\r\n", __func__, gp_stm8t143_dev->g_ps);
        
        return err;
}

static int 
stm8t143_set_ps_interval(int val)
{
        printk("%s: val = %d.\r\n", __func__,val);
        
        if(val < 100)
                val = 100;
        else if(val > 800)
                val = 800;
        
        gp_stm8t143_dev->g_ps_sample_interval = val;
        return 0;
}

/* Poll the proximity sensor */
static void 
stm8t143_ps_timer_func(unsigned long data)
{
        //printk(KERN_ERR "%s: data = %d.\r\n", __func__, data);
        schedule_work(&g_stm8t143_ps_work);
        mod_timer(&gp_stm8t143_dev->g_timer_ps, jiffies + msecs_to_jiffies(gp_stm8t143_dev->g_ps_sample_interval));
}

static int 
active_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret = 0;
        //printk(KERN_ERR "%s()\r\n", __func__);
        ret = sprintf(buf, "%d\n", gp_stm8t143_dev->g_ps_state);
        return ret;
}

static int 
active_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        int enable = strcmp(buf, "1\n")? 0 : 1;
        //printk(KERN_ERR "%s: enable = %d, user_count = %d.\r\n", __func__, enable, gp_stm8t143_dev->g_ps_user_count);
        
        if(enable)
        {
                if(gp_stm8t143_dev->g_ps_user_count == 0)
                {
                        //printk(KERN_ERR "%s: Enable STM8T143!\r\n", __func__);
                        
                        /* Power-up STM8T143 Here */
                        stm8t143_power_enable(1);
                        
                        gp_stm8t143_dev->g_irq_pout_triggered = 0;
                        gp_stm8t143_dev->g_irq_tout_triggered = 0;
                        
                        /* Enable timer function to start polling */
                        setup_timer(&gp_stm8t143_dev->g_timer_ps, stm8t143_ps_timer_func, 0);
                        mod_timer(&gp_stm8t143_dev->g_timer_ps, jiffies + msecs_to_jiffies(gp_stm8t143_dev->g_ps_sample_interval));
                        
                        gp_stm8t143_dev->g_ps_state = 1;
                        dev_info(dev, "status on\n");
                }
                else
                {
                        printk(KERN_ERR "%s: STM8T143 has been enabled!\r\n", __func__);
                }
                
                gp_stm8t143_dev->g_ps_user_count++;
                //printk(KERN_ERR "%s: User count = %d!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count);
        }
        else if(gp_stm8t143_dev->g_ps_user_count > 0) /* Set disable and has been set enable */
        {
                if(gp_stm8t143_dev->g_ps_user_count == 1)
                {
                        //printk(KERN_ERR "%s: Disable stm8t143!\r\n", __func__);
                        
                        /* Disable timer function to stop polling */
                        del_timer_sync(&gp_stm8t143_dev->g_timer_ps);
                        flush_work(&g_stm8t143_ps_work);
                        
                        /* Power-down STM8T143 Here */
                        stm8t143_power_enable(0);
                        
                        gp_stm8t143_dev->g_ps_state = 0;
                        dev_info(dev, "status off\n");
                }
                else
                {
                        printk(KERN_ERR "%s: STM8T143 still has users (%d)!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count - 1);
                }
                
                gp_stm8t143_dev->g_ps_user_count--;
                //printk(KERN_ERR "%s: User count = %d!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count);
        }
        
        return count;
}

static int 
interval_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret = 0;
        //printk(KERN_ERR "%s()\r\n", __func__);
        ret = sprintf(buf, "%d\n", gp_stm8t143_dev->g_ps_sample_interval);
        return ret;
}

static int 
interval_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        unsigned int val = 0;
        char msg[13] = "\n";
        //printk(KERN_ERR "%s()\r\n", __func__);
        
        if(count > (sizeof(msg) - 1))
                count = sizeof(msg) - 1;
        memcpy(msg, buf, count);
        msg[count] = '\0';
        
        val = (unsigned int)simple_strtoul(msg, NULL, 10);
        //printk(KERN_ERR "%s: val = %d.\r\n", __func__, val);
        stm8t143_set_ps_interval(val);        
        dev_info(dev, "stm8t143 proximity sensor sample interval is %d ms\n", gp_stm8t143_dev->g_ps_sample_interval);
                
        return count;
}
                
static DEVICE_ATTR(active, 644, active_show, active_set);
static DEVICE_ATTR(interval, 644, interval_show, interval_set);

static struct attribute *stm8t143_attributes[] = 
{
        &dev_attr_active.attr,
        &dev_attr_interval.attr,
        NULL
};

static struct attribute_group stm8t143_attribute_group = 
{
        .attrs = stm8t143_attributes
};

static int
stm8t143_add_fs(struct device *device)
{
        return sysfs_create_group(&device->kobj, &stm8t143_attribute_group);
}

static void
stm8t143_workq_func(struct work_struct *work)
{
        //printk(KERN_ERR "%s()\r\n", __func__);

        if(g_atte1_status != 0)
        {
                //atte1_set_command(0);
                g_atte1_status = 0;
        }
        
        if(g_atte2_status != 0)
        {
                //atte2_set_command(0);
                g_atte2_status = 0;
        }                                                                
}
#endif //T-Lite2
//No need any more
#if 0           

static void atte1_set_command(int cmd)
{
        int rf_sar_ctl1 = mfp_to_gpio(GPIO8_GPIO), /* RF_SAR_CTL1 */
            rf_sar_ctl2 = mfp_to_gpio(GPIO9_GPIO), /* RF_SAR_CTL2 */
            rf_sar_ctl6 = mfp_to_gpio(GPIO13_GPIO); /* RF_SAR_CTL6 */      
        //printk(KERN_ERR "%s(%d)\r\n", __func__, cmd);
        
        if(gpio_request(rf_sar_ctl1, "rf_sar_ctl1"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl1 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl1);
                goto fail_req_1;
        }
        if(gpio_request(rf_sar_ctl2, "rf_sar_ctl2"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl2 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl2);
                goto fail_req_2;
        }
        if(gpio_request(rf_sar_ctl6, "rf_sar_ctl6"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl6 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl6);
                goto fail_req_6;
        }
        
        // Control GPIOs with input commands
        switch(cmd)
        {
                case 0: // Command 0 => CTL1-Low, CTL2-High, CTL6-High
                        printk(KERN_ERR "%s: Command 0.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl1, 0);
                        gpio_direction_output(rf_sar_ctl2, 1);
                        gpio_direction_output(rf_sar_ctl6, 1);
                        break;
                        
                case 3: // Command 3 => CTL1-High, CTL2-Low, CTL6-Low
                        printk(KERN_ERR "%s: Command 3.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl1, 1);
                        gpio_direction_output(rf_sar_ctl2, 0);
                        gpio_direction_output(rf_sar_ctl6, 0);
                        break;
                        
                case 5: // Command 5 => CTL1-Low, CTL2-Low, CTL6-High
                        printk(KERN_ERR "%s: Command 5.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl1, 0);
                        gpio_direction_output(rf_sar_ctl2, 0);
                        gpio_direction_output(rf_sar_ctl6, 1);
                        break;
                        
                case 8: // Command 8 => CTL1-High, CTL2-High, CTL6-Low
                        printk(KERN_ERR "%s: Command 8.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl1, 1);
                        gpio_direction_output(rf_sar_ctl2, 1);
                        gpio_direction_output(rf_sar_ctl6, 0);
                        break;
                        
                default: // Unknown command
                        printk(KERN_ERR "%s: Unknown command.\r\n", __func__);
                        break;
        }

        // Normal exit
        gpio_free(rf_sar_ctl1);
        gpio_free(rf_sar_ctl2);
        gpio_free(rf_sar_ctl6);
        //printk(KERN_ERR "%s: over.\r\n", __func__);
        return;
        
fail_req_6:
        gpio_free(rf_sar_ctl2);        
fail_req_2:
        gpio_free(rf_sar_ctl1);
fail_req_1:
        printk(KERN_ERR "%s: error exit.\r\n", __func__);         
}

static void atte2_set_command(int cmd)
{
        int rf_sar_ctl3 = mfp_to_gpio(GPIO10_GPIO), /* RF_SAR_CTL3 */
            rf_sar_ctl4 = mfp_to_gpio(GPIO11_GPIO), /* RF_SAR_CTL4 */
            rf_sar_ctl5 = mfp_to_gpio(GPIO12_GPIO); /* RF_SAR_CTL5 */
        //printk(KERN_ERR "%s(%d)\r\n", __func__, cmd);
        
        if(gpio_request(rf_sar_ctl3, "rf_sar_ctl3"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl3 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl3);
                goto fail_req_3;
        }
        if(gpio_request(rf_sar_ctl4, "rf_sar_ctl4"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl4 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl4);
                goto fail_req_4;
        }
        if(gpio_request(rf_sar_ctl5, "rf_sar_ctl5"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl5 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl5);
                goto fail_req_5;
        }
        
        // Control GPIOs with input commands
        switch(cmd)
        {
                case 0: // Command 0 => CTL3-Low, CTL4-High, CTL5-High   
                        printk(KERN_ERR "%s: Command 0.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl3, 0);
                        gpio_direction_output(rf_sar_ctl4, 1);
                        gpio_direction_output(rf_sar_ctl5, 1);
                        break;
                        
                case 3: // Command 3 => CTL3-High, CTL4-Low, CTL5-Low
                        printk(KERN_ERR "%s: Command 3.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl3, 1);
                        gpio_direction_output(rf_sar_ctl4, 0);
                        gpio_direction_output(rf_sar_ctl5, 0);
                        break;
                
                case 5: // Command 5 => CTL3-Low, CTL4-Low, CTL5-High
                        printk(KERN_ERR "%s: Command 5.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl3, 0);
                        gpio_direction_output(rf_sar_ctl4, 0);
                        gpio_direction_output(rf_sar_ctl5, 1);
                        break;
                        
                case 8: // Command 8 => CTL3-High, CTL4-High, CTL5-Low
                        printk(KERN_ERR "%s: Command 8.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl3, 1);
                        gpio_direction_output(rf_sar_ctl4, 1);
                        gpio_direction_output(rf_sar_ctl5, 0);
                        break;
                
                default: // Unknown command
                         printk(KERN_ERR "%s: Unknown command.\r\n", __func__);
                         break;
        }

        // Normal exit
        gpio_free(rf_sar_ctl3);
        gpio_free(rf_sar_ctl4);
        gpio_free(rf_sar_ctl5);
        //printk(KERN_ERR "%s: over.\r\n", __func__);
        return;
        
fail_req_5:
        gpio_free(rf_sar_ctl4);
fail_req_4:
        gpio_free(rf_sar_ctl3);
fail_req_3:
        printk(KERN_ERR "%s: error exit.\r\n", __func__);
}

static void atte3_set_command(int cmd)
{
        int rf_sar_ctl7 = mfp_to_gpio(GPIO21_GPIO), /* RF_SAR_CTL7 */
            rf_sar_ctl8 = mfp_to_gpio(GPIO22_GPIO), /* RF_SAR_CTL8 */
            rf_sar_ctl9 = mfp_to_gpio(GPIO16_GPIO); /* RF_SAR_CTL9 */
            //printk(KERN_ERR "%s(%d)\r\n", __func__, cmd);
        
        if(gpio_request(rf_sar_ctl7, "rf_sar_ctl7"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl7 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl7);
                goto fail_req_7;
        }
        if(gpio_request(rf_sar_ctl8, "rf_sar_ctl8"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl8 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl8);
                goto fail_req_8;
        }
        if(gpio_request(rf_sar_ctl9, "rf_sar_ctl9"))
        {
                printk(KERN_ERR "%s: request rf_sar_ctl9 (gpio %d) failed!.\r\n", __func__, rf_sar_ctl9);
                goto fail_req_9;
        }
        
        // Control GPIOs with input commands
        switch(cmd)
        {
                case 0: // Command 0 => CTL7-Low, CTL8-High, CTL9-High 
                        printk(KERN_ERR "%s: Command 0.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl7, 0);
                        gpio_direction_output(rf_sar_ctl8, 1);
                        gpio_direction_output(rf_sar_ctl9, 1);
                        break;
                case 3: // Command 3 => CTL7-High, CTL8-Low, CTL9-Low
                        printk(KERN_ERR "%s: Command 3.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl7, 1);
                        gpio_direction_output(rf_sar_ctl8, 0);
                        gpio_direction_output(rf_sar_ctl9, 0); 
                        break;
                case 5: // Command 5 => CTL7-Low, CTL8-Low, CTL9-High 
                        printk(KERN_ERR "%s: Command 5.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl7, 0);
                        gpio_direction_output(rf_sar_ctl8, 0);
                        gpio_direction_output(rf_sar_ctl9, 1);
                        break;
                case 8: // Command 8 => CTL7-High, CTL8-High, CTL9-Low 
                        printk(KERN_ERR "%s: Command 8.\r\n", __func__);
                        gpio_direction_output(rf_sar_ctl7, 1);
                        gpio_direction_output(rf_sar_ctl8, 1);
                        gpio_direction_output(rf_sar_ctl9, 0);
                        break;
                default: // Unknown command
                        printk(KERN_ERR "%s: Unknown command.\r\n", __func__);
                        break;
        }
        
        // Normal exit
        gpio_free(rf_sar_ctl7);
        gpio_free(rf_sar_ctl8);
        gpio_free(rf_sar_ctl9);
        //printk(KERN_ERR "%s: over.\r\n", __func__);
        return;

fail_req_9:
        gpio_free(rf_sar_ctl8);
fail_req_8:
        gpio_free(rf_sar_ctl7);
fail_req_7:
        printk(KERN_ERR "%s: error exit.\r\n", __func__);
}
#endif

#ifdef CONFIG_PROC_FS
static ssize_t 
atte1_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
                        
        len = sprintf(page, "%d\n", g_atte1_status);
        //printk(KERN_ERR "%s: atte1_status = %d.\r\n", __func__, g_atte1_status);
        
        return len;
}

static ssize_t
atte2_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
                            
        len = sprintf(page, "%d\n", g_atte2_status);
        //printk(KERN_ERR "%s: atte2_status = %d.\r\n", __func__, g_atte2_status);
                                             
        return len;
}

static ssize_t
atte3_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
        
        len = sprintf(page, "%d\n", g_atte3_status);
        //printk(KERN_ERR "%s: atte3_status = %d.\r\n", __func__, g_atte3_status);
                                                                   
        return len;
}

static ssize_t 
atte1_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
        char messages[10];
        unsigned int atte1_cmd = 0; 
        //printk(KERN_ERR "%s()\r\n", __func__);
        memset(messages, 0, 10);
        
        //printk(KERN_ERR "len = %d.\r\n", __func__, len);
        if(len > 10)
                len = 10;
                
        if(copy_from_user(messages, buff, len))
        {
                printk(KERN_ERR "%s: copy_from_user failed!.\r\n", __func__);
                return -EFAULT;        
        }
        
        atte1_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);
        printk(KERN_ERR "%s: atte1_cmd = %d.\r\n", __func__, atte1_cmd);
        switch(atte1_cmd)
        {
                case 0: 
                case 3: 
                case 5: 
                case 8: // control atte circit. 
                        //printk(KERN_ERR "%s: command %d\r\n", __func__, atte1_cmd);
                        //atte1_set_command(atte1_cmd);
                        g_atte1_status = atte1_cmd; 
                        break;
                         
                default: //Unkown command, print error message and return
                         printk(KERN_ERR "%s: unkown command (%d)!\r\n", __func__, atte1_cmd);
                         break;
        }
        
        return len;    
}


static ssize_t 
atte2_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
        char messages[10];
        unsigned int atte2_cmd = 0; 
        //printk(KERN_ERR "%s()\r\n", __func__);
        memset(messages, 0, 10);
        
        //printk(KERN_ERR "len = %d.\r\n", __func__, len);
        if(len > 10)
                len = 10;
        
        if(copy_from_user(messages, buff, len))
        {
                printk(KERN_ERR "%s: copy_from_user failed!.\r\n", __func__);
                return -EFAULT;
        }
        
        atte2_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);  
        printk(KERN_ERR "%s: atte2_cmd = %d.\r\n", __func__, atte2_cmd);
        switch(atte2_cmd)
        {
                case 0: 
                case 3: 
                case 5: 
                case 8: // control atte circit. 
                        //printk(KERN_ERR "%s: command %d\r\n", __func__, atte2_cmd);
                        //atte2_set_command(atte2_cmd);
                        g_atte2_status = atte2_cmd;
                        break;
                                                                                                                                                                                                 
                default: //Unkown command, print error message and return
                         printk(KERN_ERR "%s: unkown command (%d)!\r\n", __func__, atte2_cmd);
                         break;
        }
        
        return len;                                                    
}

static ssize_t
atte3_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
        char messages[10];
        unsigned int atte3_cmd = 0; 
        //printk(KERN_ERR "%s()\r\n", __func__);
        memset(messages, 0, 10);
        
        //printk(KERN_ERR "len = %d.\r\n", __func__, len);
        if(len > 10)
                len = 10;
                
        if(copy_from_user(messages, buff, len))
        {
                printk(KERN_ERR "%s: copy_from_user failed!.\r\n", __func__);
                return -EFAULT;
        }
        
        atte3_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);  
        printk(KERN_ERR "%s: atte3_cmd = %d.\r\n", __func__, atte3_cmd);
        switch(atte3_cmd)
        {
                case 0: 
                case 3: 
                case 5: 
                case 8: // control atte circit. 
                        //printk(KERN_ERR "%s: command %d\r\n", __func__, atte3_cmd);
                        //atte3_set_command(atte3_cmd);                    
                        g_atte3_status = atte3_cmd;
                        break;
                                      
                default: //Unkown command, print error message and return
                         printk(KERN_ERR "%s: unkown command (%d)!\r\n", __func__, atte3_cmd);
                         break;                                                                                                                                                break;
        }
        
        return len;
}                                       
static void create_atte1_proc_file(void)
{
        //printk(KERN_ERR "%s()\r\n", __func__);
        struct proc_dir_entry *atte1_proc_file = create_proc_entry("driver/atte1", 0666, NULL);        
        printk(KERN_ERR "%s: atte1_proc_file = %p.\r\n", __func__, atte1_proc_file);
        if(!atte1_proc_file)
        {
                printk(KERN_ERR "%s: fail to create proc file!\r\n", __func__);
                return;
        }
        
        atte1_proc_file->read_proc = (read_proc_t *)atte1_read_proc;
        atte1_proc_file->write_proc = (write_proc_t *)atte1_write_proc;
        g_atte1_status = 0;
        printk(KERN_ERR "%s: read_proc = %p, write_proc = %p, status = %d.\r\n", __func__, atte1_proc_file->read_proc,    \
                                                                                atte1_proc_file->write_proc, \
                                                                                g_atte1_status);
}

static void create_atte2_proc_file(void)
{
        //printk(KERN_ERR "%s()\r\n", __func__);
        struct proc_dir_entry *atte2_proc_file = create_proc_entry("driver/atte2", 0666, NULL);
        printk(KERN_ERR "%s: atte2_proc_file = %p.\r\n", __func__, atte2_proc_file);
        if(!atte2_proc_file)
        {
                printk(KERN_ERR "%s: fail to create proc file!\r\n", __func__);
                return;
        }
                                                                                         
        atte2_proc_file->read_proc = (read_proc_t *)atte2_read_proc;
        atte2_proc_file->write_proc = (write_proc_t *)atte2_write_proc;
        g_atte2_status = 0;
        printk(KERN_ERR "%s: read_proc = %p, write_proc = %p, status = %d.\r\n", __func__, atte2_proc_file->read_proc,    \
                                                                                           atte2_proc_file->write_proc, \
                                                                                           g_atte2_status);
}

static void create_atte3_proc_file(void)
{
        //printk(KERN_ERR "%s()\r\n", __func__);
        struct proc_dir_entry *atte3_proc_file = create_proc_entry("driver/atte3", 0666, NULL);
        printk(KERN_ERR "%s: atte3_proc_file = %p.\r\n", __func__, atte3_proc_file);
        if(!atte3_proc_file)
        {
                printk(KERN_ERR "%s: fail to create proc file!\r\n", __func__);
                return;
        }
        
        atte3_proc_file->read_proc = (read_proc_t *)atte3_read_proc;
        atte3_proc_file->write_proc = (write_proc_t *)atte3_write_proc;
        g_atte3_status = 0;
        printk(KERN_ERR "%s: read_proc = %p, write_proc = %p, status = %d.\r\n", __func__, atte3_proc_file->read_proc,    \
                                                                                           atte3_proc_file->write_proc, \  
                                                                                           g_atte3_status);
}
#endif //CONFIG_PROC_FS

// Power enable function for STM8T143 Cap-sensor
static irqreturn_t rf_psen_pout_isr(int irq, void *dev_instance)
{
        int rf_psen_pout = mfp_to_gpio(GPIO106_GPIO), /* RF_PSEN_POUT */
            rf_psen_pout_status = gpio_get_value(rf_psen_pout);
        //pr_info("%s: irq trigger!\n", __func__);

        if(!gp_stm8t143_dev->g_irq_pout_triggered && !rf_psen_pout_status) /* Trigger falling edge such that the current status should be LOW */
        {
                /*
                pr_info("%s: g_irq_pout_triggered! (previous_trigger = %d, int_status = %d)\n", __func__, \
                                                                                                gp_stm8t143_dev->g_irq_pout_triggered, \
                                                                                                rf_psen_pout_status);
                */
                //pr_info("%s: g_irq_pout_triggered!\n", __func__);
                gp_stm8t143_dev->g_irq_pout_triggered = 1;
                
                schedule_work(&g_stm8t143_int_work);
        }
        else
        {
                /*
                pr_info("%s: g_irq_pout_triggered cancel! (previous_trigger = %d, int_status = %d)\n", __func__, \
                                                                                                       gp_stm8t143_dev->g_irq_pout_triggered, \
                                                                                                       rf_psen_pout_status);
                */
                //pr_info("%s: g_irq_pout_triggered cancel!\n", __func__);
                gp_stm8t143_dev->g_irq_pout_triggered = 0;
        }
        
        return IRQ_HANDLED;
}

static irqreturn_t rf_psen_tout_isr(int irq, void *dev_instance)
{
        int rf_psen_tout = mfp_to_gpio(GPIO107_GPIO), /* RF_PSEN_TOUT */
            rf_psen_tout_status = gpio_get_value(rf_psen_tout);
        //pr_info("%s: irq trigger!\n", __func__);
		
        if(!gp_stm8t143_dev->g_irq_tout_triggered && !rf_psen_tout_status) /* Trigger falling edge such that the current status should be LOW */
        {
                /*
                pr_info("%s: g_irq_tout_triggered! (previous_trigger = %d, int_status = %d)\n", __func__, \
                                                                                                gp_stm8t143_dev->g_irq_tout_triggered, \
                                                                                                rf_psen_tout_status);        
                */
                pr_info("%s: g_irq_tout_triggered!\n", __func__);
                gp_stm8t143_dev->g_irq_tout_triggered = 1;
                
                schedule_work(&g_stm8t143_int_work);
        }
        else
        {
                /*
                pr_info("%s: g_irq_tout_triggered cancel! (previous_trigger = %d, int_status = %d)\n", __func__, \
                                                                                                       gp_stm8t143_dev->g_irq_tout_triggered, \
                                                                                                       rf_psen_tout_status);
                */
                pr_info("%s: g_irq_tout_triggered cancel!\n", __func__);
                gp_stm8t143_dev->g_irq_tout_triggered = 0;
        }
        
        return IRQ_HANDLED;
}

static void stm8t143_3v3_power(unsigned int on)
{
	static struct regulator *pmic_3p3v_stm8t143;//LDO8 for 1.8V
	static int ldoenable;	
	if (!pmic_3p3v_stm8t143) 
	{
        
		pmic_3p3v_stm8t143 = regulator_get(NULL, "PMIC_LDO3"); /*Fix Me: conflict with schematics*/
		if (IS_ERR(pmic_3p3v_stm8t143)) {
			pmic_3p3v_stm8t143 = NULL;
			return;
		}
	}

	if (on) 
	{
		printk("&&&&&&&&&&&&&&&&&&&&&&&&&&&stm8t143 3V3 on\r\n");
		regulator_set_voltage(pmic_3p3v_stm8t143, 3300000, 3300000);
		if(!ldoenable)
		{
			regulator_enable(pmic_3p3v_stm8t143);
			ldoenable = 1;
		}
	}


	if(!on) {
		printk("!!!!!!!!!!!!!!!!!!!!!!!!!!!!1stm8t143 3V3 off\r\n");
		if(ldoenable)
		{		
			regulator_disable(pmic_3p3v_stm8t143);
			regulator_put(pmic_3p3v_stm8t143);
			pmic_3p3v_stm8t143 = NULL;
			ldoenable = 0;
		} 
       }

}


static void stm8t143_power_enable(int enable)
{
	
#if 0 //T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source, Paul @ 20110418 
        int psr_en = mfp_to_gpio(GPIO152_GPIO152), /* 3G_3V3_EN */
            rf_psen_pout = mfp_to_gpio(GPIO106_GPIO106), /* RF_PSEN_POUT */
#else
        int rf_psen_pout = mfp_to_gpio(GPIO106_GPIO), /* RF_PSEN_POUT */
#endif //T-Lite2
            rf_psen_tout = mfp_to_gpio(GPIO107_GPIO), /* RF_PSEN_TOUT */
            irq_rf_psen_pout = gpio_to_irq(rf_psen_pout),
            irq_rf_psen_tout = gpio_to_irq(rf_psen_tout),
            ret = 0;
        printk(KERN_ERR "%s(%d)\n", __func__, enable);
/*
		//T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source
        ret = gpio_request(psr_en, "psr_en");
        if(ret)
        {
                printk(KERN_ERR "psr_en (gpio %d) request failed!(error = %d)\n", psr_en, ret);
                goto error_request_psr_en;
        }
		//T-Lite2
*/ 
        
        ret = gpio_request(rf_psen_pout, "rf_psen_pout");
        if(ret)
        {
                printk(KERN_ERR "rf_psen_pout (gpio %d) request failed!(error = %d)\n", rf_psen_pout, ret);
                goto error_request_rf_psen_pout;
        }
        ret = gpio_request(rf_psen_tout, "rf_psen_tout");
        if(ret)
        {
                printk(KERN_ERR "rf_psen_tout (gpio %d) request failed!(error = %d)\n", rf_psen_tout, ret);       
                goto error_request_rf_psen_tout;
        }

        if(!enable)
        {
                if(gp_stm8t143_dev->g_irq_rf_psen_pout)
                {
                        pr_info("%s: disable irq_rf_psen_pout (%d).\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_pout);
                        disable_irq(gp_stm8t143_dev->g_irq_rf_psen_pout);
                }
                if(gp_stm8t143_dev->g_irq_rf_psen_tout)
                {
                        pr_info("%s: disable irq_rf_psen_tout (%d).\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_tout);
                        disable_irq(gp_stm8t143_dev->g_irq_rf_psen_tout);
                }
/*                
				//T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source
                pr_info("%s: disable psr_en (gpio %d).\n", __func__, psr_en);
                gpio_direction_output(psr_en, 0);
				//T-Lite2
*/ 
                mdelay(1);
        }
        else // enable
        {
                // Set irq rf_psen_pout
                if(!gp_stm8t143_dev->g_irq_rf_psen_pout)
                {
                        pr_info("%s: enable rf_psen_pout (gpio %d) input.\n", __func__, rf_psen_pout);
                        gpio_direction_input(rf_psen_pout);
                        irq_set_irq_type(irq_rf_psen_pout, IRQ_TYPE_EDGE_BOTH);
                        ret = request_irq(irq_rf_psen_pout, &rf_psen_pout_isr, IRQF_SHARED, "rf_psen_pout", &rf_psen_pout_isr);
                        if(ret)
                        {
                                pr_err("%s: request_irq() for rf_psen_pout failed (%d)\n", __func__, ret);
                                goto error;
                        }
                        
                        gp_stm8t143_dev->g_irq_rf_psen_pout = irq_rf_psen_pout;
                        pr_info("%s: irq_rf_psen_pout = %d.\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_pout);
                }
                else //gp_stm8t143_dev->g_irq_rf_psen_pout is requested
                {
                        pr_info("%s: enable rf_psen_pout (%d).\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_pout);
                        enable_irq(gp_stm8t143_dev->g_irq_rf_psen_pout);
                }

                // Set irq rf_psen_tout
                if(!gp_stm8t143_dev->g_irq_rf_psen_tout)
                {
                        pr_info("%s: enable rf_psen_tout (gpio %d) input.\n", __func__, rf_psen_tout);
                        gpio_direction_input(rf_psen_tout);
                        irq_set_irq_type(irq_rf_psen_tout, IRQ_TYPE_EDGE_BOTH);
                        ret = request_irq(irq_rf_psen_tout, &rf_psen_tout_isr, IRQF_SHARED, "rf_psen_tout", &rf_psen_tout_isr);
                        if(ret)
                        {
                                pr_err("%s: request_irq() rf_psen_pout failed (%d)\n", __func__, ret);
                                free_irq(irq_rf_psen_pout, gp_stm8t143_dev);
                                gp_stm8t143_dev->g_irq_rf_psen_pout = 0;
                                goto error;
                        }
                        gp_stm8t143_dev->g_irq_rf_psen_tout = irq_rf_psen_tout;
                        pr_info("%s: irq_rf_psen_tout = %d.\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_tout);
                }
                else
                {
                        pr_info("%s: enable rf_psen_tout (%d).\n", __func__, gp_stm8t143_dev->g_irq_rf_psen_tout);
                        enable_irq(gp_stm8t143_dev->g_irq_rf_psen_tout);
                }
/*
				//T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source
                // Set psr_en output
                pr_info("%s: enable psr_en (gpio %d) output.\n", __func__, psr_en);
                gpio_direction_output(psr_en, 1);
				//T-Lite2
*/
        }

        msleep(1);
        printk(KERN_ERR "%s: release rf_psen_tout (gpio %d).\n", __func__, rf_psen_tout);
        gpio_free(rf_psen_tout);
        printk(KERN_ERR "%s: release rf_psen_pout (gpio %d).\n", __func__, rf_psen_pout);
        gpio_free(rf_psen_pout);
/*        
        //T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source
        printk(KERN_ERR "%s: release psr_en (gpio %d).\n", __func__, psr_en);
        gpio_free(psr_en);
        //T-Lite2
*/
        printk(KERN_ERR "%s: Set power(%d) successfully!\n", __func__, enable);
        return;
error:
        printk(KERN_ERR "%s: release rf_psen_tout (gpio %d).\n", __func__, rf_psen_tout);
        gpio_free(rf_psen_tout);
error_request_rf_psen_tout:
        printk(KERN_ERR "%s: release rf_psen_pout (gpio %d).\n", __func__, rf_psen_pout);        
        gpio_free(rf_psen_pout);
error_request_rf_psen_pout:
/*
        //T-Lite2, not to control 3G_3V3_EN since 3G function also uses this power source
        printk(KERN_ERR "%s: release psr_en (gpio %d).\n", __func__, psr_en);       
        gpio_free(psr_en); 
error_request_psr_en:
        //T-Lite2          
*/ 
        printk(KERN_ERR "%s: Fail to set power(%d)!\n", __func__, enable);
        return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void stm8t143_early_suspend(struct early_suspend *h)
{
        printk(KERN_ERR "%s: state = %d.\r\n", __func__, gp_stm8t143_dev->g_ps_state);
        #if 0
        if(gp_stm8t143_dev->g_ps_state)
        {
                /* Disable timer function and flush unfinished jobs */
                printk(KERN_ERR "%s: Stop timer and flush unfinished jobs...\r\n", __func__);
                del_timer_sync(&gp_stm8t143_dev->g_timer_ps);
                flush_work(&g_stm8t143_ps_work); 
                
                /* Power-down STM8T143 Here */
                stm8t143_power_enable(0);
                
                printk("%s: status off\n", __func__);
        }
	#endif
}

static void stm8t143_late_resume(struct early_suspend *h)
{
        printk(KERN_ERR "%s: state = %d.\r\n", __func__, gp_stm8t143_dev->g_ps_state);
        #if 0
        if(gp_stm8t143_dev->g_ps_state)
        {
                /* Power-up STM8T143 Here */
                stm8t143_power_enable(1);
                
                /* Enable timer function to start polling */
                printk(KERN_ERR "%s: Start timer and jobs...\r\n", __func__);
                setup_timer(&gp_stm8t143_dev->g_timer_ps, stm8t143_ps_timer_func, 0);
                mod_timer(&gp_stm8t143_dev->g_timer_ps, jiffies + msecs_to_jiffies(gp_stm8t143_dev->g_ps_sample_interval));
                                                                        
                printk("%s: status on\n", __func__);       
        }
	#endif
}
#endif //CONFIG_HAS_EARLYSUSPEND

/*
static void hdmi_switch_work(struct work_struct *work)
{
	int state;
	struct hdmi_switch_data	*data =
		container_of(work, struct hdmi_switch_data, work);

	state = gpio_get_value(data->gpio);
	if(state!=0)
		state = 0;
	else
		state = 1; 
	switch_set_state(&data->sdev, state);
	hdmi_hpd_det(state);
}

static irqreturn_t hdmi_irq_handler(int irq, void *dev_id)
{
	struct hdmi_switch_data *switch_data =
	    (struct hdmi_switch_data *)dev_id;
	schedule_work(&switch_data->work);
	return IRQ_HANDLED;
}

static ssize_t switch_hdmi_print_state(struct switch_dev *sdev, char *buf)
{
	struct stm8t143_data *switch_data =
		container_of(sdev, struct stm8t143_data, sdev);
	const char *state;
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}
*/

static int stm8t143_status = 1;
static ssize_t stm8t143_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(stm8t143_status);                  
        len = sprintf(page, "%d\n", stm8t143_status);

	return len;
}
static ssize_t stm8t143_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];
 	int stm8t143_pwr_en;
	unsigned int stm8t143_cmd;
	//unsigned char tmp = INITIAL_LSB_DATA;	
	printk(KERN_ERR "%s: n\n", __func__);
	if (len > 256)
		len = 256;
	if (copy_from_user(messages, buff, len))
	{
		printk(KERN_ERR "%s error: %s \n", __func__,messages);
		return -EFAULT;
	}
	stm8t143_cmd = (unsigned int) simple_strtoul(messages, NULL, 10);
	switch(stm8t143_cmd){
	case 0:
        	if(gp_stm8t143_dev->g_ps_user_count > 0) /* Set disable and has been set enable */
        	{
                	if(gp_stm8t143_dev->g_ps_user_count == 1)
                	{
                        	/* Disable timer function to stop polling */
                        	del_timer_sync(&gp_stm8t143_dev->g_timer_ps);
                        	flush_work(&g_stm8t143_ps_work);
                        
                       		/* Power-down STM8T143 Here */
                        	stm8t143_power_enable(0);
                        
                        	gp_stm8t143_dev->g_ps_state = 0;
                        //	dev_info(dev, "status off\n");
                	}
                	else
                	{
                	        printk(KERN_ERR "%s: STM8T143 still has users (%d)!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count - 1);
                	}
                
                	gp_stm8t143_dev->g_ps_user_count--;
                	printk(KERN_ERR "%s: User count = %d!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count);
        	}
		break;

	 
	case 1:
                if(gp_stm8t143_dev->g_ps_user_count == 0)
                {
                        
                        /* Power-up STM8T143 Here */
                        stm8t143_power_enable(1);
                        
                        gp_stm8t143_dev->g_irq_pout_triggered = 0;
                        gp_stm8t143_dev->g_irq_tout_triggered = 0;
                        
                        /* Enable timer function to start polling */
                        setup_timer(&gp_stm8t143_dev->g_timer_ps, stm8t143_ps_timer_func, 0);
                        mod_timer(&gp_stm8t143_dev->g_timer_ps, jiffies + msecs_to_jiffies(gp_stm8t143_dev->g_ps_sample_interval));
                        
                        gp_stm8t143_dev->g_ps_state = 1;
                      //  dev_info(dev, "status on\n");
                }
                else
                {
                        printk(KERN_ERR "%s: STM8T143 has been enabled!\r\n", __func__);
                }
                
                gp_stm8t143_dev->g_ps_user_count++;
                //printk(KERN_ERR "%s: User count = %d!\r\n", __func__, gp_stm8t143_dev->g_ps_user_count);
		break;
	
	 default:
		printk(KERN_ERR,"default\n");
		pr_info("%s  default  \n", __func__);
		break;
	}
			printk(KERN_ERR "%s failed \n", __func__,stm8t143_cmd);	
	return len;
}
static void create_stm8t143_proc_file(void)
{
	struct proc_dir_entry *stm8t143_proc_file = create_proc_entry("driver/stm8t143", 0644, NULL);
	if (stm8t143_proc_file) {
		printk(KERN_INFO "%s proc file create successful\n", __func__);
	} else
		printk(KERN_ERR "proc file create failed!\n");
	stm8t143_proc_file->read_proc = (read_proc_t *)stm8t143_read_proc;
	stm8t143_proc_file->write_proc = (write_proc_t  *)stm8t143_write_proc;

}
static int stm8t143_probe(struct platform_device *pdev)
{
	struct gpio_switch_platform_data *pdata = pdev->dev.platform_data;
	struct stm8t143_data *switch_data;
	int ret = 0;

	if (!pdata)
		return -EBUSY;

        /* Init driver data */
	switch_data = kzalloc(sizeof(struct stm8t143_data), GFP_KERNEL);
	if (!switch_data)
		return -ENOMEM;

	switch_data->sdev.name = pdata->name;
	switch_data->gpio = pdata->gpio;
	switch_data->name_on = pdata->name_on;
	switch_data->name_off = pdata->name_off;
	switch_data->state_on = pdata->state_on;
	switch_data->state_off = pdata->state_off;
	
	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;
		
	/* Initialize driver data for STM8T143 */	
	gp_stm8t143_dev = switch_data; //T-Lite2, set switch_data to global section, Paul @ 20110401
	gp_stm8t143_dev->g_ps_user_count = 0;
	gp_stm8t143_dev->g_ps_state = 0;
    gp_stm8t143_dev->g_irq_pout_triggered = 0;
    gp_stm8t143_dev->g_irq_tout_triggered = 0;
	gp_stm8t143_dev->g_irq_rf_psen_pout = 0;
	gp_stm8t143_dev->g_irq_rf_psen_tout = 0;
	
	stm8t143_3v3_power(0);
	udelay(50);
	stm8t143_3v3_power(1);
	/* Power-down STM8T143 */  
	stm8t143_power_enable(0);

	/* Enable timer */
	stm8t143_set_ps_interval(200);
	/* Init proximity works */
	INIT_WORK(&g_stm8t143_ps_work, stm8t143_report_ps_value);
	INIT_WORK(&g_stm8t143_int_work, stm8t143_workq_func);
	/* Register an input for event update */
	ret = stm8t143_ps_register_device(&switch_data->sdev);
	if(ret)
	{
	        printk("%s: register input device for stm8t143 proximity sensor! (error: %d)\r\n", __func__, ret);
	        return ret;
	}
	
	/* Register attributes for userspace to control STM8T143 */
	stm8t143_add_fs(&gp_stm8t143_dev->g_ps->dev);
//No need any more
#if 0           
	
	/* Set ATTE initial state */
	atte1_set_command(0); // atte1 commond 0 - 0db
	atte2_set_command(0); // atte2 command 0 - 0db
	atte3_set_command(0); // atte3 command 0 - 0db
#endif	
#ifdef  CONFIG_PROC_FS
        /* create proc for sirf control */
        create_atte1_proc_file();
        create_atte2_proc_file();
        create_atte3_proc_file();
	create_stm8t143_proc_file();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
        g_stm8t143_early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
        g_stm8t143_early_suspend.suspend = stm8t143_early_suspend;
        g_stm8t143_early_suspend.resume  = stm8t143_late_resume;
        register_early_suspend(&g_stm8t143_early_suspend); 
#endif //CONFIG_HAS_EARLYSUSPEND

	return 0;

err_switch_dev_register:
	kfree(switch_data);

	return ret;
}

static int __devexit stm8t143_remove(struct platform_device *pdev)
{
	struct stm8t143_data *switch_data = platform_get_drvdata(pdev);

	cancel_work_sync(&g_stm8t143_int_work); //T-Lite2
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->gpio);
    	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver stm8t143_driver = {
	.probe		= stm8t143_probe,
	.remove		= __devexit_p(stm8t143_remove),
	.driver		= {
		.name	= "stm8t143",
		.owner	= THIS_MODULE,
	},
};

static int __init stm8t143_init(void)
{
	return platform_driver_register(&stm8t143_driver);
}

static void __exit stm8t143_exit(void)
{
	platform_driver_unregister(&stm8t143_driver);
}

module_init(stm8t143_init);
module_exit(stm8t143_exit);

MODULE_AUTHOR("Mike Lockwood <lockwood@android.com>");
MODULE_DESCRIPTION("STM8T143 driver");
MODULE_LICENSE("GPL");
