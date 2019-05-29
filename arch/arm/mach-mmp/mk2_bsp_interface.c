#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <mach/mfp-mmp2.h>
#include <mach/wistron.h>
#include <mach/camera.h>
#include <linux/module.h>

static char nvs_buffer[EMMC_BLOCK_SIZE];
struct WISTRON_NVS *nvs = (struct WISTRON_NVS *)nvs_buffer;
struct delayed_work monitor_work;

extern s32 wiskbc_reg_read(u8);
extern s32 wiskbc_reg_write(u8, u8);

static int create_proc_dir(char *dir)
{
	struct proc_dir_entry *proc_dir;
	static int created = 0;
	
	if(created)
		goto end;

	if (!(proc_dir = proc_mkdir(dir, NULL))) {
		printk(KERN_ERR "--> %d %s %s\n", __LINE__, __func__, __FILE__);
        	return -1;
    	}
	else
		created = 1;
end:
	return 1;
}

#define UBOOT_VER_LEN 32		/* string length */
static char ver_buf[UBOOT_VER_LEN + 1];	/* 1 byte for %NUL-terminated */

static int uboot_ver_setup(char *str)
{
	strncpy(ver_buf, str, sizeof(ver_buf) - 1);
	ver_buf[sizeof(ver_buf) - 1] = 0;

	return 1;
}
/* uboot version setup */
__setup("androidboot.bootloader=", uboot_ver_setup);

static ssize_t ubootversion_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	return sprintf(page, "[B]%s\n", ver_buf) + 1;
}

static void create_ubootversion_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_BOOTLOADER_VER, 0444, NULL);

	if (proc_file)
		proc_file->read_proc = ubootversion_read_proc;
	else
		printk(KERN_INFO "proc file create failed!\n");
}

static ssize_t osversion_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	return sprintf(page, "[K]%d\n", WISTRON_VERSION_STRING) + 1;
}

static void create_osversion_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_OS_VER, 0444, NULL);

	if (proc_file)
		proc_file->read_proc = osversion_read_proc;
	else
		printk(KERN_INFO "proc file create failed!\n");
}

static ssize_t manufacturer_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	return sprintf(page, "%s\n", MANUFACTURER) + 1;
}

static void create_manufacturer_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_MANUFACTURER, 0444, NULL);

	if (proc_file)
		proc_file->read_proc = manufacturer_read_proc;
	else
		printk(KERN_INFO "proc file create failed!\n");
}

int mk2_bsp_interface_set_flash(enum FLASH_TYPE state){

	s32 val;

	int gpio84 = mfp_to_gpio(GPIO84_GPIO);
	int gpio85 = mfp_to_gpio(GPIO85_GPIO);

	if (gpio_request(gpio84, "FLASH_LED_PIN_1")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", gpio84);
		return -1;
	}

	if (gpio_request(gpio85, "FLASH_LED_PIN_2")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", gpio85);
		return -1;
	}

	if(state == FLASH_TYPE_OFF){
		gpio_direction_output(gpio84, 0);
		gpio_direction_output(gpio85, 0);
	}
	else if(state == FLASH_TYPE_FLASH){
		val = wiskbc_reg_read(0x40);
		val |= 1<<4;
		wiskbc_reg_write(0x40, val);

		gpio_direction_output(gpio84, 0);
		gpio_direction_output(gpio85, 0);
		msleep(10);
		gpio_direction_output(gpio84, 1);

		/* reschedule for the next time */
		schedule_delayed_work(&monitor_work, msecs_to_jiffies(3000));
	}
	else if(state == FLASH_TYPE_TORCH){
		val = wiskbc_reg_read(0x40);
		val |= 1<<4;
		wiskbc_reg_write(0x40, val);

		gpio_direction_output(gpio84, 0);
		gpio_direction_output(gpio85, 0);
		msleep(10);
		gpio_direction_output(gpio84, 0);
		gpio_direction_output(gpio85, 1);
	}
	else{
		gpio_direction_output(gpio84, 0);
		gpio_direction_output(gpio85, 0);
	}

	gpio_free(gpio84);
	gpio_free(gpio85);
	
	return 0;
};
EXPORT_SYMBOL(mk2_bsp_interface_set_flash);

static ssize_t flash_led_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[5];

	if(len > sizeof(messages)){
		if (copy_from_user(messages, buff, sizeof(messages)))
			return -EFAULT;
	}
	else{
		if (copy_from_user(messages, buff, len))
			return -EFAULT;
	}
			
	if (strncmp(messages, "off", strlen("off")) == 0) 
		mk2_bsp_interface_set_flash(FLASH_TYPE_OFF);
	else if (strncmp(messages, "flash", strlen("flash")) == 0)
		mk2_bsp_interface_set_flash(FLASH_TYPE_FLASH);
	else if (strncmp(messages, "torch", strlen("torch")) == 0)
		mk2_bsp_interface_set_flash(FLASH_TYPE_TORCH);
	else
		printk("usage: {torch/flash/off}\n");

	return len;
}

static void flash_led_monitor(struct work_struct *work)
{
	mk2_bsp_interface_set_flash(FLASH_TYPE_OFF);
}

static void flash_led_proc_file(void)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry(PROJ_PROC_INFO_DIR"/flashLED", 0644, NULL);

	int gpio82 = mfp_to_gpio(GPIO82_GPIO);
	int gpio84 = mfp_to_gpio(GPIO84_GPIO);
	int gpio85 = mfp_to_gpio(GPIO85_GPIO);

	INIT_DELAYED_WORK(&monitor_work, flash_led_monitor);

	if (proc_file)
		proc_file->write_proc = (write_proc_t *)flash_led_write_proc;
	else
		printk(KERN_INFO "proc file create failed!\n");

	if (gpio_request(gpio82, "CAMERA VCM")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", gpio82);
	}

	if (gpio_request(gpio84, "FLASH_LED_PIN_1")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", gpio84);
	}

	if (gpio_request(gpio85, "FLASH_LED_PIN_2")) {
		printk(KERN_ERR "Request GPIO failed, gpio: %d\n", gpio85);
	}

	gpio_direction_output(gpio82, 0);
	gpio_direction_output(gpio84, 0);
	gpio_direction_output(gpio85, 0);

	gpio_free(gpio82);
	gpio_free(gpio84);
	gpio_free(gpio85);
}

static int nvs_read(void){

	struct file *fp;
	loff_t f_ops;
	mm_segment_t old_fs;
	ssize_t result;

	fp = filp_open("/dev/mmcblk0", O_RDONLY, 0); 
	if(IS_ERR(fp)){
		fp = filp_open("/dev/block/mmcblk0", O_RDONLY, 0); 
	}
	if(IS_ERR(fp))
		goto fail;	

	if(!fp->f_op->read)
		goto fail_close;
	if(!fp->f_op->llseek) 
		goto fail_close;

	old_fs = get_fs();
	set_fs(get_ds());

	f_ops = fp->f_op->llseek(fp, NVS_OFFSET, 0);
	result = fp->f_op->read(fp, nvs_buffer, EMMC_BLOCK_SIZE, &fp->f_pos);
	if(result <0)
		goto fail_close2;

	set_fs(old_fs);
	filp_close(fp, NULL);

	return 0;

fail_close2:
	set_fs(old_fs);
fail_close:
	filp_close(fp, NULL);
fail:
	return -1;
}

/*
 * getBasebandType() is used by wwan_init to determine current sku on device!
 */

const char* getBasebandType() {
	return nvs->baseband;
}

static ssize_t serial_0_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data){

	if(off > 0){
		*eof = 1;
		return 0;
	}

	nvs_read();
	return sprintf(page, "%s\n", nvs->sn_mb) + 1;
}

static ssize_t serial_1_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data){

	if(off > 0){
		*eof = 1;
		return 0;
	}

	nvs_read();
	return sprintf(page, "%s\n", nvs->sn_system) + 1;
}

static ssize_t serial_2_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	nvs_read();
	return sprintf(page, "%s\n", nvs->sn_customer) + 1;
}

static ssize_t language_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	nvs_read();
	return sprintf(page, "ro.product.locale.language=%s\n", nvs->language) + 1;
}

static ssize_t region_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	if(off > 0){
		*eof = 1;
		return 0;
	}

	nvs_read();
	return sprintf(page, "ro.product.locale.region=%s\n", nvs->region) + 1;
}

static ssize_t baseband_read_proc(char *page, char **start, off_t off,
                int count, int *eof, void *data)
{
        if(off > 0){
                *eof = 1;
                return 0;
        }

        nvs_read();
        return sprintf(page, "%s\n", nvs->baseband) + 1;
}


static int nvs_write(void){
	struct file *fp;
	loff_t f_ops;
	mm_segment_t old_fs;
	ssize_t result;

	fp = filp_open("/dev/mmcblk0", O_RDWR | O_SYNC, 0); 
	if(IS_ERR(fp)){
printk("===> %d %s %s\n", __LINE__, __func__, __FILE__);
		fp = filp_open("/dev/block/mmcblk0", O_RDWR | O_SYNC, 0); 
	}
	if(IS_ERR(fp)){
printk("===> %d %s %s\n", __LINE__, __func__, __FILE__);
		goto fail;
	}	

	if(!fp->f_op->read){
printk("===> %d %s %s\n", __LINE__, __func__, __FILE__);
		goto fail_close;
	}
	if(!fp->f_op->llseek){ 
printk("===> %d %s %s\n", __LINE__, __func__, __FILE__);
		goto fail_close;
	}

	old_fs = get_fs();
	set_fs(get_ds());

	f_ops = fp->f_op->llseek(fp, NVS_OFFSET, 0);
	result = fp->f_op->write(fp, nvs_buffer, EMMC_BLOCK_SIZE, &fp->f_pos);
	if(result <0){
printk("===> %d %s %s\n", __LINE__, __func__, __FILE__);
		goto fail_close2;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	return 0;

fail_close2:
	set_fs(old_fs);
fail_close:
	filp_close(fp, NULL);
fail:
	return -1;
}

static ssize_t serial_0_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[SN_SIZE] = {0};
	
	if(len >= (SN_SIZE - 1))
		len = SN_SIZE -1;
	else
		len -=1;		//ignore "\n"

	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	
	messages[SN_SIZE-1] = 0x0;
	sprintf(nvs->sn_mb, "%s", messages);
	nvs_write();
	nvs_read();

	return strlen(buff);
}

static ssize_t serial_1_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[SN_SIZE] = {0};
	
	if(len >= (SN_SIZE - 1))
		len = SN_SIZE -1;
	else
		len -=1;		//ignore "\n"

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	messages[SN_SIZE-1] = 0x0;
	sprintf(nvs->sn_system, "%s", messages);
	nvs_write();
	nvs_read();

	return strlen(buff);
}

static ssize_t serial_2_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[SN_SIZE] = {0};
	
	if(len >= (SN_SIZE - 1))
		len = SN_SIZE -1;
	else
		len -=1;		//ignore "\n"

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	messages[SN_SIZE-1] = 0x0;
	sprintf(nvs->sn_customer, "%s", messages);
	nvs_write();
	nvs_read();

	return strlen(buff);
}

static ssize_t language_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[LANGUAGE_SIZE] = {0};
	
	if(len >= (LANGUAGE_SIZE - 1))
		len = LANGUAGE_SIZE -1;
	else
		len -=1;		//ignore "\n"

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	messages[LANGUAGE_SIZE-1] = 0x0;
	sprintf(nvs->language, "%s", messages);
	nvs_write();
	nvs_read();

	return strlen(buff);
}

static ssize_t region_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[REGION_SIZE] = {0};
	
	if(len >= (REGION_SIZE - 1))
		len = REGION_SIZE -1;
	else
		len -=1;		//ignore "\n"

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	messages[REGION_SIZE-1] = 0x0;
	sprintf(nvs->region, "%s", messages);
	nvs_write();
	nvs_read();

	return strlen(buff);
}

static ssize_t baseband_write_proc(struct file *filp,
                const char *buff, size_t len, loff_t *off)
{
        char messages[BASEBAND_SIZE] = {0};

        if(len >= (BASEBAND_SIZE - 1))
                len = BASEBAND_SIZE -1;
        else
                len -=1;                //ignore "\n"

        if (copy_from_user(messages, buff, len))
                return -EFAULT;

        messages[BASEBAND_SIZE-1] = 0x0;
        sprintf(nvs->baseband, "%s", messages);
        nvs_write();
        nvs_read();

        return strlen(buff);
}

static void create_serial_proc_file(void)
{
	struct proc_dir_entry *proc_file_0 =
		create_proc_entry(PROJ_PROC_INFO_DIR"/serial-mb", 0644, NULL);
	struct proc_dir_entry *proc_file_1 =
		create_proc_entry(PROJ_PROC_INFO_DIR"/serial-sys", 0644, NULL);
	struct proc_dir_entry *proc_file_2 =
		create_proc_entry(PROJ_PROC_INFO_DIR"/serial", 0644, NULL);
	struct proc_dir_entry *proc_file_language =
		create_proc_entry(PROJ_PROC_INFO_DIR"/language", 0644, NULL);
	struct proc_dir_entry *proc_file_region =
		create_proc_entry(PROJ_PROC_INFO_DIR"/region", 0644, NULL);
        struct proc_dir_entry *proc_file_baseband =
                create_proc_entry(PROJ_PROC_INFO_DIR"/baseband", 0644, NULL);



	if (proc_file_0){
		proc_file_0->read_proc = serial_0_read_proc;
		proc_file_0->write_proc = (write_proc_t *)serial_0_write_proc;
	}
	if (proc_file_1){
		proc_file_1->read_proc = serial_1_read_proc;
		proc_file_1->write_proc = (write_proc_t *)serial_1_write_proc;
	}
	if (proc_file_2){
		proc_file_2->read_proc = serial_2_read_proc;
		proc_file_2->write_proc = (write_proc_t *)serial_2_write_proc;
	}
	if (proc_file_language){
		proc_file_language->read_proc = language_read_proc;
		proc_file_language->write_proc = (write_proc_t *)language_write_proc;
	}
	if (proc_file_region){
		proc_file_region->read_proc = region_read_proc;
		proc_file_region->write_proc = (write_proc_t *)region_write_proc;
	}
        if (proc_file_baseband){
                proc_file_baseband->read_proc = baseband_read_proc;
                proc_file_baseband->write_proc = (write_proc_t *)baseband_write_proc;		
	} 
	else
		printk(KERN_INFO "proc file create failed!\n");
}
void create_wis_proj_procfs(){
	create_proc_dir(PROJ_PROC_INFO_DIR);
	create_ubootversion_proc_file();
	create_osversion_proc_file();
	create_manufacturer_proc_file();
	create_serial_proc_file();
	flash_led_proc_file();
}

