/*
 *  linux/arch/arm/mach-mmp/ttc_dkb.c
 *
 *  Support for the Marvell PXA910-based TTC_DKB Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/onenand.h>
#include <linux/interrupt.h>
#include <linux/mfd/88pm860x.h>
#include <linux/i2c/pca9575.h>
#include <linux/i2c/pca953x.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/cmmb.h>
#include <linux/proc_fs.h>

#include <asm/mach-types.h>
#include <asm/uaccess.h>
#include <asm/mach/arch.h>
#include <asm/mach/flash.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa910.h>
#include <mach/pxa910.h>

#include "common.h"

#define TTCDKB_NR_IRQS		(IRQ_BOARD_START + 24)

static int is_td_dkb;
static int __init td_dkb_setup(char *__unused)
{
	return is_td_dkb = 1;
}
__setup("td_dkb", td_dkb_setup);

static unsigned long ttc_dkb_pin_config[] __initdata = {
	/* UART2 */
	GPIO47_UART2_RXD,
	GPIO48_UART2_TXD,

	/* DFI */
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,
	DF_nCS0_SM_nCS2_nCS0,
	DF_ALE_SM_WEn_ND_ALE,
	DF_CLE_SM_OEn_ND_CLE,
	DF_WEn_DF_WEn,
	DF_REn_DF_REn,
	DF_RDY0_DF_RDY0,

	/* one wire */
	ONEWIRE_CLK_REQ,

	/*keypad*/
	GPIO00_KP_MKIN0,
	GPIO01_KP_MKOUT0,
	GPIO02_KP_MKIN1,
	GPIO03_KP_MKOUT1,
	GPIO04_KP_MKIN2,
	GPIO05_KP_MKOUT2,
	GPIO06_KP_MKIN3,
	GPIO07_KP_MKOUT3,
	GPIO08_KP_MKIN4,
	GPIO09_KP_MKOUT4,
	GPIO12_KP_MKIN6,
};

static struct mtd_partition ttc_dkb_onenand_partitions[] = {
	{
		.name		= "bootloader",
		.offset		= 0,
		.size		= SZ_1M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_128K,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "reserved",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_8M,
		.mask_flags	= MTD_WRITEABLE,
	}, {
		.name		= "kernel",
		.offset		= MTDPART_OFS_APPEND,
		.size		= (SZ_2M + SZ_1M),
		.mask_flags	= 0,
	}, {
		.name		= "filesystem",
		.offset		= MTDPART_OFS_APPEND,
		.size		= SZ_48M,
		.mask_flags	= 0,
	}
};

static struct onenand_platform_data ttc_dkb_onenand_info = {
	.parts		= ttc_dkb_onenand_partitions,
	.nr_parts	= ARRAY_SIZE(ttc_dkb_onenand_partitions),
};

static struct resource ttc_dkb_resource_onenand[] = {
	[0] = {
		.start	= SMC_CS0_PHYS_BASE,
		.end	= SMC_CS0_PHYS_BASE + SZ_1M,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device ttc_dkb_device_onenand = {
	.name		= "onenand-flash",
	.id		= -1,
	.resource	= ttc_dkb_resource_onenand,
	.num_resources	= ARRAY_SIZE(ttc_dkb_resource_onenand),
	.dev		= {
		.platform_data	= &ttc_dkb_onenand_info,
	},
};

static unsigned int ttc_dkb_matrix_key_map[] = {
	KEY(0, 0, KEY_BACKSPACE),
	KEY(0, 1, KEY_END),
	KEY(0, 2, KEY_RIGHTCTRL),
	KEY(0, 3, KEY_0),
	KEY(0, 4, KEY_1),

	KEY(1, 0, KEY_MENU),
	KEY(1, 1, KEY_HOME),
	KEY(1, 2, KEY_SEND),
	KEY(1, 3, KEY_8),
	KEY(1, 4, KEY_9),

	KEY(2, 0, KEY_OK),
	KEY(2, 1, KEY_2),
	KEY(2, 2, KEY_3),
	KEY(2, 3, KEY_4),
	KEY(2, 4, KEY_5),

	KEY(3, 0, KEY_6),
	KEY(3, 1, KEY_VOLUMEUP),
	KEY(3, 2, KEY_7),
	KEY(3, 3, KEY_VOLUMEDOWN),
	KEY(3, 4, KEY_RECORD),

	KEY(4, 0, KEY_KPASTERISK),
	KEY(4, 1, KEY_KPDOT),
	KEY(4, 2, KEY_F2),
	KEY(4, 3, KEY_CAMERA),
	KEY(4, 4, KEY_CAMERA),

	KEY(6, 0, KEY_F1),
	KEY(6, 1, KEY_UP),
	KEY(6, 2, KEY_DOWN),
	KEY(6, 3, KEY_LEFT),
	KEY(6, 4, KEY_RIGHT),
};

static struct pxa27x_keypad_platform_data ttc_dkb_keypad_info __initdata = {
	.matrix_key_rows	= 7,
	.matrix_key_cols	= 5,
	.matrix_key_map		= ttc_dkb_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(ttc_dkb_matrix_key_map),
	.debounce_interval	= 30,
};

static struct pm860x_backlight_pdata ttc_dkb_backlight[] = {
	{
		.id	= PM8606_ID_BACKLIGHT,
		.iset	= PM8606_WLED_CURRENT(4),
		.flags	= PM8606_BACKLIGHT1,
	},
};

static struct pm860x_led_pdata ttc_dkb_led[] = {
	{
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_RED,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_GREEN,
	}, {
		.id	= PM8606_ID_LED,
		.iset	= PM8606_LED_CURRENT(12),
		.flags	= PM8606_LED1_BLUE,
	},
};

static struct pm860x_platform_data ttc_dkb_pm8607_info = {
	.backlight	= &ttc_dkb_backlight[0],
	.led		= &ttc_dkb_led[0],
	.companion_addr	= 0x11,
	.irq_mode	= 0,
	.irq_base	= IRQ_BOARD_START,

	.i2c_port	= GI2C_PORT,
	.num_backlights	= ARRAY_SIZE(ttc_dkb_backlight),
	.num_leds	= ARRAY_SIZE(ttc_dkb_led),
};

#if defined(CONFIG_GPIO_PCA9575)
static struct pca9575_platform_data pca9575_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT1(0),
	},
};
#endif

#if defined(CONFIG_GPIO_PCA953X)
static struct pca953x_platform_data max7312_data[] = {
	[0] = {
		.gpio_base      = GPIO_EXT0(0),
	},
};
#endif

static struct i2c_board_info ttc_dkb_i2c_info[] = {
	{
		.type		= "88PM860x",
		.addr		= 0x34,
		.platform_data	= &ttc_dkb_pm8607_info,
		.irq		= IRQ_PXA910_PMIC_INT,
	},
#if defined(CONFIG_GPIO_PCA9575)
	{
		.type           = "pca9575",
		.addr           = 0x20,
		.irq            = IRQ_GPIO(19),
		.platform_data  = &pca9575_data,
	},
#endif
#if defined(CONFIG_GPIO_PCA953X)
	{
		.type           = "max7312",
		.addr           = 0x23,
		.irq            = IRQ_GPIO(80),
		.platform_data  = &max7312_data,
	},
#endif
};

static struct i2c_pxa_platform_data dkb_i2c_pdata = {
	.fast_mode		 = 1,
	/* ilcr:fs mode b17~9=0x22,about 380K, standard mode b8~0=0x7E,100K */
	.ilcr			 = 0x082C447E,
	/* iwcr:b5~0=b01010 recommended value according to spec*/
	.hardware_lock		= pxa910_ripc_lock,
	.hardware_unlock	= pxa910_ripc_unlock,
	.hardware_trylock	= pxa910_ripc_trylock,
};

static struct i2c_pxa_platform_data ttc_dkb_pwr_i2c_pdata = {
	.fast_mode		 = 1,
	/* ilcr:fs mode b17~9=0x22,about 380K, standard mode b8~0=0x7E,100K */
	.ilcr			 = 0x082C447E,
	/* iwcr:b5~0=b01010 recommended value according to spec*/
	.iwcr			= 0x0000142A,
};

static struct platform_device *ttc_dkb_devices[] = {
	&ttc_dkb_device_onenand,
	&pxa910_device_rtc,
};

#if (defined CONFIG_CMMB)

static unsigned long cmmb_pin_config[] = {
	GPIO33_SSP0_CLK,
	GPIO35_SSP0_RXD,
	GPIO36_SSP0_TXD,
};

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect = 1,
	.enable_dma = 1,
};

static int cmmb_power_reset(void)
{
	int cmmb_rst;

	cmmb_rst = GPIO_EXT1(7);

	if (gpio_request(cmmb_rst, "cmmb rst")) {
		pr_warning("failed to request GPIO for CMMB RST\n");
		return -EIO;
	}

	/* reset cmmb, keep low for about 1ms */
	gpio_direction_output(cmmb_rst, 0);
	msleep(100);

	/* get cmmb go out of reset state */
	gpio_direction_output(cmmb_rst, 1);
	gpio_free(cmmb_rst);

	return 0;
}

static int cmmb_power_on(void)
{
	int cmmb_en, cmmb_rst;

	cmmb_en = GPIO_EXT1(6);
	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("[ERROR] failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}
	gpio_direction_output(cmmb_en, 0);
	msleep(100);

	gpio_direction_output(cmmb_en, 1);
	gpio_free(cmmb_en);

	msleep(100);

	cmmb_power_reset();

	return 0;
}

static int cmmb_power_off(void)
{
	int cmmb_en;

	cmmb_en = GPIO_EXT1(6);

	if (gpio_request(cmmb_en, "cmmb power")) {
		pr_warning("failed to request GPIO for CMMB POWER\n");
		return -EIO;
	}

	gpio_direction_output(cmmb_en, 0);
	gpio_free(cmmb_en);
	msleep(100);

	return 0;
}
/*.
 ** Add two functions: cmmb_cs_assert and cmmb_cs_deassert.
 ** Provide the capbility that
 ** cmmb driver can handle the SPI_CS by itself.
 **/
static int cmmb_cs_assert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 0);
	return 0;
}

static int cmmb_cs_deassert(void)
{
	int cs;
	cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	gpio_direction_output(cs, 1);
	return 0;
}

static struct cmmb_platform_data cmmb_info = {
	.power_on = cmmb_power_on,
	.power_off = cmmb_power_off,
	.power_reset = cmmb_power_reset,
	.cs_assert = cmmb_cs_assert,
	.cs_deassert = cmmb_cs_deassert,

	.gpio_power = GPIO_EXT1(6),
	.gpio_reset = GPIO_EXT1(7),
	.gpio_cs = mfp_to_gpio(GPIO34_SSP0_FRM),
	.gpio_defined = 1,
};

static void cmmb_if101_cs(u32 cmd)
{
/* Because in CMMB read/write,the max data size is more than 8kB
 * 8k = max data length per dma transfer for pxaxxx
 * But till now,The spi_read/write driver doesn't support muti DMA cycles
 *
 * Here the spi_read/write will not affect the SPI_CS,but provides
 * cs_assert and cs_deassert in the struct cmmb_platform_data
 *
 * And cmmb driver can/should control SPI_CS by itself
 */
}

static struct pxa2xx_spi_chip cmmb_if101_chip = {
	.rx_threshold   = 1,
	.tx_threshold   = 1,
	.cs_control     = cmmb_if101_cs,
};

/* bus_num must match id in pxa2xx_set_spi_info() call */
static struct spi_board_info spi_board_info[] __initdata = {
	{
		.modalias		= "cmmb_if",
		.platform_data	= &cmmb_info,
		.controller_data	= &cmmb_if101_chip,
		.irq			= gpio_to_irq(mfp_to_gpio(GPIO14_GPIO)),
		.max_speed_hz	= 8000000,
		.bus_num		= 1,
		.chip_select	= 0,
		.mode			= SPI_MODE_0,
	},
};

static void __init ttc_dkb_init_spi(void)
{
	int err;
	int cmmb_int, cmmb_cs;

	mfp_config(ARRAY_AND_SIZE(cmmb_pin_config));
	cmmb_cs = mfp_to_gpio(GPIO34_SSP0_FRM);
	err = gpio_request(cmmb_cs, "cmmb cs");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB CS\n");
		return;
	}
	gpio_direction_output(cmmb_cs, 1);

	cmmb_int = mfp_to_gpio(GPIO14_GPIO);

	err = gpio_request(cmmb_int, "cmmb irq");
	if (err) {
		pr_warning("[ERROR] failed to request GPIO for CMMB IRQ\n");
		return;
	}
	gpio_direction_input(cmmb_int);

	pxa910_add_ssp(0);
	pxa910_add_spi(1, &pxa_ssp_master_info);
	if (spi_register_board_info(spi_board_info,
			ARRAY_SIZE(spi_board_info))) {
		pr_warning("[ERROR] failed to register spi device.\n");
		return;
	}
}

#endif /*defined CONFIG_CMMB*/

/* GPS: power on/off control */
static void gps_power_on(void)
{
	unsigned int gps_ldo, gps_rst_n;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	mdelay(1);
	gpio_direction_output(gps_ldo, 1);

	pr_info("sirf gps chip powered on\n");

	gpio_free(gps_rst_n);
out:
	gpio_free(gps_ldo);
	return;
}

static void gps_power_off(void)
{
	unsigned int gps_ldo, gps_rst_n, gps_on;

	gps_ldo = (is_td_dkb) ? GPIO_EXT1(8) : GPIO_EXT1(7);
	if (gpio_request(gps_ldo, "gpio_gps_ldo")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_ldo);
		return;
	}

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed,gpio: %d\n", gps_on);
		goto out1;
	}

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_debug("Request GPIO failed, gpio: %d\n", gps_rst_n);
		goto out2;
	}

	gpio_direction_output(gps_ldo, 0);
	gpio_direction_output(gps_rst_n, 0);
	gpio_direction_output(gps_on, 0);

	pr_info("sirf gps chip powered off\n");

	gpio_free(gps_rst_n);
out2:
	gpio_free(gps_on);
out1:
	gpio_free(gps_ldo);
	return;
}

static void gps_reset(int flag)
{
	unsigned int gps_rst_n;

	gps_rst_n = (is_td_dkb) ? GPIO_EXT1(11) : mfp_to_gpio(MFP_PIN_GPIO15);
	if (gpio_request(gps_rst_n, "gpio_gps_rst")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_rst_n);
		return;
	}

	gpio_direction_output(gps_rst_n, flag);
	gpio_free(gps_rst_n);
	pr_info("sirf gps chip reset\n");
}

static void gps_on_off(int flag)
{
	unsigned int gps_on;

	gps_on = (is_td_dkb) ? GPIO_EXT1(10) : GPIO_EXT1(1);
	if (gpio_request(gps_on, "gpio_gps_on")) {
		pr_err("Request GPIO failed, gpio: %d\n", gps_on);
		return;
	}
	gpio_direction_output(gps_on, flag);
	gpio_free(gps_on);
	pr_info("sirf gps chip offon\n");
}

#if defined(CONFIG_PROC_FS)

#define SIRF_STATUS_LEN	16
static char sirf_status[SIRF_STATUS_LEN] = "off";

static ssize_t sirf_read_proc(char *page, char **start, off_t off,
		int count, int *eof, void *data)
{
	int len = strlen(sirf_status);

	sprintf(page, "%s\n", sirf_status);
	return len + 1;
}

static ssize_t sirf_write_proc(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	char messages[256];
	int flag, ret;
	char buffer[7];

	if (len > 255)
		len = 255;

	memset(messages, 0, sizeof(messages));

	if (!buff || copy_from_user(messages, buff, len))
		return -EFAULT;

	if (strlen(messages) > (SIRF_STATUS_LEN - 1)) {
		pr_warning("[ERROR] messages too long! (%d) %s\n",
			strlen(messages), messages);
		return -EFAULT;
	}

	if (strncmp(messages, "off", 3) == 0) {
		strcpy(sirf_status, "off");
		gps_power_off();
	} else if (strncmp(messages, "on", 2) == 0) {
		strcpy(sirf_status, "on");
		gps_power_on();
	} else if (strncmp(messages, "reset", 5) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_reset(flag);
	} else if (strncmp(messages, "sirfon", 6) == 0) {
		strcpy(sirf_status, messages);
		ret = sscanf(messages, "%s %d", buffer, &flag);
		if (ret == 2)
			gps_on_off(flag);
	} else
		pr_info("usage: echo {on/off} > /proc/driver/sirf\n");

	return len;
}

static void create_sirf_proc_file(void)
{
	struct proc_dir_entry *sirf_proc_file = NULL;

	sirf_proc_file = create_proc_entry("driver/sirf", 0644, NULL);
	if (!sirf_proc_file) {
		pr_err("sirf proc file create failed!\n");
		return;
	}

	sirf_proc_file->read_proc = sirf_read_proc;
	sirf_proc_file->write_proc = (write_proc_t  *)sirf_write_proc;
}
#endif

static void __init ttc_dkb_init(void)
{
	mfp_config(ARRAY_AND_SIZE(ttc_dkb_pin_config));

	/* on-chip devices */
	pxa910_add_uart(1);
	pxa910_add_1wire();

	pxa910_add_keypad(&ttc_dkb_keypad_info);
	pxa910_add_cnm();
	pxa910_add_twsi(0, &dkb_i2c_pdata, ARRAY_AND_SIZE(ttc_dkb_i2c_info));

	/* off-chip devices */
	platform_add_devices(ARRAY_AND_SIZE(ttc_dkb_devices));

#if defined(CONFIG_PROC_FS)
	/* create proc for sirf GPS control */
	create_sirf_proc_file();
#endif

#if (defined CONFIG_CMMB)
	 /* spi device */
	if (is_td_dkb)
		ttc_dkb_init_spi();
#endif
}

MACHINE_START(TTC_DKB, "PXA910-based TTC_DKB Development Platform")
	.map_io		= mmp_map_io,
	.nr_irqs	= TTCDKB_NR_IRQS,
	.init_irq       = pxa910_init_irq,
	.timer          = &pxa910_timer,
	.init_machine   = ttc_dkb_init,
MACHINE_END
