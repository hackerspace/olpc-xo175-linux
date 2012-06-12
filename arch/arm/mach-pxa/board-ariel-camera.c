
static struct sensor_power_pin ov640_power_on_seq[] = {
	{CAMP_AVDD,	LEVEL_POS,	0},
	{CAMP_AFVCC,	LEVEL_POS,	5},
	{CAMP_PWD,	LEVEL_POS,	0},
	{CAMP_RST,	LEVEL_POS,	5},
	{CAMP_END}
};
static struct sensor_power_pin ov640_power_off_seq[] = {
	{CAMP_RST,	LEVEL_NEG,	0},
	{CAMP_PWD,	LEVEL_NEG,	0},
	{CAMP_AFVCC,	LEVEL_NEG,	0},
	{CAMP_AVDD,	LEVEL_NEG,	5},
	{CAMP_END}
};

static struct sensor_power_pin ov564x_power_on_seq[] = {
	{CAMP_AVDD,	LEVEL_POS,	0},
	{CAMP_AFVCC,	LEVEL_POS,	5},
	{CAMP_PWD,	LEVEL_NEG,	0},
	{CAMP_RST,	LEVEL_POS,	100},
	{CAMP_END}
};
static struct sensor_power_pin ov564x_power_off_seq[] = {
	{CAMP_RST,	LEVEL_NEG,	50},
	{CAMP_PWD,	LEVEL_POS,	0},
	{CAMP_AFVCC,	LEVEL_NEG,	0},
	{CAMP_AVDD,	LEVEL_NEG,	50},
	{CAMP_END}
};

static struct sensor_power_pin icphd_power_on_seq[] = {
	{CAMP_AVDD,	LEVEL_POS,	5},
	{CAMP_PWD,	LEVEL_NEG,	0},
	{CAMP_RST,	LEVEL_POS,	100},
	{CAMP_END}
};
static struct sensor_power_pin icphd_power_off_seq[] = {
	{CAMP_RST,	LEVEL_NEG,	0},
	{CAMP_PWD,	LEVEL_POS,	0},
	{CAMP_AVDD,	LEVEL_NEG,	50},
	{CAMP_END}
};

static struct sensor_power_pin ov9740_power_on_seq[] = {
	{CAMP_AVDD,	LEVEL_POS,	5},
	{CAMP_PWD,	LEVEL_NEG,	1},
	{CAMP_END}
};
static struct sensor_power_pin ov9740_power_off_seq[] = {
	{CAMP_PWD,	LEVEL_POS,	0},
	{CAMP_AVDD,	LEVEL_NEG,	0},
	{CAMP_END}
};

static struct layout_mapping cam_layout_dkb21[] = {
	[CAMP_AVDD]	= {"AVDD_2V8",	PIN_TYPE_LDO, \
			{.ldo = "v_cam"}, LEVEL_HIGH, LEVEL_LOW, 2800000},
	[CAMP_AFVCC]	= {"AFVCC_3V3",	PIN_TYPE_LDO, \
			{"v_cam_af_vcc"}, LEVEL_HIGH, LEVEL_LOW, 3300000},
	[CAMP_PWD_MAIN]	= {"MAIN_PWDN",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO25}, LEVEL_HIGH, LEVEL_LOW},
	[CAMP_RST_MAIN]	= {"MAIN_RST",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO24}, LEVEL_HIGH, LEVEL_LOW},
	[CAMP_PWD_SUB]	= {"SUB_PWDN",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO26}, LEVEL_HIGH, LEVEL_LOW},
};

static struct layout_mapping cam_layout_dkb20[] = {
	[CAMP_AFVCC]	= {"AFVCC_3V3",	PIN_TYPE_LDO, \
			{.ldo = "v_cam"}, LEVEL_HIGH, LEVEL_LOW, 3300000},
	[CAMP_PWD_MAIN]	= {"MAIN_PWDN",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO25}, LEVEL_HIGH, LEVEL_LOW},
	[CAMP_RST_MAIN]	= {"MAIN_RST",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO24}, LEVEL_HIGH, LEVEL_LOW},
	[CAMP_PWD_SUB]	= {"SUB_PWDN",	PIN_TYPE_GPIO, \
			{.gpio = MFP_PIN_GPIO26}, LEVEL_HIGH, LEVEL_LOW},
};

static __u8 main_sensor_mapping[] = {
	[CAMP_AVDD]	= CAMP_AVDD,
	[CAMP_AFVCC]	= CAMP_AFVCC,
	[CAMP_PWD]	= CAMP_PWD_MAIN,
	[CAMP_RST]	= CAMP_RST_MAIN,
};

static __u8 sub_sensor_mapping[] = {
	[CAMP_AVDD]	= CAMP_AVDD,
	[CAMP_AFVCC]	= CAMP_AFVCC,
	[CAMP_PWD]	= CAMP_PWD_SUB,
};

struct layout_mapping *g_board;

static int cam_sensor_power(struct device *dev, int flag)
{
	struct soc_camera_link *icl = dev->platform_data;
	struct sensor_platform_data *pdata = icl->priv;
	struct sensor_power_pin *seq;
	struct layout_mapping *pin;
	int ret = 0, level;
	__u8 *pos_map;

	if (flag == 1)
		seq = pdata->power_on_seq;
	else
		seq = pdata->power_off_seq;
	/* If no power sequence is required, time to get out of this mess */
	if (seq == NULL)
		return 0;
	/* Front sensor use sub sensor pinmux */
	if (pdata->mount_pos & SENSOR_POS_FRONT)
		pos_map = sub_sensor_mapping;
	else
		pos_map = main_sensor_mapping;
next:
	if (seq->id >= CAMP_END)
		return 0;
	pin = g_board + pos_map[seq->id];
	if (pin->type == PIN_TYPE_NULL) {
		/* This pin is NA on this board */
		printk(KERN_DEBUG "cam: power: pin %d is empty " \
						"on this board\n", seq->id);
		seq++;
		goto next;
	}
	if (pin->handle.gpio != 0)
		goto get_level;

	/* Initialize ping for 1st use */
	switch (pin->type) {
	case PIN_TYPE_GPIO:
		pin->handle.gpio = mfp_to_gpio(pin->layout.gpio);
		if (!gpio_request(pin->handle.gpio, pin->name)) {
			printk(KERN_ERR "cam: power: failed to request %s" \
				" at %d\n", pin->name, pin->layout.gpio);
			/* Erase this pin */
			pin = NULL;
			return -EBUSY;
		}
		printk(KERN_INFO "cam: power: register GPIO %d as %s\n", \
					pin->layout.gpio, pin->name);
		break;
	case PIN_TYPE_LDO:
		pin->handle.ldo = regulator_get(NULL, pin->layout.ldo);
		if (IS_ERR(pin->handle.ldo)) {
			printk(KERN_ERR "cam: power: " \
					"failed to request %s\n", pin->name);
			pin = NULL;
			return -ENODEV;
		}
		/* Set the output voltage */
		if (pin->init_data)
			regulator_set_voltage(pin->handle.ldo, \
						pin->init_data, pin->init_data);
		printk(KERN_INFO "cam: power: register LDO '%s' as %s\n", \
					pin->layout.ldo, pin->name);
		break;
	}
	/* Yes, we never release these gpio or ldo. Because:
	 * if the pin is dedicated for camera, we should monopolize it
	 * if the pin is shared for others, we should never change it
	 */
get_level:
	if (seq->value == LEVEL_POS)
		level = pin->level_on;
	else
		level = pin->level_off;

	switch (pin->type) {
	case PIN_TYPE_GPIO:
		ret = gpio_direction_output(pin->handle.gpio, level);
		printk(KERN_DEBUG "cam: power: change %s = %d\n", \
							pin->name, level);
		break;
	case PIN_TYPE_LDO:
		if (level == LEVEL_HIGH)
			ret = regulator_enable(pin->handle.ldo);
		else
			ret = regulator_disable(pin->handle.ldo);
		printk(KERN_DEBUG "cam: power: change %s = %d\n", \
							pin->name, level);
		break;
	case PIN_TYPE_END:
		return 0;
	default:
		return -EINVAL;
	}

	if (ret < 0) {
		printk(KERN_ERR "cam: power: error when change %s = %d\n", \
							pin->name, level);
		return ret;
	}
	if (seq->delay_ms > 0)
		msleep(seq->delay_ms);
	seq++;

	goto next;
}

/* Camera LDO */
static struct regulator *vcamera;
/* Camera sensor PowerDowN pins */
static int pwd_main, pwd_sub, pwd_isp;

#if defined(CONFIG_VIDEO_PXA955)
static int camera0_power(struct device *dev, int flag)
{

	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		/* Initialize AF_VCC */
		if (vcamera == NULL) {
			vcamera = regulator_get(NULL, "v_cam");
			if (IS_ERR(vcamera))
				return -ENODEV;
		}

		/* Driver SWPD pin for main cam */
		if (flag) {
			gpio_direction_output(pwd_main, 0);	/* enable */
			usleep_range(1000, 2000);
			regulator_enable(vcamera);
		} else {
			regulator_disable(vcamera);
			gpio_direction_output(pwd_main, 1);	/* disable */
		}
		return 0;
	/* Default for SAAR-C 2.5 and 3, carries M6MO+OV8820 and OV9740 */
	default:
		switch (flag) {
		case SENSOR_OPEN:
			gpio_direction_output(pwd_isp, 1);	/* enable */
			/* Wait 100ms here to let ISP boot up and working */
			msleep(100);
			break;
		case SENSOR_CLOSE:
			gpio_direction_output(pwd_isp, 0);	/* disable */
			mdelay(50);
			break;
		case ISP_SENSOR_OPEN:
			gpio_direction_output(pwd_main, 1);	/* enable */
			mdelay(50);
			break;
		case ISP_SENSOR_CLOSE:
			gpio_direction_output(pwd_main, 0);	/* disable */
			mdelay(50);
			break;
		}
		return 0;
	}

	return 0;
}

static int camera1_power(struct device *dev, int flag)
{
	switch (get_board_id()) {
	case OBM_SAAR_C2_NEVO_A0_V10_BOARD:
		/* Initialize AF_VCC */
		if (vcamera == NULL) {
			vcamera = regulator_get(NULL, "v_cam");
			if (IS_ERR(vcamera))
				return -ENODEV;
		}
		/* Driver SWPD pin for sub cam */
		if (flag) {
			gpio_direction_output(pwd_sub, 0);
			usleep_range(2000, 3000);
			regulator_enable(vcamera);
		} else {
			regulator_disable(vcamera);
			gpio_direction_output(pwd_sub, 1);
		}
		return 0;
	/* Default for SAAR-C 2.5 and 3, carries M6MO+OV8820 and OV9740 */
	default:
		if (flag) {
			gpio_direction_output(pwd_sub, 0);
			usleep_range(1000, 2000);
		} else {
			gpio_direction_output(pwd_sub, 1);
		}
		return 0;
	}

	return 0;
}

static struct i2c_board_info camera_i2c[] = {
	{
		I2C_BOARD_INFO("m6mo", 0x1F),
	},
	{
		I2C_BOARD_INFO("ov9740", 0x10),
	},
	{
		I2C_BOARD_INFO("ov5642", 0x3c),
	},
	{
		I2C_BOARD_INFO("ov7692", 0x3c),
	},
	{
		I2C_BOARD_INFO("icp-hd", 0x3D),
	},
	{
		I2C_BOARD_INFO("ov640", 0x24),
	},
	{
		I2C_BOARD_INFO("ov5640", 0x3c),
	},
};

static struct pxa95x_csi_dev csidev[] = {
	{
		.id		= 0,
		.irq_num	= 71,
		.reg_start	= 0x50020000,
	},
	{
		.id		= 1,
		.irq_num	= 58,
		.reg_start	= 0x50022000,
	},
};

enum {
	ID_M6MO_SAARC	= 0,
	ID_M6MO_DKB,
	ID_OV9740,
	ID_OV5642,
	ID_OV7692,
	ID_ICPHD,
	ID_OV640,
	ID_OV5640,
};

static struct sensor_platform_data camera_sensor[] = {
	[ID_M6MO_SAARC] = {/* M6MO on SaarC 3*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO18,	/* OV8820 PWD pin */
		.pin_irq	= MFP_PIN_GPIO102,	/* M6MO irq pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "SaarC 3",
	},
	[ID_M6MO_DKB] = {/* M6MO on DKB 2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO25,	/* OV8820 PWD pin */
		.pin_irq	= MFP_PIN_GPIO18,	/* M6MO irq pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "DKB 2",
	},
	[ID_OV9740] = {/* OV9740 on SaarC3 and DKB2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[1],	/* connected to CSI1 */
		.pin_pwdn	= MFP_PIN_GPIO26,
		.af_cap		= 0,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "SaarC 2/3",
		.power_on_seq	= ov9740_power_on_seq,
		.power_off_seq	= ov9740_power_off_seq,
	},
	[ID_OV5642] = {/* OV5642 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO25,
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "TRULY",
		.board_name	= "tavor",
		.power_on_seq	= ov564x_power_on_seq,
		.power_off_seq	= ov564x_power_off_seq,
	},
	[ID_OV7692] = {/* OV7692 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_FRONT | SENSOR_RES_LOW,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE,
		.csi_ctlr	= &csidev[1],	/* connected to CSI1*/
		.pin_pwdn	= MFP_PIN_GPIO26,
		.af_cap		= 0,
		.mclk_mhz	= 26,
		.vendor_info	= "N/A",
		.board_name	= "SaarC 2",
	},
	[ID_ICPHD] = {/* Aptina on DKB 2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.pin_pwdn	= MFP_PIN_GPIO24,	/* M6MO PWD pin */
		.pin_aux	= MFP_PIN_GPIO25,	/* OV8820 PWD pin */
		.af_cap		= 1,
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "DKB 2",
		.power_on_seq	= icphd_power_on_seq,
		.power_off_seq	= icphd_power_off_seq,
	},
	[ID_OV640] = {/* OV ISP on DKB 2*/
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.mclk_mhz	= 26,
		.vendor_info	= "SUNNY",
		.board_name	= "DKB 2",
		.power_on_seq	= ov640_power_on_seq,
		.power_off_seq	= ov640_power_off_seq,
	},
	[ID_OV5640] = {/* OV5640 */
		.mount_pos	= SENSOR_USED \
					| SENSOR_POS_BACK | SENSOR_RES_HIGH,
		/* Configure this domain according to hardware connection */
		/* both of the 2 MIPI lanes are connected to pxa97x on EVB */
		.interface	= SOCAM_MIPI_1LANE | SOCAM_MIPI_2LANE,
		.csi_ctlr	= &csidev[0],	/* connected to CSI0 */
		.mclk_mhz	= 26,
		.vendor_info	= "TRULY",
		.board_name	= "tavor",
		.power_on_seq	= ov564x_power_on_seq,
		.power_off_seq	= ov564x_power_off_seq,
	},
};

static struct soc_camera_link iclink[] = {
	[ID_M6MO_SAARC] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "m6mo",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_M6MO_SAARC],
	},
	[ID_M6MO_DKB] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[0],
		.i2c_adapter_id		= 1,
		.power = camera0_power,
		.module_name		= "m6mo",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_M6MO_DKB],
	},
	[ID_OV9740] = {
		.bus_id			= 1, /* Must match with the camera ID */
		.board_info		= &camera_i2c[1],
		.i2c_adapter_id		= 1,
		.power			= cam_sensor_power,
		.module_name		= "ov9740",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV9740],
	},
	[ID_OV5642] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[2],
		.i2c_adapter_id		= 1,
		.power			= cam_sensor_power,
		.module_name		= "ov5642",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV5642],
	},
	[ID_OV7692] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[3],
		.i2c_adapter_id		= 2,
		.power = camera1_power,
		.module_name		= "ov7692",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV7692],
	},
	[ID_ICPHD] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[4],
		.i2c_adapter_id		= 1,
		.power			= cam_sensor_power,
		.module_name		= "icp-hd",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_ICPHD],
	},
	[ID_OV640] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[5],
		.i2c_adapter_id		= 1,
		.power			= cam_sensor_power,
		.module_name		= "ov640",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV640],
	},
	[ID_OV5640] = {
		.bus_id			= 0, /* Must match with the camera ID */
		.board_info		= &camera_i2c[6],
		.i2c_adapter_id		= 2,
		.power			= cam_sensor_power,
		.module_name		= "ov5640",
		/* When flags[31] is set, priv domain is pointing to a */
		/* sensor_platform_data to pass sensor parameters */
		.flags			= 0x80000000,
		.priv			= &camera_sensor[ID_OV5640],
	},
};

static struct platform_device __attribute__((unused)) camera[] = {
	[ID_M6MO_SAARC] = {
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[ID_M6MO_SAARC],
		},
	},
	[ID_M6MO_DKB] = {
		.name	= "soc-camera-pdrv",
		.id	= 0,
		.dev	= {
			.platform_data = &iclink[ID_M6MO_DKB],
		},
	},
	[ID_OV9740] = {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[ID_OV9740],
		},
	},
	[ID_OV5642] = {
		.name	= "soc-camera-pdrv",
		.id	= 5,
		.dev	= {
			.platform_data = &iclink[ID_OV5642],
		},
	},
	[ID_OV7692] = {
		.name	= "soc-camera-pdrv",
		.id	= 1,
		.dev	= {
			.platform_data = &iclink[ID_OV7692],
		},
	},
	[ID_ICPHD] = {
		.name	= "soc-camera-pdrv",
		.id	= 2,
		.dev	= {
			.platform_data = &iclink[ID_ICPHD],
		},
	},
	[ID_OV640] = {
		.name	= "soc-camera-pdrv",
		.id	= 3,
		.dev	= {
			.platform_data = &iclink[ID_OV640],
		},
	},
	[ID_OV5640] = {
		.name	= "soc-camera-pdrv",
		.id	= 4,
		.dev	= {
			.platform_data = &iclink[ID_OV5640],
		},
	},
};
#endif

static void __init init_cam(void)
{
	/* PWD pin GPIO initialize */
	switch (get_board_id()) {
	case OBM_TK_ARIEL_P10:
		pwd_main = mfp_to_gpio(MFP_PIN_GPIO25);
		pwd_sub = mfp_to_gpio(MFP_PIN_GPIO26);
		pwd_isp	= mfp_to_gpio(MFP_PIN_GPIO24);
		if (get_pmic_id() == PM800_CHIP_C0)
			g_board = cam_layout_dkb21;
		else
			g_board = cam_layout_dkb20;
		break;
	default: /* For SaarC 2.5 and 3*/
		BUG();
		break;
	}

	if (pwd_isp && gpio_request(pwd_isp, "CAM_ISP_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
			"gpio: %d\n", pwd_isp);
		pwd_isp = 0;
		return;
	}

	/* Camera hold these GPIO forever, should not be accquired by others */
	if (pwd_main && gpio_request(pwd_main, "CAM_HI_SENSOR_PWD")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", pwd_main);
		pwd_main = 0;
		return;
	}
	if (pwd_sub && gpio_request(pwd_sub, "CAM_LOW_SENSOR_PWD")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", pwd_sub);
		pwd_sub = 0;
		return;
	}

#if defined(CONFIG_VIDEO_PXA955)
	switch (get_board_id()) {
	case OBM_TK_ARIEL_P10:
		printk(KERN_NOTICE "cam: saarc: Probing camera on DKB 2\n");
#if defined(CONFIG_SOC_CAMERA_M6MO)
		platform_device_register(&camera[ID_M6MO_DKB]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV9740)
		platform_device_register(&camera[ID_OV9740]);
#endif
#if defined(CONFIG_SOC_CAMERA_ICPHD)
		platform_device_register(&camera[ID_ICPHD]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV640)
		platform_device_register(&camera[ID_OV640]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV5640)
		platform_device_register(&camera[ID_OV5640]);
#endif
#if defined(CONFIG_SOC_CAMERA_OV5642)
		platform_device_register(&camera[ID_OV5642]);
#endif
		break;
	default: /* For SaarC 2.5 and 3*/
		BUG();
		break;
	}
	platform_device_register(&pxa95x_device_cam0);
	platform_device_register(&pxa95x_device_cam1);
#endif

}
