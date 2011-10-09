/******************************************************************************
 * MODULE     : rohm_bh1772_driver.h
 * FUNCTION   : drivers of BH1772GLI of header
 * PROGRAMMED : SEI
 * DATE(ORG)  : Mar-30-2011(Mar-30-2011)
 * REMARKS    :
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011-2015 ROHM CO.,LTD.
 * HISTORY    :
 * 1.00A Mar-30-2011  SEI   Made a new file
 *****************************************************************************/
#ifndef _LINUX_I2C_ROHM_BH1772_DRIVER_H
#define _LINUX_I2C_ROHM_BH1772_DRIVER_H

#define BH1772_SLAVE_ADDR (0x38)

#define DRIVER_OK (0)
#define DRIVER_NG (-1)
#define DRIVER_NON_PARAM  (0x80000000)
#define DRIVER_READ_BLOCK (0x80000001)
#define MASK_SHORT (0xFFFF)
#define MASK_CHAR  (0xFF)
#define PS_INT_DISABLE  (0xFE)
#define ALS_INT_DISABLE  (0xFD)
#define PS_HYSTERESIS_MASK (0x10)
#define FLAG_OFF   (0)
#define FLAG_ON    (1)

/* BH1772 REGSTER */
#define REG_ALSCONTROL     (0x40)
#define REG_PSCONTROL      (0x41)
#define REG_ILED           (0x42)
#define REG_ALSPSMEAS      (0x44)
#define REG_PSMEASRATE     (0x45)
#define REG_ALSMEASRATE    (0x46)
#define REG_ALSDATA        (0x4C)
#define REG_ALSDATA_L      (0x4C)
#define REG_ALSDATA_H      (0x4D)
#define REG_ALSPSSTATUS    (0x4E)
#define REG_PSDATA         (0x4F)
#define REG_INTERRUPT      (0x52)
#define REG_PSTHLED        (0x53)
#define REG_ALSTHUP        (0x56)
#define REG_ALSTHUP_L      (0x56)
#define REG_ALSTHUP_H      (0x57)
#define REG_ALSTHLOW       (0x58)
#define REG_ALSTHLOW_L     (0x58)
#define REG_ALSTHLOW_H     (0x59)
#define REG_ALSSENSITIVITY (0x5A)
#define REG_PERSISTENCE    (0x5B)
#define REG_PSTHLLED       (0x5C)

#define POWER_OFF   (0)
#define POWER_ON    (1)
#define ALS_POWERON    (0x0F)
#define PS_POWERON  (0x03)
#define INT_OFF     (0)
#define INT_ON      (1)
#define TRRIGER_OFF (0)
#define TRRIGER_ON  (1)

/*REG_ALSCONTROL(0x40)*/
#define ALSRES_HMODE   (0 << 3)
#define ALSRES_MMODE   (1 << 3)
#define SWRST_NON      (0 << 2)
#define SWRST_EXCUTE   (1 << 2)
#define CTL_SATBY      (0 << 0)
#define CTL_FORCE      (2 << 0)
#define CTL_STAND      (3 << 0)
#define REG_ALSCTL_MAX (16)

/*REG_PSCONTROL(0x41)*/
/* the same define with REG_ALSCONTROL
#define CTL_SATBY     (0 << 0)
#define CTL_FORCE     (2 << 0)
#define CTL_STAND     (3 << 0) */
#define REG_PSCTL_MAX (0x4)

/*REG_ILED(0x42)*/
#define LEDCURRENT_005MA    (0)
#define LEDCURRENT_010MA    (1)
#define LEDCURRENT_020MA    (2)
#define LEDCURRENT_050MA    (3)
#define LEDCURRENT_100MA    (4)
#define LEDCURRENT_150MA    (5)
#define LEDCURRENT_200MA    (6)
#define REG_ILED_MAX        (8)
#define REG_ILED_DEF        (0x18)

/*REG_ALSPSMEAS(0x44)*/
#define BOTH_INGNORED      (0)
#define ALSTRRG_STR        (1 << 1)
#define PSTRRG_STR         (1 << 0)
#define REG_ALSPSMEASD_MAX (2)
#define ALSPSMEAS_INIMAX   (4)

/*REG_PSMEASRATE(0x45)*/
#define PSMEAS_0010MS      (0)
#define PSMEAS_0020MS      (1)
#define PSMEAS_0030MS      (2)
#define PSMEAS_0050MS      (3)
#define PSMEAS_0070MS      (4)
#define PSMEAS_0100MS      (5)
#define PSMEAS_0200MS      (6)
#define PSMEAS_0500MS      (7)
#define PSMEAS_1000MS      (8)
#define PSMEAS_2000MS      (9)
#define REG_PSMEASRATE_MAX (16)

/*REG_ALSMEASRATE(0x46)*/
#define ALSRATE_ENABLE    (0 << 7)
#define ALSRATE_DISABLE   (1 << 7)
#define ALSMEAS_0100MS     (0)
#define ALSMEAS_0200MS     (1)
#define ALSMEAS_0500MS     (2)
#define ALSMEAS_1000MS     (3)
#define ALSMEAS_2000MS     (4)
#define ALSMEASRATE_MAX    (8)

/*REG_INTERRUPT(0x4E)*/
#define LED1_INT_ACTIVE    (1 << 1)

/*REG_INTERRUPT(0x52)*/
#define PSH_THLETH         (0 << 4)
#define PSHL_THLETH        (1 << 4)
#define OUTPUT_ANYTIME     (0 << 3)
#define OUTPUT_LUTCH       (1 << 3)
#define POLA_ACTIVEL       (0 << 2)
#define POLA_INACTIVEL     (1 << 2)
#define MODE_NONUSE        (0)
#define MODE_PROXIMITY     (1)
#define MODE_ILLUMINANCE   (2)
#define MODE_BOTH          (3)
#define REG_INTERRUPT_MAX  (0x20)
#define INTR_VAL_MAKS      (0x1B)
/*#define INTR_VAL_MAKS_PS   (0x2B)*/

#define PS_TH_LED1			0x4E
#define PS_THL_LED1			0x2E
#define ALS_SEN_MAX			0x18
#define ALS_SEN_MIN			0x35
#define ERROR_RETURN(ret) if (ret < 0) return ret
#define delay_to_jiffies(d) ((d) ? msecs_to_jiffies(d) : 1)

int bh1772_driver_init(unsigned char led, unsigned char als_ps_me,
		       unsigned char ps_rate, unsigned char als_rate,
		       unsigned char interrupt, unsigned char ps_th_h,
		       unsigned short als_th_up, unsigned short als_th_low,
		       unsigned char als_sems, unsigned char persistence,
		       unsigned char ps_th_l, struct i2c_client *client);
int bh1772_driver_close(struct i2c_client *client);
int bh1772_driver_als_power_on(unsigned char power_als,
			       struct i2c_client *client);
int bh1772_driver_ps_power_on(unsigned char power_ps, unsigned char interrupt,
			      struct i2c_client *client);
int bh1772_driver_read_illuminance(unsigned short *data,
				   struct i2c_client *client);
int bh1772_driver_read_proximity(unsigned char *data,
				 struct i2c_client *client);
int bh1772_driver_read_datum(unsigned short *als_data, unsigned char *ps_data,
			     unsigned char *status, struct i2c_client *client);
int bh1772_driver_write_iled(unsigned char data, struct i2c_client *client);
int bh1772_driver_write_trigger(unsigned char als_trigger,
				unsigned char ps_trigger,
				struct i2c_client *client);
int bh1772_driver_write_ps_meas_rate(unsigned char data,
				     struct i2c_client *client);
int bh1772_driver_write_als_meas_rate(unsigned char rate, unsigned char disable,
				      struct i2c_client *client);
int bh1772_driver_write_interrupt(unsigned char hysteresis,
				  unsigned char output, unsigned char polarity,
				  unsigned char terminal,
				  struct i2c_client *client);
int bh1772_driver_write_ps_th_h(unsigned char data, struct i2c_client *client);
int bh1772_driver_write_als_th(unsigned short high_data,
			       unsigned short low_data,
			       struct i2c_client *client);
int bh1772_driver_write_ps_th_l(unsigned char data, struct i2c_client *client);
int bh1772_driver_write_persistence(unsigned char als_pers,
				    unsigned char ps_pers,
				    struct i2c_client *client);
int bh1772_driver_write_als_sens(unsigned char data, struct i2c_client *client);
int bh1772_driver_general_read(unsigned char adr_reg, unsigned char *addr,
			       unsigned char size, struct i2c_client *client);
void bh1772_driver_register_show(struct i2c_client *client);
#endif /* _LINUX_I2C_ROHM_BH1772_DRIVER_H */
