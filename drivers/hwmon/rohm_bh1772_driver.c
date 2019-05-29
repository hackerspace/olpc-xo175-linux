/******************************************************************************
 * MODULE     : rohm_bh1772_driver.c
 * FUNCTION   : drivers of BH1772GLC
 * PROGRAMMED : SEI
 * DATE(ORG)  : Mar-30-2011(Mar-30-2011)
 * REMARKS    :
 * C-FORM     : 1.00A
 * COPYRIGHT  : Copyright (C) 2011-2015 ROHM CO.,LTD.
 * HISTORY    :
 * 1.00A Mar-30-2011  SEI   Made a new file
 *****************************************************************************/
#include <linux/i2c.h>
#include <linux/rohm_bh1772_driver.h>
#include <asm/irq.h>
#include <linux/interrupt.h>
#include <linux/rohm_als_ps_driver.h>
static int _make_result_data(int *data, unsigned long mask);
static void _set_slave_address(struct i2c_client *client);

typedef struct {
	unsigned char als_data_low;
	unsigned char als_data_high;
	unsigned char state;
	unsigned char ps_data;
} MULTI_READ_DATA;

typedef struct {
	unsigned short high;
	unsigned short low;
} ALS_THRE_DATA;

typedef struct {
	unsigned char led;
	unsigned char reserved1;
	unsigned char als_ps_me;
	unsigned char ps_rate;
	unsigned char als_rate;
	unsigned char interrupt;
	unsigned char ps_th_h;
	unsigned char reserved6;
	unsigned char reserved7;
	unsigned char alsth_hl;
	unsigned char alsth_hh;
	unsigned char alsth_ll;
	unsigned char alsth_lh;
	unsigned char als_sems;
	unsigned char persistence;
	unsigned char ps_th_l;
} INIT_WRITE_DATA;

void bh1772_driver_register_show(struct i2c_client *client)
{
	printk(KERN_INFO "BH1771 reg0x40 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x40));
	printk(KERN_INFO "BH1771 reg0x41 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x41));
	printk(KERN_INFO "BH1771 reg0x42 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x42));
	printk(KERN_INFO "BH1771 reg0x43 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x43));
	printk(KERN_INFO "BH1771 reg0x44 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x44));
	printk(KERN_INFO "BH1771 reg0x45 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x45));
	printk(KERN_INFO "BH1771 reg0x46 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x46));
	printk(KERN_INFO "BH1771 reg0x52 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x52));
	printk(KERN_INFO "BH1771 reg0x53 = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x53));
	printk(KERN_INFO "BH1771 reg0x5A = 0x%x\n",
	       i2c_smbus_read_byte_data(client, 0x5A));
}

/******************************************************************************
 * NAME       : bh1772_driver_init
 * FUNCTION   : initiailze BH1772
 * PROCESS    : set the initalization value to registers
 * INPUT      : led         - LED current
 *            : als_ps_me   - ALS PS MEAS
 *            : ps_rate     - PS MEAS RATE
 *            : als_rate    - ALS MEAS RATE
 *            : interrupt   - INTERRUPT
 *            : ps_th_h     - PS Interrupt high threshold
 *            : als_th_up   - ALS Interrupt up threshold
 *            : als_th_low  - ALS Interrupt low threshold
 *            : als_sems    - ALS sensitivity
 *            : persistence - ALS and PS persistence
 *            : ps_th_l     - PS Interrupt low threshold
 *            : client      - structre that OS provides as the standard
 * RETURN     : result      - OK      : DRIVER_OK(0)
 *            :             - NG      : DRIVER_NG(-1)
 *            :             - PRAM NG : DRIVER_NON_PARAM
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_init(unsigned char led, unsigned char als_ps_me,
		       unsigned char ps_rate, unsigned char als_rate,
		       unsigned char interrupt, unsigned char ps_th_h,
		       unsigned short als_th_up, unsigned short als_th_low,
		       unsigned char als_sems, unsigned char persistence,
		       unsigned char ps_th_l, struct i2c_client *client)
{

	int result;
	struct rohm_ls_data *ls = i2c_get_clientdata(client);
	INIT_WRITE_DATA init_write_data;
	result = DRIVER_OK;
	/* execute software reset */
	result = bh1772_driver_close(client);
	if (DRIVER_OK != result)
		return result;

	/* not check prameter is als_rate, ps_th_h, als_th_up, als_th_low, als_sems, persistence, ps_th_l */
	/* check the parameter of current LED */
	if (led >= REG_ILED_MAX)
		return DRIVER_NON_PARAM;
	/* check the parameter of ALS PS MEAS RATE */
	if (als_ps_me >= ALSPSMEAS_INIMAX)
		return DRIVER_NON_PARAM;
	/* check the parameter of PS MEAS RATE */
	if (ps_rate >= REG_PSMEASRATE_MAX)
		return DRIVER_NON_PARAM;
	/* check the parameter of interrupt */
	if (interrupt >= REG_INTERRUPT_MAX)
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);
	init_write_data.led = led;	/* recommend value is 0x04 */
	init_write_data.reserved1 = 0x3;
	init_write_data.als_ps_me = als_ps_me;	/* recommend value is 0x00 */
	init_write_data.ps_rate = ps_rate;	/* recommend value is 0x04 */
	init_write_data.als_rate = als_rate;	/* recommend value is 0x80 */
	init_write_data.interrupt = interrupt;	/* recommend value is 0x09 */
	init_write_data.ps_th_h = ps_th_h;	/* recommend value is 0x5E */
	init_write_data.reserved6 = 0xff;
	init_write_data.reserved7 = 0xff;
	init_write_data.alsth_hl = (als_th_up & MASK_CHAR);	/* recommend value is 0xFF */
	init_write_data.alsth_hh = (als_th_up >> 8);	/* recommend value is 0x00 */
	init_write_data.alsth_ll = (als_th_low & MASK_CHAR);	/* recommend value is 0x00 */
	init_write_data.alsth_lh = (als_th_low >> 8);	/* recommend value is 0x00 */
	init_write_data.als_sems = als_sems;	/* recommend value is 0x35 */
	init_write_data.persistence = persistence;	/* recommend value is 0x04 */
	init_write_data.ps_th_l = ps_th_l;	/* recommend value is 0x00 */
	result =
	    i2c_smbus_write_i2c_block_data(client, REG_ILED,
					   sizeof(INIT_WRITE_DATA),
					   (unsigned char *)&init_write_data);
	if (result < 0)
		return result;
	ls->interrupt = interrupt;
#ifdef _SEI_DEBUG
	bh1772_driver_register_show(client);
#endif
	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_close
 * FUNCTION   : close BH1772
 * PROCESS    : set the closing value to registers
 * INPUT      : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_close(struct i2c_client *client)
{
	int result;
	/* set I2C slave address */
	_set_slave_address(client);

	result =
	    i2c_smbus_write_byte_data(client, REG_ALSCONTROL, SWRST_EXCUTE);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_als_power_on
 * FUNCTION   : start or close ALS
 * PROCESS    : set the starting or closing value and interruption to registers.
 * INPUT      : power_als - ALS START or STOP
 *            : interrupt   - ALS Interrupt use or not use
 *            : client    - structre that OS provides as the standard
 * RETURN     : result    - OK : DRIVER_OK(0)
 *            :           - NG : return the error value in i2c function
 *            :           - PRAM NG : DRIVER_NON_PARAM
 * REMARKS    :
 * HISTORY    :
 *****************************************************************************/
int bh1772_driver_als_power_on(unsigned char power_als,
			       struct i2c_client *client)
{
	int result = 0;
	int als_ctl;
	struct rohm_ls_data *ls = i2c_get_clientdata(client);
	/* check the parameter of als power , ps power , als interrupt and ps interrupt */
	if ((power_als >= REG_ALSCTL_MAX))
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);
	als_ctl = power_als;
	if ((als_ctl & CTL_STAND) != CTL_SATBY) {
		result =
		    i2c_smbus_write_byte_data(client, REG_ALSCONTROL, als_ctl);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
		if ((als_ctl & CTL_STAND) == CTL_FORCE) {
			result =
			    i2c_smbus_write_byte_data(client,
						      REG_ALSPSMEAS,
						      ALSTRRG_STR);
			if (result < 0) {
				/* i2c communication error */
				return result;
			}
		} else {
			mutex_lock(&ls->sensor_lock);
			cancel_delayed_work_sync(&ls->als_work);
			result =
			    i2c_smbus_write_byte_data(ls->client,
						      REG_ALSMEASRATE,
						      ls->als_meas_time);
			if (result < 0)
				return result;
			schedule_delayed_work(&ls->als_work,
					      delay_to_jiffies
					      (ls->als_poll_delay));
			mutex_unlock(&ls->sensor_lock);
		}
	} else {
		result =
		    i2c_smbus_write_byte_data(client, REG_ALSCONTROL, als_ctl);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
		mutex_lock(&ls->sensor_lock);
		cancel_delayed_work_sync(&ls->als_work);
		mutex_unlock(&ls->sensor_lock);
	}
#ifdef _SEI_DEBUG
	bh1772_driver_register_show(client);
#endif
	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_ps_power_on
 * FUNCTION   : start or close PS
 * PROCESS    : set the starting or closing value and interruption to registers.
 * INPUT      : power_ps  - PS START or STOP
 *            : interrupt    - PS Interrupt use or not use
 *            : client    - structre that OS provides as the standard
 * RETURN     : result    - OK : DRIVER_OK(0)
 *            :           - NG : return the error value in i2c function
 *            :           - PRAM NG : DRIVER_NON_PARAM
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_ps_power_on(unsigned char power_ps, unsigned char interrupt,
			      struct i2c_client *client)
{
	int result = 0;
	unsigned char ps_ctl;

	/* check the parameter of als power , ps power , als interrupt and ps interrupt */
	if ((power_ps >= REG_PSCTL_MAX) || (interrupt >= REG_INTERRUPT_MAX))
		return DRIVER_NON_PARAM;

	ps_ctl = power_ps;
	_set_slave_address(client);
	/* set I2C slave address */
	if (ps_ctl != CTL_SATBY) {
		result =
		    i2c_smbus_write_byte_data(client, REG_PSCONTROL, ps_ctl);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
		result = bh1772_driver_write_als_sens(0xFD, client);
		if (result < 0)
			return result;
		result =
		    i2c_smbus_write_byte_data(client, REG_INTERRUPT, interrupt);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
		if (interrupt & MODE_PROXIMITY) {
			result =
			    i2c_smbus_write_byte_data(client,
						      REG_PSTHLED, PS_TH_LED1);
			if (result < 0) {
				/* i2c communication error */
				return result;
			}
			if ((interrupt & PS_HYSTERESIS_MASK)) {
				result =
				    i2c_smbus_write_byte_data
				    (client, REG_PSTHLLED, PS_THL_LED1);
				if (result < 0) {
					/* i2c communication error */
					return result;
				}
			}
		}
		if (ps_ctl == CTL_FORCE) {
			result =
			    i2c_smbus_write_byte_data(client,
						      REG_ALSPSMEAS,
						      PSTRRG_STR);
			if (result < 0) {
				/* i2c communication error */
				return result;
			}
		}
	} else {
		result =
		    i2c_smbus_write_byte_data(client, REG_INTERRUPT, interrupt);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
		result =
		    i2c_smbus_write_byte_data(client, REG_PSCONTROL, ps_ctl);
		if (result < 0) {
			/* i2c communication error */
			return result;
		}
	}
#ifdef _SEI_DEBUG
	bh1772_driver_register_show(client);
#endif
	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_read_illuminance
 * FUNCTION   : read the value of illuminance in BH1772
 * PROCESS    : read the illuminance value from registers
 * INPUT      : data   - the value of illuminance
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_read_illuminance(unsigned short *data,
				   struct i2c_client *client)
{
	int result;
	int get_data;

	/* set I2C slave address */
	_set_slave_address(client);

	/* read register to BH1772GLC via i2c */
	get_data = i2c_smbus_read_word_data(client, REG_ALSDATA);

	/* check the return value */
	result = _make_result_data(&get_data, MASK_SHORT);

	/* set to the returned variable */
	*data = (unsigned short)get_data;

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_read_proximity
 * FUNCTION   : read the value of proximity in BH1772
 * PROCESS    : read the proximity value from registers
 * INPUT      : data   - the value of proximity
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_read_proximity(unsigned char *data, struct i2c_client *client)
{
	int result;
	int get_data;

	/* set I2C slave address */
	_set_slave_address(client);

	/* read register to BH1772GLC via i2c */
	get_data = i2c_smbus_read_byte_data(client, REG_PSDATA);

	/* check the return value */
	result = _make_result_data(&get_data, MASK_CHAR);

	/* set to the returned variable */
	*data = (unsigned char)get_data;

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_read_datum
 * FUNCTION   : read the value of proximity and illuminance and status in BH1772
 * PROCESS    : read the multi values from registers
 * INPUT      : als_data - the value of illuminance
 *            : ps_data  - the value of proximity
 *            : status   - the value of status
 *            : client   - structre that OS provides as the standard
 * RETURN     : result   - OK : DRIVER_OK(0)
 *            :          - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_read_datum(unsigned short *als_data, unsigned char *ps_data,
			     unsigned char *status, struct i2c_client *client)
{
	int result;
	int return_length;
	MULTI_READ_DATA multi;
	unsigned short data;

	/* set I2C slave address */
	_set_slave_address(client);

	/* block read */
	return_length =
	    i2c_smbus_read_i2c_block_data(client, REG_ALSDATA, sizeof(multi),
					  (unsigned char *)&multi);
	if (sizeof(multi) == return_length) {
		data = (multi.als_data_high << 8);
		data = data + multi.als_data_low;
		*als_data = data;
		*status = multi.state;
		*ps_data = multi.ps_data;
		result = DRIVER_OK;
	} else if (return_length < 0) {
		result = return_length;
	} else {
		result = DRIVER_READ_BLOCK;
	}

	if (multi.ps_data < PS_TH_LED1)
		printk(KERN_INFO "clear interrupt. BH1771 reg0x52 = 0x%x\n\n", i2c_smbus_read_byte_data(client, 0x52));	/*clear interrupt */
	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_iled
 * FUNCTION   : read the value of proximity and illuminance and status in BH1772
 * PROCESS    : read the multi values from registers
 * INPUT      : data   - LED current
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_iled(unsigned char data, struct i2c_client *client)
{
	int result;

	/* check the parameter of current LED */
	if (data >= REG_ILED_MAX)
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);

	/* read register to BH1772GLC via i2c */
	data = data + REG_ILED_DEF;
	result = i2c_smbus_write_byte_data(client, REG_ILED, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_trigger
 * FUNCTION   : write ALS/PS trigger
 * PROCESS    : set the value of trigger when BH1772 have forced mode
 * INPUT      : als_trigger - use or not use als trigger when BH1772 have forced mode
 *            : ps_trigger  - use or not use ps trigger when BH1772 have forced mode
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_trigger(unsigned char als_trigger,
				unsigned char ps_trigger,
				struct i2c_client *client)
{
	int result;
	unsigned char data;

	/* check the parameter of current LED */
	if (als_trigger > TRRIGER_ON)
		return DRIVER_NON_PARAM;
	/* check the parameter of ALS PS MEAS RATE */
	if (ps_trigger > TRRIGER_ON)
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);

	/* make data to write */
	data = (als_trigger << 1) + ps_trigger;

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_ALSPSMEAS, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_ps_meas_rate
 * FUNCTION   : set ps meas rate
 * PROCESS    : write value of ps meas rate
 * INPUT      : data   - PS MEAS RATE
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_ps_meas_rate(unsigned char data,
				     struct i2c_client *client)
{
	int result;

	/* check the parameter of PS MEAS RATE */
	if (data >= REG_PSMEASRATE_MAX)
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_PSMEASRATE, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_als_meas_rate
 * FUNCTION   : set als meas rate
 * PROCESS    : write value of als meas rate
 * INPUT      : data   - ALS MEAS RATE
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_als_meas_rate(unsigned char rate, unsigned char disable,
				      struct i2c_client *client)
{
	int result;
	unsigned char alsmeas_rate;

	/* check the parameter of ALS MEAS RATE */
	if (rate >= ALSMEASRATE_MAX)
		return DRIVER_NON_PARAM;
	/* check variable to use or not use  */
	if ((ALSRATE_DISABLE != disable) && (ALSRATE_ENABLE != disable))
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);

	alsmeas_rate = disable + rate;

	/* write register to BH1772GLC via i2c */
	result =
	    i2c_smbus_write_byte_data(client, REG_ALSMEASRATE, alsmeas_rate);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_interrupt
 * FUNCTION   : set interrupt register
 * PROCESS    : write value of interrupt
 * INPUT      : hysteresis - interrupt threshold hysteresis
 *            : output     - interrupt output mode
 *            : polarity   - interrput polarity
 *            : terminal   - interrupt mode
 *            : client     - structre that OS provides as the standard
 * RETURN     : result     - OK : DRIVER_OK(0)
 *            :            - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_interrupt(unsigned char hysteresis,
				  unsigned char output, unsigned char polarity,
				  unsigned char terminal,
				  struct i2c_client *client)
{
	int result;
	unsigned char int_data;

	/* check the parameter of hysteresis */
	if ((PSH_THLETH != hysteresis) && (PSHL_THLETH != hysteresis))
		return DRIVER_NON_PARAM;
	/* check the parameter of output */
	if ((OUTPUT_ANYTIME != output) && (OUTPUT_LUTCH != output))
		return DRIVER_NON_PARAM;
	/* check the parameter of polarity */
	if ((POLA_ACTIVEL != polarity) && (POLA_INACTIVEL != polarity))
		return DRIVER_NON_PARAM;
	/* check the parameter of terminal */
	if (terminal > MODE_BOTH)
		return DRIVER_NON_PARAM;
	/* set I2C slave address */
	_set_slave_address(client);

	int_data = hysteresis + output + polarity + terminal;

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_INTERRUPT, int_data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_ps_th_h
 * FUNCTION   : set ps interrupt standard of high level
 * PROCESS    : write value of interrupt standard
 * INPUT      : data   - value of interrupt threshold of high level
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_ps_th_h(unsigned char data, struct i2c_client *client)
{
	int result;

	/* set I2C slave address */
	_set_slave_address(client);

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_PSTHLED, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_als_th_up
 * FUNCTION   : set als interrupt standard of up and low
 * PROCESS    : write value of
 * INPUT      : high_data - value of interrupt threshold of high level
 *            : low_data - value of interrupt threshold of low level
 *            : client    - structre that OS provides as the standard
 * RETURN     : result    - OK : DRIVER_OK(0)
 *            :           - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_als_th(unsigned short high_data,
			       unsigned short low_data,
			       struct i2c_client *client)
{
	int result;
	ALS_THRE_DATA data;

	/* set I2C slave address */
	_set_slave_address(client);

	data.high = high_data;
	data.low = low_data;

	/* write register to BH1772GLC via i2c */
	result =
	    i2c_smbus_write_i2c_block_data(client, REG_ALSTHUP, sizeof(data),
					   (unsigned char *)&data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_ps_th_l
 * FUNCTION   : set ps interrupt standard of low level
 * PROCESS    : write value of interrupt standard
 * INPUT      : data   - value of interrupt threshold of low level
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_ps_th_l(unsigned char data, struct i2c_client *client)
{
	int result;

	/* set I2C slave address */
	_set_slave_address(client);

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_PSTHLLED, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_persistence
 * FUNCTION   : set als/ps persistence
 * PROCESS    : write value of als/ps persistence
 * INPUT      : als_pers - value of als persistence
 *            : ps_pers  - value of ps persistence
 *            : client   - structre that OS provides as the standard
 * RETURN     : result   - OK : DRIVER_OK(0)
 *            :          - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
#define PERSISTANCE_MAX (0x10)
#define ALS_PERSIS_SFT  (4)
int bh1772_driver_write_persistence(unsigned char als_pers,
				    unsigned char ps_pers,
				    struct i2c_client *client)
{
	int result;
	unsigned char data;

	/* check the parameter of hysteresis */
	if ((PERSISTANCE_MAX <= als_pers) || (PERSISTANCE_MAX <= ps_pers))
		return DRIVER_NON_PARAM;

	/* set I2C slave address */
	_set_slave_address(client);

	/* write register to BH1772GLC via i2c */
	data = (als_pers << ALS_PERSIS_SFT) + ps_pers;
	result = i2c_smbus_write_byte_data(client, REG_PERSISTENCE, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_write_als_sens
 * FUNCTION   : set als sensitivity
 * PROCESS    : write value of als sensitivity
 * INPUT      : data   - value of als sensitivity
 *            : client - structre that OS provides as the standard
 * RETURN     : result - OK : DRIVER_OK(0)
 *            :        - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_write_als_sens(unsigned char data, struct i2c_client *client)
{
	unsigned int result;

	/* set I2C slave address */
	_set_slave_address(client);

	/* write register to BH1772GLC via i2c */
	result = i2c_smbus_write_byte_data(client, REG_ALSSENSITIVITY, data);

	return result;
}

/******************************************************************************
 * NAME       : bh1772_driver_general_read
 * FUNCTION   : read general multi bytes
 * PROCESS    : read the multi bytes from specification address
 * INPUT      : adr_reg - top register address(start reading address)
 *            : addr    - pointer of receive data
 *            : size    - reading size
 *            : client  - structre that OS provides as the standard
 * RETURN     : result  - OK : DRIVER_OK(0)
 *            :         - NG : return the error value in i2c function
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-10-2011  SEI   Made a new function
 *****************************************************************************/
int bh1772_driver_general_read(unsigned char adr_reg, unsigned char *addr,
			       unsigned char size, struct i2c_client *client)
{
	int result;

	/* check the parameter of regster */
	if ((adr_reg < REG_ALSCONTROL) || (adr_reg > REG_PSTHLLED))
		return DRIVER_NON_PARAM;

	/* set I2C slave address */
	_set_slave_address(client);

	/* block read */
	result =
	    i2c_smbus_read_i2c_block_data(client, adr_reg, size,
					  (unsigned char *)addr);
	if (size == result)
		result = DRIVER_OK;
	else if (result > 0)
		result = DRIVER_READ_BLOCK;

	return result;
}

/******************************************************************************
 * NAME       : _make_result_data
 * FUNCTION   : make the returned value of function which has data
 * PROCESS    : make the returned value of function which has data
 * INPUT      : data - the returned value of function
 *            : mask - mask data
 * RETURN     : result - the returned value of function which is called
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-18-2011  SEI   Made a new function
 *****************************************************************************/
static int _make_result_data(int *data, unsigned long mask)
{
	int result;

	if (0 <= *data) {
		result = DRIVER_OK;
		*data = *data & mask;
	} else {
		result = *data;
		*data = 0;
	}

	return result;
}

/******************************************************************************
 * NAME       : _set_slave_address
 * FUNCTION   : set slave address
 * PROCESS    : set slave address
 * INPUT      : client  - structure  that OS provides as the standard
 * RETURN     : result - the returned value of function which is called
 * REMARKS    :
 * HISTORY    :
 * 1.00A FEB-18-2011  SEI   Made a new function
 *****************************************************************************/
static void _set_slave_address(struct i2c_client *client)
{
	/* set I2C slave address */
	client->addr = BH1772_SLAVE_ADDR;

	return;
}
