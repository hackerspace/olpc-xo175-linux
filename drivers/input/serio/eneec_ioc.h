//
// Copyright (C) ENE TECHNOLOGY INC. 2009

#ifndef _ENEEC_IOC_H_
#define _ENEEC_IOC_H_

#define ENEEC_IOC_MAGIC             ('e' + 0x80)

#define ENEEC_IOC_GET_CHIPID        _IOR (ENEEC_IOC_MAGIC,  1, unsigned long)  // *arg=output chipIdL-chipIdH-revId.
#define ENEEC_IOC_ENTER_CODE_IN_RAM _IO  (ENEEC_IOC_MAGIC,  2)
#define ENEEC_IOC_EXIT_CODE_IN_RAM  _IO  (ENEEC_IOC_MAGIC,  3)
#define ENEEC_IOC_READ_REG          _IOWR(ENEEC_IOC_MAGIC, 11, unsigned short) // *arg=input regL-regH, also, =output byte.
#define ENEEC_IOC_WRITE_REG         _IOW (ENEEC_IOC_MAGIC, 12, unsigned long)  // *arg=input regL-regH-byte.

#define LID_SWITCH	0x23    //LID state
#define AC_IN		0x24    //AC IN/OUT state
#define MB_TYPE		0x30 	//Mimas mother board = 'M' Titan MB ='T'
#define VERSION_1 	0x31
#define VERSION_2	0x32
#define RSOC            0x40    //Relative state of charge(%)
#define CHARGE_C_L      0x41    //Charge Current Low byte(mA)
#define CHARGE_C_H      0x42    //Charge Current High byte(mA)
#define CHARGE_V_L      0x43    //Charge Voltage Low byte(mV)
#define CHARGE_V_H      0x44    //Charge Voltage High byte(mV)
#define CURRENT_L       0x49    //Current Low byte(mA)
#define CURRENT_H       0x4A    //Current High byte(mA)
#define VOLTAGE_L       0x4B    //Voltage Low byte(mV)
#define VOLTAGE_H       0x4C    //Voltage High byte(mV)
#define CAP_REMAIN_L    0x4D    //Remaining Capacity Low Byte(mAh)
#define CAP_REMAIN_H    0x4E    //Remaining Capacity High Byte(mAH)
#define TEMP_C          0x51    //Temperature tC.
#define MANUFACTURE_L   0x52    //Manufacture Access Low Byte
#define MANUFACTURE_H   0x53    //Manufacture Access High Byte
#define BATTERY_MODE_L  0x54    //Battery mode Low Byte
#define BATTERY_MODE_H  0x55    //Battery mode High Byte
#define AVG_CURRENT_L   0x56    //Average Current Low byte(mA)
#define AVG_CURRENT_H   0x57    //Average Current High byte(mA)
#define ASOC            0x5A    //Absolute State of charge(%)
#define CAP_FULL_L      0x5C    //FULL Charge Capacity Low Byte(mAh)
#define CAP_FULL_H      0x5D    //Full Charge Capacity High Byte(mAh)
#define CELL_1_V_L      0x60    //Cell 1 Voltage Low byte(mV)
#define CELL_1_V_H      0x61    //Cell 1 Voltage High byte(mV)
#define CELL_2_V_L      0x62    //Cell 2 Voltage Low byte(mV)
#define CELL_2_V_H      0x63    //Cell 2 Voltage High byte(mV)
#define CELL_3_V_L      0x64    //Cell 3 Voltage Low byte(mV)
#define CELL_3_V_H      0x65    //Cell 3 Voltage High byte(mV)
#define CELL_4_V_L      0x66    //Cell 4 Voltage Low byte(mV)
#define CELL_4_V_H      0x67    //Cell 4 Voltage High byte(mV)
#define MAX_ERROR_L     0x68
#define MAX_ERROR_H     0x69
#define CAP_DESIGN_L    0x74    //Design Capacity Low Byte(mAh)
#define CAP_DESIGN_H    0x75    //Design Capacity High Byte(mAH)


#endif // _ENEEC_IOC_H_
