/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
#ifndef _SENSORTEK_STK3338_I2C_CONFIG_H_
#define _SENSORTEK_STK3338_I2C_CONFIG_H_

#include "sensortek_stk3338_i2c.h"
/*default(ps=stanby,als=stanby)*/
#define PS_ALS_SET_STATE            (EN_WAIT_DISABLE | EN_ALS_DISABLE | EN_PS_DISABLE)
//#define PS_ALS_SET_PSCTRL           (PRST_PS_16TIMES  | GAIN_PS_08TIMES  | IT_PS_0384US )
#define PS_ALS_SET_PSCTRL           (0x00)
//#define PS_ALS_SET_ALSCTRL1         (PRST_ALS_01TIMES | GAIN_ALS_04TIMES | IT_ALS_0100MS)
#define PS_ALS_SET_ALSCTRL1         (0x12)
//#define PS_ALS_SET_VCSELCTRL        (IRDR_VCSEL_025000UA | EN_CTIRFC_DISABLE | EN_CTIR_ENABLE)
#define PS_ALS_SET_VCSELCTRL        (0xA0)
//#define PS_ALS_SET_INTCTRL1         (INT_CTRL_PS_OR_ALS_ACTIVE | EN_INVALID_PS_INT_DISABLE  | EN_ALS_INT_DISABLE | PS_INT_MODE)
#define PS_ALS_SET_INTCTRL1         (0x01)
#define PS_ALS_SET_WAIT             (0x003C)
#define PS_ALS_SET_THDH_PS          (0x0400)
#define PS_ALS_SET_THDL_PS          (0x0100)
#define PS_ALS_SET_THDH_ALS         (0xFFFF)
#define PS_ALS_SET_THDL_ALS         (0x0000)
#define PS_ALS_SET_DATA_PS_OFFSET   (0x0000)
#define PS_ALS_SET_ALSCTRL2         (0x00)
#define PS_ALS_SET_INTELLI_WAIT_PS  (0x00)
#define PS_ALS_SET_SOFT_RESET       (0x00)
#define PS_ALS_SET_SYSCTRL1         (EN_BGIR_DISABLE)
//#define PS_ALS_SET_PSPDCTRL         (PS_PS3_DISABLE | PS_PS2_DISABLE | PS_PS1_DISABLE | PS_PS0_ENABLE)
#define PS_ALS_SET_PSPDCTRL         (0x7F)
#define PS_ALS_SET_INTCTRL2         (EN_ALS_DR_INT_DISABLE | EN_PS_DR_INT_DISABLE)
#define PS_ALS_SET_THD_PSOFF        (0xFFFF)
#define PS_ALS_SET_BGIR             (0x7F)

#define COEFFICIENT                 (4)
const long          data0_coefficient[COEFFICIENT] = {708, 852, 614, 207};
const long          data1_coefficient[COEFFICIENT] = {-16, 676, 380,  89};
const unsigned long judge_coefficient[COEFFICIENT] = { 20,  80, 140, 231};

#endif /* _SENSORTEK_STK3338_I2C_CONFIG_H_ */

