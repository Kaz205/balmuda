/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
#ifndef _SENSORTEK_STK3338_I2C_IF_H_
#define _SENSORTEK_STK3338_I2C_IF_H_

/************ definition to dependent on sensor IC ************/
#define STK3338_I2C_NAME          "stk3x5a_i2c"
#define STK3338_I2C_ADDRESS       (0x46)  //7 bits slave address 110 0111

/************ command definition of ioctl ************/
#define IOCTL_APP_SET_TIMER       (10)
#define IOCTL_APP_SET_PWRSET_ALS  (11)
#define IOCTL_APP_SET_PWRSET_PS   (12)
#define IOCTL_APP_SET_INTR_MODE   (13)
#define IOCTL_APP_SET_PS_TH_HIGH  (14)
#define IOCTL_APP_SET_ALS_TH_UP   (15)
#define IOCTL_APP_SET_ALS_TH_LOW  (16)
#define IOCTL_APP_SET_PERSISTENCE (18)
#define IOCTL_APP_SET_PS_TH_LOW   (19)
#define IOCTL_APP_SET_MEASUR_TIME (20)
#define IOCTL_APP_SET_GENERAL     (253)
#define IOCTL_APP_READ_GENERAL    (254)
#define IOCTL_APP_READ_DRIVER_VER (255)

/************ define register for IC ************/
/* MTK3338 REGSTER */
#define REG_STATE                   (0x00)
#define REG_PSCTRL                  (0x01)
#define REG_ALSCTRL1                (0x02)
#define REG_VCSELCTRL               (0x03)
#define REG_INTCTRL1                (0x04)
#define REG_WAIT                    (0x05)
#define REG_THDH_PS                 (0x06)
    #define REG_THDH_PS_MSB         (0x06)
    #define REG_THDH_PS_LSB         (0x07)
#define REG_THDL_PS                 (0x08)
    #define REG_THDL_PS_MSB         (0x08)
    #define REG_THDL_PS_LSB         (0x09)
#define REG_THDH_ALS                (0x0A)
    #define REG_THDH_ALS_MSB        (0x0A)
    #define REG_THDH_ALS_LSB        (0x0B)
#define REG_THDL_ALS                (0x0C)
    #define REG_THDL_ALS_MSB        (0x0C)
    #define REG_THDL_ALS_LSB        (0x0D)
#define REG_FLAG                    (0x10)
#define REG_DATA_PS                 (0x11)
    #define REG_DATA_PS_MSB         (0x11)
    #define REG_DATA_PS_LSB         (0x12)
#define REG_DATA_ALS                (0x13)
    #define REG_DATA_ALS_MSB        (0x13)
    #define REG_DATA_ALS_LSB        (0x14)
#define REG_DATA_ALS1               (0x17)
    #define REG_DATA_ALS1_MSB       (0x17)
    #define REG_DATA_ALS1_LSB       (0x18)
#define REG_DATA_C                  (0x1B)
    #define REG_DATA_C_MSB          (0x1B)
    #define REG_DATA_C_LSB          (0x1C)
#define REG_DATA_PS_OFFSET          (0x1D)
    #define REG_DATA_PS_OFFSET_MSB  (0x1D)
    #define REG_DATA_PS_OFFSET_LSB  (0x1E)
#define REG_PDT_ID                  (0x3E)
#define REG_ALSCTRL2                (0x4E)
#define REG_INTELLI_WAIT_PS         (0x4F)
#define REG_SOFT_RESET              (0x80)
#define REG_SYSCTRL1                (0xA0)
#define REG_PSPDCTRL                (0xA1)
#define REG_INTCTRL2                (0xA5)
#define REG_THD_PSOFF               (0xA8)
    #define REG_THD_PSOFF_MSB       (0xA8)
    #define REG_THD_PSOFF_LSB       (0xA9)
#define REG_THD_BGIR                (0xAA)

/* for power_on and _rupt */
#define PS_ALS_DISABLE              (0)
#define PS_ALS_ENABLE               (1)

/* structure to activate sensor */
typedef struct {
    unsigned char power_als; /* value of whether to start ALS or not */
    unsigned char intr;      /* value of whether to use interrupt or not */
} POWERON_ALS_ARG;

/* structure to activate sensor */
typedef struct {
    unsigned char power_ps;  /* value of whether to start PS or not */
    unsigned char intr;      /* value of whether to use interrupt or not */
} POWERON_PS_ARG;

/* structure to read register value from sensor */
typedef struct {
    unsigned char adr_reg;   /* start register value */
    unsigned char *addr;     /* address to save value which read from sensor */
    unsigned char size;      /* length to read */
} GENREAD_ARG;

typedef struct {
	unsigned char state;
	unsigned char psctrl;
	unsigned char alsctrl1;
	unsigned char vcselctrl;
	unsigned char intctrl1;
	unsigned char wait;

	unsigned char thdh_ps_msb;
	unsigned char thdh_ps_lsb;

	unsigned char thdl_ps_msb;
	unsigned char thdl_ps_lsb;
	
	unsigned char thdh_als_msb;
	unsigned char thdh_als_lsb;
	
    unsigned char thdl_als_msb;
	unsigned char thdl_als_lsb;

    unsigned char flag;

    unsigned char data_ps_msb;
	unsigned char data_ps_lsb;

    unsigned char data_als_msb;
	unsigned char data_als_lsb;

    unsigned char data_c_msb;
	unsigned char data_c_lsb;

    unsigned char data_ps_offset_msb;
    unsigned char data_ps_offset_lsb;

    unsigned char pdt_id;
    unsigned char alsctrl2;
    unsigned char intelli_wait_ps;
    unsigned char soft_reset;
    unsigned char sysctrl1;
    unsigned char pspdctrl;
    unsigned char intctrl2;

    unsigned char thd_psoff_msb;
    unsigned char thd_psoff_lsb;

	unsigned char thd_bgir;
} STK3338_REGS;

#endif /* _SENSORTEK_STK3338_I2C_IF_H_ */

