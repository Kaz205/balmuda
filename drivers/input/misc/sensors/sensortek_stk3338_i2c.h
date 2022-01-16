/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
#ifndef _SENSORTEK_STK3338_I2C_H_
#define _SENSORTEK_STK3338_I2C_H_

#include "sensortek_stk3338_i2c_if.h"

#define STK3338_DRIVER_VER ("0.1.0")
#define CALC_ERROR         (0x80000000)
#define SM_TIME_UNIT       (1000)
#define MN_TIME_UNIT       (1000000)
#define MASK_CHAR          (0xFF)
#define CLR_LOW2BIT        (0xFC)
#define CLR_LOW4BIT        (0xF0)
#define INIT_MODE_MASK     (0x0F)
#define INIT_PS_MODE_MASK  (0x40)
#define INIT_ALS_MODE_MASK (0x80)
#define UNRELATEDNESS      (0xFF)
#define IRQ_NON_USE        (0)
#define IRQ_USE            (1)
#define MASK_LONG          (0xFFFFFFFF)
#define PS_FLAG_MASK       (0b00000010)
#define HOST_ENABLE        (1)
#define HOST_DISABLE       (0)

#define _ALS_BIG_ENDIAN_   (1)
#ifdef _ALS_BIG_ENDIAN_
#define CONVERT_TO_BE(value) ((((value) >> 8) & 0xFF) | (((value) << 8) & 0xFF00))
#else
#define CONVERT_TO_BE(value) (value)
#endif

/* structure to read data value from sensor */
typedef struct {
    unsigned short data_ps;          /* data value of PS data from sensor        */
    unsigned short data_als;         /* data value of ALS data from sensor       */
    unsigned short data_als1;        /* data value of ALS1 data from sensor      */
    unsigned short data_c;           /* data value of CLEAR data from sensor     */
} READ_DATA_BUF;

typedef struct {
    unsigned short data_als;         /* data value of ALS data from sensor       */
    unsigned short data_als1;        /* data value of ALS1 data from sensor      */
    unsigned short data_c;           /* data value of CLEAR data1 from sensor    */
} READ_DATA_ALS_BUF;

typedef struct {
    unsigned char  ps_ctrl;          /* data value of PS control from sensor     */
    unsigned short data_ps;          /* data value of PS data from sensor        */
    unsigned char  flag_ps;          /* data value of PS_INT_TH_FLAG from sensor */
} READ_DATA_PS_BUF;

/* structure to set initial value to sensor */
typedef struct {
    unsigned char  state;            /* value of PS and ALS state                */
    unsigned char  psctrl;           /* value of PS control                      */
    unsigned char  alsctrl1;         /* value of ALS control 1                   */
    unsigned char  vcselctrl;        /* value of VCSEL control                   */
    unsigned char  intctrl1;         /* interruption setting value1              */
    unsigned char  wait;             /* value of wait control                    */
    unsigned short thdh_ps;          /* threshold value of high level for PS     */
    unsigned short thdl_ps;          /* threshold value of low level for PS      */
    unsigned short thdh_als;          /* threshold value of high level for PS     */
    unsigned short thdl_als;          /* threshold value of low level for PS      */
    unsigned short data_ps_offset;   /* offset for PS data                       */
    unsigned char  alsctrl2;         /* value of ALS control 2                   */
    unsigned char  intelli_wait_ps;  /* INTELLIGENT WAIT                         */
    unsigned char  sysctrl1;         /* value of system control 1                */
    unsigned char  pspdctrl;         /* Enable of PS0-PS3                        */
    unsigned char  intctrl2;         /* interruption setting value2              */
    unsigned short thd_psoff;        /* value of PS threshold offset             */
    unsigned char  thd_bgir;         /* threshold value of strong IR protect     */
} INIT_ARG;

/* structure to read state value from sensor */
typedef struct {
    unsigned char als_state;         /* state value of ALS from sensor           */
    unsigned char ps_state;          /* state value of PS from sensor            */
} PWR_ST;

/* structure to activate sensor */
typedef struct {
    unsigned char power_als;         /* value of whether to start ALS or not     */
    unsigned char power_ps;          /* value of whether to start PS or not      */
    unsigned char intr;              /* value of whether to use interrupt or not */
} POWERON_ARG;

enum light_source {
	LIGHT_CANDESCENT = 0,
	LIGHT_INTERMEDIATE,
	LIGHT_SOLAR,
	LIGHT_LED_BULB,
	LIGHT_FLUORESCENT,
	LIGHT_OVERFLOW,
	LIGHT_SRC_MAX,
};

enum psals_pinctrl_id {
	PSALS_PIN_DEFAULT = 0,
	PSALS_PIN_ACTIVE,
	PSALS_PIN_SUSPEND,
	PSALS_PIN_VPROX_32_ACTIVE,
	PSALS_PIN_VPROX_32_SUSPEND,
	PSALS_PIN_VPROX_18_ACTIVE,
	PSALS_PIN_VPROX_18_SUSPEND,
	PSALS_PIN_ID_MAX
};

typedef struct {
    unsigned long  lux;
    unsigned long  als;
    unsigned long  als1;
    unsigned long  c;
    unsigned char  gain_als;
    unsigned char  gain_c;
    unsigned short als_output;
    unsigned short als1_output;
    unsigned short c_output;
    unsigned long  ratio;
    enum light_source src;
    unsigned short alpha;
    unsigned short beta;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;


typedef struct {
    /* register STATE */
    bool            en_intelli_prst;
    bool            en_wait;
    bool            en_als;
    bool            en_ps;
    /* register PSCTRL */
    unsigned char   prst_ps;
    unsigned char   gain_ps;
    unsigned short  it_ps;
    /* register ALSCTRL1 */
    unsigned char   prst_als;
    unsigned char   gain_als;
    unsigned short  it_als;
    /* register VCSELCTRL */
    unsigned char   irdr_vcsel;
    unsigned char   en_ctirfc;
    unsigned char   en_ctir;
    /* register INTCTRL1 */
    bool            int_ctrl;
    bool            en_invalid_ps_int;
    bool            en_als_int;
    bool            ps_int_mode;
    bool            ps_nf_mode;
    bool            en_ps_int;
    /* register WAIT */
    unsigned char   wait;
    /* register FLAG */
    bool            flg_als_dr; 
    bool            flg_ps_dr;
    bool            flg_als_int; 
    bool            flg_ps_int;
    bool            flg_als_sat;
    bool            flg_invalid_ps_int; 
    bool            flg_nf; 
    /* register DATA_PS_OFFSET */
    unsigned short  data_ps_offset; 
    /* register PDT_ID */
    unsigned char   pdt_id;
    /* register ALSCTRL2  */
    unsigned char   gain_c;
    /* register INTELLI_WAIT_PS  */
    unsigned char   intelli_wait_ps;
    /* register SYSCTRL1  */
    bool            en_bgir;
    /* register PSPDCTRL  */
    unsigned char   ps_sel;
    /* register INTCTRL2  */
    bool            en_als_dr_int;
    bool            en_ps_dr_int;
    /* register THD_PSOFF */
    unsigned short  thd_psoff;
    /* register THD_BGIR */
    unsigned char   thd_bgir;
} DEVICE_VAL;

/************ define parameter for register ************/
/* REG_STATE(0x00) */
//#define EN_INTELLI_PRST_ENABLE         (0b1 << 3)
//#define EN_INTELLI_PRST_DISABLE        (0b0 << 3)
#define EN_WAIT_ENABLE                 (0b1 << 2)
#define EN_WAIT_DISABLE                (0b0 << 2)
#define EN_ALS_ENABLE                  (0b1 << 1)
#define EN_ALS_DISABLE                 (0b0 << 1)
#define EN_PS_ENABLE                   (0b1 << 0)
#define EN_PS_DISABLE                  (0b0 << 0)

/* REG_PSCTRL(0x01) */
#define PRST_PS_01TIMES                (0b00 << 6)
#define PRST_PS_02TIMES                (0b01 << 6)
#define PRST_PS_04TIMES                (0b10 << 6)
#define PRST_PS_16TIMES                (0b11 << 6)
#define GAIN_PS_01TIMES                (0b00 << 4)
#define GAIN_PS_02TIMES                (0b01 << 4)
#define GAIN_PS_04TIMES                (0b10 << 4)
#define GAIN_PS_08TIMES                (0b11 << 4)
#define IT_PS_0096US                   (0b0000 << 0)
#define IT_PS_0192US                   (0b0001 << 0)
#define IT_PS_0384US                   (0b0010 << 0)
#define IT_PS_0768US                   (0b0011 << 0)
#define IT_PS_1540US                   (0b0100 << 0)
#define IT_PS_3070US                   (0b0101 << 0)
#define IT_PS_6140US                   (0b0110 << 0)
#define PRST_MASK                      (0b11 << 6)
#define GAIN_MASK                      (0b11 << 4)
#define IT_PS_MASK                     (0b1111 << 0)

/* REG_ALSCTRL1(0x02) */
#define PRST_ALS_01TIMES               (0b00 << 6)
#define PRST_ALS_02TIMES               (0b01 << 6)
#define PRST_ALS_04TIMES               (0b10 << 6)
#define PRST_ALS_08TIMES               (0b11 << 6)
#define GAIN_ALS_01TIMES               (0b00 << 4)
#define GAIN_ALS_04TIMES               (0b01 << 4)
#define GAIN_ALS_16TIMES               (0b10 << 4)
#define GAIN_ALS_64TIMES               (0b11 << 4)
#define IT_ALS_0025MS                  (0b0000 << 0)
#define IT_ALS_0050MS                  (0b0001 << 0)
#define IT_ALS_0100MS                  (0b0010 << 0)
#define IT_ALS_0200MS                  (0b0011 << 0)
#define IT_ALS_0400MS                  (0b0100 << 0)
#define IT_ALS_0800MS                  (0b0101 << 0)
#define IT_ALS_1600MS                  (0b0110 << 0)

/* REG_VCSELCTRL(0x03) */
#define IRDR_VCSEL_003125UA            (0b000 << 5)
#define IRDR_VCSEL_006250UA            (0b001 << 5)
#define IRDR_VCSEL_012500UA            (0b010 << 5)
#define IRDR_VCSEL_025000UA            (0b011 << 5)
#define EN_CTIRFC_ENABLE               (0b1 << 1)
#define EN_CTIRFC_DISABLE              (0b0 << 1)
#define EN_CTIR_ENABLE                 (0b1 << 0)
#define EN_CTIR_DISABLE                (0b0 << 0)

/* REG_INTCTRL1(0x04) */
#define INT_CTRL_PS_AND_ALS_ACTIVE     (0b1 << 7)
#define INT_CTRL_PS_OR_ALS_ACTIVE      (0b0 << 7)
#define EN_INVALID_PS_INT_ENABLE       (0b1 << 5)
#define EN_INVALID_PS_INT_DISABLE      (0b0 << 5)
#define EN_ALS_INT_ENABLE              (0b1 << 3)
#define EN_ALS_INT_DISABLE             (0b0 << 3)
#define PS_INT_MODE                    (0b001 << 0)

/* REG_FLAG(0x10) */
#define FLG_ALS_DR_CLEAR               (0b0 << 7)
#define FLG_PS_DR_CLEAR                (0b0 << 6)
#define FLG_ALS_INT_CLEAR              (0b0 << 5)
#define FLG_PS_INT_CLEAR               (0b0 << 4)
#define FLG_INVALID_ALS_SAT_CLEAR      (0b0 << 2)
#define FLG_INVALID_PS_INT_CLEAR       (0b0 << 1)
#define FLG_NF_CLEAR                   (0b0 << 0)
#define FLG_ALS_DR_MASK                (0b1 << 7)
#define FLG_PS_DR_MASK                 (0b1 << 6)
#define FLG_ALS_INT_MASK               (0b1 << 5)
#define FLG_PS_INT_MASK                (0b1 << 4)
#define FLG_INVALID_ALS_SAT_MASK       (0b1 << 2)
#define FLG_INVALID_PS_INT_MASK        (0b1 << 1)
#define FLG_NF_MASK                    (0b1 << 0)

/* REG_ALSCTRL2(0x4E) */
#define GAIN_C_01TIMES                 (0b00 << 4)
#define GAIN_C_04TIMES                 (0b01 << 4)
#define GAIN_C_16TIMES                 (0b10 << 4)
#define GAIN_C_64TIMES                 (0b11 << 4)

/* REG_SYSCTRL1(0xA0)  */
#define EN_BGIR_ENABLE                 (0b1 << 4)
#define EN_BGIR_DISABLE                (0b0 << 4)

/* REG_PSPDCTRL(0xA1)  */
#define PS_PS3_ENABLE                  (0b1 << 3)
#define PS_PS3_DISABLE                 (0b0 << 3)
#define PS_PS2_ENABLE                  (0b1 << 2)
#define PS_PS2_DISABLE                 (0b0 << 2)
#define PS_PS1_ENABLE                  (0b1 << 1)
#define PS_PS1_DISABLE                 (0b0 << 1)
#define PS_PS0_ENABLE                  (0b1 << 0)
#define PS_PS0_DISABLE                 (0b0 << 0)

/* REG_INTCTRL2(0xA5)  */
#define EN_ALS_DR_INT_ENABLE           (0b1 << 1)
#define EN_ALS_DR_INT_DISABLE          (0b0 << 1)
#define EN_PS_DR_INT_ENABLE            (0b1 << 0)
#define EN_PS_DR_INT_DISABLE           (0b0 << 0)

/* REG_THD_BGIR(0xAA) */
#define THD_BGIR_MAX                   (0x7F)
#define THD_BGIR_DEF                   (0x64)

/* moved mode of ALS or PS  */
#define CTL_STANDBY         (0)
#define CTL_STANDALONE      (1)

/* REG_PSTH(0x4B) */
#define REG_PSTH_MAX        (0xFFFF)

/* REG_PSTL(0x4D) */
#define REG_PSTL_MAX        (0xFFFF)


#define ALS_ON_DELAY_MS     300
#define ALS_DATA_DELAY_MS   250

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void);
void ps_als_exit(void);
bool als_sensor_get_en_first(void);
ssize_t als_val_show(char *buf);
ssize_t ps_val_show(char *buf);
ssize_t als_status_show(char *buf);
ssize_t ps_status_show(char *buf);
ssize_t als_d0d1_show(char *buf);
ssize_t als_imit_show(char *buf);
ssize_t als_imit_store(const char *buf);
ssize_t als_properties_store(const char *buf);
ssize_t als_properties_show(char *buf);
ssize_t ps_properties_store(const char *buf);
ssize_t ps_properties_show(char *buf);
ssize_t ps_valid_show(char *buf);
ssize_t ps_valid_store(unsigned int valid);
ssize_t als_flush_store(unsigned int flush);
ssize_t ps_flush_store(unsigned int flush);
int32_t als_get_initialize_state(void);
int32_t ps_get_initialize_state(void);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
int32_t ps_sensor_activate(bool enable);
int32_t als_sensor_activate(bool enable);
uint32_t ps_sensor_get_count(void);
void ps_calib_start(void);
void ps_get_threshold(uint16_t* th_detect, uint16_t* th_no_detect, uint16_t* distance_th);
void prox_cal_result_set(int8_t result);
void prox_cal_interrupt(int8_t cal_result);
bool ps_get_thresh_adj(void);
#endif /* _SENSORTEK_STK3338_I2C_H_ */
