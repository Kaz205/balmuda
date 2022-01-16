/* This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 * (C) 2020 KYOCERA Corporation
 * (C) 2021 KYOCERA Corporation
 */
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/sort.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#ifdef CONFIG_USE_SENSOR_KC_BOARD
#include <linux/kc_board.h>
#endif

#include "sensortek_stk3338_i2c.h"
#include "sensortek_stk3338_i2c_config.h"
#include "sensortek_stk3338_i2c_if.h"
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#include "sensor_driver.h"
#else
#include "sensor_util.h"
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
#include "als_sensor.h"
#include "ps_sensor.h"
#ifdef CONFIG_OEM_HKADC
#undef TEMPERATURE_AVAILABLE
#endif

#define DUMMY_TEMP	1000
static int camera_temp = DUMMY_TEMP;
module_param(camera_temp, int, 0644);
static struct i2c_client *client_stk3338 = NULL;
#define	WAKE_LOCK_TIME_QUEUEING	(20)

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
#define	WAKE_LOCK_TIME_DETECT	(500)
#define	WAKE_LOCK_TIME_NODETECT	(1000)
static uint32_t stk3338_initialize = 0;
#else
static atomic_t s_cal_result = ATOMIC_INIT(PS_CAL_OK);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
static int psals_test_log = 0;
module_param(psals_test_log, int, S_IRUGO | S_IWUSR);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

static bool prox_adj = false;

/******************************* define *******************************/
struct ps_als_reg_data {
	struct regulator* v_reg;
	uint32_t min_uV;
	uint32_t max_uV;
	uint32_t on_load_uA;
	uint32_t off_load_uA;
};

struct ps_als_power_ctrl_data {
	bool enabled;
	ktime_t power_off_time;
	int prev_mode;
	struct mutex ps_als_power_mutex;
	int power_off_on_interval_ms;
	int power_on_wait_ms;
	int power_normal_wait_ms;
};

#define NUM_OF_ALS_VAL 20
#define MVAVE_SAMPLING_NUM (((NUM_OF_ALS_VAL * NUM_OF_ALS_VAL)+NUM_OF_ALS_VAL)/2)

struct als_val {
	int			index;
	bool		ave_enable;
	uint32_t	report_lux;
	CALC_DATA	calc_data[NUM_OF_ALS_VAL];
};

#define ERRINFO_I2C_RESET_OCCURRED	0x00000001
#define ERRINFO_SOFT_RESET_OCCURRED	0x00000002
#define ERRINFO_I2C_RECOVERY_FAILED	0x10000000
#define ERRINFO_RECOVERY_FAILED		0x20000000
#define ERRINFO_RECOVERY_FAIL_MASK	0xF0000000

/* structure of peculiarity to use by system */

static const char * const psals_pinctrl_names[PSALS_PIN_ID_MAX] = {
	[PSALS_PIN_DEFAULT]          = "default",
	[PSALS_PIN_ACTIVE]           = "active",
	[PSALS_PIN_SUSPEND]          = "suspend",
	[PSALS_PIN_VPROX_32_ACTIVE]  = "proxmity_vprox_32_active",
	[PSALS_PIN_VPROX_32_SUSPEND] = "proxmity_vprox_32_suspend",
	[PSALS_PIN_VPROX_18_ACTIVE]  = "proxmity_vprox_18_active",
	[PSALS_PIN_VPROX_18_SUSPEND] = "proxmity_vprox_18_suspend",
};

static struct pinctrl *psals_pinctrl;
static struct pinctrl_state *psals_pinctrl_state[PSALS_PIN_ID_MAX];

typedef struct {
    int c;
    int als;
    int als1;
} als_data_set;

typedef struct {
	struct i2c_client	*client;		/* structure pointer for i2c bus            */
	struct hrtimer		timer;			/* structure for timer handler              */
	struct work_struct	ps_work;		/* structure for work queue                 */
	struct work_struct	als_work;		/* structure for work queue                 */
	struct delayed_work	monitor_dwork;
	struct delayed_work boot_ps_report_work;
    struct work_struct ps_cal_work;
	int			delay_time;				/* delay time to set from application       */
	struct ps_sensor_info	ps_info;
	struct als_sensor_info	als_info;
	struct ps_als_reg_data	ps_als_vdd;
	struct ps_als_reg_data	ps_als_vleda;
	struct ps_als_power_ctrl_data	ps_als_power;
	bool		ps_als_vdd_is_ldo;
	bool		ps_als_is_PMIC_supply_ctrl;
	bool		ps_als_vdd_is_gpio;
	bool		ps_als_vleda_is_gpio;
	bool		ps_is_calibrating;
	int			vprox_gpio;
	int 			irq_gpio;
	INIT_ARG		init_data;
	POWERON_ARG		power_data;
	POWERON_ARG		power_data_last;
	int			als_en_cnt;
	int			ps_en_cnt;
	DEVICE_VAL		dev_val;
	struct als_val		als_val;
	READ_DATA_PS_BUF	ps_val;
	int			ps_det;
	int			int_flg;
	int			color;
	bool			als_en_first;
    als_data_set als_last_data;
	struct als_imit		imit;
	struct mutex		control_lock;
	struct mutex		ps_data_lock;
	struct mutex		als_data_lock;
	struct mutex		ps_calib_lock;
	uint32_t		errinfo;
    uint16_t		distance_threshold;
	struct power_supply	*psy;
	uint8_t		is_shtdwn_running;
	bool		host_en;
} PS_ALS_DATA;

/* logical functions */
static int                  get_from_device(DEVICE_VAL *calc_data, struct i2c_client *client);
static void                 als_work_func(struct work_struct *work);
static void                 ps_work_func(struct work_struct *work);
static enum hrtimer_restart als_timer_func(struct hrtimer *timer);
static int                  ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int                  ps_als_remove(struct i2c_client *client);
static void                 ps_als_shutdown(struct i2c_client *client);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void);
void ps_als_exit(void);
#else
static int __init ps_als_init(void);
static void __exit ps_als_exit(void);
#endif
/* access functions */
static int ps_als_driver_init(INIT_ARG *data, struct i2c_client *client);
static int ps_als_driver_reset(struct i2c_client *client);
static int ps_als_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len);
static int ps_als_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len);
static int ps_als_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command);
static int ps_als_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value);
static int ps_als_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values);
//static int ps_als_driver_power_on_off(POWERON_ARG data, struct i2c_client *client);
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client);
static int als_driver_read_data(READ_DATA_ALS_BUF *data, struct i2c_client *client);
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client);
static int ps_als_suspend(struct device *dev);
static int ps_als_resume(struct device *dev);

/**************************** variable declaration ****************************/
static const char              stk3338_driver_ver[] = STK3338_DRIVER_VER;
static struct workqueue_struct *sensortek_workqueue;

/**************************** structure declaration ****************************/
/* I2C device IDs supported by this driver */
static const struct i2c_device_id ps_als_id[] = {
    { STK3338_I2C_NAME, 0 }, /* sensortek stk3338 driver */
    { }
};
MODULE_DEVICE_TABLE(i2c, ps_als_id)

static const struct of_device_id ps_als_of_match[] = {
	{ .compatible = STK3338_I2C_NAME,},
	{ },
};
MODULE_DEVICE_TABLE(of, ps_als_of_match);

static SIMPLE_DEV_PM_OPS(ps_als_pm, ps_als_suspend, ps_als_resume);

/* represent an I2C device driver */
static struct i2c_driver stk3338_driver = {
	.driver = {                      /* device driver model driver */
		.name = STK3338_I2C_NAME,
		.of_match_table = ps_als_of_match,
		.pm   = &ps_als_pm,
	},
	.probe    = ps_als_probe,        /* callback for device binding */
	.remove   = ps_als_remove,       /* callback for device unbinding */
	.shutdown = ps_als_shutdown,
	.id_table = ps_als_id,           /* list of I2C devices supported by this driver */
};

/* gain table */
#define GAIN_FACTOR (4)
static const struct GAIN_TABLE_PS {
	unsigned char GAIN_PS;
} GAIN_TABLE_PS[GAIN_FACTOR] = {
	{   1},   /*  0b00 */
	{   2},   /*  0b01 */
	{   4},   /*  0b10 */
	{   8},   /*  0b11 */
};

static const struct GAIN_TABLE_ALS {
	unsigned char GAIN_ALS;
} GAIN_TABLE_ALS[GAIN_FACTOR] = {
	{   1},   /*  0b00 */
	{   4},   /*  0b01 */
	{  16},   /*  0b10 */
	{  64},   /*  0b11 */
};

static const struct GAIN_TABLE_C {
	unsigned char GAIN_C;
} GAIN_TABLE_C[GAIN_FACTOR] = {
	{    1},   /*  0b00 */
	{    4},   /*  0b01 */
	{   16},   /*  0b10 */
	{   64},   /*  0b11 */
};

#define NUM_OF_COLOR 5
#define ALS_EXP_OF_NV_TH 1000
#define ALS_EXP_OF_NV_COEF 10000
#define ALS_ALS_C_TH 0x0100

static u16 nv_proximity_detect[NUM_OF_COLOR] =
	{0x0320, 0x0320, 0x0320, 0x0320, 0x0320};
static u16 nv_proximity_no_detect[NUM_OF_COLOR] =
	{0x02BC, 0x02BC, 0x02BC, 0x02BC, 0x02BC};
static u16 nv_proximity_offset[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_th0[NUM_OF_COLOR] =
	{0x0032, 0x0032, 0x0032, 0x0032, 0x0032};
static u16 nv_photosensor_th1[NUM_OF_COLOR] =
	{0x0226, 0x0226, 0x0226, 0x0226, 0x0226};
static u16 nv_photosensor_th2[NUM_OF_COLOR] =
	{0x0384, 0x0384, 0x0384, 0x0384, 0x0384};
static u16 nv_photosensor_th3[NUM_OF_COLOR] =
	{0x0802, 0x0802, 0x0802, 0x0802, 0x0802};
static u16 nv_photosensor_th4[NUM_OF_COLOR] =
	{0x0BB8, 0x0BB8, 0x0BB8, 0x0BB8, 0x0BB8};
static u16 nv_photosensor_a0[NUM_OF_COLOR] =
	{0x1FD5, 0x1FD5, 0x1FD5, 0x1FD5, 0x1FD5};
static u16 nv_photosensor_a1[NUM_OF_COLOR] =
	{0x20D0, 0x20D0, 0x20D0, 0x20D0, 0x20D0};
static u16 nv_photosensor_a2[NUM_OF_COLOR] =
	{0x1E14, 0x1E14, 0x1E14, 0x1E14, 0x1E14};
static u16 nv_photosensor_a3[NUM_OF_COLOR] =
	{0x251C, 0x251C, 0x251C, 0x251C, 0x251C};
static u16 nv_photosensor_a4[NUM_OF_COLOR] =
	{0x2710, 0x2710, 0x2710, 0x2710, 0x2710};
static u16 nv_photosensor_b0[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b1[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b2[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b3[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_photosensor_b4[NUM_OF_COLOR] =
	{0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
static u16 nv_proximity_gap[NUM_OF_COLOR] =
	{0x00C8, 0x00C8, 0x00C8, 0x00C8, 0x00C8};

static u8 nv_proximity_temp[] =
	{0x14, 0x28, 0xFF};

enum ps_als_power_ctrl_mode{
	PS_ALS_POWER_CTRL_OFF = 0,
	PS_ALS_POWER_CTRL_LOW,
	PS_ALS_POWER_CTRL_NORMAL,
	PS_ALS_POWER_CTRL_MAX
};
/************************************************************
 *                      logic function                      *
 ***********************************************************/
 
static inline bool chk_ps_onoff(const unsigned char state){
    return (state & EN_PS_ENABLE) ? true:false;
}
static inline bool chk_als_onoff(const unsigned char state){
    return (state & EN_ALS_ENABLE) ? true:false;
}


 
/******************************************************************************
 * NAME       : get_from_device
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static int get_from_device(DEVICE_VAL *dev_val, struct i2c_client *client)
{
	int           result;

	SENSOR_D_LOG("start");
	/* get measure PSCTRL parameter */
	result = ps_als_i2c_read_byte_data(client, REG_PSCTRL);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of dev_val");
		return (result);
	}
	dev_val->prst_ps	= (result & 0b11000000) >> 6;
	dev_val->gain_ps	= (result & 0b00110000) >> 4;
	dev_val->it_ps		= (result & 0b00001111) >> 0;

	/* get measure ALSCTRL1 parameter */
	result = ps_als_i2c_read_byte_data(client, REG_ALSCTRL1);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of dev_val");
		return (result);
	}
	dev_val->prst_als	= (result & 0b11000000) >> 6;
	dev_val->gain_als	= (result & 0b00110000) >> 4;
	dev_val->it_als		= (result & 0b00001111) >> 0;

	/* get measure ALSCTRL2 parameter */
    result = ps_als_i2c_read_byte_data(client, REG_ALSCTRL2);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data of dev_val");
		return (result);
	}
	dev_val->gain_c		= (result & 0b00110000) >> 4;
	SENSOR_D_LOG("end");

	return 0;
}

static int als_get_coef(CALC_DATA *calc_data, PS_ALS_DATA *ps_als)
{
	if (calc_data->ratio < nv_photosensor_th0[ps_als->color]) {
		calc_data->src = LIGHT_CANDESCENT;
		calc_data->alpha = nv_photosensor_a0[ps_als->color];
		calc_data->beta = nv_photosensor_b0[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th0[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th1[ps_als->color])) {
		calc_data->src = LIGHT_INTERMEDIATE;
		calc_data->alpha = nv_photosensor_a1[ps_als->color];
		calc_data->beta = nv_photosensor_b1[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th1[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th2[ps_als->color])) {
		calc_data->src = LIGHT_SOLAR;
		calc_data->alpha = nv_photosensor_a2[ps_als->color];
		calc_data->beta = nv_photosensor_b2[ps_als->color];
	} else if ((calc_data->ratio >= nv_photosensor_th2[ps_als->color]) &&
		   (calc_data->ratio < nv_photosensor_th3[ps_als->color])) {
		calc_data->src = LIGHT_LED_BULB;
		calc_data->alpha = nv_photosensor_a3[ps_als->color];
		calc_data->beta = nv_photosensor_b3[ps_als->color];
	} else if (((calc_data->ratio >= nv_photosensor_th3[ps_als->color]) &&
		    (calc_data->ratio <  nv_photosensor_th4[ps_als->color])) ||
		    (calc_data->als_output <= ALS_ALS_C_TH && calc_data->als1_output <= ALS_ALS_C_TH)) {
		calc_data->src = LIGHT_FLUORESCENT;
		calc_data->alpha = nv_photosensor_a4[ps_als->color];
		calc_data->beta = nv_photosensor_b4[ps_als->color];
	} else {
		calc_data->src = LIGHT_SRC_MAX;
		SENSOR_ERR_LOG("als_get_coef Unexpected.");
		return -1;
	}
	return 0;
}

static int als_calculate_data(READ_DATA_ALS_BUF data, PS_ALS_DATA *ps_als)
{
	int *i;
	int j, k;
	int prev;
	CALC_DATA *calc_data;
	CALC_DATA *calc_data_h;
	DEVICE_VAL *dev_val;
	int result;
	unsigned long temp = 0;

	i = &ps_als->als_val.index;
	calc_data = &ps_als->als_val.calc_data[*i];
	calc_data_h = ps_als->als_val.calc_data;
	dev_val = &ps_als->dev_val;

	/* set the value of measured als data */
	calc_data->als			= data.data_als;
	calc_data->als1			= data.data_als1;
	calc_data->c			= data.data_c;
	calc_data->gain_als		= GAIN_TABLE_ALS[dev_val->gain_als].GAIN_ALS;
	calc_data->gain_c		= GAIN_TABLE_C[dev_val->gain_c].GAIN_C;

	calc_data->als_output 	= calc_data->als;  		 /* Is it need? / calc_data->gain_als; */
	calc_data->als1_output	= calc_data->als1;       /* Is it need? / calc_data->gain_als1;*/
	calc_data->c_output 	= calc_data->c; 	     /* Is it need? / calc_data->gain_c;   */
	if (!calc_data->als_output || !calc_data->als1_output)
		calc_data->ratio = 1 * ALS_EXP_OF_NV_TH;
	else
		calc_data->ratio =
			(calc_data->als_output * ALS_EXP_OF_NV_TH) / calc_data->als1_output;

	result = als_get_coef(calc_data, ps_als);
	if (result) {
		prev = *i - 1;
		if (prev < 0)
			prev = NUM_OF_ALS_VAL - 1;
		calc_data->lux = ps_als->als_val.calc_data[prev].lux;
	} else {
		temp = ((calc_data->alpha * calc_data->als_output) +
				  (calc_data->beta * calc_data->als1_output));
		calc_data->lux =  temp / ALS_EXP_OF_NV_COEF;
	}

	if (!calc_data->als1)
		calc_data->lux = 0;

	SENSOR_D_LOG("lux=%lu als_output=%lu als1_output=%lu gain_als=%u gain_c=%u raw_als=%u raw_als1=%u ratio=%lu src=%d a=%u b=%u",
		calc_data->lux, calc_data->als_output, calc_data->als1_output,
		calc_data->gain_als, calc_data->gain_c,
		calc_data->als, calc_data->als1,
		calc_data->ratio, calc_data->src,
		calc_data->alpha, calc_data->beta);

	if (ps_als->als_val.ave_enable) {
		temp = 0;
		for (j = 0; j < NUM_OF_ALS_VAL; j++) {
			k = (*i + j + 1) % NUM_OF_ALS_VAL;
			temp += calc_data_h[k].lux * (j + 1);
		}
		ps_als->als_val.report_lux = temp / MVAVE_SAMPLING_NUM;
	} else {
		ps_als->als_val.report_lux = calc_data_h[0].lux;
	}

	ps_als->als_val.index++;
	ps_als->als_val.index %= NUM_OF_ALS_VAL;
	if ((!ps_als->als_val.ave_enable) &&
	    (ps_als->als_val.index == (NUM_OF_ALS_VAL - 1)))
		ps_als->als_val.ave_enable = true;

	return 0;
}

/******************************************************************************
 * NAME       : als_driver_read_data
 * FUNCTION   : read the value of ALS data and status in RPR0521
 * REMARKS    :
 *****************************************************************************/
static int als_driver_read_data(READ_DATA_ALS_BUF *data, struct i2c_client *client)
{
    int           ret;
    uint8_t data_buf[2];
	unsigned short data_als;
	unsigned short data_als1;
	unsigned short data_c;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	/* read als and als1 and als_c */
    ret = ps_als_i2c_read(client, REG_DATA_ALS, data_buf, sizeof(data_buf));
    if(ret < 0){
        SENSOR_ERR_LOG("read data_als error. reg[0x%02x]", REG_DATA_ALS);
        return ret;
    }
    data_als = (unsigned short)((data_buf[0]) | (unsigned short)(data_buf[1] << 8));

    ret = ps_als_i2c_read(client, REG_DATA_ALS1, data_buf, sizeof(data_buf));
    if(ret < 0){
        SENSOR_ERR_LOG("read data_als error. reg[0x%02x]", REG_DATA_ALS1);
        return ret;
    }
    data_als1 = (unsigned short)((data_buf[0]) | (unsigned short)(data_buf[1] << 8));

    ret = ps_als_i2c_read(client, REG_DATA_C, data_buf, sizeof(data_buf));
    if(ret < 0){
        SENSOR_ERR_LOG("read data_als error. reg[0x%02x]", REG_DATA_ALS);
        return ret;
    }
    data_c = (unsigned short)((data_buf[0]) | (unsigned short)(data_buf[1] << 8));
    SENSOR_D_LOG("data_als[0x%04x] data_c[0x%04x]", data_als, data_c);
	data->data_als		= CONVERT_TO_BE(data_als);
	data->data_als1		= CONVERT_TO_BE(data_als1);
	data->data_c		= CONVERT_TO_BE(data_c);
    ps_als->als_last_data.als 	= data->data_als;
	ps_als->als_last_data.als1	= data->data_als1;
    ps_als->als_last_data.c		= data->data_c;
	if (unlikely(ps_als->imit.imit_flg)) {
		data->data_als	= ps_als->imit.imit_d0;
		data->data_c	= ps_als->imit.imit_d1;
	}

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	SENSOR_D_LOG("data->als=%d, data->c=%d", data->data_als, data->data_c);
	return 0;
}

/******************************************************************************
 * NAME       : ps_driver_read_data
 * FUNCTION   : read the value of PS data in STK3338
 * REMARKS    :
 *****************************************************************************/
static int ps_driver_read_data(struct i2c_client *client)
{
	int           result;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);
	unsigned short data_ps;
	GENREAD_ARG   gene_read;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	/* read start PS_DATA address */
	gene_read.adr_reg = REG_DATA_PS;
	gene_read.addr    = (char *)&data_ps;
	gene_read.size    = sizeof(data_ps);

	/* block read */
	result = ps_als_driver_general_read(gene_read, client);
	if (likely(result > 0)) {
		ps_als->ps_val.data_ps = CONVERT_TO_BE(data_ps);
		result = 0;
	} else {
		ps_als->ps_val.data_ps = 0;
		SENSOR_ERR_LOG("ps_als_driver_general_read err:%d",result);
	}
	
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	return (result);
}

/******************************************************************************
 * NAME       : als_work_func
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static void als_work_func(struct work_struct *als_work)
{
	int           result;
	READ_DATA_ALS_BUF read_data_buf;
	PS_ALS_DATA   *ps_als = container_of(als_work, PS_ALS_DATA, als_work);
	long          get_timer;
	long          wait_sec;
	unsigned long wait_nsec;
	PWR_ST        pwr_st;

	SENSOR_D_LOG("start");

	if(ps_als->is_shtdwn_running){
		SENSOR_ERR_LOG("This process was skipped due to shutdown sequence running.");
		return;
	}

	read_data_buf.data_als 	= 0;
	read_data_buf.data_c	= 0;

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.als_state == CTL_STANDBY)) {
		goto exit;
	}

	mutex_lock(&ps_als->als_data_lock);

	result = als_driver_read_data(&read_data_buf, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ERROR! read data");
		goto exit_lock;
	}

	/* read value from device */
	result = get_from_device(&ps_als->dev_val, ps_als->client);
	if (result < 0) {
		SENSOR_ERR_LOG("ERROR! read data from device.");
		goto exit_lock;
	}

	result = als_calculate_data(read_data_buf, ps_als);
	if (result) {
		SENSOR_ERR_LOG("ERROR! calculate als data.");
		goto exit_lock;
	}
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	sensor_report_data(SENSOR_LIGHT, ps_als->als_val.report_lux);
#else
	als_sensor_report_event(&ps_als->als_info, ps_als->als_val.report_lux, ps_als->als_en_first);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	if (unlikely(ps_als->als_en_first))
		ps_als->als_en_first = false;

	/* the setting value from application */
	get_timer = ps_als->delay_time;
	/* 125ms(8Hz) at least */
	wait_sec  = (get_timer / SM_TIME_UNIT);
	wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
	hrtimer_start(&ps_als->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
#else
	result = hrtimer_start(&ps_als->timer, ktime_set(wait_sec, wait_nsec), HRTIMER_MODE_REL);
	if (result != 0) {
		SENSOR_ERR_LOG("can't start timer");
		goto exit_lock;
	}
#endif /* LINUX_VERSION_CODE */
exit_lock:
	mutex_unlock(&ps_als->als_data_lock);
exit:
	SENSOR_D_LOG("end");
	return;
}

/******************************************************************************
 * NAME       : als_timer_func
 * FUNCTION   : call work function (thread of timer)
 * REMARKS    :
 *****************************************************************************/
static enum hrtimer_restart als_timer_func(struct hrtimer *timer)
{
    PS_ALS_DATA *ps_als;
    int         result;

    ps_als = container_of(timer, PS_ALS_DATA, timer);
    result = queue_work(sensortek_workqueue, &ps_als->als_work);
    if (result == 0) {
        SENSOR_ERR_LOG("can't register que.");
        SENSOR_ERR_LOG("result = 0x%x", result);
    }

    return (HRTIMER_NORESTART);
}

static int ps_check_status(PS_ALS_DATA *ps_als)
{
	int tmp;
	SENSOR_D_LOG("start");
	if ((ps_als->ps_val.flag_ps & FLG_NF_MASK) == (0))
		tmp = PROX_STATUS_NEAR;
	else if ((ps_als->ps_val.flag_ps & FLG_NF_MASK) == (1))
		tmp = PROX_STATUS_FAR;
	else
		return 0;

	if (ps_als->ps_det != tmp) {
		ps_als->ps_det = tmp;
		SENSOR_D_LOG("end return=1");
		return 1;
	} else {
		SENSOR_D_LOG("end return=0");
		return 0;
	}
}

static int ps_sensor_clr_interrupt(struct i2c_client *client,
                                    unsigned char current_flag_ps)
{
    int result;
    uint8_t w_val = current_flag_ps & ~FLG_PS_INT_MASK;
	SENSOR_D_LOG("start");
	result = ps_als_i2c_write_byte_data(client, REG_FLAG, w_val);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to clear interrupt");
		return result;
	}
	SENSOR_D_LOG("end");
    return result;
}

#ifdef USE_STK3338
static int  ps_sensor_chg_persist(struct i2c_client *client,
                                    unsigned char current_flag_ps)
{
	int result;
	uint8_t nf_ps = current_flag_ps & FLG_NF_MASK;
	SENSOR_D_LOG("start");
	if (nf_ps > 0){
		result = ps_als_i2c_write_byte_data(client, REG_PSCTRL, PS_ALS_SET_PSCTRL | PRST_PS_16TIMES );
	}
	else{
		result = ps_als_i2c_write_byte_data(client, REG_PSCTRL, PS_ALS_SET_PSCTRL | PRST_PS_01TIMES );
	}
	SENSOR_D_LOG("end");
	return result;
}
#endif

static void ps_sensor_report_event_proc(PS_ALS_DATA *ps_als, uint32_t detect)
{
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	long wake_lock_time;
	struct ps_sensor_info *info = &ps_als->ps_info;
	SENSOR_D_LOG("start");

	if (unlikely(!info)) {
		SENSOR_ERR_LOG("fail bad parm --> info");
		return;
	}

	if (!atomic_read(&info->valid)) {
		SENSOR_ERR_LOG("ps sensor invalid. skip.");
		return;
	}

	sensor_notify(SENSOR_PROX, detect);
	wake_lock_time = detect ? WAKE_LOCK_TIME_DETECT : WAKE_LOCK_TIME_NODETECT;
	pm_wakeup_event(info->dev, wake_lock_time);
#else
	SENSOR_D_LOG("start");
	ps_sensor_report_event(&ps_als->ps_info, detect);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	SENSOR_D_LOG("end");
}

static void ps_work_func_internal(PS_ALS_DATA *ps_als)
{
	int         	result;
	PWR_ST        	pwr_st;
	GENREAD_ARG   	gene_data;
	unsigned char 	flag_ps;
	SENSOR_D_LOG("start");
	if(ps_als->is_shtdwn_running){
		SENSOR_ERR_LOG("This process was skipped due to shutdown sequence running.");
		return;
	}
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	if (psals_test_log){
		s64 enter_us;
		ktime_t enter;
		SENSOR_ERR_LOG("[IT_TEST] wake_lock_start");

		enter = ktime_get();
		enter_us = ktime_to_us(enter);
		printk(KERN_NOTICE "[IT_TEST] %s: time:%lld\n",__func__,enter_us);
	}
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	/* read interrupt flag */
	gene_data.adr_reg = REG_FLAG;
	gene_data.addr    = &flag_ps;
	gene_data.size    = sizeof(flag_ps);
	result = ps_als_driver_general_read(gene_data, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("general read can't execute ");
		SENSOR_ERR_LOG("can't read FLAG register ");
		goto exit;
	}

	/* read the state of sensor */
	result = ps_als_driver_read_power_state(&pwr_st, ps_als->client);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("power_state error ");
		goto exit;
	}
	/* check the state of sensor */
	if (unlikely(pwr_st.ps_state == CTL_STANDBY))
		goto exit;

	mutex_lock(&ps_als->ps_data_lock);
	ps_als->ps_val.flag_ps = flag_ps;
#ifdef USE_STK3338
	ps_sensor_chg_persist(ps_als->client, flag_ps);
#endif
	SENSOR_D_LOG("ps_flag=0x%x",ps_als->ps_val.flag_ps);
	if (unlikely((ps_als->ps_val.flag_ps & FLG_INVALID_PS_INT_MASK) == (1))) {
			goto exit_lock;
	}

	if (ps_check_status(ps_als) || !ps_als->int_flg) {
			SENSOR_D_LOG("ps_als->ps_det=%d", ps_als->ps_det);
			ps_sensor_report_event_proc(ps_als, ps_als->ps_det);
	}

exit_lock:
	mutex_unlock(&ps_als->ps_data_lock);
exit:
    ps_sensor_clr_interrupt(ps_als->client, flag_ps);
	ps_als->int_flg = true;
	SENSOR_D_LOG("end");
}

static void ps_work_func(struct work_struct *ps_work)
{
        PS_ALS_DATA *ps_als = container_of(ps_work, PS_ALS_DATA, ps_work);
        ps_work_func_internal(ps_als);
}

/******************************************************************************
 * NAME       : ps_irq_handler
 * FUNCTION   : interruption function (irq)
 * REMARKS    :
 *****************************************************************************/
static irqreturn_t ps_irq_handler(int irq, void *dev_id)
{
	PS_ALS_DATA *ps_als = dev_id;
	SENSOR_D_LOG("start");
	pm_wakeup_event(ps_als->ps_info.dev, WAKE_LOCK_TIME_QUEUEING);
	queue_work(sensortek_workqueue,&ps_als->ps_work);
	SENSOR_D_LOG("end");
	return (IRQ_HANDLED);
}

static void ps_als_set_errinfo(PS_ALS_DATA *ps_als, uint32_t errinfo)
{
	static DEFINE_MUTEX(errinfo_write_lock);
	SENSOR_ERR_LOG("start errinfo[0x%x] ps_als->errinfo[0x%x]", errinfo, ps_als->errinfo);

	mutex_lock(&errinfo_write_lock);
	SENSOR_N_LOG("some errors occured. errinfo[0x%x]", errinfo);
	ps_als->errinfo |= errinfo;
	mutex_unlock(&errinfo_write_lock);

	if (errinfo & ERRINFO_RECOVERY_FAIL_MASK) {
		ps_als->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);
	}

	SENSOR_V_LOG("end");
}

static bool ps_als_is_device_dead(PS_ALS_DATA *ps_als)
{
	return ((ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) != 0) ? true : false;
}

static int ps_als_confirm_regs(PS_ALS_DATA *ps_als)
{
	STK3338_REGS 	regs;
	int				ret = 0;
	unsigned char	work;
	u16				ps_th;
	u16				ps_tl;
	u16				data_ps_offset;
	u16				ps_thd_offset;

	SENSOR_V_LOG("start");

	ret = ps_als_i2c_read(ps_als->client, REG_STATE, (char *)&regs.state, (14));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}
	
	ret = ps_als_i2c_read(ps_als->client, REG_FLAG, (char *)&regs.flag, (5));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}
	
	ret = ps_als_i2c_read(ps_als->client, REG_DATA_C, (char *)&regs.data_c_msb, (4));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}
	
	ret = ps_als_i2c_read(ps_als->client, REG_PDT_ID, (char *)&regs.pdt_id, (1));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}
	
	ret = ps_als_i2c_read(ps_als->client, REG_ALSCTRL2, (char *)&regs.alsctrl2, (2));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	ret = ps_als_i2c_read(ps_als->client, REG_SOFT_RESET, (char *)&regs.soft_reset, (1));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	ret = ps_als_i2c_read(ps_als->client, REG_SYSCTRL1, (char *)&regs.sysctrl1, (2));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	ret = ps_als_i2c_read(ps_als->client, REG_INTCTRL2, (char *)&regs.intctrl2, (1));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}
	
	ret = ps_als_i2c_read(ps_als->client, REG_THD_PSOFF, (char *)&regs.thd_psoff_msb, (3));
	if (ret < 0) {
		SENSOR_ERR_LOG("i2c transfer failed.");
		return -1;
	}

	work = PS_ALS_SET_STATE;
	if (ps_als->power_data.power_als)
		work |= EN_ALS_ENABLE;
	if (ps_als->power_data.power_ps)
		work |= EN_PS_ENABLE;

	if (regs.state != work) {
		SENSOR_ERR_LOG("unexpected state[0x%x]. expect[0x%x]", regs.state, work);
		return -1;
	}

	if ((ps_als->ps_val.flag_ps & FLG_NF_MASK) > 0 ){
		if(regs.psctrl != (PS_ALS_SET_PSCTRL | PRST_PS_16TIMES)){
			SENSOR_ERR_LOG("unexpected psctrl[0x%x]. expect[0x%x]", regs.psctrl, PS_ALS_SET_PSCTRL | PRST_PS_16TIMES);
			return -1;
		}
	} else {
		if(regs.psctrl != (PS_ALS_SET_PSCTRL | PRST_PS_01TIMES)){
			SENSOR_ERR_LOG("unexpected psctrl[0x%x]. expect[0x%x]", regs.psctrl, PS_ALS_SET_PSCTRL | PRST_PS_01TIMES);
			return -1;
		}
	}

	if (regs.alsctrl1 != PS_ALS_SET_ALSCTRL1) {
		SENSOR_ERR_LOG("unexpected psctrl[0x%x]. expect[0x%x]", regs.alsctrl1, PS_ALS_SET_ALSCTRL1);
		return -1;
	}

	if (regs.vcselctrl != PS_ALS_SET_VCSELCTRL) {
		SENSOR_ERR_LOG("unexpected vcselctrl[0x%x]. expect[0x%x]", regs.vcselctrl, PS_ALS_SET_VCSELCTRL);
		return -1;
	}

	if (regs.intctrl1 != PS_ALS_SET_INTCTRL1) {
		SENSOR_ERR_LOG("unexpected intctrl1[0x%x]. expect[0x%x]", regs.intctrl1, PS_ALS_SET_INTCTRL1);
		return -1;
	}

	if (regs.wait != PS_ALS_SET_WAIT) {
		SENSOR_ERR_LOG("unexpected wait[0x%x]. expect[0x%x]", regs.wait, PS_ALS_SET_WAIT);
		return -1;
	}

	if (regs.alsctrl2 != PS_ALS_SET_ALSCTRL2) {
		SENSOR_ERR_LOG("unexpected alsctrl2[0x%x]. expect[0x%x]", regs.alsctrl2, PS_ALS_SET_ALSCTRL2);
		return -1;
	}

	if (regs.intelli_wait_ps != PS_ALS_SET_INTELLI_WAIT_PS) {
		SENSOR_ERR_LOG("unexpected intelli_wait_ps[0x%x]. expect[0x%x]", regs.intelli_wait_ps, PS_ALS_SET_INTELLI_WAIT_PS);
		return -1;
	}

	if (regs.soft_reset != PS_ALS_SET_SOFT_RESET) {
		SENSOR_ERR_LOG("unexpected reset[0x%x]. expect[0x%x]", regs.soft_reset, PS_ALS_SET_SOFT_RESET);
		return -1;
	}

	if (regs.sysctrl1 != PS_ALS_SET_SYSCTRL1 ) {
		SENSOR_ERR_LOG("unexpected sysctrl1[0x%x]. expect[0x%x]", regs.sysctrl1, PS_ALS_SET_SYSCTRL1);
		return -1;
	}

	if (regs.pspdctrl != PS_ALS_SET_PSPDCTRL) {
		SENSOR_ERR_LOG("unexpected alsctrl2[0x%x]. expect[0x%x]", regs.pspdctrl, PS_ALS_SET_PSPDCTRL);
		return -1;
	}

	if (regs.intctrl2 != PS_ALS_SET_INTCTRL2) {
		SENSOR_ERR_LOG("unexpected intctrl2[0x%x]. expect[0x%x]", regs.intctrl2, PS_ALS_SET_INTCTRL2);
		return -1;
	}

	if (regs.thd_bgir != PS_ALS_SET_BGIR ) {
		SENSOR_ERR_LOG("unexpected thd_bgir[0x%x]. expect[0x%x]", regs.thd_bgir, PS_ALS_SET_BGIR);
		return -1;
	}

	if (ps_als->power_data.power_ps) {
		mutex_lock(&ps_als->ps_data_lock);
		ps_th 			= ps_als->init_data.thdh_ps;
		ps_tl 			= ps_als->init_data.thdl_ps;
		data_ps_offset 		= ps_als->init_data.data_ps_offset;
		ps_thd_offset 	= ps_als->init_data.thd_psoff;
		mutex_unlock(&ps_als->ps_data_lock);

		if ((regs.thdh_ps_msb 			!= (u8)(ps_th 				>> 8)) 		||
		    (regs.thdh_ps_lsb 			!= (u8)(ps_th 				>> 0)) 		||
		    (regs.thdl_ps_msb 			!= (u8)(ps_tl 				>> 8)) 		||
		    (regs.thdl_ps_lsb 			!= (u8)(ps_tl 				>> 0)) 		||
		    (regs.data_ps_offset_msb 	!= (u8)(data_ps_offset 		>> 8)) 		||
		    (regs.data_ps_offset_lsb 	!= (u8)(data_ps_offset 		>> 0)) 		||
		    (regs.thd_psoff_msb		 	!= (u8)(ps_thd_offset		>> 8)) 		||
		    (regs.thd_psoff_lsb 		!= (u8)(ps_thd_offset 		>> 0))) 	{
			SENSOR_ERR_LOG("unexpected thdh_ps_msb		 [0x%x]. expect[0x%x] \
							unexpected thdh_ps_lsb		 [0x%x]. expect[0x%x] \
							unexpected thdl_ps_msb		 [0x%x]. expect[0x%x] \
							unexpected thdl_ps_lsb		 [0x%x]. expect[0x%x] \
							unexpected data_ps_offset_msb[0x%x]. expect[0x%x] \
							unexpected data_ps_offset_lsb[0x%x]. expect[0x%x] \
							unexpected thd_psoff_msb	 [0x%x]. expect[0x%x] \
							unexpected thd_psoff_lsb	 [0x%x]. expect[0x%x]",  \
							regs.thdh_ps_msb, 		 (u8)(ps_th 			>> 8), \
							regs.thdh_ps_lsb,		 (u8)(ps_th 			>> 0), \
							regs.thdl_ps_msb, 		 (u8)(ps_tl 			>> 8), \
							regs.thdl_ps_lsb, 		 (u8)(ps_tl 			>> 0), \
							regs.data_ps_offset_msb, (u8)(data_ps_offset    >> 8), \
							regs.data_ps_offset_lsb, (u8)(data_ps_offset    >> 0), \
							regs.thd_psoff_msb, 	 (u8)(ps_thd_offset     >> 8), \
							regs.thd_psoff_lsb, 	 (u8)(ps_thd_offset     >> 0));
			return -1;
		}
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int als_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable);
static int ps_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable);
static int ps_als_recover_state(PS_ALS_DATA *ps_als)
{
	int result;

	SENSOR_V_LOG("start");

	mutex_lock(&ps_als->ps_data_lock);
	mutex_lock(&ps_als->als_data_lock);
	result = ps_als_driver_init(&ps_als->init_data, ps_als->client);
	mutex_unlock(&ps_als->als_data_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	if (result < 0) {
		SENSOR_ERR_LOG("fail ps_als_driver_init");
		return result;
	}
	if (ps_als->power_data.power_als) {
		result = als_enable_proc(ps_als, false);
		if (result < 0) {
			SENSOR_ERR_LOG("fail als_enable_proc");
			return result;
		}
		result = als_enable_proc(ps_als, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail als_enable_proc");
			return result;
		}
	}
	if (ps_als->power_data.power_ps) {
		result = ps_enable_proc(ps_als, false);
		if (result < 0) {
			SENSOR_ERR_LOG("fail ps_enable_proc");
			return result;
		}
		result = ps_enable_proc(ps_als, true);
		if (result < 0) {
			SENSOR_ERR_LOG("fail ps_enable_proc");
			return result;
		}
	}

	SENSOR_V_LOG("end");
	return 0;
}

static int ps_als_recovery(PS_ALS_DATA *ps_als)
{
#define RECOVERY_RETRY_MAX	3
	int retries = RECOVERY_RETRY_MAX;
	int result;
	SENSOR_N_LOG("start");

	while (retries--) {
		ps_als_set_errinfo(ps_als, ERRINFO_SOFT_RESET_OCCURRED);
		result = ps_als_driver_reset(ps_als->client);
		if (result < 0) {
			SENSOR_ERR_LOG("ps_als_driver_reset error result:%d retries:%d",result,retries);
			continue;
		}
		usleep_range(10000, 10100);

		result = ps_als_i2c_read_byte_data(ps_als->client, REG_STATE);
		if (result != 0x00) {
			SENSOR_ERR_LOG("ps_als_i2c_read_byte_data error result:%d retries:%d",result,retries);
			continue;
		}

		result = ps_als_recover_state(ps_als);
		return result;
	}

	SENSOR_ERR_LOG("recovery failed");
	ps_als_set_errinfo(ps_als, ERRINFO_RECOVERY_FAILED);
	return -1;
#undef RECOVERY_RETRY_MAX
}

static void ps_als_check_error(PS_ALS_DATA *ps_als)
{
	int ret;

	SENSOR_V_LOG("start");
	ret = ps_als_confirm_regs(ps_als);
	if (ret < 0) {
		ret = ps_als_recovery(ps_als);
		if (ret < 0) {
			SENSOR_ERR_LOG("failed ps_als_recovery");
		}
	}

	SENSOR_V_LOG("end");
	return;
}

static int ps_get_temperature(PS_ALS_DATA *ps_als, int *temperature)
{
	int	ret = -1;
#ifdef TEMPERATURE_AVAILABLE
	union power_supply_propval propval = {20,};
	SENSOR_V_LOG("start");

	if (!ps_als->psy) {
		ps_als->psy = power_supply_get_by_name("hkadc");
	}
	if (ps_als->psy) {
		ret = power_supply_get_property(ps_als->psy, POWER_SUPPLY_PROP_OEM_OUT_CAMERA_THERM, &propval);
		*temperature = propval.intval;
	}
#else
	SENSOR_V_LOG("start TEMPERATURE_AVAILABLE is not defined.");
	ret = 0;
#endif
	if (camera_temp != DUMMY_TEMP) {
		*temperature = camera_temp;
		ret = 0;
	}
	SENSOR_V_LOG("end");
	return ret;
}

static void ps_get_new_thresholds(PS_ALS_DATA *ps_als,
	u16 *ps_th_ret, u16 *ps_tl_ret, u16 *data_ps_offset_ret)
{
	int	result;
	u16	ps_th;
	u16	ps_tl;
	u16	data_ps_offset;
	u16	ps_th_base;
	u16	ps_tl_base;
	int	temperature = 25;
	int	coef_1;
	int	coef_2;
	int	coef_temp;

	SENSOR_V_LOG("start");

	mutex_lock(&ps_als->ps_data_lock);
	ps_th_base = nv_proximity_detect[ps_als->color];
	ps_tl_base = nv_proximity_no_detect[ps_als->color];
	data_ps_offset = nv_proximity_offset[ps_als->color];
	coef_1 = nv_proximity_temp[0];
	coef_2 = nv_proximity_temp[1];
	mutex_unlock(&ps_als->ps_data_lock);
	result = ps_get_temperature(ps_als, &temperature);

	if (result < 0 || !coef_2) {
		SENSOR_V_LOG("invalid param. result[%d] coef_2[%d]", result, coef_2);
		ps_th = ps_th_base;
		ps_tl = ps_tl_base;
	} else {
		SENSOR_V_LOG("ps_th_base[%x] ps_tl_base[%x] coef_1[%x] coef_2[%x] temperature[%d]",
			ps_th_base, ps_tl_base, coef_1, coef_2, temperature);

		if (temperature < -20)
			temperature = -20;

		if (temperature <= 10) {
			coef_temp = temperature + 20;

			ps_th = (u16)((long)ps_th_base + coef_1 - ((coef_temp * coef_1) / coef_2));
			ps_tl = (u16)((long)ps_tl_base + coef_1 - ((coef_temp * coef_1) / coef_2));

			if (ps_th > REG_PSTH_MAX || ps_tl > REG_PSTL_MAX) {
				SENSOR_N_LOG("invalid ps_th[%d] or ps_tl[%d]. set default", ps_th, ps_tl);
				ps_th = ps_th_base;
				ps_tl = ps_tl_base;
			}
			if (ps_tl > ps_th) {
				SENSOR_N_LOG("notice. ps_tl > ps_th");
			}
		} else {
			ps_th = ps_th_base;
			ps_tl = ps_tl_base;
		}
	}

	*ps_th_ret = ps_th;
	*ps_tl_ret = ps_tl;
	*data_ps_offset_ret = data_ps_offset;

	SENSOR_V_LOG("end ps_th[%d] ps_tl[%d] data_ps_offset[%d]", ps_th, ps_tl, data_ps_offset);
	return;
}

static bool ps_update_thresholds(PS_ALS_DATA *ps_als)
{
	bool	changed;
	u16	ps_th;
	u16	ps_tl;
	u16	data_ps_offset;

	SENSOR_V_LOG("start");

	ps_get_new_thresholds(ps_als, &ps_th, &ps_tl, &data_ps_offset);

	mutex_lock(&ps_als->ps_data_lock);
	if (ps_th != ps_als->init_data.thdh_ps || ps_tl != ps_als->init_data.thdl_ps
		|| data_ps_offset != ps_als->init_data.data_ps_offset) {
		ps_als->init_data.thdh_ps = ps_th;
		ps_als->init_data.thdl_ps = ps_tl;
		ps_als->init_data.data_ps_offset = data_ps_offset;
		changed = true;
	} else {
		changed = false;
	}
	mutex_unlock(&ps_als->ps_data_lock);

	SENSOR_V_LOG("end");
	return changed;
}

static int ps_send_config(PS_ALS_DATA *ps_als)
{
	int				result;
	STK3338_REGS	regs;
    //unsigned char	th_ps_buf[4] = {0};

	SENSOR_V_LOG("start");
	mutex_lock(&ps_als->ps_data_lock);
	regs.thdh_ps_lsb = ps_als->init_data.thdh_ps >> 0;
	regs.thdh_ps_msb = ps_als->init_data.thdh_ps >> 8;
	regs.thdl_ps_lsb = ps_als->init_data.thdl_ps >> 0;
	regs.thdl_ps_msb = ps_als->init_data.thdl_ps >> 8;
	regs.data_ps_offset_lsb = ps_als->init_data.data_ps_offset >> 0;
	regs.data_ps_offset_msb = ps_als->init_data.data_ps_offset >> 8;
	regs.thd_psoff_lsb		= ps_als->init_data.thd_psoff >> 0;
	regs.thd_psoff_msb		= ps_als->init_data.thd_psoff >> 8;
	mutex_unlock(&ps_als->ps_data_lock);

    SENSOR_ERR_LOG("thdh1_ps[0x%02x] thdh2_ps[0x%02x] thdl1_ps[0x%02x] thdl2_ps[0x%02x]",
    regs.thdh_ps_msb, regs.thdh_ps_lsb, regs.thdl_ps_msb, regs.thdl_ps_lsb);
	result = ps_als_i2c_write(ps_als->client, REG_THDH_PS , &regs.thdh_ps_msb, 4);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set thresholds");
		return -1;
	}
	SENSOR_D_LOG("data_ps_offset[0x%02x] [0x%02x]",regs.data_ps_offset_msb,regs.data_ps_offset_lsb);
	result = ps_als_i2c_write(ps_als->client, REG_DATA_PS_OFFSET, &regs.data_ps_offset_msb, 2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set offset");
		return -1;
	}
	SENSOR_D_LOG("psctrl[0x%02x]",PS_ALS_SET_PSCTRL);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_PSCTRL, PS_ALS_SET_PSCTRL);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("vcselstrl[0x%02x]",PS_ALS_SET_VCSELCTRL);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_VCSELCTRL, PS_ALS_SET_VCSELCTRL);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("intctrl1[0x%02x]",PS_ALS_SET_INTCTRL1);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_INTCTRL1, PS_ALS_SET_INTCTRL1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("alsctrl2[0x%02x]",PS_ALS_SET_ALSCTRL2);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_ALSCTRL2,  PS_ALS_SET_ALSCTRL2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("intelli_wait_ps[0x%02x]",PS_ALS_SET_INTELLI_WAIT_PS);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_INTELLI_WAIT_PS, PS_ALS_SET_INTELLI_WAIT_PS);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("sysctrl1[0x%02x]",PS_ALS_SET_SYSCTRL1);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_SYSCTRL1, PS_ALS_SET_SYSCTRL1);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("pspdctrl[0x%02x]",PS_ALS_SET_PSPDCTRL);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_PSPDCTRL, PS_ALS_SET_PSPDCTRL);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("intctrl2[0x%02x]",PS_ALS_SET_INTCTRL2);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_INTCTRL2, PS_ALS_SET_INTCTRL2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("thd_psoff[0x%02x]",regs.thd_psoff_msb);
	result = ps_als_i2c_write(ps_als->client, REG_THD_PSOFF, &regs.thd_psoff_msb, 2);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}
	SENSOR_D_LOG("thd_bgir[0x%02x]",PS_ALS_SET_BGIR);
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_THD_BGIR, PS_ALS_SET_BGIR);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("failed to set interrupt reg");
		return -1;
	}

	SENSOR_V_LOG("end");
	return 0;
}

static void ps_check_thresholds_update(PS_ALS_DATA *ps_als)
{
	SENSOR_V_LOG("start");
	if (!ps_als->power_data.power_ps) {
		SENSOR_D_LOG("exit power_ps:%d",ps_als->power_data.power_ps);
		return;
	}

	if (!ps_update_thresholds(ps_als)) {
		SENSOR_D_LOG("exit ps_update_thresholds false");
		return;
	}

	disable_irq(ps_als->client->irq);
	ps_send_config(ps_als);
	enable_irq(ps_als->client->irq);

	SENSOR_V_LOG("end");
	return;
}

static void ps_als_schedule_monitor(PS_ALS_DATA *ps_als)
{
	SENSOR_V_LOG("start");
	SENSOR_V_LOG("ps_als->host_en = %d",ps_als->host_en);
//	if(ps_als->host_en == HOST_ENABLE){
//	queue_delayed_work(sensortek_workqueue, &ps_als->monitor_dwork, msecs_to_jiffies(5000));
//	}
}

static void ps_als_monitor_func(struct work_struct *work)
{
	PS_ALS_DATA   *ps_als;

	SENSOR_D_LOG("start");
	ps_als = container_of(work, PS_ALS_DATA, monitor_dwork.work);
	mutex_lock(&ps_als->control_lock);
	SENSOR_V_LOG("ps_als->host_en = %d",ps_als->host_en);
	if ((ps_als->power_data.power_ps || ps_als->power_data.power_als) &&
	!ps_als_is_device_dead(ps_als) && (!ps_als->ps_is_calibrating) && ps_als->host_en ==  HOST_ENABLE) {
		SENSOR_D_LOG("start ps_als_check_error");
		ps_als_check_error(ps_als);
		ps_als_schedule_monitor(ps_als);
	}
	mutex_unlock(&ps_als->control_lock);
	SENSOR_D_LOG("end");
}

static void ps_als_schedule_boot_ps_report(PS_ALS_DATA *ps_als)
{
	SENSOR_D_LOG("start");
	queue_delayed_work(sensortek_workqueue, &ps_als->boot_ps_report_work, msecs_to_jiffies(200));
	SENSOR_D_LOG("end");
}

static void ps_als_boot_ps_report_func(struct work_struct *work)
{
	PS_ALS_DATA *ps_als = container_of(work, PS_ALS_DATA, boot_ps_report_work.work);
	SENSOR_D_LOG("start");
	ps_work_func_internal(ps_als);
	SENSOR_D_LOG("end");
}

static int ps_als_power_ctrl(PS_ALS_DATA *ps_als, enum ps_als_power_ctrl_mode request_mode);
static int als_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable)
{
	int result;
	unsigned char power_state;
    bool is_ps_enable = false;
	long get_timer;
	long wait_sec;
	unsigned long wait_nsec;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	SENSOR_D_LOG("start. enable=%d", enable);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	SENSOR_ERR_LOG("[IT_TEST] start. enable=%d", enable);
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	if (unlikely(ps_als->power_data.power_als == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	mutex_lock(&ps_als->ps_calib_lock);
	if (enable) {
		result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
			mutex_unlock(&ps_als->ps_calib_lock);
			return result;
		}
	}

	result = ps_als_i2c_read_byte_data(ps_als->client, REG_STATE);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ps_als_i2c_read_byte_data error result:%d",result);
		/* i2c communication error */
		mutex_unlock(&ps_als->ps_calib_lock);
		return result;
	}

	power_state  = (unsigned char)(result & ~(EN_ALS_ENABLE | EN_WAIT_ENABLE));
    if(chk_ps_onoff(power_state)) is_ps_enable = true;
	if (enable) {
		power_state |= EN_ALS_ENABLE;
	} else {
		power_state &= ~EN_ALS_ENABLE;
		if (is_ps_enable) {
			power_state |= EN_WAIT_ENABLE;
		}
	}

	result = ps_als_i2c_write_byte_data(ps_als->client,
					   REG_STATE, power_state);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ps_als_i2c_write_byte_data error result:%d",result);
		/* i2c communication error */
		mutex_unlock(&ps_als->ps_calib_lock);
		return result;
	}

	if (enable) {
		mutex_lock(&ps_als->als_data_lock);
		ps_als->als_val.index = 0;
		ps_als->als_val.ave_enable = false;
		ps_als->als_en_first = true;
		memset(ps_als->als_val.calc_data, 0,
			sizeof(CALC_DATA) * NUM_OF_ALS_VAL);
		mutex_unlock(&ps_als->als_data_lock);

		/* the setting value from application */
		get_timer = ALS_ON_DELAY_MS;
		/* 125ms(8Hz) at least */
		wait_sec  = (get_timer / SM_TIME_UNIT);
		wait_nsec = ((get_timer - (wait_sec * SM_TIME_UNIT)) * MN_TIME_UNIT);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		hrtimer_start(&ps_als->timer,
				       ktime_set(wait_sec, wait_nsec),
				       HRTIMER_MODE_REL);
#else
		result = hrtimer_start(&ps_als->timer,
				       ktime_set(wait_sec, wait_nsec),
				       HRTIMER_MODE_REL);
		if (unlikely(result)) {
			SENSOR_ERR_LOG("can't start timer");
			mutex_unlock(&ps_als->ps_calib_lock);
			return result;
		}
#endif /* LINUX_VERSION_CODE */
		ps_als_schedule_monitor(ps_als);
	} else {
		hrtimer_cancel(&ps_als->timer);
		cancel_work_sync(&ps_als->als_work);
		hrtimer_cancel(&ps_als->timer);

		if (!ps_als->power_data.power_ps) {
			result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
			if(unlikely(result < 0)) {
				SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
				mutex_unlock(&ps_als->ps_calib_lock);
				return result;
			}
		}
	}

	ps_als->power_data.power_als = enable;
	mutex_unlock(&ps_als->ps_calib_lock);

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

static int ps_enable_proc(PS_ALS_DATA *ps_als, unsigned char enable)
{
	int result;
	unsigned char power_state;

	SENSOR_D_LOG("start. enable=%d", enable);
	SENSOR_N_LOG("enable=%d", enable);
	if (unlikely(ps_als->power_data.power_ps == enable)) {
		SENSOR_D_LOG("same status. skip");
		return 0;
	}

	mutex_lock(&ps_als->ps_calib_lock);
	if (enable) {
		result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
		if(unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
			mutex_unlock(&ps_als->ps_calib_lock);
			return result;
		}
		ps_update_thresholds(ps_als);
		result = ps_send_config(ps_als);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("failed to send config");
			mutex_unlock(&ps_als->ps_calib_lock);
			return result;
		}
	} else {
		ps_als->power_data.power_ps = enable;
	}

	result = ps_als_i2c_read_byte_data(ps_als->client, REG_STATE);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ps_als_i2c_read_byte_data failed:%d",result);
		/* i2c communication error */
		mutex_unlock(&ps_als->ps_calib_lock);
		return result;
	}

	if (enable) {
		enable_irq(ps_als->client->irq);
	}

	power_state  = (unsigned char)(result & ~(EN_PS_ENABLE | EN_WAIT_ENABLE));

	if (enable) {
		power_state |= EN_PS_ENABLE;
		if (!chk_als_onoff(power_state)) {
			power_state |= EN_WAIT_ENABLE;
		}
	} else {
		power_state &= ~EN_PS_DISABLE;
	}

	result = ps_als_i2c_write_byte_data(ps_als->client,
					   REG_STATE, power_state);
	if (unlikely(result < 0)) {
		SENSOR_ERR_LOG("ps_als_i2c_write_byte_data failed:%d",result);
		/* i2c communication error */
		if (enable) {
			disable_irq(ps_als->client->irq);
		}
		mutex_unlock(&ps_als->ps_calib_lock);
		return result;
	}
	mutex_unlock(&ps_als->ps_calib_lock);

	ps_als->int_flg = false;

	if (enable) {
		ps_als_schedule_monitor(ps_als);
		ps_als->ps_det = PROX_DUMMY_VALUE;
		ps_als->power_data.power_ps = enable;
		ps_als_schedule_boot_ps_report(ps_als);
	} else {
		disable_irq(ps_als->client->irq);

		ps_als->ps_det = PROX_STATUS_FAR;
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);

		if (!ps_als->power_data.power_als) {
			result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
			if(unlikely(result < 0)) {
				SENSOR_ERR_LOG("failed to ps_als_power_ctrl");
				return result;
			}
		}
	}

	SENSOR_D_LOG("end. enable=%d", enable);
	return 0;
}

ssize_t als_val_show(char *buf)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->als_val.report_lux);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}

ssize_t ps_val_show(char *buf)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"%d\n", ps_als->ps_det);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
}

ssize_t als_status_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;
	int i;
	CALC_DATA *calc_data_h;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->als_data_lock);

	calc_data_h = ps_als->als_val.calc_data;

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"report_lux = %d\n", ps_als->als_val.report_lux);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	i = ps_als->als_val.index - 1;
	if (i < 0) {
		i = NUM_OF_ALS_VAL - 1;
		SENSOR_ERR_LOG("ps_als->als_val.index error :%d->%d",(ps_als->als_val.index-1),i);
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"index[%d], ave_enable[%d]\n", i, ps_als->als_val.ave_enable);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	for (i = 0; i < NUM_OF_ALS_VAL; i++) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].lux = %ld\n", i, calc_data_h[i].lux);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].als = %ld\n", i, calc_data_h[i].als);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
			"  calc_data[%d].c = %ld\n", i, calc_data_h[i].c);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_als = %d\n",
				   i, calc_data_h[i].gain_als);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].gain_c = %d\n",
				   i, calc_data_h[i].gain_c);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].als_output = %d\n",
				   i, calc_data_h[i].als_output);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].c_output = %d\n",
				   i, calc_data_h[i].c_output);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].ratio = %ld\n",
				   i, calc_data_h[i].ratio);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		switch(calc_data_h[i].src) {
		case LIGHT_FLUORESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_FLUORESCENT\n", i);
			break;
		case LIGHT_LED_BULB:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_LED_BULB\n", i);

			break;
		case LIGHT_INTERMEDIATE:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_INTERMEDIATE\n", i);

			break;
		case LIGHT_SOLAR:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SOLAR\n", i);

			break;
		case LIGHT_CANDESCENT:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_CANDESCENT\n", i);

			break;
		default:
			count += scnprintf(buf + count, PAGE_SIZE - count,
					   "  calc_data[%d].src = "
					   "LIGHT_SRC_MAX\n", i);

			break;
		}
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].alpha = 0x%04x\n",
				   i, calc_data_h[i].alpha);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}

		count += scnprintf(buf + count, PAGE_SIZE - count,
				   "  calc_data[%d].beta = 0x%04x\n",
				   i, calc_data_h[i].beta);
		if (count >= PAGE_SIZE) {
			SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
			goto fail_exit;
		}
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}

ssize_t ps_status_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;
	unsigned short ps_data;
	GENREAD_ARG gene_read;
	int result;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->control_lock);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps detect = %d\n", ps_als->ps_det);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps interrupt flg = %d\n", ps_als->int_flg);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps count = %d\n", ps_als->ps_val.data_ps);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps int flag = 0x%x\n", ps_als->ps_val.flag_ps);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	/* read interrupt flag */
	result = ps_als_i2c_read_byte_data(ps_als->client ,REG_FLAG);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"flag_ps = %x\n", result);
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_als->int_flg = %x\n", ps_als->int_flg);
	
	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	/* read start address */
	gene_read.adr_reg = REG_DATA_PS;
	gene_read.addr    = (char *)&ps_data;
	gene_read.size    = sizeof(ps_data);

	/* block read */
	result = ps_als_driver_general_read(gene_read, ps_als->client);
	if (result > 0) {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = %d\n",
				CONVERT_TO_BE(ps_data));
	} else {
		count += scnprintf(buf + count, PAGE_SIZE - count,
				"current ps count = 0\n");
	}
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_th = %d\n", ps_als->init_data.thdh_ps);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"ps_tl = %d\n", ps_als->init_data.thdl_ps);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			"data_ps_offset = %d\n", ps_als->init_data.data_ps_offset);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	count += scnprintf(buf + count, PAGE_SIZE - count,
						"--------\n");
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
		"errinfo = %d (detail:0x%08x)\n",
		(ps_als->errinfo & ERRINFO_RECOVERY_FAIL_MASK) ? 3 :
		(ps_als->errinfo & ERRINFO_SOFT_RESET_OCCURRED) ? 2 :
		(ps_als->errinfo & ERRINFO_I2C_RESET_OCCURRED) ? 1 : 0,
		ps_als->errinfo);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->control_lock);
	return PAGE_SIZE - 1;
}

ssize_t als_d0d1_show(char *buf)
{
	PS_ALS_DATA *ps_als= i2c_get_clientdata(client_stk3338);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "d0 : 0x%04x "
				   "d1 : 0x%04x\n",
				   ps_als->als_last_data.als,
				   ps_als->als_last_data.c);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}

ssize_t als_imit_show(char *buf)
{
	PS_ALS_DATA *ps_als= i2c_get_clientdata(client_stk3338);
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "imit_flg : %d "
				   "imit_d0 : %d "
				   "imit_d1 : %d\n",
				   ps_als->imit.imit_flg,
				   ps_als->imit.imit_d0,
				   ps_als->imit.imit_d1);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
}

ssize_t als_imit_store(const char *buf)
{
	PS_ALS_DATA *ps_als;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->als_data_lock);

	sscanf(buf, "%d %d %d",
		&ps_als->imit.imit_flg,
		&ps_als->imit.imit_d0,
		&ps_als->imit.imit_d1);

	SENSOR_ERR_LOG("imit_flg: %d imit_d0: %d imit_d1: %d",
					   ps_als->imit.imit_flg,
					   ps_als->imit.imit_d0,
					   ps_als->imit.imit_d1);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return 0;
}

ssize_t als_properties_store(const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[15] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&work[ 0], &work[ 1], &work[ 2], &work[ 3], &work[ 4],
		&work[ 5], &work[ 6], &work[ 7], &work[ 8], &work[ 9],
		&work[10], &work[11], &work[12], &work[13], &work[14]);

	if (cnt != 15) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&ps_als->als_data_lock);
	ps_als->color = color;
	nv_photosensor_th0[ps_als->color] = work[ 0];
	nv_photosensor_th1[ps_als->color] = work[ 1];
	nv_photosensor_th2[ps_als->color] = work[ 2];
	nv_photosensor_th3[ps_als->color] = work[ 3];
	nv_photosensor_th4[ps_als->color] = work[ 4];
	nv_photosensor_a0[ps_als->color]  = work[ 5];
	nv_photosensor_a1[ps_als->color]  = work[ 6];
	nv_photosensor_a2[ps_als->color]  = work[ 7];
	nv_photosensor_a3[ps_als->color]  = work[ 8];
	nv_photosensor_a4[ps_als->color]  = work[ 9];
	nv_photosensor_b0[ps_als->color]  = work[10];
	nv_photosensor_b1[ps_als->color]  = work[11];
	nv_photosensor_b2[ps_als->color]  = work[12];
	nv_photosensor_b3[ps_als->color]  = work[13];
	nv_photosensor_b4[ps_als->color]  = work[14];
	mutex_unlock(&ps_als->als_data_lock);
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}

ssize_t als_properties_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->als_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th0[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th1[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th2[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th3[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_th4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_th4[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a0[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a1[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a2[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a3[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_a4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_a4[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b0[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b0[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b1[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b1[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b2[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b2[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b3[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b3[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_photosensor_b4[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_photosensor_b4[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->als_data_lock);
	return count;
fail_exit:
	SENSOR_ERR_LOG("end - fail");
	mutex_unlock(&ps_als->als_data_lock);
	return PAGE_SIZE - 1;
}

ssize_t ps_properties_store(const char *buf)
{
	PS_ALS_DATA *ps_als;
	int cnt;
	int color;
	int offset;
	int work[7] = {0};
	const char *p;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->control_lock);

	cnt = sscanf(buf, "%d %n", &color, &offset);
	if (cnt != 1) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}
	if ((color < 0) ||(color >= NUM_OF_COLOR)) {
		SENSOR_ERR_LOG("error color:%d",color);
		goto fail_exit;
	}

	p = buf + offset;
	cnt = sscanf(p, "%x %x %x %x %x %x %x",
		&work[0], &work[1], &work[2], &work[3], &work[4], &work[5], &work[6]);
	if (cnt != ARRAY_SIZE(work) && cnt != 3) {
		SENSOR_ERR_LOG("sscanf errors cnt:%d",cnt);
		goto fail_exit;
	}

	mutex_lock(&ps_als->ps_data_lock);
	ps_als->color = color;
	nv_proximity_detect[ps_als->color] = work[0];
	nv_proximity_no_detect[ps_als->color] = work[1];
	nv_proximity_offset[ps_als->color] = work[2];
	if (cnt > 3) {
		nv_proximity_temp[0] = work[3];
		nv_proximity_temp[1] = work[4];
		nv_proximity_temp[2] = work[5];
	}
	nv_proximity_gap[ps_als->color]  = work[6];
	mutex_unlock(&ps_als->ps_data_lock);

	ps_check_thresholds_update(ps_als);

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return 0;
fail_exit:
	SENSOR_ERR_LOG("end - fail");
	mutex_unlock(&ps_als->control_lock);
	return (-EINVAL);
}

ssize_t ps_properties_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	ssize_t count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);
	mutex_lock(&ps_als->ps_data_lock);

	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "color is [%d]\n",
			   ps_als->color);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_detect[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_no_detect[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_no_detect[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_offset[%d] is [0x%04x]\n",
			   ps_als->color,
			   nv_proximity_offset[ps_als->color]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}
	count += scnprintf(buf + count, PAGE_SIZE - count,
			   "nv_proximity_temp is [0x%02x,0x%02x,0x%02x]\n",
			   nv_proximity_temp[0],
			   nv_proximity_temp[1],
			   nv_proximity_temp[2]);
	if (count >= PAGE_SIZE) {
		SENSOR_ERR_LOG("scnprintf count PAGE_SIZE over :%zd",count);
		goto fail_exit;
	}

	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->ps_data_lock);
	return count;
fail_exit:
	SENSOR_D_LOG("end - fail");
	mutex_unlock(&ps_als->ps_data_lock);
	return PAGE_SIZE - 1;
}

static int ps_als_suspend(struct device *dev)
{
	int result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	SENSOR_D_LOG("start");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	ps_als->host_en = HOST_DISABLE;
	flush_workqueue(sensortek_workqueue);
	mutex_lock(&ps_als->control_lock);
	ps_als->power_data_last.power_als = ps_als->power_data.power_als;
	ps_als->power_data_last.power_ps = ps_als->power_data.power_ps;

	if ((ps_als->power_data.power_als == PS_ALS_DISABLE) &&
	    (ps_als->power_data.power_ps == PS_ALS_DISABLE)) {
		SENSOR_D_LOG("skip [power_als]=%d [power_ps]=%d", ps_als->power_data.power_als,ps_als->power_data.power_ps);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	    exit = ktime_get();
		func_time_us = ktime_to_us(ktime_sub(exit, enter));
		enter_us = ktime_to_us(enter);
		exit_us = ktime_to_us(exit);
		printk(KERN_NOTICE "[IT_TEST] ERR ROUTE %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
		mutex_unlock(&ps_als->control_lock);
		return 0;
	}

	if (ps_als->power_data.power_als == PS_ALS_ENABLE) {
		result = als_enable_proc(ps_als, PS_ALS_DISABLE);
		if (result)
			SENSOR_ERR_LOG("Failed to disable light sensor[%d]", result);
	}
	if (ps_als->power_data.power_ps == PS_ALS_ENABLE) {
		enable_irq_wake(client->irq);
		SENSOR_D_LOG("call enable_irq_wake().");
	}

#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return 0;
}

ssize_t ps_valid_show(char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = i2c_get_clientdata(client_stk3338);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%u\n", atomic_read(&ps_als->ps_info.valid));
	SENSOR_D_LOG("end");
	return count;

}

ssize_t ps_valid_store(unsigned int valid)
{
	PS_ALS_DATA *ps_als;
	int last_valid;

	ps_als = i2c_get_clientdata(client_stk3338);

	SENSOR_D_LOG("start. valid = %d", valid);

	last_valid = atomic_read(&ps_als->ps_info.valid);

	mutex_lock(&ps_als->ps_data_lock);
	if (!valid)
		ps_sensor_report_event_proc(ps_als, 0);

	atomic_set(&ps_als->ps_info.valid, valid);

	if (valid && !last_valid)
		ps_sensor_report_event_proc(ps_als, ps_als->ps_det);

	mutex_unlock(&ps_als->ps_data_lock);

	SENSOR_D_LOG("end");
	return 0;

}

#define ALS_ELEM_NUM                (10)
#define PS_ELEM_NUM                 (10)
#define THROW_AWAY_NUM              (3)
#define ALS_IR_CANDESCENT_TH        (600)
#define ALS_IR_INTERMEDIATE_TH      (15000)
#define ALS_IR_SOLAR_TH             (15000)
#define ALS_IR_FLUORESCENT_TH       (1000)
#define ALS_IR_LEDBULB_TH           (1000)
#define ALS_IR_OVERFLOW_TH          (65535)
#define PS_CAL_DISTANCE_TH_WITHCAP  (174)
#define PS_CAL_DISTANCE_TH          (174+2000)



static void ps_cal_schedule(PS_ALS_DATA *ps_als)
{
	SENSOR_D_LOG("start");
	queue_work(sensortek_workqueue, &ps_als->ps_cal_work);
}

static int alsIR_cmp(const void *a, const void *b)
{
	const als_data_set *tmp1 = a;
	const als_data_set *tmp2 = b;
	int t1, t2;

	t1 = tmp1->c;
	t2 = tmp2->c;

	return t1 -t2;
}

static int psDATA_cmp(const void *a, const void *b)
{
	const int *t1 = a;
	const int *t2 = b;
	return *t1 - *t2;
}

static int32_t calc_ave(const int32_t sum, const int32_t param){return ( sum / param );}


static int32_t ps_calib_chk_lighterr(int8_t type, int32_t ir)
{
	int32_t result = PS_CAL_OK;

	SENSOR_D_LOG("start");
	switch(type){
		case LIGHT_CANDESCENT:
			if(ir >= ALS_IR_CANDESCENT_TH){
				result = PS_CAL_ERR_NEARLIGHT;
				SENSOR_ERR_LOG("Near the LightSource[%d], ir[%d]", LIGHT_CANDESCENT, ir);
			}
			break;
		case LIGHT_INTERMEDIATE:
			if(ir >= ALS_IR_INTERMEDIATE_TH){
				result = PS_CAL_ERR_NEARLIGHT;
				SENSOR_ERR_LOG("Near the LightSource[%d], ir[%d]", LIGHT_INTERMEDIATE, ir);
			}
			break;
		case LIGHT_SOLAR:
			if(ir >= ALS_IR_SOLAR_TH){
				result = PS_CAL_ERR_SUNLIGHT;
				SENSOR_ERR_LOG("Near the LightSource[%d], ir[%d]", LIGHT_SOLAR, ir);
			}
			break;
		case LIGHT_LED_BULB:
			if(ir >= ALS_IR_LEDBULB_TH){
				result = PS_CAL_ERR_NEARLIGHT;
				SENSOR_ERR_LOG("Near the LightSource[%d], ir[%d]", LIGHT_LED_BULB, ir);
			}
			break;
		case LIGHT_FLUORESCENT:
			if(ir >= ALS_IR_FLUORESCENT_TH){
				result = PS_CAL_ERR_NEARLIGHT;
				SENSOR_ERR_LOG("Under direct Sunlight[%d], ir[%d]", LIGHT_FLUORESCENT, ir);
			}
			break;

		default:
			result = PS_CAL_ERR_OTHER;
			SENSOR_ERR_LOG("Error:Unknown Light Source.");
			break;
	}

	SENSOR_D_LOG("end[%d]", result);
	return result;
}

static int8_t ps_calib_lightsrc_estimation(PS_ALS_DATA *ps_als, int32_t als_ave, int32_t c_ave, int32_t ratio)
{
	int8_t lsrc = -1;
	SENSOR_D_LOG("start");
	if( c_ave == ALS_IR_OVERFLOW_TH){
		lsrc = LIGHT_OVERFLOW;
		SENSOR_ERR_LOG("Light src is OVERFLOW");
	}
	else if( (0 <= ratio) && (ratio < nv_photosensor_th0[ps_als->color]) ){
		lsrc = LIGHT_CANDESCENT;
		SENSOR_ERR_LOG("Light src is CANDESCENT");
	}
	else if ( (nv_photosensor_th0[ps_als->color] <= ratio) && (ratio < nv_photosensor_th1[ps_als->color]) ){
		lsrc = LIGHT_INTERMEDIATE;
		SENSOR_ERR_LOG("Light src is INTERMEDIATE");
	}
	else if ( (nv_photosensor_th1[ps_als->color] <= ratio) && (ratio < nv_photosensor_th2[ps_als->color]) ){
		lsrc = LIGHT_SOLAR;
		SENSOR_ERR_LOG("Light src is SOLAR");
	}
	else if (nv_photosensor_th2[ps_als->color] <= ratio) {
		lsrc = LIGHT_LED_BULB;
		SENSOR_ERR_LOG("Light src is LED_BULB");
	}
	else if (nv_photosensor_th3[ps_als->color] <= ratio) {
		lsrc = LIGHT_FLUORESCENT;
		SENSOR_ERR_LOG("Light src is FLUORESCENT");
	}
	else {
		lsrc = -1;
		SENSOR_ERR_LOG("Unknown Light Source.");
	}
	SENSOR_D_LOG("end[%d]", lsrc);

	return lsrc;
}
static int32_t ps_calib_chk_lightsrc(PS_ALS_DATA *ps_als)
{
	int8_t chk_result = PS_CAL_OK;
	int result = 0;
	int32_t c_sum = 0;
	int32_t c_ave = 0;
	int32_t als_sum = 0;
	int32_t als_ave = 0;
	int32_t ratio = 0;
	int8_t lightsrc = 0;
	int i = 0;
	bool is_lightsrc_chk = false;
	READ_DATA_ALS_BUF read_data_buf;
	DEVICE_VAL *dev_val= &ps_als->dev_val;
	als_data_set data[ALS_ELEM_NUM];

	SENSOR_D_LOG("start");

	if(ALS_ELEM_NUM <= THROW_AWAY_NUM*2){
		SENSOR_ERR_LOG("Relation ALS_ELEM_NUM[%d] and THROW_AWAY_NUM[%d] is incorrect", ALS_ELEM_NUM, THROW_AWAY_NUM);
		return PS_CAL_ERR_OTHER;
	}

	for(i = 0; i < ALS_ELEM_NUM; i++){
		msleep(100);
			result = als_driver_read_data(&read_data_buf, ps_als->client);
		if (unlikely(result < 0)) {
			SENSOR_ERR_LOG("ERROR! read data");
			return PS_CAL_ERR_OTHER;
		}
		data[i].als		= (int)(read_data_buf.data_als / GAIN_TABLE_ALS[dev_val->gain_als].GAIN_ALS);
		data[i].c      	= (int)(read_data_buf.data_c   / GAIN_TABLE_C[dev_val->gain_c].GAIN_C);
		if(data[i].c > 0){
			is_lightsrc_chk = true;
		}
	}
	if(!is_lightsrc_chk){
		SENSOR_D_LOG("ALL IR values are 0");
		return chk_result;
	}

	sort(data, ALS_ELEM_NUM, sizeof(als_data_set), alsIR_cmp, NULL);

	for(i = THROW_AWAY_NUM; i < (ALS_ELEM_NUM-THROW_AWAY_NUM); i++){
		c_sum += (int32_t)data[i].c;
		als_sum += (int32_t)data[i].als;
	}
	c_ave = calc_ave(c_sum, (ALS_ELEM_NUM-(THROW_AWAY_NUM*2)));
	als_ave = calc_ave(als_sum, (ALS_ELEM_NUM-(THROW_AWAY_NUM*2)));

	if(als_ave == 0){
		SENSOR_ERR_LOG("ERROR! Visible ave is 0");
		return PS_CAL_ERR_OTHER;
	}
	ratio = (als_ave * 1000) / c_ave;

	lightsrc = ps_calib_lightsrc_estimation(ps_als, als_ave, c_ave, ratio);
	chk_result = ps_calib_chk_lighterr(lightsrc, c_ave);

	SENSOR_D_LOG("end [%d]", chk_result);

	return chk_result;
}

static void ps_calib_set_new_thresh(PS_ALS_DATA *ps_als, const uint16_t ps_th_base)
{
	SENSOR_D_LOG("start.");
	nv_proximity_detect[ps_als->color] = ps_th_base + nv_proximity_gap[ps_als->color];
	nv_proximity_no_detect[ps_als->color] = ps_th_base + nv_proximity_gap[ps_als->color] - 50;
	ps_update_thresholds(ps_als);
	ps_send_config(ps_als);
	SENSOR_D_LOG("end");
}

static int8_t ps_calib_chk_pscnt(PS_ALS_DATA *ps_als, uint16_t* ps_th_base)
{
	int32_t ps_cnt[PS_ELEM_NUM];
	int32_t ps_cnt_sum = 0;
	int32_t ps_cnt_ave = 0;
	int8_t chk_result = PS_CAL_OK;
	int i = 0;

	SENSOR_D_LOG("start.");

	for(i = 0; i < PS_ELEM_NUM; i++){
		msleep(50);
		ps_driver_read_data(client_stk3338);
		ps_cnt[i] = (int32_t)ps_als->ps_val.data_ps;
	}
	sort(ps_cnt, PS_ELEM_NUM, sizeof(int32_t), psDATA_cmp, NULL);

	for(i = THROW_AWAY_NUM; i < (PS_ELEM_NUM-THROW_AWAY_NUM); i++){
		ps_cnt_sum += ps_cnt[i];
	}
	ps_cnt_ave = calc_ave(ps_cnt_sum, (PS_ELEM_NUM-(THROW_AWAY_NUM*2)));

	if(ps_cnt_ave <= (int32_t)ps_als->distance_threshold){
		*ps_th_base = ps_cnt_ave;
	} else {
		SENSOR_ERR_LOG("ps cnt is over threshold");
		chk_result = PS_CAL_ERR_NEAROBJ;
	}

	SENSOR_D_LOG("end");
	return chk_result;
}
static void ps_cal_func(struct work_struct *ps_cal_work)
{
	uint8_t back_statectrl = 0;
	int8_t result = 0;
	int8_t cal_result = PS_CAL_OK;
	uint16_t ps_th_base = 0;
	PS_ALS_DATA   *ps_als = container_of(ps_cal_work, PS_ALS_DATA, ps_cal_work);

	SENSOR_D_LOG("start");

	if(ps_als->is_shtdwn_running){
		SENSOR_ERR_LOG("This process was skipped due to shutdown sequence running.");
		return;
	}

	SENSOR_ERR_LOG("Proximity Sensor Calibration Start...");

	mutex_lock(&ps_als->ps_calib_lock);
	ps_als->ps_is_calibrating = true;
	back_statectrl = ps_als_i2c_read_byte_data(ps_als->client, REG_STATE);
	if(unlikely(back_statectrl < 0)) {
		SENSOR_ERR_LOG("i2c communication error");
		cal_result = PS_CAL_ERR_OTHER;
		goto cal_end_mdctrl_bak;
	}
	result = ps_als_i2c_write_byte_data(ps_als->client, REG_STATE, 0x03);
	if(unlikely(result < 0)) {
		SENSOR_ERR_LOG("i2c communication error");
		cal_result = PS_CAL_ERR_OTHER;
		goto cal_end;
	}

	cal_result = ps_calib_chk_lightsrc(ps_als);
	if(cal_result < 0){
		SENSOR_ERR_LOG("Light source error[%d]", cal_result);
		goto cal_end;
	}

	cal_result = ps_calib_chk_pscnt(ps_als, &ps_th_base);
	if(cal_result < 0){
		SENSOR_ERR_LOG("Proximity count error [%d]", cal_result);
		goto cal_end;
	}

	ps_calib_set_new_thresh(ps_als, ps_th_base);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	prox_cal_interrupt(cal_result);
#else
    ps_sensor_cal_report_event(&ps_als->ps_info, cal_result);
#endif
cal_end:
	ps_als_i2c_write_byte_data(ps_als->client, REG_STATE, (unsigned char)back_statectrl);

cal_end_mdctrl_bak:
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	prox_cal_result_set(cal_result);
#else
    atomic_set(&s_cal_result, cal_result);
#endif
	ps_als->ps_is_calibrating = false;

	mutex_unlock(&ps_als->ps_calib_lock);
	SENSOR_ERR_LOG("Proximity Sensor Calibration End... result=>%d", cal_result);
	SENSOR_D_LOG("end");
}

void ps_calib_start(void)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);

	SENSOR_D_LOG("start.");
#ifdef CONFIG_USE_SENSOR_KC_BOARD
	if (OEM_get_board() == OEM_BOARD6_TYPE || OEM_get_board() == OEM_BOARD7_TYPE) {
		ps_als->distance_threshold = PS_CAL_DISTANCE_TH;
	}
	else {
		ps_als->distance_threshold = PS_CAL_DISTANCE_TH_WITHCAP;
	}
#else
	ps_als->distance_threshold = PS_CAL_DISTANCE_TH;
#endif
	SENSOR_D_LOG("set distance_threshold [%d]", ps_als->distance_threshold);
	if(!ps_als->ps_is_calibrating)
		ps_cal_schedule(ps_als);
	else
		SENSOR_ERR_LOG("Do not run PS calirb.[Already Running].");
	SENSOR_D_LOG("end");
}

void ps_get_threshold(uint16_t* th_detect, uint16_t* th_no_detect, uint16_t* distance_th)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);
	SENSOR_D_LOG("start");
	*th_detect = (uint16_t)ps_als->init_data.thdh_ps;
	*th_no_detect = (uint16_t)ps_als->init_data.thdl_ps;
	*distance_th = ps_als->distance_threshold;
	SENSOR_D_LOG("end");
}

bool ps_get_thresh_adj(void)
{
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end");
	return prox_adj;
}

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t als_flush_store(unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_stk3338);
	als_sensor_report_flush_event(&ps_als->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int als_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_stk3338);
	als_sensor_report_flush_event(&ps_als->als_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
ssize_t ps_flush_store(unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_stk3338);
	ps_sensor_report_flush_event(&ps_als->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#else /* CONFIG_USE_MICON_SOFT_STRUCTURE */
static int ps_flush_store(struct sensor_api_info *sai, unsigned int flush)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	SENSOR_D_LOG("start");

	ps_als = i2c_get_clientdata(client_stk3338);
	ps_sensor_report_flush_event(&ps_als->ps_info);

	SENSOR_D_LOG("end");
	return result;
}
#endif /* !CONFIG_USE_MICON_SOFT_STRUCTURE */

static int ps_als_resume(struct device *dev)
{
	int result = 0;
	struct i2c_client *client = to_i2c_client(dev);
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

	SENSOR_D_LOG("start");
	mutex_lock(&ps_als->control_lock);
	ps_als->host_en = HOST_ENABLE;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	if ((ps_als->power_data_last.power_als == PS_ALS_DISABLE) &&
	    (ps_als->power_data_last.power_ps == PS_ALS_DISABLE)) {
		SENSOR_D_LOG("skip");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	    exit = ktime_get();
		func_time_us = ktime_to_us(ktime_sub(exit, enter));
		enter_us = ktime_to_us(enter);
		exit_us = ktime_to_us(exit);
		printk(KERN_NOTICE "[IT_TEST] ERR ROUTE %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
		goto exit;
	}


	if (ps_als->power_data_last.power_als == PS_ALS_ENABLE) {
		result = als_enable_proc(ps_als, ps_als->power_data_last.power_als);
		if (result)
			SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
	}
	if (ps_als->power_data_last.power_ps == PS_ALS_ENABLE) {
		disable_irq_wake(client->irq);
		SENSOR_D_LOG("call disable_irq_wake().");
	}

exit:
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
    exit = ktime_get();
    func_time_us = ktime_to_us(ktime_sub(exit, enter));
    enter_us = ktime_to_us(enter);
    exit_us = ktime_to_us(exit);
    printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	return result;
}

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
static int als_val_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;

	SENSOR_D_LOG("start");
	count = (int)als_val_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int ps_val_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;

	SENSOR_D_LOG("start");
	count = (int)ps_val_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int als_status_no_micon_show(struct sensor_api_info *sai, char *buf)
{
    int ret;
	SENSOR_D_LOG("start");
    ret = (int)als_status_show(buf);
	SENSOR_D_LOG("end");
    return ret;
}

static int ps_status_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	ssize_t count = 0;
	SENSOR_D_LOG("start");
    count = (int)ps_status_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int als_d0d1_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;

	SENSOR_D_LOG("start");
    count = (int)als_d0d1_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int als_imit_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;

	SENSOR_D_LOG("start");
    count = (int)als_imit_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int als_imit_no_micon_store(struct sensor_api_info *sai, const char *buf)
{
    return als_imit_store(buf);
}

static int als_enable_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, als_info.sai);
	mutex_lock(&ps_als->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", ps_als->als_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return count;
}

static int ps_enable_show(struct sensor_api_info *sai, char *buf)
{
	PS_ALS_DATA *ps_als;
	int count = 0;

	SENSOR_D_LOG("start");
	ps_als = container_of(sai, PS_ALS_DATA, ps_info.sai);
	mutex_lock(&ps_als->control_lock);

	count += scnprintf(buf, PAGE_SIZE - count,
				   "%d\n", ps_als->ps_en_cnt);
	SENSOR_D_LOG("end");
	mutex_unlock(&ps_als->control_lock);
	return count;
}

static int als_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
    bool en = !!enable;
    return (int)als_sensor_activate(en);
}

static int ps_enable_store(struct sensor_api_info *sai, unsigned int enable)
{
    bool en = !!enable;
    return (int)ps_sensor_activate(en);
}

static int als_properties_no_micon_store(struct sensor_api_info *sai, const char *buf)
{
	int ret;
    ret = (int)als_properties_store(buf);
	if (ret) {
		SENSOR_ERR_LOG("als_properties_store errors :%d",ret);
		goto fail_exit;
	}
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}

static int als_properties_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;
	SENSOR_D_LOG("start");
    count = als_properties_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int ps_properties_no_micon_store(struct sensor_api_info *sai, const char *buf)
{
    int ret;
    ret = (int)ps_properties_store(buf);
	if (ret) {
		SENSOR_ERR_LOG("ps_properties_store errors ret:%d",ret);
		goto fail_exit;
	}
	SENSOR_D_LOG("end");
	return 0;
fail_exit:
	SENSOR_D_LOG("end - fail");
	return (-EINVAL);
}

static int ps_properties_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;
	SENSOR_D_LOG("start");
    count = (int)ps_properties_show(buf);
	SENSOR_D_LOG("end");
    return count;
}

static int ps_valid_no_micon_show(struct sensor_api_info *sai, char *buf)
{
	int count = 0;

	SENSOR_D_LOG("start");
	count =  ps_valid_show(buf);
	SENSOR_D_LOG("end");
	return count;
}

static int ps_valid_no_micon_store(struct sensor_api_info *sai, unsigned int valid)
{
	return (int)ps_valid_store(valid);
}

static int ps_thresh_show(struct sensor_api_info *sai, char *buf)
{
    int count = 0;
    uint16_t th_detect = 0;
    uint16_t th_no_detect = 0;
    uint16_t distance_th = 0;

    SENSOR_D_LOG("start");
    ps_get_threshold(&th_detect, &th_no_detect, &distance_th);
    count = (int)sprintf(buf, "%x %x %x\n", th_detect, th_no_detect, distance_th);
    SENSOR_D_LOG("end");
    return count;
}

#define PS_CAL_NORMAL   1

static int ps_calib_store(struct sensor_api_info *sai, const char *buf)
{
    unsigned long calib_kind = PS_CAL_NORMAL;
    SENSOR_D_LOG("start");
    atomic_set(&s_cal_result, PS_CALIBRATING);
    switch(calib_kind){
        case PS_CAL_NORMAL :
            ps_calib_start();
            break;
        default:
            break;
    }
    SENSOR_D_LOG("end [%d]", atomic_read(&s_cal_result));
    return 0;
}

static int ps_calib_show(struct sensor_api_info *sai, char *buf)
{
    int count = 0;
    SENSOR_D_LOG("start");
    count = (int)sprintf(buf, "%d\n", atomic_read(&s_cal_result));
    SENSOR_D_LOG("end");
    return count;
}

const static struct ps_sensor_info ps_info_base = {
	.sai  = {
		.enable_store = ps_enable_store,
		.enable_show  = ps_enable_show,
		.value_show   = ps_val_no_micon_show,
		.status_show  = ps_status_no_micon_show,
		.prop_store   = ps_properties_no_micon_store,
		.prop_show    = ps_properties_no_micon_show,
		.valid_store  = ps_valid_no_micon_store,
		.valid_show   = ps_valid_no_micon_show,
		.flush_store  = ps_flush_store,
		.thresh_show  = ps_thresh_show,
		.calib_store  = ps_calib_store,
		.calib_show   = ps_calib_show,
	}
};

const static struct als_sensor_info als_info_base = {
	.sai  = {
		.enable_store = als_enable_store,
		.enable_show = als_enable_show,
		.value_show = als_val_no_micon_show,
		.status_show = als_status_no_micon_show,
		.imit_store = als_imit_no_micon_store,
		.imit_show = als_imit_no_micon_show,
		.prop_store = als_properties_no_micon_store,
		.prop_show = als_properties_no_micon_show,
		.flush_store = als_flush_store,
	}
};
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

static void ps_als_get_regulator_info(PS_ALS_DATA *ps_als)
{
    SENSOR_ERR_LOG("start");
	if (!ps_als->ps_als_vdd_is_gpio) {
		ps_als->ps_als_vdd.v_reg = regulator_get(&ps_als->client->dev, "psals-vdd");
	}
	if (!ps_als->ps_als_vleda_is_gpio) {
		ps_als->ps_als_vleda.v_reg = regulator_get(&ps_als->client->dev, "psals-vleda");
	}
    SENSOR_D_LOG("end");
}


static int ps_als_parse_dt(PS_ALS_DATA *ps_als)
{
	struct device_node *of_node = ps_als->client->dev.of_node;
	struct device_node *child_node;
	int ret = 0 ;
	const char *desc = "stk3338,irq-gpio";
	int chip_vendor = 0;

	//todo:delete
	SENSOR_ERR_LOG("start");

	child_node = of_get_child_by_name(of_node, "chip-vendor");
	ps_als->ps_als_vdd_is_ldo = of_property_read_bool(of_node, "stk3338,psals-supply-is-ldo");
	ps_als->ps_als_vleda_is_gpio = of_property_read_bool(of_node, "stk3338,psals-vleda-is-gpio");
	ps_als->ps_als_vdd_is_gpio = of_property_read_bool(of_node, "stk3338,psals-vdd-is-gpio");
	ps_als->ps_als_is_PMIC_supply_ctrl = of_property_read_bool(of_node, "stk3338,psals-PMIC-supply-ctrl");
	prox_adj = of_property_read_bool(of_node, "need_prox_thresh_adj");

	if (!child_node) {
		SENSOR_ERR_LOG("child_node %s is not found", "chip-vendor");
		return -EINVAL;
	}

	if(of_property_read_u32(child_node, "type", &chip_vendor)){
		SENSOR_ERR_LOG("chip-vendor type is not found");
		return -EINVAL;
	}
	if(chip_vendor == 1){
		ps_als->irq_gpio = of_get_named_gpio(of_node, "stk3338,irq-gpio", 0);
		SENSOR_D_LOG("ps_als->irq_gpio = %d #################\n",ps_als->irq_gpio);
		if(ps_als->irq_gpio < 0) {
			SENSOR_ERR_LOG("failed to get irq GPIO=%d",ps_als->irq_gpio);
			return -1;
		}else{
			ret = devm_gpio_request_one(&ps_als->client->dev, ps_als->irq_gpio, GPIOF_IN,desc);
			SENSOR_D_LOG("devm_gpio_request_one ############ ret = %d \n", ret);
			if (ret < 0) {
				printk("%s: gpio request fail \n", __func__);
				return -1;
			}
			ps_als->client->irq = gpio_to_irq(ps_als->irq_gpio);
			SENSOR_D_LOG("ps_als->client->irq = %d ##################\n",ps_als->client->irq);
			if (ps_als->client->irq < 0) {
				printk("Unable to get irq number for GPIO %d\n",ps_als->irq_gpio);
				return ret;
			}
		}
	}

    if(ps_als->ps_als_is_PMIC_supply_ctrl){
         of_property_read_u32(of_node,
            "psals-vdd-min-voltage", &ps_als->ps_als_vdd.min_uV);
         of_property_read_u32(of_node,
             "psals-vdd-max-voltage", &ps_als->ps_als_vdd.max_uV);
         of_property_read_u32(of_node,
            "psals-vdd-on-load-current", &ps_als->ps_als_vdd.on_load_uA);
         of_property_read_u32(of_node,
                 "psals-vdd-off-load-current", &ps_als->ps_als_vdd.off_load_uA);
         SENSOR_D_LOG("regulator Vdd min_uV = %d, max_uV = %d, "
                     "on_load_uA = %d, off_load_uA = %d",
                     ps_als->ps_als_vdd.min_uV,
                     ps_als->ps_als_vdd.max_uV,
                     ps_als->ps_als_vdd.on_load_uA,
                     ps_als->ps_als_vdd.off_load_uA);

         of_property_read_u32(of_node,
             "psals-vleda-min-voltage", &ps_als->ps_als_vleda.min_uV);
         of_property_read_u32(of_node,
             "psals-vleda-max-voltage", &ps_als->ps_als_vleda.max_uV);
         of_property_read_u32(of_node,
            "psals-vleda-on-load-current", &ps_als->ps_als_vleda.on_load_uA);
         of_property_read_u32(of_node,
             "psals-vleda-off-load-current", &ps_als->ps_als_vleda.off_load_uA);
         SENSOR_D_LOG("regulator Vleda min_uV = %d, max_uV = %d, "
                     "on_load_uA = %d, off_load_uA = %d",
                     ps_als->ps_als_vleda.min_uV,
                     ps_als->ps_als_vleda.max_uV,
                     ps_als->ps_als_vleda.on_load_uA,
                     ps_als->ps_als_vleda.off_load_uA);
         }

	ps_als_get_regulator_info(ps_als);
/*
	of_property_read_u32(of_node,
			"stk3338,psals-power-offon-interval-ms",
			&ps_als->ps_als_power.power_off_on_interval_ms);
	of_property_read_u32(of_node,
			"stk3338,psals-power-on-wait-ms",
			&ps_als->ps_als_power.power_on_wait_ms);
	of_property_read_u32(of_node,
			"stk3338,psals-power-normal-wait-ms",
			&ps_als->ps_als_power.power_normal_wait_ms);
*/
	SENSOR_D_LOG("end");
	return 0;
}

static void ps_als_regulator_low(PS_ALS_DATA *ps_als)
{
	int err;
	SENSOR_D_LOG("start");
	if (!ps_als->ps_als_vdd_is_ldo) {
		if(ps_als->ps_als_is_PMIC_supply_ctrl){
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
			err = regulator_set_load(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.off_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_load fail. err=%d\n", __func__, err);
			}

          err = regulator_set_voltage(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.min_uV, ps_als->ps_als_vdd.max_uV);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_voltage fail(2). err=%d\n", __func__, err);
			}

			err = regulator_set_load(ps_als->ps_als_vleda.v_reg, ps_als->ps_als_vleda.off_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_load fail(3). err=%d\n", __func__, err);
			}

          err = regulator_set_voltage(ps_als->ps_als_vleda.v_reg, ps_als->ps_als_vleda.min_uV, ps_als->ps_als_vleda.max_uV);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_voltage fail(4). err=%d\n", __func__, err);
			}
#else
			err = regulator_set_optimum_mode(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.off_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
			}

#endif /* LINUX_VERSION_CODE */
		} else {
			SENSOR_D_LOG("[ALS_PS]%s Don't control PMIC supply\n", __func__);
		}
	}
	SENSOR_D_LOG("end");
}

static void ps_als_regulator_normal(PS_ALS_DATA *ps_als)
{
	int err;
	SENSOR_D_LOG("start");
//#if 0
	if (!ps_als->ps_als_vdd_is_ldo) {
		if(ps_als->ps_als_is_PMIC_supply_ctrl){
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
			err = regulator_set_load(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.on_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_load fail. err=%d\n", __func__, err);
			}

          err = regulator_set_voltage(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.min_uV, ps_als->ps_als_vdd.max_uV);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_voltage fail(2). err=%d\n", __func__, err);
			}

			err = regulator_set_load(ps_als->ps_als_vleda.v_reg, ps_als->ps_als_vleda.on_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_load fail(3). err=%d\n", __func__, err);
			}

          err = regulator_set_voltage(ps_als->ps_als_vleda.v_reg, ps_als->ps_als_vleda.min_uV, ps_als->ps_als_vleda.max_uV);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_voltage fail(4). err=%d\n", __func__, err);
			}
#else
			err = regulator_set_optimum_mode(ps_als->ps_als_vdd.v_reg, ps_als->ps_als_vdd.on_load_uA);
			if( err < 0 ) {
				SENSOR_ERR_LOG("[ALS_PS]%s regulator_set_optimum_mode fail. err=%d\n", __func__, err);
			}

#endif /* LINUX_VERSION_CODE */
		} else {
			SENSOR_D_LOG("[ALS_PS]%s Don't control PMIC supply\n", __func__);
		}
	}
//#endif
	SENSOR_D_LOG("end");
}

static int ps_als_regulator_enable(PS_ALS_DATA *ps_als, bool enable)
{
	int err = 0;
#if 0
	SENSOR_D_LOG("start enable[%d]", enable);

	if (enable != ps_als->ps_als_power.enabled) {
		if (enable) {
			if (ktime_to_ms(ktime_sub(ktime_get(), ps_als->ps_als_power.power_off_time)) < ps_als->ps_als_power.power_off_on_interval_ms) {
				usleep_range(ps_als->ps_als_power.power_off_on_interval_ms*1000,
							 ps_als->ps_als_power.power_off_on_interval_ms*1000);
			}
			if (ps_als->ps_als_vdd_is_ldo) {
				gpio_set_value(ps_als->vprox_gpio, 1);
			} else {
				err = regulator_enable(ps_als->ps_als_vcc.v_reg);
			}
			if (!err) {
				ps_als->ps_als_power.power_off_time = ktime_get();
			} else  {
				SENSOR_ERR_LOG("regulator_enable fail. err=%d\n", err);
			}
		} else {
			if (ps_als->ps_als_vdd_is_ldo) {
				gpio_set_value(ps_als->vprox_gpio, 0);
			} else {
				err = regulator_disable(ps_als->ps_als_vcc.v_reg);
			}
			if (!err) {
				ps_als->ps_als_power.power_off_time = ktime_get();
			} else {
				SENSOR_ERR_LOG("regulator_disable fail. err=%d\n", err);
			}
		}

		if (!err) {
			ps_als->ps_als_power.enabled = enable;
		}
	}
#endif
	SENSOR_D_LOG("end. err[%d]", err);

	return err;
}

static int ps_als_pinctrl_select(enum psals_pinctrl_id pin_id)
{
	int ret;
	struct pinctrl_state *pins_state = psals_pinctrl_state[pin_id];

	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(psals_pinctrl, pins_state);
		if (ret) {
			SENSOR_ERR_LOG("can not set %s pins", psals_pinctrl_names[pin_id]);
			return ret;
		}
	} else {
		SENSOR_ERR_LOG("not a valid '%s' pinstate",
						psals_pinctrl_names[pin_id]);
	}

	return 0;
}
static int ps_als_pinctrl_init(PS_ALS_DATA *ps_als)
{
	int ret = 0;
	int i = 0;
	struct device *dev = &ps_als->client->dev;

	SENSOR_D_LOG("start");
	psals_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(psals_pinctrl)) {
		SENSOR_ERR_LOG("Target does not use pinctrl");
		ret = PTR_ERR(psals_pinctrl);
		psals_pinctrl = NULL;
		return ret;
	}

	for(i = 0; i < PSALS_PIN_ID_MAX; i++){
		 psals_pinctrl_state[i]
		 = pinctrl_lookup_state(psals_pinctrl, psals_pinctrl_names[i]);
		if (IS_ERR(psals_pinctrl_state[i])) {
				SENSOR_ERR_LOG("cannot find '%s'", psals_pinctrl_names[i]);
				psals_pinctrl_state[i] = NULL;
				continue;
		}
		SENSOR_D_LOG("found psals pinctrl '%s'", psals_pinctrl_names[i]);
	}

	SENSOR_D_LOG("end");

	return 0;
}

static int ps_als_power_ctrl(PS_ALS_DATA *ps_als, enum ps_als_power_ctrl_mode request_mode)
{
	int ret = 0;
	int wait_ms = 0;

	SENSOR_D_LOG("start prev_mode[%d] request_mode[%d]", ps_als->ps_als_power.prev_mode, request_mode);
	if (request_mode >= PS_ALS_POWER_CTRL_MAX) {
		SENSOR_ERR_LOG("param err");
		return -1;
	}

	mutex_lock(&ps_als->ps_als_power.ps_als_power_mutex);
	if (request_mode != ps_als->ps_als_power.prev_mode) {
		switch(request_mode) {
		case PS_ALS_POWER_CTRL_OFF:
			ret = ps_als_pinctrl_select(PSALS_PIN_SUSPEND);
			if (ret) {
				SENSOR_ERR_LOG("ps_als_pinctrl_select err ret:%d",ret);
				break;
			}
			ps_als_regulator_low(ps_als);
			ret = ps_als_regulator_enable(ps_als, false);
			break;
		case PS_ALS_POWER_CTRL_LOW:
			ret = ps_als_pinctrl_select(PSALS_PIN_ACTIVE);
			if (ret) {
				SENSOR_ERR_LOG("ps_als_pinctrl_select err ret:%d",ret);
				break;
			}
			ps_als_regulator_low(ps_als);
			ret = ps_als_regulator_enable(ps_als, true);
			wait_ms = (ps_als->ps_als_power.prev_mode == PS_ALS_POWER_CTRL_OFF) ?
				ps_als->ps_als_power.power_on_wait_ms : 0;
			break;
		case PS_ALS_POWER_CTRL_NORMAL:
			ret = ps_als_pinctrl_select(PSALS_PIN_ACTIVE);
			if (ret) {
				SENSOR_ERR_LOG("ps_als_pinctrl_select err ret:%d",ret);
				break;
			}
			ps_als_regulator_normal(ps_als);
			ret = ps_als_regulator_enable(ps_als, true);
			wait_ms = (ps_als->ps_als_power.prev_mode == PS_ALS_POWER_CTRL_OFF) ?
				ps_als->ps_als_power.power_on_wait_ms : ps_als->ps_als_power.power_normal_wait_ms;

			usleep_range(1000,1000);

			break;
		default:
			SENSOR_ERR_LOG("invalid request_mode :%d",request_mode);
			break;
		}
	}

	if (!ret) {
//		if (wait_ms) {
//			SENSOR_D_LOG("wait %dms", wait_ms);
//			usleep_range(wait_ms*1000,wait_ms*1000);
//		}
		ps_als->ps_als_power.prev_mode = request_mode;
	} else {
		SENSOR_ERR_LOG("power_ctrl fail. ret=%d\n", ret);
	}

	mutex_unlock(&ps_als->ps_als_power.ps_als_power_mutex);

	SENSOR_D_LOG("end. ret[%d]", ret);
	return ret;
}

static int ps_als_power_src_enable(PS_ALS_DATA *ps_als, bool enable)
{
    const int temp_wait_us = 2000;
    //test
    SENSOR_ERR_LOG("start");
	if (enable) {
		SENSOR_ERR_LOG("STK3338 Power ON");

		//VDD ON
		if(ps_als->ps_als_vdd_is_gpio){
			ps_als_pinctrl_select(PSALS_PIN_VPROX_18_ACTIVE);
		} else {
			regulator_enable(ps_als->ps_als_vdd.v_reg);
		}
		usleep_range(temp_wait_us, temp_wait_us);
		//VLEDA ON
		if(ps_als->ps_als_vleda_is_gpio){
			ps_als_pinctrl_select(PSALS_PIN_VPROX_32_ACTIVE);
		} else {
			regulator_enable(ps_als->ps_als_vleda.v_reg);
		}

		msleep(35);
	} else {
		//pow off
		SENSOR_ERR_LOG("STK3338 Power OFF");
		//STATUS to standby
		
		//VLEDA OFF
		if(ps_als->ps_als_vleda_is_gpio){
			ps_als_pinctrl_select(PSALS_PIN_VPROX_32_SUSPEND);
		} else {
			regulator_disable(ps_als->ps_als_vleda.v_reg);
		}
		usleep_range(temp_wait_us, temp_wait_us);
		//VDD OFF
		if(ps_als->ps_als_vdd_is_gpio){
			ps_als_pinctrl_select(PSALS_PIN_VPROX_18_SUSPEND);
		} else {
			regulator_disable(ps_als->ps_als_vdd.v_reg);
		}

	}

    SENSOR_ERR_LOG("end");
	return 0;
}

/******************************************************************************
 * NAME       : ps_als_probe
 * FUNCTION   : initialize system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	PS_ALS_DATA *ps_als;
	int result;
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	s64 func_time_us, enter_us, exit_us;
	ktime_t enter, exit;
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	SENSOR_ERR_LOG("called ps_als_probe for stk3338 start!!");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	enter = ktime_get();
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	stk3338_initialize = 0;
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	result = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (unlikely(!result)) {
		SENSOR_ERR_LOG("need I2C_FUNC_I2C");
		result = -ENODEV;
		goto err_check_functionality_failed;
	}
	ps_als = kzalloc(sizeof(*ps_als), GFP_KERNEL);
	if (unlikely(ps_als == NULL)) {
		result = -ENOMEM;
		SENSOR_ERR_LOG("kzalloc error");
		goto err_alloc_data_failed;
	}
	INIT_WORK(&ps_als->als_work, als_work_func);
	INIT_WORK(&ps_als->ps_work, ps_work_func);
	INIT_WORK(&ps_als->ps_cal_work, ps_cal_func);
	INIT_DELAYED_WORK(&ps_als->monitor_dwork, ps_als_monitor_func);
	INIT_DELAYED_WORK(&ps_als->boot_ps_report_work, ps_als_boot_ps_report_func);
	mutex_init(&ps_als->control_lock);
	mutex_init(&ps_als->ps_data_lock);
	mutex_init(&ps_als->als_data_lock);
	mutex_init(&ps_als->ps_calib_lock);
	mutex_lock(&ps_als->control_lock);
	mutex_lock(&ps_als->ps_data_lock);
	mutex_lock(&ps_als->als_data_lock);
	ps_als->client = client;
	i2c_set_clientdata(client, ps_als);

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	ps_als->als_info = als_info_base;
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	ps_als->als_info.dev = &client->dev;
	result = als_sensor_register(&ps_als->als_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to als_sensor_register");
		goto err_als_register_failed;
	}

#ifndef CONFIG_USE_MICON_SOFT_STRUCTURE
	ps_als->ps_info = ps_info_base;
#else
//	atomic_set(&ps_als->ps_info.valid, 1);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	ps_als->ps_info.dev = &client->dev;
	result = ps_sensor_register(&ps_als->ps_info);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("Failed to ps_sensor_register");
		goto err_ps_register_failed;
	}

	result = ps_als_parse_dt(ps_als);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to parse dt");
		goto err_drv_init_failed;
	}
	ps_als->ps_als_power.enabled = false;

	result = ps_als_pinctrl_init(ps_als);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to ps_als_pinctrl_init");
	//	goto err_gpio_free;
	}

	result = ps_als_power_src_enable(ps_als, true);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("ps_als_power_src_enable error:%d",result);
		goto err_drv_init_failed;
	}

	ps_als->ps_als_power.prev_mode = PS_ALS_POWER_CTRL_OFF;
	mutex_init(&ps_als->ps_als_power.ps_als_power_mutex);
	result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_NORMAL);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_gpio_free;
	}

	ps_als->init_data.state       		= PS_ALS_SET_STATE;
	ps_als->init_data.psctrl      		= PS_ALS_SET_PSCTRL;
	ps_als->init_data.alsctrl1    		= PS_ALS_SET_ALSCTRL1;
	ps_als->init_data.vcselctrl   		= PS_ALS_SET_VCSELCTRL;
	ps_als->init_data.intctrl1    		= PS_ALS_SET_INTCTRL1;
	ps_als->init_data.wait        		= PS_ALS_SET_WAIT;
	ps_als->init_data.thdh_ps     		= nv_proximity_detect[ps_als->color];
	ps_als->init_data.thdl_ps     		= nv_proximity_no_detect[ps_als->color];
	ps_als->init_data.data_ps_offset   	= nv_proximity_offset[ps_als->color];
	ps_als->init_data.alsctrl2     		= PS_ALS_SET_ALSCTRL2;
	ps_als->init_data.intelli_wait_ps	= PS_ALS_SET_INTELLI_WAIT_PS;
	ps_als->init_data.sysctrl1     		= PS_ALS_SET_SYSCTRL1;
	ps_als->init_data.pspdctrl		  	= PS_ALS_SET_PSPDCTRL;
	ps_als->init_data.intctrl2    		= PS_ALS_SET_INTCTRL2;
	ps_als->init_data.thd_psoff    		= PS_ALS_SET_THD_PSOFF;
	ps_als->init_data.thd_bgir   		= PS_ALS_SET_BGIR;

	result = ps_als_driver_init(&ps_als->init_data, client);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to driver init");
		goto err_power_on_failed;
	}
	atomic_set(&ps_als->ps_info.valid, 1);

	result = ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_LOW);
	if (unlikely(result)) {
		SENSOR_ERR_LOG("failed to power on device");
		goto err_power_on_failed;
	}

	ps_als->power_data.power_als = PS_ALS_DISABLE;
	ps_als->power_data.power_ps = PS_ALS_DISABLE;
	ps_als->power_data_last.power_als = PS_ALS_DISABLE;
	ps_als->power_data_last.power_ps = PS_ALS_DISABLE;

	ps_als->is_shtdwn_running = false;
	result = ps_als_i2c_read_byte_data(ps_als->client, 0x3E);
	if (result != 0x51) {
		SENSOR_ERR_LOG("ps_als_i2c_read_byte_data error result retries\n");
	}
	/* check whether to use interrupt or not */
	if (client->irq) {
		/* interrupt process */
		result = request_threaded_irq(client->irq, NULL, ps_irq_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ps_als);

		/* 1 : interrupt mode/ 0 : polling mode */
		if (result == 0) {
			enable_irq(client->irq);
		} else {
			SENSOR_ERR_LOG("request IRQ Failed==>result : %d", result);
			SENSOR_ERR_LOG("client->irq        = 0x%x", client->irq);
			SENSOR_ERR_LOG("ps_irq_handler 	   = 0x%lx", (long)ps_irq_handler);
			SENSOR_ERR_LOG("interrupt flag     = 0x%x", IRQF_TRIGGER_FALLING);
			SENSOR_ERR_LOG("interrupt name     = %s", client->name);
			SENSOR_ERR_LOG("base address       = 0x%lx", (long)ps_als);
			goto err_power_on_failed;
		}
	} else {
		SENSOR_ERR_LOG("client->irq is NULL");
	}
	/* timer process */
	hrtimer_init(&ps_als->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ps_als->timer.function = als_timer_func;

	/* initialize static variable */
	ps_als->delay_time = ALS_DATA_DELAY_MS;
	client_stk3338 = client;

	device_init_wakeup(ps_als->als_info.dev, 1);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	stk3338_initialize = 1;
	device_init_wakeup(ps_als->ps_info.dev, 1);
#endif	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/

	mutex_unlock(&ps_als->control_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->als_data_lock);
	SENSOR_N_LOG("called ps_als_probe for STK3338!!");
#ifdef CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG
	exit = ktime_get();
	func_time_us = ktime_to_us(ktime_sub(exit, enter));
	enter_us = ktime_to_us(enter);
	exit_us = ktime_to_us(exit);
	printk(KERN_NOTICE "[IT_TEST] %s: bgn:%lld end:%lld time:%lld\n",__func__,enter_us,exit_us,func_time_us);
#endif /* CONFIG_USE_KC_SNS_SENSORS_PERFORMANCE_MEASURE_LOG */
	ps_als->host_en = HOST_ENABLE;
	SENSOR_ERR_LOG("called ps_als_probe for stk3338 Normal End!!");
	return (result);
err_power_on_failed:
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
err_gpio_free:
	ps_als_power_src_enable(ps_als, false);
err_drv_init_failed:
	ps_sensor_unregister(&ps_als->ps_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&ps_als->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
err_ps_register_failed:
	als_sensor_unregister(&ps_als->als_info);
err_als_register_failed:
	mutex_unlock(&ps_als->control_lock);
	mutex_unlock(&ps_als->ps_data_lock);
	mutex_unlock(&ps_als->als_data_lock);
	kfree(ps_als);
err_alloc_data_failed:
err_check_functionality_failed:

	return (result);

}

/******************************************************************************
 * NAME       : ps_als_remove
 * FUNCTION   : close system
 * REMARKS    :
 *****************************************************************************/
static int ps_als_remove(struct i2c_client *client)
{
	PS_ALS_DATA *ps_als;

	ps_als = i2c_get_clientdata(client);
	als_enable_proc(ps_als, 0);
	ps_enable_proc(ps_als, 0);
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
	ps_als_power_src_enable(ps_als, false);
	device_init_wakeup(ps_als->als_info.dev, 0);
	ps_sensor_unregister(&ps_als->ps_info);
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
	atomic_set(&ps_als->ps_info.valid, 0);
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
	als_sensor_unregister(&ps_als->als_info);
	kfree(ps_als);

	return 0;
}

/******************************************************************************
 * NAME       : ps_als_shutdown
 * FUNCTION   : shutdown
 * REMARKS    :
 *****************************************************************************/
static void ps_als_shutdown(struct i2c_client *client)
{
	PS_ALS_DATA *ps_als;

	SENSOR_V_LOG("start");
	ps_als = i2c_get_clientdata(client);
	ps_als->is_shtdwn_running = true;
	als_enable_proc(ps_als, 0);
	ps_enable_proc(ps_als, 0);
	ps_als_power_ctrl(ps_als, PS_ALS_POWER_CTRL_OFF);
	SENSOR_V_LOG("end");
}

/******************************************************************************
 * NAME       : ps_als_init
 * FUNCTION   : register driver to kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void)
{
    sensortek_workqueue = create_singlethread_workqueue("sensortek_workqueue");
    if (!sensortek_workqueue) {
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        return (-ENOMEM);
    }

    return (i2c_add_driver(&stk3338_driver));
}
EXPORT_SYMBOL(ps_als_init);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static int __init ps_als_init(void)
{
	SENSOR_ERR_LOG("kick ps_als_init");
    sensortek_workqueue = create_singlethread_workqueue("sensortek_workqueue");
    if (!sensortek_workqueue) {
        SENSOR_ERR_LOG("create_singlethread_workqueue error");
        return (-ENOMEM);
    }

    return (i2c_add_driver(&stk3338_driver));
}
#endif /*CONFIG_USE_MICON_SOFT_STRUCTURE*/

/******************************************************************************
 * NAME       : ps_als_exit
 * FUNCTION   : remove driver from kernel
 * REMARKS    :
 *****************************************************************************/
#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
void ps_als_exit(void)
{
    i2c_del_driver(&stk3338_driver);
    if (sensortek_workqueue) {
        destroy_workqueue(sensortek_workqueue);
    }

    return;
}
EXPORT_SYMBOL(ps_als_exit);
#else /*CONFIG_USE_MICON_SOFT_STRUCTURE*/
static void __exit ps_als_exit(void)
{
    i2c_del_driver(&stk3338_driver);
    if (sensortek_workqueue) {
        destroy_workqueue(sensortek_workqueue);
    }

    return;
}
#endif /*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

/************************************************************
 *                     access function                      *
 ***********************************************************/
#define I2C_WRITE_MSG_NUM	1
#define I2C_READ_MSG_NUM	2
#define I2C_RETRY_MAX		3

static int ps_als_i2c_read(const struct i2c_client *client, uint8_t reg, uint8_t *rbuf, int len)
{
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_READ_MSG_NUM];
	uint8_t buff;
	int retry = I2C_RETRY_MAX + 1;
	int i;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (client == NULL) {
        SENSOR_ERR_LOG("client null");
		return -ENODEV;
	}

	if (ps_als_is_device_dead(i2c_get_clientdata(client))) {
        SENSOR_ERR_LOG("ps_als_is_device_dead error");
		return -1;
	}

	while (retry--) {
		i2cMsg[0].addr = client->addr;
		i2cMsg[0].flags = 0;
		i2cMsg[0].len = 1;
		i2cMsg[0].buf = &buff;
		buff = reg;
		i2cMsg[1].addr = client->addr;
		i2cMsg[1].flags = I2C_M_RD;
		i2cMsg[1].len = len;
		i2cMsg[1].buf = rbuf;

		ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_READ_MSG_NUM);
		SENSOR_V_LOG("i2c_transfer() called. ret[%d]",ret);
		if (ret == I2C_READ_MSG_NUM) {
			SENSOR_V_LOG("end. exec mesg[%d]",(int)ret);
			for (i = 0; i < len; i++) {
				SENSOR_V_LOG("i2c read reg[%02X] value[%02X]",
				(unsigned int)(reg + i), (unsigned int)*(rbuf + i));
			}
			SENSOR_V_LOG("end - return len=%d", len);
			return len;
		} else if((ret == -EACCES) || (ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS)) {
			SENSOR_ERR_LOG("fault i2c_transfer()-->ret[%d] remaining RETRY[%d]",ret ,retry );
			msleep(100);
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );
	return -1;
}

static int ps_als_i2c_write(const struct i2c_client *client, uint8_t reg, const uint8_t *wbuf, int len)
{
#define BUF_SIZE	0x20
	int ret = 0;
	struct i2c_msg i2cMsg[I2C_WRITE_MSG_NUM];
	uint8_t buff[BUF_SIZE];
	int retry = I2C_RETRY_MAX + 1;
	int i;

	SENSOR_V_LOG("start reg[%02X] len[%d]", reg, len );

	if (unlikely(client == NULL)) {
		SENSOR_ERR_LOG("client null");
		return -ENODEV;
	}

	if (ps_als_is_device_dead(i2c_get_clientdata(client))) {
		SENSOR_ERR_LOG("ps_als_is_device_dead error");
		return -1;
	}

	if (unlikely(len > BUF_SIZE - 1)) {
		SENSOR_ERR_LOG("size over len[%d]", len);
		return -ENOMEM;
	}

	while (retry--) {
		buff[0] = reg;
		memcpy(&buff[1], wbuf, len);

		i2cMsg[0].addr = client->addr;
		i2cMsg[0].flags = 0;
		i2cMsg[0].len = len + 1;
		i2cMsg[0].buf = (uint8_t *)buff;

		ret = i2c_transfer(client->adapter, &i2cMsg[0], I2C_WRITE_MSG_NUM);
		SENSOR_V_LOG("i2c_transfer() called. ret[%d]",ret);

		if (ret == I2C_WRITE_MSG_NUM) {
			SENSOR_V_LOG("end. exec mesg[%d]",ret);
			for (i = 0; i < len; i++) {
				SENSOR_V_LOG("i2c write reg[%02X] value[%02X]",
				             (unsigned int)(reg + i), (unsigned int)*(wbuf + i));
			}
			SENSOR_V_LOG("end - return 0");
			return 0;
		} else if((ret == -EACCES) || (ret == -EBUSY) || (ret == -ETIMEDOUT) || (ret == -EINPROGRESS)) {
			SENSOR_ERR_LOG("fault i2c_transfer()-->ret[%d] remaining RETRY[%d]",ret ,retry );
			msleep(100);
		} else {
			SENSOR_ERR_LOG("i2c transfer error[%d] in while.",ret );
			ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RESET_OCCURRED);
		}
	}
	ps_als_set_errinfo(i2c_get_clientdata(client), ERRINFO_I2C_RECOVERY_FAILED);
	SENSOR_ERR_LOG("i2c transfer error[%d]",ret );

	return -1;
#undef BUF_SIZE
}

static int ps_als_i2c_read_byte_data(const struct i2c_client *client,
                                      u8 command)
{
	int ret;
	uint8_t buff = 0;

	ret = ps_als_i2c_read(client, command, &buff, 1);

	return (ret < 0) ? ret : buff;
}

static int ps_als_i2c_write_byte_data(const struct i2c_client *client,
                                      u8 command, u8 value)
{
	int ret;

	ret = ps_als_i2c_write(client, command, &value, 1);

	return ret;
}

static int ps_als_i2c_write_i2c_block_data(const struct i2c_client *client, u8 command,
                                           u8 length, const u8 *values)
{
	int ret;

	ret = ps_als_i2c_write(client, command, values, length);

	return ret;
}

/******************************************************************************
 * NAME       : ps_als_driver_init
 * FUNCTION   : initialize RPR0521
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_init(INIT_ARG *data, struct i2c_client *client)
{
	struct init_func_write_data1 {
	unsigned char  state;
    unsigned char  psctrl;
    unsigned char  alsctrl1;
    unsigned char  vcselctrl;
    unsigned char  intctrl1;
    unsigned char  wait;
	unsigned char  thdh_ps_msb;
	unsigned char  thdh_ps_lsb;
	unsigned char  thdl_ps_msb;
	unsigned char  thdl_ps_lsb;
	unsigned char  thdh_als_msb;
	unsigned char  thdh_als_lsb;
	unsigned char  thdl_als_msb;
	unsigned char  thdl_als_lsb;
	} write_data_block1;

	struct init_func_write_data2 {
	unsigned char  data_ps_offset_msb;
	unsigned char  data_ps_offset_lsb;
	} write_data_block2;

	struct init_func_write_data3 {
	unsigned char  alsctrl2;
	unsigned char  intelli_wait_ps;
	} write_data_block3;

	struct init_func_write_data4 {
	unsigned char  sysctrl1;
	unsigned char  pspdctrl;
	} write_data_block4;

	struct init_func_write_data5 {
	unsigned char  intctrl2;
	} write_data_block5;

	struct init_func_write_data6 {
	unsigned char  thd_psoff_msb;
	unsigned char  thd_psoff_lsb;
	unsigned char  thd_bgir;
	} write_data_block6;
	int result;

	/* not check parameters are thdh_ps, thdl_ps, thdh_upper, thdl_low */
	/* check the PS orerating mode */
	if (0 != (data->state & INIT_MODE_MASK)) {
		SENSOR_ERR_LOG("data->state = 0x%x", data->state);
		return (-EINVAL);
	}

	/* check the parameter of proximity sensor threshold high */
	if (data->thdh_ps > REG_PSTH_MAX) {
		SENSOR_ERR_LOG("data->thdh_ps = 0x%x", data->thdh_ps);
		return (-EINVAL);
	}
	/* check the parameter of proximity sensor threshold low */
	if (data->thdl_ps > REG_PSTL_MAX) {
		SENSOR_ERR_LOG("data->thdl_ps = 0x%x", data->thdl_ps);
		return (-EINVAL);
	}
	write_data_block1.state		  			= data->state;
	write_data_block1.psctrl 				= data->psctrl;
	write_data_block1.alsctrl1   			= data->alsctrl1;
	write_data_block1.vcselctrl 			= 0;
	write_data_block1.intctrl1 				= 0;
	write_data_block1.wait 				= data->wait;
	write_data_block1.thdh_ps_msb			= (data->thdh_ps) >> 8;
	write_data_block1.thdh_ps_lsb			= (data->thdh_ps) & MASK_CHAR;
	write_data_block1.thdl_ps_msb			= (data->thdl_ps) >> 8;
	write_data_block1.thdl_ps_lsb			= (data->thdl_ps) & MASK_CHAR;
	write_data_block1.thdh_als_msb			= (data->thdh_als) >> 8;
	write_data_block1.thdh_als_lsb			= (data->thdh_als) & MASK_CHAR;
	write_data_block1.thdl_als_msb			= (data->thdl_als) >> 8;
	write_data_block1.thdl_als_lsb			= (data->thdl_als) & MASK_CHAR;
	write_data_block2.data_ps_offset_msb	= (data->data_ps_offset) >> 8;
	write_data_block2.data_ps_offset_lsb	= (data->data_ps_offset) & MASK_CHAR;
	write_data_block3.alsctrl2				= (data->alsctrl2);
	write_data_block3.intelli_wait_ps		= (data->intelli_wait_ps);
	write_data_block4.sysctrl1				= (data->sysctrl1);
	write_data_block4.pspdctrl				= (data->pspdctrl);
	write_data_block5.intctrl2				= (data->intctrl2) ;
	write_data_block6.thd_psoff_msb			= (data->thd_psoff) >> 8;
	write_data_block6.thd_psoff_lsb			= (data->thd_psoff) & MASK_CHAR;
	write_data_block6.thd_bgir				= (data->thd_bgir);
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_STATE,
						sizeof(write_data_block1),
						(unsigned char *)&write_data_block1);

	result = ps_als_i2c_write_i2c_block_data(client,
						REG_DATA_PS_OFFSET,
						sizeof(write_data_block2),
						(unsigned char *)&write_data_block2);
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_ALSCTRL2,
						sizeof(write_data_block3),
						(unsigned char *)&write_data_block3);
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_SYSCTRL1,
						sizeof(write_data_block4),
						(unsigned char *)&write_data_block4);
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_INTCTRL2,
						sizeof(write_data_block5),
						(unsigned char *)&write_data_block5);
	result = ps_als_i2c_write_i2c_block_data(client,
						REG_THD_PSOFF,
						sizeof(write_data_block6),
						(unsigned char *)&write_data_block6);
	return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_reset
 * FUNCTION   : reset RPR0521 register
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_reset(struct i2c_client *client)
{
    int result;

    /* set soft ware reset */
    result = ps_als_i2c_write_byte_data(client, REG_SOFT_RESET, (1));

    return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_read_power_state
 * FUNCTION   : read the value of PS and ALS status in STK3338
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_read_power_state(PWR_ST *pwr_st, struct i2c_client *client)
{
	int result;

	/* read control state of ps and als */
	result = ps_als_i2c_read_byte_data(client, REG_STATE);
	if (unlikely(result < 0)) {
		pwr_st->als_state = CTL_STANDBY;
		pwr_st->ps_state  = CTL_STANDBY;
        SENSOR_ERR_LOG("ps_als_i2c_read_byte_data error:%d",result);
	} else {
		/* check power state of als from control state */
		if (result & EN_ALS_ENABLE) {
			pwr_st->als_state = CTL_STANDALONE;
		} else {
			pwr_st->als_state = CTL_STANDBY;
		}

		/* check power state of ps from control state */
		if (result & EN_PS_ENABLE) {
			pwr_st->ps_state = CTL_STANDALONE;
		} else {
			pwr_st->ps_state = CTL_STANDBY;
		}
	}

	return (result);
}

/******************************************************************************
 * NAME       : ps_als_driver_general_read
 * FUNCTION   : read general multi bytes
 * REMARKS    :
 *****************************************************************************/
static int ps_als_driver_general_read(GENREAD_ARG data, struct i2c_client *client)
{
	int            result;

    SENSOR_D_LOG("start adr_reg[0x%02x] size[0x%02x]", data.adr_reg, data.size);
	if (data.size == 0) {
        SENSOR_ERR_LOG("data.size 0 error");
		return (-EINVAL);
	}
	/* check the parameter of register */
	if ((data.adr_reg < REG_STATE) || (data.adr_reg > REG_THD_BGIR )) {
        SENSOR_ERR_LOG("data.adr_reg :0x%x",data.adr_reg);
		return (-EINVAL);
	}

	result = ps_als_i2c_read(client, data.adr_reg, data.addr, data.size);
	if (result < 0) {
		SENSOR_ERR_LOG("transfer error");
	}

    SENSOR_D_LOG("end");
	return (result);
}

int32_t als_sensor_activate(bool enable)
{
	PS_ALS_DATA *ps_als;
	int result = 0;

	ps_als = i2c_get_clientdata(client_stk3338);

	if (unlikely(enable > PS_ALS_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. als_en_cnt = %d, enable = %d",
					ps_als->als_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->als_en_cnt <= 0) {
			ps_als->als_en_cnt = 1;
		} else {
			ps_als->als_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->als_en_cnt--;
		if (ps_als->als_en_cnt < 0) {
			ps_als->als_en_cnt = 0;
			goto exit;
		} else if (ps_als->als_en_cnt > 0){
			goto exit;
		}
	}

	result = als_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable light sensor[%d]", result);
		if (enable)
			ps_als->als_en_cnt--;
		else
			ps_als->als_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->als_en_cnt = %d", ps_als->als_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}

int32_t ps_sensor_activate(bool enable)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);
	int result = 0;

	if (unlikely(enable > PS_ALS_ENABLE)) {
		SENSOR_D_LOG("INVALID enable val=%d",enable);
		return -EINVAL;
	}

	SENSOR_D_LOG("start. ps_en_cnt = %d, enable = %d",
					ps_als->ps_en_cnt, enable);
	mutex_lock(&ps_als->control_lock);
	if (enable) {
		if (ps_als->ps_en_cnt <= 0) {
			ps_als->ps_en_cnt = 1;
		} else {
			ps_als->ps_en_cnt++;
			goto exit;
		}
	} else {
		ps_als->ps_en_cnt--;
		if (ps_als->ps_en_cnt < 0) {
			ps_als->ps_en_cnt = 0;
			goto exit;
		} else if (ps_als->ps_en_cnt > 0){
			goto exit;
		}
	}

	result = ps_enable_proc(ps_als, enable);
	if (result) {
		SENSOR_ERR_LOG("Failed to enable proximity sensor[%d]", result);
		if (enable)
			ps_als->ps_en_cnt--;
		else
			ps_als->ps_en_cnt++;
	}
exit:
	SENSOR_D_LOG("end. ps_als->ps_en_cnt = %d", ps_als->ps_en_cnt);
	mutex_unlock(&ps_als->control_lock);
	return result;
}

uint32_t ps_sensor_get_count(void)
{
	uint32_t ps_count = 0;
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);

	ps_driver_read_data(client_stk3338);
	ps_count = (uint32_t)ps_als->ps_val.data_ps;

	return ps_count;
}


#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
bool als_sensor_get_en_first(void)
{
	PS_ALS_DATA *ps_als = i2c_get_clientdata(client_stk3338);
	SENSOR_D_LOG("start");
	SENSOR_D_LOG("end");
	return ps_als->als_en_first;
}

int32_t als_get_initialize_state(void){
	return stk3338_initialize;
}
int32_t ps_get_initialize_state(void){
	return stk3338_initialize;
}
#else	/*CONFIG_USE_MICON_SOFT_STRUCTURE*/
module_init(ps_als_init);
module_exit(ps_als_exit);
#endif	/*!CONFIG_USE_MICON_SOFT_STRUCTURE*/

MODULE_DESCRIPTION("SENSORTEK Proximity Sensor & Ambient Light Sensor Driver");
MODULE_LICENSE("GPL");
