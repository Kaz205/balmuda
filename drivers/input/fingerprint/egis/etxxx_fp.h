#ifndef _FP_LINUX_DIRVER_H_
#define _FP_LINUX_DIRVER_H_
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/irq.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

/*#define FP_SPI_DEBUG*/
//#define FP_SPI_DEBUG
#ifdef FP_SPI_DEBUG
#define DEBUG_PRINT(fmt, args...) pr_err(fmt, ## args)
//#define DEBUG_PRINT printk

#else
#define DEBUG_PRINT(fmt, args...)
#endif

//#define ET5XX
#define EGIS_FP_MAJOR				100 /* assigned */
#define N_SPI_MINORS				32  /* ... up to 256 */

#define EGIS_DEV_NAME            "esfp0"
#define EGIS_INPUT_NAME       "egis_fp"    /*"egis_fp" */

#define EGIS_CHRD_DRIVER_NAME    "egis_fp"
#define EGIS_CLASS_NAME          "egis_fp"
#define EGIS_MODEL_NAME          "ET520"

//#define PLATFORM_SPI
#define FP_DRM

/* ------------------------- Opcode -------------------------------------*/
#define FP_REGISTER_READ			0x01
#define FP_REGISTER_WRITE			0x02
#define FP_GET_ONE_IMG				0x03
#define FP_SENSOR_RESET				0x04
#define FP_POWER_ONOFF				0x05
#define FP_SET_SPI_CLOCK			0x06
#define FP_RESET_SET				0x07
#define FP_WAKELOCK_ENABLE			0x08
#define FP_WAKELOCK_DISABLE			0x09
#define GET_IO_STUS					0x21
#define GET_DROP_DETECT_STUS		0x22
#define CLERA_DROP_DETECT_STUS		0x23
#define GET_NAVI_EVENT 				0x31
/* trigger signal initial routine*/
#define INT_TRIGGER_INIT			0xa4
/* trigger signal close routine*/
#define INT_TRIGGER_CLOSE		0xa5
/* read trigger status*/
#define INT_TRIGGER_READ			0xa6
/* polling trigger status*/
#define INT_TRIGGER_POLLING		0xa7
/* polling abort*/
#define INT_TRIGGER_ABORT		0xa8

#define FP_FREE_GPIO			        0xaf
#define FP_SPICLK_ENABLE			0xaa
#define FP_SPICLK_DISABLE			0xab
#define DELETE_DEVICE_NODE		0xac

#define FP_SPIPIN_SETTING			0xad
#define FP_SPIPIN_PULLLOW		0xae

#define FP_POWERSETUP 				0xb0
#define FP_WAKELOCK_TIMEOUT_ENABLE 	0xb1
#define FP_WAKELOCK_TIMEOUT_DISABLE 0xb2
#define GET_SCREEN_ONOFF 			0xb3


#define DRDY_IRQ_ENABLE				1
#define DRDY_IRQ_DISABLE			0

#define SPI_DEFAULT_SPEED      (1000000 * 20) //20M
#define ET713_Tpwr_off_delay         50
#define ET713_Tpwr_on_delay          5

#define ET6XX_Tpwr_off_delay         3
#define ET6XX_Tpwr_on_delay          3
#define ET6XX_Rst_on_delay          3

#define ET5XX_Tpwr_off_delay         3
#define ET5XX_Tpwr_on_delay          3
#define ET5XX_Rst_off_delay          3
#define ET5XX_Rst_on_delay           12

/* +++ add for Kyocera power drop detect +++ */
#define DROP_IRQ_LOW_SLEEP_MS     10
#define DROP_IRQ_HIGH_SLEEP_MS    50
#define DROP_IRQ_TIMEOUT_MS     2000
/* --- add for Kyocera power drop detect --- */

#define EGIS_NAV_INPUT_UP				KEY_UP
#define EGIS_NAV_INPUT_DOWN				KEY_DOWN
#define EGIS_NAV_INPUT_LEFT				KEY_LEFT
#define EGIS_NAV_INPUT_RIGHT			KEY_RIGHT
#define EGIS_NAV_INPUT_CLICK			KEY_PAGEUP
#define EGIS_NAV_INPUT_DOUBLE_CLICK		KEY_PAGEDOWN
#define EGIS_NAV_INPUT_LONG_PRESS		KEY_HOME
#define EGIS_NAV_INPUT_FINGER_DOWN		KEY_CAMERA
#define EGIS_NAV_INPUT_FINGER_UP		KEY_CAMERA

enum egis_navi_event {
	NAVI_EVENT_ON,
	NAVI_EVENT_OFF,
	NAVI_EVENT_UP,
	NAVI_EVENT_DOWN,
	NAVI_EVENT_LEFT,
	NAVI_EVENT_RIGHT,
	NAVI_EVENT_CLICK,
	NAVI_EVENT_DOUBLE_CLICK,
	NAVI_EVENT_LONG_PRESS,
};

/* interrupt polling */
unsigned int fps_interrupt_poll(
	struct file *file,
	struct poll_table_struct *wait
);
struct interrupt_desc {
	int gpio;
	int number;
	char *name;
	int int_count;
	struct timer_list timer;
	bool finger_on;
	int detect_period;
	int detect_threshold;
	bool drdy_irq_flag;
	bool drdy_irq_abort;
};

struct egis_key_map {
	unsigned int type;
	unsigned int code;
};

/* ------------------------- Structure ------------------------------*/

struct egistec_data {
	dev_t devt;
	spinlock_t spi_lock;
#if defined(MTK_PLATFORM) || defined(PLATFORM_SPI)
	struct spi_device  *dd;	
#else
	struct platform_device *dd;
#endif
#ifdef PLATFORM_SPI
	u8 *tx_buffer;
	u8 *rx_buffer;
#endif
	struct list_head device_entry;
	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex buf_lock;
	unsigned users;
	u8 *buffer;
	unsigned int irqPin;	    /* interrupt GPIO pin number */
	unsigned int rstPin; 	    /* Reset GPIO pin number */
	unsigned int vdd_18v_Pin;	/* Vdd GPIO pin number */
	unsigned int vcc_33v_Pin;	/* Vcc GPIO pin number */
    struct input_dev	*input_dev;
	bool property_navigation_enable;
	bool egistec_platformInit_done;
	int gpio_irq;
	bool request_irq_done;
	spinlock_t irq_lock;
	struct wakeup_source wakeup_source_fp;
	bool pwr_by_gpio;
	bool ext_vdd_source;
	bool power_enable;
#if defined(FP_DRM)
	bool finger_down_present;
	int screen_onoff;
#endif
	struct regulator *vcc;
	struct regulator *vdd;
	uint32_t  regulator_max_voltage;
	uint32_t  regulator_min_voltage;
	uint32_t  regulator_current;
	uint32_t  max_voltage;
	uint32_t  regulator_vdd_max_voltage;
	uint32_t  regulator_vdd_min_voltage;
	struct timer_list navi_timer;
#ifdef SAMSUNG_PLATFORM
	bool clk_enabled;
	struct clk *core_clk;
	struct clk *iface_clk;
#endif
	int clk_speed;
#ifdef CONFIG_OF
	struct pinctrl *pinctrl;
	struct pinctrl_state *reset_high;
	struct pinctrl_state *reset_low;
	struct pinctrl_state *vcc_high;
	struct pinctrl_state *vcc_low;
	struct pinctrl_state *irq_active;
	struct pinctrl_state *drop_irq_active;
	struct pinctrl_state *spi_active;
	struct pinctrl_state *spi_default;
#endif	
	unsigned int drop_irq_gpio;	/* kc power drop interrupt GPIO pin number */
	bool drop_det_available;
	bool drop_det_en;
	bool drop_detected;
	bool drop_flag;
	struct work_struct drop_work;
#if defined(FP_DRM)
	struct notifier_block fb_notif;
	unsigned int fb_state;
#endif
};


/* ------------------------- Interrupt ------------------------------*/
/* interrupt init */
int Interrupt_Init(
	struct egistec_data *egistec,
	int int_mode,
	int detect_period,
	int detect_threshold);
/* interrupt free */
int Interrupt_Free(struct egistec_data *egistec);
int egis_power_onoff(struct egistec_data *egis, int power_onoff);
void fps_interrupt_abort(void);
#endif
