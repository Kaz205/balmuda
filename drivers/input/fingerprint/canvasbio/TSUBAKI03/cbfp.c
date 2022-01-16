/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
/* Canvasbio BTP Sensor Driver
 *
 * Copyright (c) 2017 Canvasbio
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

//#define SUPPORT_PINCTRL
#define SUPPORT_DEIVE_ID

#if defined(CONFIG_FB)
#define SUPPORT_LCD_CONTROL
#endif /* CONFIG_FB */

#define SUPPORT_VOLTAGE_DETECTOR

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/wakelock.h>

#if defined(SUPPORT_LCD_CONTROL)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif /* SUPPORT_LCD_CONTROL */


#define CB_KERNEL_VERSION           "1.1.1"

/**************************debug******************************/
#define ERR_LOG     (0)
#define INFO_LOG    (1)
#define DEBUG_LOG   (2)

/* debug log setting */
u8 debug_level = DEBUG_LOG;

#define CB_LOG(level, fmt, args...) do { \
    if (debug_level >= level) { \
         printk("[CBFP] " fmt, ##args); \
    } \
} while (0)

/* -------------------------------------------------------------------- */
/* driver constants                                                     */
/* -------------------------------------------------------------------- */
#define CBFP_DEV_NAME           "btp"

#define CB_WAKE_LOCK_HOLD_TIME           1000


/* Constants for THERMAL_CONTROL and LCD_CONTROL */
#define CBFP_IRQ_FP_OFF         0  /* Reading value from GPIO */
#define CBPF_IRQ_FP_ON          1  /* Reading value from GPIO */
#define CBFP_IRQ_FP_STANDBY     2
#define CBFP_IRQ_FP_DISABLED    3
#define CBFP_IRQ_FP_PWR_DROP    4 /* Voltage drop occured */

/* -------------------------------------------------------------------- */
/* sysfs permission                                                     */
/* -------------------------------------------------------------------- */
#define HW_CONTROL_MODE_RW   (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH )
#define HW_CONTROL_MODE_R    (S_IRUSR | S_IRGRP | S_IROTH)

/* -------------------------------------------------------------------- */
/* Board constants                                                      */
/* -------------------------------------------------------------------- */
#define BOARD_REV_V0                0xFF00
#define BOARD_REV_V1                0xFF01


#if defined(SUPPORT_PINCTRL)
static const char * pinctrl_names[] = {
    "cbfp_spi_sleep",
    "cbfp_spi_active",
    "cbfp_spi_cs_sleep",
    "cbfp_reset_reset",
    "cbfp_reset_active",
    "cbfp_irq_active",
};
#endif /* SUPPORT_PINCTRL */

#if defined(SUPPORT_LCD_CONTROL)
/* -------------------------------------------------------------------- */
/* LCD state constants                                                  */
/* -------------------------------------------------------------------- */
typedef enum {
    LCD_STATE_OFF = 0,
    LCD_STATE_ON,
    LCD_STATE_FP_STANDBY,
} lcd_state_t;
#endif /* SUPPORT_LCD_CONTROL */

/* -------------------------------------------------------------------- */
/* POWER state constants                                                  */
/* -------------------------------------------------------------------- */
typedef enum {
    FP_POWER_STATE_OFF = 0,
    FP_POWER_STATE_ON,
    FP_POWER_STATE_DROP,
} power_state_t;


#define CBFP_CLASS_NAME             "btp"
#define CBFP_MAJOR                  0xFF    // 255
#define N_SPI_MINORS                32      /* ... up to 256 */
static DECLARE_BITMAP(minors, N_SPI_MINORS);

/* define ioctl commands */
#define CBFP_IOC_MAGIC              0xF0

#define CBFP_IOC_INIT_DEVICE        _IO(CBFP_IOC_MAGIC, 0)
#define CBFP_IOC_ENABLE_POWER       _IO(CBFP_IOC_MAGIC, 1)
#define CBFP_IOC_DISABLE_POWER      _IO(CBFP_IOC_MAGIC, 2)
#define CBFP_IOC_ENABLE_IRQ         _IO(CBFP_IOC_MAGIC, 3)
#define CBFP_IOC_DISABLE_IRQ        _IO(CBFP_IOC_MAGIC, 4)
#define CBFP_IOC_REMOVE_DEVICE      _IO(CBFP_IOC_MAGIC, 5)
#define CBFP_IOC_RESET              _IO(CBFP_IOC_MAGIC, 6)
#define CBFP_IOC_ON_SEQUENCE        _IO(CBFP_IOC_MAGIC, 7)
#define CBFP_IOC_OFF_SEQUENCE       _IO(CBFP_IOC_MAGIC, 8)
#define CBFP_IOC_RST_ASSERT         _IO(CBFP_IOC_MAGIC, 9)
#define CBFP_IOC_RST_DEASSERT       _IO(CBFP_IOC_MAGIC, 10)
#define CBFP_IOC_REG_TASK           _IO(CBFP_IOC_MAGIC, 11)

#define SIGPWRDRP                   45


/* -------------------------------------------------------------------- */
/* data types                                                           */
/* -------------------------------------------------------------------- */
struct cb_data {
    struct platform_device *pdev;
    struct device *device;
    struct semaphore mutex;

    /* GPIOs */
    int fp_rst_gpio;
    int fp_int_gpio;
    int fp_vdd_gpio;
    int fp_vddio_gpio;

    u32 irq;
    int interrupt_done;

    struct wake_lock fp_wake_lock;

    struct class    *class;
    struct cdev     cdev;
    dev_t           devno;

    u16 board_rev;
    power_state_t   pwr_state;

#if defined(SUPPORT_PINCTRL)
    // Pin control
    struct pinctrl * pinctrl;
    struct pinctrl_state * pinctrl_state[ARRAY_SIZE(pinctrl_names)];
#endif /* SUPPORT_PINCTRL */

#if defined(SUPPORT_LCD_CONTROL)
    lcd_state_t lcd_state;
    struct notifier_block fb_notifier;
#endif /* SUPPORT_LCD_CONTROL */

#if defined(SUPPORT_VOLTAGE_DETECTOR)
    int vdet_vdd_gpio;
    int vdet_vddio_gpio;
    int vdet_vdd_irq;
    int vdet_vddio_irq;
    int irq_drop;
#endif /* SUPPORT_VOLTAGE_DETECTOR */

    struct task_struct *task;
};
typedef struct cb_data cbfp_data_t;

ssize_t (* cb_navi_show_event)(struct device *, struct device_attribute *, char *) = NULL;
ssize_t (* cb_navi_store_event)(struct device *, struct device_attribute *, char *, size_t) = NULL;

/* -------------------------------------------------------------------- */
/* function prototypes                          */
/* -------------------------------------------------------------------- */
static int cbfp_drv_init(void);
static void cbfp_drv_exit(void);
static int cbfp_drv_probe(struct platform_device *pdev);
static int cbfp_drv_remove(struct platform_device *pdev);

static ssize_t cb_show_attr_reset(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cb_store_attr_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t cb_show_attr_irq(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t cb_show_attr_navi_event(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cb_store_attr_navi_event(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#if defined (SUPPORT_LCD_CONTROL)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
#endif /* SUPPORT_LCD_CONTROL */

#if defined(SUPPORT_DEIVE_ID)
static int device_id_confirm(struct platform_device *pdev);
#endif/* SUPPORT_DEIVE_ID */

static irqreturn_t cb_interrupt_handler(int irq, void * data);

static int cb_manage_sysfs(struct cb_data *cbfp, bool create);

int cb_reset(struct cb_data *cbfp);


#if defined(SUPPORT_PINCTRL)
static int select_pin_ctrl(struct cb_data *cbfp, const char * pin_name);
#endif /* SUPPORT_PINCTRL */

static ssize_t btp_read(struct file *file, char __user *buf, size_t count, loff_t *f_ops);
static ssize_t btp_write(struct file *file, const char __user *buf, size_t count, loff_t *f_ops);
static long btp_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static long btp_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
static int btp_open(struct inode *node, struct file *file);
static int btp_release(struct inode *node, struct file *file);
static unsigned int btp_poll(struct file *file, struct poll_table_struct *wait);

static int btp_create_class(struct cb_data *cbfp);
static int btp_create_device(struct cb_data *cbfp);

static void cbfp_hw_power_enable(cbfp_data_t *cbfp, u8 onoff);
static int cbfp_get_gpio_dts_info(cbfp_data_t *cbfp);
static void cbfp_gpio_free(cbfp_data_t *cbfp);
static void cbfp_cfg_irq_gpio(cbfp_data_t *cbfp);
static void cbfp_reset_gpio_control(cbfp_data_t *cbfp, u8 assert);
#if defined(SUPPORT_PINCTRL)
static void cbfp_cfg_pinctrl(cbfp_data_t *cbfp);
#endif /* SUPPORT_PINCTRL */

#if defined(SUPPORT_VOLTAGE_DETECTOR)
static void cbfp_cfg_vdet_irq_gpio(cbfp_data_t *cbfp, bool onoff);
static ssize_t cb_show_attr_irq_drop(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cb_reset_attr_irq_drop(struct device *dev,struct device_attribute *attr,const char *buf,size_t count);

#endif /* SUPPORT_VOLTAGE_DETECTOR */

/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */

static const struct file_operations btp_fops = {
    .owner              = THIS_MODULE,
    .write              = btp_write,
    .read               = btp_read,
    .unlocked_ioctl     = btp_ioctl,
    .compat_ioctl       = btp_compat_ioctl,
    .open               = btp_open,
    .release            = btp_release,
    .poll               = btp_poll,
};

/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
static DEVICE_ATTR(reset,  HW_CONTROL_MODE_RW, cb_show_attr_reset, cb_store_attr_reset);
static DEVICE_ATTR(irq,   HW_CONTROL_MODE_R, cb_show_attr_irq, NULL);
static DEVICE_ATTR(irq_drop,   HW_CONTROL_MODE_RW, cb_show_attr_irq_drop, cb_reset_attr_irq_drop);
static DEVICE_ATTR(navi_event,  HW_CONTROL_MODE_RW, cb_show_attr_navi_event, cb_store_attr_navi_event);

static struct attribute *cb_hw_control_attrs[] = {
    &dev_attr_reset.attr,
    &dev_attr_irq.attr,
#if defined(SUPPORT_VOLTAGE_DETECTOR)
    &dev_attr_irq_drop.attr,
#endif
    &dev_attr_navi_event.attr,
    NULL
};

static const struct attribute_group cb_hw_control_attr_group = {
    .attrs = cb_hw_control_attrs,
};

/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static ssize_t cb_show_attr_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
    cbfp_data_t *cbfp = dev_get_drvdata(dev);

    if (cbfp->pwr_state == FP_POWER_STATE_OFF) {
        return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_DISABLED);
        //return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_PWR_DROP);
    }

#if defined(SUPPORT_LCD_CONTROL)
    if (cbfp->lcd_state == LCD_STATE_ON) {
        cbfp->lcd_state = LCD_STATE_FP_STANDBY;
        return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_STANDBY);
    } else if (cbfp->lcd_state == LCD_STATE_OFF) {
        return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_DISABLED);
    }
#endif /* SUPPORT_LCD_CONTROL */

    if (cbfp->pwr_state == FP_POWER_STATE_OFF) {
        return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_DISABLED);
    }

    // Finger is on sensor or not.
    return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(cbfp->fp_int_gpio));
}
#if defined(SUPPORT_VOLTAGE_DETECTOR)
static ssize_t cb_show_attr_irq_drop(struct device *dev, struct device_attribute *attr, char *buf)
{
    cbfp_data_t *cbfp = dev_get_drvdata(dev);

    if (cbfp->pwr_state == FP_POWER_STATE_OFF) {
        CB_LOG(ERR_LOG, "%s: CBFP_IRQ_FP_DISABLED\n", __func__);
        return scnprintf(buf, PAGE_SIZE, "%d\n", CBFP_IRQ_FP_DISABLED);
    }

    // Finger is on sensor or not.
    CB_LOG(ERR_LOG, "%s: irq_drop = 0x%02x\n", __func__,cbfp->irq_drop);
    return scnprintf(buf, PAGE_SIZE, "0x%02x\n", cbfp->irq_drop);
}
static ssize_t cb_reset_attr_irq_drop(struct device *dev,struct device_attribute *attr,const char *buf,size_t count)
{
    cbfp_data_t *cbfp = dev_get_drvdata(dev);
    cbfp->irq_drop =0x00;

    CB_LOG(ERR_LOG, "%s: cb_reset_attr_irq_drop\n", __func__);
    return count;
}
#endif

/* -------------------------------------------------------------------- */
static ssize_t cb_show_attr_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct cb_data *cbfp = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(cbfp->fp_rst_gpio));
}

/* -------------------------------------------------------------------- */
static ssize_t cb_store_attr_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int ret;
    char cmd_buf[16];
    struct cb_data *cbfp = dev_get_drvdata(dev);

    ret = sscanf(buf, "%s", cmd_buf);
    if (ret != 1)
        return -EINVAL;

    if (!strcasecmp(cmd_buf, "high")) {
        gpio_set_value(cbfp->fp_rst_gpio, 1);
    } else if (!strcasecmp(cmd_buf, "low")) {
        gpio_set_value(cbfp->fp_rst_gpio, 0);
    } else if (!strcasecmp(cmd_buf, "reset")) {
        cb_reset(cbfp);
    } else {
        CB_LOG(ERR_LOG, "%s: reset command is wrong(%s)\n", __func__, cmd_buf);
        return -EINVAL;
    }

    return strnlen(buf, count);
}

static ssize_t cb_show_attr_navi_event(struct device *dev, struct device_attribute *attr, char *buf)
{
    if(cb_navi_show_event != NULL)
        return cb_navi_show_event(dev, attr, buf);

    return 0;
}

static ssize_t cb_store_attr_navi_event(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    if(cb_navi_store_event != NULL)
        return cb_navi_store_event(dev, attr, (char *)buf, count);

    return 0;
}

#if defined (SUPPORT_LCD_CONTROL)
/* -------------------------------------------------------------------- */
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    lcd_state_t last_lcd_state;

    cbfp_data_t *cbfp = container_of(self, struct cb_data, fb_notifier);
    if (!cbfp)
        return 0;

    if ((event != FB_EVENT_BLANK && event != FB_EARLY_EVENT_BLANK && event != FB_R_EARLY_EVENT_BLANK) || !evdata)
        return 0;
    last_lcd_state = cbfp->lcd_state;

    blank = evdata->data;
    if (*blank <= FB_BLANK_NORMAL) {
        //if (event == FB_EVENT_BLANK) {
        //} else if (event == FB_EARLY_EVENT_BLANK || event == FB_R_EARLY_EVENT_BLANK) {
        //}
        if (cbfp->lcd_state == LCD_STATE_OFF) {
            CB_LOG(DEBUG_LOG, "%s: LCD is ON!!!(%d)\n", __func__, *blank);
            cbfp->lcd_state = LCD_STATE_ON;
        }
    } else if (*blank == FB_BLANK_POWERDOWN) {
        //if (event == FB_EVENT_BLANK) {
        //} else if (event == FB_EARLY_EVENT_BLANK || event == FB_R_EARLY_EVENT_BLANK) {
        //}
        if (cbfp->lcd_state != LCD_STATE_OFF) {
            CB_LOG(DEBUG_LOG, "%s: LCD is OFF!!!(%d)\n", __func__, *blank);
            cbfp->lcd_state = LCD_STATE_OFF;
        }
    }

    if (cbfp->lcd_state != last_lcd_state) {
        CB_LOG(DEBUG_LOG, "%s: lcd state %d -> %d\n", __func__, last_lcd_state, cbfp->lcd_state);
        sysfs_notify(&cbfp->pdev->dev.kobj, NULL, dev_attr_irq.attr.name);
    }
    return 0;
}
#endif /* SUPPORT_LCD_CONTROL */

// 1. set handler to thread_fn
/* -------------------------------------------------------------------- */
static irqreturn_t cb_interrupt_handler(int irq, void * data)
{
    struct cb_data *cbfp = NULL;

    //CB_LOG(INFO_LOG, "%s\n", __func__);

    if (!data)
        return IRQ_NONE;

    cbfp = (struct cb_data *) data;
    if (!cbfp)
        return IRQ_NONE;

    smp_rmb();
    wake_lock_timeout(&cbfp->fp_wake_lock, msecs_to_jiffies(CB_WAKE_LOCK_HOLD_TIME));

    cbfp->interrupt_done = 1;

    CB_LOG(INFO_LOG, "%s: gpio value = %d\n", __func__, gpio_get_value(cbfp->fp_int_gpio));
    sysfs_notify(&cbfp->pdev->dev.kobj, NULL, dev_attr_irq.attr.name);

    return IRQ_HANDLED;
}

/* -------------------------------------------------------------------- */
int cb_reset(struct cb_data *cbfp)
{
    u8 value = 0;

    CB_LOG(INFO_LOG, "%s\n", __func__);

    if (!gpio_is_valid(cbfp->fp_rst_gpio)) {
        CB_LOG(ERR_LOG, "%s: RESET GPIO(%d) is INVALID\n", __func__, cbfp->fp_rst_gpio);
        return 0;
    }

    value = gpio_get_value(cbfp->fp_rst_gpio);
    CB_LOG(DEBUG_LOG, "%s: RESET GPIO(%d) state is %s\n", __func__, cbfp->fp_rst_gpio, (value == 0) ? "low" : "high");
    if (value != 0) {
        gpio_set_value(cbfp->fp_rst_gpio, 0);
        mdelay(1);
    }

    gpio_set_value(cbfp->fp_rst_gpio, 1);
    mdelay(1);

    value = gpio_get_value(cbfp->fp_rst_gpio);
    CB_LOG(DEBUG_LOG, "%s: FIANL RESET GPIO(%d) state is %s\n", __func__, cbfp->fp_rst_gpio, (value == 0) ? "low" : "high");

    //disable_irq_nosync(cbfp->irq);
    disable_irq(cbfp->irq);
    cbfp->interrupt_done = 0;

    mdelay(1);
    enable_irq(cbfp->irq);

    return 0;
}

/* -------------------------------------------------------------------- */
static int cb_request_named_gpio(struct cb_data * cbfp, const char *label, int *gpio)
{
    struct device *dev = &cbfp->pdev->dev;
    struct device_node *np = dev->of_node;

    int rc = 0;
    int pin = 0;

    //CB_LOG(INFO_LOG, "%s\n", __func__);
    *gpio = -EINVAL;

    rc = of_get_named_gpio(np, label, 0);
    if (rc < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to get gpio(%s)\n", __func__, label);
        return rc;
    }
    pin = rc;
    CB_LOG(DEBUG_LOG, "%s: Succeed to get gpio [%s(%d)], Trying to request gpio\n", __func__, label, rc);

    rc = devm_gpio_request(dev, pin, label);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to request gpio(%s-%d)\n", __func__, label, *gpio);
        return rc;
    }
    *gpio = pin;

    return 0;
}

#if 0
/* -------------------------------------------------------------------- */
static void cb_hw_reset(struct cb_data *cbfp, int delay)
{
    gpio_set_value(cbfp->fp_rst_gpio, 1);
    mdelay(1);

    gpio_set_value(cbfp->fp_rst_gpio, 0);
    //gpio_set_value(cbfp->fp_rst_gpio, 1);
    mdelay(1);

    gpio_set_value(cbfp->fp_rst_gpio, 1);

    if (delay) {
        udelay(delay);
    }

    disable_irq(cbfp->irq);
    cbfp->interrupt_done = 0;
    enable_irq(cbfp->irq);

    return;
}
#endif

/* -------------------------------------------------------------------- */
static int cb_manage_sysfs(struct cb_data *cbfp, bool create)
{
    struct platform_device *pdev = cbfp->pdev;
    int rc = 0;

    if (create) {
        rc = sysfs_create_group(&pdev->dev.kobj, &cb_hw_control_attr_group);
        if (rc) {
            CB_LOG(ERR_LOG, "%s: sysfs_create_group failed. %d\n", __func__, rc);
        }
    } else {
        sysfs_remove_group(&pdev->dev.kobj, &cb_hw_control_attr_group);
    }

#if defined(SUPPORT_PINCTRL)
    //select_pin_ctrl(NULL, "cbfp_irq_active");
#endif /* SUPPORT_PINCTRL */
    return rc;
}


#if defined(SUPPORT_PINCTRL)
static int select_pin_ctrl(struct cb_data *cbfp, const char * pin_name)
{
    int rc = 0;
    int i = 0;

    struct device *dev = NULL;

    if (!cbfp) {
        CB_LOG(ERR_LOG, "%s: Invalid parameter\n", __func__);
        return -EINVAL;
    }
    dev = &cbfp->pdev->dev;

    if (!dev) {
        CB_LOG(ERR_LOG, "%s: Device is NULL\n", __func__);
        return -EINVAL;
    }

    if (IS_ERR(cbfp->pinctrl)) {
        CB_LOG(ERR_LOG, "%s: Cannot use pinctrl\n", __func__);
        return -EINVAL;
    }

    for (i=0 ; i<ARRAY_SIZE(cbfp->pinctrl_state) ; i++) {
        const char * n = pinctrl_names[i];
        if (!strncmp(n, pin_name, strlen(n))) {
            //rc = pinctrl_select_state(cbfp->pinctrl, cbfp->pinctrl_state[i]);
            if (rc) {
                CB_LOG(ERR_LOG, "%s: Cannot select pin [%s]\n", __func__, pin_name);
            } else {
                CB_LOG(DEBUG_LOG, "%s: Selected pin [%s]\n", __func__, pin_name);
            }
            goto exit;
        }
    } 
    CB_LOG(ERR_LOG, "%s: Cannot found pin [%s]\n", __func__, pin_name);
    rc = -EINVAL;
exit:
    return rc;
}
#endif /* SUPPORT_PINCTRL */

static ssize_t btp_read(struct file *file, char __user *buf, size_t count, loff_t *f_ops)
{
    CB_LOG(ERR_LOG, "%s: Not supported operation in TEE version\n", __func__);
    return -EFAULT;
}

static ssize_t btp_write(struct file *file, const char __user *buf, size_t count, loff_t *f_ops)
{
    CB_LOG(ERR_LOG, "%s: Not supported operation in TEE version\n", __func__);
    return -EFAULT;
}

static long btp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int rc = 0;
    cbfp_data_t *cbfp = NULL;

#if defined(SUPPORT_VOLTAGE_DETECTOR)
    /* For boot check */
    int fp_therm_n_1 = 1;
    int fp_therm_n_2 = 1;
#endif /* SUPPORT_VOLTAGE_DETECTOR */
    u32 delay;
    CB_LOG(INFO_LOG, "%s: ioctl cmd : 0x%0x\n", __func__, cmd);
    cbfp = (cbfp_data_t *)file->private_data;
    switch (cmd) {
        case CBFP_IOC_INIT_DEVICE:
            CB_LOG(DEBUG_LOG, "%s: Initialize device\n", __func__);
            //rc = cbfp_get_gpio_dts_info(cbfp);
            //if (rc) return -1;
            //cbfp_cfg_irq_gpio(cbfp);
            //cb_manage_sysfs(cbfp, true);
#if defined(SUPPORT_PINCTRL)
            cbfp_cfg_pinctrl(cbfp);
#endif /* SUPPORT_PINCTRL */
            CB_LOG(DEBUG_LOG, "%s: Initialize done\n", __func__);
            break;
        case CBFP_IOC_ENABLE_POWER:
            cbfp_hw_power_enable(cbfp, 1);
            delay = 10; // 10 ms
#if defined(SUPPORT_VOLTAGE_DETECTOR)
            do {
                fp_therm_n_1 = gpio_get_value(cbfp->vdet_vdd_gpio);
                fp_therm_n_2 = gpio_get_value(cbfp->vdet_vddio_gpio);
                //CB_LOG(DEBUG_LOG, "%s: BOOTCHECK_1(%d  %d)\n", __func__, fp_therm_n_1, fp_therm_n_2);
                if (!fp_therm_n_1 && !fp_therm_n_2) break;
                mdelay(1);
            } while(--delay);
            if (fp_therm_n_1 || fp_therm_n_2) {
                CB_LOG(DEBUG_LOG, "%s: Failed to check boot(1)(%d  %d)\n", __func__, fp_therm_n_1, fp_therm_n_2);
                return -1;
            }
            delay = 2000; // 2secs
            do {
                fp_therm_n_1 = gpio_get_value(cbfp->vdet_vdd_gpio);
                fp_therm_n_2 = gpio_get_value(cbfp->vdet_vddio_gpio);
                //CB_LOG(DEBUG_LOG, "%s: BOOTCHECK_2(%d  %d)\n", __func__, fp_therm_n_1, fp_therm_n_2);
                if (fp_therm_n_1 && fp_therm_n_2) break;
                mdelay(1);
            } while(--delay);
            if (!fp_therm_n_1 || !fp_therm_n_2) {
                CB_LOG(DEBUG_LOG, "%s: Failed to check boot(2)(%d  %d)\n", __func__, fp_therm_n_1, fp_therm_n_2);
                return -1;
            }
#endif /* SUPPORT_VOLTAGE_DETECTOR */
            break;
        case CBFP_IOC_DISABLE_POWER:
            cbfp_hw_power_enable(cbfp, 0);
            break;
        case CBFP_IOC_ENABLE_IRQ:
            CB_LOG(DEBUG_LOG, "%s: Enable IRQ(%d)\n", __func__, cbfp->fp_int_gpio);
            break;
        case CBFP_IOC_DISABLE_IRQ:
            CB_LOG(DEBUG_LOG, "%s: Disable IRQ(%d)\n", __func__, cbfp->fp_int_gpio);
            break;
        case CBFP_IOC_REMOVE_DEVICE:
            CB_LOG(DEBUG_LOG, "%s: Remove device\n", __func__);
            cb_manage_sysfs(cbfp, false);
            disable_irq(cbfp->irq);
            break;
        case CBFP_IOC_RESET:
            CB_LOG(DEBUG_LOG, "%s: HW Reset%d)\n", __func__, cbfp->fp_rst_gpio);
            break;
        case CBFP_IOC_ON_SEQUENCE:
            CB_LOG(DEBUG_LOG, "%s: ON Sequence\n", __func__);
            /* power on -> spi config -> RST assert (high) */
            cbfp_hw_power_enable(cbfp, 1); // power on
            CB_LOG(DEBUG_LOG, "%s: Power enabled\n", __func__);
#if defined(SUPPORT_PINCTRL)
            select_pin_ctrl(cbfp, "cbfp_spi_cs_sleep");
#endif /* SUPPORT_PINCTRL */
            cbfp_reset_gpio_control(cbfp, 0); // deassert
            enable_irq(cbfp->irq);
            CB_LOG(DEBUG_LOG, "%s: reset asserted (high)\n", __func__);
            break;
        case CBFP_IOC_OFF_SEQUENCE:
            CB_LOG(DEBUG_LOG, "%s: OFF Sequence\n", __func__);
            /* RST assert (low) -> spi config to initial -> power off */
            cbfp_reset_gpio_control(cbfp, 1); // assert
            cbfp_hw_power_enable(cbfp, 0); // power off
            break;
        case CBFP_IOC_RST_ASSERT:
            CB_LOG(DEBUG_LOG, "%s: RST_N ASSERT\n", __func__);
            cbfp_reset_gpio_control(cbfp, 1);
            break;
        case CBFP_IOC_RST_DEASSERT:
            CB_LOG(DEBUG_LOG, "%s: RST_N DEASSERT\n", __func__);
            cbfp_reset_gpio_control(cbfp, 0);
            break;
        case CBFP_IOC_REG_TASK:
            CB_LOG(DEBUG_LOG, "%s: REG_TASK\n", __func__);
            cbfp->task = get_current();
            break;
        default:
            rc = -ENOTTY;
            goto err;
    };

    return 0;

err:
    return rc;
}

static long btp_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    CB_LOG(INFO_LOG, "%s: ioctl cmd : 0x%x\n", __func__, cmd);
    return btp_ioctl(file, cmd, (unsigned long)(arg));
}

static int btp_open(struct inode *node, struct file *file)
{
    cbfp_data_t *cbfp = NULL;

    CB_LOG(INFO_LOG, "%s\n", __func__);

    cbfp = (cbfp_data_t *)container_of(node->i_cdev, cbfp_data_t, cdev);
    file->private_data = cbfp;

    return 0;
}

static int btp_release(struct inode *node, struct file *file)
{
    cbfp_data_t *cbfp = NULL;
    CB_LOG(INFO_LOG, "%s\n", __func__);

    cbfp = file->private_data;
    file->private_data = NULL; // release private data

    return 0;
}

static unsigned int btp_poll(struct file *file, struct poll_table_struct *wait)
{
    unsigned int rc = 0;

    CB_LOG(INFO_LOG, "%s\n", __func__);
    return rc;
}

static int btp_create_class(struct cb_data *cbfp)
{
    int rc = 0;

    CB_LOG(INFO_LOG, "%s\n", __func__);

    BUILD_BUG_ON(N_SPI_MINORS > 256);
    rc = register_chrdev(CBFP_MAJOR, CBFP_DEV_NAME, &btp_fops);
    cbfp->class = class_create(THIS_MODULE, CBFP_CLASS_NAME);
    if (IS_ERR(cbfp->class)) {
        CB_LOG(ERR_LOG, "%s: Failed to create class\n", __func__);
        rc = PTR_ERR(cbfp->class);
    }

    return rc;
}

static int btp_create_device(struct cb_data *cbfp)
{
    int rc = 0;
    unsigned long minor;

    CB_LOG(INFO_LOG, "%s\n", __func__);
    minor = find_first_zero_bit(minors, N_SPI_MINORS);
    if (minor < N_SPI_MINORS) {
        struct device *dev;

        cbfp->devno = MKDEV(CBFP_MAJOR, minor);
        dev = device_create(cbfp->class, &cbfp->pdev->dev, cbfp->devno,
                cbfp, CBFP_DEV_NAME);
        rc = IS_ERR(dev) ? PTR_ERR(dev) : 0;
    } else {
        CB_LOG(ERR_LOG, "%s: no minor number available\n", __func__);
        rc = -ENODEV;
    }
    set_bit(minor, minors);
    return rc;
}

static void cbfp_hw_power_enable(cbfp_data_t *cbfp, u8 onoff)
{
    // To reduce the time, check the validation of gpios
    if (cbfp == NULL) {
        CB_LOG(ERR_LOG, "%s: platform data is INVALID\n", __func__);
        return;
    }

    if (!gpio_is_valid(cbfp->fp_vdd_gpio) || !gpio_is_valid(cbfp->fp_vddio_gpio)) {
        CB_LOG(ERR_LOG, "%s: VDD or VDDIO gpio is invalid\n", __func__);
        return;
    }

    if (onoff) {
#if defined(SUPPORT_VOLTAGE_DETECTOR)
        cbfp_cfg_vdet_irq_gpio(cbfp, true);
#endif /* SUPPORD_VOLTAGE_DETECTOR */

        // on
        gpio_set_value(cbfp->fp_vdd_gpio, 1);
        mdelay(1);
        gpio_set_value(cbfp->fp_vddio_gpio, 1);

        cbfp->pwr_state = FP_POWER_STATE_ON;
    } else {
        cbfp->pwr_state = FP_POWER_STATE_OFF;

#if defined(SUPPORT_VOLTAGE_DETECTOR)
        cbfp_cfg_vdet_irq_gpio(cbfp, false);
#endif /* SUPPORT_VOLTAGE_DETECTOR */

        // off
        gpio_set_value(cbfp->fp_vddio_gpio, 0);
        mdelay(1);
        gpio_set_value(cbfp->fp_vdd_gpio, 0);
    }

    sysfs_notify(&cbfp->pdev->dev.kobj, NULL, dev_attr_irq.attr.name);
}

static int cbfp_get_gpio_dts_info(cbfp_data_t *cbfp)
{
    int rc = 0;

    CB_LOG(INFO_LOG, "%s\n", __func__);
    rc = cb_request_named_gpio(cbfp, "fp-vdd-en", &cbfp->fp_vdd_gpio);
    if (rc) {
        return -1;
    }
    gpio_direction_output(cbfp->fp_vdd_gpio, 0);

    rc = cb_request_named_gpio(cbfp, "fp-vddio-en", &cbfp->fp_vddio_gpio);
    if (rc) {
        return -1;
    }
    gpio_direction_output(cbfp->fp_vddio_gpio, 0);

    rc = cb_request_named_gpio(cbfp, "fpint-gpio", &cbfp->fp_int_gpio);
    if (rc) {
        return -1;
    }
    gpio_direction_input(cbfp->fp_int_gpio);

    rc = cb_request_named_gpio(cbfp, "fpreset-gpio", &cbfp->fp_rst_gpio);
    if (rc) {
        return -1;
    }
    gpio_direction_output(cbfp->fp_rst_gpio, 0);
    gpio_set_value(cbfp->fp_rst_gpio, 0);

    CB_LOG(DEBUG_LOG, "%s: VDD[%d] VDDIO[%d] INT[%d] RST[%d]\n", __func__,
            cbfp->fp_vdd_gpio, cbfp->fp_vddio_gpio, cbfp->fp_int_gpio, cbfp->fp_rst_gpio);

#if defined(SUPPORT_VOLTAGE_DETECTOR)
    rc = cb_request_named_gpio(cbfp, "fpvdet-vdd-gpio", &cbfp->vdet_vdd_gpio);
    if (rc) {
        return -1;
    }
    gpio_direction_input(cbfp->vdet_vdd_gpio);

    rc = cb_request_named_gpio(cbfp, "fpvdet-vddio-gpio", &cbfp->vdet_vddio_gpio);
    if (rc) {
        return 0; // temp - no gpio
        //return -1;
    }
    gpio_direction_input(cbfp->vdet_vddio_gpio);

    CB_LOG(DEBUG_LOG, "%s: VDET_VDD[%d] VDET_VDDIO[%d]\n", __func__,
            cbfp->vdet_vdd_gpio, cbfp->vdet_vddio_gpio);
#endif /* SUPPORT_VOLTAGE_DETECTOR */

    return 0;
}

static void cbfp_gpio_free(cbfp_data_t *cbfp)
{
    struct device * dev = &cbfp->pdev->dev;

    if (gpio_is_valid(cbfp->fp_int_gpio)) {
        if(cbfp->irq) free_irq(cbfp->irq, cbfp);
        devm_gpio_free(dev, cbfp->fp_int_gpio);
    }

    if (gpio_is_valid(cbfp->fp_rst_gpio)) {
        devm_gpio_free(dev, cbfp->fp_rst_gpio);
    }

    if (gpio_is_valid(cbfp->fp_vdd_gpio)) {
        devm_gpio_free(dev, cbfp->fp_vdd_gpio);
    }

    if (gpio_is_valid(cbfp->fp_vddio_gpio)) {
        devm_gpio_free(dev, cbfp->fp_vddio_gpio);
    }

#if defined(SUPPORT_VOLTAGE_DETECTOR)
    if (gpio_is_valid(cbfp->vdet_vdd_gpio)) {
        devm_gpio_free(dev, cbfp->vdet_vdd_gpio);
    }

    if (gpio_is_valid(cbfp->vdet_vddio_gpio)) {
        devm_gpio_free(dev, cbfp->vdet_vddio_gpio);
    }
#endif /* SUPPORT_VOLTAGE_DETECTOR */
}

static void cbfp_cfg_irq_gpio(cbfp_data_t *cbfp)
{
    int rc = 0;

    cbfp->irq = gpio_to_irq(cbfp->fp_int_gpio);
    if (cbfp->irq < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to get irq\n", __func__);
        return;
    }

    rc = devm_request_threaded_irq(&cbfp->pdev->dev, cbfp->irq,
                                    NULL, cb_interrupt_handler,
                                    IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                    dev_name(&cbfp->pdev->dev), cbfp);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to request devm_request_threaded_irq(%i)\n", __func__, cbfp->irq);
        cbfp->irq = -EINVAL;
        return;
    }

    //enable_irq_wake(cbfp->irq);
    //disable_irq(cbfp->irq);

    disable_irq(cbfp->irq);
    cbfp->interrupt_done = 0;
    enable_irq(cbfp->irq);
    enable_irq_wake(cbfp->irq);
}

static void cbfp_reset_gpio_control(cbfp_data_t *cbfp, u8 assert)
{
    if (assert) {
        // LOW
        if (gpio_is_valid(cbfp->fp_rst_gpio)) {
            gpio_set_value(cbfp->fp_rst_gpio, 0);
        }
    } else {
        // HIGH
        if (gpio_is_valid(cbfp->fp_rst_gpio)) {
            gpio_set_value(cbfp->fp_rst_gpio, 1);
        }
    }
}

#if defined(SUPPORT_PINCTRL)
static void cbfp_cfg_pinctrl(cbfp_data_t *cbfp)
{
    //int rc = 0;
    struct device *dev = &cbfp->pdev->dev;
    int i = 0;

    cbfp->pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(cbfp->pinctrl)) {
        if (PTR_ERR(cbfp->pinctrl) == -EPROBE_DEFER) {
            CB_LOG(ERR_LOG, "%s: Failed to get pinctrl\n", __func__);
            return;
        }
        CB_LOG(ERR_LOG, "%s: Target device doesn't use pinctrl\n", __func__);
        cbfp->pinctrl = NULL;
        return;
    }

    for (i=0 ; i<ARRAY_SIZE(cbfp->pinctrl_state) ; i++) {
        const char * n = pinctrl_names[i];
        struct pinctrl_state *state = pinctrl_lookup_state(cbfp->pinctrl, n);
        if (IS_ERR(state)) {
            CB_LOG(ERR_LOG, "%s: Failed to find pinctrl state\n", __func__);
            continue;
        }
        CB_LOG(DEBUG_LOG, "%s: Found pinctrl %s\n", __func__, n);
        cbfp->pinctrl_state[i] = state;
    }

    return;
}
#endif /* SUPPORT_PINCTRL */

#if defined(SUPPORT_VOLTAGE_DETECTOR)
static irqreturn_t cbfp_vdet_irq_handler(int irq, void * data)
{
    cbfp_data_t *cbfp = NULL;
	struct siginfo info;
    const uint8_t flag_vdd =0x01;
    const uint8_t flag_vddio =0x10;

    CB_LOG(INFO_LOG, "%s: irq no. %d\n", __func__, irq);

    if (!data)
        return IRQ_NONE;

    cbfp = (cbfp_data_t *) data;
    if (!cbfp)
        return IRQ_NONE;

    // ignore when power state is off (OFF sequence)
    if (cbfp->pwr_state == FP_POWER_STATE_OFF)
        return IRQ_NONE;

    smp_rmb();
    //wake_lock_timeout(&cbfp->fp_wake_lock, msecs_to_jiffies(CB_WAKE_LOCK_HOLD_TIME));

    if (irq == cbfp->vdet_vdd_irq) {
        CB_LOG(INFO_LOG, "%s: VDD IRQ occured, gpio value = %d\n", __func__, gpio_get_value(cbfp->fp_vdd_gpio));
        cbfp->irq_drop |= flag_vdd;
    }
    if (irq == cbfp->vdet_vddio_irq) {
        CB_LOG(INFO_LOG, "%s: VDDIO IRQ occured, gpio value = %d\n", __func__, gpio_get_value(cbfp->fp_vddio_gpio));
        cbfp->irq_drop|=flag_vddio;
    } else {
        return IRQ_NONE;
    }

    memset(&info, 0x00, sizeof(struct siginfo));
    info.si_signo = SIGPWRDRP;
    info.si_code = SI_QUEUE;
    info.si_int = 1;

    if (cbfp->task != NULL) {
        CB_LOG(INFO_LOG, "%s: Send signal to HAL\n", __func__);
        if (send_sig_info(SIGPWRDRP, &info, cbfp->task) < 0) {
            CB_LOG(ERR_LOG, "%s: Failed to send signal\n", __func__);
        }
    }

    return IRQ_HANDLED;
}

static void cbfp_cfg_vdet_irq_gpio(cbfp_data_t *cbfp, bool onoff)
{
    int rc = 0;

    if (!onoff) {
        if (cbfp->vdet_vdd_irq > 0) {
            devm_free_irq(&cbfp->pdev->dev, cbfp->vdet_vdd_irq, NULL);
            cbfp->vdet_vdd_irq = 0;
        }
        if (cbfp->vdet_vddio_irq > 0) {
            devm_free_irq(&cbfp->pdev->dev, cbfp->vdet_vddio_irq, NULL);
            cbfp->vdet_vddio_irq = 0;
        }
        return;
    }

    if (!gpio_is_valid(cbfp->vdet_vdd_gpio) || !gpio_is_valid(cbfp->vdet_vddio_gpio)) {
        CB_LOG(ERR_LOG, "%s: GPIO is invalid, no irq config\n", __func__);
        return;
    }

    cbfp->vdet_vdd_irq = gpio_to_irq(cbfp->vdet_vdd_gpio);
    if (cbfp->vdet_vdd_irq < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to get vdet_vdd_irq\n", __func__);
        return;
    }

    rc = devm_request_threaded_irq(&cbfp->pdev->dev, cbfp->vdet_vdd_irq,
                                    NULL, cbfp_vdet_irq_handler,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                    dev_name(&cbfp->pdev->dev), cbfp);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to request devm_request_threaded_irq(%i)\n", __func__, cbfp->vdet_vdd_irq);
        cbfp->vdet_vdd_irq = -EINVAL;
        return;
    }

    //enable_irq_wake(cbfp->irq);
    //disable_irq(cbfp->irq);

    disable_irq(cbfp->vdet_vdd_irq);
    enable_irq(cbfp->vdet_vdd_irq);
    //enable_irq_wake(cbfp->vdet_vdd_irq);

    cbfp->vdet_vddio_irq = gpio_to_irq(cbfp->vdet_vddio_gpio);
    if (cbfp->vdet_vddio_irq < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to get vdet_vddio_irq\n", __func__);
        return;
    }

    rc = devm_request_threaded_irq(&cbfp->pdev->dev, cbfp->vdet_vddio_irq,
                                    NULL, cbfp_vdet_irq_handler,
                                    IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
                                    dev_name(&cbfp->pdev->dev), cbfp);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to request devm_request_threaded_irq(%i)\n", __func__, cbfp->vdet_vddio_irq);
        cbfp->vdet_vddio_irq = -EINVAL;
        return;
    }

    //enable_irq_wake(cbfp->irq);
    //disable_irq(cbfp->irq);

    disable_irq(cbfp->vdet_vddio_irq);
    enable_irq(cbfp->vdet_vddio_irq);
    //enable_irq_wake(cbfp->vdet_vddio_irq);

}
#endif /* SUPPORT_VOLTAGE_DETECTOR */

/* -------------------------------------------------------------------- */
/* Platform drivers description and functions                           */
/* -------------------------------------------------------------------- */
#ifdef CONFIG_OF
static struct of_device_id cb_of_match[] = {
  { .compatible = "cb,btp", },
  {}
};

MODULE_DEVICE_TABLE(of, cb_of_match);
#endif

static struct platform_driver cbfp_driver = {
    .driver = {
        .name   = CBFP_DEV_NAME,
        .owner  = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = cb_of_match,
#endif
    },
    .probe  = cbfp_drv_probe,
    .remove = cbfp_drv_remove,
};

module_init(cbfp_drv_init);
module_exit(cbfp_drv_exit);

static int cbfp_drv_init(void)
{
    CB_LOG(INFO_LOG, "%s\n", __func__);

    if (platform_driver_register(&cbfp_driver))
        return -EINVAL;

    return 0;
}

/* -------------------------------------------------------------------- */
static void cbfp_drv_exit(void)
{
    CB_LOG(INFO_LOG, "%s\n", __func__);

    platform_driver_unregister(&cbfp_driver);
}

/* -------------------------------------------------------------------- */
static int cbfp_drv_probe(struct platform_device *pdev)
{
    cbfp_data_t *cbfp = NULL;

    int rc = 0;

    CB_LOG(INFO_LOG, "%s: entry\n", __func__);

#if defined(SUPPORT_DEIVE_ID)
    rc = device_id_confirm(pdev);
    if(rc){
        CB_LOG(ERR_LOG, "%s: this device is not supported\n", __func__);
        return -EINVAL;
    }
#endif

    cbfp = kzalloc(sizeof(cbfp_data_t), GFP_KERNEL);
    if (!cbfp) {
        CB_LOG(ERR_LOG, "%s: Failed to allocate memory for cbfp_data_t\n", __func__);
        return -ENOMEM;
    }
    cbfp->pdev = pdev;

    /* Initialize driver data */
    sema_init(&cbfp->mutex, 0);

    cbfp->fp_rst_gpio = -EINVAL;
    cbfp->fp_int_gpio = -EINVAL;
    cbfp->fp_vdd_gpio = -EINVAL;
    cbfp->fp_vddio_gpio = -EINVAL;

#if defined(SUPPORT_VOLTAGE_DETECTOR)
    cbfp->vdet_vdd_gpio = -EINVAL;
    cbfp->vdet_vddio_gpio = -EINVAL;
#endif /* SUPPORT_VOLTAGE_DETECTOR */

    cbfp->irq = -EINVAL;
    cbfp->interrupt_done = 0;

    rc = cbfp_get_gpio_dts_info(cbfp);
    if (rc) goto err3;
    cbfp_cfg_irq_gpio(cbfp);
    cb_manage_sysfs(cbfp, true);

#if defined(SUPPORT_LCD_CONTROL)
    cbfp->fb_notifier.notifier_call = fb_notifier_callback;
    rc = fb_register_client(&cbfp->fb_notifier);
    if (rc)
        CB_LOG(ERR_LOG, "%s: Failed to register FB client(%d)\n", __func__, rc);
#endif /* SUPPORT_LCD_CONTROL */

    /* setup cbfp device configuration */
    CB_LOG(INFO_LOG, "%s: Trying to create btp class\n", __func__);
    rc = btp_create_class(cbfp);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to allocate memory for struct cb_data\n", __func__);
        goto err3;
    }

    CB_LOG(INFO_LOG, "%s: Trying to create btp device\n", __func__);
    rc = btp_create_device(cbfp);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to allocate memory for struct cb_data\n", __func__);
        goto err2;
    }

    cdev_init(&cbfp->cdev, &btp_fops);
    cbfp->cdev.owner = THIS_MODULE;
    rc = cdev_add(&cbfp->cdev, cbfp->devno, 1);
    if (rc) {
        CB_LOG(ERR_LOG, "%s: Failed to add chrdev\n", __func__);
        goto err1;
    }

    wake_lock_init(&cbfp->fp_wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");
    platform_set_drvdata(pdev, cbfp);

    up(&cbfp->mutex);
    CB_LOG(INFO_LOG, "%s: exit(OK)\n", __func__);

    return 0;

err1:
    device_destroy(cbfp->class, cbfp->devno);
err2:
    class_destroy(cbfp->class);
err3:
    up(&cbfp->mutex);
    kfree(cbfp);
    CB_LOG(INFO_LOG, "%s: exit(ERR-%d)\n", __func__, rc);
    return -ENODEV;
}

/* -------------------------------------------------------------------- */
static int cbfp_drv_remove(struct platform_device *pdev)
{
    cbfp_data_t *cbfp = platform_get_drvdata(pdev);
    struct task_struct *ref_task = get_current();

    if (ref_task != NULL && cbfp->task != NULL) {
        if (ref_task == cbfp->task) cbfp->task = NULL;
    }

    cbfp_gpio_free(cbfp);
    wake_lock_destroy(&cbfp->fp_wake_lock);

    cdev_del(&cbfp->cdev);
    device_destroy(cbfp->class, cbfp->devno);
    class_destroy(cbfp->class);

    kfree(cbfp);
    return 0;
}

//TODO
#if defined(SUPPORT_DEIVE_ID)
/* -------------------------------------------------------------------- */
static int device_id_confirm(struct platform_device *pdev)
{
    int rc =0;
    int fp_dev_gpio = 0;
    struct device *dev = &pdev->dev;
    const char *id_label = "fpdev-id-gpio";
    struct device_node *np = dev->of_node;
    struct pinctrl *devid_pinctrl;
    struct pinctrl_state *devid_state;

    devid_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(devid_pinctrl)) {
        CB_LOG(ERR_LOG, "%s: Failed to devm_pinctrl_get\n", __func__);
        return -1;
    }
    devid_state = pinctrl_lookup_state(devid_pinctrl,"fp_devid_start");
    if (IS_ERR(devid_state)) {
        CB_LOG(ERR_LOG, "%s: Failed to pinctrl_lookup_state\n", __func__);
        return -1;
    }
    rc =pinctrl_select_state(devid_pinctrl,devid_state);
    if (rc < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to pinctrl_select_state\n", __func__);
        return -1;
    }
    mdelay(10);

    rc = of_get_named_gpio(np, id_label, 0);
    fp_dev_gpio = rc;

    if (rc < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to get gpio(%s)\n", __func__, id_label);
        return -1;
    }

    rc = devm_gpio_request(dev, rc, id_label);
    if (rc) {
		CB_LOG(ERR_LOG, "%s:failed to request gpio %d\n", __func__,fp_dev_gpio);
		return -1;
    }

    if(gpio_get_value(fp_dev_gpio)){
        CB_LOG(ERR_LOG, "%s: GPIO(%d) ID is different\n ", __func__, fp_dev_gpio);
        return -1;
    }
    mdelay(10);

    devid_state = pinctrl_lookup_state(devid_pinctrl,"fp_devid_end");
    if (IS_ERR(devid_state)) {
        CB_LOG(ERR_LOG, "%s: Failed to pinctrl_lookup_state\n", __func__);
        return -1;
    }
    rc =pinctrl_select_state(devid_pinctrl,devid_state);
    if (rc < 0) {
        CB_LOG(ERR_LOG, "%s: Failed to pinctrl_select_state\n", __func__);
        return -1;
    }
    return 0;
}
#endif /* SUPPORT_DEIVE_ID */

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Canvasbio <www.canvasbio.com>");
MODULE_DESCRIPTION("BTP sensor platform driver.");
