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

//#define DEBUG

#define IRQ_SYSFS
#define SUPPORT_PM_VREG_CONTROL
#define SUPPORT_PM_GPIO_CONTROL

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/platform_device.h>

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#if defined(CONFIG_THERMAL)
#include <linux/thermal.h>
#include <linux/qpnp/qpnp-adc.h>
#endif /* CONFIG_THERMAL */

#if defined(SUPPORT_PM_VREG_CONTROL)
#include <linux/regulator/consumer.h>
#endif /* SUPPORT_PM_VREG_CONTROL */


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Canvasbio <www.canvasbio.com>");
MODULE_DESCRIPTION("BTP sensor platform driver.");

#define CB_VERSION                      "1.0.0"

/**************************debug******************************/
#define ERR_LOG     (0)
#define INFO_LOG    (1)
#define DEBUG_LOG   (2)

/* debug log setting */
u8 debug_level = INFO_LOG;

#define LOG_PRINT(level, fmt, args...) do { \
    if (debug_level >= level) { \
         pr_debug(KERN_DEBUG "CanvasBio [CB_] " fmt, ##args); \
    } \
} while (0)

/* -------------------------------------------------------------------- */
/* driver constants                                                     */
/* -------------------------------------------------------------------- */
#define CB_WAKE_LOCK_HOLD_TIME           1000

/* -------------------------------------------------------------------- */
/* sysfs permission                                                     */
/* -------------------------------------------------------------------- */
#define HW_CONTROL_MODE_RW   (S_IWUSR | S_IRUSR | S_IRGRP | S_IWGRP | S_IROTH )
#define HW_CONTROL_MODE_R    (S_IRUSR | S_IRGRP | S_IROTH)


#if defined(SUPPORT_PM_VREG_CONTROL)
/* -------------------------------------------------------------------- */
/* Board constants                                                      */
/* -------------------------------------------------------------------- */
// B0 : Control VDD 3.3V only
// B1 : Control VDD 2.8V and VDD_IO 1.8V
#define JA32_EV_B0_BOARD            0xFF00
#define JA32_EV_B1_BOARD            0xFF01

/* -------------------------------------------------------------------- */
/* Regulator constants                                                  */
/* -------------------------------------------------------------------- */
#define SUPPLY_1P8V                 1800000UL
#define SUPPLY_2P8V                 2800000UL
#define SUPPLY_3P3V                 3300000UL

#define SUPPLY_VDD_IO_MIN           SUPPLY_1P8V
#define SUPPLY_VDD_IO_MAX           SUPPLY_1P8V
#define SUPPLY_VDD_MIN_B0           SUPPLY_3P3V
#define SUPPLY_VDD_MAX_B0           SUPPLY_3P3V
#define SUPPLY_VDD_MIN_B1           SUPPLY_2P8V
#define SUPPLY_VDD_MAX_B1           SUPPLY_2P8V
#define SUPPLY_TX_MIN               SUPPLY_3P3V
#define SUPPLY_TX_MAX               SUPPLY_3P3V

typedef enum {
    PM_VREG_VDD     = 0,
    PM_VREG_VDD_IO  = 1,
    PM_VREG_MAX     = PM_VREG_VDD_IO,
} pm_vreg_t;
#endif /* SUPPORT_PM_VREG_CONTROL */

#if defined(SUPPORT_PIN_CTRL)
static const char * pin_ctrl_names[] = {
    //"cbfp_spi_active",
    "cbfp_reset_reset",
    "cbfp_reset_active",
    "cbfp_irq_active",
};
#endif /* SUPPORT_PIN_CTRL */

#if defined(CONFIG_THERMAL)
/* -------------------------------------------------------------------- */
/* Thermistor constants                                                 */
/* -------------------------------------------------------------------- */
typedef enum {
    THERMAL_SENSOR_NORMAL = 0,
    THERMAL_SENSOR_TOO_HOT,
} thermal_status_t;
#endif /* CONFIG_THERMAL */

/* -------------------------------------------------------------------- */
/* data types                                                           */
/* -------------------------------------------------------------------- */
struct cb_platform_data {
    int irq_gpio;
    int reset_gpio;
};

struct cb_data {
    struct platform_device *pdev;
    struct device *device;
    struct semaphore mutex;
    u32 reset_gpio;
    u32 irq_gpio;
    u32 irq;
    int interrupt_done;
#if !defined (IRQ_SYSFS)
    struct input_dev *input;
#endif

#if defined(SUPPORT_PM_VREG_CONTROL)
    u16 board_rev;

    // PMIC regulator control
    struct regulator * vreg_vdd;
    struct regulator * vreg_vdd_io;
#endif /* SUPPORT_PM_VREG_CONTROL */

#if defined(SUPPORT_PM_GPIO_CONTROL)
    u32 fp_vdd_gpio;
    u32 fp_vdd_io_gpio;
#endif /* SUPPORT_PM_GPIO_CONTROL */

#if defined(SUPPORT_PIN_CTRL)
    // Pin control
    struct pinctrl * cbfp_pinctrl;
    struct pinctrl_state * pinctrl_state[ARRAY_SIZE(pin_ctrl_names)];
#endif /* SUPPORT_PIN_CTRL */

#if defined(CONFIG_FB)
    struct notifier_block fb_notifier;
#endif
    struct wake_lock fp_wake_lock;

#if defined(CONFIG_THERMAL)
    const char * thermal_zone_name;
    u32 fp_thermal_polling_interval;

    s32 fp_temp_thresh_hot;
    s32 fp_temp_thresh_normal;
    s32 fp_high_thr;
    s32 fp_low_thr;

    struct thermal_zone_device *tzd_fp;
    struct delayed_work fp_thermal_monitor;
    struct qpnp_adc_tm_chip	*adc_tm_dev;
    struct qpnp_adc_tm_btm_param adc_param;
    struct mutex		fp_thermal_complete;

    enum qpnp_tm_state state_temp;

    thermal_status_t fp_thermal_status;

    bool fp_need_notify;
#endif /* CONFIG_THERMAL */
};

ssize_t (* cb_navi_show_event)(struct device *, struct device_attribute *, char *) = NULL;
ssize_t (* cb_navi_store_event)(struct device *, struct device_attribute *, char *, size_t) = NULL;

/* -------------------------------------------------------------------- */
/* function prototypes                          */
/* -------------------------------------------------------------------- */
static int cb_init(void);
static void cb_exit(void);
static int cb_probe(struct platform_device *pdev);
static int cb_remove(struct platform_device *pdev);

static ssize_t cb_show_attr_reset(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cb_store_attr_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t cb_show_attr_irq(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t cb_show_attr_navi_event(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t cb_store_attr_navi_event(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#if defined (CONFIG_OF)
static int cb_get_of_pdata(struct cb_data * cbfp, struct device *dev, struct cb_platform_data *pdata);
#endif

#if defined (CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
#endif

static irqreturn_t cb_interrupt_handler(int irq, void * data);

#if !defined (IRQ_SYSFS)
static int cb_register_input_device(struct cb_data *cbfp, bool is_register);
#endif

static int cb_reset_init(struct cb_data *cbfp, struct cb_platform_data *pdata);
static int cb_irq_init(struct cb_data *cbfp, struct cb_platform_data *pdata);
static int cb_manage_sysfs(struct cb_data *cbfp, struct platform_device *pdev, bool create);

int cb_reset(struct cb_data *cbfp);

#if defined(SUPPORT_PM_VREG_CONTROL)
static int pm_vreg_release(struct regulator * vreg);
static int pm_vreg_configure(struct cb_data *cbfp, const char * vreg_name, pm_vreg_t vreg_type, int min_uV, int max_uV);
static int pm_vreg_enable(struct cb_data *cbfp, struct regulator * vreg, bool enable);
#endif /* SUPPORT_PM_VREG_CONTROL */

#if defined(SUPPORT_PM_GPIO_CONTROL)
static u32 pm_gpio_configure(struct cb_data * cbfp, const char * label);
#endif /* SUPPORT_PM_GPIO_CONTROL */

#if defined(SUPPORT_PIN_CTRL)
static int select_pin_ctrl(struct cb_data *cbfp, const char * pin_name);
#endif /* SUPPORT_PIN_CTRL */

#if defined(CONFIG_THERMAL)
static int cb_init_thermal_control(struct cb_data *cbfp);
static void cb_thermal_monitor(struct work_struct *work);
static void cb_thermal_adc_notification(enum qpnp_tm_state state, void *ctx);
#endif /* CONFIG_THERMAL */

/* -------------------------------------------------------------------- */
/* External interface                                                   */
/* -------------------------------------------------------------------- */
#if defined(CONFIG_THERMAL)
late_initcall(cb_init);
#else
module_init(cb_init);
#endif /* CONFIG_THERMAL */
module_exit(cb_exit);

#ifdef CONFIG_OF
static struct of_device_id cb_of_match[] = {
  { .compatible = "cb,btp", },
  {}
};

MODULE_DEVICE_TABLE(of, cb_of_match);
#endif

static struct platform_driver cb_driver = {
    .driver = {
        .name   = "btp",
        .owner  = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = cb_of_match,
#endif
    },
    .probe  = cb_probe,
    .remove = cb_remove,
};

/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
static DEVICE_ATTR(reset,  HW_CONTROL_MODE_RW, cb_show_attr_reset, cb_store_attr_reset);
static DEVICE_ATTR(irq,   HW_CONTROL_MODE_R, cb_show_attr_irq, NULL);
static DEVICE_ATTR(navi_event,  HW_CONTROL_MODE_RW, cb_show_attr_navi_event, cb_store_attr_navi_event);

static struct attribute *cb_hw_control_attrs[] = {
    &dev_attr_reset.attr,
    &dev_attr_irq.attr,
    &dev_attr_navi_event.attr,
    NULL
};

static const struct attribute_group cb_hw_control_attr_group = {
    .attrs = cb_hw_control_attrs,
};

/* -------------------------------------------------------------------- */
/* function definitions                                                 */
/* -------------------------------------------------------------------- */
static int cb_init(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (platform_driver_register(&cb_driver))
        return -EINVAL;

    return 0;
}

/* -------------------------------------------------------------------- */
static void cb_exit(void)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    platform_driver_unregister(&cb_driver);
}

/* -------------------------------------------------------------------- */
static int cb_probe(struct platform_device *pdev)
{
    struct cb_platform_data *cb_pdata;
    struct cb_platform_data pdata_of;
    struct device *dev = &pdev->dev;
    struct cb_data *cbfp = NULL;
    int rc = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    cbfp = kzalloc(sizeof(struct cb_data), GFP_KERNEL);
    if (!cbfp) {
        dev_err(&pdev->dev, "[ERROR] failed to allocate memory for struct cb_data\n");
        return -ENOMEM;
    }

    sema_init(&cbfp->mutex, 0);

    platform_set_drvdata(pdev, cbfp);
    cbfp->pdev = pdev;

    cb_pdata = dev_get_platdata(&pdev->dev);
    if (!cb_pdata) {
        rc = cb_get_of_pdata(cbfp, dev, &pdata_of);
        if (rc)
            goto err;
        cb_pdata = &pdata_of;
    }

    if (!cb_pdata) {
        dev_err(&pdev->dev, "[ERROR] spi->dev.platform_data is NULL.\n");
        rc = -EINVAL;
        goto err;
    }

#if defined(SUPPORT_PM_VREG_CONTROL)
    if (cbfp->board_rev == JA32_EV_B0_BOARD) {
        // B0 Board. Set only VDD 3.3V
        dev_err(&pdev->dev, "%s: Configuring vreg of PMIC of B0 Board\n", __func__);
        pm_vreg_configure(cbfp, "vdd", PM_VREG_VDD, SUPPLY_VDD_MIN_B0, SUPPLY_VDD_MAX_B0); //3.3V
        pm_vreg_enable(cbfp, cbfp->vreg_vdd, true);
        dev_err(&pdev->dev, "%s: Configuring vreg done, Power ON!!!\n", __func__);
    }
#endif /* SUPPORT_PM_VREG_CONTROL */

#if defined(SUPPORT_PIN_CTRL)
    select_pin_ctrl(cbfp, "cbfp_spi_active");
#endif /* SUPPORT_PIN_CTRL */

    cbfp->reset_gpio = -EINVAL;
    cbfp->irq_gpio = -EINVAL;
    cbfp->irq = -EINVAL;

#if !defined (IRQ_SYSFS)
    rc = cb_register_input_device(cbfp, true);
    if (rc)
        goto err;
#endif

    rc = cb_irq_init(cbfp, cb_pdata);
    if (rc) {
        dev_err(&pdev->dev, "[ERROR] cb_irq_init() failed. error %d\n", rc);
        LOG_PRINT(ERR_LOG, "%s: Failed to irq gpio\n", __func__);
        goto err1;
    }

    rc = cb_reset_init(cbfp, cb_pdata);
    if (rc) {
        dev_err(&pdev->dev, "[ERROR] cb_reset_init() failed. error %d\n", rc);
        LOG_PRINT(ERR_LOG, "%s: Failed to init reset gpio\n", __func__);
        goto err1;
    }
    pr_debug(KERN_DEBUG ": [DEBUG] irq : %d  reset : %d\n", cb_pdata->irq_gpio, cb_pdata->reset_gpio);

    rc = cb_manage_sysfs(cbfp, pdev, true);
    if (rc)
        goto err1;

#if defined(CONFIG_FB)
    cbfp->fb_notifier.notifier_call = fb_notifier_callback;
    rc = fb_register_client(&cbfp->fb_notifier);
    if (rc)
        dev_err(&pdev->dev, "Unable to register fb_notifier: %d\n", rc);
#else
    dev_err(&pdev->dev, "%s, Warning!!CONFIG_FB define is inactivity.\n", __FUNCTION__);
#endif

#if defined(CONFIG_THERMAL)
    cb_init_thermal_control(cbfp);
#endif /* CONFIG_THERMAL */

    up(&cbfp->mutex);

    return 0;

err1:
#if !defined (IRQ_SYSFS)
    rc = cb_register_input_device(cbfp, false);
#endif
err:
    up(&cbfp->mutex);
    return rc;
}

/* -------------------------------------------------------------------- */
static int cb_remove(struct platform_device *pdev)
{
    struct cb_data *cbfp = platform_get_drvdata(pdev);

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    sysfs_remove_group(&cbfp->pdev->dev.kobj, &cb_hw_control_attr_group);

#if !defined (IRQ_SYSFS)
    cb_register_input_device(cbfp, false);
#endif

    if (cbfp->irq >= 0)
        free_irq(cbfp->irq, cbfp);

    if (gpio_is_valid(cbfp->irq_gpio))
        gpio_free(cbfp->irq_gpio);

    if (gpio_is_valid(cbfp->reset_gpio))
        gpio_free(cbfp->reset_gpio);

#if defined(SUPPORT_PM_GPIO_CONTROL)
    if (gpio_is_valid(cbfp->fp_vdd_gpio))
        gpio_free(cbfp->fp_vdd_gpio);

    if (gpio_is_valid(cbfp->fp_vdd_io_gpio))
        gpio_free(cbfp->fp_vdd_io_gpio);
#endif /* SUPPORT_PM_GPIO_CONTROL */

    wake_lock_destroy(&cbfp->fp_wake_lock);
    mutex_destroy(&cbfp->fp_thermal_complete);
    return 0;
}

/* -------------------------------------------------------------------- */
static ssize_t cb_show_attr_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct cb_data *cbfp = dev_get_drvdata(dev);

    //pr_debug(KERN_DEBUG ": [DEBUG] %s: fp_need_notify = %d\n", __func__, cbfp->fp_need_notify);
#if defined(CONFIG_THERMAL)
    if (cbfp->fp_need_notify) {
        cbfp->fp_need_notify = false;

        //pr_debug(KERN_DEBUG ": [DEBUG] %s: fp_thermal_status = %d\n", __func__, cbfp->fp_thermal_status);
        if (cbfp->fp_thermal_status == THERMAL_SENSOR_NORMAL)
            return scnprintf(buf, PAGE_SIZE, "%d\n", 2);
    }

    if (cbfp->fp_thermal_status == THERMAL_SENSOR_TOO_HOT)
        return scnprintf(buf, PAGE_SIZE, "%d\n", 3);
#endif /* CONFIG_THERMAL */
    return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(cbfp->irq_gpio));
}

/* -------------------------------------------------------------------- */
static ssize_t cb_show_attr_reset(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct cb_data *cbfp = dev_get_drvdata(dev);

    return scnprintf(buf, PAGE_SIZE, "%d\n", gpio_get_value(cbfp->reset_gpio));
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
        gpio_set_value(cbfp->reset_gpio, 1);
    } else if (!strcasecmp(cmd_buf, "low")) {
        gpio_set_value(cbfp->reset_gpio, 0);
    } else if (!strcasecmp(cmd_buf, "reset")) {
        cb_reset(cbfp);
    } else {
        dev_err(dev, "%s: reset command is wrong(%s)\n", __func__, cmd_buf);
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

#ifdef CONFIG_OF
/* -------------------------------------------------------------------- */
static int cb_get_of_pdata(struct cb_data * cbfp, struct device *dev, struct cb_platform_data *pdata)
{
    const struct device_node *node = dev->of_node;
    const void *irq_prop = of_get_property(node, "fpint-gpio",   NULL);
    const void *rst_prop = of_get_property(node, "fpreset-gpio", NULL);

    const void *vdd_prop = of_get_property(node, "fp-vdd-en", NULL);
    const void *vddio_prop = of_get_property(node, "fp-vdd-io-en", NULL);

    int i = 0;
    u32 gpio = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (node == NULL) {
        dev_err(dev, "%s: Could not find OF device node\n", __func__);
        goto of_err;
    }

    if (!irq_prop || !rst_prop ) {
        dev_err(dev, "%s: Missing OF property\n", __func__);
        goto of_err;
    }

    if (!vdd_prop || !vddio_prop) {
        dev_info(dev, "%s: Cannot find vdd and vdd-io gpio, this board is b0\n", __func__);
        cbfp->board_rev = JA32_EV_B0_BOARD;
    } else {
        dev_info(dev, "%s: Found vdd and vdd-io gpio, this board is b1\n", __func__);
        cbfp->board_rev = JA32_EV_B1_BOARD;
    }

#if defined(SUPPORT_PIN_CTRL)
    cbfp->cbfp_pinctrl = devm_pinctrl_get(dev);
    if (IS_ERR(cbfp->cbfp_pinctrl)) {
        if (PTR_ERR(cbfp->cbfp_pinctrl) == -EPROBE_DEFER) {
            dev_err(dev, "%s: pinctrl is not ready\n", __func__);
            return -EPROBE_DEFER;
        }
        dev_err(dev, "%s: Target is not using pinctrl\n", __func__);
        cbfp->cbfp_pinctrl = NULL;
        return -EINVAL;
    };

    for (i=0 ; i<ARRAY_SIZE(cbfp->pinctrl_state) ; i++) {
        const char * n = pin_ctrl_names[i];
        struct pinctrl_state * state = pinctrl_lookup_state(cbfp->cbfp_pinctrl, n);
        if (IS_ERR(state)) {
            dev_err(dev, "%s: cannot find pin ctrl [%s]\n", __func__, n);
            //return -EINVAL;
            continue;
        }
        dev_info(dev, "%s: Found pin ctrl [%s]\n", __func__, n);
        cbfp->pinctrl_state[i] = state;
    }
#endif /* SUPPORT_PIN_CTRL */

#if defined(SUPPORT_PM_GPIO_CONTROL)
    if (cbfp->board_rev == JA32_EV_B1_BOARD) {
        gpio = pm_gpio_configure(cbfp, "fp-vdd-en");
        if (gpio > 0 && gpio_is_valid(gpio)) {
            cbfp->fp_vdd_gpio = gpio;
        }

        gpio = pm_gpio_configure(cbfp, "fp-vdd-io-en");
        if (gpio > 0 && gpio_is_valid(gpio)) {
            cbfp->fp_vdd_io_gpio = gpio;
        }
    }
#endif /* SUPPORT_PM_GPIO_CONTROL */
    (void) i;
    (void) gpio;

    return 0;

of_err:
    pdata->reset_gpio = -EINVAL;
    pdata->irq_gpio   = -EINVAL;

    return -ENODEV;
}
#endif

#if defined (CONFIG_FB)
/* -------------------------------------------------------------------- */
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    struct cb_data *cbfp = NULL;
    cbfp = container_of(self, struct cb_data, fb_notifier);
    if (!cbfp)
        return 0;

    if (down_interruptible(&cbfp->mutex))
        return 0;

    if (evdata && evdata->data && event == FB_EVENT_BLANK && cbfp) {
        blank = (evdata->data);
        dev_dbg(&cbfp->pdev->dev, "%s  %d \n", __func__, *blank);

        if (*blank == FB_BLANK_UNBLANK) {
            // resume
        } else {
            // suspend
        }
    }
    up(&cbfp->mutex);

    return 0;
}
#endif

// 1. set handler to thread_fn

/* -------------------------------------------------------------------- */
static irqreturn_t cb_interrupt_handler(int irq, void * data)
{
    struct cb_data *cbfp = NULL;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
    if (!data)
        return IRQ_NONE;

    cbfp = (struct cb_data *) data;
    if (!cbfp)
        return IRQ_NONE;

    smp_rmb();
    wake_lock_timeout(&cbfp->fp_wake_lock, msecs_to_jiffies(CB_WAKE_LOCK_HOLD_TIME));

    cbfp->interrupt_done = 1;

#if !defined(IRQ_SYSFS)
    input_report_rel(cbfp->input, REL_MISC, 1);
    input_sync(cbfp->input);
#else
    sysfs_notify(&cbfp->pdev->dev.kobj, NULL, dev_attr_irq.attr.name);
#endif

    // IRQ_HANDLE or IRQ_WAKE_THREAD ?????
    return IRQ_HANDLED;
    //return IRQ_WAKE_THREAD;
}

/* -------------------------------------------------------------------- */
int cb_reset(struct cb_data *cbfp)
{
    u8 value = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    if (!gpio_is_valid(cbfp->reset_gpio)) {
        pr_debug(KERN_DEBUG "%s: RESET GPIO(%d) is INVALID\n", __func__, cbfp->reset_gpio);
        return 0;
    }

    value = gpio_get_value(cbfp->reset_gpio);
    pr_debug(KERN_DEBUG "%s: RESET GPIO(%d) state is %s\n", __func__, cbfp->reset_gpio, (value == 0) ? "low" : "high");
    if (value != 0) {
        gpio_set_value(cbfp->reset_gpio, 0);
        mdelay(1);
    }

    gpio_set_value(cbfp->reset_gpio, 1);
    mdelay(1);

    value = gpio_get_value(cbfp->reset_gpio);
    pr_debug(KERN_DEBUG "%s: FIANL RESET GPIO(%d) state is %s\n", __func__, cbfp->reset_gpio, (value == 0) ? "low" : "high");

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

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);

    rc = of_get_named_gpio(np, label, 0);
    if (rc < 0) {
        dev_err(dev, "[ERROR] %s: failed to get %s\n", __func__, label);
        return rc;
    }

    *gpio = rc;
    pr_debug(KERN_DEBUG ": [DEBUG] %s: Succeed to get gpio [%s(%d)], Trying to request gpio\n", __func__, label, rc);

    rc = devm_gpio_request(dev, *gpio, label);
    if (rc) {
        dev_err(dev, "[ERROR] %s: failed to request gpio %d\n", __func__, *gpio);
        return rc;
    }

    return 0;
}

#if 0
/* -------------------------------------------------------------------- */
static void cb_hw_reset(struct cb_data *cbfp, int delay)
{
    gpio_set_value(cbfp->reset_gpio, 1);
    mdelay(1);

    gpio_set_value(cbfp->reset_gpio, 0);
    //gpio_set_value(cbfp->reset_gpio, 1);
    mdelay(1);

    gpio_set_value(cbfp->reset_gpio, 1);

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
static int cb_reset_init(struct cb_data *cbfp, struct cb_platform_data *pdata)
{
    int rc = 0;

    rc = cb_request_named_gpio(cbfp, "fpreset-gpio", &pdata->reset_gpio);
    if (rc) {
        dev_err(&cbfp->pdev->dev,  "[ERROR] %s: failed to get named gpio(fpreset-gpio)\n", __func__);
        return rc;
    }

    cbfp->reset_gpio = pdata->reset_gpio;
    rc = gpio_direction_output(cbfp->reset_gpio, 1);
    if (rc) {
        dev_err(&cbfp->pdev->dev,  "[ERROR] %s, failed to set direction output to 1.\n", __func__);
        return rc;
    }

    //cb_hw_reset(cbfp, 1250);
    mdelay(1);
    gpio_set_value(cbfp->reset_gpio, 1);
    mdelay(1);

#if defined(SUPPORT_PIN_CTRL)
    select_pin_ctrl(cbfp, "cbfp_reset_active");
    mdelay(1);
    select_pin_ctrl(cbfp, "cbfp_reset_reset");
    mdelay(1);
    select_pin_ctrl(cbfp, "cbfp_reset_active");
    mdelay(2);
#endif /* SUPPORT_PIN_CTRL */

    return 0;
}

/* -------------------------------------------------------------------- */
static int cb_irq_init(struct cb_data *cbfp, struct cb_platform_data *pdata)
{
    int rc = 0;

    rc = cb_request_named_gpio(cbfp, "fpint-gpio", &pdata->irq_gpio);
    if (rc) {
        dev_err(&cbfp->pdev->dev,  "[ERROR] %s: failed to get named gpio(fpint-gpio)\n", __func__);
        return rc;
    }

    if (!gpio_is_valid(pdata->irq_gpio)) {
        dev_err(&cbfp->pdev->dev, "[ERROR] %s: GPIO IRQ (fpint-gpio) is INVALID\n", __func__);
    }

    cbfp->irq_gpio = pdata->irq_gpio;

    rc = gpio_direction_input(cbfp->irq_gpio);
    if (rc) {
        dev_err(&cbfp->pdev->dev, "[ERROR] %s: failed to set direction to input.\n", __func__);
        return rc;
    }

    cbfp->irq = gpio_to_irq(cbfp->irq_gpio);
    if (cbfp->irq < 0) {
        dev_err(&cbfp->pdev->dev, "[ERROR] %s, gpio_to_irq failed.\n", __func__);
        rc = cbfp->irq;
        return rc;
    }

    wake_lock_init(&cbfp->fp_wake_lock, WAKE_LOCK_SUSPEND, "fp_wake_lock");
    //OLD enable_irq_wake( gpio_to_irq( cbfp->irq ) );
    //enable_irq_wake(cbfp->irq);

#if defined(SUPPORT_PIN_CTRL)
    select_pin_ctrl(cbfp, "cbfp_irq_active");
#endif /* SUPPORT_PIN_CTRL */

    /*regist irq*/
#if 1
    rc = devm_request_threaded_irq(&cbfp->pdev->dev, cbfp->irq,
                                    NULL, cb_interrupt_handler,
                                    IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                                    dev_name(&cbfp->pdev->dev), cbfp);
#else
    rc = devm_request_irq(&cbfp->pdev->dev, cbfp->irq, cb_interrupt_handler,
                            IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&cbfp->pdev->dev), cbfp);
#endif

    if (rc) {
        dev_err(&cbfp->pdev->dev, "[ERROR] %s, devm_request_threaded_irq %i failed.\n", __func__, cbfp->irq);
        cbfp->irq = -EINVAL;
        return -EINVAL;
    }

    disable_irq(cbfp->irq);
    cbfp->interrupt_done = 0;
    enable_irq(cbfp->irq);
    enable_irq_wake(cbfp->irq);

    return rc;
}

/* -------------------------------------------------------------------- */
static int cb_manage_sysfs(struct cb_data *cbfp, struct platform_device *pdev, bool create)
{
    int rc = 0;

    if (create) {
        rc = sysfs_create_group(&cbfp->pdev->dev.kobj, &cb_hw_control_attr_group);
        if (rc) {
            dev_err(&cbfp->pdev->dev, " [DEBUG IX][ERROR] sysfs_create_group failed. %d\n", rc);
        }
    } else {
        sysfs_remove_group(&cbfp->pdev->dev.kobj, &cb_hw_control_attr_group);
    }

    return rc;
}

#if !defined (IRQ_SYSFS)
/* -------------------------------------------------------------------- */
static int cb_register_input_device(struct cb_data *cbfp, bool is_register)
{
    int rc = 0;

    if (is_register) {
        /* register input device */
        cbfp->input = input_allocate_device();
        if(!cbfp->input) {
            dev_err(&cbfp->pdev->dev, "input_allocate_deivce failed.");
            rc = -ENOMEM;
            goto err;
        }

        cbfp->input->name = "fingerprint";
        cbfp->input->dev.init_name = "cb_fingerprint";

        input_set_capability(cbfp->input, EV_REL, REL_MISC);
        input_set_drvdata(cbfp->input, cbfp);
        rc = input_register_device(cbfp->input);
        if (rc) {
            dev_err(&cbfp->pdev->dev, "input_register_device failed.");
            input_free_device(cbfp->input);
            cbfp->input = NULL;
            goto err;
        }
    } else {
        if (cbfp->input) {
            input_unregister_device(cbfp->input);
            input_free_device(cbfp->input);
        }
        cbfp->input = NULL;
    }
    return 0;

err:
    return -1;
}
#endif

#if defined(SUPPORT_PM_VREG_CONTROL)
/* -------------------------------------------------------------------- */
static int pm_vreg_release(struct regulator * vreg)
{
    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
    if (vreg != NULL) {
        regulator_put(vreg);
        vreg = NULL;
    }

    return 0;
}

static int pm_vreg_configure(struct cb_data *cbfp, const char * vreg_name, pm_vreg_t vreg_type, int min_uV, int max_uV)
{
    int rc = 0;
    struct regulator * vreg;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
    vreg = regulator_get(&cbfp->pdev->dev, vreg_name);
    if (IS_ERR(vreg)) {
        dev_err(&cbfp->pdev->dev, "pm_vreg_configure: Failed to get regulator\n");
        rc = PTR_ERR(vreg);
        goto exit;
    }

    if (regulator_count_voltages(vreg) > 0) {
        rc = regulator_set_voltage(vreg, min_uV, max_uV);
        if (rc) {
            dev_err(&cbfp->pdev->dev, "pm_vreg_configure: Failed to set volatage\n");
            goto exit;
        }
    }

    if (vreg_type == PM_VREG_VDD) {
        cbfp->vreg_vdd = vreg;
    } else if (vreg_type == PM_VREG_VDD_IO) {
        cbfp->vreg_vdd_io = vreg;
    }
    return 0;

exit:
    pm_vreg_release(vreg);
    return rc;
}

static int pm_vreg_enable(struct cb_data *cbfp, struct regulator * vreg, bool enable)
{
    int rc = 0;

    pr_debug(KERN_DEBUG ": [DEBUG] %s:%i %s\n", __FILE__, __LINE__, __func__);
    if (vreg == NULL) {
        dev_err(&cbfp->pdev->dev, "pm_vreg_enable: vreg is not configured yet\n");
        return -EINVAL;
    }

    if (enable) {
        rc = (regulator_is_enabled(vreg) == 0) ? regulator_enable(vreg) : 0;
        if (rc) {
            dev_err(&cbfp->pdev->dev, "pm_vreg_enable: Failed to enable regulator\n");
            goto exit;
        }
    } else {
        rc = (regulator_is_enabled(vreg) > 0) ? regulator_disable(vreg) : 0;
        if (rc) {
            dev_err(&cbfp->pdev->dev, "pm_vreg_enable: Failed to disable regulator\n");
            goto exit;
        }
    }

    return 0;

exit:
    pm_vreg_release(vreg);
    return rc;
}
#endif /* SUPPORT_PM_VREG_CONTROL */

#if defined(SUPPORT_PM_GPIO_CONTROL)
static u32 pm_gpio_configure(struct cb_data * cbfp, const char * label)
{
    int rc = 0;
    u32 gpio;

    rc = cb_request_named_gpio(cbfp, label, &gpio);
    if (rc) {
        dev_err(&cbfp->pdev->dev,  "[ERROR] %s: failed to get named gpio(%s)\n", __func__, label);
        return 0;
    }

    rc = gpio_direction_output(gpio, 1);
    if (rc) {
        dev_err(&cbfp->pdev->dev,  "[ERROR] %s, failed to set direction output to 1.\n", __func__);
        return 0;
    }

    mdelay(1);
    gpio_set_value(gpio, 1);
    mdelay(1);

    return gpio;
}
#endif /* SUPPORT_PM_GPIO_CONTROL */

#if defined(SUPPORT_PIN_CTRL)
static int select_pin_ctrl(struct cb_data *cbfp, const char * pin_name)
{
    int rc = 0;
    int i = 0;

    struct device *dev = &cbfp->pdev->dev;

    if (!dev) {
        dev_err(&cbfp->pdev->dev, "select_pin_ctrl: Device is NULL\n");
        return -EINVAL;
    }

    for (i=0 ; i<ARRAY_SIZE(cbfp->pinctrl_state) ; i++) {
        const char * n = pin_ctrl_names[i];
        if (!strncmp(n, pin_name, strlen(n))) {
            rc = pinctrl_select_state(cbfp->cbfp_pinctrl, cbfp->pinctrl_state[i]);
            if (rc) {
                dev_err(&cbfp->pdev->dev, "select_pin_ctrl: Cannot select pin [%s]\n", pin_name);
                break;
            } else {
                dev_err(&cbfp->pdev->dev, "select_pin_ctrl: Selected pin [%s]\n", pin_name);
                return 0;
            }
        }
    }
    dev_err(&cbfp->pdev->dev, "select_pin_ctrl: Cannot found pin [%s]\n", pin_name);
    rc = -EINVAL;

    return rc;
}
#endif /* SUPPORT_PIN_CTRL */

#if defined(CONFIG_THERMAL)
static int cb_init_thermal_control(struct cb_data *cbfp)
{
    struct device *dev = &cbfp->pdev->dev;
    struct device_node *np = dev->of_node;

    int rc = 0;

    cbfp->fp_need_notify = false;

    rc = of_property_read_string(np, "fp-thermal-zone", &cbfp->thermal_zone_name);
    if (rc) {
        dev_err(dev, "%s: Failed to get thermal zone name[%s]\n", __func__, "fp-thermal-zone");
        return -EINVAL;
    }

    rc = of_property_read_u32(np, "fp-poll-interval", &cbfp->fp_thermal_polling_interval);
    if (rc) {
        dev_err(dev, "%s: Failed to get polling interval from device tree\n", __func__);
        dev_err(dev, "%s: Set polling interval to 5 seconds as default\n", __func__);
        cbfp->fp_thermal_polling_interval = 5000;
    }

    rc = of_property_read_s32(np, "fp-hot-temp", &cbfp->fp_temp_thresh_hot);
    if (rc) {
        dev_err(dev, "%s: Failed to get shutdown temperature from device tree\n", __func__);
        dev_err(dev, "%s: Set shutdown temperature to 70 degree as default\n", __func__);
        cbfp->fp_temp_thresh_hot = 70;
    }

    rc = of_property_read_s32(np, "fp-cold-temp", &cbfp->fp_temp_thresh_normal);
    if (rc) {
        dev_err(dev, "%s: Failed to get normal temperature from device tree\n", __func__);
        dev_err(dev, "%s: Set normal temperature to 68 degree as default\n", __func__);
        cbfp->fp_temp_thresh_normal = 68;
    }

    rc = of_property_read_s32(np, "fp-high-v", &cbfp->fp_high_thr);
    if (rc) {
        dev_err(dev, "%s: Failed to get high threshold from device tree\n", __func__);
        dev_err(dev, "%s: Set high threshold to 345740 as default\n", __func__);
        cbfp->fp_high_thr = 345740;
    }

	rc = of_property_read_s32(np, "fp-low-v", &cbfp->fp_low_thr);
    if (rc) {
        dev_err(dev, "%s: Failed to get low threshold from device tree\n", __func__);
        dev_err(dev, "%s: Set low threshold to 292482 as default\n", __func__);
        cbfp->fp_low_thr = 292482;
    }

    dev_dbg(dev, "%s: Trying to get thermal_zone_device for fp[%s]\n", __func__, cbfp->thermal_zone_name);
    cbfp->tzd_fp = thermal_zone_get_zone_by_name(cbfp->thermal_zone_name);
    if (rc) {
        dev_err(dev, "%s: Failed to get thermal_zone_device(%s)\n", __func__, cbfp->thermal_zone_name);
        return PTR_ERR(cbfp->tzd_fp);
    }

    cbfp->fp_thermal_status = THERMAL_SENSOR_NORMAL;
    mutex_init(&cbfp->fp_thermal_complete);
    dev_dbg(dev, "%s: Trying to init monitor task for thermal control\n", __func__);
    dev_dbg(dev, "%s: Shutdown temperature : %d\n", __func__, cbfp->fp_temp_thresh_hot);
    dev_dbg(dev, "%s: Working  temperature : %d\n", __func__, cbfp->fp_temp_thresh_normal);
    dev_dbg(dev, "%s: Polling interval     : %d\n", __func__, cbfp->fp_thermal_polling_interval);

    cbfp->adc_tm_dev = qpnp_get_adc_tm(&cbfp->pdev->dev, "cbfp");
    if (IS_ERR(cbfp->adc_tm_dev)) {
        cbfp->adc_tm_dev = NULL;
        dev_err(dev,"%s: adc_tm property missing\n", __func__);
    }
    else {
        cbfp->adc_param.low_thr = cbfp->fp_low_thr;
        cbfp->adc_param.high_thr = cbfp->fp_high_thr;
        cbfp->adc_param.timer_interval3 = ADC_MEAS3_INTERVAL_1S;
        cbfp->adc_param.state_request = ADC_TM_WARM_THR_ENABLE;
        cbfp->adc_param.btm_ctx = cbfp;
        cbfp->adc_param.threshold_notification =
                cb_thermal_adc_notification;
        cbfp->adc_param.channel = VADC_AMUX_THM2_PU2;
        cbfp->state_temp = ADC_TM_COOL_STATE;
        rc = qpnp_adc_tm_channel_measure(cbfp->adc_tm_dev, &cbfp->adc_param);
        if (rc) {
            cbfp->adc_tm_dev = NULL;
            dev_err(dev,"%s: requesting ADC error\n", __func__);
        }
        else {
            dev_dbg(dev,"%s: Request adc notification sucesss\n", __func__);
        }
    }

    INIT_DELAYED_WORK(&cbfp->fp_thermal_monitor, cb_thermal_monitor);
    schedule_delayed_work(&cbfp->fp_thermal_monitor, msecs_to_jiffies(cbfp->fp_thermal_polling_interval));

    return 0;
}

static void cb_thermal_monitor(struct work_struct *work)
{
    struct cb_data *cbfp = (struct cb_data *) container_of(work, struct cb_data, fp_thermal_monitor.work);

    struct device *dev = &cbfp->pdev->dev;

    s32 current_temp = 0;
    int rc = 0;
    mutex_lock(&cbfp->fp_thermal_complete);
    rc = thermal_zone_get_temp(cbfp->tzd_fp, &current_temp);
    if (rc) {
        dev_err(dev, "%s: Failed to get temperature\n", __func__);
        goto schedule;
    }

    if ((current_temp >= cbfp->fp_temp_thresh_hot)||(cbfp->state_temp == ADC_TM_WARM_STATE)){
        dev_dbg(dev, "%s: fp temperature is too hot (%d)\n", __func__, current_temp);
        if (cbfp->fp_thermal_status == THERMAL_SENSOR_NORMAL) {
            dev_dbg(dev, "%s: Shutdown fp power supply\n", __func__);
            gpio_set_value(cbfp->fp_vdd_io_gpio, 0);
            mdelay(1);
            gpio_set_value(cbfp->fp_vdd_gpio, 0);

            cbfp->fp_thermal_status = THERMAL_SENSOR_TOO_HOT;
            sysfs_notify(&cbfp->pdev->dev.kobj, NULL, dev_attr_irq.attr.name);
        }
    } else if (current_temp <= cbfp->fp_temp_thresh_normal) {
        dev_dbg(dev, "%s: fp temperature is OK (%d)\n", __func__, current_temp);
        if (cbfp->fp_thermal_status == THERMAL_SENSOR_TOO_HOT) {
            dev_dbg(dev, "%s: Supplying fp power\n", __func__);
            gpio_set_value(cbfp->fp_vdd_io_gpio, 1);
            mdelay(1);
            gpio_set_value(cbfp->fp_vdd_gpio, 1);
            mdelay(1);

            cbfp->fp_thermal_status = THERMAL_SENSOR_NORMAL;

            cbfp->fp_need_notify = true;
            cb_reset(cbfp);
        }
    } else {
        dev_dbg(dev, "%s: Monitoring temperature between shutdown and supplying (%d)\n", __func__, current_temp);
    }

schedule:
    mutex_unlock(&cbfp->fp_thermal_complete);
    schedule_delayed_work(&cbfp->fp_thermal_monitor, msecs_to_jiffies(cbfp->fp_thermal_polling_interval));
}

static void cb_thermal_adc_notification(enum qpnp_tm_state state, void *ctx)
{
    struct cb_data *cbfp = ctx;
    struct device *dev = &cbfp->pdev->dev;

    if (cbfp->adc_tm_dev == NULL) {
        dev_err(dev,"%s: invalid adc tm dev\n", __func__);
        return;
    }

    if (state >= ADC_TM_STATE_NUM) {
        dev_err(dev,"%s: invalid state parameter %d\n", __func__, state);
        return;
    }
    cbfp->state_temp = state;
    dev_dbg(dev,"Warning!! state = %s\n", state == ADC_TM_WARM_STATE ? "hot" : "cold");

    wake_lock_timeout(&cbfp->fp_wake_lock, msecs_to_jiffies(CB_WAKE_LOCK_HOLD_TIME));
    cb_thermal_monitor(&cbfp->fp_thermal_monitor.work);
    if (state == ADC_TM_WARM_STATE ) {
        /* reset the adc status request */
        cbfp->adc_param.state_request = ADC_TM_COOL_THR_ENABLE;
    }
    else if (state == ADC_TM_COOL_STATE) {
        cbfp->adc_param.state_request = ADC_TM_WARM_THR_ENABLE;
    }
    if (qpnp_adc_tm_channel_measure(cbfp->adc_tm_dev, &cbfp->adc_param))
        dev_err(dev,"%s: request ADC error\n", __func__);
}


#endif /* CONFIG_THERMAL */
