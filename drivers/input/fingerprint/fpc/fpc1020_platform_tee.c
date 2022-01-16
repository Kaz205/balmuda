/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */
/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#include <drm/drm_panel.h>
#endif

#define FPC_TTW_HOLD_TIME 1000

#define RESET_LOW_SLEEP_MIN_US 5000
#define RESET_LOW_SLEEP_MAX_US (RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US 100
#define RESET_HIGH_SLEEP1_MAX_US (RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US 5000
#define RESET_HIGH_SLEEP2_MAX_US (RESET_HIGH_SLEEP2_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US 100
#define PWR_ON_SLEEP_MAX_US (PWR_ON_SLEEP_MIN_US + 900)

#define NUM_PARAMS_REG_ENABLE_SET 2

#define RELEASE_WAKELOCK_W_V "release_wakelock_with_verification"
#define RELEASE_WAKELOCK "release_wakelock"
#define START_IRQS_RECEIVED_CNT "start_irqs_received_counter"

#define PWR_ON_SLEEP_US        10000
#define PWR_OFF_SLEEP_US       10000
#define RESET_ON_SLEEP_US       1300
#define RESET_OFF_SLEEP_US       500
#define DROP_IRQ_LOW_SLEEP_MS     10
#define DROP_IRQ_HIGH_SLEEP_MS    50
#define DROP_IRQ_TIMEOUT_MS     2000

static uint device_enabled = 0;
module_param(device_enabled, uint, S_IRUGO );
MODULE_PARM_DESC(device_enabled, "device_enabled");

static uint navi_enabled = 0;
module_param(navi_enabled, uint, S_IRUGO );
MODULE_PARM_DESC(navi_enabled, "navi_enabled");

static uint navi_hold_enabled = 0;
module_param(navi_hold_enabled, uint, S_IRUGO );
MODULE_PARM_DESC(navi_hold_enabled, "navi_hold_enabled");

static uint thermal_monitor_enabled = 0;
module_param(thermal_monitor_enabled, uint, S_IRUGO );
MODULE_PARM_DESC(thermal_monitor_enabled, "thermal_monitor_enabled");

static uint drop_det_enabled = 0;
module_param(drop_det_enabled, uint, S_IRUGO );
MODULE_PARM_DESC(drop_det_enabled, "drop_det_enabled");

static uint authenticate_mode = 0;
module_param(authenticate_mode, uint, S_IRUGO );
MODULE_PARM_DESC(authenticate_mode, "authenticate_mode");

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
	"fpc1020_pwr_off",
	"fpc1020_pwr_on",
};

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};

static const struct vreg_config const vreg_conf[] = {
	{ "vdd_ana", 1800000UL, 1800000UL, 6000, },
	{ "vcc_spi", 1800000UL, 1800000UL, 10, },
	{ "vdd_io", 1800000UL, 1800000UL, 6000, },
};

struct fpc1020_data {
	struct device *dev;

	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];
	struct regulator *vreg[ARRAY_SIZE(vreg_conf)];

	struct wakeup_source ttw_wl;
	int irq_gpio;
	int rst_gpio;

	int nbr_irqs_received;
	int nbr_irqs_received_counter_start;

	struct mutex lock; /* To set/get exported values in sysfs */
	bool prepared;
	atomic_t wakeup_enabled; /* Used both in ISR and non-ISR */

	int pwr_gpio;
	unsigned int fb_state;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	atomic_t drop_det_available;
	atomic_t drop_det_en;
	atomic_t drop_detected;
	int drop_irq_gpio;
	struct work_struct drop_work;
	struct pinctrl_state *drop_pinctrl_state;
};

#if defined(CONFIG_FB)
struct drm_panel *active_panel;
#endif

static irqreturn_t fpc1020_drop_irq_handler(int irq, void *handle);

static int vreg_setup(struct fpc1020_data *fpc1020, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->vreg); i++) {
		const char *n = vreg_conf[i].name;

		if (!strncmp(n, name, strlen(n)))
			goto found;
	}

	dev_err(dev, "Regulator %s not found\n", name);

	return -EINVAL;

found:
	vreg = fpc1020->vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (IS_ERR(vreg)) {
				dev_err(dev, "Unable to get %s\n", name);
				return PTR_ERR(vreg);
			}
		}

		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 4, 0))
		rc = regulator_set_load(vreg, vreg_conf[i].ua_load);
#else
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
#endif
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);

		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fpc1020->vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fpc1020->vreg[i] = NULL;
		}
		rc = 0;
	}

	return rc;
}

/**
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility. Only prints a debug print that it is
 * disabled.
 */
static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	dev_dbg(dev,
		"clk_enable sysfs node not enabled in platform driver\n");

	return count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}

	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);

exit:
	return rc;
}

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static ssize_t regulator_enable_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int pwr_gpio = -1;
	if(fpc1020->pwr_gpio > 0){
		pwr_gpio = gpio_get_value(fpc1020->pwr_gpio);
		dev_dbg(dev, "pwr_gpio %d\n", pwr_gpio);
	}

	return scnprintf(buf, PAGE_SIZE, "%i\n", pwr_gpio);
}

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op)) {
		dev_err(dev, "sscanf invalid param\n");
		return -EINVAL;
	}
	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else {
		dev_err(dev, "invalid value op = %c\n", op);
		return -EINVAL;
	}

	mutex_lock(&fpc1020->lock);
	rc = vreg_setup(fpc1020, name, enable);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IRUSR | S_IWUSR, regulator_enable_get, regulator_enable_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;
	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");

	if (rc) {
		dev_err(dev, "fpc1020_reset_active error1\n");
		goto exit;
	}
	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc) {
		dev_err(dev, "fpc1020_reset_reset error\n");
		goto exit;
	}
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc) {
		dev_err(dev, "fpc1020_reset_active error2\n");
		goto exit;
	}
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);

exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		rc = hw_reset(fpc1020);
		mutex_unlock(&fpc1020->lock);
	} else {
		dev_err(dev, "buf is not 'reset'\n");
		return -EINVAL;
	}

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc;
	int irq_gpio;
	int drop_irq_gpio = 0;
	int i, j;
	struct device *dev = fpc1020->dev;

	dev_info(dev, "%s prepared=%d, enable=%d\n", __func__, fpc1020->prepared, enable);

	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		if (atomic_read(&fpc1020->drop_det_available)) {
			atomic_set(&fpc1020->drop_det_en, 0);
		}

		select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		for (i = 0; i < 3; i++) {
			rc = select_pin_ctl(fpc1020, "fpc1020_pwr_on");
			if (rc) {
				dev_err(dev, "fpc1020_pwr_on error\n");
				goto exit;
			}

			if (atomic_read(&fpc1020->drop_det_available)) {
				msleep(DROP_IRQ_LOW_SLEEP_MS);
				drop_irq_gpio = gpio_get_value(fpc1020->drop_irq_gpio);
				if (drop_irq_gpio == 1) {
					dev_err(dev, "DROP IRQ Low ng\n");
					select_pin_ctl(fpc1020, "fpc1020_pwr_off");
					rc = -EIO;
					goto exit;
				}
				for (j = 0; j * DROP_IRQ_HIGH_SLEEP_MS < DROP_IRQ_TIMEOUT_MS; j++) {
					drop_irq_gpio = gpio_get_value(fpc1020->drop_irq_gpio);
					dev_dbg(dev, "DROP IRQ after power on%d\n", drop_irq_gpio);
					if (drop_irq_gpio == 1) {
						dev_info(dev, "DROP IRQ ok [%d]\n", j);
						break;
					}
					msleep(DROP_IRQ_HIGH_SLEEP_MS);
				}
				if (drop_irq_gpio == 0) {
					dev_err(dev, "DROP IRQ High ng\n");
					select_pin_ctl(fpc1020, "fpc1020_pwr_off");
					rc = -EIO;
					goto exit;
				}
			}

			usleep_range(PWR_ON_SLEEP_US, PWR_ON_SLEEP_US);

			/* As we can't control chip select here the other part of the
			 * sensor driver eg. the TEE driver needs to do a _SOFT_ reset
			 * on the sensor after power up to be sure that the sensor is
			 * in a good state after power up. Okeyed by ASIC. */

			(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");

			usleep_range(RESET_ON_SLEEP_US, RESET_ON_SLEEP_US);

			irq_gpio = gpio_get_value(fpc1020->irq_gpio);
			dev_info(dev, "IRQ after reset %d\n", irq_gpio);

			if (irq_gpio == 1) {
				if (atomic_read(&fpc1020->drop_det_available)) {
					atomic_set(&fpc1020->drop_det_en, 1);
					atomic_set(&fpc1020->drop_detected, 0);
				}
				fpc1020->prepared = true;
				rc = 0;
				break;
			}
			else {
				(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");

				usleep_range(RESET_OFF_SLEEP_US, RESET_OFF_SLEEP_US);

				select_pin_ctl(fpc1020, "fpc1020_pwr_off");

				usleep_range(PWR_OFF_SLEEP_US, PWR_OFF_SLEEP_US);

				dev_info(dev, "IRQ Fail -> retry %d\n", i);
				fpc1020->prepared = false;
				rc = -EIO;
			}
		}

	} else if (!enable && fpc1020->prepared) {
		if (atomic_read(&fpc1020->drop_det_available)) {
			atomic_set(&fpc1020->drop_det_en, 0);
		}

		rc = 0;
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(RESET_OFF_SLEEP_US, RESET_OFF_SLEEP_US);

		select_pin_ctl(fpc1020, "fpc1020_pwr_off");

		usleep_range(PWR_OFF_SLEEP_US, PWR_OFF_SLEEP_US);
exit:
		fpc1020->prepared = false;
	} else {
		rc = 0;
	}
	mutex_unlock(&fpc1020->lock);

	return rc;
}

/**
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else {
		dev_err(dev, "%s:buf is invalid value\n", __func__);
		return -EINVAL;
	}

	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, "enable", strlen("enable")))
		atomic_set(&fpc1020->wakeup_enabled, 1);
	else if (!strncmp(buf, "disable", strlen("disable")))
		atomic_set(&fpc1020->wakeup_enabled, 0);
	else {
		dev_err(dev, "%s:buf is invalid value\n", __func__);
		ret = -EINVAL;
	}
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysfs node for controlling the wakelock.
 */
static ssize_t handle_wakelock_cmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, RELEASE_WAKELOCK_W_V,
		min(count, strlen(RELEASE_WAKELOCK_W_V)))) {
		if (fpc1020->nbr_irqs_received_counter_start ==
				fpc1020->nbr_irqs_received) {
			__pm_relax(&fpc1020->ttw_wl);
		} else {
			dev_dbg(dev, "Ignore releasing of wakelock %d != %d",
				fpc1020->nbr_irqs_received_counter_start,
				fpc1020->nbr_irqs_received);
		}
	} else if (!strncmp(buf, RELEASE_WAKELOCK, min(count,
				strlen(RELEASE_WAKELOCK)))) {
		__pm_relax(&fpc1020->ttw_wl);
	} else if (!strncmp(buf, START_IRQS_RECEIVED_CNT,
			min(count, strlen(START_IRQS_RECEIVED_CNT)))) {
		fpc1020->nbr_irqs_received_counter_start =
		fpc1020->nbr_irqs_received;
	} else {
		dev_err(dev, "%s:buf is invalid value\n", __func__);
		ret = -EINVAL;
	}
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(handle_wakelock, S_IWUSR, NULL, handle_wakelock_cmd);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

static ssize_t fb_state_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%i\n", fpc1020->fb_state);
}

static ssize_t fb_state_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(fb_state, S_IRUSR | S_IWUSR, fb_state_show, fb_state_store);

static ssize_t drop_detected_show(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%i\n", atomic_read(&fpc1020->drop_detected));
}
static ssize_t drop_detected_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(drop_detected, S_IRUSR | S_IWUSR, drop_detected_show, drop_detected_store);

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_fb_state.attr,
	&dev_attr_drop_detected.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	mutex_lock(&fpc1020->lock);
	if (atomic_read(&fpc1020->wakeup_enabled)) {
		fpc1020->nbr_irqs_received++;
		__pm_wakeup_event(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}
	mutex_unlock(&fpc1020->lock);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static irqreturn_t fpc1020_drop_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	if (atomic_read(&fpc1020->drop_det_en) && !work_pending(&fpc1020->drop_work)) {
		schedule_work(&fpc1020->drop_work);
	}

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct drm_panel_notifier *evdata = data;
	struct fpc1020_data *fpc1020 = container_of(self, struct fpc1020_data, fb_notif);
	int transition = *(int *)evdata->data;
	int last_fb_state =-1;

	if (transition <= DRM_PANEL_BLANK_UNBLANK) {
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			dev_info(fpc1020->dev, "%s: DRM_PANEL_EARLY UNBLANK\n", __func__);
		}
		else if (event == DRM_PANEL_EVENT_BLANK) {
			dev_info(fpc1020->dev, "%s: DRM_PANEL_BLANK_UNBLANK\n", __func__);
		}
		fpc1020->fb_state = 0;
	} else if (transition == DRM_PANEL_BLANK_POWERDOWN) {
		if (event == DRM_PANEL_EARLY_EVENT_BLANK) {
			dev_info(fpc1020->dev, "%s: DRM_PANEL_EARLY POWERDOWN\n", __func__);
		}
		else if (event == DRM_PANEL_EVENT_BLANK) {
			dev_info(fpc1020->dev, "%s: DRM_PANEL POWERDOWN\n", __func__);
		}
		fpc1020->fb_state = 1;
	}
	if (fpc1020->fb_state != last_fb_state) {
		dev_dbg(fpc1020->dev, "%s: fb_state notify!\n", __func__);
		sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_fb_state.attr.name);
	}
	return 0;
}
#endif

static void drop_work_funk(struct work_struct *work)
{
	struct fpc1020_data *fpc1020 = container_of(work, struct fpc1020_data, drop_work);
	if (!atomic_read(&fpc1020->drop_detected)) {
		dev_info(fpc1020->dev, "%s drop detect! device off\n", __func__);
		atomic_set(&fpc1020->drop_detected, 1);
		device_prepare(fpc1020, false);
	}
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	size_t i;
	int irqf;
	int drop_irqf = 0;
	struct device_node *np = dev->of_node;
	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);

#if defined(CONFIG_FB)
	int count;
	struct device_node *node;
	struct drm_panel *panel;
#endif
	
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc) {
		dev_err(dev, "gpio_irq  not found\n");
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_pwr",
			&fpc1020->pwr_gpio);
	if (rc) {
		dev_info(dev, "pwr_gpio  not found\n");
		fpc1020->pwr_gpio = -1;
	}

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			fpc1020->fingerprint_pinctrl = NULL;
			rc = -EPROBE_DEFER;
			goto err_pinctrl_get;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto err_pinctrl_get;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto err_pinctrl_lookup;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc) {
		dev_err(dev, "select_pin_ctl fpc1020_reset_reset fail\n");
		goto err_pinctrl_lookup;
	}
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc) {
		dev_err(dev, "select_pin_ctl fpc1020_irq_active fail\n");
		goto err_pinctrl_lookup;
	}

	atomic_set(&fpc1020->wakeup_enabled, 0);

	mutex_init(&fpc1020->lock);

	wakeup_source_init(&fpc1020->ttw_wl, "fpc_ttw_wl");
	
#if defined(CONFIG_FB)
	fpc1020->fb_state = 0;
	count = of_count_phandle_with_args(np, "panel", NULL);
	
	for (i = 0; i < count; i++) {
		node = of_parse_phandle(np, "panel", i);
		panel = of_drm_find_panel(node);
		of_node_put(node);

		if (!IS_ERR(panel)) {
			active_panel = panel;
			dev_info(dev, " Got active panel\n");
		}
		else{
			dev_err(dev, "Failed to get DRM active panel : %d \n",__func__,i);
		}
	}

	if (active_panel) {
		fpc1020->fb_notif.notifier_call = fb_notifier_callback;
		rc = drm_panel_notifier_register(active_panel,&fpc1020->fb_notif);
		if (rc < 0) {
			dev_err(dev, "Failed to register fb notifier client\n",__func__);
		}
	}
	else{
		dev_err(dev, "DRM active panel is not vaild\n",__func__);
	}
#endif

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto err_sysfs_create;
	}

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		irqf |= IRQF_NO_SUSPEND;
		device_init_wakeup(dev, 1);
	}

	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto err_request_irq;
	}

	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	if (of_property_read_u32(dev->of_node, "kc,enable-navi", &navi_enabled)) {
		navi_enabled = 0;
	}
	dev_info(dev, "kc,enable-navi is %d\n", navi_enabled);

	if (of_property_read_u32(dev->of_node, "kc,enable-navi-hold", &navi_hold_enabled)) {
		navi_hold_enabled = 0;
	}
	dev_info(dev, "kc,enable-navi-hold is %d\n", navi_hold_enabled);

	if (of_property_read_u32(dev->of_node, "kc,enable-thermal-monitor", &thermal_monitor_enabled)) {
		thermal_monitor_enabled = 0;
	}
	dev_info(dev, "kc,enable-thermal-monitor is %d\n", thermal_monitor_enabled);

	if (of_property_read_u32(dev->of_node, "kc,authenticate-mode", &authenticate_mode)) {
		authenticate_mode = 0;
	}
	dev_info(dev, "kc,authenticate-mode is %d\n", authenticate_mode);

	if (of_property_read_u32(dev->of_node, "kc,enable-drop-det", &drop_det_enabled)) {
		drop_det_enabled = 0;
	}
	dev_info(dev, "kc,enable-drop-det is %d\n", drop_det_enabled);

	if (drop_det_enabled) {
		atomic_set(&fpc1020->drop_det_available, 1);
	}
	else {
		atomic_set(&fpc1020->drop_det_available, 0);
	}

	atomic_set(&fpc1020->drop_det_en, 0);
	atomic_set(&fpc1020->drop_detected, 0);
	if (atomic_read(&fpc1020->drop_det_available)) {
		rc = fpc1020_request_named_gpio(fpc1020, "kc,gpio_drop_irq",
				&fpc1020->drop_irq_gpio);
		if (rc) {
			dev_err(dev, "gpio_irq  not found\n");
			goto err_drop;
		}

		fpc1020->drop_pinctrl_state = pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, "fp_drop_irq_active");
		if (IS_ERR(fpc1020->drop_pinctrl_state)) {
			dev_err(dev, "cannot find 'fp_drop_irq_active'\n");
			rc = -EINVAL;
			goto err_drop;
		}
		rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl, fpc1020->drop_pinctrl_state);
		if (rc) {
			dev_err(dev, "cannot select fp_drop_irq_active\n");
			goto err_drop;
		}

		INIT_WORK(&fpc1020->drop_work, drop_work_funk);

		drop_irqf = IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND;
		rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->drop_irq_gpio),
				NULL, fpc1020_drop_irq_handler, drop_irqf,
				dev_name(dev), fpc1020);
		if (rc) {
			dev_err(dev, "could not request drop_irq %d\n", gpio_to_irq(fpc1020->drop_irq_gpio));
			goto err_drop;
		}
		dev_dbg(dev, "requested drop_irq %d\n", gpio_to_irq(fpc1020->drop_irq_gpio));

		device_init_wakeup(dev, 1);
		enable_irq_wake(gpio_to_irq(fpc1020->drop_irq_gpio));
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		(void)device_prepare(fpc1020, true);
	}

	device_enabled = 1;

	dev_info(dev, "%s: ok\n", __func__);

	return 0;

err_drop:
	devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
err_request_irq:
	device_init_wakeup(dev, 0);
	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	wakeup_source_trash(&fpc1020->ttw_wl);
	mutex_destroy(&fpc1020->lock);
err_sysfs_create:
	fb_unregister_client(&fpc1020->fb_notif);
err_pinctrl_lookup:
	devm_pinctrl_put(fpc1020->fingerprint_pinctrl);
	fpc1020->fingerprint_pinctrl = NULL;
err_pinctrl_get:
	devm_gpio_free(dev, fpc1020->irq_gpio);
	devm_gpio_free(dev, fpc1020->pwr_gpio);
exit:
	platform_set_drvdata(pdev, NULL);

	dev_info(dev, "%s: ng\n", __func__);

	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (fpc1020) {
		if (atomic_read(&fpc1020->drop_det_available)) {
			atomic_set(&fpc1020->drop_det_available, 0);
			if (atomic_read(&fpc1020->drop_det_en)) {
				atomic_set(&fpc1020->drop_det_en, 0);
				disable_irq(gpio_to_irq(fpc1020->drop_irq_gpio));
				disable_irq_wake(gpio_to_irq(fpc1020->drop_irq_gpio));
			}
			cancel_work_sync(&fpc1020->drop_work);
			devm_free_irq(dev, gpio_to_irq(fpc1020->drop_irq_gpio), fpc1020);
			devm_gpio_free(dev, fpc1020->drop_irq_gpio);
		}
		atomic_set(&fpc1020->wakeup_enabled, 0);
		disable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
		device_init_wakeup(dev, 0);
		sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
		devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
		wakeup_source_trash(&fpc1020->ttw_wl);
		mutex_destroy(&fpc1020->lock);
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		usleep_range(RESET_OFF_SLEEP_US, RESET_OFF_SLEEP_US);
		select_pin_ctl(fpc1020, "fpc1020_pwr_off");
		devm_pinctrl_put(fpc1020->fingerprint_pinctrl);
		fpc1020->fingerprint_pinctrl = NULL;
		devm_gpio_free(dev, fpc1020->irq_gpio);
		devm_gpio_free(dev, fpc1020->pwr_gpio);
		platform_set_drvdata(pdev, NULL);
#if defined(CONFIG_FB)
		fb_unregister_client(&fpc1020->fb_notif);
		drm_panel_notifier_unregister(active_panel,&fpc1020->fb_notif);
#endif
	}
	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);

	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);

	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}


late_initcall(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
