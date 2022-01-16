/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "HKADC: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/iio/consumer.h>
#include <linux/power_supply.h>
#include <linux/spmi.h>

#define DEFAULT_TEMP			(25 * 1000)
#define DEFAULT_TIME_MS			5000

#define USB_THERM_OVERHEAT	(80 * 1000)
#define USB_THERM_NORMAL	(50 * 1000)

static struct power_supply *batt_psy = NULL;

struct monitor_setting {
	struct iio_channel	*camera_therm_chan;
	struct iio_channel	*out_camera_therm_chan;
	struct iio_channel	*in_camera_therm_chan;
	struct iio_channel	*pa_therm_chan;
	struct iio_channel	*lcd_therm_chan;
	struct iio_channel	*usb_therm_chan;
	struct iio_channel	*xo_therm_chan;
	struct iio_channel	*module_5g_1_therm_chan;
	struct iio_channel	*module_5g_2_therm_chan;
	struct iio_channel	*module_5g_3_therm_chan;
	struct iio_channel	*skin_therm_chan;
	struct iio_channel	*charger_skin_therm_chan;
};

struct oem_hkadc_chip {
	struct device			*dev;
	struct power_supply		*hkadc_psy;
	unsigned int			hkadc_monitor_ms;
	unsigned int			hkadc_monitor_resume_ms;
	struct delayed_work		hkadc_monitor_work;
	struct monitor_setting	therm_setting;
};

static enum power_supply_property oem_hkadc_power_props[] = {
	POWER_SUPPLY_PROP_OEM_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_OUT_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_IN_CAMERA_THERM,
	POWER_SUPPLY_PROP_OEM_PA_THERM,
	POWER_SUPPLY_PROP_OEM_LCD_THERM,
	POWER_SUPPLY_PROP_OEM_USB_THERM,
	POWER_SUPPLY_PROP_OEM_XO_THERM,
	POWER_SUPPLY_PROP_OEM_5G_THERM_1,
	POWER_SUPPLY_PROP_OEM_5G_THERM_2,
	POWER_SUPPLY_PROP_OEM_5G_THERM_3,
	POWER_SUPPLY_PROP_OEM_SKIN_THERM,
	POWER_SUPPLY_PROP_OEM_CHARGER_SKIN_THERM,
};

enum usb_alert_info {
    USB_ALERT_BOOT,
    USB_ALERT_1,
    USB_ALERT_2,
};

static int get_prop_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.camera_therm_chan == NULL) {
		pr_debug("camera-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.camera_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read camera therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("camera therm therm %d\n", temp);

	return temp;
}

static int get_prop_out_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.out_camera_therm_chan == NULL) {
		pr_debug("outcamera-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.out_camera_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read out camera therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("out camera therm %d\n", temp);

	return temp;
}

static int get_prop_in_camera_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.in_camera_therm_chan == NULL) {
		pr_debug("incamera-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.in_camera_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read in camera therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("in camera therm %d\n", temp);

	return temp;
}

static int get_prop_pa_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.pa_therm_chan == NULL) {
		pr_debug("pa-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.pa_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read pa therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("pa therm %d\n", temp);

	return temp;
}

static int get_prop_lcd_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.lcd_therm_chan == NULL) {
		pr_debug("lcd-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.lcd_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read lcd therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("lcd therm %d\n", temp);

	return temp;
}

static int get_prop_usb_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;
	union power_supply_propval ret = {0,};
	static int usb_temp_level = USB_ALERT_BOOT;

	if (chip->therm_setting.usb_therm_chan == NULL) {
		pr_debug("usb-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.usb_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read usb therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("usb therm %d\n", temp);

	if (temp >= USB_THERM_OVERHEAT) {
		pr_debug("usb therm overheat\n");
		ret.intval = USB_ALERT_1;
	} else if (temp <= USB_THERM_NORMAL) {
		ret.intval = USB_ALERT_BOOT;
	} else {
		ret.intval = usb_temp_level;
	}
	if (usb_temp_level != ret.intval) {
		usb_temp_level = ret.intval;
		pr_debug("set property usb_temp_level:%d\n", usb_temp_level);
		rc = power_supply_set_property(batt_psy, POWER_SUPPLY_PROP_USB_TEMP_LEVEL, &ret);
	}

	return temp;
}

static int get_prop_xo_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.xo_therm_chan == NULL) {
		pr_debug("xo-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.xo_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read xo therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("xo therm %d\n", temp);

	return temp;
}

static int get_prop_5g_therm_1(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.module_5g_1_therm_chan == NULL) {
		pr_debug("5G-therm-1 channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.module_5g_1_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read 5G therm 1 rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("5G therm 1 %d\n", temp);

	return temp;
}

static int get_prop_5g_therm_2(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.module_5g_2_therm_chan == NULL) {
		pr_debug("5G-therm-2 channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.module_5g_2_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read 5G therm 2 rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("5G therm 2 %d\n", temp);

	return temp;
}

static int get_prop_5g_therm_3(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.module_5g_3_therm_chan == NULL) {
		pr_debug("5G-therm-3 channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.module_5g_3_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read 5G therm 3 rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("5G therm 3 %d\n", temp);

	return temp;
}

static int get_prop_skin_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.skin_therm_chan == NULL) {
		pr_debug("skin-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.skin_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read skin therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("skin therm %d\n", temp);

	return temp;
}

static int get_prop_charger_skin_therm(struct oem_hkadc_chip *chip)
{
	int rc = 0;
	int temp = DEFAULT_TEMP;

	if (chip->therm_setting.charger_skin_therm_chan == NULL) {
		pr_debug("charger-skin-therm channel unavailable\n");
		return DEFAULT_TEMP;
	}

	rc = iio_read_channel_processed(chip->therm_setting.charger_skin_therm_chan, &temp);
	if (rc < 0) {
		pr_err("Unable to read charger skin therm rc=%d\n", rc);
		return DEFAULT_TEMP;
	}
	pr_debug("charger skin therm %d\n", temp);

	return temp;
}

static int oem_hkadc_power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *pval)
{
	struct oem_hkadc_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		pval->intval = get_prop_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_OUT_CAMERA_THERM:
		pval->intval = get_prop_out_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_IN_CAMERA_THERM:
		pval->intval = get_prop_in_camera_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		pval->intval = get_prop_pa_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_LCD_THERM:
		pval->intval = get_prop_lcd_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		pval->intval = get_prop_usb_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_XO_THERM:
		pval->intval = get_prop_xo_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_5G_THERM_1:
		pval->intval = get_prop_5g_therm_1(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_5G_THERM_2:
		pval->intval = get_prop_5g_therm_2(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_5G_THERM_3:
		pval->intval = get_prop_5g_therm_3(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_SKIN_THERM:
		pval->intval = get_prop_skin_therm(chip);
		break;
	case POWER_SUPPLY_PROP_OEM_CHARGER_SKIN_THERM:
		pval->intval = get_prop_charger_skin_therm(chip);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void oem_hkadc_monitor(struct work_struct *work)
{
	struct oem_hkadc_chip *chip = container_of(work,
		struct oem_hkadc_chip, hkadc_monitor_work.work);

	power_supply_changed(chip->hkadc_psy);

	schedule_delayed_work(&chip->hkadc_monitor_work,
		msecs_to_jiffies(chip->hkadc_monitor_ms));
}

static int oem_hkadc_parse_dt(struct oem_hkadc_chip *chip)
{
	int rc;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	/* Normal monitor time ms */
	rc = of_property_read_u32(node, "oem,monitor-time-ms",
		&chip->hkadc_monitor_ms);
	if (rc < 0) {
		chip->hkadc_monitor_ms = DEFAULT_TIME_MS;
		pr_err("Missing required properties rc=%d\n", rc);
	}
	pr_info("chip->hkadc_monitor_ms=%d\n", chip->hkadc_monitor_ms);

	/* Resume monitor time ms */
	rc = of_property_read_u32(node, "oem,resume-mon-time-ms",
		&chip->hkadc_monitor_resume_ms);
	if (rc < 0) {
		chip->hkadc_monitor_resume_ms = DEFAULT_TIME_MS;
		pr_err("Missing required properties rc=%d\n", rc);
	}
	pr_info("chip->hkadc_monitor_resume_ms=%d\n", chip->hkadc_monitor_resume_ms);

	return 0;
}

static int oem_get_iio_channel(struct oem_hkadc_chip *chip, const char *propname,
					struct iio_channel **chan)
{
	int rc = 0;

	rc = of_property_match_string(chip->dev->of_node,
					"io-channel-names", propname);
	if (rc < 0)
		return 0;

	*chan = iio_channel_get(chip->dev, propname);
	if (IS_ERR(*chan)) {
		rc = PTR_ERR(*chan);
		if (rc != -EPROBE_DEFER)
			pr_err("%s channel unavailable, %d\n", propname, rc);
		*chan = NULL;
	}

	return rc;
}

static int oem_hkadc_parse_dt_adc_channels(struct oem_hkadc_chip *chip)
{
	int rc = 0;

	rc = oem_get_iio_channel(chip, "outcamera-therm",
					&chip->therm_setting.out_camera_therm_chan);
	if (rc < 0) {
		pr_err("outcamera-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "incamera-therm",
					&chip->therm_setting.in_camera_therm_chan);
	if (rc < 0) {
		pr_err("incamera-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "pa-therm",
					&chip->therm_setting.pa_therm_chan);
	if (rc < 0) {
		pr_err("pa-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "lcd-therm",
					&chip->therm_setting.lcd_therm_chan);
	if (rc < 0) {
		pr_err("lcd-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "conn-therm",
					&chip->therm_setting.usb_therm_chan);
	if (rc < 0) {
		pr_err("conn-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "xo-therm",
					&chip->therm_setting.xo_therm_chan);
	if (rc < 0) {
		pr_err("xo-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "5g-therm-1",
					&chip->therm_setting.module_5g_1_therm_chan);
	if (rc < 0) {
		pr_err("5g-therm-1 channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "5g-therm-2",
					&chip->therm_setting.module_5g_2_therm_chan);
	if (rc < 0) {
		pr_err("5g-therm-2 channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "5g-therm-3",
					&chip->therm_setting.module_5g_3_therm_chan);
	if (rc < 0) {
		pr_err("5g-therm-3 channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "skin-therm",
					&chip->therm_setting.skin_therm_chan);
	if (rc < 0) {
		pr_err("skin-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	rc = oem_get_iio_channel(chip, "charger-skin-therm",
					&chip->therm_setting.charger_skin_therm_chan);
	if (rc < 0) {
		pr_err("charger-skin-therm channel unavailable, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static const struct power_supply_desc oem_hkadc_psy_desc = {
	.name = "hkadc",
	.type = POWER_SUPPLY_TYPE_HKADC,
	.properties = oem_hkadc_power_props,
	.num_properties = ARRAY_SIZE(oem_hkadc_power_props),
	.get_property = oem_hkadc_power_get_property,
};

static int oem_hkadc_init_psy(struct oem_hkadc_chip *chip)
{
	struct power_supply_config oem_hkadc_cfg = {};
	int rc = 0;

	oem_hkadc_cfg.drv_data = chip;
	oem_hkadc_cfg.of_node = chip->dev->of_node;
	chip->hkadc_psy = power_supply_register(chip->dev, &oem_hkadc_psy_desc, &oem_hkadc_cfg);
	if (IS_ERR(chip->hkadc_psy)) {
		pr_err("Couldn't register oem hkadc power supply\n");
		return PTR_ERR(chip->hkadc_psy);
	}

	return rc;
}

static int oem_hkadc_probe(struct platform_device *pdev)
{
	int rc = 0;
	struct oem_hkadc_chip *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	rc = oem_hkadc_parse_dt_adc_channels(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse ADC channels\n");
		return rc;
	}

	rc = oem_hkadc_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery power supply not found deferring probe\n");
		return rc;
	}

	rc = oem_hkadc_init_psy(chip);
	if (rc < 0) {
		pr_err("Couldn't initialize oem hkadc psy rc=%d\n", rc);
		return rc;
	}

	INIT_DELAYED_WORK(&chip->hkadc_monitor_work, oem_hkadc_monitor);

	schedule_delayed_work(&chip->hkadc_monitor_work, 0);

	pr_info("probe success\n");

	return 0;
}

static int oem_hkadc_remove(struct platform_device *pdev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	power_supply_unregister(chip->hkadc_psy);
	dev_set_drvdata(&pdev->dev, NULL);
	return 0;
}

static int oem_hkadc_suspend(struct device *dev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&chip->hkadc_monitor_work);
	return 0;
}

static int oem_hkadc_resume(struct device *dev)
{
	struct oem_hkadc_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->hkadc_monitor_work,
		msecs_to_jiffies(chip->hkadc_monitor_resume_ms));
	return 0;
}

static const struct of_device_id oem_hkadc_of_match[] = {
	{ .compatible = "oem_hkadc-driver", },
	{},
};

static const struct dev_pm_ops oem_hkadc_pm_ops = {
	.resume = oem_hkadc_resume,
	.suspend = oem_hkadc_suspend,
};

static struct platform_driver oem_hkadc_driver = {
	.driver = {
		.name = "oem_hkadc-driver",
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_of_match,
		.pm = &oem_hkadc_pm_ops,
	},
	.probe = oem_hkadc_probe,
	.remove = oem_hkadc_remove,
};

static int __init oem_hkadc_init(void)
{
	return platform_driver_register(&oem_hkadc_driver);
}
module_init(oem_hkadc_init);

static void __exit oem_hkadc_exit(void)
{
	return platform_driver_unregister(&oem_hkadc_driver);
}
module_exit(oem_hkadc_exit);

MODULE_DESCRIPTION("oem_hkadc driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("oem_hkadc");
