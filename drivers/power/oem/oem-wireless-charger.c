/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "WCHG: %s: " fmt, __func__

#undef FEATURE_KYOCERA_EXT_CHG_DET

#include <linux/debugfs.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/platform_device.h>
#include <linux/pmic-voter.h>

#include "oem-wireless-i2c.h"

#define DEFAULT_WCHG_ICL		(900 * 1000)
#define DEFAULT_WCHG_MSEC		(500)
#define DEFAULT_CCCV_THRESH		(4340 * 1000)
#define DEFAULT_WARM_THRESH		(431)
#define DEFAULT_TEMP_HYSTERESIS	(20)
#define DEFAULT_TEMP			(600)

#define WCHG_REMOVAL_DELAY		(2 * 1000)

#define I2C_RETRIES			10
#define I2C_RETRY_DELAY		1

#define BPP_CHARGING			0x01
#define EPP_CHARGING			0x09

#define OEM_DET_WCHG_VOTER		"OEM_DET_WCHG_VOTER"
#define OEM_TEMP_WCHG_VOTER		"OEM_TEMP_WCHG_VOTER"

typedef enum {
	WCHG_UNPLG,
	WCHG_INIT,
	WCHG_STEPPING,
	WCHG_STABLE,
	WCHG_CV,
	WCHG_FULL,
} wchg_state_type;

enum enable_reason {
	REASON_USB = BIT(0),
	REASON_CAPACITY = BIT(1),
	REASON_THERM = BIT(2),
};

enum {
	CHGER_TYPE_UNKNOWN,
	CHGER_TYPE_BPP,
	CHGER_TYPE_EPP,
};

enum {
	ITERM_LIMIT_DISABLE = 0,
	ITERM_LIMIT_ENABLE = 1,
};

enum {
	CV_1300MA = 0,
	CV_1050MA = 1,
	CV_800MA = 2,
	CV_500MA = 3,
};

struct oem_wchg_chip {
	struct device			*dev;
	struct power_supply		*batt_psy;
	struct power_supply		*dc_psy;
	struct power_supply		*hkadc_psy;
	struct power_supply		*wchg_psy;
	struct mutex			wchg_ic_disabled_lock;
	struct mutex			wchg_disabled_lock;
	struct mutex			wchg_lock;
	struct delayed_work		wchg_work;
	struct delayed_work		wchg_removal_work;
	struct delayed_work		wchg_i2c_work;
	struct delayed_work		wchg_qfactor_work;
	struct delayed_work		wchg_init_work;
	struct delayed_work		wchg_temp_work;
	struct votable			*awake_votable;
	bool					resume_completed;
	bool					initialized;
	bool					wchg_dummy;
	int						wchg_nen;
	int						wchg_en;
	int						wchg_int_n;
	int						wchg_int_irq;
	int						wchg_det_n;
	int						wchg_det_irq;
	int						wchg_ic_disabled;
	int						wchg_disabled;
	int						wchg_present;
	int						wchg_psy_type;
	int						status;
	int						prev_status;
	int						health;
	int						prev_health;
	int						wchg_icl_default;
	int						wchg_target_icl;
	int						wchg_icl_ua;
	int						wchg_init_ua;
	int						wchg_init_msec;
	int						wchg_default_init_msec;
	int						wchg_dcin_delay_msec;
	int						wchg_bpp_init_msec;
	int						wchg_epp_init_msec;
	int						wchg_offchg_init_msec;
	int						wchg_step_ua;
	int						wchg_step_msec;
	int						wchg_step_init_ua;
	int						wchg_default_step_msec;
	int						wchg_epp_icl_ua;
	int						wchg_bpp_init_ua;
	int						wchg_epp_init_ua;
	int						wchg_epp_step_ua;
	int						wchg_bpp_step_msec;
	int						wchg_epp_step_msec;
	int						wchg_offchg_step_msec;
	int						wchg_bpp_step_init_ua;
	int						wchg_epp_step_init_ua;
	int						wchg_steble_msec;
	int						wchg_cccv_threshold;
	int						wchg_cccv_threshold_warm;
	int						wchg_jeita_warm_thresh;
	int						wchg_cv_ua;
	int						*wchg_epp_cv_ua;
	int						wchg_cv_status;
	int						wchg_cv_status_levels;
	int						wchg_full_ua;
	int						charger_type;
	int						wchg_temp_levels;
	int						*wchg_temp_thresh;
	int						*wchg_temp_ua;
	int						wchg_temp_hysteresis;
	int						wchg_stop_temp;
	int						wchg_rechg_temp;
	int						wchg_enabled_capacity;
	wchg_state_type			wchg_stat;
};

static struct oem_wchg_chip *the_chip;

static int oem_chg_is_off_charge(void)
{
	static int status = -1;
	if (status == -1) {
		status = (strstr(saved_command_line, "androidboot.mode=kccharger") != NULL);
	}
	return status;
}

static int oem_get_dc_current_max(struct oem_wchg_chip *chip)
{
	union power_supply_propval ret = {0,};
	int rc;

	rc = power_supply_get_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
	if (rc) {
		dev_err(chip->dev,
			"dc psy doesn't support reading prop %d rc = %d\n",
			POWER_SUPPLY_PROP_CURRENT_MAX, rc);
		return false;
	}

	return ret.intval;
}

static int oem_wireless_set_inhibit_en(struct oem_wchg_chip *chip, bool enable)
{
	const union power_supply_propval ret = {enable,};

	pr_debug("set to inhibit enable:%d\n", enable);

	if (chip->dc_psy)
		return power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_WCHG_INHIBIT_EN,
								&ret);

	return -ENXIO;
}

static int oem_wireless_set_wchg_dummy(struct oem_wchg_chip *chip, bool dummy)
{
	const union power_supply_propval ret = {dummy,};

	chip->wchg_dummy = dummy;
	if (chip->dc_psy)
		return power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_WCHG_DUMMY,
								&ret);

	return -ENXIO;
}

static int is_wireless_present(struct oem_wchg_chip *chip)
{
	int present = 0;

	if (!chip->initialized) {
		pr_err("not initialized.");
	} else {
		present = !gpio_get_value(chip->wchg_det_n);
	}

	return present;
}

static void oem_wireless_chg_set_wchg_nen_n(struct oem_wchg_chip *chip, bool value)
{
	gpio_set_value(chip->wchg_nen, value);

	return;
}

static int oem_wireless_ic_en(struct oem_wchg_chip *chip, bool enable, enum enable_reason reason)
{
	int rc = 0, wchg_ic_disabled;

	pr_debug("wchg %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->wchg_ic_disabled == 0 ? "enabled" : "disabled",
			chip->wchg_ic_disabled, enable, reason);

	mutex_lock(&chip->wchg_ic_disabled_lock);
	if (!enable)
		wchg_ic_disabled = chip->wchg_ic_disabled | reason;
	else
		wchg_ic_disabled = chip->wchg_ic_disabled & (~reason);

	/* avoid unnecessary gpio interactions if nothing changed */
	if (!!wchg_ic_disabled == !!chip->wchg_ic_disabled) {
		goto out;
	}

	oem_wireless_chg_set_wchg_nen_n(chip, !!wchg_ic_disabled);

	pr_debug("wchg charging %s, wchg_ic_disabled = %02x\n",
			wchg_ic_disabled == 0 ? "enabled" : "disabled",
			wchg_ic_disabled);
out:
	chip->wchg_ic_disabled = wchg_ic_disabled;
	mutex_unlock(&chip->wchg_ic_disabled_lock);
	return rc;
}

static void oem_wireless_chg_set_wchg_en_n(struct oem_wchg_chip *chip, bool value)
{
	gpio_set_value(chip->wchg_en, value);

	return;
}

static int oem_wireless_chg_en(struct oem_wchg_chip *chip, bool enable, enum enable_reason reason)
{
	int rc = 0, wchg_disabled;

	pr_debug("wchg %s, susp = %02x, en? = %d, reason = %02x\n",
			chip->wchg_disabled == 0 ? "enabled" : "disabled",
			chip->wchg_disabled, enable, reason);

	mutex_lock(&chip->wchg_disabled_lock);
	if (!enable)
		wchg_disabled = chip->wchg_disabled | reason;
	else
		wchg_disabled = chip->wchg_disabled & (~reason);

	/* avoid unnecessary gpio interactions if nothing changed */
	if (!!wchg_disabled == !!chip->wchg_disabled) {
		goto out;
	}

	oem_wireless_chg_set_wchg_en_n(chip, !!wchg_disabled);

	pr_debug("wchg charging %s, wchg_disabled = %02x\n",
			wchg_disabled == 0 ? "enabled" : "disabled",
			wchg_disabled);
out:
	chip->wchg_disabled = wchg_disabled;
	mutex_unlock(&chip->wchg_disabled_lock);
	return rc;
}

static int oem_wireless_chg_icl_step(struct oem_wchg_chip *chip)
{
	union power_supply_propval prop = {0, };
	int rc = 0;
	int icl_ua = oem_get_dc_current_max(chip);
	static int last_icl_ua = 0;

	if (chip->wchg_stat != WCHG_STEPPING) {
		chip->wchg_stat = WCHG_STEPPING;
		last_icl_ua = chip->wchg_init_ua;
		icl_ua = chip->wchg_step_init_ua;
		pr_err("step start icl. icl_ua:%d\n", icl_ua);
		goto setprop;
	}

	pr_debug("wchg step. now = %d wchg_target_icl = %d \n",
					icl_ua, chip->wchg_target_icl);

	if (icl_ua < chip->wchg_target_icl) {
		icl_ua = last_icl_ua + chip->wchg_step_ua;
		last_icl_ua = icl_ua;
		pr_debug("step up icl. icl_ua:%d\n", icl_ua);
	} else if (icl_ua > chip->wchg_target_icl) {
		icl_ua = chip->wchg_target_icl;
		last_icl_ua = icl_ua;
		pr_debug("step down icl. icl_ua:%d\n", icl_ua);
	} else {
		last_icl_ua = icl_ua;
		pr_debug("current icl = taget icl. Not stepping.\n");
		chip->wchg_stat = WCHG_STABLE;

		return rc;
	}

setprop:
	pr_debug("stepping wchg current to %d uA\n", icl_ua);
	prop.intval = icl_ua;
	rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);

	if (icl_ua == chip->wchg_target_icl) {
		pr_debug("current icl reached taget. finish stepping.\n");
		chip->wchg_stat = WCHG_STABLE;
	}

	return rc;
}

static int oem_wireless_chg_set_appropriate_icl(struct oem_wchg_chip *chip)
{
	int rc = 0;

	pr_info("setting wchg current %d uA\n", chip->wchg_target_icl);

	rc = oem_wireless_chg_icl_step(chip);

	return rc;
}

static int oem_check_cv_threshold(struct oem_wchg_chip *chip)
{
	union power_supply_propval prop = {0, };
	int rc;
	int cccv_threshold = chip->wchg_cccv_threshold;

	rc = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_TEMP, &prop);
	if (prop.intval >= chip->wchg_jeita_warm_thresh) {
		cccv_threshold = chip->wchg_cccv_threshold_warm;
	}

	rc = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	pr_debug("cccv_threshold:%d, VOLTAGE_NOW:%d\n", cccv_threshold, prop.intval);
	if (prop.intval >= cccv_threshold) {
		if (chip->charger_type == CHGER_TYPE_EPP && chip->wchg_cv_status_levels > 0) {
			pr_debug("wchg_cv_status:%d\n", chip->wchg_cv_status);
			switch(chip->wchg_cv_status) {
				case CV_1300MA:
				case CV_1050MA:
					chip->wchg_cv_status = chip->wchg_cv_status + 1;
					prop.intval = chip->wchg_epp_cv_ua[chip->wchg_cv_status];
					break;
				case CV_800MA:
					chip->wchg_cv_status = chip->wchg_cv_status + 1;
					prop.intval = chip->wchg_cv_ua;
					chip->wchg_stat = WCHG_CV;
					break;
				default:
					prop.intval = chip->wchg_cv_ua;
					break;
			}
			pr_debug("CV wchg current to %d uA\n", prop.intval);
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		} else {
			chip->wchg_stat = WCHG_CV;

			pr_debug("CV wchg current to %d uA\n", chip->wchg_cv_ua);
			prop.intval = chip->wchg_cv_ua;
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		}

		if (chip->charger_type == CHGER_TYPE_BPP) {
			prop.intval = ITERM_LIMIT_ENABLE;
			rc = power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_ITERM_LIMIT_ENABLE, &prop);
		}
	} else if (chip->wchg_stat == WCHG_CV) {
		if (chip->charger_type == CHGER_TYPE_EPP && chip->wchg_cv_status_levels > 0) {
			pr_debug("wchg_cv_status:%d\n", chip->wchg_cv_status);
			switch(chip->wchg_cv_status) {
				case CV_500MA:
				case CV_800MA:
					chip->wchg_cv_status = chip->wchg_cv_status - 1;
					prop.intval = chip->wchg_epp_cv_ua[chip->wchg_cv_status];
					break;
				case CV_1050MA:
					chip->wchg_cv_status = chip->wchg_cv_status - 1;
					prop.intval = chip->wchg_target_icl;
					chip->wchg_stat = WCHG_STABLE;
					break;
				default:
					prop.intval = chip->wchg_cv_ua;
					break;
			}
			pr_debug("target wchg current to %d uA\n", prop.intval);
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		} else {
			chip->wchg_stat = WCHG_STABLE;

			pr_debug("target wchg current to %d uA\n", chip->wchg_target_icl);
			prop.intval = chip->wchg_target_icl;
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		}

		prop.intval = ITERM_LIMIT_DISABLE;
		rc = power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_ITERM_LIMIT_ENABLE, &prop);
	}

	return rc;
}

static void oem_wireless_temp_work(struct work_struct *work)
{
	struct oem_wchg_chip *chip = container_of(work,
				struct oem_wchg_chip, wchg_temp_work.work);
	union power_supply_propval prop = {0, };
	int rc;
	int temp = 0;
	bool wchg_enabled = true;

	rc = power_supply_get_property(chip->hkadc_psy, POWER_SUPPLY_PROP_OEM_XO_THERM, &prop);
	temp = prop.intval;
	pr_debug("POWER_SUPPLY_PROP_OEM_XO_THERM:%d\n", temp);

	if (temp <= chip->wchg_rechg_temp) {
		rc = oem_wireless_chg_en(chip, wchg_enabled, REASON_THERM);
		pr_debug("wchg_enabled:%d, wchg_en_n is low\n", wchg_enabled);
		if (chip->awake_votable) {
			vote(chip->awake_votable, OEM_TEMP_WCHG_VOTER, false, 0);
		}
	} else {
		schedule_delayed_work(&chip->wchg_temp_work, msecs_to_jiffies(chip->wchg_steble_msec));
	}
}

static int oem_check_temp_threshold(struct oem_wchg_chip *chip)
{
	union power_supply_propval prop = {0, };
	int rc;
	int temp = 0;
	int i;
	int new_temp_status = 0;
	int new_temp_ua = 0;
	bool wchg_enabled = true;
	static int last_temp_status = 0;

	rc = power_supply_get_property(chip->hkadc_psy, POWER_SUPPLY_PROP_OEM_XO_THERM, &prop);
	temp = prop.intval;
	pr_debug("POWER_SUPPLY_PROP_OEM_XO_THERM:%d, last_temp_status:%d\n", temp, last_temp_status);
	for (i = (chip->wchg_temp_levels - 1); i >= 0; i--) {
		if (temp >= chip->wchg_temp_thresh[i]) {
			pr_debug("new_temp_status:%d\n", new_temp_status);
			new_temp_status = i;
			break;
		}
	}

	if (last_temp_status != new_temp_status) {
		if (last_temp_status > new_temp_status) {
			if (temp >= (chip->wchg_temp_thresh[new_temp_status+1] - chip->wchg_temp_hysteresis)) {
				pr_debug("hysteresis cancels the update of temp_status:%d\n", last_temp_status);
			} else {
				pr_debug("Updating temp_status:%d\n", last_temp_status);
				last_temp_status = new_temp_status;
			}
		} else {
			pr_debug("Updating temp_status:%d\n", last_temp_status);
			last_temp_status = new_temp_status;
		}
	}

	new_temp_ua = chip->wchg_temp_ua[last_temp_status];
	rc = power_supply_get_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (prop.intval > new_temp_ua) {
		pr_debug("Updating input_current_max:%d\n", new_temp_ua);
		prop.intval = new_temp_ua;
		rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	} else {
		if (chip->wchg_stat == WCHG_STABLE) {
			pr_debug("Updating input_current_max:%d\n", new_temp_ua);
			prop.intval = new_temp_ua;
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
		}
	}

	if (temp >= chip->wchg_stop_temp) {
		if (chip->awake_votable) {
			vote(chip->awake_votable, OEM_TEMP_WCHG_VOTER, true, 0);
		}
		wchg_enabled = false;
		rc = oem_wireless_chg_en(chip, wchg_enabled, REASON_THERM);
		pr_debug("wchg_enabled:%d, wchg_en_n is high\n", wchg_enabled);
		schedule_delayed_work(&chip->wchg_temp_work, msecs_to_jiffies(chip->wchg_steble_msec));
	}

	return rc;
}

static int oem_check_full(struct oem_wchg_chip *chip)
{
	union power_supply_propval prop = {0, };
	int rc;

	rc = power_supply_get_property(chip->batt_psy, POWER_SUPPLY_PROP_STATUS, &prop);
	pr_debug("POWER_SUPPLY_PROP_STATUS:%d\n", prop.intval);
	if (prop.intval == POWER_SUPPLY_STATUS_FULL) {
		chip->wchg_stat = WCHG_FULL;

		pr_debug("FULL wchg current to %d uA\n", chip->wchg_full_ua);
		prop.intval = chip->wchg_full_ua;
		rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	} else if (chip->wchg_stat == WCHG_FULL && prop.intval != POWER_SUPPLY_STATUS_FULL) {
		chip->wchg_stat = WCHG_CV;

		pr_debug("CV wchg current to %d uA\n", chip->wchg_cv_ua);
		prop.intval = chip->wchg_cv_ua;
		rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	}

	return rc;
}

static int oem_wireless_chg_wchg_detect(struct oem_wchg_chip *chip, bool insertion)
{
	int rc = 0;

	if (insertion && is_wireless_present(chip)) {
		chip->wchg_psy_type = POWER_SUPPLY_TYPE_WIRELESS;
	} else {
		chip->wchg_psy_type = POWER_SUPPLY_TYPE_UNKNOWN;
	}

	power_supply_changed(chip->wchg_psy);

	return rc;
}

static void oem_wireless_chg_init_process(struct oem_wchg_chip *chip)
{
	union power_supply_propval prop = {0, };
	int rc = 0;

	chip->wchg_stat = WCHG_INIT;

	prop.intval = chip->wchg_init_ua;
	rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	pr_debug("Initial setting wchg current %d uA, wchg_stat = %d \n", chip->wchg_init_ua, chip->wchg_stat);

	cancel_delayed_work(&chip->wchg_work);
	schedule_delayed_work(&chip->wchg_work, msecs_to_jiffies(chip->wchg_init_msec));
}

static void oem_wireless_init_work(struct work_struct *work)
{
	struct oem_wchg_chip *chip = container_of(work,
				struct oem_wchg_chip, wchg_init_work.work);
	int off_chg_status = oem_chg_is_off_charge();

	chip->wchg_init_ua = chip->wchg_bpp_init_ua;
	chip->wchg_target_icl = chip->wchg_icl_default;
	chip->wchg_init_msec = chip->wchg_default_init_msec;
	chip->wchg_step_msec = chip->wchg_default_step_msec;
	chip->wchg_step_init_ua = chip->wchg_bpp_init_ua + chip->wchg_step_ua;

	pr_debug("off_chg_status:%d\n", off_chg_status);

	if (off_chg_status) {
			pr_debug("Detection of OFF charging\n");
			chip->wchg_init_ua = chip->wchg_bpp_init_ua;
			chip->wchg_target_icl = chip->wchg_icl_default;
			chip->wchg_init_msec = chip->wchg_offchg_init_msec;
			chip->wchg_step_msec = chip->wchg_offchg_step_msec;
			chip->wchg_step_init_ua = chip->wchg_bpp_step_init_ua;
	} else if (chip->charger_type == CHGER_TYPE_EPP) {
			pr_debug("Detection of EPP charging\n");
			chip->wchg_init_ua = chip->wchg_epp_init_ua;
			chip->wchg_target_icl = chip->wchg_epp_icl_ua;
			chip->wchg_init_msec = chip->wchg_epp_init_msec;
			chip->wchg_step_ua = chip->wchg_epp_step_ua;
			chip->wchg_step_msec = chip->wchg_epp_step_msec;
			chip->wchg_step_init_ua = chip->wchg_epp_step_init_ua;
	} else if (chip->charger_type == CHGER_TYPE_BPP) {
			pr_debug("Detection of BPP charging\n");
			chip->wchg_init_ua = chip->wchg_bpp_init_ua;
			chip->wchg_target_icl = chip->wchg_icl_default;
			chip->wchg_init_msec = chip->wchg_bpp_init_msec;
			chip->wchg_step_msec = chip->wchg_bpp_step_msec;
			chip->wchg_step_init_ua = chip->wchg_bpp_step_init_ua;
	}

	pr_debug("wchg_init_msec:%d, wchg_step_msec:%d\n", chip->wchg_init_msec, chip->wchg_step_msec);

	oem_wireless_chg_wchg_detect(chip, true);
	oem_wireless_chg_init_process(chip);
}

static void dcin_det_handler(struct oem_wchg_chip *chip)
{
	int wireless_present = is_wireless_present(chip);

	pr_debug("dcin_det triggered. wchg pres = %d\n", wireless_present);

	if (wireless_present) {
		pr_debug("wchg insertion\n");
		if (!chip->awake_votable) {
			chip->awake_votable = find_votable("AWAKE");
		}
		if (chip->awake_votable) {
			vote(chip->awake_votable, OEM_DET_WCHG_VOTER, true, 0);
		}
		schedule_delayed_work(&chip->wchg_init_work, 0);
	} else {
		oem_wireless_set_wchg_dummy(chip, true);
		cancel_delayed_work_sync(&chip->wchg_removal_work);
		cancel_delayed_work_sync(&chip->wchg_init_work);
		schedule_delayed_work(&chip->wchg_removal_work, msecs_to_jiffies(WCHG_REMOVAL_DELAY));
	}
}

static void oem_wireless_qfactor_write_work(struct work_struct *work)
{
	int cnt = 0;
	int rc = 0;

	while(cnt < I2C_RETRIES) {
		rc = oem_qiic_qfactor_write();
		pr_debug("cnt:%d, rc=%x\n", cnt, rc);

		if (rc >= 0) {
			break;
		}
		cnt++;
		msleep(I2C_RETRY_DELAY);
	}
}
/*
static irqreturn_t wchg_int_handler(int irq, void *_chip)
{
	int wchg_int_val = gpio_get_value(the_chip->wchg_int_n);

	pr_debug("wchg_int triggered. wchg_int = %d irq = %d \n", wchg_int_val, irq);

	if (!wchg_int_val) {
		pr_debug("wchg_int_n detect\n");

		schedule_delayed_work(&the_chip->wchg_qfactor_work, 0);
	}

	return IRQ_HANDLED;
}
*/
static void oem_wireless_i2c_read_work(struct work_struct *work)
{
	struct oem_wchg_chip *chip = container_of(work,
				struct oem_wchg_chip, wchg_i2c_work.work);
	int cnt = 0;
	//int rc = 0;
	u8 reg_val = 0;

	chip->charger_type = CHGER_TYPE_BPP;

	while(cnt < I2C_RETRIES) {
		reg_val = oem_input_ctrl_state_read();
		pr_debug("cnt:%d, INPUT_CTRL_STATE:0x%X\n", cnt, reg_val);

		if (reg_val == EPP_CHARGING) {
			pr_debug("Detection of EPP charging\n");
			chip->charger_type = CHGER_TYPE_EPP;
			break;
		} else if (reg_val == BPP_CHARGING) {
			pr_debug("Detection of BPP charging\n");
			chip->charger_type = CHGER_TYPE_BPP;
			break;
		}
		cnt++;
		msleep(I2C_RETRY_DELAY);
		}

	//rc = oem_qiic_param_write(chip->charger_type);
}

static irqreturn_t wchg_det_handler(int irq, void *_chip)
{
	int wchg_det_val = gpio_get_value(the_chip->wchg_det_n);

	pr_debug("wchg_det triggered. wchg_det = %d irq = %d \n", wchg_det_val, irq);

	if (wchg_det_val) {
		pr_debug("wchg_det_n detect\n");
		schedule_delayed_work(&the_chip->wchg_i2c_work, 0);
	}

	return IRQ_HANDLED;
}

static int oem_status_change(struct oem_wchg_chip *chip, int status)
{
	int rc = 0;
	int wireless_present = is_wireless_present(chip);
	static bool last_inhibit = false;

	if (status == POWER_SUPPLY_STATUS_FULL && wireless_present) {
		rc = oem_wireless_set_inhibit_en(chip, true);
		if (rc >= 0) {
			last_inhibit = true;
		}
	} else if (last_inhibit) {
		rc = oem_wireless_set_inhibit_en(chip, false);
		if (rc >= 0) {
			last_inhibit = false;
		}
	}

	return rc;
}

static void oem_wireless_chg_work(struct work_struct *work)
{
	struct oem_wchg_chip *chip = container_of(work,
				struct oem_wchg_chip, wchg_work.work);
	int rc = 0;
	int wireless_present = is_wireless_present(chip);

	pr_debug("wchg work. wchg pres = %d wchg_stat = %d \n",
					wireless_present, chip->wchg_stat);

	if(!wireless_present) {
		pr_debug("wchg is unplugged. wchg_stat = %d \n", chip->wchg_stat);
		return;
	}

	mutex_lock(&chip->wchg_lock);
	switch(chip->wchg_stat) {
		case WCHG_INIT:
		case WCHG_STEPPING:
			rc = oem_wireless_chg_set_appropriate_icl(chip);
			pr_debug("wchg_stat = %d \n", chip->wchg_stat);
			break;
		case WCHG_STABLE:
			rc = oem_check_cv_threshold(chip);
			rc = oem_check_temp_threshold(chip);
		case WCHG_CV:
		case WCHG_FULL:
			rc = oem_check_full(chip);
			pr_debug("wchg_stat = %d \n", chip->wchg_stat);
			break;
		case WCHG_UNPLG:
			pr_debug("wchg_stat = %d \n", chip->wchg_stat);
			break;
		default:
			break;
	}

	if (chip->wchg_stat == WCHG_STEPPING) {
		mutex_unlock(&chip->wchg_lock);
		schedule_delayed_work(&chip->wchg_work, msecs_to_jiffies(chip->wchg_step_msec));
	} else if (chip->wchg_stat >= WCHG_STABLE) {
		mutex_unlock(&chip->wchg_lock);
		schedule_delayed_work(&chip->wchg_work, msecs_to_jiffies(chip->wchg_steble_msec));
	} else {
		mutex_unlock(&chip->wchg_lock);
	}

	return;
}

static void oem_wireless_chg_removal_work(struct work_struct *work)
{
	struct oem_wchg_chip *chip = container_of(work,
				struct oem_wchg_chip, wchg_removal_work.work);
	union power_supply_propval prop = {0, };
	int rc;
	int wireless_present = is_wireless_present(chip);

	pr_debug("WCHG removal work wireless_present = %d \n", wireless_present);

	rc = oem_wireless_set_wchg_dummy(chip, false);

	if(wireless_present) {
		pr_debug("wchg is inserted\n");
		return;
	}

	chip->wchg_stat = WCHG_UNPLG;
	chip->wchg_cv_status = CV_1300MA;
	oem_wireless_chg_wchg_detect(chip, false);

	chip->charger_type = CHGER_TYPE_UNKNOWN;

	prop.intval = ITERM_LIMIT_DISABLE;
	rc = power_supply_set_property(chip->batt_psy, POWER_SUPPLY_PROP_ITERM_LIMIT_ENABLE, &prop);

	if (!chip->awake_votable) {
		chip->awake_votable = find_votable("AWAKE");
	}
	if (chip->awake_votable) {
		vote(chip->awake_votable, OEM_DET_WCHG_VOTER, false, 0);
	}

	pr_debug("wchg remove\n");
}

static enum power_supply_property oem_wireless_chg_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_WCHG_IC_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_REAL_TYPE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_INPUT_CURRENT_MAX,
	POWER_SUPPLY_PROP_WCHG_ICL_MAX,
	POWER_SUPPLY_PROP_WCHG_TYPE,
	POWER_SUPPLY_PROP_WCHG_ENABLED_CAPACITY,
};

static int oem_wireless_chg_set_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       const union power_supply_propval *val)
{
	union power_supply_propval ret = {0, };
	int rc = 0;
	int icl_ua = 0;
	struct oem_wchg_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		dcin_det_handler(chip);
		break;
	case POWER_SUPPLY_PROP_WCHG_IC_ENABLED:
		rc = oem_wireless_ic_en(chip, val->intval, REASON_USB);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = oem_wireless_chg_en(chip, val->intval, REASON_USB);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		chip->prev_status = chip->status;
		chip->status = val->intval;
		pr_debug("chip->status = %d, chip->prev_status = %d \n",
						chip->status, chip->prev_status);
		if (chip->status != chip->prev_status) {
			rc = oem_status_change(chip, chip->status);
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		chip->prev_health = chip->health;
		chip->health = val->intval;
		pr_debug("chip->health = %d, chip->prev_health = %d \n",
						chip->health, chip->prev_health);
		break;
	case POWER_SUPPLY_PROP_WCHG_ICL_MAX:
		icl_ua = oem_get_dc_current_max(chip);
		chip->wchg_icl_ua = val->intval;
		if ((chip->wchg_icl_ua < icl_ua) && (chip->wchg_icl_ua)){
			/* Need immediate setting */
			ret.intval = chip->wchg_icl_ua;
			rc = power_supply_set_property(chip->dc_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &ret);
		}
		break;
	case POWER_SUPPLY_PROP_WCHG_ENABLED_CAPACITY:
		chip->wchg_enabled_capacity = val->intval;
		rc = oem_wireless_chg_en(chip, chip->wchg_enabled_capacity, REASON_CAPACITY);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int oem_wireless_chg_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct oem_wchg_chip *chip = power_supply_get_drvdata(psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_wireless_present(chip);
		break;
	case POWER_SUPPLY_PROP_WCHG_IC_ENABLED:
		val->intval = chip->wchg_ic_disabled == 0;
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = chip->wchg_disabled == 0;
		break;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = POWER_SUPPLY_TYPE_WIRELESS;
		break;
	case POWER_SUPPLY_PROP_REAL_TYPE:
		val->intval = chip->wchg_psy_type;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_MAX:
		val->intval = chip->wchg_target_icl;
		break;
	case POWER_SUPPLY_PROP_WCHG_ICL_MAX:
		val->intval = chip->wchg_icl_ua;
		break;
	case POWER_SUPPLY_PROP_WCHG_TYPE:
		val->intval = chip->charger_type;
		break;
	case POWER_SUPPLY_PROP_WCHG_ENABLED_CAPACITY:
		val->intval = chip->wchg_enabled_capacity;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int oem_wireless_chg_is_writeable(struct power_supply *psy,
				       enum power_supply_property prop)
{
	int rc;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_WCHG_IC_ENABLED:
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_WCHG_ICL_MAX:
	case POWER_SUPPLY_PROP_WCHG_ENABLED_CAPACITY:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static int oem_wireless_chg_request_irqs(struct oem_wchg_chip *chip)
{
	int rc = 0;
/*
	rc = request_irq(chip->wchg_int_irq, wchg_int_handler, IRQ_TYPE_EDGE_BOTH, "wchg_int", NULL);
	if (rc < 0) {
		pr_err("Unable to request wchg_det irq: %d\n", rc);
		return -ENXIO;
	}
	rc = enable_irq_wake(chip->wchg_int_irq);
	if (rc < 0) {
		pr_err("Unable to wchg_int irq wake: %d\n", rc);
		return -ENXIO;
	}
*/
	rc = request_irq(chip->wchg_det_irq, wchg_det_handler, IRQ_TYPE_EDGE_BOTH, "wchg_det", NULL);
	if (rc < 0) {
		pr_err("Unable to request wchg_det irq: %d\n", rc);
		return -ENXIO;
	}
	rc = enable_irq_wake(chip->wchg_det_irq);
	if (rc < 0) {
		pr_err("Unable to wchg_det irq wake: %d\n", rc);
		return -ENXIO;
	}

	return rc;
}

static int oem_wireless_chg_hw_init(struct oem_wchg_chip *chip)
{
	int rc = 0;

	return rc;
}

static int oem_wireless_chg_parse_dt(struct oem_wchg_chip *chip)
{
	int rc;
	int byte_len;
	struct device_node *node = chip->dev->of_node;

	if (!node) {
		dev_err(chip->dev, "device tree info. missing\n");
		return -EINVAL;
	}

	chip->wchg_nen = of_get_named_gpio(node, "oem,wchg-nen", 0);
	if (chip->wchg_nen < 0) {
		dev_err(chip->dev, "oem,wchg_nen can't get.\n");
		rc = -EINVAL;
		return rc;
	}
	gpio_request(chip->wchg_nen, "wchg-nen");

	chip->wchg_en = of_get_named_gpio(node, "oem,wchg-en", 0);
	if (chip->wchg_en < 0) {
		dev_err(chip->dev, "oem,wchg_en can't get.\n");
		rc = -EINVAL;
		return rc;
	}
	gpio_request(chip->wchg_en, "wchg-en");
/*
	chip->wchg_int_n = of_get_named_gpio(node, "oem,wchg-int-n", 0);
	if (chip->wchg_int_n < 0) {
		pr_err("oem,wchg-int-n can't get.\n");
		return -EINVAL;
	}
	chip->wchg_int_irq = gpio_to_irq(chip->wchg_int_n);
*/
	chip->wchg_det_n = of_get_named_gpio(node, "oem,wchg-det-n", 0);
	if (chip->wchg_det_n < 0) {
		pr_err("oem,wchg-det-n can't get.\n");
		return -EINVAL;
	}
	chip->wchg_det_irq = gpio_to_irq(chip->wchg_det_n);

	rc = of_property_read_u32(node, "oem,wchg-icl-ua",
			&chip->wchg_icl_default);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-icl-ua can't get.\n");
		chip->wchg_icl_default = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-init-ua",
			&chip->wchg_init_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-init-ua can't get.\n");
		chip->wchg_init_ua = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-init-msec",
			&chip->wchg_default_init_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-init-msec can't get.\n");
		chip->wchg_default_init_msec = DEFAULT_WCHG_MSEC;
	}
	chip->wchg_init_msec = chip->wchg_default_init_msec;

	rc = of_property_read_u32(node, "oem,dcin-delay-msec",
			&chip->wchg_dcin_delay_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,dcin-delay-msec can't get.\n");
		chip->wchg_dcin_delay_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-bpp-init-msec",
			&chip->wchg_bpp_init_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-bpp-init-msec can't get.\n");
		chip->wchg_bpp_init_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-epp-ua",
			&chip->wchg_epp_icl_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-ua can't get.\n");
		chip->wchg_epp_icl_ua = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-bpp-init-ua",
			&chip->wchg_bpp_init_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-bpp-init-ua can't get.\n");
		chip->wchg_bpp_init_ua = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-epp-init-ua",
			&chip->wchg_epp_init_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-init-ua can't get.\n");
		chip->wchg_epp_init_ua = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-epp-init-msec",
			&chip->wchg_epp_init_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-init-msec can't get.\n");
		chip->wchg_epp_init_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-offchg-init-msec",
			&chip->wchg_offchg_init_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-offchg-init-msec can't get.\n");
		chip->wchg_offchg_init_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-step-ua",
			&chip->wchg_step_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-step-ua can't get.\n");
		chip->wchg_step_ua = DEFAULT_WCHG_ICL;
	}

	rc = of_property_read_u32(node, "oem,wchg-step-msec",
			&chip->wchg_default_step_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-step-msec can't get.\n");
		chip->wchg_default_step_msec = DEFAULT_WCHG_MSEC;
	}
	chip->wchg_step_msec = chip->wchg_default_step_msec;

	rc = of_property_read_u32(node, "oem,wchg-epp-step-ua",
			&chip->wchg_epp_step_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-step-ua can't get.\n");
		chip->wchg_epp_step_ua = chip->wchg_step_ua;
	}

	rc = of_property_read_u32(node, "oem,wchg-bpp-step-msec",
			&chip->wchg_bpp_step_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-bpp-step-msec can't get.\n");
		chip->wchg_bpp_step_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-epp-step-msec",
			&chip->wchg_epp_step_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-step-msec can't get.\n");
		chip->wchg_epp_step_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-offchg-step-msec",
			&chip->wchg_offchg_step_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-offchg-step-msec can't get.\n");
		chip->wchg_offchg_step_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,wchg-bpp-step-init-ua",
			&chip->wchg_bpp_step_init_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-bpp-step-init-ua can't get.\n");
		chip->wchg_bpp_step_init_ua = chip->wchg_bpp_init_ua + chip->wchg_step_ua;
	}

	rc = of_property_read_u32(node, "oem,wchg-epp-step-init-ua",
			&chip->wchg_epp_step_init_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-epp-step-init-ua can't get.\n");
		chip->wchg_epp_step_init_ua = chip->wchg_epp_init_ua + chip->wchg_epp_step_ua;
	}

	rc = of_property_read_u32(node, "oem,wchg-stable-msec",
			&chip->wchg_steble_msec);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-stable-msec can't get.\n");
		chip->wchg_steble_msec = DEFAULT_WCHG_MSEC;
	}

	rc = of_property_read_u32(node, "oem,cc-cv-threshold-uv",
			&chip->wchg_cccv_threshold);
	if (rc < 0) {
		dev_err(chip->dev, "oem,cc-cv-threshold-uv can't get.\n");
		chip->wchg_cccv_threshold = DEFAULT_CCCV_THRESH;
	}

	rc = of_property_read_u32(node, "oem,cc-cv-threshold-warm-uv",
			&chip->wchg_cccv_threshold_warm);
	if (rc < 0) {
		dev_err(chip->dev, "oem,cc-cv-threshold-warm-uv can't get.\n");
		chip->wchg_cccv_threshold_warm = DEFAULT_CCCV_THRESH;
	}

	rc = of_property_read_u32(node, "oem,jeita-warm-thresh",
			&chip->wchg_jeita_warm_thresh);
	if (rc < 0) {
		dev_err(chip->dev, "oem,jeita-warm-thresh can't get.\n");
		chip->wchg_jeita_warm_thresh = DEFAULT_WARM_THRESH;
	}

	rc = of_property_read_u32(node, "oem,wchg-cv-ua",
			&chip->wchg_cv_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-cv-ua can't get.\n");
		chip->wchg_cv_ua = DEFAULT_WCHG_ICL;
	}

	if (of_find_property(node, "oem,wchg-epp-cv-ua", &byte_len)) {
		chip->wchg_epp_cv_ua = devm_kzalloc(chip->dev, byte_len,
			GFP_KERNEL);

		if (chip->wchg_epp_cv_ua == NULL) {
			pr_err("Couldn't read cycle_count_thresh rc = %d\n", rc);
			chip->wchg_cv_status_levels = -EINVAL;
		} else {
			chip->wchg_cv_status_levels = byte_len / sizeof(u32);
			rc = of_property_read_u32_array(node,
					"oem,wchg-epp-cv-ua",
					chip->wchg_epp_cv_ua,
					chip->wchg_cv_status_levels);
			if (rc < 0) {
				pr_err("Couldn't read epp cv ua rc = %d\n", rc);
				chip->wchg_cv_status_levels = -EINVAL;
			}
		}
	}

	if (of_find_property(node, "oem,wchg-temp-thresh", &byte_len)) {
		chip->wchg_temp_thresh = devm_kzalloc(chip->dev, byte_len,
			GFP_KERNEL);

		if (chip->wchg_temp_thresh == NULL) {
			pr_err("Couldn't read wchg_temp_thresh rc = %d\n", rc);
			chip->wchg_temp_levels = -EINVAL;
		} else {
			chip->wchg_temp_levels = byte_len / sizeof(u32);
			rc = of_property_read_u32_array(node,
					"oem,wchg-temp-thresh",
					chip->wchg_temp_thresh,
					chip->wchg_temp_levels);
			if (rc < 0) {
				pr_err("Couldn't read temp thresh rc = %d\n", rc);
				chip->wchg_temp_levels = -EINVAL;
			}
		}
	}

	if (of_find_property(node, "oem,wchg-temp-ua", &byte_len) &&
			chip->wchg_temp_levels != -EINVAL) {
		chip->wchg_temp_ua = devm_kzalloc(chip->dev, byte_len,
			GFP_KERNEL);

		if (chip->wchg_temp_ua == NULL) {
			pr_err("Couldn't read wchg_temp_ua rc = %d\n", rc);
			chip->wchg_temp_levels = -EINVAL;
		} else {
			chip->wchg_temp_levels = byte_len / sizeof(u32);
			rc = of_property_read_u32_array(node,
					"oem,wchg-temp-ua",
					chip->wchg_temp_ua,
					chip->wchg_temp_levels);
			if (rc < 0) {
				pr_err("Couldn't read temp ua rc = %d\n", rc);
				chip->wchg_temp_levels = -EINVAL;
			}
		}
	}

	rc = of_property_read_u32(node, "oem,wchg-temp-hysteresis",
			&chip->wchg_temp_hysteresis);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-temp-hysteresis can't get.\n");
		chip->wchg_temp_hysteresis = DEFAULT_TEMP_HYSTERESIS;
	}

	rc = of_property_read_u32(node, "oem,wchg-stop-temp",
			&chip->wchg_stop_temp);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-stop-temp can't get.\n");
		chip->wchg_stop_temp = DEFAULT_TEMP;
	}

	rc = of_property_read_u32(node, "oem,wchg-rechg-temp",
			&chip->wchg_rechg_temp);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-rechg-temp can't get.\n");
		chip->wchg_rechg_temp = DEFAULT_TEMP;
	}

	rc = of_property_read_u32(node, "oem,wchg-full-ua",
			&chip->wchg_full_ua);
	if (rc < 0) {
		dev_err(chip->dev, "oem,wchg-full-ua can't get.\n");
		chip->wchg_full_ua = DEFAULT_WCHG_ICL;
	}

	chip->wchg_target_icl = chip->wchg_icl_default;
	chip->wchg_icl_ua = chip->wchg_icl_default;

	return 0;
}

static const struct power_supply_desc wchg_psy_desc = {
	.name		= "wireless",
	.type		= POWER_SUPPLY_TYPE_WIRELESS,
	.get_property	= oem_wireless_chg_get_property,
	.set_property	= oem_wireless_chg_set_property,
	.property_is_writeable = oem_wireless_chg_is_writeable,
	.properties		= oem_wireless_chg_properties,
	.num_properties = ARRAY_SIZE(oem_wireless_chg_properties),
};

static int oem_wireless_chg_probe(struct platform_device *pdev)
{
	int rc;
	struct oem_wchg_chip *chip;
	struct power_supply_config wchg_psy_cfg = {};
	struct power_supply *batt_psy;
	struct power_supply *dc_psy;
	struct power_supply *hkadc_psy;

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		dev_dbg(&pdev->dev, "Battery supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	dc_psy = power_supply_get_by_name("dc");
	if (!dc_psy) {
		dev_dbg(&pdev->dev, "DC supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	hkadc_psy = power_supply_get_by_name("hkadc");
	if (!hkadc_psy) {
		dev_dbg(&pdev->dev, "HKADC supply not found; defer probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "Unable to allocate memory\n");
		return -ENOMEM;
	}

	chip->dev = &pdev->dev;
	chip->batt_psy = batt_psy;
	chip->dc_psy = dc_psy;
	chip->hkadc_psy = hkadc_psy;
	dev_set_drvdata(&pdev->dev, chip);

	chip->awake_votable = find_votable("AWAKE");
	if (chip->awake_votable == NULL) {
		dev_err(&pdev->dev, "Couldn't find AWAKE votable\n");
	}

	chip->resume_completed = true;

	the_chip = chip;

	mutex_init(&chip->wchg_ic_disabled_lock);
	mutex_init(&chip->wchg_disabled_lock);
	mutex_init(&chip->wchg_lock);

	INIT_DELAYED_WORK(&chip->wchg_work, oem_wireless_chg_work);
	INIT_DELAYED_WORK(&chip->wchg_removal_work, oem_wireless_chg_removal_work);
	INIT_DELAYED_WORK(&chip->wchg_i2c_work, oem_wireless_i2c_read_work);
	INIT_DELAYED_WORK(&chip->wchg_qfactor_work, oem_wireless_qfactor_write_work);
	INIT_DELAYED_WORK(&chip->wchg_init_work, oem_wireless_init_work);
	INIT_DELAYED_WORK(&chip->wchg_temp_work, oem_wireless_temp_work);

	rc = oem_wireless_chg_parse_dt(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to parse DT nodes\n");
		return rc;
	}

	rc = oem_wireless_chg_hw_init(chip);
	if (rc < 0) {
		dev_err(&pdev->dev,
			"Unable to intialize hardware rc = %d\n", rc);
		goto fail_init;
	}

	/* Register the power supply */
	wchg_psy_cfg.drv_data = chip;
	wchg_psy_cfg.of_node = chip->dev->of_node;
	chip->wchg_psy = devm_power_supply_register(chip->dev, &wchg_psy_desc,
			&wchg_psy_cfg);
	if (IS_ERR(chip->wchg_psy)) {
		pr_err("failed to register wchg_psy rc = %ld\n",
				PTR_ERR(chip->wchg_psy));
		goto fail_init;
	}

	rc = oem_wireless_chg_request_irqs(chip);
	if (rc < 0) {
		dev_err(&pdev->dev, "Unable to request irqs rc = %d\n", rc);
		goto fail_init;
	}

	chip->initialized = true;
	pr_info("oem_wireless_chg probe success! \n");

	if (is_wireless_present(chip)) {
		pr_debug("When OFF charging starts wirelessly\n");
		dcin_det_handler(chip);
	}

	return 0;

fail_init:
	pr_err("oem_wireless_chg probe failed! \n");
	return rc;
}

static int oem_wireless_chg_remove(struct platform_device *pdev)
{
	struct oem_wchg_chip *chip = dev_get_drvdata(&pdev->dev);

	cancel_delayed_work_sync(&chip->wchg_work);
	cancel_delayed_work_sync(&chip->wchg_removal_work);
	cancel_delayed_work_sync(&chip->wchg_i2c_work);
	cancel_delayed_work_sync(&chip->wchg_qfactor_work);
	cancel_delayed_work_sync(&chip->wchg_init_work);
	cancel_delayed_work_sync(&chip->wchg_temp_work);
	mutex_destroy(&chip->wchg_ic_disabled_lock);
	mutex_destroy(&chip->wchg_disabled_lock);
	mutex_destroy(&chip->wchg_lock);

	return 0;
}

static struct of_device_id oem_wireless_chg_match_table[] = {
	{ .compatible = "oem,wireless-charger",},
	{ },
};

static const struct platform_device_id oem_wireless_chg_id[] = {
	{"wireless-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(platform, oem_wireless_chg_id);

static struct platform_driver oem_wireless_chg_driver = {
	.driver		= {
		.name		= "oem_wireless_chg",
		.owner		= THIS_MODULE,
		.of_match_table	= oem_wireless_chg_match_table,
	},
	.probe		= oem_wireless_chg_probe,
	.remove		= oem_wireless_chg_remove,
	.id_table	= oem_wireless_chg_id,
};


static int __init smbchg_init(void)
{
	return platform_driver_register(&oem_wireless_chg_driver);
}

static void __exit smbchg_exit(void)
{
	return platform_driver_unregister(&oem_wireless_chg_driver);
}

module_init(smbchg_init);
module_exit(smbchg_exit);

MODULE_DESCRIPTION("OEM Wireless Charger");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("oem_wireless_chg");
