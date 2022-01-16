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
#include <linux/i2c.h>

struct oem_wchg_chip {
	struct i2c_client		*i2c;
	struct device			*dev;
	bool					initialized;
};

static struct oem_wchg_chip *the_chip;

#define WIRELESS_I2C_NAME	"oem_wireless_i2c"

#define I2C_READ_MSG_NUM	(2)

#define INPUT_CTRL_STATE	0x4d

#define I2C_WRITE_MSG_NUM	(1)

typedef struct {
	u8 reg;  /* register */
	u8 data; /* write data */
} qiic_param;

qiic_param bpp_param [] = {
	{0x6F, 0x0C},
	{0x00, 0x00},	/* End of table */
};

qiic_param epp_param [] = {
	{0x6F, 0x11},
	{0x3E, 0x1D},
	{0x3F, 0x01},
	{0x00, 0x00},	/* End of table */
};

qiic_param qfactor_param [] = {
	{0x55, 0x3D},
	{0x00, 0x00},	/* End of table */
};

enum {
	CHGER_TYPE_UNKNOWN,
	CHGER_TYPE_BPP,
	CHGER_TYPE_EPP,
};

static int qi_i2c_read(unsigned char reg_addr, unsigned char *reg_data)
{
	struct i2c_msg msg[I2C_READ_MSG_NUM];
	int ret = -1;
	u8  reg [2] = { 0x00,reg_addr };
	u8  res [2] = { 0xFF,0xFF };

	msg[0].addr = the_chip->i2c->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = reg;
	msg[1].addr = the_chip->i2c->addr;
	msg[1].flags = 1;
	msg[1].len = 2;
	msg[1].buf = res;

	ret = i2c_transfer (the_chip->i2c->adapter, msg, 2);
	pr_info("qi_i2c_read ret=%d, result=%d\n", ret, res[0]);

	*reg_data = res[0];

	return ret;
}

int oem_input_ctrl_state_read(void)
{
	unsigned char reg_val = 0;
	int ret = 0;

	if (the_chip->initialized) {
		qi_i2c_read(INPUT_CTRL_STATE, &reg_val);
	}

	ret = reg_val & 0xFF;

	return ret;
}
EXPORT_SYMBOL(oem_input_ctrl_state_read);

static int qi_i2c_write(unsigned char reg_addr, unsigned char reg_data)
{
	struct i2c_msg msg[I2C_WRITE_MSG_NUM];
	int ret = -1;
	u8  reg [3] = { 0x00, reg_addr, reg_data };

	msg[0].addr = the_chip->i2c->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = reg;

	ret = i2c_transfer(the_chip->i2c->adapter, msg, 1);
	pr_info("qi_i2c_write ret=%d\n", ret);

	return ret;
}

int oem_qiic_param_write(int chg_det)
{
	int ret = 0;
	int i = 0;

	if (!the_chip->initialized) {
		goto out;
	}

	if (chg_det == CHGER_TYPE_EPP) {
		for (i = 0; epp_param[i].reg != 0; i++) {
			ret = qi_i2c_write(epp_param[i].reg, epp_param[i].data);
		}
	} else {
		for (i = 0; bpp_param[i].reg != 0; i++) {
			ret = qi_i2c_write(bpp_param[i].reg, bpp_param[i].data);
		}
	}

out:
	return ret;
}
EXPORT_SYMBOL(oem_qiic_param_write);

int oem_qiic_qfactor_write(void)
{
	int ret = 0;
	int i = 0;

	if (!the_chip->initialized) {
		goto out;
	}

	for (i = 0; qfactor_param[i].reg != 0; i++) {
		ret = qi_i2c_write(qfactor_param[i].reg, qfactor_param[i].data);
	}

out:
	return ret;
}
EXPORT_SYMBOL(oem_qiic_qfactor_write);

static int wireless_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct oem_wchg_chip *chip;

	pr_info("%s enter\n", __func__);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	chip = devm_kzalloc(&i2c->dev, sizeof(struct oem_wchg_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	chip->dev = &i2c->dev;
	chip->i2c = i2c;

	i2c_set_clientdata(i2c, chip);

	the_chip = chip;

	chip->initialized = true;

	pr_info("%s probe completed successfully!\n", __func__);

	return 0;
}

static int wireless_i2c_remove(struct i2c_client *i2c)
{
	struct oem_wchg_chip *chip = i2c_get_clientdata(i2c);

	pr_info("%s enter\n", __func__);

	devm_kfree(&i2c->dev, chip);
	chip = NULL;

	return 0;
}

static const struct i2c_device_id wireless_i2c_id[] = {
	{ WIRELESS_I2C_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wireless_i2c_id);

static struct of_device_id wireless_dt_match[] = {
	{ .compatible = "oem,wireless_i2c" },
	{ },
};

static struct i2c_driver wireless_i2c_driver = {
	.driver = {
		.name = WIRELESS_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(wireless_dt_match),
	},
	.probe = wireless_i2c_probe,
	.remove = wireless_i2c_remove,
	.id_table = wireless_i2c_id,
};


static int __init wireless_i2c_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&wireless_i2c_driver);
	if(ret){
		pr_err("fail to add wireless device into i2c\n");
		return ret;
	}

	return 0;
}
//late_initcall(wireless_i2c_init);
module_init(wireless_i2c_init);

static void __exit wireless_i2c_exit(void)
{
	i2c_del_driver(&wireless_i2c_driver);
}
module_exit(wireless_i2c_exit);

MODULE_DESCRIPTION("OEM Wireless I2C");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("oem_wireless_i2c");
