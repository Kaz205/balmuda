/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */
#ifndef _PI4IO_H_
#define _PI4IO_H_

/*=========================================*/
/*                include                  */
/*=========================================*/
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/types.h>
#include <linux/fs.h>

#include <linux/uaccess.h>
#include <linux/delay.h>

/*=========================================*/
/*                define                   */
/*=========================================*/
#define PI4IO_MAGIC 0xEA
#define PI4IO_DAC_RESET         _IOW(PI4IO_MAGIC, 0x01, long)

#define PI4IO_I2C_WRITE_REG     _IOW(PI4IO_MAGIC, 0x02, long)
#define PI4IO_I2C_READ_REG      _IOW(PI4IO_MAGIC, 0x03, long)

#define PI4IO_INITIAL_SETTING   _IOW(PI4IO_MAGIC, 0x04, long)

/*=========================================*/
/*            struct declaration           */
/*=========================================*/
struct pi4io_i2c_platform_data {
    unsigned int dac_gpio;
};

#endif

