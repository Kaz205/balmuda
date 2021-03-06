/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */
/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
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
#ifndef _LTR559_H
#define _LTR559_H

#ifdef CONFIG_USE_MICON_SOFT_STRUCTURE
int ps_als_init(void);
void ps_als_exitt(void);
int32_t als_get_initialize_state(void);
int32_t ps_get_initialize_state(void);
int32_t als_sensor_activate(bool enable);
int32_t ps_sensor_activate(bool enable);
#endif

struct ltr559_platform_data {
	unsigned int prox_threshold;
	unsigned int prox_hsyteresis_threshold;

	unsigned int als_poll_interval;
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(bool);
	unsigned int prox_default_noise;
};

/* POWER SUPPLY VOLTAGE RANGE */
#define LTR559_VDD_MIN_UV  3000000
#define LTR559_VDD_MAX_UV  3000000
#define LTR559_VIO_MIN_UV  1750000
#define LTR559_VIO_MAX_UV  1950000

/* LTR-559 Registers */
#define LTR559_ALS_CONTR		0x80
#define LTR559_PS_CONTR			0x81
#define LTR559_PS_LED			0x82
#define LTR559_PS_N_PULSES		0x83
#define LTR559_PS_MEAS_RATE		0x84
#define LTR559_ALS_MEAS_RATE		0x85
#define LTR559_MANUFACTURER_ID		0x87

#define LTR559_INTERRUPT		0x8F
#define LTR559_PS_THRES_UP_0		0x90
#define LTR559_PS_THRES_UP_1		0x91
#define LTR559_PS_THRES_LOW_0		0x92
#define LTR559_PS_THRES_LOW_1		0x93

#define LTR559_ALS_THRES_UP_0		0x97
#define LTR559_ALS_THRES_UP_1		0x98
#define LTR559_ALS_THRES_LOW_0		0x99
#define LTR559_ALS_THRES_LOW_1		0x9A
#define LTR559_INTERRUPT_PERSIST	0x9E

/* 559's Read Only Registers */
#define LTR559_ALS_DATA_CH1_0		0x88
#define LTR559_ALS_DATA_CH1_1		0x89
#define LTR559_ALS_DATA_CH0_0		0x8A
#define LTR559_ALS_DATA_CH0_1		0x8B
#define LTR559_ALS_PS_STATUS		0x8C
#define LTR559_PS_DATA_0		0x8D
#define LTR559_PS_DATA_1		0x8E

/* Basic Operating Modes */
#define MODE_ALS_ON_Range1		0x0B
#define MODE_ALS_ON_Range2		0x03
#define MODE_ALS_StdBy			0x00
#define MODE_PS_ON_Gain1		0x03
#define MODE_PS_ON_Gain2		0x07
#define MODE_PS_ON_Gain4		0x0B
#define MODE_PS_ON_Gain8		0x0C
#define MODE_PS_StdBy			0x00

#define PS_RANGE1			1
#define PS_RANGE2			2
#define PS_RANGE4			4
#define PS_RANGE8			8
#define ALS_RANGE1_320			1
#define ALS_RANGE2_64K			2

#define PS_DETECTED_THRES		200
#define PS_UNDETECTED_THRES		180

/* Power On response time in ms */
#define PON_DELAY			600
#define WAKEUP_DELAY			10

#endif
