 /*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

#ifndef _KC_LEDS_DRV_H
#define _KC_LEDS_DRV_H

#include <linux/leds.h>

/****************************************************************************
 * LED DRV functions
 ***************************************************************************/

extern void kc_qti_led_info_get(int num , struct led_classdev *data);

#endif