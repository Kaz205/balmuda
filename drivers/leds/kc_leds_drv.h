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

#include <linux/uaccess.h>
#include <linux/miscdevice.h>


#define TRICOLOR_DEBUG			1

#if TRICOLOR_DEBUG
#define TRICOLOR_DEBUG_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#else
#define TRICOLOR_DEBUG_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][D](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)
#endif

#define TRICOLOR_NOTICE_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][N](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TRICOLOR_ERR_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][E](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)

#define TRICOLOR_WARN_LOG( msg, ... ) \
pr_err("[TRICOLOR][%s][W](%d): " msg "\n", __func__, __LINE__, ## __VA_ARGS__)



/****************************************************************************
 * LED DRV functions
 ***************************************************************************/
#define LED_NUM 3

enum led_colors {
	RED,
	GREEN,
	BLUE,
	CYAN,
	MAGENTA,
	YELLOW,
	WHITE,
	COLOR_MAX
};

struct kc_rgb_info{
	char* 	name;
	int		point;
	struct led_classdev *dev_class;
};

struct kc_led_info {
	struct led_classdev cdev;
	struct mutex		request_lock;
	struct miscdevice	mdev;
	uint32_t			mode;
	uint32_t			on_time;
	uint32_t			off_time;
	uint32_t			off_color;
	uint32_t			pattern;
	int					rgb_current[COLOR_MAX][LED_NUM];
	bool				blue_support;
	bool				pattern_enable;
    bool				blink;
    bool				breath;
	int					led_disable; /* 0: disable 1:enable */
};

static char color_dt_prop_name[COLOR_MAX][16] = {
	"kc,rled-current",
	"kc,gled-current",
	"kc,bled-current",
	"kc,cled-current",
	"kc,mled-current",
	"kc,yled-current",
	"kc,wled-current"
};

enum lights_id {
	ID_BATTERY,
	ID_NOTIFICATIONS,
	ID_NOTIFICATIONS_EX,
	ID_ATTENTION,
	ID_EMERGENCY,
	ID_ALARM,
};

#define LEDLIGHT_BLINK_NUM		4
typedef struct _t_ledlight_ioctl {
	uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

#define LEDLIGHT			'L'
#define LEDLIGHT_SET_BLINK		_IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define LEDLIGHT_DIS           'D'
#define LEDLIGHT_SET_LED_DIS   _IOW(LEDLIGHT_DIS, 0, T_LEDLIGHT_IOCTL)

#define TRICOLOR_RGB_COLOR_RED			0x00FF0000
#define TRICOLOR_RGB_COLOR_GREEN		0x0000FF00
#define TRICOLOR_RGB_COLOR_BLUE			0x000000FF
#define TRICOLOR_FORCE_CTRL_MASK		0xFF000000

#define TRICOLOR_RGB_MAX_BRIGHT_VAL      (0xFFFFFFFFu)
#define TRICOLOR_RGB_GET_R(color)        (((color) & 0x00FF0000u) >> 16)
#define TRICOLOR_RGB_GET_G(color)        (((color) & 0x0000FF00u) >> 8 )
#define TRICOLOR_RGB_GET_B(color)        (((color) & 0x000000FFu) >> 0 )
#define TRICOLOR_RGB_MASK                (0x00FFFFFFu)
#define TRICOLOR_RGB_OFF                 (0x00000000u)

extern void kc_qti_led_info_get(int num , struct led_classdev *data);

extern void get_timer(unsigned long *on,unsigned long *off);
extern int kc_leds_get_force_off_enable(void);
void kc_leds_set_lightparam(struct led_classdev *led_cdev, bool blink, bool breath, int brightness);

#endif
