/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2021 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef __KTP_DRV_H__
#define __KTP_DRV_H__

#include <linux/types.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/cdev.h>

#define KTP_RET_OK                  "[OK]"
#define KTP_RET_NG                  "[NG]"
#define KTP_MAX_PRBUF_SIZE       PAGE_SIZE

#define STATUS_SUCCESS   0
#define STATUS_FAIL     -1

#define KTP_MAX_AREA_SIZE        (20)
#define KTP_MAX_PRESSURE_SIZE    (250)

#define KTP_GLOVE_OFF		0
#define KTP_GLOVE_ON		1
#define KTP_GLOVE_AUTO		2

#define KTP_POCKET_OFF		0
#define KTP_POCKET_ON		1

#define KTP_GSMMODE_OFF		0
#define KTP_GSMMODE_ON		1

#define BLANK   1
#define UNBLANK 0

enum ktp_power_state {
	KTP_POWER_STATE_INIT = -1,
	KTP_POWER_STATE_PWROFF_ON = 0,
	KTP_POWER_STATE_PWRON_OFF = 1,
	KTP_POWER_STATE_EXIT_LPWG = 2,
	KTP_POWER_STATE_ENTER_LPWG = 3,
};

enum ktp_touch_easywake_req {
	KTP_TOUCH_EASYWAKE_REQ_OFF,
	KTP_TOUCH_EASYWAKE_REQ_ON,
	KTP_TOUCH_EASYWAKE_REQ_MAX
};

enum ktp_charge_mode {
	KTP_CHARGE_MODE_USB_PLUGIN = 0,
	KTP_CHARGE_MODE_NONE = 1,
	KTP_CHARGE_MODE_WIRELESS = 2,
};

#define KTP_DRV_SYSFS_LOGLEVEL			1

#define KTP_DEV_LOG(fmt, arg...)		if(ts_log_level & 0x01) pr_info(fmt, ## arg)

struct kc_ts_data {
	u32 fw_ic_ver;
	u32 fw_img_ver;
};

#endif /* __KTP_DRV_H__ */
