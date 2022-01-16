/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2021 KYOCERA Corporation
 */

#ifndef _LINUX_KCTOUCH_H
#define _LINUX_KCTOUCH_H

#define KCTP_CUST

#define MIZUKI00	0
#define MIZUKI03	3
#define MIZUKI04	4

//int kctp_ctrl_esd_check_status_get(void);
//int kctp_ctrl_get_easywake_onoff(int blank);
extern int kctp_ilitek_plat_early_suspend(void *pdata, int order);
extern int kctp_ilitek_plat_late_resume(void *pdata, int order);

#endif /* _LINUX_KCTOUCH_H */
