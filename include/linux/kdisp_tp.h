/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */

#ifndef _KDISP_TP_H
#define _KDISP_TP_H

void kdisp_touch_set_info(void *data);
void kdisp_touch_set_state(bool state);
int kdisp_touch_get_easywake_onoff(int blank);

#endif /* _KDISP_TP_H */
