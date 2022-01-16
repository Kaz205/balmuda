/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */


#ifndef KC_PT_TOUCH_DISPLAY
#define KC_PT_TOUCH_DISPLAY
#define CONFIG_INCELL_TOUCH

#ifdef CONFIG_INCELL_TOUCH
extern int pt_display_suspend(void *pt_data, int order);
extern int pt_display_resume(void *pt_data, int order);
extern void pt_display_set_easywake_mode(void *pt_data, int easywake_mode);
#endif

#endif