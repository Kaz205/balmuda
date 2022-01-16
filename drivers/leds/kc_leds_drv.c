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

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/pm_wakeup.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include "kc_leds_drv.h"

#include <linux/kc_leds_drv.h>

#include <linux/fs.h>

extern int qpnp_tri_led_set_brightness(struct led_classdev *led_cdev,
		enum led_brightness brightness);

extern int qpnp_tri_led_brightness_set(struct led_classdev *led_cdev,
		int brightness);
		
extern int qpnp_tri_led_set_blink(struct led_classdev *led_cdev,
		unsigned long *on_ms, unsigned long *off_ms);

static struct kc_rgb_info *kc_rgb_data = NULL;
static struct kc_led_info *kc_leds_data;

static int led_force_off_enable = 0;


int kc_leds_get_force_off_enable(void)
{
	return led_force_off_enable;
}

void kc_qti_led_info_get(int num , struct led_classdev *data)
{
	int i;

	if(kc_rgb_data == NULL)
	{
		kc_rgb_data = kmalloc(3 * sizeof(struct kc_rgb_info), GFP_KERNEL);
		kc_rgb_data[0].name = "red";
		kc_rgb_data[0].point = -1;
		
		kc_rgb_data[1].name = "green";
		kc_rgb_data[1].point = -1;	

		kc_rgb_data[2].name = "blue";
		kc_rgb_data[2].point = -1;		
	}
	
	for(i = 0 ; i < 3 ; i++)
	{
		if(strcmp(kc_rgb_data[i].name , data->name) == 0)
		{
			kc_rgb_data[i].dev_class = data;
			kc_rgb_data[i].point = num;
		}
	}
}

void get_timer(unsigned long *on,unsigned long *off)
{
	*on = kc_leds_data->on_time;
	*off = kc_leds_data->off_time;
}

static int kc_leds_flash_current_set(int color, int *led_current)
{
	TRICOLOR_DEBUG_LOG("%s: + RGB color=%d current=%d %d %dSET.",
			__func__, color, *led_current, *(led_current+1), *(led_current+2));

	kc_leds_data->rgb_current[color][RED]   = *led_current;
	kc_leds_data->rgb_current[color][GREEN] = *(led_current+1);
	kc_leds_data->rgb_current[color][BLUE]  = *(led_current+2);

	TRICOLOR_DEBUG_LOG("%s: - RGB color=%d current=%d %d %dSET.",
			__func__,
			color,
			kc_leds_data->rgb_current[color][RED],
			kc_leds_data->rgb_current[color][GREEN],
			kc_leds_data->rgb_current[color][BLUE]);

	return 0;
}

static void kc_leds_get_rgb_current(int brightness, int *red, int *green, int *blue)
{
	int *rgb_current = NULL;
	int new_brightnes = 0;

	brightness &= 0x00FFFFFF;
	TRICOLOR_DEBUG_LOG("%s brightness=%x",__func__, brightness);

	if ((brightness&0x00FF0000) != 0) {
		new_brightnes |= (0xFF << 16);
	}
	if ((brightness&0x0000FF00) != 0) {
		new_brightnes |= (0xFF << 8);
	}
	if ((brightness&0x000000FF) != 0) {
		new_brightnes |= 0xFF;
	}

	TRICOLOR_DEBUG_LOG("%s new_brightnes=%x",__func__, new_brightnes);

	switch (new_brightnes) {
	case 0x00FF0000: /* RED */
		rgb_current = kc_leds_data->rgb_current[RED];
		break;
	case 0x0000FF00: /* GREEN */
		rgb_current = kc_leds_data->rgb_current[GREEN];
		break;
	case 0x000000FF: /* BLUE */
		rgb_current = kc_leds_data->rgb_current[BLUE];
		break;
	case 0x0000FFFF: /* CYAN */
		rgb_current = kc_leds_data->rgb_current[CYAN];
		break;
	case 0x00FF00FF: /* MAGENTA */
		rgb_current = kc_leds_data->rgb_current[MAGENTA];
		break;
	case 0x00FFFF00: /* YELLOW */
		rgb_current = kc_leds_data->rgb_current[YELLOW];
		break;
	case 0x00FFFFFF: /* WHITE */
		rgb_current = kc_leds_data->rgb_current[WHITE];
		break;
	default:
		pr_err("[KCLIHGHT] %s: error", __func__);
		break;
	}

	if (rgb_current) {
		*red   = rgb_current[0];
		*green = rgb_current[1];
		*blue  = rgb_current[2];
	} else {
		*red   = 0;
		*green = 0;
		*blue  = 0;
	}

	TRICOLOR_DEBUG_LOG("%s red=%d green=%d blue=%d",__func__, *red , *green , *blue );
}

static int _kc_leds_rgb_set(int brightness)
{
	int red,green,blue;

	unsigned long on_timer = kc_leds_data->on_time;
	unsigned long off_timer = kc_leds_data->off_time;

	if(kc_rgb_data[0].point == -1 || kc_rgb_data == NULL || brightness < 0)
	{
		TRICOLOR_DEBUG_LOG("%s RGB INFO ERROR\n",__func__);
		return -1;
	}
	
	red 	= (brightness & 0x00FF0000) >> 16;
	green 	= (brightness & 0x0000FF00) >> 8;
	blue 	= (brightness & 0x000000FF);
	
	TRICOLOR_DEBUG_LOG("%s color = %x red = %x green = %x blue = %x\n",__func__, brightness , red , green , blue );
	TRICOLOR_DEBUG_LOG("%s ontime = %x offtime = %x\n",__func__, on_timer, off_timer);
	
	kc_leds_get_rgb_current(brightness, &red, &green, &blue);

	if(kc_leds_data->blue_support == false && green == 0 && blue > 0)
		green = blue;

	if(kc_leds_data->off_time == 0)
	{
		qpnp_tri_led_brightness_set(kc_rgb_data[0].dev_class , red );
		qpnp_tri_led_brightness_set(kc_rgb_data[1].dev_class , green );

		if(kc_leds_data->blue_support == true && kc_rgb_data[2].point != -1)
		{
			qpnp_tri_led_brightness_set(kc_rgb_data[2].dev_class , blue );
		}
	}
	else
	{
		if(kc_leds_data->on_time == 0){
			qpnp_tri_led_brightness_set(kc_rgb_data[0].dev_class , 0 );
			qpnp_tri_led_brightness_set(kc_rgb_data[1].dev_class , 0 );

			if(kc_leds_data->blue_support == true && kc_rgb_data[2].point != -1)
			{
				qpnp_tri_led_brightness_set(kc_rgb_data[2].dev_class , 0 );
			}
		}

		if(red)
		{
			qpnp_tri_led_set_blink(kc_rgb_data[0].dev_class,&on_timer,&off_timer);
		}
		if(green)
		{
			qpnp_tri_led_set_blink(kc_rgb_data[1].dev_class,&on_timer,&off_timer);
		}

		if(kc_leds_data->blue_support == true && kc_rgb_data[2].point != -1)
		{
			if(blue)
			{
				qpnp_tri_led_set_blink(kc_rgb_data[2].dev_class,&on_timer,&off_timer);
			}
		}
	}

	return 0;
}


static void kc_leds_rgb_set(struct led_classdev *led_cdev,
			   enum led_brightness level)
{
    int brightness = level;
    int force_flag;


	TRICOLOR_DEBUG_LOG("%s level = %x\n",__func__, level );

	mutex_unlock(&kc_leds_data->request_lock);

    force_flag = (brightness & TRICOLOR_FORCE_CTRL_MASK);
    brightness = (brightness & TRICOLOR_RGB_MASK);

    if ((kc_leds_data->led_disable) && !force_flag) {
        pr_notice("[KCLIGHT]%s led_disable %x\n", __func__, level);
    } else {
        _kc_leds_rgb_set(brightness);
    }

	mutex_unlock(&kc_leds_data->request_lock);
}

static ssize_t led_rgb_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 color_info;
	int ret;

	ret = kstrtou32(buf, 10, &color_info);
	
	if (ret)
		return ret;
	
	_kc_leds_rgb_set(color_info);

	return count;
}

static ssize_t led_force_off_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 enable;
	int ret;

	ret = kstrtou32(buf, 10, &enable);
	if (ret)
		return ret;

	if (enable) {
		led_force_off_enable = 1;
	} else {
		led_force_off_enable = 0;
	}

	return count;
}

static ssize_t led_force_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", led_force_off_enable);
}

static DEVICE_ATTR(rgb, 0664, NULL, led_rgb_store);
static DEVICE_ATTR(force_off, 0664, led_force_off_show, led_force_off_store);

static struct attribute *led_rgb_attrs[] = {
	&dev_attr_rgb.attr,
	&dev_attr_force_off.attr,
	NULL
};

static int pwm_set(const char *buf,int mode)
{
	u32 time;
	int ret;
	
	unsigned long on_timer;
	unsigned long off_timer;

	ret = kstrtou32(buf, 10, &time);
	
	if (ret)
		return ret;
	
	if(mode == 0)
		kc_leds_data->on_time = time;
	else
		kc_leds_data->off_time = time;
	
	
	on_timer = kc_leds_data->on_time;
	off_timer = kc_leds_data->off_time;
	
	qpnp_tri_led_set_blink(kc_rgb_data[0].dev_class,&on_timer,&off_timer);
	qpnp_tri_led_set_blink(kc_rgb_data[1].dev_class,&on_timer,&off_timer);

	if(kc_leds_data->blue_support == true && kc_rgb_data[2].point != -1)
		qpnp_tri_led_set_blink(kc_rgb_data[2].dev_class,&on_timer,&off_timer);

	return 0;
}

static ssize_t pwm_lo_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	pwm_set(buf,0);

	return count;
}

static ssize_t pwm_hi_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	pwm_set(buf,1);

	return count;
}

static DEVICE_ATTR(pwm_lo, 0664, NULL, pwm_lo_store);
static DEVICE_ATTR(pwm_hi, 0664, NULL, pwm_hi_store);

static struct attribute *pwm_attrs[] = {
	&dev_attr_pwm_lo.attr,
	&dev_attr_pwm_hi.attr,
	NULL
};

static const struct attribute_group led_rgb_attr_group = {
	.attrs = led_rgb_attrs,
};

static const struct attribute_group pwm_attr_group = {
	.attrs = pwm_attrs,
};

static int kc_leds_get_pattern() {
    return kc_leds_data->pattern;
}

static uint32_t kc_leds_set_pattern(uint32_t id, uint32_t ontime, uint32_t offtime, uint32_t mode) {
	uint32_t pattern = 0, red = 0, green = 0, blue = 0, offset = 0;
    kc_leds_data->blink = true;
    kc_leds_data->breath = false;

    //kc_rgb_data
    // 0 -> red
    // 1 -> green
    // 2 -> blue

    TRICOLOR_DEBUG_LOG("%s(): id = %d, ontime = %d, offtime = %d, color = %d\n",__func__, id, ontime, offtime, mode);

    red = (mode & 0x00010000) >> 16;
    green = (mode & 0x00000200) >> 8;
    blue = (mode & 0x00000004);
    offset = red + green +blue;

    TRICOLOR_DEBUG_LOG("%s(): red = %d, green = %d, blue = %d, offset = %d\n",__func__, red, green, blue, offset);

	switch (id) {
		case ID_ATTENTION:
            // white
			TRICOLOR_DEBUG_LOG("%s(): id = %d, ID_ATTENTION",__func__, id);
            pattern = 1;
            kc_leds_set_lightparam(kc_rgb_data[0].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[1].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[2].dev_class, /*blink*/false, /*breath*/true, 120);
			if (ontime != 0 && offtime != 0){
				kc_leds_data->on_time = 100;
				kc_leds_data->off_time = 200 + offset;
			}
			break;

        case ID_ALARM:
            //yellow (red + green)
			TRICOLOR_DEBUG_LOG("%s(): id = %d, ID_ALARM",__func__, id);
            pattern = 2;
            kc_leds_set_lightparam(kc_rgb_data[0].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[1].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[2].dev_class, /*blink*/false, /*breath*/true, 120);
			if (ontime != 0 && offtime != 0){
				kc_leds_data->on_time = 100;
				kc_leds_data->off_time = 200 + offset;
			}
			break;

        case ID_NOTIFICATIONS:
            pattern = 3;
			TRICOLOR_DEBUG_LOG("%s(): id = %d, ID_NOTIFICATIONS",__func__, id);
            kc_leds_set_lightparam(kc_rgb_data[0].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[1].dev_class, /*blink*/false, /*breath*/true, 120);
            kc_leds_set_lightparam(kc_rgb_data[2].dev_class, /*blink*/false, /*breath*/true, 120);
			kc_leds_data->on_time = 500;
			if (offtime < 1000)
				kc_leds_data->off_time = 600 + offset;
			else
				kc_leds_data->off_time = 1000 + offset;
            break;

        default:
            break;
	}
	return pattern;
}

static long kclights_leds_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int32_t ret = -1;
	T_LEDLIGHT_IOCTL		st_ioctl;
	
    switch (cmd) {
    case LEDLIGHT_SET_BLINK:
        TRICOLOR_DEBUG_LOG("%s(): LEDLIGHT_SET_BLINK",__func__);
        ret = copy_from_user(&st_ioctl,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL));
        if (ret) {
            TRICOLOR_DEBUG_LOG("%s(): Error leds_ioctl(cmd = LEDLIGHT_SET_BLINK)", __func__);
            return -EFAULT;
        }

		mutex_lock(&kc_leds_data->request_lock);
        TRICOLOR_DEBUG_LOG("%s(): mutex_lock", __func__);

		kc_leds_data->mode      = st_ioctl.data[0];
		kc_leds_data->on_time   = st_ioctl.data[1];
		kc_leds_data->off_time  = st_ioctl.data[2];
		kc_leds_data->off_color = st_ioctl.data[3];

        TRICOLOR_DEBUG_LOG("%s(): mode = %d, on_time = %d, off_time = %d, id = %d", __func__, kc_leds_data->mode, kc_leds_data->on_time, kc_leds_data->off_time, kc_leds_data->off_color);
        TRICOLOR_DEBUG_LOG("%s(): pattern_enable = %d", __func__, kc_leds_data->pattern_enable);

		if (kc_leds_data->pattern_enable) {
//			if (kc_leds_data->on_time != 0 && kc_leds_data->off_time != 0)
			if (kc_leds_data->mode != 0)
				kc_leds_data->pattern = kc_leds_set_pattern(kc_leds_data->off_color, kc_leds_data->on_time, kc_leds_data->off_time, kc_leds_data->mode);
			else
				kc_leds_data->pattern = 0;
		} else {
			kc_leds_data->pattern = 0;
		}

		//Todo
		kc_leds_get_pattern();

		TRICOLOR_DEBUG_LOG("prep mode=[%d] on_time=[%d] off_time=[%d] off_color=[0x%08x]",
			kc_leds_data->mode, kc_leds_data->on_time, kc_leds_data->off_time, kc_leds_data->off_color);

        TRICOLOR_DEBUG_LOG("%s(): mutex_unlock", __func__);
		mutex_unlock(&kc_leds_data->request_lock);
        break;
    case LEDLIGHT_SET_LED_DIS:
        TRICOLOR_DEBUG_LOG("%s(): LEDLIGHT_SET_LED_DIS",__func__);
        ret = copy_from_user(&st_ioctl,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL));

        mutex_lock(&kc_leds_data->request_lock);
        TRICOLOR_DEBUG_LOG("%s(): LEDLIGHT_SET_LED_DIS mutex_lock", __func__);
        if (st_ioctl.data[0]) {
            kc_leds_data->led_disable = 1;
            _kc_leds_rgb_set(0);
        } else {
            kc_leds_data->led_disable = 0;
        }
        mutex_unlock(&kc_leds_data->request_lock);
        TRICOLOR_DEBUG_LOG("%s(): LEDLIGHT_SET_LED_DIS mutex_unlock", __func__);
        break;
    default:
        TRICOLOR_DEBUG_LOG("%s(): default", __func__);
        return -ENOTTY;
    }

    return 0;
}

static struct file_operations kclights_leds_fops = {
    .owner        = THIS_MODULE,
	.open           = simple_open,
    .unlocked_ioctl = kclights_leds_ioctl,
    .compat_ioctl   = kclights_leds_ioctl,
};

static int kc_leds_probe(struct platform_device *pdev)
{
	int rc;
	int i;
	u32 cur_array[LED_NUM];
	char *prop_name;

	kc_leds_data = kzalloc(sizeof(struct kc_led_info), GFP_KERNEL);
	
	if(!kc_leds_data)
		return -ENOMEM;

	mutex_init(&kc_leds_data->request_lock);

	dev_set_drvdata(&pdev->dev, kc_leds_data);

	kc_leds_data->cdev.name = "ledinfo";

	kc_leds_data->cdev.brightness_set = kc_leds_rgb_set;
	kc_leds_data->cdev.max_brightness = 0xFFFFFFFF;
	
	kc_leds_data->on_time = 0;
	kc_leds_data->off_time = 0;
	
	kc_leds_data->mdev.minor = MISC_DYNAMIC_MINOR;
	kc_leds_data->mdev.name = "leds-ledlight";
	kc_leds_data->mdev.fops = &kclights_leds_fops;
	rc =  misc_register(&kc_leds_data->mdev);
		
	kc_leds_data->blue_support = of_property_read_bool(pdev->dev.of_node, "blue_support");
	kc_leds_data->pattern_enable = of_property_read_bool(pdev->dev.of_node, "pattern_enable");

	if(kc_leds_data->blue_support == false)
		TRICOLOR_DEBUG_LOG("%s BLUE NOT SUPPORT\n",__func__);
	
	kc_leds_data->led_disable = 0;
	
	rc = led_classdev_register(&pdev->dev, &kc_leds_data->cdev);
	
	if(rc > 0)
		goto probe_error;
	
	rc = 0;
	rc += sysfs_create_group(&kc_leds_data->cdev.dev->kobj,&led_rgb_attr_group);
	rc += sysfs_create_group(&kc_leds_data->cdev.dev->kobj,&pwm_attr_group);
	
	if(rc > 0)
		goto probe_error;

	for (i = 0; i < COLOR_MAX; i++) {
		prop_name = color_dt_prop_name[i];
		TRICOLOR_DEBUG_LOG("%s LED prop_name=%s",__func__, prop_name);

		rc = of_property_read_u32_array(pdev->dev.of_node,
				prop_name, cur_array, LED_NUM);
		TRICOLOR_DEBUG_LOG("%s color=%d current=%d %d %d",
				__func__, i, cur_array[0], cur_array[1], cur_array[2]);
		if (rc) {
			TRICOLOR_DEBUG_LOG("[KCLIGHT] %s: red current read error");
			goto probe_error;
		}
		kc_leds_flash_current_set(i, cur_array);
	}

    TRICOLOR_DEBUG_LOG("%s(): [OUT]",__func__);

	return 0;
	
probe_error:
	led_classdev_unregister(&kc_leds_data->cdev);
	TRICOLOR_DEBUG_LOG("%s PROBE ERROR\n",__func__);
	return rc;

}

static int kc_leds_remove(struct platform_device *pdev)
{

	if(kc_rgb_data != NULL)
		kfree(kc_rgb_data);
	
	led_classdev_unregister(&kc_leds_data->cdev);
	
	sysfs_remove_group(&kc_leds_data->cdev.dev->kobj,	&led_rgb_attr_group);
	sysfs_remove_group(&kc_leds_data->cdev.dev->kobj,	&pwm_attr_group);
	
	TRICOLOR_DEBUG_LOG("%s\n",__func__);
	
	return 0;
}

static const struct of_device_id kc_led_of_match[] = {
	{ .compatible = "kc,kc_leds", },
	{}
};

MODULE_DEVICE_TABLE(of, kc_led_of_match);

static struct platform_driver kc_leds_driver = {
	.driver = {
		   	.name = "kc_leds",
		   	.owner = THIS_MODULE,
			.of_match_table = kc_led_of_match,
		   },
	.probe = kc_leds_probe,
	.remove = kc_leds_remove,
};

static int __init kc_leds_init(void)
{
	int ret;

	ret = platform_driver_register(&kc_leds_driver);

	return ret;
}

static void __exit kc_leds_exit(void)
{
	platform_driver_unregister(&kc_leds_driver);
}

late_initcall(kc_leds_init);
module_exit(kc_leds_exit);

MODULE_AUTHOR("Kyocera Inc.");
MODULE_DESCRIPTION("LED driver for MediaTek MT65xx chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("kc-leds-mt65xx");