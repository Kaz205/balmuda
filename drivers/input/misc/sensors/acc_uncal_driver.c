/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2021 KYOCERA Corporation
 */
/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "sensor_driver.h"
#include "sensor_micon_driver.h"
#include "sensor_com.h"

#define DEFAULT_ACC_UNCAL_DELAY    (30)
#define GRAVITY_EARTH        9806550
#define ABSMAX_4G            (GRAVITY_EARTH * 4)
#define ABSMIN_4G            (-GRAVITY_EARTH * 4)

static ssize_t acc_uncal_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_uncal_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_uncal_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_uncal_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf );
static ssize_t acc_uncal_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count );
static ssize_t acc_uncal_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);
static ssize_t acc_uncal_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count);
static ssize_t acc_uncal_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf);

static void acc_uncal_update_last_read_data(void);
static void acc_uncal_poll_work_func(struct work_struct *work);
static void acc_uncal_set_input_params( struct input_dev *dev );

static DEVICE_ATTR(enable,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_uncal_enable_show,
    acc_uncal_enable_store
);
static DEVICE_ATTR(delay,
    S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
    acc_uncal_delay_show,
    acc_uncal_delay_store
);
static DEVICE_ATTR(data,
    S_IRUSR|S_IRGRP,
    acc_uncal_data_show,
    NULL
);
static DEVICE_ATTR(batch,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_uncal_batch_store
);
static DEVICE_ATTR(batch_data,
    S_IRUSR|S_IRGRP,
    acc_uncal_batch_data_show,
    NULL
);
static DEVICE_ATTR(flush,
    S_IWUSR|S_IWGRP,
    NULL,
    acc_uncal_flush_store
);

static struct attribute *acc_uncal_attributes[] = {
    &dev_attr_enable.attr,
    &dev_attr_delay.attr,
    &dev_attr_data.attr,
    &dev_attr_batch.attr,
    &dev_attr_batch_data.attr,
    &dev_attr_flush.attr,
    NULL
};

static struct attribute_group acc_uncal_attr_grp = {
    .attrs = acc_uncal_attributes
};

struct sensor_input_info_str acc_uncal_input_info =
{
    NULL,
    acc_uncal_set_input_params,
    &acc_uncal_attr_grp,
};

struct sensor_poll_info_str acc_uncal_poll_info = {
    .name       = "acc_uncal_poll_wq",
    .poll_time  = ATOMIC_INIT(DEFAULT_ACC_UNCAL_DELAY),
    .poll_func  = acc_uncal_poll_work_func,
};

static struct acc_uncalib acc_uncal_last_read_data = {
    0,0,0,0,0,0,
};

static struct sensor_batch_data_str acc_uncal_batch_data;
static uint32_t g_time_stamp_acc_uncal      = 0;
static uint32_t g_input_num_acc_uncal       = 0;
extern struct mutex sensor_batch_mutex;

static ssize_t acc_uncal_batch_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;
    struct sensor_batch_info_str batch_info;

    SENSOR_N_LOG("start");

    sscanf( buf, "%d %d %d",
             &batch_info.flags,
             &batch_info.period_ns,
             &batch_info.timeout );
    SENSOR_N_LOG("parm - flags[%x] period_ns[%x] timeout[%x]",
                  (int)batch_info.flags,
                  (int)batch_info.period_ns,
                  (int)batch_info.timeout );

    ret = sensor_set_batch( SENSOR_ACC_UNCAL, batch_info);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t acc_uncal_batch_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    ssize_t ret =0;

    SENSOR_N_LOG("start");

    mutex_lock(&sensor_batch_mutex);

    ret = sprintf(buf, "%d %d %d %d\n",acc_uncal_batch_data.payload_size,
                       acc_uncal_batch_data.recode_num, g_input_num_acc_uncal,
                       g_time_stamp_acc_uncal);

    g_time_stamp_acc_uncal = 0;

    mutex_unlock(&sensor_batch_mutex);

    SENSOR_N_LOG("end");

    return ret;
}

void acc_uncal_ring_buffer_timestamp(
    uint32_t time_stamp_acc_uncal)
{
    SENSOR_N_LOG("start");
    g_time_stamp_acc_uncal = time_stamp_acc_uncal;
    SENSOR_N_LOG("end %d",g_time_stamp_acc_uncal);
}

void acc_uncal_report_batch(
    enum sensor_batch_report_e_type repo_type,
    struct sensor_batch_data_str batch_data )
{

    SENSOR_N_LOG("start");

    acc_uncal_batch_data = batch_data;
//    acc_uncal_batch_data.payload_size =
//             SENSOR_BATCH_REPORT_SIZE_ACC_UNCAL * acc_uncal_batch_data.recode_num;

    sensor_report_batch( acc_uncal_input_info.dev,
                         repo_type,
                         acc_uncal_batch_data );

    SENSOR_N_LOG("end");
    return;
}

void acc_uncal_timestamp_report(void)
{
    SENSOR_N_LOG("start");

    sensor_report_batch( acc_uncal_input_info.dev,
                         SENSOR_COMP_TIME,
                         acc_uncal_batch_data );

    SENSOR_N_LOG("end");
    return;
}

static ssize_t acc_uncal_flush_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    uint32_t ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_set_flush(SENSOR_ACC_UNCAL, acc_uncal_input_info.dev);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t acc_uncal_enable_show(struct device *dev,
    struct device_attribute *attr,
    char *buf )
{
    int enable = 0;
    SENSOR_N_LOG("start");

    enable = sensor_get_status(SENSOR_ACC_UNCAL);

    SENSOR_N_LOG("end ->enable[%d]",enable);
    return sprintf(buf, "%d\n", enable);
}

static ssize_t acc_uncal_enable_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count )
{
    unsigned long enable = 0;
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &enable);
    SENSOR_N_LOG("kstrtoul() ret[%d]->enable[%d]",ret, (int)enable);

    sensor_enable( SENSOR_ACC_UNCAL, &acc_uncal_poll_info, (bool)enable );

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t acc_uncal_delay_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    int32_t delay = 0;
    SENSOR_N_LOG("start");
    delay = atomic_read(&(acc_uncal_poll_info.poll_time));
    SENSOR_N_LOG("end ->delay[%d]",(int)delay);
    return sprintf(buf, "%d\n", delay);
}

static ssize_t acc_uncal_delay_store(struct device *dev,
    struct device_attribute *attr,
    const char *buf,
    size_t count)
{
    unsigned long delay = 0;
    int ret = 0;
    SENSOR_N_LOG("start");

    ret = kstrtoul(buf, 10, &delay);
    SENSOR_N_LOG("kstrtoul() ret[%d]->delay[%d]",ret, (int)delay);

    sensor_set_poll_time( SENSOR_ACC_UNCAL, &acc_uncal_poll_info, (int32_t)delay);

    SENSOR_N_LOG("end - return[%d]",(int)count);
    return count;
}

static ssize_t acc_uncal_data_show(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    struct acc_uncalib last_data;
    SENSOR_N_LOG("start");
    acc_uncal_update_last_read_data();
    last_data = acc_uncal_last_read_data;
    SENSOR_N_LOG("end - last_data[%d,%d,%d,%d,%d,%d]",
                  last_data.x,
                  last_data.y,
                  last_data.z,
                  last_data.cal_x,
                  last_data.cal_y,
                  last_data.cal_z );

    return sprintf( buf, "%d %d %d %d %d %d\n",
                    last_data.x,
                    last_data.y,
                    last_data.z,
                    last_data.cal_x,
                    last_data.cal_y,
                    last_data.cal_z );
}

static void acc_uncal_update_last_read_data(void)
{
    unsigned long enable = 0;
    int32_t ret = 0;
    union sensor_read_data_u read_data;

    enable = sensor_get_status(SENSOR_ACC_UNCAL);

    if (enable) {
        ret = sensor_type_get_data(SENSOR_ACC_UNCAL, &read_data);
        if (0 == ret) {
            memcpy(&acc_uncal_last_read_data,&(read_data.acc_uncal_data),sizeof(read_data.acc_uncal_data));
        }
    }
}

static void acc_uncal_poll_work_func(struct work_struct *work)
{
    SENSOR_N_LOG("start");

    if(sns_get_reset_status() == false){
        sns_iio_report_event_now(SENSOR_ACC_UNCAL);
    }

    SENSOR_N_LOG("end");
    return;
}

static void acc_uncal_set_input_params( struct input_dev *dev )
{
    SENSOR_N_LOG("start");

    if(!dev){
        SENSOR_ERR_LOG("bad parm --> dev is NULL");
        return;
    }

    dev->name = "accelerometer_uncalibrated";
    dev->id.bustype = BUS_SPI;

    input_set_capability(dev, EV_ABS, ABS_MISC);
    input_set_capability(dev, EV_ABS, ABS_RUDDER);
    input_set_capability(dev, EV_ABS, ABS_THROTTLE);
    input_set_abs_params(dev, ABS_X, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_Y, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_Z, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RX, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RY, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_RZ, ABSMIN_4G, ABSMAX_4G, 0, 0);
    input_set_abs_params(dev, ABS_MISC, INT_MIN,INT_MAX, 0, 0);
    input_set_abs_params(dev, ABS_THROTTLE, INT_MIN, INT_MAX, 0, 0);

    SENSOR_N_LOG("end");
    return;
}

void acc_uncal_driver_init( void )
{
    int ret = 0;

    SENSOR_N_LOG("start");

    ret = sensor_input_init( &acc_uncal_input_info );
    SENSOR_N_LOG("sensor_input_init()-->ret[%d] dev[%p]",
                  ret, acc_uncal_input_info.dev );

    if( (0 != ret) || (NULL == (acc_uncal_input_info.dev))) {
        SENSOR_ERR_LOG("fail sensor_input_init()");
        SENSOR_ERR_LOG("end return[%d]",-ENODEV);
        return;
    }
    sensor_poll_init(SENSOR_ACC_UNCAL, &acc_uncal_poll_info);

    SENSOR_N_LOG("end");
    return;
}
EXPORT_SYMBOL(acc_uncal_driver_init);

