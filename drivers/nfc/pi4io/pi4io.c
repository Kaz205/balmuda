/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */
#include "pi4io.h"

#define MAX_BUFFER_SIZE 512

#define DAC_RESET_HIGH  1
#define DAC_RESET_LOW   0

static struct pi4io_dev *pi4io_dev;

struct pi4io_dev    {
    struct mutex        read_mutex;
    struct i2c_client   *client;
    struct miscdevice   pi4io_device;
    unsigned int        dac_gpio;
};

static ssize_t pi4io_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{

    struct pi4io_dev *pi4io_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    pr_info("pi4io_dev_read\n");

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    pr_debug("%s : reading   %zu bytes.\n", __func__, count);

    mutex_lock(&pi4io_dev->read_mutex);

    /* Read data */
    ret = i2c_master_recv(pi4io_dev->client, tmp, count);
    mutex_unlock(&pi4io_dev->read_mutex);

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
                __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }
    return ret;
}

static ssize_t pi4io_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pi4io_dev  *pi4io_dev;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    pr_info("pi4io_dev_write\n");

    pi4io_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    // Write Data
    pr_debug("%s : Write Data writing %zu bytes.\n", __func__, count);
    /* Write data */
    ret = i2c_master_send(pi4io_dev->client, tmp, count);
    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    udelay(1000);

    return ret;
}

static int pi4io_dev_open(struct inode *inode, struct file *filp)
{
    struct pi4io_dev *pi4io_dev = container_of(filp->private_data,
            struct pi4io_dev,
            pi4io_device);

    pr_info("pi4io_dev_open\n");

    filp->private_data = pi4io_dev;

    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    return 0;
}

static void pi4io_initial_setting(unsigned long arg)
{
    int ret = 0;
    char write_data[2] = {0};
    char read_data[1] = {0};

    pr_info("%s Enter arg[0x%02X]\n", __func__, arg);

    /* DAC RESET */
    gpio_set_value(pi4io_dev->dac_gpio, DAC_RESET_HIGH);
    udelay(1000);
    gpio_set_value(pi4io_dev->dac_gpio, DAC_RESET_LOW);
    //udelay(10000);
    //30msec wait
    msleep(30);

    /* [reg:01] read */
    write_data[0] = 0x01;                       /* reg addr */
    ret = i2c_master_send(pi4io_dev->client, write_data, 1);
    if (ret < 0) {
        pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
        return;
    }
    mutex_lock(&pi4io_dev->read_mutex);
    ret = i2c_master_recv(pi4io_dev->client, read_data, 1);
    mutex_unlock(&pi4io_dev->read_mutex);
    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return;
    }

    /* buffer clear */
    memset(write_data, 0x00, sizeof(write_data));

    /* [reg:03] write */
    write_data[0] = 0x03;                       /* reg addr + data */
    write_data[1] = 0x1F;
    ret = i2c_master_send(pi4io_dev->client, write_data, 2);
    if (ret < 0) {
        pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
        return;
    }

    /* buffer clear */
    memset(write_data, 0x00, sizeof(write_data));

    /* [reg:07] write */
    write_data[0] = 0x07;                       /* reg addr + data */
    write_data[1] = 0xE0;
    ret = i2c_master_send(pi4io_dev->client, write_data, 2);
    if (ret < 0) {
        pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
        return;
    }

    /* buffer clear */
    memset(write_data, 0x00, sizeof(write_data));

    /* [reg:05] write */
    write_data[0] = 0x05;                       /* reg addr + data */
    write_data[1] = (unsigned char)arg;
    ret = i2c_master_send(pi4io_dev->client, write_data, 2);
    if (ret < 0) {
        pr_err("%s: i2c_master_send returned %d\n", __func__, ret);
        return;
    }

    return;
}

static long pi4io_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg){
    int value = 0;

    pr_info("pi4io_dev_ioctl\n");

    switch(cmd)
    {
        case PI4IO_DAC_RESET:
            if (arg == 0){ /* Change low */
                gpio_set_value(pi4io_dev->dac_gpio, DAC_RESET_LOW);
                value = gpio_get_value(pi4io_dev->dac_gpio);
                pr_info("%s GPIO DAC_RESET LOW %d \n", __func__, value);
            }else if (arg == 1){ /* Change high */
                gpio_set_value(pi4io_dev->dac_gpio, DAC_RESET_HIGH);
                value = gpio_get_value(pi4io_dev->dac_gpio);
                pr_info("%s GPIO DAC_RESET HIGHT %d \n", __func__, value);
            }else{
                pr_info("%s arg INVALID %d \n", __func__, arg);
            }
            break;
        case PI4IO_INITIAL_SETTING:
            pi4io_initial_setting(arg);
            break;
        default:
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            return -EINVAL;
    }

    pr_info("%s :exit cmd = %u, arg = %ld\n", __func__, cmd, arg);
    return 0;
}
EXPORT_SYMBOL(pi4io_dev_ioctl);

static const struct file_operations pi4io_dev_fops = {
        .owner  = THIS_MODULE,
        .llseek = no_llseek,
        .read   = pi4io_dev_read,
        .write  = pi4io_dev_write,
        .open   = pi4io_dev_open,
        .unlocked_ioctl  = pi4io_dev_ioctl,
};

static int pi4io_parse_dt(struct device *dev,
    struct pi4io_i2c_platform_data *data)
{
    struct device_node *np = dev->of_node;
    int errorno = 0;

    data->dac_gpio = of_get_named_gpio(np, "pericom,pi4io", 0);
    if ((!gpio_is_valid(data->dac_gpio)))
            return -EINVAL;
    pr_info("%s: %d error:%d\n", __func__,
        data->dac_gpio, errorno);

    return errorno;
}

static int pi4io_probe(struct i2c_client *client,
        const struct i2c_device_id *id){
    int ret = 0;
    struct pi4io_i2c_platform_data *platform_data;
    struct device_node *node = client->dev.of_node;

    pr_info("pi4io_probe\n");
    pr_info("id.name = %s, id.driver_data = %d", id->name, id->driver_data);
    pr_info("slave address = 0x%02X\n", client->addr);

#if 0
    platform_data = client->dev.platform_data;
#else
    if(node){
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct pi4io_i2c_platform_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev,
                "nfc-nci probe: Failed to allocate memory\n");
            return -ENOMEM;
        }
        ret = pi4io_parse_dt(&client->dev, platform_data);
        if (ret)
        {
            pr_info("%s pi4io_parse_dt failed", __func__);
        }
    } else {
        platform_data = client->dev.platform_data;
    }
#endif

    if (platform_data == NULL) {
        pr_err("%s : gpio expander probe fail\n", __func__);
        return  -ENODEV;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        return  -ENODEV;
    }

    ret = gpio_request(platform_data->dac_gpio, "pi4io");
    if (ret)
        return  -ENODEV;

    pi4io_dev = kzalloc(sizeof(*pi4io_dev), GFP_KERNEL);
    if (pi4io_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    pi4io_dev->dac_gpio = platform_data->dac_gpio;

    /* init mutex and queues */
    mutex_init(&pi4io_dev->read_mutex);
    pi4io_dev->pi4io_device.minor = MISC_DYNAMIC_MINOR;
    pi4io_dev->pi4io_device.name = "pi4io";
    pi4io_dev->pi4io_device.fops = &pi4io_dev_fops;

    pi4io_dev->client   = client;

    ret = misc_register(&pi4io_dev->pi4io_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }

    device_init_wakeup(&client->dev, true);
    i2c_set_clientdata(client, pi4io_dev);

    return 0;

    err_exit:
    err_misc_register:
    mutex_destroy(&pi4io_dev->read_mutex);
    return ret;
}

static int pi4io_remove(struct i2c_client *client){
    struct pi4io_dev *pi4io_dev;
    pi4io_dev = i2c_get_clientdata(client);

    misc_deregister(&pi4io_dev->pi4io_device);
    mutex_destroy(&pi4io_dev->read_mutex);
    gpio_free(pi4io_dev->dac_gpio);

    kfree(pi4io_dev);
    return 0;
}

static struct i2c_device_id pi4io_id_table[] = {
    {"pi4io", 0},
    {}
};

static struct of_device_id pi4io_i2c_dt_match[] = {
    {
        .compatible = "pericom,pi4io",
    },
    {}
};

/*
 * Function List
 */
static struct i2c_driver pi4io_driver = {
        .id_table   = pi4io_id_table,
        .probe      = pi4io_probe,
        .remove     = pi4io_remove,
        .driver     = {
                .owner = THIS_MODULE,
                .name  = "pi4io",
                .of_match_table = pi4io_i2c_dt_match,
//                .pm = &pi4io_dev_fops,
        },
};
// module_i2c_driver(pi4io_driver);


/*
 * module load/unload record keeping
 */

static int __init pi4io_dev_init(void)
{
    pr_info("Loading pi4io driver\n");
    return i2c_add_driver(&pi4io_driver);
}
module_init(pi4io_dev_init);

static void __exit pi4io_dev_exit(void)
{
    pr_info("Unloading pi4io driver\n");
    i2c_del_driver(&pi4io_driver);
}
module_exit(pi4io_dev_exit);

MODULE_AUTHOR("Kyocera");
MODULE_DESCRIPTION("PI4IOE5V6408 GPIO Expander");
MODULE_LICENSE("GPL");

