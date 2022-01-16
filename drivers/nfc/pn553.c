/*
 * Copyright (C) 2010 Trusted Logic S.A.
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
 *
 */
/******************************************************************************
 *
 *  The original Work has been changed by NXP Semiconductors.
 *
 *  Copyright (C) 2013-2020 NXP Semiconductors
 *   *
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 ******************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/workqueue.h>

#include <linux/pinctrl/pinctrl.h>
#undef kc_leds

#ifdef kc_leds
#include <linux/kc_leds_drv.h>
#endif

#define KCDEBUG   0



/* HiKey Compilation fix */
#define HiKey_620_COMPILATION_FIX 0
#ifndef HiKey_620_COMPILATION_FIX
#include <linux/wakelock.h>
#endif

#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 11, 0)
#include <linux/sched/signal.h>
#include <linux/fs.h>
#endif

#include <linux/timer.h>
#include "pn553.h"
#include "cold_reset.h"
#include "../nfc-spi/p73.h"

#define DRAGON_NFC 1
#define SIG_NFC 44
#define MAX_BUFFER_SIZE 554
#define MAX_SECURE_SESSIONS 1
#define VEN_PWR_ON  1
#define VEN_PWR_OFF 0
#define PN553   						1

#define PLAT_MVER_DMT1	0x0510
#define PLAT_MVER_DMT2	0x0520
int platform_mver = 0x0520;

#define PWR_EN_ON  1
#define PWR_EN_OFF 0
#define FET_G_ON  1
#define FET_G_OFF 0

struct pinctrl *pinctrl_i2c;
struct pinctrl_state *i2c_nfc_active, *i2c_nfc_suspend;
struct pinctrl_state *nfc_rst_active, *nfc_rst_suspend, *nfc_rst_pre_suspend, *nfc_rst_pre_active;
struct pinctrl_state *i2c_nfc_init, *i2c_nfc_deinit;
struct pinctrl_state *pwr_en_active, *pwr_en_suspend;

/* This macro evaluates to 1 if the cold reset is requested by driver(SPI/UWB). */
#define IS_PROP_CMD_REQUESTED(flags) (flags & (MASK_ESE_COLD_RESET | RST_PROTECTION_ENABLED))
/* This macro evaluates to 1 if eSE cold reset response is received */
#define IS_PROP_RSP(buf)                                                                          \
                (((MSG_NFCC_RSP | MSG_PROP_GID) == buf[0]) && ((ESE_CLD_RST_OID == buf[1]) ||     \
                 (RST_PROTECTION_OID == buf[1]) ))

/* VEN is kept ON all the time if you define the macro VEN_ALWAYS_ON.
Used for SN100 usecases */
//#define VEN_ALWAYS_ON
/* Macro added to disable SVDD power toggling */
/* #define JCOP_4X_VALIDATION */


/* HiKey Compilation fix */
#ifndef HiKey_620_COMPILATION_FIX
struct wake_lock nfc_wake_lock;
#if HWINFO
struct hw_type_info hw_info;
#endif
static bool  sIsWakeLocked = false;
#endif
static struct pn544_dev *pn544_dev;
static struct semaphore ese_access_sema;
static struct semaphore svdd_sync_onoff_sema;
static struct completion dwp_onoff_sema;
static struct timer_list secure_timer;
static void release_ese_lock(p61_access_state_t  p61_current_state);
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout);
static long set_jcop_download_state(unsigned long arg);
static long start_seccure_timer(unsigned long timer_value);
static long secure_timer_operation(struct pn544_dev *pn544_dev, unsigned long arg);
extern void rcv_prop_resp_status(const char * const buf);
extern long ese_cold_reset(ese_cold_reset_origin_t src);
extern void ese_reset_resource_init(void);
extern void ese_reset_resource_destroy(void);
extern void set_force_reset(bool value);
extern int do_reset_protection(bool enable);

#if HWINFO
static void check_hw_info(void);
#endif
#define SECURE_TIMER_WORK_QUEUE "SecTimerCbWq"

void i2c_nfc_pinctrl(int status);
void i2c_nfc_pinctrl_init(int status);
void nfc_en_fet_g_pinctrl(int status);
void nfc_pwr_en_pinctrl(int state);

struct pn544_dev * get_nfcc_dev_data(void) {
  return pn544_dev;
}


static void Control_GPIO_for_kflc_gpio( int arg)
{

    if ( KCDEBUG ) pr_info("%s start %d \n", __func__ , arg);


	if 		(10 ==  arg ) gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_ON);
	else if (11 ==  arg ) gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
	else if (12 ==  arg ) gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
	else if (13 ==  arg ) gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
	else if (14 ==  arg ) gpio_set_value(pn544_dev->firm_gpio, 1);
	else if (15 ==  arg ) gpio_set_value(pn544_dev->firm_gpio, 0);

    if ( KCDEBUG ) pr_info("%s end %d \n", __func__ , arg);

}

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
    unsigned long flags;
    if ( KCDEBUG ) pr_info("%s start  \n", __func__ );

    spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
    if (pn544_dev->irq_enabled) {
        disable_irq_nosync(pn544_dev->client->irq);
    if ( KCDEBUG ) pr_info("%s: disable_irq_nosync() \n", __func__);

//        disable_irq_wake(pn544_dev->client->irq);
        pn544_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);

    if ( KCDEBUG ) pr_info("%s End \n", __func__ );

}

static int pn544_dev_release(struct inode *inode, struct file *filp) {

    if ( KCDEBUG ) pr_info("%s start  \n", __func__ );

    pn544_dev->state_flags &= ~(P544_FLAG_NFC_VEN_RESET|P544_FLAG_NFC_ON|P544_FLAG_FW_DNLD);
    set_force_reset(false);
    if (pn544_dev->firm_gpio)
        gpio_set_value(pn544_dev->firm_gpio, 0);
    pr_info(KERN_ALERT "Exit %s: NFC driver release  nfc hal  \n", __func__);

   if ( KCDEBUG ) pr_info("%s End \n", __func__ );

    return 0;
}
static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
    struct pn544_dev *pn544_dev = dev_id;

    if ( KCDEBUG ) pr_info("%s start \n", __func__ ); 

   	if (device_may_wakeup(&pn544_dev->client->dev))
	//pm_wakeup_event(&&pn544_dev->client->dev, WAKEUP_SRC_TIMEOUT);
    pm_wakeup_event(&pn544_dev->client->dev, 2000);
    pn544_disable_irq(pn544_dev);
    /* HiKey Compilation fix */
    #ifndef HiKey_620_COMPILATION_FIX
    if (sIsWakeLocked == false)
    {
        pr_info("%s : sIsWakeLocked == false after pn544_disable_irq(pn544_dev); \n", __func__ );
        wake_lock(&nfc_wake_lock);
        sIsWakeLocked = true;

    } else {
            pr_debug("%s already wake locked!\n", __func__);
            pr_info("%s : sIsWakeLocked == true after pn544_disable_irq(pn544_dev); \n", __func__ );
    }
    #endif
    /* Wake up waiting readers */
    wake_up(&pn544_dev->read_wq);


    if ( KCDEBUG ) pr_info("%s End \n", __func__ );

    return IRQ_HANDLED;
}

ssize_t pn544_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev *pn544_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    if ( KCDEBUG ) pr_info("%s start \n", __func__ );
    
    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    //pr_debug("%s : reading   %zu bytes.\n", __func__, count);

    mutex_lock(&pn544_dev->read_mutex);

    if (!gpio_get_value(pn544_dev->irq_gpio)) {
            if ( KCDEBUG ) pr_info("%s : !gpio_get_value(pn544_dev->irq_gpio) \n", __func__ );

        if (filp->f_flags & O_NONBLOCK) {
            if ( KCDEBUG ) pr_info("%s : filp->f_flags & O_NONBLOCK \n", __func__ );

            ret = -EAGAIN;
            goto fail;
        }

        while (1) {
            if ( KCDEBUG ) pr_info("%s enable_irq() \n", __func__ );
            pn544_dev->irq_enabled = true;
            enable_irq(pn544_dev->client->irq);
            if ( KCDEBUG ) pr_info("%s enable_irq_wake() \n", __func__ );
            enable_irq_wake(pn544_dev->client->irq);
            ret = wait_event_interruptible(
                    pn544_dev->read_wq,
                    !pn544_dev->irq_enabled);

//            pn544_disable_irq(pn544_dev);

            if (ret){
                if ( KCDEBUG ) pr_info("%s : *1 \n", __func__ );
                goto fail;
            }
            pr_info("%s pn544_disable_irq() \n", __func__ );            
            pn544_disable_irq(pn544_dev);
            
            if(pn544_dev->state_flags & P544_FLAG_NFC_VEN_RESET) {
                pr_warning("%s: releasing read  \n", __func__);
                pn544_dev->state_flags &= ~P544_FLAG_NFC_VEN_RESET;
                ret =  -EL3RST;
                if ( KCDEBUG ) pr_info("%s : *2 \n", __func__ );

                goto fail;
            }
            if (gpio_get_value(pn544_dev->irq_gpio))
            {
                if ( KCDEBUG ) pr_info("%s : *3 \n", __func__ );
                break;
            }

            pr_warning("%s: spurious interrupt detected\n", __func__);
        }
    }

    /* Read data */
    ret = i2c_master_recv(pn544_dev->client, tmp, count);
    #ifndef HiKey_620_COMPILATION_FIX
    /* HiKey Compilation fix */
    if (sIsWakeLocked == true) {
        if ( KCDEBUG ) pr_info("%s : sIsWakeLocked == true after i2c_master_recv(pn544_dev->client, tmp, count); \n", __func__ );
        wake_unlock(&nfc_wake_lock);
        sIsWakeLocked = false;

    }
    else
    {
        if ( KCDEBUG ) pr_info("%s : sIsWakeLocked == false after i2c_master_recv(pn544_dev->client, tmp, count); \n", __func__ );
    }
    #endif


    /* If ese cold reset has been requested then read the response */
    if(IS_PROP_CMD_REQUESTED(pn544_dev->state_flags) && IS_PROP_RSP(tmp)) {
      rcv_prop_resp_status(tmp);
      /* Request is from driver, consume the response */
      mutex_unlock(&pn544_dev->read_mutex);
      if ( KCDEBUG ) pr_info("%s End: IS_PROP_CMD_REQUESTED(pn544_dev->state_flags) && IS_PROP_RSP(tmp) \n", __func__ );
      return 0;
    }
    mutex_unlock(&pn544_dev->read_mutex);

    /* pn544 seems to be slow in handling I2C read requests
     * so add 1ms delay after recv operation */
#if !NEXUS5x
    udelay(1000);
#endif

    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        if ( KCDEBUG ) pr_info("%s End \n", __func__ );
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
                __func__, ret);
       if ( KCDEBUG )  pr_info("%s End \n", __func__ );
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
		if ( KCDEBUG ) pr_info("%s End \n", __func__ );
        return -EFAULT;
    }
    
	if ( KCDEBUG ) pr_info("%s End \n", __func__ );
    return ret;

    fail:
    mutex_unlock(&pn544_dev->read_mutex);
	if ( KCDEBUG ) pr_info("%s End: fail:mutex_unlock(&pn544_dev->read_mutex); \n", __func__ );
    return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn544_dev  *pn544_dev;
    char tmp[MAX_BUFFER_SIZE];
    int ret;

    pn544_dev = filp->private_data;

	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    
    if (count > MAX_BUFFER_SIZE)
        count = MAX_BUFFER_SIZE;

    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    //pr_debug("%s : writing %zu bytes.\n", __func__, count);
    /* Write data */
    ret = i2c_master_send(pn544_dev->client, tmp, count);
    if (ret != count) {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }
    /* pn544 seems to be slow in handling I2C write requests
     * so add 1ms delay after I2C send oparation */
    udelay(1000);

	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
    return ret;
}

static void p61_update_access_state(struct pn544_dev *pn544_dev, p61_access_state_t current_state, bool set)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s: Enter current_state = %x\n", __func__, pn544_dev->p61_current_state);
    if (current_state)
    {
        if(set){
            if(pn544_dev->p61_current_state == P61_STATE_IDLE)
                pn544_dev->p61_current_state = P61_STATE_INVALID;
            pn544_dev->p61_current_state |= current_state;
        }
        else{
            pn544_dev->p61_current_state ^= current_state;
            if(!pn544_dev->p61_current_state)
                pn544_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    pr_info("%s: Exit current_state = %x\n", __func__, pn544_dev->p61_current_state);
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
}

static void p61_get_access_state(struct pn544_dev *pn544_dev, p61_access_state_t *current_state)
{
	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );
    
    if (current_state == NULL) {
        //*current_state = P61_STATE_INVALID;
        pr_err("%s : invalid state of p61_access_state_t current state  \n", __func__);
    } else {
        *current_state = pn544_dev->p61_current_state;
    }
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );
}
static void p61_access_lock(struct pn544_dev *pn544_dev)
{
	if ( KCDEBUG )	pr_info("%s Start \n", __func__ );
    mutex_lock(&pn544_dev->p61_state_mutex);
	if ( KCDEBUG )	pr_info("%s End \n", __func__ );
}
static void p61_access_unlock(struct pn544_dev *pn544_dev)
{
	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );
    mutex_unlock(&pn544_dev->p61_state_mutex);
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );
}

static int signal_handler(p61_access_state_t state, long nfc_pid)
{
    struct siginfo sinfo;
    pid_t pid;
    struct task_struct *task;
    int sigret = 0, ret = 0;
	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );

    if(nfc_pid == 0)
    {
        pr_info("nfc_pid is clear don't call signal_handler.\n");
    }
    else
    {
        memset(&sinfo, 0, sizeof(struct siginfo));
        sinfo.si_signo = SIG_NFC;
        sinfo.si_code = SI_QUEUE;
        sinfo.si_int = state;
        pid = nfc_pid;

        task = pid_task(find_vpid(pid), PIDTYPE_PID);
        if(task)
        {
            pr_info("%s.\n", task->comm);
            sigret = send_sig_info(SIG_NFC, &sinfo, task);
            if(sigret < 0){
                pr_info("send_sig_info failed..... sigret %d.\n", sigret);
                ret = -1;
                //msleep(60);
            }
        }
        else{
             pr_info("finding task from PID failed\r\n");
             ret = -1;
        }
    }
    pr_info("%s: Exit ret = %d\n", __func__, ret);
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );
    return ret;
}
static void svdd_sync_onoff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; //100 ms timeout
    unsigned long tempJ = msecs_to_jiffies(timeout);
    
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s: Enter nfc_service_pid: %ld\n", __func__, nfc_service_pid);
    if(nfc_service_pid)
    {
        if (0 == signal_handler(origin, nfc_service_pid))
        {
            sema_init(&svdd_sync_onoff_sema, 0);
            pr_info("Waiting for svdd protection response");
            if(down_timeout(&svdd_sync_onoff_sema, tempJ) != 0)
            {
                pr_info("svdd wait protection: Timeout");
            }
            pr_info("svdd wait protection : released");
        }
    }
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
}
static int release_svdd_wait(void)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    up(&svdd_sync_onoff_sema);
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
    return 0;
}

static void dwp_OnOff(long nfc_service_pid, p61_access_state_t origin)
{
    int timeout = 100; //100 ms timeout
    unsigned long tempJ = msecs_to_jiffies(timeout);
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    if(nfc_service_pid)
    {
        if (0 == signal_handler(origin, nfc_service_pid))
        {
            init_completion(&dwp_onoff_sema);
            if(wait_for_completion_timeout(&dwp_onoff_sema, tempJ) != 0)
            {
                pr_info("Dwp On/off wait protection: Timeout");
            }
            pr_info("Dwp On/Off wait protection : released");
        }
    }
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

}
static int release_dwpOnOff_wait(void)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    complete(&dwp_onoff_sema);
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

    return 0;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
    struct pn544_dev *pn544_dev = container_of(filp->private_data,
            struct pn544_dev,
            pn544_device);

	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );

    filp->private_data = pn544_dev;
    pn544_dev->state_flags |= (P544_FLAG_NFC_ON);
    pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

    return 0;
}

static int set_nfc_pid(unsigned long arg)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s : The NFC Service PID is %ld\n", __func__, arg);
    pn544_dev->nfc_service_pid = arg;

	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

    return 0;
}

static void Control_GPIO_for_preFWDL()
{
    pr_info("%s Start \n", __func__ );
    /*NFC Service called FW dwnld*/
    if (pn544_dev->firm_gpio) {
        p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
        if(platform_mver == PLAT_MVER_DMT1) {
            gpio_set_value(pn544_dev->firm_gpio, 1);
        } else {
            pr_info("%s FW GPIO set to 0x01 \n", __func__ );
            gpio_set_value(pn544_dev->firm_gpio, 1);
            msleep(10);
        }
        pn544_dev->state_flags |= (P544_FLAG_FW_DNLD);
        msleep(10);
    }
    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_postFWDL()
{
    pr_info("%s Start \n", __func__ );
    if (pn544_dev->firm_gpio) {
        gpio_set_value(pn544_dev->firm_gpio, 0);
        pn544_dev->state_flags &= ~(P544_FLAG_FW_DNLD);
    }
    pr_info("%s FW GPIO set to 0x00 >>>>>>>\n", __func__);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_preFWCK()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);
    
    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_postFWCK()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_preRDTAG()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_postRDTAG()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_preWINF()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_postWINF()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

    pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_preSETSB()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo mode 13 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 2 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 2 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

   pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_postSETSB()
{
   pr_info("%s Start\n", __func__);
    // NOP
   pr_info("%s End \n", __func__ );
}

static void Control_GPIO_for_RELSB()
{
   pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "echo mode 13 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 14 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 15 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 16 0 > /sys/devices/platform/pinctrl/mt_gpio"
    spi_ese_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

   pr_info("%s End \n", __func__ );

}

static void Control_GPIO_for_CKCMP()
{
    pr_info("%s Start\n", __func__);

//adb shell "echo dir 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_direction_output(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "sleep .1"
    msleep(100);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "echo dir 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo dir 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo out 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 0 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 0 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 0);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 169 0 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
    nfc_en_fet_g_pinctrl(FET_G_OFF);

//adb shell "echo out 17 0 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->firm_gpio, 0);

//adb shell "echo out 169 1 > /sys/devices/platform/pinctrl/mt_gpio"
//    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
    nfc_en_fet_g_pinctrl(FET_G_ON);

//adb shell "sleep .01"
    msleep(10);

//adb shell "echo out 159 1 > /sys/devices/platform/pinctrl/mt_gpio"
    gpio_set_value(pn544_dev->pwr_en_gpio, 1);

//adb shell "echo pullen 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullen 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo pullsel 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 51 1 > /sys/devices/platform/pinctrl/mt_gpio"
//adb shell "echo mode 50 1 > /sys/devices/platform/pinctrl/mt_gpio"
    i2c_nfc_pinctrl(1);

   pr_info("%s End \n", __func__ );

}

long  pn544_dev_ioctl(struct file *filp, unsigned int cmd,
        unsigned long arg)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s enter!!!! \n", __func__);
    /* Free pass autobahn area, not protected. Use it carefullly. START */
    switch(cmd)
    {
        case P544_GET_ESE_ACCESS:
            return get_ese_lock(P61_STATE_WIRED, arg);
        break;
        case P544_REL_SVDD_WAIT:
            return release_svdd_wait();
        break;
        case P544_SET_NFC_SERVICE_PID:
            return set_nfc_pid(arg);
        break;
        case P544_REL_DWPONOFF_WAIT:
            return release_dwpOnOff_wait();
        break;
        case P544_GET_IRQ_STATUS:
            return gpio_get_value(pn544_dev->irq_gpio);
        break;
        default:
        break;
    }
    /* Free pass autobahn area, not protected. Use it carefullly. END */

    p61_access_lock(pn544_dev);
    switch (cmd) {
    case PN544_SET_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 2) {
            if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO) && (pn544_dev->chip_pwr_scheme != PN80T_EXT_PMU_SCHEME))
            {
                /* NFCC fw/download should not be allowed if p61 is used
                 * by SPI
                 */
                pr_info("%s NFCC should not be allowed to reset/FW download \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
            pn544_dev->nfc_ven_enabled = true;
            if ((pn544_dev->spi_ven_enabled == false && !(pn544_dev->secure_timer_cnt))
            || (pn544_dev->chip_pwr_scheme == PN80T_EXT_PMU_SCHEME))
            {
                /* power on with firmware download (requires hw reset)
                 */
                pr_info("%s power on with firmware\n", __func__);
                if(platform_mver == PLAT_MVER_DMT1) {
                    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);

                    spi_ese_pinctrl(1);
                    msleep(10);
                    if (pn544_dev->firm_gpio) {
                        p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
                        gpio_set_value(pn544_dev->firm_gpio, 1);

                        pn544_dev->state_flags |= (P544_FLAG_FW_DNLD);
                    }

                    msleep(10);
                    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
                    spi_ese_pinctrl(0);
                    msleep(10);
                    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
                    spi_ese_pinctrl(1);
                    msleep(10);
                } else {
                    /* power on */
                    gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_ON);
                    spi_ese_pinctrl(1);
                    msleep(100);
                    i2c_nfc_pinctrl(1);

                    /* start DL mode */
//                    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
                    nfc_en_fet_g_pinctrl(FET_G_ON);
                    msleep(10);
                    if (pn544_dev->firm_gpio) {
                        p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
                        gpio_set_value(pn544_dev->firm_gpio, 1);
                        pn544_dev->state_flags |= (P544_FLAG_FW_DNLD);
                    }
                    msleep(10);
//                    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
                    nfc_en_fet_g_pinctrl(FET_G_OFF);
                    msleep(10);
                }
            }
        } else if (arg == 1) {
            /* power on */
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                if(current_state & P61_STATE_DWNLD){
                    p61_update_access_state(pn544_dev, P61_STATE_DWNLD, false);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
                pn544_dev->state_flags &= ~(P544_FLAG_FW_DNLD);
            }

            pn544_dev->nfc_ven_enabled = true;
            #ifndef VEN_ALWAYS_ON
            if (pn544_dev->spi_ven_enabled == false || (pn544_dev->chip_pwr_scheme == PN80T_EXT_PMU_SCHEME)) {
                if(platform_mver == PLAT_MVER_DMT1) {
                    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
                    spi_ese_pinctrl(1);
                } else {
                    gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_ON);
                    spi_ese_pinctrl(1);
                    msleep(100);
                    i2c_nfc_pinctrl(1);
                }
            }
            #endif
        } else if (arg == 0) {
            /* power off */
            if (pn544_dev->firm_gpio) {
                if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0){
                    p61_update_access_state(pn544_dev, P61_STATE_IDLE, true);
                }
                gpio_set_value(pn544_dev->firm_gpio, 0);
            }

            pn544_dev->nfc_ven_enabled = false;
            /* Don't change Ven state if spi made it high */
            #ifndef VEN_ALWAYS_ON
            if ((pn544_dev->spi_ven_enabled == false && !(pn544_dev->secure_timer_cnt))
            || (pn544_dev->chip_pwr_scheme == PN80T_EXT_PMU_SCHEME)) {
                if(platform_mver == PLAT_MVER_DMT1) {
                    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
                    spi_ese_pinctrl(0);
                } else {
//                    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
                    nfc_en_fet_g_pinctrl(FET_G_ON);
                    i2c_nfc_pinctrl(0);
                    msleep(10);
                    //
//                    gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
                    spi_ese_pinctrl(0);
                    msleep(10);
//                    gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
                    nfc_en_fet_g_pinctrl(FET_G_OFF);
                    msleep(10);
                }
            }
            #endif
            /* HiKey Compilation fix */
            #ifndef HiKey_620_COMPILATION_FIX
            if (sIsWakeLocked == true) {
                pr_info("%s : sIsWakeLocked == true after if (arg == 0) \n", __func__ );
                wake_unlock(&nfc_wake_lock);
                sIsWakeLocked = false;                

            }
            else{
                 pr_info("%s : sIsWakeLocked == false after if (arg == 0) \n", __func__ );
            }
            #endif
        } else if (arg == 3) {
            /*NFC Service called ISO-RST*/
            p61_access_state_t current_state = P61_STATE_INVALID;
            p61_get_access_state(pn544_dev, &current_state);
            if(current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
            if(current_state & P61_STATE_WIRED) {
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, false);
            }
#ifdef ISO_RST
            gpio_set_value(pn544_dev->iso_rst_gpio, 0);
            msleep(50);
            gpio_set_value(pn544_dev->iso_rst_gpio, 1);
            msleep(50);
            pr_info("%s ISO RESET from DWP DONE\n", __func__);
#endif
        } else if (arg == 4) {
            pr_info("%s FW dwldioctl called from NFC \n", __func__);
            /*NFC Service called FW dwnld*/
            if (pn544_dev->firm_gpio) {
                p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
                if(platform_mver == PLAT_MVER_DMT1) {
                    gpio_set_value(pn544_dev->firm_gpio, 1);
                } else {
                    gpio_set_value(pn544_dev->firm_gpio, 1);
                    msleep(10);
                }
                pn544_dev->state_flags |= (P544_FLAG_FW_DNLD);
                msleep(10);
            }
        } else if (arg == 5) {
            pn544_dev->state_flags |= P544_FLAG_NFC_VEN_RESET;
            pn544_disable_irq(pn544_dev);
            wake_up(&pn544_dev->read_wq);
            msleep(10);
            if(platform_mver == PLAT_MVER_DMT1) {
                gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
                spi_ese_pinctrl(0);
                msleep(10);
                gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
                spi_ese_pinctrl(1);
                msleep(10);
            } else {
//                gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
                nfc_en_fet_g_pinctrl(FET_G_ON);
                msleep(10);
//                gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
                nfc_en_fet_g_pinctrl(FET_G_OFF);
                msleep(10);
            }
            pr_info("%s VEN reset DONE >>>>>>>\n", __func__);
        }  else if (arg == 6) {
            if (pn544_dev->firm_gpio) {
                gpio_set_value(pn544_dev->firm_gpio, 0);
                pn544_dev->state_flags &= ~(P544_FLAG_FW_DNLD);
            }
            pr_info("%s FW GPIO set to 0x00 >>>>>>>\n", __func__);
            
        }  else if (arg == 10 || arg == 11 || arg == 12 || arg == 13 || arg == 14 || arg == 15) {
            
            pr_info("%s Control GPIO function for kflc_gpio \n", __func__);            
            
            Control_GPIO_for_kflc_gpio(arg);
            
            pr_info("%s Control GPIO function for kflc_gpio \n", __func__);
            
        }else {
            pr_err("%s bad arg %lu\n", __func__, arg);
            /* changed the p61 state to idle*/
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P61_SET_SPI_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1) {
            pr_info("%s : PN61_SET_SPI_PWR - power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
            {
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                /*To handle triple mode protection signal
                NFC service when SPI session started*/
                if (!(current_state & P61_STATE_JCP_DWNLD)){
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        /*signal_handler(P61_STATE_SPI, pn544_dev->nfc_service_pid);*/
                        dwp_OnOff(pn544_dev->nfc_service_pid, P61_STATE_SPI);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
                pn544_dev->spi_ven_enabled = true;

                if(pn544_dev->chip_pwr_scheme == PN80T_EXT_PMU_SCHEME)
                    break;
                #ifndef VEN_ALWAYS_ON
                if (pn544_dev->nfc_ven_enabled == false)
                {
                    /* provide power to NFCC if, NFC service not provided */
                    if(platform_mver == PLAT_MVER_DMT1) {
                        gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
                        spi_ese_pinctrl(1);
                        msleep(10);
                    } else {
                        gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_ON);
                        spi_ese_pinctrl(1);
                        msleep(100);
                        i2c_nfc_pinctrl(1);
                    }
                }
                #endif
                /* pull the gpio to high once NFCC is power on*/
//                gpio_set_value(pn544_dev->ese_pwr_gpio, 1);

                /* Delay (10ms) after SVDD_PWR_ON to allow JCOP to bootup (5ms jcop boot time + 5ms guard time) */
                usleep_range(10000, 12000);

            } else {
                pr_info("%s : PN61_SET_SPI_PWR -  power on ese failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : PN61_SET_SPI_PWR - power off ese\n", __func__);
            if(current_state & P61_STATE_SPI_PRIO){
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                if (!(current_state & P61_STATE_JCP_DWNLD))
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        if(!(current_state & P61_STATE_WIRED))
                        {
                            svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START |
                                                     P61_STATE_SPI_PRIO_END);
                        }else {
                            signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                        }
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                } else if (!(current_state & P61_STATE_WIRED)) {
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
                }
                pn544_dev->spi_ven_enabled = false;

                if(pn544_dev->chip_pwr_scheme == PN80T_EXT_PMU_SCHEME)
                    break;

                /* if secure timer is running, Delay the SPI close by 25ms after sending End of Apdu to enable eSE go into DPD
                    gracefully (20ms after EOS + 5ms DPD settlement time) */
                if(pn544_dev->secure_timer_cnt)
                    usleep_range(25000, 30000);

                if (!(current_state & P61_STATE_WIRED) && !(pn544_dev->secure_timer_cnt))
                {
#ifndef JCOP_4X_VALIDATION
//                    gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                    /* Delay (2.5ms) after SVDD_PWR_OFF for the shutdown settlement time */
                    usleep_range(2500, 3000);
#endif
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                }
#ifndef JCOP_4X_VALIDATION
                #ifndef VEN_ALWAYS_ON
                if ((pn544_dev->nfc_ven_enabled == false) && !(pn544_dev->secure_timer_cnt)) {
                    if(platform_mver == PLAT_MVER_DMT1) {
                         gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
                         spi_ese_pinctrl(0);
                         msleep(10);
                    } else {
//                        gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
                        nfc_en_fet_g_pinctrl(FET_G_ON);
                        i2c_nfc_pinctrl(0);
                        msleep(10);
                        gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
                        spi_ese_pinctrl(0);
                        msleep(10);
//                        gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
                        nfc_en_fet_g_pinctrl(FET_G_OFF);
                        msleep(10);
                    }
                }
                #endif
#endif
              }else if(current_state & P61_STATE_SPI){
                  p61_update_access_state(pn544_dev, P61_STATE_SPI, false);
                  if (!(current_state & P61_STATE_WIRED) &&
                      (pn544_dev->chip_pwr_scheme != PN80T_EXT_PMU_SCHEME) &&
                      !(current_state & P61_STATE_JCP_DWNLD))
                  {
                      if(pn544_dev->nfc_service_pid){
                          pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                          svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
                       }
                       else{
                           pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                       }
                       /* if secure timer is running, Delay the SPI close by 25ms after sending End of Apdu to enable eSE go into DPD
                            gracefully (20ms after EOS + 5ms DPD settlement time) */
                       if(pn544_dev->secure_timer_cnt)
                            usleep_range(25000, 30000);

                      if (!(pn544_dev->secure_timer_cnt)) {
#ifndef JCOP_4X_VALIDATION
//                          gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                          /* Delay (2.5ms) after SVDD_PWR_OFF for the shutdown settlement time */
                          usleep_range(2500, 3000);
#endif
                          svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                       }
                  }
                  /*If JCOP3.2 or 3.3 for handling triple mode
                  protection signal NFC service */
                  else
                  {
                      if (!(current_state & P61_STATE_JCP_DWNLD))
                      {
                          if(pn544_dev->nfc_service_pid){
                              pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                              if(pn544_dev->chip_pwr_scheme == PN80T_LEGACY_PWR_SCHEME)
                              {
                                  svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START | P61_STATE_SPI_END);
                              } else {
                                  signal_handler(P61_STATE_SPI_END, pn544_dev->nfc_service_pid);
                              }
                           }
                           else{
                               pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                           }
                      } else if (pn544_dev->chip_pwr_scheme == PN80T_LEGACY_PWR_SCHEME) {
                          svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
                      }
                      if(pn544_dev->chip_pwr_scheme == PN80T_LEGACY_PWR_SCHEME)
                      {
//#ifndef JCOP_4X_VALIDATION
//                          gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
//#endif
                          svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
//                          pr_info("PN80T legacy ese_pwr_gpio off %s", __func__);
                      }
                  }
                  pn544_dev->spi_ven_enabled = false;
#ifndef VEN_ALWAYS_ON
                  if (pn544_dev->nfc_ven_enabled == false && (pn544_dev->chip_pwr_scheme != PN80T_EXT_PMU_SCHEME)
                       && !(pn544_dev->secure_timer_cnt)) {
                    if(platform_mver == PLAT_MVER_DMT1) {
                        gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
                        spi_ese_pinctrl(0);
                        msleep(10);
                    } else {
//                        gpio_set_value(pn544_dev->fet_g_gpio, FET_G_ON);
                        nfc_en_fet_g_pinctrl(FET_G_ON);
                        i2c_nfc_pinctrl(0);
                        msleep(10);
                        gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
                        spi_ese_pinctrl(0);
                        msleep(10);
//                        gpio_set_value(pn544_dev->fet_g_gpio, FET_G_OFF);
                        nfc_en_fet_g_pinctrl(FET_G_OFF);
                        msleep(10);
                    }
                  }
#endif
            } else {
                pr_err("%s : PN61_SET_SPI_PWR - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }else if (arg == 2) {
            pr_info("%s : PN61_SET_SPI_PWR - reset\n", __func__);
            if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
                if (pn544_dev->spi_ven_enabled == false)
                {
                    pn544_dev->spi_ven_enabled = true;
                    #ifndef VEN_ALWAYS_ON
                    if ((pn544_dev->nfc_ven_enabled == false) && (pn544_dev->chip_pwr_scheme != PN80T_EXT_PMU_SCHEME)) {
                        /* provide power to NFCC if, NFC service not provided */
                        if(platform_mver == PLAT_MVER_DMT1) {
                            gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
                            spi_ese_pinctrl(1);
                            msleep(10);
                        } else {
                            gpio_set_value(pn544_dev->pwr_en_gpio, PWR_EN_ON);
                            spi_ese_pinctrl(1);
                            msleep(100);
                            i2c_nfc_pinctrl(1);
                        }
                    }
                    #endif
                }
                if(pn544_dev->chip_pwr_scheme != PN80T_EXT_PMU_SCHEME  && !(pn544_dev->secure_timer_cnt))
                {
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
//#ifndef JCOP_4X_VALIDATION
//                    gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
//#endif
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                    msleep(10);
//                    if(!gpio_get_value(pn544_dev->ese_pwr_gpio))
//                        gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
//                    msleep(10);
                }
            } else {
                pr_info("%s : PN61_SET_SPI_PWR - reset  failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 3) {
            int ret = ese_cold_reset(ESE_COLD_RESET_SOURCE_NFC);
            p61_access_unlock(pn544_dev);
            return ret;
        }else if (arg == 4) {
            if (current_state & P61_STATE_SPI_PRIO)
            {
                pr_info("%s : PN61_SET_SPI_PWR - Prio Session Ending...\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
                /*after SPI prio timeout, the state is changing from SPI prio to SPI */
                p61_update_access_state(pn544_dev, P61_STATE_SPI, true);
                if (current_state & P61_STATE_WIRED)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
               }
            }
            else
            {
                pr_info("%s : PN61_SET_SPI_PWR -  Prio Session End failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBADRQC; /* Device or resource busy */
            }
        } else if(arg == 5){
            release_ese_lock(P61_STATE_SPI);
        } else if (arg == 6) {
            /*SPI Service called ISO-RST*/
            p61_access_state_t current_state = P61_STATE_INVALID;
            p61_get_access_state(pn544_dev, &current_state);
            if(current_state & P61_STATE_WIRED) {
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
            if(current_state & P61_STATE_SPI) {
                p61_update_access_state(pn544_dev, P61_STATE_SPI, false);
            }else if(current_state & P61_STATE_SPI_PRIO) {
                p61_update_access_state(pn544_dev, P61_STATE_SPI_PRIO, false);
            }
#ifdef ISO_RST
            gpio_set_value(pn544_dev->iso_rst_gpio, 0);
            msleep(50);
            gpio_set_value(pn544_dev->iso_rst_gpio, 1);
            msleep(50);
            pr_info("%s ISO RESET from SPI DONE\n", __func__);
#endif
        } else if(arg == 7){
          long ret;
          set_force_reset(true);
          ret = do_reset_protection(true);
        } else if(arg == 8){
          long ret;
          set_force_reset(false);
          ret = do_reset_protection(false);
        } else {
            pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
            p61_access_unlock(pn544_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;

    case P61_GET_PWR_STATUS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        pr_info("%s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
    }
    break;

    case PN544_SET_DWNLD_STATUS:
    {
        long ret;
        ret = set_jcop_download_state(arg);
        if(ret < 0)
        {
            p61_access_unlock(pn544_dev);
            return ret;
        }
    }
    break;

    case P61_SET_WIRED_ACCESS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn544_dev, &current_state);
        if (arg == 1)
        {
            if (current_state)
            {
                pr_info("%s : P61_SET_WIRED_ACCESS - enabling\n", __func__);
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, true);
                if (current_state & P61_STATE_SPI_PRIO)
                {
                    if(pn544_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn544_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                    }
                }
//                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0 && (pn544_dev->chip_pwr_scheme == PN67T_PWR_SCHEME))
//                    gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
            } else {
                pr_info("%s : P61_SET_WIRED_ACCESS -  enabling failed \n", __func__);
                p61_access_unlock(pn544_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : P61_SET_WIRED_ACCESS - disabling \n", __func__);
            if (current_state & P61_STATE_WIRED){
                p61_update_access_state(pn544_dev, P61_STATE_WIRED, false);
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0 && (pn544_dev->chip_pwr_scheme == PN67T_PWR_SCHEME))
                {
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
//                    gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                    svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
                }
            } else {
                pr_err("%s : P61_SET_WIRED_ACCESS - failed, current_state = %x \n",
                        __func__, pn544_dev->p61_current_state);
                p61_access_unlock(pn544_dev);
                return -EPERM; /* Operation not permitted */
            }
        }
        else if(arg == 2)
        {
             pr_info("%s : P61_ESE_GPIO_LOW  \n", __func__);
             if(pn544_dev->chip_pwr_scheme == PN67T_PWR_SCHEME)
             {
                 svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_START);
//                 gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
                 svdd_sync_onoff(pn544_dev->nfc_service_pid, P61_STATE_SPI_SVDD_SYNC_END);
             }
        }
        else if(arg == 3)
        {
//            pr_info("%s : P61_ESE_GPIO_HIGH  \n", __func__);
//            if(pn544_dev->chip_pwr_scheme == PN67T_PWR_SCHEME)
//            gpio_set_value(pn544_dev->ese_pwr_gpio, 1);
        }
        else if(arg == 4)
        {
            release_ese_lock(P61_STATE_WIRED);
        }
        else {
             pr_info("%s P61_SET_WIRED_ACCESS - bad arg %lu\n", __func__, arg);
             p61_access_unlock(pn544_dev);
             return -EBADRQC; /* Invalid request code */
        }
    }
    break;
    case P544_SET_POWER_SCHEME:
    {
        if(arg == PN67T_PWR_SCHEME)
        {
            pn544_dev->chip_pwr_scheme = PN67T_PWR_SCHEME;
            pr_info("%s : The power scheme is set to PN67T legacy \n", __func__);
        }
        else if(arg == PN80T_LEGACY_PWR_SCHEME)
        {
            pn544_dev->chip_pwr_scheme = PN80T_LEGACY_PWR_SCHEME;
            pr_info("%s : The power scheme is set to PN80T_LEGACY_PWR_SCHEME,\n", __func__);
        }
        else if(arg == PN80T_EXT_PMU_SCHEME)
        {
            pn544_dev->chip_pwr_scheme = PN80T_EXT_PMU_SCHEME;
            pr_info("%s : The power scheme is set to PN80T_EXT_PMU_SCHEME,\n", __func__);
        }
        else
        {
            pr_info("%s : The power scheme is invalid,\n", __func__);
        }
    }
    break;
    case P544_SECURE_TIMER_SESSION:
    {
       secure_timer_operation(pn544_dev, arg);
    }
    break;
    
    
    case PN544_SET_FUNC:
    
		pr_info("%s PN544_SET_FUNC \n", __func__);    
		if		(arg == 0)		i2c_nfc_pinctrl(1); //active  I2C
		else if (arg == 1)		i2c_nfc_pinctrl(0); //suspend I2C
		else if (arg == 2)		spi_ese_pinctrl(1); //active  SPI
		else if (arg == 3)		spi_ese_pinctrl(0); //suspend SPI
		pr_info("%s PN544_SET_FUNC \n", __func__);    
    
    break;
    case P544_SET_FWDL_GPIO:
    {
        if(arg == 0) {
            Control_GPIO_for_preFWDL();
        } else if (arg == 1) {
            Control_GPIO_for_postFWDL();
        } else {
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P544_SET_FWCK_GPIO:
    {
        if(arg == 0) {
            Control_GPIO_for_preFWCK();
        } else if (arg == 1) {
            Control_GPIO_for_postFWCK();
        } else {
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P544_SET_RDTAG_GPIO:
    {
        if(arg == 0) {
            Control_GPIO_for_preRDTAG();
        } else if (arg == 1) {
            Control_GPIO_for_postRDTAG();
        } else {
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P544_SET_WINF_GPIO:
    {
        if(arg == 0) {
            Control_GPIO_for_preWINF();
        } else if (arg == 1) {
            Control_GPIO_for_postWINF();
        } else {
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P544_SET_SETSB_GPIO:
    {
        if(arg == 0) {
            Control_GPIO_for_preSETSB();
        } else if (arg == 1) {
            Control_GPIO_for_postSETSB();
        } else {
            pr_err("%s bad ioctl %u\n", __func__, cmd);
            p61_access_unlock(pn544_dev);
            return -EINVAL;
        }
    }
    break;
    case P544_SET_RELSB_GPIO:
    {
        Control_GPIO_for_RELSB();
    }
    break;
    case P544_SET_CKCMP_GPIO:
    {
        Control_GPIO_for_CKCMP();
    }
    break;
    default:
        pr_err("%s bad ioctl %u\n", __func__, cmd);
        p61_access_unlock(pn544_dev);
        return -EINVAL;
    }
    p61_access_unlock(pn544_dev);
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

    return 0;
}
EXPORT_SYMBOL(pn544_dev_ioctl);

static void secure_timer_workqueue(struct work_struct *Wq)
{
  p61_access_state_t current_state = P61_STATE_INVALID;
  printk( KERN_INFO "secure_timer_callback: called (%lu).\n", jiffies);
  /* Locking the critical section: ESE_PWR_OFF to allow eSE to shutdown peacefully :: START */
  get_ese_lock(P61_STATE_WIRED, MAX_ESE_ACCESS_TIME_OUT_MS);
  p61_update_access_state(pn544_dev, P61_STATE_SECURE_MODE, false);
  p61_get_access_state(pn544_dev, &current_state);


	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );


  if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
  {
//      printk( KERN_INFO "secure_timer_callback: make se_pwer_gpio low, state = %d", current_state);
//      gpio_set_value(pn544_dev->ese_pwr_gpio, 0);
      /* Delay (2.5ms) after SVDD_PWR_OFF for the shutdown settlement time */
      usleep_range(2500, 3000);
      #ifndef VEN_ALWAYS_ON
      if(pn544_dev->nfc_service_pid == 0x00)
      {
          gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
          spi_ese_pinctrl(0);
          printk( KERN_INFO "secure_timer_callback :make ven_gpio low, state = %d", current_state);
      }
      #endif
  }
  pn544_dev->secure_timer_cnt = 0;
  /* Locking the critical section: ESE_PWR_OFF to allow eSE to shutdown peacefully :: END */
  release_ese_lock(P61_STATE_WIRED);
  
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );

  return;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
static void secure_timer_callback( unsigned long data )
{
(void)data;
#else
static void secure_timer_callback(struct timer_list *unused)
{
#endif
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );


    /* Flush and push the timer callback event to the bottom half(work queue)
    to be executed later, at a safer time */
    flush_workqueue(pn544_dev->pSecureTimerCbWq);
    queue_work(pn544_dev->pSecureTimerCbWq, &pn544_dev->wq_task);
    
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );

    return;
}

static long start_seccure_timer(unsigned long timer_value)
{
    long ret = -EINVAL;
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
   

    /* Delete the timer if timer pending */
    if(timer_pending(&secure_timer) == 1)
    {
        pr_info("start_seccure_timer: delete pending timer \n");
        /* delete timer if already pending */
        del_timer(&secure_timer);
    }
    /* Start the timer if timer value is non-zero */
    if(timer_value)
    {
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0)
        init_timer(&secure_timer);
        setup_timer( &secure_timer, secure_timer_callback, 0 );
#else
        timer_setup(&secure_timer, secure_timer_callback, 0);
#endif
        pr_info("start_seccure_timer: timeout %lums (%lu)\n",timer_value, jiffies );
        ret = mod_timer( &secure_timer, jiffies + msecs_to_jiffies(timer_value));
        if (ret)
            pr_info("start_seccure_timer: Error in mod_timer\n");
    }
    
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
    
    return ret;
}

static long secure_timer_operation(struct pn544_dev *pn544_dev, unsigned long arg)
{
    long ret = -EINVAL;
    unsigned long timer_value =  arg;

	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );

    printk( KERN_INFO "secure_timer_operation, %d\n",pn544_dev->chip_pwr_scheme);
    if(pn544_dev->chip_pwr_scheme == PN80T_LEGACY_PWR_SCHEME)
    {
        ret = start_seccure_timer(timer_value);
        if(!ret)
        {
            pn544_dev->secure_timer_cnt  = 1;
            p61_update_access_state(pn544_dev, P61_STATE_SECURE_MODE, true);
        }
        else
        {
            pn544_dev->secure_timer_cnt  = 0;
            p61_update_access_state(pn544_dev, P61_STATE_SECURE_MODE, false);
            pr_info("%s :Secure timer reset \n", __func__);
        }
    }
    else
    {
        pr_info("%s :Secure timer session not applicable  \n", __func__);
    }

	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
    
    return ret;
}

static long set_jcop_download_state(unsigned long arg)
{
        p61_access_state_t current_state = P61_STATE_INVALID;
        long ret = 0;
        p61_get_access_state(pn544_dev, &current_state);
		if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
        pr_info("%s:Enter PN544_SET_DWNLD_STATUS:JCOP Dwnld state arg = %ld",__func__, arg);
        if(arg == JCP_DWNLD_INIT)
        {
            if(pn544_dev->nfc_service_pid)
            {
                pr_info("nfc service pid %s   ---- %ld", __func__, pn544_dev->nfc_service_pid);
                signal_handler((p61_access_state_t)JCP_DWNLD_INIT, pn544_dev->nfc_service_pid);
            }
            else
            {
                if (current_state & P61_STATE_JCP_DWNLD)
                {
                    ret = -EINVAL;
                }
                else
                {
                    p61_update_access_state(pn544_dev, P61_STATE_JCP_DWNLD, true);
                }
            }
        }
        else if (arg == JCP_DWNLD_START)
        {
            if (current_state & P61_STATE_JCP_DWNLD)
            {
                ret = -EINVAL;
            }
            else
            {
                p61_update_access_state(pn544_dev, P61_STATE_JCP_DWNLD, true);
            }
        }
        else if (arg == JCP_SPI_DWNLD_COMPLETE)
        {
            if(pn544_dev->nfc_service_pid)
            {
                signal_handler((p61_access_state_t)JCP_DWP_DWNLD_COMPLETE, pn544_dev->nfc_service_pid);
            }
            p61_update_access_state(pn544_dev, P61_STATE_JCP_DWNLD, false);
        }
        else if (arg == JCP_DWP_DWNLD_COMPLETE)
        {
            p61_update_access_state(pn544_dev, P61_STATE_JCP_DWNLD, false);
        }
        else
        {
            pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
            p61_access_unlock(pn544_dev);
            return -EBADRQC; /* Invalid request code */
        }
        pr_info("%s: PN544_SET_DWNLD_STATUS  = %x",__func__, current_state);
		if ( KCDEBUG )    pr_info("%s End \n", __func__ );


    return ret;
}

int get_ese_lock(p61_access_state_t  p61_current_state, int timeout)
{
    unsigned long tempJ = msecs_to_jiffies(timeout);
    
	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );
    
    if(down_timeout(&ese_access_sema, tempJ) != 0)
    {
        printk("get_ese_lock: timeout p61_current_state = %d\n", p61_current_state);
        return -EBUSY;
    }
    
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );

    return 0;
}
EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(p61_access_state_t  p61_current_state)
{
	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );
    up(&ese_access_sema);
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );

}


static const struct file_operations pn544_dev_fops = {
        .owner  = THIS_MODULE,
        .llseek = no_llseek,
        .read   = pn544_dev_read,
        .write  = pn544_dev_write,
        .open   = pn544_dev_open,
        .release = pn544_dev_release,
        .unlocked_ioctl  = pn544_dev_ioctl,
};
#if DRAGON_NFC
static int pn544_parse_dt(struct device *dev,
    struct pn544_i2c_platform_data *data)
{
    struct device_node *np = dev->of_node;
    int errorno = 0;
//    int ret;


	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );

#if !NEXUS5x
#if PN553


    /* get platform major version */
//    ret = of_property_read_u32(np, "nfc_i2c_pver", &platform_mver);
//    if(ret) {
//        pr_err("%s : major ver read error!! ret[%d]\n", __func__, ret);
//        return -EINVAL;
//    }
//    pr_info("%s : platform_mver[0x%X]\n", __func__, platform_mver);



        data->irq_gpio = of_get_named_gpio(np, "nxp,pn553-irq", 0);
        if ((!gpio_is_valid(data->irq_gpio)))
                return -EINVAL;

    if(platform_mver == PLAT_MVER_DMT1) {
        data->ven_gpio = of_get_named_gpio(np, "nxp,pn553-ven", 0);
        if ((!gpio_is_valid(data->ven_gpio)))
                return -EINVAL;
    } else {
        data->pwr_en_gpio = of_get_named_gpio(np, "nxp,pn553-pwr-en", 0);
        if ((!gpio_is_valid(data->pwr_en_gpio)))
                return -EINVAL;

        data->fet_g_gpio = of_get_named_gpio(np, "nxp,pn553-fet-g", 0);
        if ((!gpio_is_valid(data->fet_g_gpio)))
                return -EINVAL;
    }
        data->firm_gpio = of_get_named_gpio(np, "nxp,pn553-fw-dwnld", 0);
        if ((!gpio_is_valid(data->firm_gpio)))
                return -EINVAL;
//        data->ese_pwr_gpio = of_get_named_gpio(np, "nxp,pn553-ese-pwr", 0);
//        if ((!gpio_is_valid(data->ese_pwr_gpio)))
//                return -EINVAL;
#ifdef ISO_RST
        data->iso_rst_gpio = of_get_named_gpio(np, "nxp,pn553-iso-pwr-rst", 0);
        if ((!gpio_is_valid(data->iso_rst_gpio)))
                return -EINVAL;
#endif // ISO_RST
#else // PN553
        data->irq_gpio = of_get_named_gpio(np, "nxp,pn544-irq", 0);
        if ((!gpio_is_valid(data->irq_gpio)))
                return -EINVAL;

        data->ven_gpio = of_get_named_gpio(np, "nxp,pn544-ven", 0);
        if ((!gpio_is_valid(data->ven_gpio)))
                return -EINVAL;

        data->firm_gpio = of_get_named_gpio(np, "nxp,pn544-fw-dwnld", 0);
        if ((!gpio_is_valid(data->firm_gpio)))
                return -EINVAL;

//        data->ese_pwr_gpio = of_get_named_gpio(np, "nxp,pn544-ese-pwr", 0);
//        if ((!gpio_is_valid(data->ese_pwr_gpio)))
//                return -EINVAL;
        data->iso_rst_gpio = of_get_named_gpio(np, "nxp,pn544-iso-pwr-rst", 0);
        if ((!gpio_is_valid(data->iso_rst_gpio)))
                return -EINVAL;
#endif // PN553
#else
        data->ven_gpio = of_get_named_gpio_flags(np,
                                        "nxp,gpio_ven", 0, NULL);
        data->firm_gpio = of_get_named_gpio_flags(np,
                                        "nxp,gpio_mode", 0, NULL);
        data->irq_gpio = of_get_named_gpio_flags(np,
                                        "nxp,gpio_irq", 0, NULL);
#endif
//    pr_info("%s: %d, %d, %d, %d, %d error:%d\n", __func__,
//        data->irq_gpio, data->ven_gpio, data->firm_gpio, data->iso_rst_gpio,
//        data->ese_pwr_gpio, errorno);
    if(platform_mver == PLAT_MVER_DMT1) {
        pr_info("%s: %d, %d, %d, %d error:%d\n", __func__,
            data->irq_gpio, data->ven_gpio, data->firm_gpio, data->iso_rst_gpio, errorno);
    } else {
        pr_info("%s: %d, %d, %d, %d, %d error:%d\n", __func__,
            data->irq_gpio, data->pwr_en_gpio, data->fet_g_gpio, data->firm_gpio, data->iso_rst_gpio, errorno);
    }

	if ( KCDEBUG )  pr_info("%s End \n", __func__ );

    return errorno;
}
#endif

void i2c_nfc_pinctrl(int status)
{
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s : Enter status[%d]\n", __func__, status);

    /* check param */
    if(pinctrl_i2c == NULL)
    {
        pr_info("%s : pinctrl_i2c is NULL!!\n", __func__);
        return;
    }

    if(i2c_nfc_active == NULL)
    {
        pr_info("%s : i2c_nfc_active is NULL!!\n", __func__);
        return;
    }

    if(i2c_nfc_suspend == NULL)
    {
        pr_info("%s : i2c_nfc_suspend is NULL!!\n", __func__);
        return;
    }

    if (status)
    {
        pinctrl_select_state(pinctrl_i2c, i2c_nfc_active);
    }
    else
    {
        pinctrl_select_state(pinctrl_i2c, i2c_nfc_suspend);
    }

	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
}

void i2c_nfc_pinctrl_init(int status)
{
	if ( KCDEBUG )    pr_info("[NFC DEBUG] %s Start \n", __func__ );
    pr_info("%s : Enter status[%d]\n", __func__, status);

    /* check param */
    if(pinctrl_i2c == NULL)
    {
        pr_info("%s : pinctrl_i2c is NULL!!\n", __func__);
        return;
    }

    if(i2c_nfc_init == NULL)
    {
        pr_info("%s : i2c_nfc_init is NULL!!\n", __func__);
        return;
    }

    if(i2c_nfc_deinit == NULL)
    {
        pr_info("%s : i2c_nfc_deinit is NULL!!\n", __func__);
        return;
    }

    if (status)
    {
        pinctrl_select_state(pinctrl_i2c, i2c_nfc_init);
    }
    else
    {
        pinctrl_select_state(pinctrl_i2c, i2c_nfc_deinit);
    }

	if ( KCDEBUG )    pr_info("[NFC DEBUG] %s End \n", __func__ );
}


void nfc_en_fet_g_pinctrl(int state)
{
    int ret = 0;
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s : Enter state[%d]\n", __func__, state);

    /* check param */
    if(pinctrl_i2c == NULL)
    {
        pr_info("%s : pinctrl_i2c is NULL!!\n", __func__);
        return;
    }

    if(nfc_rst_pre_active == NULL)
    {
        pr_info("%s : nfc_rst_pre_active is NULL!!\n", __func__);
        return;
    }

    if(nfc_rst_active == NULL)
    {
        pr_info("%s : nfc_rst_active is NULL!!\n", __func__);
        return;
    }

    if(nfc_rst_pre_suspend == NULL)
    {
        pr_info("%s : nfc_rst_pre_suspend is NULL!!\n", __func__);
        return;
    }

    if(nfc_rst_suspend == NULL)
    {
        pr_info("%s : nfc_rst_suspend is NULL!!\n", __func__);
        return;
    }

    switch (state) {
    case FET_G_OFF:
        if (gpio_get_value(pn544_dev->fet_g_gpio)){
            if (gpio_get_value(pn544_dev->pwr_en_gpio)){
                //VEN : Low->High
                pr_info("VEN : Low->High sequence.\n", __func__);
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_pre_active);
                if (ret)
                    pr_info("%s : cannot set pin to pre_active state!!\n", __func__);
                else
                    pr_info("%s : can set pin to pre_active state!!\n", __func__);

                msleep(10);
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_active);
                if (ret)
                    pr_info("%s : cannot set pin to active state!!\n", __func__);
                else
                    pr_info("%s : can set pin to active state!!\n", __func__);

                msleep(40);
            }else{
                //VEN : no change
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_active);
                if (ret)
                    pr_info("%s : cannot set pin to active state!!\n", __func__);
            }
        }else{
            //nop
            pr_info("%s : pn544_dev->fet_g_gpio is already set to Low\n", __func__);
        }
        break;
    case FET_G_ON:
        if (gpio_get_value(pn544_dev->fet_g_gpio)){
            //nop
            pr_info("%s : pn544_dev->fet_g_gpio is already set to High\n", __func__);
        }else{
            if (gpio_get_value(pn544_dev->pwr_en_gpio)){
                //VEN : High->Low
                pr_info("VEN : High->Low sequence.\n", __func__);
                
                
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_pre_suspend);
                if (ret)
                    pr_info("%s : cannot set pin to pre_suspend state!!\n", __func__);
                else
                    pr_info("%s : can set pin to pre_suspend state!!\n", __func__);
                    
                msleep(10);
               
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_suspend);
                if (ret)
                    pr_info("%s : cannot set pin to suspend state!!\n", __func__);
                else    
                    pr_info("%s : can set pin to suspend state!!\n", __func__);

                msleep(40);
            }else{
                //VEN : no change
                ret = pinctrl_select_state(pinctrl_i2c, nfc_rst_suspend);
                if (ret)
                    pr_info("%s : cannot set pin to suspend state!!\n", __func__);
            }
        }
        break;
    default :
        pr_info("%s : state error\n", __func__);
        break;
    }

    pr_info("%s : Exit\n", __func__);
	if ( KCDEBUG )    pr_info("%s End \n", __func__ );
}

void nfc_pwr_en_pinctrl(int state)
{
    int ret = 0;
	if ( KCDEBUG )    pr_info("%s Start \n", __func__ );
    pr_info("%s : Enter state[%d]\n", __func__, state);

    /* check param */
    if(pinctrl_i2c == NULL)
    {
        pr_info("%s : pinctrl_i2c is NULL!!\n", __func__);
        return;
    }

    if(pwr_en_active == NULL)
    {
        pr_info("%s : pwr_en_active is NULL!!\n", __func__);
        return;
    }

    if(pwr_en_suspend == NULL)
    {
        pr_info("%s : pwr_en_suspend is NULL!!\n", __func__);
        return;
    }

    switch (state) {
    case 1:
        ret = pinctrl_select_state(pinctrl_i2c, pwr_en_active);
        if (ret)
            pr_info("%s : cannot set pin to pre_active state!!\n", __func__);
        break;
    case 0:
        ret = pinctrl_select_state(pinctrl_i2c, pwr_en_suspend);
        if (ret)
            pr_info("%s : cannot set pin to suspend state!!\n", __func__);
        break;
    default :
        pr_info("%s : state error\n", __func__);
        break;
    }

	if ( KCDEBUG )  pr_info("%s End \n", __func__ );
}

static int pn544_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    int ret;
    int off_charge_mode;
    int factory_mode;
    struct pn544_i2c_platform_data *platform_data;
    //struct pn544_dev *pn544_dev;

#if !DRAGON_NFC
    platform_data = client->dev.platform_data;
#else
    struct device_node *node = client->dev.of_node;
	if ( KCDEBUG )
	{	pr_info("[NFC DEBUG] %s Start \n", __func__ );
// #if IS_BUILTIN(NFC_PN553_DEVICES)
//		msm_gpio_dbg_print();
// #endif
	}

    if (node) {
        platform_data = devm_kzalloc(&client->dev,
            sizeof(struct pn544_i2c_platform_data), GFP_KERNEL);
        if (!platform_data) {
            dev_err(&client->dev,
                "nfc-nci probe: Failed to allocate memory\n");
            return -ENOMEM;
        }
        ret = pn544_parse_dt(&client->dev, platform_data);
        if (ret)
        {
            pr_info("%s pn544_parse_dt failed", __func__);
        }
        client->irq = gpio_to_irq(platform_data->irq_gpio);
        if (client->irq < 0)
        {
            pr_info("%s gpio to irq failed", __func__);
        }
    } else {
        platform_data = client->dev.platform_data;
    }
#endif
    if (platform_data == NULL) {
        pr_err("%s : nfc probe fail\n", __func__);
        return  -ENODEV;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        return  -ENODEV;
    }
#if !DRAGON_NFC
    ret = gpio_request(platform_data->irq_gpio, "nfc_int");
    if (ret)
        return  -ENODEV;
    ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
    if (ret)
        goto err_ven;
//    ret = gpio_request(platform_data->ese_pwr_gpio, "nfc_ese_pwr");
//    if (ret)
//        goto err_ese_pwr;
    if (platform_data->firm_gpio) {
        ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
        if (ret)
            goto err_firm;
    }
#ifdef ISO_RST
    if(platform_data->iso_rst_gpio) {
        ret = gpio_request(platform_data->iso_rst_gpio, "nfc_iso_rst");
        if (ret)
            goto err_iso_rst;
    }
#endif
#endif

    if(platform_mver == PLAT_MVER_DMT2) {
        pinctrl_i2c = devm_pinctrl_get(&client->dev);
        if (IS_ERR(pinctrl_i2c)) {
            ret = PTR_ERR(pinctrl_i2c);
            pr_info("%s :  Cannot find pinctrl_i2c! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        i2c_nfc_active = pinctrl_lookup_state(pinctrl_i2c, "i2c_nfc_active");
        if (IS_ERR(i2c_nfc_active)) {
            ret = PTR_ERR(i2c_nfc_active);
            pr_info("%s :  Cannot find i2c_nfc_active! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        i2c_nfc_suspend = pinctrl_lookup_state(pinctrl_i2c, "i2c_nfc_suspend");
        if (IS_ERR(i2c_nfc_suspend)) {
            ret = PTR_ERR(i2c_nfc_suspend);
            pr_info("%s :  Cannot find i2c_nfc_suspend! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        i2c_nfc_init = pinctrl_lookup_state(pinctrl_i2c, "i2c_nfc_init");
        if (IS_ERR(i2c_nfc_init)) {
            ret = PTR_ERR(i2c_nfc_init);
            pr_info("%s :  Cannot find i2c_nfc_init! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        i2c_nfc_deinit = pinctrl_lookup_state(pinctrl_i2c, "i2c_nfc_deinit");
        if (IS_ERR(i2c_nfc_deinit)) {
            ret = PTR_ERR(i2c_nfc_deinit);
            pr_info("%s :  Cannot find i2c_nfc_deinit! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        nfc_rst_pre_active = pinctrl_lookup_state(pinctrl_i2c, "nfc_rst_pre_active");
        if (IS_ERR(nfc_rst_pre_active)) {
            ret = PTR_ERR(nfc_rst_pre_active);
            pr_info("%s :  Cannot find nfc_rst_pre_active! ret = %d\n", __func__, ret);
            goto err_exit;
        }

        nfc_rst_active = pinctrl_lookup_state(pinctrl_i2c, "nfc_rst_active");
        if (IS_ERR(nfc_rst_active)) {
            ret = PTR_ERR(nfc_rst_active);
            pr_info("%s :  Cannot find nfc_rst_active! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        nfc_rst_pre_suspend = pinctrl_lookup_state(pinctrl_i2c, "nfc_rst_pre_suspend");
        if (IS_ERR(nfc_rst_pre_suspend)) {
            ret = PTR_ERR(nfc_rst_pre_suspend);
            pr_info("%s :  Cannot find nfc_rst_pre_suspend! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        nfc_rst_suspend = pinctrl_lookup_state(pinctrl_i2c, "nfc_rst_suspend");
        if (IS_ERR(nfc_rst_suspend)) {
            ret = PTR_ERR(nfc_rst_suspend);
            pr_info("%s :  Cannot find nfc_rst_suspend! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        pwr_en_active = pinctrl_lookup_state(pinctrl_i2c, "pwr_en_active");
        if (IS_ERR(pwr_en_active)) {
            ret = PTR_ERR(pwr_en_active);
            pr_info("%s :  Cannot find pwr_en_active! ret = %d\n", __func__, ret);
            goto err_exit;
        }
        pwr_en_suspend = pinctrl_lookup_state(pinctrl_i2c, "pwr_en_suspend");
        if (IS_ERR(pwr_en_suspend)) {
            ret = PTR_ERR(pwr_en_suspend);
            pr_info("%s :  Cannot find nfc_rst_suspend! ret = %d\n", __func__, ret);
            goto err_exit;
        }
    }

    pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
    if (pn544_dev == NULL) {
        dev_err(&client->dev,
                "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    pn544_dev->irq_gpio = platform_data->irq_gpio;
    pn544_dev->ven_gpio  = platform_data->ven_gpio;
    pn544_dev->pwr_en_gpio  = platform_data->pwr_en_gpio;
    pn544_dev->fet_g_gpio  = platform_data->fet_g_gpio;
    pn544_dev->firm_gpio  = platform_data->firm_gpio;
//    pn544_dev->ese_pwr_gpio  = platform_data->ese_pwr_gpio;
#ifdef ISO_RST
    pn544_dev->iso_rst_gpio = platform_data->iso_rst_gpio;
#endif
    pn544_dev->p61_current_state = P61_STATE_IDLE;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;
//  pn544_dev->chip_pwr_scheme = PN67T_PWR_SCHEME;
    pn544_dev->chip_pwr_scheme = PN80T_LEGACY_PWR_SCHEME;
    pn544_dev->client   = client;
    pn544_dev->secure_timer_cnt = 0;

    pn544_dev->state_flags = 0x00;
    ret = gpio_direction_input(pn544_dev->irq_gpio);
    if (ret < 0) {
        pr_err("%s :not able to set irq_gpio as input\n", __func__);
        goto err_ven;
    }
    /* Get BootMode */
    off_charge_mode = 0;
    factory_mode = 0;

    off_charge_mode = (strstr(saved_command_line, "androidboot.mode=kccharger") != NULL);
    factory_mode = (strstr(saved_command_line, "androidboot.mode=kcfactory") != NULL);

    #ifndef VEN_ALWAYS_ON
    if((off_charge_mode == 1) || (factory_mode == 1) ) {
        /* VEN OFF */
        if(platform_mver == PLAT_MVER_DMT1) {
            ret = gpio_direction_output(pn544_dev->ven_gpio, VEN_PWR_OFF);
            if (ret < 0) {
                pr_err("%s : not able to set ven_gpio as output\n", __func__);
                goto err_firm;
            }
        } else {
            ret = gpio_direction_output(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
            if (ret < 0) {
                pr_err("%s : not able to set pwr_en_gpio as output\n", __func__);
                goto err_firm;
            }

//            ret = gpio_direction_output(pn544_dev->fet_g_gpio, FET_G_OFF);
//            if (ret < 0) {
//                pr_err("%s : not able to set fet_g_gpio as output\n", __func__);
//                goto err_firm;
//            }
        }
    } else {
        /* VEN ON */
        if(platform_mver == PLAT_MVER_DMT1) {
            ret = gpio_direction_output(pn544_dev->ven_gpio, VEN_PWR_ON);
            if (ret < 0) {
                pr_err("%s : not able to set ven_gpio as output\n", __func__);
                goto err_firm;
            }
            spi_ese_pinctrl(1);
        } else {
//            ret = gpio_direction_output(pn544_dev->pwr_en_gpio, PWR_EN_ON);
//            if (ret < 0) {
//                pr_err("%s : not able to set pwr_en_gpio as output\n", __func__);
//                goto err_firm;
//            }
//            spi_ese_pinctrl(1);
//            msleep(100);
//            i2c_nfc_pinctrl(1);
//
//            ret = gpio_direction_output(pn544_dev->fet_g_gpio, FET_G_ON);
//            if (ret < 0) {
//                pr_err("%s : not able to set fet_g_gpio as output\n", __func__);
//                goto err_firm;
//            }
//            i2c_nfc_pinctrl(0);
//            msleep(10);
//            ret = gpio_direction_output(pn544_dev->pwr_en_gpio, PWR_EN_OFF);
//            if (ret < 0) {
//                pr_err("%s : not able to set pwr_en_gpio as output\n", __func__);
//                goto err_firm;
//            }
//            spi_ese_pinctrl(0);
//            msleep(10);
//            ret = gpio_direction_output(pn544_dev->fet_g_gpio, FET_G_OFF);
//            if (ret < 0) {
//                pr_err("%s : not able to set fet_g_gpio as output\n", __func__);
//                goto err_firm;
//            }
//            msleep(10);

            nfc_pwr_en_pinctrl(1);
            ret = gpio_direction_output(pn544_dev->pwr_en_gpio, PWR_EN_ON);
            if (ret < 0) {
                pr_err("%s : not able to set pwr_en_gpio as output\n", __func__);
                goto err_firm;
            }
            
            
            if ( spi_ese_pinctrl_init(1) )
            {
                 pr_err(" p61 driver has not initialized yet, deferring probe()\n");
                 return - EPROBE_DEFER;
            }
            
            i2c_nfc_pinctrl_init(1);
            
            spi_ese_pinctrl(1);
            msleep(100);
            i2c_nfc_pinctrl(1);
            
            //FET_G RESET
            nfc_en_fet_g_pinctrl(FET_G_ON);
            nfc_en_fet_g_pinctrl(FET_G_OFF);
        }
    }
    #else
    ret = gpio_direction_output(pn544_dev->ven_gpio, VEN_PWR_ON);
    if (ret < 0) {
        pr_err("%s : not able to set ven_gpio as output\n", __func__);
        goto err_firm;
    }
    #endif
//    ret = gpio_direction_output(pn544_dev->ese_pwr_gpio, 0);
//    if (ret < 0) {
//        pr_err("%s : not able to set ese_pwr gpio as output\n", __func__);
//        goto err_ese_pwr;
//    }
    if (platform_data->firm_gpio) {
        ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set firm_gpio as output\n",
                    __func__);
            goto err_exit;
        }
    }
#ifdef ISO_RST
    ret = gpio_direction_output(pn544_dev->iso_rst_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set iso rst gpio as output\n", __func__);
        goto err_iso_rst;
    }
#endif
    /* init mutex and queues */
    ese_reset_resource_init();
    init_waitqueue_head(&pn544_dev->read_wq);
    mutex_init(&pn544_dev->read_mutex);
    sema_init(&ese_access_sema, 1);
    mutex_init(&pn544_dev->p61_state_mutex);
    spin_lock_init(&pn544_dev->irq_enabled_lock);
    pn544_dev->pSecureTimerCbWq = create_workqueue(SECURE_TIMER_WORK_QUEUE);
    INIT_WORK(&pn544_dev->wq_task, secure_timer_workqueue);
    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name = "pn553";
    pn544_dev->pn544_device.fops = &pn544_dev_fops;

    ret = misc_register(&pn544_dev->pn544_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }
    /* HiKey Compilation fix */
    #ifndef HiKey_620_COMPILATION_FIX
    wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "NFCWAKE");
    #endif
#ifdef ISO_RST
    /* Setting ISO RESET pin high to power ESE during init */
    gpio_set_value(pn544_dev->iso_rst_gpio, 1);
#endif
    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
    pn544_dev->irq_enabled = true;
    ret = request_irq(client->irq, pn544_dev_irq_handler,
            IRQF_TRIGGER_HIGH, client->name, pn544_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
//    enable_irq_wake(pn544_dev->client->irq);
    pn544_disable_irq(pn544_dev);
    device_init_wakeup(&client->dev, true);
    device_set_wakeup_capable(&client->dev, true);
    i2c_set_clientdata(client, pn544_dev);

#ifdef VEN_ALWAYS_ON
    msleep(5); /* VBAT--> VDDIO(HIGH) + Guardtime of min 5ms --> VEN(HIGH) */
    /* VEN toggle(reset) to proceed */
    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
    msleep(5);
    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
#endif

#if HWINFO
    /*
     * This function is used only if
     * hardware info is required during probe*/
    check_hw_info();
#endif
    if ( KCDEBUG )
    {
		pr_info("[NFC DEBUG] %s End \n", __func__ );
//#if IS_BUILTIN(NFC_PN553_DEVICES)
//		msm_gpio_dbg_print();
//#endif
    }
    return 0;

    err_request_irq_failed:
    misc_deregister(&pn544_dev->pn544_device);
    err_misc_register:
    ese_reset_resource_destroy();
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    kfree(pn544_dev);
    err_exit:
    if (pn544_dev->firm_gpio)
        gpio_free(platform_data->firm_gpio);
    err_firm:
//    gpio_free(platform_data->ese_pwr_gpio);
//    err_ese_pwr:
    gpio_free(platform_data->ven_gpio);
    gpio_free(platform_data->pwr_en_gpio);
    gpio_free(platform_data->fet_g_gpio);
    err_ven:
    gpio_free(platform_data->irq_gpio);
#ifdef ISO_RST
    err_iso_rst:
    gpio_free(platform_data->iso_rst_gpio);
#endif
	if ( KCDEBUG )
	{
		pr_info("[NFC DEBUG] err_request_irq_failed  %s End \n", __func__ );
//#if IS_BUILTIN(NFC_PN553_DEVICES)
//		msm_gpio_dbg_print();
//#endif
	}
    return ret;
}

static int pn544_remove(struct i2c_client *client)
{
    struct pn544_dev *pn544_dev;

	if ( KCDEBUG )
	{
		pr_info("[NFC DEBUG] %s Start \n", __func__ );
//#if IS_BUILTIN(NFC_PN553_DEVICES)
//		msm_gpio_dbg_print();
//#endif
	}
	
	i2c_nfc_pinctrl_init(0);
	
    pn544_dev = i2c_get_clientdata(client);
    free_irq(client->irq, pn544_dev);
    misc_deregister(&pn544_dev->pn544_device);
    mutex_destroy(&pn544_dev->read_mutex);
    mutex_destroy(&pn544_dev->p61_state_mutex);
    gpio_free(pn544_dev->irq_gpio);
    gpio_free(pn544_dev->ven_gpio);
    gpio_free(pn544_dev->pwr_en_gpio);
    gpio_free(pn544_dev->fet_g_gpio);
//    gpio_free(pn544_dev->ese_pwr_gpio);
    destroy_workqueue(pn544_dev->pSecureTimerCbWq);
#ifdef ISO_RST
    gpio_free(pn544_dev->iso_rst_gpio);
#endif
    pn544_dev->p61_current_state = P61_STATE_INVALID;
    pn544_dev->nfc_ven_enabled = false;
    pn544_dev->spi_ven_enabled = false;
    ese_reset_resource_destroy();

    if (pn544_dev->firm_gpio)
        gpio_free(pn544_dev->firm_gpio);
    kfree(pn544_dev);

	if ( KCDEBUG )
	{
		pr_info("[NFC DEBUG] %s End \n", __func__ );
//#if IS_BUILTIN(NFC_PN553_DEVICES)
//		msm_gpio_dbg_print();
//#endif
	}
	
    return 0;
}

#ifdef CONFIG_PM
static int pn544_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    int ret;

	if ( KCDEBUG )   pr_info("%s Start\n", __func__ );
    pr_info("%s : device_may_wakeup=%d, irq=%d\n",
                      __func__, device_may_wakeup(&client->dev), client->irq);
#ifdef kc_leds
    _kc_leds_rgb_set(0xFF0000);  //Add Red LED ON
#endif
    
    if(device_may_wakeup(&client->dev)) {
        ret = enable_irq_wake(client->irq);
        pr_debug("%s : enable_irq_wake() ret=%d\n", __func__, ret);

#ifdef kc_leds
        _kc_leds_rgb_set(0x00FF00);  //Add Green LED ON
#endif

    }

	if ( KCDEBUG )  pr_info("%s End \n", __func__ );
    return 0;
}

static int pn544_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    int ret;

	if ( KCDEBUG )  pr_info("%s Start \n", __func__ );
    pr_info("%s : device_may_wakeup=%d, irq=%d\n",
                      __func__, device_may_wakeup(&client->dev), client->irq);
#ifdef kc_leds
    _kc_leds_rgb_set(0xFFFFFF);  //Add White LED ON
#endif

    if(device_may_wakeup(&client->dev)) {
        ret = disable_irq_wake(client->irq);
        pr_debug("%s : disable_irq_wake() ret=%d\n", __func__, ret);

#ifdef kc_leds
        _kc_leds_rgb_set(0x0000FF);  //Add Blue LED ON
#endif
    }
	if ( KCDEBUG )  pr_info("%s End \n", __func__ );

    return 0;
}

static SIMPLE_DEV_PM_OPS(pn544_pm_ops, pn544_suspend, pn544_resume);
#endif
static const struct i2c_device_id pn544_id[] = {
#if NEXUS5x
        { "pn548", 0 },
#else
#if PN553
        { "pn553", 0 },
#else  // PN553
        { "pn544", 0 },
#endif // PN553
#endif
        { }
};
#if DRAGON_NFC
static struct of_device_id pn544_i2c_dt_match[] = {
    {
#if NEXUS5x
        .compatible = "nxp,pn548",
#else
#if PN553
        .compatible = "nxp,pn553",
#else  // PN553
        .compatible = "nxp,pn544",
#endif // PN553
#endif
    },
    {}
};
#endif
static struct i2c_driver pn544_driver = {
        .id_table   = pn544_id,
        .probe      = pn544_probe,
        .remove     = pn544_remove,
        .driver     = {
                .owner = THIS_MODULE,
#if NEXUS5x
                .name  = "pn548",
#else
#if PN553
                .name  = "pn553",
#else  // PN553
                .name  = "pn544",
#endif // PN553
#endif
#if DRAGON_NFC
                .of_match_table = pn544_i2c_dt_match,
#endif
#ifdef CONFIG_PM
                .pm = &pn544_pm_ops,
#endif
        },
};
#if HWINFO
/******************************************************************************
 * Function         check_hw_info
 *
 * Description      This function is called during pn544_probe to retrieve
 *                  HW info.
 *                  Useful get HW information in case of previous FW download is
 *                  interrupted and core reset is not allowed.
 *                  This function checks if core reset  is allowed, if not
 *                  sets DWNLD_REQ(firm_gpio) , ven reset and sends firmware
 *                  get version command.
 *                  In response HW information will be received.
 *
 * Returns          None
 *
 ******************************************************************************/
static void check_hw_info() {
    char read_data[20];
    int ret, get_version_len = 8, retry_count = 0;
    static uint8_t cmd_reset_nci[] = {0x20, 0x00, 0x01, 0x00};
    char get_version_cmd[] =
    {0x00, 0x04, 0xF1, 0x00, 0x00, 0x00, 0x6E, 0xEF};

    pr_info("%s :Enter\n", __func__);

    /*
     * Ven Reset  before sending core Reset
     * This is to check core reset is allowed or not.
     * If not allowed then previous FW download is interrupted in between
     * */
    pr_info("%s :Ven Reset \n", __func__);
    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
    spi_ese_pinctrl(1);
    msleep(10);
    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
    spi_ese_pinctrl(0);
    msleep(10);
    gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
    spi_ese_pinctrl(1);
    msleep(10);
    ret = i2c_master_send(pn544_dev->client, cmd_reset_nci, 4);

    if (ret == 4) {
        pr_info("%s : core reset write success\n", __func__);
    } else {

        /*
         * Core reset  failed.
         * set the DWNLD_REQ , do ven reset
         * send firmware download info command
         * */
        pr_err("%s : write failed\n", __func__);
        pr_info("%s power on with firmware\n", __func__);
        gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
        spi_ese_pinctrl(1);
        msleep(10);
        if (pn544_dev->firm_gpio) {
            p61_update_access_state(pn544_dev, P61_STATE_DWNLD, true);
            gpio_set_value(pn544_dev->firm_gpio, 1);
        }
        msleep(10);
        gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_OFF);
        spi_ese_pinctrl(0);
        msleep(10);
        gpio_set_value(pn544_dev->ven_gpio, VEN_PWR_ON);
        spi_ese_pinctrl(1);
        msleep(10);
        ret = i2c_master_send(pn544_dev->client, get_version_cmd, get_version_len);
        if (ret != get_version_len) {
            ret = -EIO;
            pr_err("%s : write_failed \n", __func__);
        }
        else {
            pr_info("%s :data sent\n", __func__);
        }

        ret = 0;

        while (retry_count < 10) {

            /*
             * Wait for read interrupt
             * If spurious interrupt is received retry again
             * */
            pn544_dev->irq_enabled = true;
            enable_irq(pn544_dev->client->irq);
//            enable_irq_wake(pn544_dev->client->irq);
            ret = wait_event_interruptible(
                    pn544_dev->read_wq,
                    !pn544_dev->irq_enabled);

            pn544_disable_irq(pn544_dev);

            if (gpio_get_value(pn544_dev->irq_gpio))
                break;

            pr_warning("%s: spurious interrupt detected\n", __func__);
            retry_count ++;
        }

        if(ret) {
            return;
        }

        /*
         * Read response data and copy into hw_type_info
         * */
        ret = i2c_master_recv(pn544_dev->client, read_data, 14);

        if(ret) {
            memcpy(hw_info.data, read_data, ret);
            hw_info.len = ret;
            pr_info("%s :data received len  : %d\n", __func__,hw_info.len);
        }
        else {
            pr_err("%s :Read Failed\n", __func__);
        }
    }
}
#endif
/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
    pr_info("Loading pn544 driver\n");
    return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
    pr_info("Unloading pn544 driver\n");
    i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_ALIAS("of:nxp,pn544");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
