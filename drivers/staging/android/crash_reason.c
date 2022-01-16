/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
                              <crash_reason.c>
DESCRIPTION

EXTERNALIZED FUNCTIONS

This software is contributed or developed by KYOCERA Corporation.
(C) 2019 KYOCERA Corporation

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 and
only version 2 as published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
02110-1301, USA.

*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
#define pr_fmt(fmt) "crash_reason: " fmt
/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * INCLUDE FILES
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
#include <linux/bio.h>
#include <linux/genhd.h>
#include <linux/delay.h>
#include <linux/pstore.h>
#include <linux/pstore_ram.h>
#include <linux/proc_fs.h>
#include <linux/kc_phymap.h>
#include <linux/crash_reason.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/soc/qcom/smem.h>
#include "resetlog.h"

#define EVENT_NONE   0
#define EVENT_HAPPEN 1

wait_queue_head_t crash_reason_wait_queue;

static atomic_t event_status;

unsigned char crash_system[32] = {};

#ifdef CONFIG_LOGCAPTURE_DEBUG
#define CRASH_SYSTEM_PRINT(fmt, ...) printk(KERN_DEBUG "crash_reason: " fmt, ##__VA_ARGS__)
#else
#define CRASH_SYSTEM_PRINT(fmt, ...) do {} while (0)
#endif

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * FUNCTION DEFINITIONS
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*
 * check_smem_crash_reason()
 *
 * Note: Check crash reason from smem.
 */
static unsigned char check_smem_crash_reason( ram_log_info_type *plog_info )
{
	unsigned char ret = 0;
	unsigned char cmp_system[ CRASH_SYSTEM_SIZE ];
	unsigned char cmp_kind[ CRASH_KIND_SIZE ];

	memcpy( cmp_system, plog_info->crash_system, CRASH_SYSTEM_SIZE );

	/* system check */
	if ( (0 == strncmp(cmp_system, CRASH_SYSTEM_KERNEL    , strlen(CRASH_SYSTEM_KERNEL   ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_MODEM     , strlen(CRASH_SYSTEM_MODEM    ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_PRONTO    , strlen(CRASH_SYSTEM_PRONTO   ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_ADSP      , strlen(CRASH_SYSTEM_ADSP     ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_VENUS     , strlen(CRASH_SYSTEM_VENUS    ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_ANDROID   , strlen(CRASH_SYSTEM_ANDROID  ))) ||
		 (0 == strncmp(cmp_system, CRASH_SYSTEM_DM_VERITY , strlen(CRASH_SYSTEM_DM_VERITY))) )
	{
		ret = 1;
	}

	memcpy( cmp_kind, plog_info->crash_kind, CRASH_KIND_SIZE );

	/* kind check */
	if ( (0 == strncmp(cmp_kind, CRASH_KIND_PANIC            , strlen(CRASH_KIND_PANIC           ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_FATAL            , strlen(CRASH_KIND_FATAL           ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_EXCEPTION        , strlen(CRASH_KIND_EXCEPTION       ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_WDOG_HW          , strlen(CRASH_KIND_WDOG_HW         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_WDOG_SW          , strlen(CRASH_KIND_WDOG_SW         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_SYS_SERVER       , strlen(CRASH_KIND_SYS_SERVER      ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_PWR_KEY          , strlen(CRASH_KIND_PWR_KEY         ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_KDFS_REBOOT      , strlen(CRASH_KIND_KDFS_REBOOT     ))) ||
		 (0 == strncmp(cmp_kind, CRASH_KIND_DEVICE_CORRUPTED , strlen(CRASH_KIND_DEVICE_CORRUPTED))) )
	{
		ret = 1;
	}
	return ret;
}

/*
 * set_smem_crash_info_data()
 *
 * Note: Set qalog info data to smem.
 */
void set_smem_crash_info_data( const char *pdata )
{
	ram_log_info_type *plog_info;
	size_t            len_data;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( ( plog_info == NULL ) || ( pdata == NULL ) ) {
		return;
	}
	if ( strnlen( (const char *)plog_info->crash_info_data, CRASH_INFO_DATA_SIZE ) > 0 ) {
		return;
	}
	memset( plog_info->crash_info_data, '\0', sizeof(plog_info->crash_info_data) );
	len_data = strnlen( pdata, CRASH_INFO_DATA_SIZE );
	memcpy( plog_info->crash_info_data, pdata, len_data );
	mb();
}

/*
 * set_smem_crash_info_data_add_line()
 *
 * Note: Set qalog info line data to smem.
 */
void set_smem_crash_info_data_add_line( const unsigned int line, const char *func )
{
	char buf[33];
	memset( buf, '\0', sizeof(buf) );
	snprintf( buf, sizeof(buf), "%x;%s", line, func);
	set_smem_crash_info_data( (const char *)buf );
}

/*
 * set_smem_crash_info_data_add_reg()
 *
 * Note: Set qalog info reg data to smem.
 */
void set_smem_crash_info_data_add_reg( const unsigned long crash_pc, const unsigned long crash_lr, const char *task_comm )
{
	char buf[33];

	memset( buf, '\0', sizeof(buf) );
	snprintf( buf, sizeof(buf), "%lx,%lx,%s", crash_pc, crash_lr, task_comm );
	set_smem_panic_info_data( (const char *)buf );
}

/*
 * set_smem_panic_info_data()
 *
 * Note: Set qalog panic info data to smem.
 */
void set_smem_panic_info_data( const char *pdata )
{
	ram_log_info_type *plog_info;
	size_t            len_data;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( ( plog_info == NULL ) || ( pdata == NULL ) ) {
		return;
	}
	if ( strnlen( (const char *)plog_info->panic_info_data, CRASH_INFO_DATA_SIZE ) > 0 ) {
		return;
	}
	memset( plog_info->panic_info_data, '\0', sizeof(plog_info->panic_info_data) );
	len_data = strnlen( pdata, CRASH_INFO_DATA_SIZE );
	memcpy( plog_info->panic_info_data, pdata, len_data );
	mb();
}

/*
 * get_smem_crash_system()
 *
 * Note: Get crash system from smem.
 */
void get_smem_crash_system(unsigned char* crash_system,unsigned int bufsize)
{
	ram_log_info_type *pSmemLogInfo;

	mb();
	pSmemLogInfo = (ram_log_info_type *)kc_smem_alloc(SMEM_CRASH_LOG, SIZE_SMEM_ALLOC);

	if (pSmemLogInfo == NULL) {
		return;
	}
	if (check_smem_crash_reason(pSmemLogInfo) == 0) {
		return;
	}
	if (bufsize > CRASH_SYSTEM_SIZE) {
		bufsize = CRASH_SYSTEM_SIZE;
	}

	memcpy( crash_system, &pSmemLogInfo->crash_system[0], bufsize );
}

/*
 * clear_smem_crash_info_data()
 *
 * Note: Clear qalog info data from smem.
 */
void clear_smem_crash_info_data( void )
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( plog_info == NULL ) {
		return;
	}
	memset( plog_info->crash_info_data, 0x00, sizeof(plog_info->crash_info_data) );
	mb();
}

/*
 * clear_smem_panic_info_data()
 *
 * Note: Clear qalog panic info data from smem.
 */
void clear_smem_panic_info_data( void )
{
	ram_log_info_type *plog_info;

	mb();
	plog_info = (ram_log_info_type *)kc_smem_alloc( SMEM_CRASH_LOG, SIZE_SMEM_ALLOC );

	if ( plog_info == NULL ) {
		return;
	}
	memset( plog_info->panic_info_data, 0x00, sizeof(plog_info->panic_info_data) );
	mb();
}

static inline int get_event_status(void)
{
	return atomic_read(&event_status);
}

static inline void set_event_status(int status)
{
	atomic_set(&event_status, status);
}

void subsys_notice_kerr(void)
{

	memset(&crash_system[0], 0x00, sizeof(crash_system));
	get_smem_crash_system(crash_system, sizeof(crash_system));

	CRASH_SYSTEM_PRINT("subsystem restart notice kerr (%s)\n",crash_system);

	if (crash_system[0] == 0) {
		pr_err("subsystem restart notice kerr (crash_system[0] == 0)");
		return;
	}

	set_event_status(EVENT_HAPPEN);
	wake_up_interruptible(&crash_reason_wait_queue);

	return;
}

static ssize_t crash_reason_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	int ret = 0;

	if (EVENT_NONE == get_event_status()) {
		if (filp->f_flags & O_NONBLOCK) {
			CRASH_SYSTEM_PRINT("return nonblocking is specified");
			return -EAGAIN;
		}
		ret = wait_event_interruptible(crash_reason_wait_queue, EVENT_NONE != get_event_status());
		if (ret) {
			CRASH_SYSTEM_PRINT("wait_event_interruptible ret = %d", ret);
			return ret;
		}
	}

	CRASH_SYSTEM_PRINT("event_status = %d", get_event_status());

	set_event_status(EVENT_NONE);
	count = sizeof(crash_system);
	if (copy_to_user(buf, &crash_system, count)) {
		pr_err("invalid buffer address");
		return -EFAULT;
	}

	return count;
}

static unsigned int crash_reason_poll(struct file *file, poll_table *wait)
{

	if (EVENT_NONE == get_event_status()) {
		poll_wait(file, &crash_reason_wait_queue, wait);
	}

	CRASH_SYSTEM_PRINT("exit poll event_status = %d", get_event_status());

	if (EVENT_NONE == get_event_status()) {
		return 0;
	}

	return POLLIN | POLLRDNORM;
}

static int crash_reason_open(struct inode *inode, struct file *file)
{
	CRASH_SYSTEM_PRINT("open");
	return 0;
}

static int crash_reason_release(struct inode *inode, struct file *file)
{
	CRASH_SYSTEM_PRINT("release\n");
	return 0;
}

static const struct file_operations crash_reason_fops = {
	.owner = THIS_MODULE,
	.read = crash_reason_read,
	.poll = crash_reason_poll,
	.open = crash_reason_open,
	.release = crash_reason_release,
};

static struct miscdevice crash_reason_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "crash_reason",
	.fops = &crash_reason_fops,
};

static int __init crash_reason_init(void) {
	int ret;

	atomic_set(&event_status, EVENT_NONE);
	init_waitqueue_head(&crash_reason_wait_queue);

	ret = misc_register(&crash_reason_misc);
	if(unlikely(ret)) {
		pr_err("failed to register device (MISC_DYNAMIC_MINOR)\n");
		return ret;
	}
	CRASH_SYSTEM_PRINT("crash_reason initialized\n");
	return 0;
}

static void __exit crash_reason_exit(void) {
	misc_deregister(&crash_reason_misc);
	CRASH_SYSTEM_PRINT("crash_reason exit\n");
}

module_init(crash_reason_init);
module_exit(crash_reason_exit);
