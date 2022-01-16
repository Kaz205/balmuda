/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2020 KYOCERA Corporation
 */

#ifndef __QCOM_SMEM_H__
#define __QCOM_SMEM_H__

#define QCOM_SMEM_HOST_ANY -1

#define KCC_SMEM_CHG_PARAM_SIZE               200
#define KCC_SMEM_KERR_LOG_SIZE                1024
#define KCC_SMEM_CRASH_LOG_SIZE               1024
#define KCC_SMEM_FACTORY_CMDLINE_SIZE         1024
#define KCC_SMEM_FACTORY_OPTIONS_SIZE         4
#define KCC_SMEM_DDR_DATA_INFO_SIZE           8
#define KCC_SMEM_KC_WM_UUID_SIZE              6
#define KCC_SMEM_SECUREBOOT_FLAG_SIZE         4
#define KCC_SMEM_BFSS_DATA_SIZE               4096
#define KCC_SMEM_KC_POWER_ON_STATUS_INFO_SIZE 8
#define KCC_SMEM_BOOT_PW_ON_CHECK_SIZE        8
#define KCC_SMEM_HW_ID_SIZE                   4
#define KCC_SMEM_VENDOR_ID_SIZE               4
#define KCC_SMEM_IMEI_SIZE                    9
#define KCC_SMEM_NFC_RFS_SIG_STATE_SIZE       1
#define KCC_SMEM_UICC_INFO_SIZE               2
#define KCC_SMEM_SDDL_FLAG_SIZE               16
#define KCC_SMEM_FBDL_ENABLE_SIZE             8
#define KCC_SMEM_COVERT_MODE_STATUS_SIZE      1
#define KCC_SMEM_XBL_LOADER_BOOTLOG_SIZE      8192

enum {
	SMEM_ID_VENDOR2              = 136,

	SMEM_KCC_BASE                = 1000,
	SMEM_CHG_PARAM               = SMEM_KCC_BASE + 1,
	SMEM_KERR_LOG                = SMEM_KCC_BASE + 2,
	SMEM_CRASH_LOG               = SMEM_KCC_BASE + 3,
	SMEM_FACTORY_CMDLINE         = SMEM_KCC_BASE + 4,
	SMEM_FACTORY_OPTIONS         = SMEM_KCC_BASE + 5,
	SMEM_DDR_DATA_INFO           = SMEM_KCC_BASE + 6,
	SMEM_KC_WM_UUID              = SMEM_KCC_BASE + 7,
	SMEM_SECUREBOOT_FLAG         = SMEM_KCC_BASE + 8,
	SMEM_BFSS_DATA               = SMEM_KCC_BASE + 9,
	SMEM_KC_POWER_ON_STATUS_INFO = SMEM_KCC_BASE + 10,
	SMEM_BOOT_PW_ON_CHECK        = SMEM_KCC_BASE + 11,
	SMEM_HW_ID                   = SMEM_KCC_BASE + 12,
	SMEM_VENDOR_ID               = SMEM_KCC_BASE + 13,
	SMEM_IMEI                    = SMEM_KCC_BASE + 14,
	SMEM_NFC_RFS_SIG_STATE       = SMEM_KCC_BASE + 15,
	SMEM_UICC_INFO               = SMEM_KCC_BASE + 16,
	SMEM_SDDL_FLAG               = SMEM_KCC_BASE + 17,
	SMEM_FBDL_ENABLE             = SMEM_KCC_BASE + 18,
	SMEM_COVERT_MODE_STATUS      = SMEM_KCC_BASE + 19,
	SMEM_XBL_LOADER_BOOTLOG      = SMEM_KCC_BASE + 20
};

int qcom_smem_alloc(unsigned host, unsigned item, size_t size);
void *qcom_smem_get(unsigned host, unsigned item, size_t *size);

int qcom_smem_get_free_space(unsigned host);

phys_addr_t qcom_smem_virt_to_phys(void *p);

void *kc_smem_alloc(unsigned id, unsigned buf_size);

#endif
