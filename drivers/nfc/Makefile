# SPDX-License-Identifier: GPL-2.0
#
# Makefile for nfc devices
#

obj-$(CONFIG_NFC_FDP)		+= fdp/
obj-$(CONFIG_NFC_PN544)		+= pn544/
obj-$(CONFIG_NFC_MICROREAD)	+= microread/
obj-$(CONFIG_NFC_PN533)		+= pn533/
obj-$(CONFIG_NFC_MEI_PHY)	+= mei_phy.o
obj-$(CONFIG_NFC_SIM)		+= nfcsim.o
obj-$(CONFIG_NFC_PORT100)	+= port100.o
obj-$(CONFIG_NFC_MRVL)		+= nfcmrvl/
obj-$(CONFIG_NFC_TRF7970A)	+= trf7970a.o
obj-$(CONFIG_NFC_ST21NFCA)  	+= st21nfca/
obj-$(CONFIG_NFC_ST_NCI)	+= st-nci/
obj-$(CONFIG_NFC_NXP_NCI)	+= nxp-nci/
obj-$(CONFIG_NFC_S3FWRN5)	+= s3fwrn5/
obj-$(CONFIG_NFC_ST95HF)	+= st95hf/
# obj-$(CONFIG_NFC_NQ)            += nq-nci.o
obj-$(CONFIG_NFC_PN553_DEVICES)     += pn553_i2c.o
pn553_i2c-objs                  := cold_reset.o pn553.o
ccflags-$(CONFIG_NFC_PN553_DEVICES) := -DDEBUG

obj-y     += pi4io/
#obj-$(CONFIG_NFC_PI4IO_DEVICES)     += pi4io/
