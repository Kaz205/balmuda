# This software is contributed or developed by KYOCERA Corporation.
# (C) 2021 KYOCERA Corporation
#
# For TI Smart Audio.

# tfa drivers code path:
TAS_PATH := sound/soc/codecs/tas256x

# include path defination.

# debugging support (also enables trace_printk)
EXTRA_CFLAGS += -DDEBUG

# tas source code:
snd-soc-tas256x-objs := tas2563-codec.o
snd-soc-tas256x-objs += tas2563-misc.o
snd-soc-tas256x-objs += tas2563-regmap.o
snd-soc-tas256x-objs += tiload.o

obj-$(CONFIG_SND_SOC_TAS256X) += snd-soc-tas256x.o
#obj-y	+= snd-soc-tas256x.o
