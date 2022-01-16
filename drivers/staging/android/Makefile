ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
obj-$(CONFIG_ANDROID_TIMED_OUTPUT)	+= timed_output.o
obj-$(CONFIG_ANDROID_KCJLOG)		+= kcjlog.o
obj-$(CONFIG_ANDROID_CRASH_REASON)	+= kclog_smem.o crash_reason.o

CFLAGS_kcjlog.o += -DTARGET_BUILD_VARIANT='"$(TARGET_BUILD_VARIANT)"'
