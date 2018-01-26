LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := greybus
LOCAL_MODULE_TAGS := optional
LOCAL_ADDITIONAL_DEPENDENCIES := build-greybus
include $(BUILD_PHONY_PACKAGE)

GB_SRC_PATH := $(LOCAL_PATH)
GB_KDIRARG := KERNELDIR="${ANDROID_PRODUCT_OUT}/obj/KERNEL_OBJ"

TARGET_KERNEL_CROSS_COMPILE_PREFIX := $(strip $(TARGET_KERNEL_CROSS_COMPILE_PREFIX))
ifeq ($(TARGET_KERNEL_CROSS_COMPILE_PREFIX),)
GB_KERNEL_TOOLS_PREFIX := arm-eabi-
else
GB_KERNEL_TOOLS_PREFIX := $(TARGET_KERNEL_CROSS_COMPILE_PREFIX)
endif

GB_ARCHARG := ARCH=$(TARGET_ARCH)
GB_FLAGARG := EXTRA_CFLAGS+=-fno-pic
GB_ARGS := $(GB_KDIRARG) $(GB_ARCHARG) $(GB_FLAGARG)

#Create vendor/lib/modules directory if it doesn't exist
$(shell mkdir -p $(TARGET_OUT_VENDOR)/lib/modules)

ifeq ($(GREYBUS_DRIVER_INSTALL_TO_KERNEL_OUT),true)
GB_MODULES_OUT := $(KERNEL_MODULES_OUT)
else
GB_MODULES_OUT := $(TARGET_OUT_VENDOR)/lib/modules/
endif

build-greybus: $(ACP) $(INSTALLED_KERNEL_TARGET)
	$(MAKE) clean -C $(GB_SRC_PATH)
	$(MAKE) -j$(MAKE_JOBS) -C $(GB_SRC_PATH) CROSS_COMPILE=$(GB_KERNEL_TOOLS_PREFIX) $(GB_ARGS)
	ko=`find $(GB_SRC_PATH) -type f -name "*.ko"`;\
	for i in $$ko;\
	do $(GB_KERNEL_TOOLS_PREFIX)strip --strip-unneeded $$i;\
	$(ACP) -fp $$i $(GB_MODULES_OUT);\
	done
