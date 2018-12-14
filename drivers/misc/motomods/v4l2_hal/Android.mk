LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := v4l2_hal
LOCAL_MODULE_TAGS := optional
LOCAL_ADDITIONAL_DEPENDENCIES := build-local
include $(BUILD_PHONY_PACKAGE)

LCL_SRC_PATH := $(LOCAL_PATH)
LCL_KDIRARG := KERNELDIR="${ANDROID_PRODUCT_OUT}/obj/KERNEL_OBJ"
ifeq ($(TARGET_ARCH),arm64)
  LCL_KERNEL_TOOLS_PREFIX=aarch64-linux-android-
else
  LCL_KERNEL_TOOLS_PREFIX:=arm-linux-androideabi-
endif
LCL_ARCHARG := ARCH=$(TARGET_ARCH)
LCL_FLAGARG := EXTRA_CFLAGS+=-fno-pic
LCL_ARGS := $(LCL_KDIRARG) $(LCL_ARCHARG) $(LCL_FLAGARG)

#Create vendor/lib/modules directory if it doesn't exist
$(shell mkdir -p $(TARGET_OUT_VENDOR)/lib/modules)

ifeq ($(GREYBUS_DRIVER_INSTALL_TO_KERNEL_OUT),true)
V4L2_HAL_MODULES_OUT := $(KERNEL_MODULES_OUT)
else
V4L2_HAL_MODULES_OUT := $(TARGET_OUT_VENDOR)/lib/modules/
endif

build-local: $(ACP) $(INSTALLED_KERNEL_TARGET)
	$(MAKE) clean -C $(LCL_SRC_PATH)
	$(MAKE) -j$(MAKE_JOBS) -C $(LCL_SRC_PATH) CROSS_COMPILE=$(LCL_KERNEL_TOOLS_PREFIX) $(LCL_ARGS)
	ko=`find $(LCL_SRC_PATH) -type f -name "*.ko"`;\
	for i in $$ko;\
	do $(LCL_KERNEL_TOOLS_PREFIX)strip --strip-unneeded $$i;\
	$(ACP) -fp $$i $(V4L2_HAL_MODULES_OUT);\
	done
