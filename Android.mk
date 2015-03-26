.PHONY: build-greybus

LOCAL_PATH := $(my-dir)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := greybus.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-phy.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-slice.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-es1.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-es2.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-vibrator.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-battery.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_SRC_FILES := gb-vendor-moto.ko
LOCAL_MODULE := $(LOCAL_SRC_FILES)
LOCAL_MODULE_CLASS := EXECUTABLES
LOCAL_MODULE_PATH := $(PRODUCT_OUT)/system/lib/modules
$(LOCAL_PATH)/$(LOCAL_SRC_FILES): build-greybus
include $(BUILD_PREBUILT)

#######################
# Local build code
#######################
GREYBUS_SRC_PATH := $(LOCAL_PATH)
KERNEL_TOOLS_PREFIX:=arm-eabi-
KDIRARG := KERNELDIR="${ANDROID_PRODUCT_OUT}/obj/KERNEL_OBJ"
ifneq ($(ANDROID_64),)
  ARCHARG := ARCH=arm64
  FLAGARG := EXTRA_CFLAGS+=-fno-pic
else
  ARCHARG := ARCH=arm
  FLAGARG := EXTRA_CFLAGS+=-fno-pic
endif
ARGS := $(KDIRARG) $(ARCHARG) $(FLAGARG)

# if building with mm then assume kernel is already built
ifeq (,$(ONE_SHOT_MAKEFILE))
LOCAL_GREYBUS_DEPEND :=  $(KERNEL_OUT)/arch/arm/boot/Image
endif

build-greybus: $(LOCAL_GREYBUS_DEPEND)
	make clean -C $(GREYBUS_SRC_PATH)
	cd $(GREYBUS_SRC_PATH) &&\
	$(MAKE) -j$(MAKE_JOBS) CROSS_COMPILE=$(KERNEL_TOOLS_PREFIX) $(ARGS)
	ko=`find $(GREYBUS_SRC_PATH) -type f -name "*.ko"`;\
	for i in $$ko; do $(KERNEL_TOOLS_PREFIX)strip --strip-unneeded $$i; done
