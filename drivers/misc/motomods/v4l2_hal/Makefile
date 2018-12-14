v4l2-hal-y := v4l2_hal.o v4l2_misc.o v4l2_hal_ext_ctrls.o
obj-m += v4l2-hal.o

KERNELVER		?= $(shell uname -r)
KERNELDIR 		?= /lib/modules/$(KERNELVER)/build
INSTALL_MOD_PATH	?= /..
PWD			:= $(shell pwd)

# kernel config option that shall be enable
CONFIG_OPTIONS_ENABLE := SYSFS

# kernel config option that shall be disable
CONFIG_OPTIONS_DISABLE :=

# this only run in kbuild part of the makefile
ifneq ($(KERNELRELEASE),)
# This function returns the argument version if current kernel version is minor
# than the passed version, return 1 if equal or the current kernel version if it
# is greater than argument version.
kvers_cmp=$(shell [ "$(KERNELVERSION)" = "$(1)" ] && echo 1 || printf "$(1)\n$(KERNELVERSION)" | sort -V | tail -1)

ifneq ($(call kvers_cmp,"3.19.0"),3.19.0)
    CONFIG_OPTIONS_ENABLE += LEDS_CLASS_FLASH
endif

ifneq ($(call kvers_cmp,"4.2.0"),4.2.0)
    CONFIG_OPTIONS_ENABLE += V4L2_FLASH_LED_CLASS
endif

$(foreach opt,$(CONFIG_OPTIONS_ENABLE),$(if $(CONFIG_$(opt)),, \
     $(error CONFIG_$(opt) is disabled in the kernel configuration and must be enable \
     to continue compilation)))
$(foreach opt,$(CONFIG_OPTIONS_DISABLE),$(if $(filter m y, $(CONFIG_$(opt))), \
     $(error CONFIG_$(opt) is enabled in the kernel configuration and must be disable \
     to continue compilation),))
endif

# add -Wall to try to catch everything we can.
ccflags-y := -Wall

# needed for trace events
ccflags-y += -I$(src)

all: module

module:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

check:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) C=2 CF="-D__CHECK_ENDIAN__"

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

coccicheck:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) coccicheck

install: module
	mkdir -p $(INSTALL_MOD_PATH)/lib/modules/$(KERNELVER)/kernel/drivers/greybus/
	cp -f *.ko $(INSTALL_MOD_PATH)/lib/modules/$(KERNELVER)/kernel/drivers/greybus/
	depmod -b $(INSTALL_MOD_PATH) -a $(KERNELVER)
