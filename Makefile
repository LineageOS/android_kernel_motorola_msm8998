greybus-y :=	core.o		\
		debugfs.o	\
		hd.o		\
		manifest.o	\
		interface.o	\
		bundle.o	\
		connection.o	\
		protocol.o	\
		control.o	\
		svc.o		\
		firmware.o	\
		operation.o

gb-phy-y :=	gpbridge.o	\
		sdio.o	\
		uart.o	\
		pwm.o	\
		gpio.o	\
		i2c.o	\
		spi.o	\
		usb.o

gb-audio-y :=   audio.o \
		audio-gb-cmds.o \
		mods_codec.o

gb-mods-y :=	muc_core.o \
		mods_ap.o \
		muc_gpio.o \
		muc_svc.o \
		mods_init.o \
		muc_buffer.o \
		muc_spi.o \
		muc_i2c.o \
		apba.o \
		mods_uart.o \
		mods_uart_pm.o \
		mods_nw.o \
		crc.o

# Prefix all modules with gb-
gb-vibrator-y := vibrator.o
gb-battery-y := battery.o
gb-loopback-y := loopback.o
gb-light-y := light.o
gb-raw-y := raw.o
gb-hid-y := hid.o
gb-es2-y := es2.o
gbsim-mods-sim-y := muc_sim.o
gb-db3-y := db3-platform.o
gb-camera-y := camera.o
gb-vendor-moto-y := vendor_moto.o
gb-ptp-y := ptp.o
gb-camera_ext-y := camera_ext_gb.o camera_ext_mod_v4l2.o camera_ext_ctrls.o
gb-usb_ext-y := usb-ext.o
gb-display-y := display.o
gb-sensors_ext-y := sensors_ext.o sensors_ext_iio.o

obj-m += greybus.o
#obj-m += gb-phy.o

# gb-audio depends on MODS_CODEC_BUS
ifneq ($(filter m y, $(CONFIG_MODS_CODEC_BUS)),)
obj-m += gb-audio.o
endif

#obj-m += gb-vibrator.o
obj-m += gb-battery.o
#obj-m += gb-loopback.o
obj-m += gb-light.o
obj-m += gb-hid.o
obj-m += gb-raw.o
# Remove the ES1/2 and db3 host device drivers
#obj-m += gb-es2.o
obj-m += gbsim-mods-sim.o
#obj-m += gb-db3.o
#obj-m += gb-camera.o
obj-m += gb-mods.o
obj-m += gb-vendor-moto.o
obj-m += gb-ptp.o
obj-m += gb-camera_ext.o

# gb-display depends on MOD_DISPLAY
ifneq ($(filter m y, $(CONFIG_MOD_DISPLAY)),)
obj-m += gb-display.o
endif

# gb-usb_ext depends on MODS_USB_EXT_BRIDGE
ifneq ($(filter m y, $(CONFIG_MODS_USB_EXT_BRIDGE)),)
obj-m += gb-usb_ext.o
endif

obj-m += gb-sensors_ext.o

KERNELVER		?= $(shell uname -r)
KERNELDIR 		?= /lib/modules/$(KERNELVER)/build
INSTALL_MOD_PATH	?= /..
PWD			:= $(shell pwd)

# kernel config option that shall be enable
CONFIG_OPTIONS_ENABLE := POWER_SUPPLY PWM SYSFS SPI USB SND_SOC MMC LEDS_CLASS

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

#The new mods hw designation has add an gpio for spi/i2c select
ifeq ($(CONFIG_MODS_2ND_GEN), y)
CONFIG_OPTIONS_ENABLE += MODS_2ND_GEN
endif

ifeq ($(CONFIG_MODS_USE_EXTCODEC_MI2S), y)
CONFIG_OPTIONS_ENABLE += MODS_USE_EXTCODEC_MI2S
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
