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

obj-$(CONFIG_MOTOMODS) += greybus.o
#obj-$(CONFIG_MOTOMODS) += gb-phy.o

# gb-audio depends on MODS_CODEC_BUS
ifneq ($(filter m y, $(CONFIG_MODS_CODEC_BUS)),)
obj-$(CONFIG_MOTOMODS) += gb-audio.o
endif

#obj-$(CONFIG_MOTOMODS) += gb-vibrator.o
obj-$(CONFIG_MOTOMODS) += gb-battery.o
#obj-$(CONFIG_MOTOMODS) += gb-loopback.o
obj-$(CONFIG_MOTOMODS) += gb-light.o
obj-$(CONFIG_MOTOMODS) += gb-hid.o
obj-$(CONFIG_MOTOMODS) += gb-raw.o
# Remove the ES1/2 and db3 host device drivers
#obj-$(CONFIG_MOTOMODS) += gb-es2.o
obj-$(CONFIG_MOTOMODS) += gbsim-mods-sim.o
#obj-$(CONFIG_MOTOMODS) += gb-db3.o
#obj-$(CONFIG_MOTOMODS) += gb-camera.o
obj-$(CONFIG_MOTOMODS) += gb-mods.o
obj-$(CONFIG_MOTOMODS) += gb-vendor-moto.o
obj-$(CONFIG_MOTOMODS) += gb-ptp.o
obj-$(CONFIG_MOTOMODS) += gb-camera_ext.o

# gb-display depends on MOD_DISPLAY
ifneq ($(filter m y, $(CONFIG_MOD_DISPLAY)),)
obj-$(CONFIG_MOTOMODS) += gb-display.o
endif

# gb-usb_ext depends on MODS_USB_EXT_BRIDGE
ifneq ($(filter m y, $(CONFIG_MODS_USB_EXT_BRIDGE)),)
obj-$(CONFIG_MOTOMODS) += gb-usb_ext.o
endif

obj-$(CONFIG_MOTOMODS) += gb-sensors_ext.o

KERNELVER		?= $(shell uname -r)
KERNELDIR 		?= /lib/modules/$(KERNELVER)/build
INSTALL_MOD_PATH	?= /..
PWD			:= $(shell pwd)

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
