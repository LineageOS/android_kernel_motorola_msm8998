#
# Makefile for the Himax Touchscreen driver
# Version: 1.2.2.50_moto_02
#

obj-$(CONFIG_TOUCHSCREEN_HIMAX_CHIPSET) += himax_mmi.o

himax_mmi-y := 	himax_ic_HX83112.o \
		himax_ic_HX83102.o \
		himax_platform.o \
		himax_inspection.o \
		himax_ic_incell_core.o \
		himax_common.o \
		himax_debug.o

ccflags-y += -Wno-strlcpy-strlcat-size
