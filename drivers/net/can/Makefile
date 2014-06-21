#
#  Makefile for the Linux Controller Area Network drivers.
#

obj-$(CONFIG_CAN_VCAN)		+= vcan.o
obj-$(CONFIG_CAN_FLEXCAN)	+= flexcan/

obj-$(CONFIG_CAN_DEV)		+= can-dev.o

can-dev-y			:= dev.o

obj-y				+= usb/

obj-$(CONFIG_CAN_ASM9260)	+= asm9260/
obj-$(CONFIG_CAN_AT91)		+= at91_can.o

ccflags-$(CONFIG_CAN_DEBUG_DEVICES) := -DDEBUG
