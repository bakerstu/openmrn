OPENMRNPATH ?= $(realpath ../../../..)
include $(OPENMRNPATH)/etc/lib.mk

CORECFLAGS += -DTARGET_LPC1768 -DINTERRUPT_ATTRIBUTE=
INCLUDES += \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC1768" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC1768"
