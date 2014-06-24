OPENMRNPATH ?= $(realpath ../../../..)
include $(OPENMRNPATH)/etc/lib.mk

CORECFLAGS += -DTARGET_LPC2368
INCLUDES += \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC2368" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC2368"
