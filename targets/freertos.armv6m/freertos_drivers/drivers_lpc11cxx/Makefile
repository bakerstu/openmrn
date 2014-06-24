OPENMRNPATH ?= $(realpath ../../../..)
include $(OPENMRNPATH)/etc/lib.mk

DEPS += CMSIS_LPC11_PATH

CORECFLAGS += -DTARGET_LPC11Cxx
INCLUDES += \
	-I$(CMSIS_LPC11_PATH)/inc \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC11U24" \
