include $(OPENMRNPATH)/etc/path.mk

ifdef MBEDPATH
ifneq ($(wildcard $(MBEDPATH)/mbed/cpp/mbed.h),)
MBEDSRCPATH=$(MBEDPATH)/mbed
HAVE_MBED = 1
endif

ifneq ($(wildcard $(MBEDPATH)/mbed/src/cpp/mbed.h),)
MBEDSRCPATH=$(MBEDPATH)/mbed/src
HAVE_MBED = 1
endif

ifndef HAVE_MBED
$(error Mbed source not found under $(MBEDPATH))
endif

INCLUDES += -I"$(MBEDSRCPATH)/cpp" -I"$(MBEDPATH)/USBDevice/USBDevice" -I"$(MBEDPATH)/USBDevice/USBSerial" -I"$(MBEDSRCPATH)/capi"

endif #MBEDPATH
