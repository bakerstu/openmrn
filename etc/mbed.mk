MBEDPATH ?= $(shell \
sh -c "if [ \"X`printenv MBEDPATH`\" != \"X\" ]; then printenv MBEDPATH; \
     elif [ -d ~/lpc-workspace/libmbed_2387/mbed/USBDevice ]; then echo ~/lpc-workspace/libmbed_2387/mbed; \
     elif [ -d /opt/mbed/default/libraries/USBDevice ]; then echo /opt/mbed/default/libraries; \
     else echo MBED not found; fi" \
)

ifneq ($(MBEDPATH),MBED not found)


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

endif
