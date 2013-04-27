MBEDPATH ?= $(shell \
sh -c "if [ \"X`printenv MBEDPATH`\" != \"X\" ]; then printenv MBEDPATH; \
     elif [ -d ~/lpc-workspace/libmbed_2387/mbed/USBDevice ]; then echo ~/lpc-workspace/libmbed_2387/mbed; \
     else echo MBED not found; fi" \
)

ifneq ($(MBEDPATH),MBED not found)

HAVE_MBED = 1

endif
