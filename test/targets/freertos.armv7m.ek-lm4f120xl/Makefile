OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find StellarisWare
include $(OPENMRNPATH)/etc/stellarisware.mk

LDFLAGSEXTRA = -L$(STELLARISWAREPATH)/driverlib/gcc-cm3 \
               -L$(STELLARISWAREPATH)/usblib/gcc-cm3
SYSLIBRARIESEXTRA = -lusb-cm3 -ldriver-cm3
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/StellarisCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/StellarisUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/StellarisUsbCdcDevice.o

CXXFLAGSEXTRA += -DTARGET_LM4F120XL

TARGET := freertos.armv7m
include $(OPENMRNPATH)/etc/prog.mk

