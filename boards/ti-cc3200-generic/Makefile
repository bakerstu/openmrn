APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find driverlib
include $(OPENMRNPATH)/etc/cc3200.mk

LDFLAGSEXTRA = -L$(TICC3200SDKPATH)/driverlib/gcc/exe
SYSLIBRARIESEXTRA = -ldriver
OBJEXTRA =

#$(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaCan.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUart.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUsbCdcDevice.o

CXXFLAGSEXTRA += -DTARGET_IS_CC3200

TARGET := freertos.armv7m
#BOARD := BOARD_LAUNCHPAD_EK
include $(OPENMRNPATH)/etc/prog.mk
