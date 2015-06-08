APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find TivaWare
include $(OPENMRNPATH)/etc/tivaware.mk

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \
               -L$(TIVAWAREPATH)/usblib/gcc
SYSLIBRARIESEXTRA = -lusb -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaDCC.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUsbCdcDevice.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o

CXXFLAGSEXTRA += -DPART_TM4C1233H6PM -DSNIFF_ON_SERIAL

TARGET := freertos.armv7m
BOARD := BOARD_LAUNCHPAD_EK


include $(OPENMRNPATH)/etc/prog.mk

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	stellaris-flash.sh $<

gdb:
	stellaris-gdb.sh $(EXECUTABLE)$(EXTENTION)
