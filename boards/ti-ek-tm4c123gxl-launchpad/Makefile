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

export TARGET ?= freertos.armv7m
ifndef BOARD
export BOARD := BOARD_LAUNCHPAD_EK
endif

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \
               -L$(TIVAWAREPATH)/usblib/gcc
SYSLIBRARIESEXTRA = -lusb -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaDCC.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaI2C.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUsbCdcDevice.o

CXXFLAGSEXTRA += -DPART_TM4C123GH6PM

include $(OPENMRNPATH)/etc/prog.mk

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	stellaris-flash.sh $<

gdb:
	stellaris-gdb.sh $(EXECUTABLE)$(EXTENTION)

ifndef ADDRESS
ADDRESS=0x04
endif

rflash: $(EXECUTABLE).bin
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -c tiva123 -r -n 0x050101011F$$(printf %02x $(ADDRESS)) -f $<
