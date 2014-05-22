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
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUsbCdcDevice.o

CXXFLAGSEXTRA += -DPART_TM4C1233H6PM

TARGET := freertos.armv7m
BOARD := BOARD_LAUNCHPAD_EK
include $(OPENMRNPATH)/etc/prog.mk

all: $(EXECUTABLE).lst

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d $< > $@
