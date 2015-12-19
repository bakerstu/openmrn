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
export BOARD := BOARD_BRACZ_ACC

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \

SYSLIBRARIESEXTRA = -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaI2C.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o \


CXXFLAGSEXTRA += -DPART_TM4C123AE6PM

include $(OPENMRNPATH)/etc/prog.mk

ifndef ADDRESS
ADDRESS=0x4a
endif

CXXFLAGS+= -DNODEID_LOW_BITS=$(ADDRESS)


flash: $(EXECUTABLE).elf $(EXECUTABLE).lst
	ti-flash.sh $<

rflash: $(EXECUTABLE).bin $(EXECUTABLE).lst
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -c tiva123 -r -n 0x0501010114$$(printf %02x $(ADDRESS)) -f $<

#	train enter-bootloader 354



gdb:
	ti-gdb.sh $(EXECUTABLE).elf

# This will force re-building NodeId.cxx every invocation, because the address
# might have changed.
NodeId.o: FORCE

FORCE:
