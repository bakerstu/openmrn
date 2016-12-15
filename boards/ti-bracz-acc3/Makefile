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
export DEFAULT_ADDRESS := 0x4c

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \

SYSLIBRARIESEXTRA = -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaI2C.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o \

CXXFLAGSEXTRA += -DPART_TM4C123AE6PM

include $(OPENMRNPATH)/etc/prog.mk

OPENOCDARGS = -f ~/bin/olimex-armtinyh.cfg -f ~/bin/swd.cfg --file ~/bin/olcb-myb.cfg
#OPENOCDARGS = -f interface/olimex-arm-usb-tiny-h.cfg -f  ~/bin/olcb-myb.cfg

ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt"

rflash: $(EXECUTABLE).bin $(EXECUTABLE).lst
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -c tiva123 -r -n 0x0501010114$$(printf %02x $(ADDRESS)) -f $<

else

flash rflash gdb:
	echo missing openocd backend: $(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH) ; exit 1

endif
