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
export BOARD := BOARD_BRACZ_RAILCOM
endif

OPENOCDARGS = -f board/ek-lm4f120xl.cfg

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaI2C.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUsbCdcDevice.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o

CXXFLAGSEXTRA += -DPART_TM4C123GH6PM

include $(OPENMRNPATH)/etc/prog.mk


$(info $(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH))
ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "continue" # -ex "monitor reset halt"


else

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	stellaris-flash.sh $<

gdb:
	stellaris-gdb.sh $(EXECUTABLE)$(EXTENTION)

endif


ifndef ADDRESS
ADDRESS=0x62
endif
ADDRHIGH ?=0x14
ifndef ADDRHIGH
ADRTEXT+=;\#define NODEID_HIGH_BITS $(ADDRHIGH)
endif

rflash: $(EXECUTABLE).bin
	$(MAKE) -C $(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -c tiva123 -r -n 0x05010101$$(printf %02x $(ADDRHIGH))$$(printf %02x $(ADDRESS)) -f $<


$(EXECUTABLE).lflash.bin: $(EXECUTABLE).bin $(EXECUTABLE).lst
	@[ -x $(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client ] || (echo ======================== ; echo ***please run: ; echo make -C $(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/ ; exit 1)
	$(OPENMRNPATH)/applications/bootloader_client/targets/linux.x86/bootloader_client -c tiva123 -f $< -D $@

lflash: $(EXECUTABLE).lflash.bin
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "set remotetimeout 6000" -ex "monitor program $< 0x00000000 verify reset exit" -ex "detach" -ex "quit"

