APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET := freertos.armv7m

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find TivaWare
include $(OPENMRNPATH)/etc/tivaware.mk

# Find FreeRTOSTCP
include $(OPENMRNPATH)/etc/freertos_tcp.mk

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUsbCdcDevice.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaFlash.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o \


CXXFLAGSEXTRA += -DPART_TM4C1294NCPDT

TARGET := freertos.armv7m
ifndef BOARD
BOARD := BOARD_LAUNCHPAD_EK
endif
export BOARD

OPENOCDARGS = -f board/ek-tm4c1294xl.cfg

# this works for XDS110 with SWD (2 wire)
#OPENOCDARGS = -c 'set TRANSPORT swd' -f interface/xds110.cfg -c 'transport select swd' -c 'set WORKAREASIZE 0x8000' -c 'set CHIPNAME tm4c1294ncpdt' -f target/stellaris.cfg

#OPENOCDARGS = -f interface/xds110.cfg -c 'set WORKAREASIZE 0x8000' -c 'set CHIPNAME tm4c1294ncpdt' -f target/stellaris.cfg

# this works for XDS110 with JTAG (4 wire)
#OPENOCDARGS = -f interface/xds110.cfg  -c 'transport select jtag' -c 'set WORKAREASIZE 0x8000' -c 'set CHIPNAME tm4c1294ncpdt' -f target/stellaris.cfg

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(call find_missing_deps,OPENOCDSCRIPTSPATH OPENOCDPATH),)
flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	cp $< last-flashed-$<
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset init"  -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt"

reboot:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex 'monitor reset init' -ex 'monitor reset run' -ex 'detach' -ex 'quit' 


endif
