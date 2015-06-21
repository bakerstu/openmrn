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

LDFLAGSEXTRA = -L$(TIVAWAREPATH)/driverlib/gcc \
               -L$(TIVAWAREPATH)/usblib/gcc

SYSLIBRARIESEXTRA = -lusb -ldriver
OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaCan.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUart.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaDCC.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaUsbCdcDevice.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaFlash.o \
           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/tivaware/TivaEEPROMEmulation.o \



CXXFLAGSEXTRA += -DPART_TM4C1294NCPDT

TARGET := freertos.armv7m
ifndef BOARD
BOARD := BOARD_LAUNCHPAD_EK
endif

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(call find_missing_deps,OPENOCDSCRIPTSPATH OPENOCDPATH),)
flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	if ps l -C openocd | grep -q openocd.*ek-tm4c129 ; then echo killing openocd ; ps l -C openocd | grep  openocd.*ek-tm4c129 | cut -d ' ' -f 3 | xargs kill ; sleep 2 ; fi
	$(PREFIX)gdb $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --file board/ek-tm4c1294xl.cfg --search $(OPENOCDSCRIPTSPATH)" -ex 'monitor reset halt' -ex 'load' -ex 'monitor reset init' -ex 'monitor reset run' -ex 'detach' -ex 'quit'
#	./flash.sh $(EXECUTABLE)$(EXTENTION)

gdb:
	if ps l -C openocd | grep -q openocd.*ek-tm4c129 ; then echo openocd already running. ; exit 1 ; fi
	$(PREFIX)gdb $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --file board/ek-tm4c1294xl.cfg --search $(OPENOCDSCRIPTSPATH)" -ex 'monitor halt' 

reboot:
	if ps l -C openocd | grep -q openocd.*ek-tm4c129 ; then echo openocd already running. ; exit 1 ; fi
	$(PREFIX)gdb $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --file board/ek-tm4c1294xl.cfg --search $(OPENOCDSCRIPTSPATH)" -ex 'monitor reset init' -ex 'monitor reset run' -ex 'detach' -ex 'quit' 


endif
