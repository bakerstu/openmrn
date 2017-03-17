APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(realpath ../../../..)

TARGET := freertos.armv7m

# Find driverlib
include $(OPENMRNPATH)/etc/cc322x.mk

LDFLAGSEXTRA +=
SYSLIBRARIESEXTRA +=
OBJEXTRA =

CXXFLAGSEXTRA += -DTARGET_IS_CC3200

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)

#OPENOCDARGS = -f /opt/ti/CC3200SDK/default/cc3200-sdk/tools/gcc_scripts/cc3200.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f target/cc32xx.cfg
OPENOCDARGS = -c 'set TRANSPORT swd' -f board/ti-cc3200-launchxl.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
#OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg

run: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "source $(OPENMRNPATH)/boards/ti-cc3200-generic/utils.gdb" -ex "monitor reset halt" -ex "load" -ex "reset"

# "monitor reset init" -ex "monitor reset run"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt"

else

flash gdb:
	echo missing openocd backend: $(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH) ; exit 1

endif
