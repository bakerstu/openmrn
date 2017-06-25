APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

OPENMRNPATH ?= $(realpath ../../../..)

TARGET := freertos.armv7m

# Find driverlib
include $(OPENMRNPATH)/etc/cc32xx.mk

LDFLAGSEXTRA +=
SYSLIBRARIESEXTRA +=
OBJEXTRA =

CXXFLAGSEXTRA += -DTARGET_IS_CC3200

include $(OPENMRNPATH)/etc/prog.mk


INCLUDES += -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx \
            -I$(OPENMRNPATH)/src/freertos_drivers/net_cc32xx/dummy \
            -I$(TICC3200SDKPATH)/simplelink/include \
            -I$(TICC3200SDKPATH)/simplelink/source \
            -I$(TICC3200SDKPATH)/simplelink \
            -I$(TICC3200SDKPATH)/oslib \
            -I$(TICC3200SDKPATH)/driverlib \
            -I$(TICC3200SDKPATH)/inc

CXXFLAGS += -include $(OPENMRNPATH)/src/freertos_drivers/net_cc32xx/CC3200/user.h \
            -D__USER_H__


ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)

#OPENOCDARGS = -f /opt/ti/CC3200SDK/default/cc3200-sdk/tools/gcc_scripts/cc3200.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f target/cc32xx.cfg
OPENOCDARGS = -c 'set TRANSPORT swd' -f board/ti-cc3200-launchxl.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
#OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
# This is for the RedBear wifi mini board
#OPENOCDARGS = -f interface/cmsis-dap.cfg -c 'cmsis_dap_vid_pid 0x0D28 0x0204' -c 'transport select swd' -c 'reset_config srst_only srst_gates_jtag' -f target/cc32xx.cfg



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
