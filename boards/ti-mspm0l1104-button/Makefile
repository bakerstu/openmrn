APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
-include local_config.mk

OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find MSPM0 SDK libraries
include $(OPENMRNPATH)/etc/ti-mspm0sdk.mk

CFLAGS += -D__MSPM0L1304__
CXXFLAGS += -D__MSPM0L1304__

OBJEXTRA += $(TIMSPM0SDKPATH)/source/ti/driverlib/lib/gcc/m0p/mspm0l11xx_l13xx/driverlib.a

include $(OPENMRNPATH)/boards/ti-mspm0l1x-generic/find-emulator.mk

export TARGET := bare.armv6m
include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)
all: $(EXECUTABLE).bin

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "continue" # -ex "monitor reset halt"

else

flash gdb:
	echo OPENOCD not found ; exit 1

endif
