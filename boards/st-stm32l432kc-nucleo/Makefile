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

# Find STM32CubeL4 libraries
include $(OPENMRNPATH)/etc/stm32cubel4.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubel432xx
OBJEXTRA += 

CFLAGS += -DSTM32L432xx
CXXFLAGS += -DSTM32L432xx
OPENOCDARGS = -f board/st_nucleo_l476rg.cfg

ifndef TARGET
export TARGET := freertos.armv7m
endif
include $(OPENMRNPATH)/etc/prog.mk

ifndef DEFAULT_ADDRESS
DEFAULT_ADDRESS=0x16
endif

include $(OPENMRNPATH)/etc/node_id.mk

# How to use: make multibin ADDRESS=0x20 ADDRHIGH=0x45 NUM=3
# starting address, high bits (user range), count
multibin:
	for i in $$(seq 1 $(NUM)) ; do $(MAKE) $(EXECUTABLE).bin ADDRESS=$$(printf 0x%02x $$(($(ADDRESS)+$$i))) ; cp $(EXECUTABLE).bin $(EXECUTABLE).f303.$$(printf %02x%02x $(ADDRHIGH) $$(($(ADDRESS)+$$i-1))).bin ; done

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
