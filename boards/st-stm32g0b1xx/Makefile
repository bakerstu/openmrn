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

# Find STM32CubeG0 libraries
include $(OPENMRNPATH)/etc/stm32cubeg0.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubeg0b1xe
OBJEXTRA += 

CFLAGS += -DSTM32G0B1xx
CXXFLAGS += -DSTM32G0B1xx
include find-emulator.mk

ifndef TARGET
export TARGET := freertos.armv6m
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

# Prints the option bytes and other relevant memory areas. For debugging.
option_view_xds:
	openocd -f interface/xds110.cfg -c 'transport select swd' -c 'adapter_khz 950' -f target/stm32l4x.cfg -c 'reset_config separate' -c 'init' -c 'echo "option flash"' -c 'read_memory 0x1fff7800 32 8' -c 'echo "option reg"' -c 'read_memory 0x40022020 32 4' -c 'stm32l4x option_read 0 0x20'  -c 'stm32l4x option_read 0 0x2C' -c 'echo "flash begin"' -c 'read_memory 0x08000000 32 8' -c 'exit'

# Recovers a "level-1" locked microcontroller to an empty state. Uses an
# XDS110.
factory_reset_xds:
	openocd -f interface/xds110.cfg -c 'transport select swd' -c 'adapter_khz 950' -f target/stm32l4x.cfg -c 'reset_config separate' -c 'init' -c 'stm32l4x option_write 0 0x20 0xFFFFF8BB 0xFFFFFFFF' -c 'exit'
	$(TIUNIFLASH4PATH)/simplelink/imagecreator/bin/xds110reset
	@echo ========= power cycle the board ^ =========== ; echo press ENTER ; read
	openocd -f interface/xds110.cfg -c 'transport select swd' -c 'adapter_khz 950' -f target/stm32l4x.cfg -c 'reset_config separate' -c 'init' -c 'stm32l4x option_write 0 0x20 0xFFFFF8AA 0xFFFFFFFF' -c 'stm32l4x option_write 0 0x2C 0xFF00FFFF 0xFFFFFFFF' -c 'exit'
	@echo ========= power cycle the board ^^ =========== ; echo press ENTER ; read
	$(TIUNIFLASH4PATH)/simplelink/imagecreator/bin/xds110reset
	$(MAKE) option_view_xds


else

flash gdb:
	echo OPENOCD not found ; exit 1

endif
