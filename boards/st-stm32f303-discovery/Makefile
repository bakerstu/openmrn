-include ../../config.mk
OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find STM32CubeF0 libraries
include $(OPENMRNPATH)/etc/stm32cubef3.mk

LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA +=  -Wl,--whole-archive \
                      -lfreertos_drivers_stm32cubef303x_28x_58x_98x \
                      -Wl,--no-whole-archive
OBJEXTRA += 

CFLAGS += -DSTM32F303xC
CXXFLAGS += -DSTM32F303xC
OPENOCDARGS = -f board/stm32f3discovery.cfg

TARGET := freertos.armv7m
include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(call find_missing_deps,OPENOCDPATH OPENOCDSCRIPTSPATH),)

flash: $(EXECUTABLE)$(EXTENTION) $(EXECUTABLE).lst
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $< -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt" -ex "load" -ex "monitor reset init" -ex "monitor reset run"  -ex "detach" -ex "quit"

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex "monitor reset halt"

else

flash gdb:
	echo OPENOCD not found ; exit 1

endif
