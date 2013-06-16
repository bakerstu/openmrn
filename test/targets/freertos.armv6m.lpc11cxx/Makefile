OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

TARGET := freertos.armv6m

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/11CXX_rom_can.o

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

$(EXECUTABLE)$(EXTENTION): target.ld

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC11C24 -d $@

endif #MISSING_DEPS
