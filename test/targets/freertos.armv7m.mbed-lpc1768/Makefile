OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

TARGET := freertos.armv7m.mbed-lpc1768

DEPS += HAVE_MBED

SYSLIBRARIESEXTRA = -lmbed

OBJEXTRA = $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_can.o \
           $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_usbserial.o

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC1768 -d $@

$(EXECUTABLE)$(EXTENTION): depmake

.PHONY: depmake

depmake:
	+$(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) all

endif  #MISSING_DEPS
