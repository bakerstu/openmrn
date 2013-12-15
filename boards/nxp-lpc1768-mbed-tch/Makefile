APP_PATH ?= $(realpath ../..)
include $(APP_PATH)/config.mk

export TARGET := freertos.armv7m.mbed-lpc1768

DEPS += HAVE_MBED

SYSLIBRARIESEXTRA = -lmbed

OBJEXTRA = $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_can.o \
           $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_usbserial.o \


include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d -C $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC1768 -d $@

#rr-crossing.c : rr-crossing.cout

.PHONY: rr-crossing.cout

rr-crossing.cout :
	+$(MAKE) -C $(APP_PATH)/../automata rr-crossing.cout
	cp -f $(APP_PATH)/../automata/rr-crossing.cout $@

cleangen:
	rm -f rr-crossing.cout

veryclean clean: cleangen

endif  #MISSING_DEPS
