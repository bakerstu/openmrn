APP_PATH ?= $(realpath ../..)

include $(APP_PATH)/config.mk

export TARGET := freertos.armv4t

DEPS += HAVE_MBED

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv4t/freertos_drivers/mbed_can.o \
	$(OPENMRNPATH)/targets/freertos.armv4t/freertos_drivers/mbed_i2c.o \
	$(OPENMRNPATH)/targets/freertos.armv4t/freertos_drivers/mbed_usbserial.o

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin $(EXECUTABLE).lst

%.lst: %.elf
	$(SIZE) $<
	$(OBJDUMP) -d -C $< > $@

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC2362 -d $@

$(EXECUTABLE)$(EXTENTION): depmake

.PHONY: depmake

#depmake:
#	+$(MAKE) -C $(OPENMRNPATH)/targets/$(TARGET) all

endif  #missing deps
