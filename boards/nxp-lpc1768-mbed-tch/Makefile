APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

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


endif  #MISSING_DEPS
