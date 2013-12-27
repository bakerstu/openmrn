APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk

export TARGET := freertos.armv6m

DEPS += HAVE_MBED

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/11cxx_async_can.o \


#           $(OPENMRNPATH)/targets/freertos.armv6m/freertos_drivers/mbed_i2c.o \


SYSLIBRARIESEXTRA = -lmbed

LDFLAGSEXTRA = -Wl,--wrap=__cxa_pure_virtual  -Wl,--wrap=__cxa_atexit  -Wl,--wrap=exit

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
