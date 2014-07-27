APP_PATH ?= $(realpath ../..)

include $(APP_PATH)/config.mk

export TARGET := freertos.armv4t

include $(OPENMRNPATH)/etc/mbed.mk
DEPS += MBEDSRCPATH

OBJEXTRA = $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/drivers_lpc2368/mbed_can.o \
	$(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/drivers_lpc2368/mbed_async_usbserial.o \
	$(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/drivers_lpc2368/stack_malloc.o \

#	$(OPENMRNPATH)/targets/freertos.armv4t/freertos_drivers/mbed_i2c.o \

CORECFLAGS += -DTARGET_LPC2368
INCLUDES += \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC2368" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC2368"

CGMINSIZE=2000

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC2362 -d $@

flash: $(EXECUTABLE).bin
	panda-flash.sh $<

endif  #missing deps
