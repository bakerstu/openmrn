-include ../../config.mk

export TARGET := freertos.armv7m

include $(OPENMRNPATH)/etc/mbed.mk
DEPS += HAVE_MBED

SYSLIBRARIESEXTRA = -lfreertos_drivers_mbed_lpc1768

OBJEXTRA = $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_lpc1768/mbed_can.o \
           $(OPENMRNPATH)/targets/$(TARGET)/freertos_drivers/mbed_lpc1768/mbed_usbserial.o \

CORECFLAGS += -DTARGET_LPC1768 -DINTERRUPT_ATTRIBUTE=
INCLUDES += \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/capi/LPC1768" \
	-I"$(MBEDPATH)/mbed/vendor/NXP/cmsis/LPC1768"

include $(OPENMRNPATH)/etc/prog.mk

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).bin

%.bin: %.elf
	$(OBJCOPY) -O binary $< $@
	$(CHECKSUM) -p LPC1768 -d $@

endif  #MISSING_DEPS
