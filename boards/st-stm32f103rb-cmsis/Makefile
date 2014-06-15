APP_PATH ?= $(realpath ../..)
include $(APP_PATH)/config.mk
include $(OPENMRNPATH)/etc/path.mk

export TARGET := freertos.armv7m
DEPS += STM32PLIBPATH

#LDFLAGSEXTRA = 


DISCARD += \
	-L$(LIBMAPLEPATH)/support/ld/stm32/series/stm32f1/performance \
	-L$(LIBMAPLEPATH)/support/ld/stm32/mem/sram_20k_flash_128k  \
	-L$(LIBMAPLEPATH)/support/ld  \
	-L$(LIBMAPLEPATH)/support/ld/toolchains/gcc-arm-embedded  \
	\
	-I$(LIBMAPLEPATH)/wirish/include \
	-I$(LIBMAPLEPATH)/wirish \
	-I$(LIBMAPLEPATH)/libmaple/stm32f1/include \
	-I$(LIBMAPLEPATH)/libmaple \
	-I$(LIBMAPLEPATH)/libmaple/include \
	-I$(LIBMAPLEPATH)/libmaple/usb/usb_lib \
	-I$(LIBMAPLEPATH)/wirish/boards/maple/include \



INCLUDES += \
	-I. \
	-I$(STM32PLIBPATH)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x \
	-I$(STM32PLIBPATH)/Libraries/STM32F10x_StdPeriph_Driver/inc \
	-I$(STM32PLIBPATH)/Libraries/CMSIS/CM3/CoreSupport \






#SYSLIBRARIESEXTRA = -lusb -ldriver
#OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaCan.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUart.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUsbCdcDevice.o

include $(OPENMRNPATH)/etc/prog.mk

ARCHFLAGS += -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER

# -DBOARD_maple -DMCU_STM32F103RB -DERROR_LED_PORT=GPIOA -DERROR_LED_PIN=5 -DVECT_TAB_FLASH -DBOOTLOADER_maple

CFLAGS += -Wno-strict-prototypes

ASFLAGS += $(INCLUDES)

VPATH:=$(VPATH):$(STM32PLIBPATH)/Libraries/CMSIS/CM3/CoreSupport:$(STM32PLIBPATH)/Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x:$(STM32PLIBPATH)/Libraries/STM32F10x_StdPeriph_Driver/src

CSRCS += \
	core_cm3.c \
	system_stm32f10x.c \
	misc.c \
	stm32f10x_gpio.c \
	stm32f10x_rcc.c \

#maybe systick should not be there

$(info objs=$(OBJS) )
$(info vpath=$(VPATH) )

$(EXECUTABLE)$(EXTENTION): $(OBJS)

# This is the usb ID that the maple bootloader presents to us. The production
# device (after we switched to user code) has a different ID, so if we failed
# to reset the chip we will not talk DFU.
BOARD_USB_VENDOR_ID  := 1EAF
BOARD_USB_PRODUCT_ID := 0003

$(EXECUTABLE).bin: $(EXECUTABLE)$(EXTENTION)
	$(OBJCOPY) -O binary $< $@

flash: $(EXECUTABLE).bin all
	echo hit the reset button && \
        sleep 2                  && \
        dfu-util -a1 -d $(BOARD_USB_VENDOR_ID):$(BOARD_USB_PRODUCT_ID) -D $< -R

# Due to a bug in the CMSIS version we need to compile the core support
# libraries without optimization to avoid an error like this:
# Error: registers may not be the same -- `strexb r0,r0,[r1]'
core_cm3.o : CFLAGS+=-O0
