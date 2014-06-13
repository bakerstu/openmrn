APP_PATH ?= $(realpath ../..)
include $(APP_PATH)/config.mk
include $(OPENMRNPATH)/etc/path.mk

export TARGET := freertos.armv7m
DEPS += LIBMAPLEPATH

LDFLAGSEXTRA = -L$(LIBMAPLEPATH)/support/ld/stm32/series/stm32f1/performance \
	-L$(LIBMAPLEPATH)/support/ld/stm32/mem/sram_20k_flash_128k  \
	-L$(LIBMAPLEPATH)/support/ld  \
	-L$(LIBMAPLEPATH)/support/ld/toolchains/gcc-arm-embedded  \

INCLUDES += \
	-I$(LIBMAPLEPATH)/wirish/include \
	-I$(LIBMAPLEPATH)/wirish \
	-I$(LIBMAPLEPATH)/libmaple/stm32f1/include \
	-I$(LIBMAPLEPATH)/libmaple \
	-I$(LIBMAPLEPATH)/libmaple/include \
	-I$(LIBMAPLEPATH)/libmaple/usb/usb_lib \
	-I$(LIBMAPLEPATH)/wirish/boards/maple/include \


#SYSLIBRARIESEXTRA = -lusb -ldriver
#OBJEXTRA = $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaCan.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUart.o \
#           $(OPENMRNPATH)/targets/freertos.armv7m/freertos_drivers/TivaUsbCdcDevice.o

include $(OPENMRNPATH)/etc/prog.mk

ARCHFLAGS += -DBOARD_maple -DMCU_STM32F103RB -DERROR_LED_PORT=GPIOA -DERROR_LED_PIN=5 -DVECT_TAB_FLASH -DBOOTLOADER_maple

CFLAGS += -Wno-strict-prototypes


VPATH:=$(VPATH):$(LIBMAPLEPATH)

CPPSRCS += wirish/boards.cpp \
	wirish/stm32f1/boards_setup.cpp \
	wirish/usb_serial.cpp \


ASMSRCS += wirish/start.S \

CSRCS += \
	libmaple/stm32f1/adc.c \
	libmaple/stm32f1/rcc.c \
	libmaple/stm32f1/gpio.c \
	libmaple/usb/stm32f1/usb_cdcacm.c \
	libmaple/usb/stm32f1/usb_reg_map.c \
	libmaple/usb/stm32f1/usb.c \
	libmaple/usb/usb_lib/usb_init.c \
	libmaple/usb/usb_lib/usb_core.c	\
	libmaple/usb/usb_lib/usb_regs.c \
	libmaple/usb/usb_lib/usb_mem.c \
	libmaple/gpio.c \
	libmaple/adc.c \
	libmaple/timer.c \
	libmaple/flash.c \
	libmaple/rcc.c \
	libmaple/nvic.c \
	libmaple/systick.c \
	wirish/start_c.c \


#maybe systick should not be there



$(CPPSRCS): wirish

$(EXECUTABLE)$(EXTENTION): $(OBJS)

.PHONY: wirish
wirish:
	mkdir -p wirish
	mkdir -p wirish/stm32f1
	mkdir -p libmaple/stm32f1
	mkdir -p libmaple/usb/stm32f1
	mkdir -p libmaple/usb/usb_lib

$(info objs=$(OBJS) )
