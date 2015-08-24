-include ../../config.mk
OPENMRNPATH ?= $(shell \
sh -c "if [ \"X`printenv OPENMRNPATH`\" != \"X\" ]; then printenv OPENMRNPATH; \
     elif [ -d /opt/openmrn/src ]; then echo /opt/openmrn; \
     elif [ -d ~/openmrn/src ]; then echo ~/openmrn; \
     elif [ -d ../../../src ]; then echo ../../..; \
     else echo OPENMRNPATH not found; fi" \
)

# Find STM32CubeF1 libraries
include $(OPENMRNPATH)/etc/stm32cubef1.mk


LDFLAGSEXTRA += 
SYSLIBRARIESEXTRA += -lfreertos_drivers_stm32cubef103xb
OBJEXTRA += 

CFLAGS += -DSTM32F103xB
CXXFLAGS += -DSTM32F103xB

# This is the usb ID that the maple bootloader presents to us. The production
# device (after we switched to user code) has a different ID, so if we failed
# to reset the chip we will not talk DFU.
BOARD_USB_VENDOR_ID  := 1EAF
BOARD_USB_PRODUCT_ID := 0003

TARGET := freertos.armv7m
include $(OPENMRNPATH)/etc/prog.mk

flash: $(EXECUTABLE).bin all
	echo hit the reset button && \
        sleep 2                  && \
        dfu-util -a1 -d $(BOARD_USB_VENDOR_ID):$(BOARD_USB_PRODUCT_ID) -D $< -R

gdb:
	maple-gdb.sh $(EXECUTABLE).elf
