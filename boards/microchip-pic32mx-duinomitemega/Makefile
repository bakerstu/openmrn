APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
export TARGET := freertos.mips4k.pic32mx

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/Pic32mxCan.o \
	$(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/p32mx795f512h.o \

SYSLIBRARIESEXTRA = -lfreertos_drivers_plib_pic32mx795

include $(OPENMRNPATH)/etc/prog.mk

CORECFLAGS += -D_SUPPRESS_PLIB_WARNING
CFLAGS += -fgnu89-inline

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).hex $(EXECUTABLE).delf

%.delf: %.elf
	$(OBJCOPY) $$($(OBJDUMP) -h $< | grep ' b[df]' | cut  -b 5- | sed -e 's/^\([^ ]*\) .*$$/--change-section-lma \1-0xA0000000/' | tr '\n' ' ') $< $@
#	$(OBJDUMP) -h $< | grep ' b[df]'
#	cp -f $< $@

%.hex: %.delf
	$(OBJDUMP) -h $<
	$(OBJCOPY) -O ihex $< $@

endif  #MISSING_DEPS
