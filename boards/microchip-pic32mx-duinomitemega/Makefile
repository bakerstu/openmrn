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

all: $(EXECUTABLE).hex $(EXECUTABLE).delf $(EXECUTABLE).lst


%.delf: %.elf
ifeq ($(OS),Windows_NT)
	$(OBJDUMP) -h $< > objcopy.input
	$(OPENMRNPATH)/bin/pic32_change_lma < objcopy.input > objcopy.params
else
	$(OBJDUMP) -h $< | grep ' b[df]' | cut  -b 5- | sed -e 's/^\([^ ]*\) .*$$/--change-section-lma \1-0xA0000000/' | tr '\n' ' ' > objcopy.params
endif
	$(OBJCOPY) @objcopy.params $< $@
	rm -f objcopy.params objcopy.input

clean: clean-objcopy-params
	
clean-objcopy-params:
	rm -f objcopy.params objcopy.input


%.hex: %.delf
	$(OBJDUMP) -h $<
	$(OBJCOPY) -O ihex $< $@


flash: $(EXECUTABLE).hex
	misc/pic32prog $<

endif  #MISSING_DEPS
