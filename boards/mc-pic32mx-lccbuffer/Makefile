APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
export TARGET := freertos.mips4k.pic32mx

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx530/Pic32mxCan.o \
	$(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx530/p32mx530f128h.o \

SYSLIBRARIESEXTRA = -lfreertos_drivers_plib_pic32mx530

include $(OPENMRNPATH)/etc/prog.mk

CORECFLAGS += -D_SUPPRESS_PLIB_WARNING -D_DISABLE_OPENADC10_CONFIGSCAN_WARNING
PIC32PROC=530F128H
BASEDEFS += -D__PIC32_FEATURE_SET__=530
CFLAGS += -fgnu89-inline

ifeq ($(MISSING_DEPS),)

all: $(EXECUTABLE).hex $(EXECUTABLE).phy.hex $(EXECUTABLE).phy.elf $(EXECUTABLE).lst


%.phy.elf: %.elf
ifeq ($(OS),Windows_NT)
	$(OBJDUMP) -h $< > objcopy.input
	$(OPENMRNPATH)/bin/pic32_change_lma < objcopy.input > objcopy.params
else
#	$(OBJDUMP) -h $< > objcopy.input
#	$(OPENMRNPATH)/bin/pic32_change_lma < objcopy.input > objcopy.params
	$(OBJDUMP) -h $< | grep ' 9[df]' | cut  -b 5- | sed -e 's/^\([^ ]*\) .*$$/--change-section-lma \1-0x80000000/' | tr '\n' ' ' > objcopy.params
endif
	$(OBJCOPY) @objcopy.params $< $@
	rm -f objcopy.params objcopy.input

clean: clean-objcopy-params

clean-objcopy-params:
	rm -f $(wildcard $(EXECUTABLE).phy.*) $(EXECUTABLE).hex $(EXECUTABLE).delf objcopy.params objcopy.input


%.hex: %.elf
	$(OBJDUMP) -h $<
	$(OBJCOPY) -O ihex $< $@


flash: $(EXECUTABLE).phy.hex
	misc/pic32prog $<

endif  #MISSING_DEPS
