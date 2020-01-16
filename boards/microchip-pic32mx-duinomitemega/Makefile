APP_PATH ?= $(realpath ../..)
-include $(APP_PATH)/config.mk
export TARGET := freertos.mips4k.pic32mx

OBJEXTRA = $(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/Pic32mxCan.o \
	$(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/drivers_pic32mx795/p32mx795f512h.o \
	$(OPENMRNPATH)/targets/freertos.mips4k.pic32mx/freertos_drivers/usb_pic32mx795/Pic32mxUsbCdcDevice.o \

SYSLIBRARIESEXTRA = -lfreertos_drivers_plib_pic32mx795 -lfreertos_drivers_usb_pic32mx795 -lfreertos_drivers_drivers_pic32mx795

-include local_config.mk
include $(OPENMRNPATH)/etc/prog.mk

CORECFLAGS += -D_SUPPRESS_PLIB_WARNING
PIC32PROC=795F512H
BASEDEFS += -D__32MX795F512H__ -D__PIC32_FEATURE_SET__=795
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

# These commands can be used to debug the pic32mx target using a pickit2.
gdbpk2:
	true # killall -9 ejtagproxy || true
	true # misc/ejtagproxy & disown # needs to be launched in a separate window.
	$(GDB) $(EXECUTABLE).elf -x misc/gdb_boot.gdb

OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -c 'transport select jtag' -c 'adapter_khz 1000' -c 'set CPUTAPID 0x1430e053' -f target/pic32mx.cfg -c 'gdb_memory_map disable'

gdb:
	@if ps ax -o comm | grep -q openocd ; then echo openocd already running. quit existing first. ; exit 1 ; fi
	$(GDB) $(EXECUTABLE)$(EXTENTION) -ex "target remote | $(OPENOCDPATH)/openocd -c \"gdb_port pipe\" --search $(OPENOCDSCRIPTSPATH) $(OPENOCDARGS)" -ex " set remote hardware-breakpoint-limit 6" -ex "set remote hardware-watchpoint-limit 2"



endif  #MISSING_DEPS
