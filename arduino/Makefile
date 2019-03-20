all:

ELFFILE=$(shell ls /tmp/arduino_build_*/*elf -r | tail -n 1)
ESP32BINDIR=$(wildcard $(HOME)/.arduino15/packages/esp32/tools/xtensa-esp32-elf-gcc/*/bin/)

PREFIX=$(ESP32BINDIR)/xtensa-esp32-elf-
GDB=$(PREFIX)gdb
OBJDUMP=$(PREFIX)objdump
NM=$(PREFIX)nm

OPENOCD=/opt/openocd/default-esp32/src/openocd
OPENOCDSCRIPTS=/opt/openocd/default-esp32/tcl

gdbx:
	$(GDB) $(ELFFILE)

gdb:
	killall -9 openocd || exit 0
	$(OPENOCD) --search $(OPENOCDSCRIPTS) -f interface/ftdi/olimex-arm-usb-tiny-h.cfg  -f board/esp32-solo-1.cfg -c "adapter_khz 1000" & disown
	sleep 2
	$(GDB) $(ELFFILE) -ex "target remote :3333" -ex "set remotetimeout 6000"  -ex "set remote hardware-watchpoint-limit 2" -ex "mon reset halt" -ex "flushregs" -ex "thb app_main" -ex "c"
	killall -9 openocd || exit 0

#	$(OPENOCD) --search $(OPENOCDSCRIPTS) -f interface/ftdi/olimex-arm-usb-tiny-h.cfg  -f board/esp-wroom-32.cfg -c "adapter_khz 1000" & disown


#target remote :3333
#set remote hardware-watchpoint-limit 2
#mon reset halt
#flushregs
#thb app_main
#c

lst: 
	$(OBJDUMP) -C -d -h $(ELFFILE) > target.lst

nm:
	$(NM) -C $(ELFFILE) | sort > target.nm
