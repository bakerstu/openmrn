# Searches the connected USB devices for a known emulator for the
# TI MSPM0. Exports the OPENOCDARGS variable when found.
# All emulators here are configured to use 2-wire SWD mode.

USBLIST=$(shell lsusb)

ifneq ($(findstring 0483:374b,$(USBLIST)),)
$(info emulator ST-Link V2.1 or nucleo)
OPENOCDARGS = -f interface/stlink.cfg -c 'transport select swd' -c 'adapter_khz 950' -f target/ti_mspm0.cfg
else ifneq ($(findstring 0451:bef3,$(USBLIST)),)
$(info emulator TI CC3220SF launchpad CMSIS-DAP)
OPENOCDARGS = -f board/ti_mspm0_launchpad.cfg
else ifneq ($(findstring 15ba:002a,$(USBLIST)),)
$(info emulator Olimex arm-usb-tiny-H)
OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/ti_mspm0.cfg
else ifneq ($(findstring 15ba:002b,$(USBLIST)),)
$(info emulator Olimex arm-usb-ocd-H)
OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/ti_mspm0.cfg
else
OPENOCDARGS = " -emulator-not-found-add-your-usb-device-to-boards_ti_mspm0l1x_find-emulator.mk "
endif
