# Searches the connected USB devices for a known emulator for the
# STM32G0. Exports the OPENOCDARGS variable when found.
# All emulators here are configured to use 2-wire SWD mode.

USBLIST=$(shell lsusb)

ifneq ($(findstring 0483:374b,$(USBLIST)),)
$(info emulator ST-Link V2.1 or nucleo)
OPENOCDARGS = -f board/st_nucleo_g0.cfg
else ifneq ($(findstring 0451:bef3,$(USBLIST)),)
$(info emulator TI CC3220SF launchpad CMSIS-DAP)
OPENOCDARGS = -f interface/xds110.cfg -c 'transport select swd' -c 'adapter_khz 950' -f target/stm32g0x.cfg -c 'reset_config srst_only' -c 'adapter_khz 950'
else ifneq ($(findstring 15ba:002a,$(USBLIST)),)
$(info emulator Olimex arm-usb-tiny-H)
OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/stm32g0x.cfg
else ifneq ($(findstring 15ba:002b,$(USBLIST)),)
$(info emulator Olimex arm-usb-ocd-H)
OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-ocd-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/stm32g0x.cfg
else
OPENOCDARGS = " -emulator-not-found-add-your-usb-device-to-boards_st-stm32f0x_find-emulator.mk "
endif
