# Searches the connected USB devices for a known emulator for the
# CC32xx. Exports the OPENOCDARGS variable when found.
# All emulators here are configured to use 2-wire SWD mode.

USBLIST=$(shell lsusb)

ifneq ($(findstring 15ba:002a,$(USBLIST)),)
$(info emulator Olimex arm-usb-tiny-H)
OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
else ifneq ($(findstring 0451:c32a,$(USBLIST)),)
$(info emulator TI CC3200 launchpad FTDI)
OPENOCDARGS = -c 'set TRANSPORT swd' -f board/ti-cc3200-launchxl.cfg
else
OPENOCDARGS = " -emulator-not-found-add-your-usb-device-to-openmrn_boards_ti-cc3200-generic_find-emulator.mk "
endif


# Here are some more alternatives
#OPENOCDARGS = -f /opt/ti/CC3200SDK/default/cc3200-sdk/tools/gcc_scripts/cc3200.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f target/cc32xx.cfg
#OPENOCDARGS = -c 'set TRANSPORT swd' -f board/ti-cc3200-launchxl.cfg
#OPENOCDARGS = -f interface/cmsis-dap.cfg -c 'transport select swd' -c 'adapter_khz 1000' -f target/cc32xx.cfg -c 'reset_config connect_deassert_srst'
#OPENOCDARGS = -f interface/cmsis-dap.cfg -c 'transport select swd' -f target/cc32xx.cfg -c 'reset_config srst_only'
# This is for using a CC3220 launchpad or XDS110 debug probe with a CC3200 device
#OPENOCDARGS = -f interface/cmsis-dap.cfg -c 'cmsis_dap_vid_pid 0x0451 0xbef3' -c 'transport select swd' -c 'reset_config srst_only srst_gates_jtag' -f target/cc32xx.cfg
#OPENOCDARGS = -f interface/ftdi/ti-icdi.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
#OPENOCDARGS = -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f interface/ftdi/swd-resistor-hack.cfg -f target/cc32xx.cfg
