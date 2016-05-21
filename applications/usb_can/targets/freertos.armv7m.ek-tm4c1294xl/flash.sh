#!/bin/sh
/Applications/Energia.app/Contents/Resources/Java/hardware/tools/lm4f/bin/arm-none-eabi-objcopy -O binary usb_can.elf usb_can.bin
/Applications/Energia.app/Contents/Resources/Java/hardware/tools/lm4f/bin/lm4flash -v usb_can.bin
