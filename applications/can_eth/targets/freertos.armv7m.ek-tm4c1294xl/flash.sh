#!/bin/sh
/Applications/Energia.app/Contents/Resources/Java/hardware/tools/lm4f/bin/arm-none-eabi-objcopy -O binary $1.elf $1.bin
/Applications/Energia.app/Contents/Resources/Java/hardware/tools/lm4f/bin/lm4flash -v $1.bin
