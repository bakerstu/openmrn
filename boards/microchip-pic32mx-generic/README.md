# Board support files for PIC32MX boards

## Compiler

The compilation and linking of binaries for the Microchip PIC32MX in OpenMRN
does not use the official Microchip toolchains. Instead, a vanilla
cross-compiler for MIPS4K processor cores is used. An easy way to get one is to
download the CodeSourcery cross-compiler for MIPS. The URL form which to
download is in etc/path.mk.

The main reason for this compiler choice is that Microchip charges money for
compiling code with any sensible optimization level, specifically it is not
possible to use xc32 free edition to generate mips16 code. The resulting code is
inefficient and uses a lot of flash.

The drawback of this choice is that Microchip's custom (non-standards-compliant)
syntax that they added into their compiler fork of GCC will not work. This is
usually a problem only if low-level code (e.g. interrupt vector routines) needs
to be ported from Microchip example code to OpenMRN.

## Booting

The PIC32MX has two flash banks: a large (e.g. 512K) user flash and a small
(e.g. 12K) boot flash. The processor always boots from the boot flash. Therefore
either a bootloader needs to be installed there, or the program needs to deposit
a reset trampoline to start. Both of these options are supported.

The default `target.ld` assumes a bootloader is present and it fits into the 12K
flash region, with the user app reset address configured to 0x9D000000. This is
the address where the reset function and the interrupt vectors will be put via
linking.

The alternative `target_no_bootloader.ld` will deposit the reset code and
interrupt vectors into the boot flash, and the application code starting at the
user flash. This assumes there is no bootloader and when flashed, will overwrite
if a bootloader is present.

## Bootloader

A compiled binary of the AN1388 bootloader for PIC32MX using USB-HID is in
`bootloader.hex`. This assumes LED and button pinout of the Duinomite Mega
board. This binary fits in 12K of flash.

To install the bootloader, flash `bootloader.hex` using a programmer like the
PicKit (2/3).

The source code for this bootloader is available from the microchip website
under AN1388, but to compile it using the free compiler, here is a fork with
appropriate changes and makefiles:

https://github.com/balazsracz/mr-misc/tree/master/pic32/bootloader

## Flashing

To flash a standalone binary, use the PicKit, and use the program pic32prog. The
source code including compiled binaries are available at
https://github.com/sergev/pic32prog
A linux-64 binary is deposited here in the `misc` folder.

*NOTE:* A common program for HID based microchip bootloaders called `mphidflash`
will NOT work.

### To flash the bootloader

Plug in the PicKit to the computer and the board (make sure your udev rules are
set up so your user can access it), then:

```
pic32prog bootloader.hex
```

### To flash an application binary via the bootloader

The same flashing tool knows how to talk to the AN1388 bootloader. Plug in the
board via USB. Press and hold the BUT button. Press brifly the RST button. The
green LED will start blinking about 2Hz, indicating that it's in bootloader
mode.

You can then do either `make flash` or by hand, using

```
misc/pic32prog async_blink.phy.hex
```

It is important that the .phy.hex be used, because an address translation needs
to be applied for the bootloader to be able to write to the flash.

The application binary must have been created using the `target.ld` file
designed for use with the bootloader.

### To flash an application binary using no bootloader

Unplug the board from USB. Plug in the PicKit to the computer and the board,
then:

```
misc/pic32prog async_blink.phy.hex
```

The application binary must have been created using the
`target_no_bootloader.ld` file designed for use standalone. This process will
erase a bootloader if one was present.

## Debugging

There is a tool to use as gdbserver for the PicKit 2/3:
https://github.com/sergev/ejtagproxy
A linux-64 binary is added to the `misc` folder.

To start a debugging session, go to the target directory, and do:

```
make gdb
```

Caveats:

- It seems that connecting with the debugger causes the processor to reset. That
  makes it impossible to figure out why the processor has crashed if you have
  not started the entire session via the debugger.
- gdb is incredibly slow via the ejtagproxy.
- I suspect There is something wrong with the clocks, and the processor is not
  running at the appropriate frequency. This is a problem due to the CAN clock
  being incorrectly set when running inside the debugger.

### Connecting via JTAG

It seems that using an Olimex ARM-USB-TINY-H emulator it is possible to connect
to the PIC32 with JTAG. The same software needs to be used as above.  The wiring
is in the datasheet: RB10,11,12,13 for TMS, TDO, TCK, TDI, respectively. The
reset does not need to be connected.

Connecting, pausing, running, backtrace, memory evaluation works fine.

Breakpoints do not seem to work.


## Drivers

Driver sources are located at `src/freertos_drivers/pic32mx`.

### Interrupts

To add a driver that uses an interrupt, edit the file `ISRwrapper.S`. Interrupts
must save and restore FreeRTOS context using the logic in this file.
