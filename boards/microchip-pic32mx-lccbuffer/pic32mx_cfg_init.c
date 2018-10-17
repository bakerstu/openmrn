/* pic32mx_cfg_init.c -- Initialize PIC32MX "configuration" registers
**
** PIC32MX initialization requires 3 configuration registers, which are
** read from reserved flash locations during chip reset:
** - initial values for configuration registers live at top of PIC32MX
**   "boot flash", at addresses 9FC0.2FF4, 2FF8, and 2FFC
** - Additional "ID value" for arbitrary use is at 9FC0.2FF0
**
** This module defines the static values for the configuration registers,
** and locates them in a specific "section". The linker script then
** places this section at the correct address.
**
** See "Configuration Registers" DEVCFG0, DEVCFG1, and DEVCFG2 in the
** PIC32MX documentation. DEVCFG3 is in the same address range but is
** simply a user-defined ID value (not related to startup processing).
**
**
** Note: Microchip added "#pragma config"'s for configuration register
** initialization to their proprietary build of GNU C++. Unfortunately:
** - Microchip additions are incompatible other alternative compilers,
** - pragmas get scattered around the code and possibly duplicated
**  (see Microchip sample code for examples of why this is a bad idea)
**
** This file replaces the Microchip proprietary mechanism with
** something simpler, centralized, and usable across all GNU-based builds.
**
**
** Copyright (c) 2009 DRNadler - All rights reserved.
**
** The authors hereby grant permission to use, copy, modify, distribute,
** and license this software and its documentation for any purpose, provided
** that existing copyright notices are retained in all copies and that this
** notice is included verbatim in any distributions.  No written agreement,
** license, or royalty fee is required for any of the authorized uses.
** Modifications to this software may be copyrighted by their authors
** and need not follow the licensing terms described here, provided that
** the new terms are clearly indicated on the first page of each file where
** they apply.
*/

/**

   Changelog:

   2013-08-10 Balazs Racz: updated for PIC32MX795 (changes in devcfg3) and
   fixed a few bugs and comments.
 * 
   2016-12-21 Rob Heikens: updated for PIC32MX530

 */

// Usage:
// - fill in your desired value for the "userid" word
// - uncomment EXACTLY ONE of the bitfield definitions for each bitfield

const unsigned int cfg_words[4] __attribute__((section(".PIC32MX_cfg_init"))) = {
    // DEVCFG3 at .0BF0 - user-defined ID word
    0x0FFFFFFF | // all unused bits remain set to '1'

    0x10000000 | // PMDL1WAY allow only one reconfiguration
//  0x00000000 | // PMDL1WAY allow multiple reconfigurations

    0x20000000 | // IOL1WAY allow only one reconfiguration
//  0x00000000 | // IOL1WAY allow multiple reconfigurations

//  0x40000000 | // USBID pin is controlled by USB module
    0x00000000 | // USBID pin is a port.

//  0x80000000 | // Vbuson pin is controlled by USB module
    0x00000000 | // Vbuson pin is a port.

    0,

    // DEVCFG2 at .0BF4 - PLL control word
    0xFFF87888 | // all unused bits remain set to '1'
    // 00070000:FPLLODIV:PLL Output Divider Value
//  0x00000000 | // Divide by 1
    0x00010000 | // Divide by 2
//  0x00020000 | // Divide by 4
//  0x00030000 | // Divide by 8
//  0x00040000 | // Divide by 16
//  0x00050000 | // Divide by 32
//  0x00060000 | // Divide by 64
//  0x00070000 | // Divide by 256
    // 00008000:UPLLEN:USB PLL Enable bit
    0x00000000 | // Enabled
//  0x00008000 | // Disabled
    // 00000700:UPLLIDIV:USB PLL Input Divider bits
//  0x00000000 | // Divide by 1
    0x00000100 | // Divide by 2 - 8MHz crystal => 4MHz required USB clock
//  0x00000200 | // Divide by 3
//  0x00000300 | // Divide by 4
//  0x00000400 | // Divide by 5
//  0x00000500 | // Divide by 6
//  0x00000600 | // Divide by 10
//  0x00000700 | // Divide by 12
    // 00000070:FPLLMUL:PLL Multiplier bits
//  0x00000000 | // Multiply by 15
//  0x00000010 | // Multiply by 16
//  0x00000020 | // Multiply by 17
//  0x00000030 | // Multiply by 18
//  0x00000040 | // Multiply by 19
    0x00000050 | // Multiply by 20 // (8MHz crystal/2)/2=2MHz) x 20 = 40MHz
//  0x00000060 | // Multiply by 21
//  0x00000070 | // Multiply by 24
    // 00000007:FPLLIDIV:PLL Input Divider bits
//  0x00000000 | // Divide by 1
    0x00000001 | // Divide by 2 // 8MHz -> 4MHz
//  0x00000002 | // Divide by 3
//  0x00000003 | // Divide by 4
//  0x00000004 | // Divide by 5
//  0x00000005 | // Divide by 6
//  0x00000006 | // Divide by 10
//  0x00000007 | // Divide by 12
    0,

    // DEVCFG1 at .0BF8 - oscillators and clocks
    0xFC200858 | // all unused bits remain set to '1'
    // 03000000:FWDTWINSZ:Watchdog Timer Window Size bits
//  0x00000000 | // 75%
//  0x01000000 | // 50%
//  0x02000000 | // 37.5%
    0x03000000 | // 25%
    // 00800000:FWDTEN:Watchdog Timer Enable bit
    0x00000000 | // Disabled -- controlled by software
//  0x00800000 | // Enabled
    // 00400000:WINDIS:Watchdog Timer Window Enable bit
//  0x00000000 | // Window mode
    0x00400000 | // non-Window mode
    // 001F0000:WDTPS:Watchdog Timer Postscale Select bits
//  0x00000000 | // 1:1
//  0x00010000 | // 1:2
//  0x00020000 | // 1:4
//  0x00030000 | // 1:8
//  0x00040000 | // 1:16
//  0x00050000 | // 1:32
//  0x00060000 | // 1:64
//  0x00070000 | // 1:128
//  0x00080000 | // 1:256
//  0x00090000 | // 1:512
//  0x000A0000 | // 1:1024
//  0x000B0000 | // 1:2048
//  0x000C0000 | // 1:4096
//  0x000D0000 | // 1:8,192
//  0x000E0000 | // 1:16,384
//  0x000F0000 | // 1:32,768
//  0x00100000 | // 1:65,536
//  0x00110000 | // 1:131,072
//  0x00120000 | // 1:262,144
//  0x00130000 | // 1:524,288
    0x00140000 | // 1:1,048,576
    // 0000C000:FCKSM:Clock Switching and Monitor Selection bits
//  0x00000000 | // Clock Switching Enabled, Clock Monitoring Enabled
//  0x00004000 | // Clock Switching Enabled, Clock Monitoring Disabled
    0x0000C000 | // Clock Switching Disabled, Clock Monitoring Disabled
    // 00003000:FPBDIV:Bootup PBCLK divider
    0x00000000 | // Divide by 1 PB_Clk = SYS_Clk/1
//  0x00001000 | // Divide by 2
//  0x00002000 | // Divide by 4
//  0x00003000 | // Divide by 8
    // 00000400:OSCIOFNC:CLKO output Enable bit
//  0x00000000 | // Enabled
    0x00000400 | // Disabled
    // 00000300:POSCMOD:Primary Oscillator bits
//  0x00000000 | // EC oscillator (external clock)
    0x00000100 | // XT oscillator
//  0x00000200 | // HS oscillator
//  0x00000300 | // Disabled
    // 00000080:IESO:Internal External Switch Over bit
    0x00000000 | // Disabled
//  0x00000080 | // Enabled  (two speed startup)
    // 00000020:FSOSCEN:Secondary oscillator Enable bit
    0x00000000 | // Disabled
//  0x00000020 | // Enabled - 32.768 kHz
    // 00000007:FNOSC:Oscillator Selection bits
//  0x00000000 | // Fast RC oscillator
//  0x00000001 | // Fast RC oscillator w/ PLL
//  0x00000002 | // Primary oscillator (XT, HS, EC)
    0x00000003 | // Primary oscillator (XT, HS, EC) w/ PLL
//  0x00000004 | // Secondary oscillator
//  0x00000005 | // Low power RC oscillator
//  0x00000006 | // Fast RC oscillator with divide by 16
//  0x00000007 | // Fast RC oscillator with divide
    0,

    // DEVCFG0 at .0BFC - debug pins and write protect
    0x6EF003E0 | // all unused bits remain set to '1', except high bit required 0
    // 10000000:CP:Code Protect Enable bit
//  0x00000000 | // Enabled
    0x10000000 | // Disabled (writable)
    // 01000000:BWP:Boot Flash Write Protect bit
//  0x00000000 | // Enabled
    0x01000000 | // Disabled (writable)
    // 000FFC00:PWP:Program Flash Write Protect bit
    0x000FFC00 | // Disabled (all sectors writable; subsets can be protected...)
    // 00000018:ICESEL:ICE/ICD Communication Channel Select
    0x00000000 | // Reserved
//  0x00000008 | // ICE pins are shared with PGC3, PGD3
//  0x00000010 | // ICE pins are shared with PGC2, PGD2
    0x00000018 | // ICE pins are shared with PGC1, PGD1
    // 00000004:JTAGEN:JTAG Enable bit
    0x00000000 | // Disabled
//  0x00000004 | // Enabled
    // 00000003:DEBUG:Background Debugger Enable bit
//  0x00000002 | // Enabled - Background debug events branch to 0xbfc0480
//               //         - Do NOT enable this except for debug tools that
//               //           specifically require it, otherwise your device
//               //           will not come out of reset properly.
    0x00000003 | // Disabled
    0

};

