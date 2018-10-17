/* PIC32MX linker script for CodeSourcery GCC toolchain
**
** Links a complete application to run from PIC32MX flash, including
** initialization from power-on reset, interrupt vectors, etc.
**
** History:
**   20181017 bracz: Refactor for external included memory map
**   20090926 DRNadler: Per CodeSourcery, ENTRY is __cs3_reset_PIC32MX
**   20090830 DRNadler: Minimal C++ demonstration version (KSEG1)
**   20090713 DRNadler: Reset reason stored in RAM0
**   20090511 DRNadler: Original coding
**
** For reference, see:
** - CodeSourcery G++ 4.3-80 linker script for malta-24kc
** - Microchip "elf32pic32mx.x" generic PIC32 link script
** - Microchip 32MX440F256H processor-specific "procdef.ld" definitions
*/


/*
 * Copyright (c) 2009 Dave Nadler
 * Copyright (c) 2007, 2008 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */
OUTPUT_ARCH(mips)
ENTRY(__cs3_reset_PIC32MX) /* for debugger support, use actual HW-level entry and not _start */
SEARCH_DIR(.)
/* Force repeated search of CS3 and GCC libraries */
GROUP(-lcs3hal -lgcc -lc -lcs3 -lcs3unhosted)
/**/

INCLUDE memory_map.ld

/*
** NOTE: Device programmers require PIC32MX "physical" addresses.
** PIC32MX "physical" addresses ::= "virtual" addresses & 0x1FFFffff
** LD's >AT directive is incapable of performing this mapping,
** as it does not respect alignment/padding performed in the virtual space.
** Therefore, this address mask operation must be performed by:
** - address masking by the device programmer's software, or
** - post-processing of ELF or HEX file prior device programming
*/

MEMORY
{
  /* tiny part of RAM reserved for startup code */
  ram0 (!r!x) : ORIGIN = 0xA0000000, LENGTH = 16
  ram  (!r!x) : ORIGIN = 0xA0000010, LENGTH = ___user_ram_size - 16
  /*
  ** Memory map here is based on executing in KSEG1 mode.
  ** PIC32MX flash is broken into user flash and boot flash.
  ** Boot flash contains the configuration words required for reset.
  ** The following are KSEG1 logical addresses (??? should change to KSEG0 to enable caches).
  */
  flash_user (rx) : ORIGIN = 0x9D000000, LENGTH = ___user_flash_size
  
  /*
  ** When PIC32MX comes out of reset, it branches to the start of the boot flash
  ** The last (4kb) page in Boot Flash Memory contains the DEBUG Page, which is reserved for 
  ** use by (some) debugger tool while debugging (by convention, not by hardware).
  */
  flash_boot (rx) : ORIGIN = 0x9FC00000, LENGTH = ___boot_flash_size - 16
  flash_cfg  (r!x): ORIGIN = 0x9FC00000 + ___boot_flash_size - 16, LENGTH = 16
}

/* This include defines region aliases to select whether we are building with bootloader or without. */
INCLUDE region.ld

/*
** Interrupt vector spacing 1 (ie 0x20 or 32 bytes/vector).
*/
_vector_spacing = 0x00000001;

/* These force the linker to search for particular symbols from
 * the start of the link process and thus ensure the user's
 * overrides are picked up
 */
EXTERN(__cs3_reset_PIC32MX)
/* NOT NEEDED -- we have our own entry routine, not using cs3's
EXTERN(_start)
EXTERN(__cs3_start_c main __cs3_stack __cs3_heap_end) */

/*
 * Provide for a minimum stack and heap size
 * - _min_stack_size - represents the minimum space that must be made
 *                     available for the stack.  Can be overridden from
 *                     the command line using the linker's --defsym option.
 * - _min_heap_size  - represents the minimum space that must be made
 *                     available for the heap.  Can be overridden from
 *                     the command line using the linker's --defsym option.
 */
EXTERN (_min_stack_size _min_heap_size)
PROVIDE(__cs3_heap_start = _end );
PROVIDE(__cs3_heap_end = __cs3_region_start_ram + __cs3_region_size_ram - _min_stack_size);
PROVIDE(__cs3_stack = __cs3_region_start_ram + __cs3_region_size_ram); /* stack grows down from end of RAM */

HEAP_SPACE_AVAILABLE = (__cs3_heap_end - __cs3_heap_start);
ASSERT( HEAP_SPACE_AVAILABLE >= _min_heap_size , "Not enough heap space");


PROVIDE(__cs3_region_num = (__cs3_regions_end - __cs3_regions) / 20);


SECTIONS
{
  /*
  ** First thing in  flash area must be 'reset' code, which must be
  ** short enough it doesn't land in bootstrap exception address (unless
  ** bootstrap exception is unused).
  */
  .boot_flash_text :
  {
    PROVIDE(__cs3_reset_PIC32MX = _start);
    __cs3_reset = __cs3_reset_PIC32MX;
    *(.cs3.reset)

  } >REGION_RESET

  /* ============================  Interrupt Branch Vectors  =========================== */
  .gen_exception _ebase_address + 0x180 :
  {
    KEEP(*(.gen_handler))
  } >REGION_INT


  .vector_0 _ebase_address + 0x200 :
  {
    KEEP(*(.vector_0))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_0) <= (_vector_spacing << 5), "function at exception vector 0 too large")
  .vector_1 _ebase_address + 0x200 + (_vector_spacing << 5) * 1 :
  {
    KEEP(*(.vector_1))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_1) <= (_vector_spacing << 5), "function at exception vector 1 too large")
  .vector_2 _ebase_address + 0x200 + (_vector_spacing << 5) * 2 :
  {
    KEEP(*(.vector_2))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_2) <= (_vector_spacing << 5), "function at exception vector 2 too large")
  .vector_3 _ebase_address + 0x200 + (_vector_spacing << 5) * 3 :
  {
    KEEP(*(.vector_3))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_3) <= (_vector_spacing << 5), "function at exception vector 3 too large")
  .vector_4 _ebase_address + 0x200 + (_vector_spacing << 5) * 4 :
  {
    KEEP(*(.vector_4))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_4) <= (_vector_spacing << 5), "function at exception vector 4 too large")
  .vector_5 _ebase_address + 0x200 + (_vector_spacing << 5) * 5 :
  {
    KEEP(*(.vector_5))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_5) <= (_vector_spacing << 5), "function at exception vector 5 too large")
  .vector_6 _ebase_address + 0x200 + (_vector_spacing << 5) * 6 :
  {
    KEEP(*(.vector_6))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_6) <= (_vector_spacing << 5), "function at exception vector 6 too large")
  .vector_7 _ebase_address + 0x200 + (_vector_spacing << 5) * 7 :
  {
    KEEP(*(.vector_7))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_7) <= (_vector_spacing << 5), "function at exception vector 7 too large")
  .vector_8 _ebase_address + 0x200 + (_vector_spacing << 5) * 8 :
  {
    KEEP(*(.vector_8))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_8) <= (_vector_spacing << 5), "function at exception vector 8 too large")
  .vector_9 _ebase_address + 0x200 + (_vector_spacing << 5) * 9 :
  {
    KEEP(*(.vector_9))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_9) <= (_vector_spacing << 5), "function at exception vector 9 too large")
  .vector_10 _ebase_address + 0x200 + (_vector_spacing << 5) * 10 :
  {
    KEEP(*(.vector_10))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_10) <= (_vector_spacing << 5), "function at exception vector 10 too large")
  .vector_11 _ebase_address + 0x200 + (_vector_spacing << 5) * 11 :
  {
    KEEP(*(.vector_11))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_11) <= (_vector_spacing << 5), "function at exception vector 11 too large")
  .vector_12 _ebase_address + 0x200 + (_vector_spacing << 5) * 12 :
  {
    KEEP(*(.vector_12))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_12) <= (_vector_spacing << 5), "function at exception vector 12 too large")
  .vector_13 _ebase_address + 0x200 + (_vector_spacing << 5) * 13 :
  {
    KEEP(*(.vector_13))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_13) <= (_vector_spacing << 5), "function at exception vector 13 too large")
  .vector_14 _ebase_address + 0x200 + (_vector_spacing << 5) * 14 :
  {
    KEEP(*(.vector_14))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_14) <= (_vector_spacing << 5), "function at exception vector 14 too large")
  .vector_15 _ebase_address + 0x200 + (_vector_spacing << 5) * 15 :
  {
    KEEP(*(.vector_15))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_15) <= (_vector_spacing << 5), "function at exception vector 15 too large")
  .vector_16 _ebase_address + 0x200 + (_vector_spacing << 5) * 16 :
  {
    KEEP(*(.vector_16))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_16) <= (_vector_spacing << 5), "function at exception vector 16 too large")
  .vector_17 _ebase_address + 0x200 + (_vector_spacing << 5) * 17 :
  {
    KEEP(*(.vector_17))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_17) <= (_vector_spacing << 5), "function at exception vector 17 too large")
  .vector_18 _ebase_address + 0x200 + (_vector_spacing << 5) * 18 :
  {
    KEEP(*(.vector_18))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_18) <= (_vector_spacing << 5), "function at exception vector 18 too large")
  .vector_19 _ebase_address + 0x200 + (_vector_spacing << 5) * 19 :
  {
    KEEP(*(.vector_19))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_19) <= (_vector_spacing << 5), "function at exception vector 19 too large")
  .vector_20 _ebase_address + 0x200 + (_vector_spacing << 5) * 20 :
  {
    KEEP(*(.vector_20))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_20) <= (_vector_spacing << 5), "function at exception vector 20 too large")
  .vector_21 _ebase_address + 0x200 + (_vector_spacing << 5) * 21 :
  {
    KEEP(*(.vector_21))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_21) <= (_vector_spacing << 5), "function at exception vector 21 too large")
  .vector_22 _ebase_address + 0x200 + (_vector_spacing << 5) * 22 :
  {
    KEEP(*(.vector_22))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_22) <= (_vector_spacing << 5), "function at exception vector 22 too large")
  .vector_23 _ebase_address + 0x200 + (_vector_spacing << 5) * 23 :
  {
    KEEP(*(.vector_23))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_23) <= (_vector_spacing << 5), "function at exception vector 23 too large")
  .vector_24 _ebase_address + 0x200 + (_vector_spacing << 5) * 24 :
  {
    KEEP(*(.vector_24))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_24) <= (_vector_spacing << 5), "function at exception vector 24 too large")
  .vector_25 _ebase_address + 0x200 + (_vector_spacing << 5) * 25 :
  {
    KEEP(*(.vector_25))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_25) <= (_vector_spacing << 5), "function at exception vector 25 too large")
  .vector_26 _ebase_address + 0x200 + (_vector_spacing << 5) * 26 :
  {
    KEEP(*(.vector_26))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_26) <= (_vector_spacing << 5), "function at exception vector 26 too large")
  .vector_27 _ebase_address + 0x200 + (_vector_spacing << 5) * 27 :
  {
    KEEP(*(.vector_27))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_27) <= (_vector_spacing << 5), "function at exception vector 27 too large")
  .vector_28 _ebase_address + 0x200 + (_vector_spacing << 5) * 28 :
  {
    KEEP(*(.vector_28))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_28) <= (_vector_spacing << 5), "function at exception vector 28 too large")
  .vector_29 _ebase_address + 0x200 + (_vector_spacing << 5) * 29 :
  {
    KEEP(*(.vector_29))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_29) <= (_vector_spacing << 5), "function at exception vector 29 too large")
  .vector_30 _ebase_address + 0x200 + (_vector_spacing << 5) * 30 :
  {
    KEEP(*(.vector_30))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_30) <= (_vector_spacing << 5), "function at exception vector 30 too large")
  .vector_31 _ebase_address + 0x200 + (_vector_spacing << 5) * 31 :
  {
    KEEP(*(.vector_31))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_31) <= (_vector_spacing << 5), "function at exception vector 31 too large")
  .vector_32 _ebase_address + 0x200 + (_vector_spacing << 5) * 32 :
  {
    KEEP(*(.vector_32))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_32) <= (_vector_spacing << 5), "function at exception vector 32 too large")
  .vector_33 _ebase_address + 0x200 + (_vector_spacing << 5) * 33 :
  {
    KEEP(*(.vector_33))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_33) <= (_vector_spacing << 5), "function at exception vector 33 too large")
  .vector_34 _ebase_address + 0x200 + (_vector_spacing << 5) * 34 :
  {
    KEEP(*(.vector_34))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_34) <= (_vector_spacing << 5), "function at exception vector 34 too large")
  .vector_35 _ebase_address + 0x200 + (_vector_spacing << 5) * 35 :
  {
    KEEP(*(.vector_35))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_35) <= (_vector_spacing << 5), "function at exception vector 35 too large")
  .vector_36 _ebase_address + 0x200 + (_vector_spacing << 5) * 36 :
  {
    KEEP(*(.vector_36))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_36) <= (_vector_spacing << 5), "function at exception vector 36 too large")
  .vector_37 _ebase_address + 0x200 + (_vector_spacing << 5) * 37 :
  {
    KEEP(*(.vector_37))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_37) <= (_vector_spacing << 5), "function at exception vector 37 too large")
  .vector_38 _ebase_address + 0x200 + (_vector_spacing << 5) * 38 :
  {
    KEEP(*(.vector_38))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_38) <= (_vector_spacing << 5), "function at exception vector 38 too large")
  .vector_39 _ebase_address + 0x200 + (_vector_spacing << 5) * 39 :
  {
    KEEP(*(.vector_39))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_39) <= (_vector_spacing << 5), "function at exception vector 39 too large")
  .vector_40 _ebase_address + 0x200 + (_vector_spacing << 5) * 40 :
  {
    KEEP(*(.vector_40))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_40) <= (_vector_spacing << 5), "function at exception vector 40 too large")
  .vector_41 _ebase_address + 0x200 + (_vector_spacing << 5) * 41 :
  {
    KEEP(*(.vector_41))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_41) <= (_vector_spacing << 5), "function at exception vector 41 too large")
  .vector_42 _ebase_address + 0x200 + (_vector_spacing << 5) * 42 :
  {
    KEEP(*(.vector_42))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_42) <= (_vector_spacing << 5), "function at exception vector 42 too large")
  .vector_43 _ebase_address + 0x200 + (_vector_spacing << 5) * 43 :
  {
    KEEP(*(.vector_43))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_43) <= (_vector_spacing << 5), "function at exception vector 43 too large")
  .vector_44 _ebase_address + 0x200 + (_vector_spacing << 5) * 44 :
  {
    KEEP(*(.vector_44))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_44) <= (_vector_spacing << 5), "function at exception vector 44 too large")
  .vector_45 _ebase_address + 0x200 + (_vector_spacing << 5) * 45 :
  {
    KEEP(*(.vector_45))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_45) <= (_vector_spacing << 5), "function at exception vector 45 too large")
  .vector_46 _ebase_address + 0x200 + (_vector_spacing << 5) * 46 :
  {
    KEEP(*(.vector_46))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_46) <= (_vector_spacing << 5), "function at exception vector 46 too large")
  .vector_47 _ebase_address + 0x200 + (_vector_spacing << 5) * 47 :
  {
    KEEP(*(.vector_47))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_47) <= (_vector_spacing << 5), "function at exception vector 47 too large")
  .vector_48 _ebase_address + 0x200 + (_vector_spacing << 5) * 48 :
  {
    KEEP(*(.vector_48))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_48) <= (_vector_spacing << 5), "function at exception vector 48 too large")
  .vector_49 _ebase_address + 0x200 + (_vector_spacing << 5) * 49 :
  {
    KEEP(*(.vector_49))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_49) <= (_vector_spacing << 5), "function at exception vector 49 too large")
  .vector_50 _ebase_address + 0x200 + (_vector_spacing << 5) * 50 :
  {
    KEEP(*(.vector_50))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_50) <= (_vector_spacing << 5), "function at exception vector 50 too large")
  .vector_51 _ebase_address + 0x200 + (_vector_spacing << 5) * 51 :
  {
    KEEP(*(.vector_51))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_51) <= (_vector_spacing << 5), "function at exception vector 51 too large")
  .vector_52 _ebase_address + 0x200 + (_vector_spacing << 5) * 52 :
  {
    KEEP(*(.vector_52))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_52) <= (_vector_spacing << 5), "function at exception vector 52 too large")
  .vector_53 _ebase_address + 0x200 + (_vector_spacing << 5) * 53 :
  {
    KEEP(*(.vector_53))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_53) <= (_vector_spacing << 5), "function at exception vector 53 too large")
  .vector_54 _ebase_address + 0x200 + (_vector_spacing << 5) * 54 :
  {
    KEEP(*(.vector_54))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_54) <= (_vector_spacing << 5), "function at exception vector 54 too large")
  .vector_55 _ebase_address + 0x200 + (_vector_spacing << 5) * 55 :
  {
    KEEP(*(.vector_55))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_55) <= (_vector_spacing << 5), "function at exception vector 55 too large")
  .vector_56 _ebase_address + 0x200 + (_vector_spacing << 5) * 56 :
  {
    KEEP(*(.vector_56))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_56) <= (_vector_spacing << 5), "function at exception vector 56 too large")
  .vector_57 _ebase_address + 0x200 + (_vector_spacing << 5) * 57 :
  {
    KEEP(*(.vector_57))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_57) <= (_vector_spacing << 5), "function at exception vector 57 too large")
  .vector_58 _ebase_address + 0x200 + (_vector_spacing << 5) * 58 :
  {
    KEEP(*(.vector_58))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_58) <= (_vector_spacing << 5), "function at exception vector 58 too large")
  .vector_59 _ebase_address + 0x200 + (_vector_spacing << 5) * 59 :
  {
    KEEP(*(.vector_59))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_59) <= (_vector_spacing << 5), "function at exception vector 59 too large")
  .vector_60 _ebase_address + 0x200 + (_vector_spacing << 5) * 60 :
  {
    KEEP(*(.vector_60))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_60) <= (_vector_spacing << 5), "function at exception vector 60 too large")
  .vector_61 _ebase_address + 0x200 + (_vector_spacing << 5) * 61 :
  {
    KEEP(*(.vector_61))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_61) <= (_vector_spacing << 5), "function at exception vector 61 too large")
  .vector_62 _ebase_address + 0x200 + (_vector_spacing << 5) * 62 :
  {
    KEEP(*(.vector_62))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_62) <= (_vector_spacing << 5), "function at exception vector 62 too large")
  .vector_63 _ebase_address + 0x200 + (_vector_spacing << 5) * 63 :
  {
    KEEP(*(.vector_63))
  } >REGION_INT
  ASSERT (_vector_spacing == 0 || SIZEOF(.vector_63) <= (_vector_spacing << 5), "function at exception vector 63 too large")
  /* ============================  End Interrupt Branch Vectors  =========================== */


  /*
  ** "Normal" code in user flash
  */
  .text _textoffs :
  {
    CREATE_OBJECT_SYMBOLS /* create symbols for each input file - why ??? */

    __cs3_region_start_rom = .;
    *(.cs3.region-head.rom)
    ASSERT (. == __cs3_region_start_rom, ".cs3.region-head.rom not permitted");

    /* "normal" code follows... */
    *(.text .text.* .gnu.linkonce.t.*)

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.jcr))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .jcr))
    KEEP (*crtend.o(.jcr))

    . = ALIGN(0x4);
    *(.gcc_except_table .gcc_except_table.*)
    *(.gnu_extab .gnu_extab.*)
  } >REGION_PGM
  
  .eh_frame_hdr : ALIGN (4)
  {
    KEEP (*(.eh_frame_hdr))
    *(.eh_frame_entry .eh_frame_entry.*)
  } >REGION_PGM
  
  .eh_frame : ALIGN (4)
  {
    KEEP (*(.eh_frame .eh_frame.*))
  } >REGION_PGM
  
  /* MDI semihosting uses a pointer in this section.  */
  .sdeosabi : ALIGN (4)
  {
    *(.sdeosabi)
  } >ram
  
  .rodata : ALIGN (4)
  {
    *(.rodata .rodata.* .gnu.linkonce.r.*)

    . = ALIGN(4);
    KEEP(*(.init))

    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP (*(.preinit_array))
    __preinit_array_end = .;

    . = ALIGN(4);
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;

    . = ALIGN(4);
    KEEP(*(.fini))

    . = ALIGN(4);
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))

    . = ALIGN(0x4);
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    *(.lit)

    . = ALIGN(4);
    __cs3_regions = .;
    LONG (0)
    LONG (__cs3_region_init_ram)
    LONG (__cs3_region_start_ram)
    LONG (__cs3_region_init_size_ram)
    LONG (__cs3_region_zero_size_ram)
    __cs3_regions_end = .;

    . = ALIGN (8);
    /* Redundant, removed: . = ALIGN (8); */

    _etext = .; /* first double-word past end of text, and start address for RAM initialization data stored in ROM */
  }  >REGION_PGM
  
  __cs3_region_size_rom = LENGTH(flash_user); /* just total size of flash */

  /*
  ** Initialized data is linked for RAM, but placed in ROM using the AT> directive.
  ** Startup code copies it to RAM during initialization.
  ** WARNING: Must follow _etext !!
  */
  .data : ALIGN (8)
  {
    __cs3_region_start_ram = .;
    *(.got.plt) *(.got)
    *(.shdata)
    *(.data .data.* .gnu.linkonce.d.*)
    . = ALIGN(8);
    /*
    ** GP-relative addressing permits fast access via single instruction addressing,
    ** using a signed 16-bit offset (i.e. -32768 to 32767) from the gp register.
    ** Set GP to the middle of the "small data" area, then link in up to 64k
    ** of "small data"....
    */
    _gp = . + 0x8000; /* beginning of "small data" plus 32k */
    *(.lit8)
    *(.lit4)
    *(.sdata .sdata.* .gnu.linkonce.s.*)

    /* this magic is needed for the device tables of openMRN */
    . = ALIGN (8);
    KEEP(*( SORT (.device.table.*))) ;

    . = ALIGN (8);
    *(.ram)
    _edata = .;
  } >ram  AT>REGION_PGM

  /* Wrong, length just gives 256k size of flash: USER_FLASH_USED = LENGTH(PIC32MX_flash_user)+SIZEOF(.data); */
  
  /* DRN: The following is wrong; doesn't mark end of RAM copy...
  /* For diagnostic use only, mark end of initialized-data copy in ROM */
  /* .edata_copy_end :
  /* {
  /*   _edata_copy_end = .;
  /*   LONG(0) /* force LD to output this section (empty sections are discarded) */
  /* } >PIC32MX_flash_user
  */

  /* data written by startup code and NOT to be initialized by CS3 */
  .startup_data :
  {
    *(.reset_data)
  } >ram0

  /* Un-initialized data, zero'd by CS3 C-language initialization module */
  .bss : ALIGN (8)
  {
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.scommon)
    *(.shbss)
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    *(.ram.b)
    _end = .;
    __end = .;
  } >ram

  /* __cs3_region_end_ram is deprecated */
  /*__cs3_region_end_ram = __cs3_region_start_ram + LENGTH(ram);*/
  __cs3_region_size_ram = LENGTH(ram);
  /* WRONG: load address is PHYSICAL, not logical address:  __cs3_region_init_ram = LOADADDR (.data);  WRONG */
  /*  __cs3_region_init_ram = _etext; DRN updated this; seems correct...  bracz: not really. */
  __cs3_region_init_ram = LOADADDR(.data);
  __cs3_region_init_size_ram = _edata - ADDR (.data);
  __cs3_region_zero_size_ram = _end - _edata;

  /* Compute space available for stack+heap... */
  AVAILABLE_RAM_FOR_STACK_PLUS_HEAP = LENGTH(ram)-(SIZEOF(.data)+SIZEOF(.bss));


  


  .stab 0 (NOLOAD) : { *(.stab) }
  .stabstr 0 (NOLOAD) : { *(.stabstr) }
  /* DWARF debug sections.
   * Symbols in the DWARF debugging sections are relative to the beginning
   * of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
}
