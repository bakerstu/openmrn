OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(reset_handler)
SEARCH_DIR(.)
GROUP(-lgcc -lc)

/* include device specific memory map */
INCLUDE memory_map.ld

__top_RAM = ORIGIN(RAM) + LENGTH(RAM);

__cs3_heap_end = __top_RAM - 1024;

__start_ram = ORIGIN(RAM);
__end_ram = __top_RAM;

SECTIONS
{
    /* MAIN TEXT SECTION */    
    .text : ALIGN(4)
    {
        FILL(0xff)
        KEEP(*(.interrupt_vector))

        /* Makes place for the app header to be filled in by the bootloader
        checksum program. */
        . = __app_header_offset;
        QUAD(0); /* App header: checksum for vector table. */
        QUAD(0);
        LONG(0); /* App header: data size */
        QUAD(0); /* App header: checksum for payload. */
        QUAD(0);

        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data)) ;
        LONG(  SIZEOF(.data));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(__cs3_heap_end);
        LONG(__top_RAM - __cs3_heap_end - 64);
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

        *(SORT(.text*))
        *(.rodata)
        *(SORT(.rodata.*))
        . = ALIGN(4);

        /* C++ constructors etc */
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

        KEEP(*(.fini));

        . = ALIGN(4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))
        /* End C++ */

        __text_section_guard = .;
        LONG( 0 );
    } > FLASH

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    __exidx_start = .;

    .ARM.exidx : ALIGN(4)
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > FLASH
    __exidx_end = .;

    _etext = .;


    /* MAIN DATA SECTION */


    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RAM

    /* It seems that in order for LPCXpresso to properly flash the data
    section, an alignment of 256 bytes is necessary. Otherwise the separate
    flashing of the data section will corrupt the end of the text section. */
    .data : ALIGN(256)
    {
        FILL(0xff)
        _data = .;
        *(vtable)
        __impure_data_start = .;
        *(.data.impure_data)
        __impure_data_end = .;
        *(.data*)

        /* this magic is needed for the device tables of openMRN */
        . = ALIGN (8);
        KEEP(*( SORT (.device.table.*))) ;
        . = ALIGN (4);

        _edata = .;
    } > RAM AT>FLASH


    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RAM

    
    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RAM

    PROVIDE(_pvHeapStart = .);
    PROVIDE(__cs3_heap_start = .);
    /* This pointer will be written to the SP register at reset. */
    PROVIDE(__stack = __top_RAM);

    __impure_data_size = (__impure_data_end - __impure_data_start);
}
