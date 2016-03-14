OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(reset_handler)
SEARCH_DIR(.)
GROUP(-lgcc -lc)

/* include device specific memory map */
INCLUDE memory_map.ld

__top_RAM = ORIGIN(RAM) + LENGTH(RAM);

__cs3_heap_end = __top_RAM;

__start_ram = ORIGIN(RAM);
__end_ram = __top_RAM;

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.interrupt_vector))

        /* Makes place for the app header to be filled in by the bootloader
        checksum program. */
        /*. = __app_header_offset;*/
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
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

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

        *(.text*)
        *(.rodata*)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        __text_section_guard = .;
        LONG(0);
    } > RAM

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH
    
    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } > RAM

    _etext = .;

    __init_data = .;

    .data : AT(__init_data)
    {
        _data = .;
        *(.data*)
        . = ALIGN (8);
        _edata = .;
    } > RAM

    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
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
/*
    PROVIDE(__impure_data_size = __impure_data_end - __impure_data_start);
*/
}
