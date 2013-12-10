OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(ResetISR)
SEARCH_DIR(.)

GROUP(
 libgcc.a
 libc_s.a
 libstdc++_s.a
 libm.a
 crti.o
 crtn.o
 crtbegin.o
 crtend.o
)

MEMORY
{
  /* Define each memory region */
  MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x18000  /*32k */
  /*MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x10000 /* 64k TEMPORARY */
  /* Top 32 bytes is used by the flash writing firmware. 0x50..0xb8 by the ROM CAN drivers. 0 to 0x50 is used by the */

  RamLoc8 (rwx) : ORIGIN = 0x10000100, LENGTH = 0x1ee0 /* 7k */

}
  /* Define a symbol for the top of each memory region */
  __top_MFlash32 = ORIGIN(MFlash32) + LENGTH(MFlash32);
  __top_RamLoc8 = ORIGIN(RamLoc8) + LENGTH(RamLoc8);

  /* The cortex-m0 FreeRTOS is not using the original stack once the scheduler
  has started, so we have no reason to reserve the top of RAM. */
  __cs3_heap_end = __top_RamLoc8;

  __start_ram = ORIGIN(RamLoc8);
  __end_ram = __top_RamLoc8;

SECTIONS
{

    /* MAIN TEXT SECTION */    
    .text : ALIGN(4)
    {
        FILL(0xff)
        KEEP(*(.isr_vector))
        
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
        

        *(.after_vectors*)
        
        *(.text*)
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
    } > MFlash32

    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this. 
     */
    .ARM.extab : ALIGN(4)
    {
    	*(.ARM.extab* .gnu.linkonce.armextab.*)
    } > MFlash32
    __exidx_start = .;
    
    .ARM.exidx : ALIGN(4)
    {
    	*(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } > MFlash32
    __exidx_end = .;
    
    _etext = .;
        
    
    /* MAIN DATA SECTION */
    

    .uninit_RESERVED : ALIGN(4)
    {
        KEEP(*(.bss.$RESERVED*))
        . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > RamLoc8

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
    } > RamLoc8 AT>MFlash32


    /* MAIN BSS SECTION */
    .bss : ALIGN(4)
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(end = .);
    } > RamLoc8
        
    
    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        *(.noinit*) 
         . = ALIGN(4) ;
        _end_noinit = .;
    } > RamLoc8
    
    PROVIDE(_pvHeapStart = .);
    PROVIDE(__cs3_heap_start = .);
    /* This pointer will be written to the SP register at reset. */
    PROVIDE(_vStackTop = __top_RamLoc8);

    PROVIDE(__impure_data_size = __impure_data_end - __impure_data_start);

}

