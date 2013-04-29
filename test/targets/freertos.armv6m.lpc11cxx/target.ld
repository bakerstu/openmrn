OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
ENTRY(ResetISR)
SEARCH_DIR(.)

GROUP(
 libgcc.a
 libc.a
 libstdc++.a
 libm.a
 crti.o
 crtn.o
 crtbegin.o
 crtend.o
)

MEMORY
{
  /* Define each memory region */
  MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x8000  /*32k */
  /*MFlash32 (rx) : ORIGIN = 0x0, LENGTH = 0x10000 /* 64k TEMPORARY */
  RamLoc8 (rwx) : ORIGIN = 0x10000100, LENGTH = 0x1ee0 /* 7k */

}
  /* Define a symbol for the top of each memory region */
  __top_MFlash32 = ORIGIN(MFlash32) + LENGTH(MFlash32);
  __top_RamLoc8 = ORIGIN(RamLoc8) + LENGTH(RamLoc8);

  Stack_Size = 512;
  __cs3_heap_end = __top_RamLoc8 - Stack_Size;

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
        *(.rodata .rodata.*)
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

    .data : ALIGN(4)
    {
        FILL(0xff)
        _data = .;
        *(vtable)
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
    PROVIDE(_vStackTop = __top_RamLoc8 - 32);
}
