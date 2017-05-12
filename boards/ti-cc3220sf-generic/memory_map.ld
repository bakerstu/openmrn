___total_ram = 256K;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x01000000, LENGTH = 1024K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}


/* TODO: the bootloader specific defines need to be revised once we know how the bootloader will work at all. */
__application_code_start = ORIGIN(RAM);
__application_code_end = ORIGIN(RAM) + LENGTH(RAM);
__app_header_address = ORIGIN(RAM) + 0x270;
__bootloader_magic_ptr = ORIGIN(RAM) + ___total_ram - 4;
/* This pointer will be written to the SP register at reset. */
PROVIDE(__stack = ORIGIN(RAM) + LENGTH(RAM) - 4);
__cs3_heap_end = ORIGIN(RAM) + LENGTH(RAM);
