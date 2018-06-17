___total_ram = 256K;

MEMORY
{
  FLASH (rx) : ORIGIN = 0x01000800, LENGTH = 1022K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
  /* used in the presence of a bootloader */
  FLASH_APP (rx) : ORIGIN = 0x0100c000, LENGTH = 464K
  /* used for the debug signature when flashing an app binary directly */
  FLASH_HDR (rx) : ORIGIN = 0x01000000, LENGTH = 2K
}

REGION_ALIAS("REGION_RAM", RAM);

/* TODO: the bootloader specific defines need to be revised once we know how the bootloader will work at all. */
__application_code_start = ORIGIN(RAM);
__application_code_end = ORIGIN(RAM) + LENGTH(RAM);
__app_header_address = ORIGIN(RAM) + 0x270;
__bootloader_magic_ptr = 0x4402FC24;
/* This pointer will be written to the SP register at reset. */
PROVIDE(__stack = ORIGIN(RAM) + LENGTH(RAM) - 4);
__cs3_heap_end = ORIGIN(RAM) + LENGTH(RAM);
