MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 504K
  BOOTLOADER (rx) : ORIGIN = 0x0007E000, LENGTH = 8K
  RAM  (rwx) : ORIGIN = 0x10000000, LENGTH = 32K
  RAM2 (rwx) : ORIGIN = 0x2007c000, LENGTH = 32K /* heap and stack */
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
