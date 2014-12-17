MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 120K
  BOOTLOADER (rx) : ORIGIN = 0x0001E000, LENGTH = 8K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
