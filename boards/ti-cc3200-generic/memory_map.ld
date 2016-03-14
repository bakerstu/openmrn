MEMORY
{
  RAM  (rwx) : ORIGIN = 0x20004000, LENGTH = 168K
  BOOTLOADER (rx) : ORIGIN = 0x2002E000, LENGTH = 8K
}

__bootloader_start = ORIGIN(BOOTLOADER);
__bootloader_end = ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER);
/*__app_header_offset = 0x270;*/
__bootloader_magic_ptr = ORIGIN(RAM);
