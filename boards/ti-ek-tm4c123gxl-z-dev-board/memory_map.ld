MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 240K
  EEPROMEMU (r): ORIGIN = 0x3C000, LENGTH = 8K
  BOOTLOADER (rx) : ORIGIN = 0x0003E000, LENGTH = 8K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__bootloader_end = ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
