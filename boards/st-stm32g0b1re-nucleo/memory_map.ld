MEMORY
{
  FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 472K
  EEPROMEMU  (r)   : ORIGIN = 0x08076000, LENGTH = 32K
  BOOTLOADER (rx)  : ORIGIN = 0x0807E000, LENGTH = 8K
  RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
