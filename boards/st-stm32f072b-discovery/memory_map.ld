MEMORY
{
  FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 116K
  EEPROMEMU  (r)   : ORIGIN = 0x0801D000, LENGTH = 4K
  BOOTLOADER (rx)  : ORIGIN = 0x0801E000, LENGTH = 8K
  RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 16K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
