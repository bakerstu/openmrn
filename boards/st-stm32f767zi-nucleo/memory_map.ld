MEMORY
{
  FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  EEPROMEMU  (r)   : ORIGIN = 0x08180000, LENGTH = 512K
  BOOTLOADER (rx)  : ORIGIN = 0x0807A000, LENGTH = 512K
  RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__stm32_eeprom_flash_sector_start = 10;
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
