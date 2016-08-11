MEMORY
{
  FLASH (rx)      : ORIGIN = 0x00000000, LENGTH = 416K
  EEPROMEMU (r)   : ORIGIN = 0x00068000, LENGTH = 64K
  BOOTLOADER (rx) : ORIGIN = 0x00078000, LENGTH = 32K
  RAM  (rwx)      : ORIGIN = 0x10000200, LENGTH = 32K - 32 - 0x200 /* subtract IAP scratchpad */
  RAM2 (rwx)      : ORIGIN = 0x2007c000, LENGTH = 32K /* heap and stack */
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__bootloader_magic_ptr = ORIGIN(RAM);
