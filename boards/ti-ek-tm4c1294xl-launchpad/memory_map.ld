MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 980K
  EEPROMEMU (r): ORIGIN = 0xf4000, LENGTH = 32K
  /* We could have less space for the bootloader, but since flash can only be
  erased at 16k boundaries, it does not help. */
  BOOTLOADER (rx) : ORIGIN = 0x000fc000, LENGTH = 16K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x20C;
