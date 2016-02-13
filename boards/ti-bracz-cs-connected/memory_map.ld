MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  AUTOMATA (rw) : ORIGIN = 0x40000, LENGTH = 256K
  EEPROMEMU (r): ORIGIN = 0xf0000, LENGTH = 48K
  BOOTLOADER (rx) : ORIGIN = 0x000fc000, LENGTH = 16K
  RAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__bootloader_end = ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER);
__app_header_offset = 0x20C;

__automata_start = ORIGIN(AUTOMATA);
__automata_end = ORIGIN(AUTOMATA) + LENGTH(AUTOMATA);
__flash_page_size = 16K;
