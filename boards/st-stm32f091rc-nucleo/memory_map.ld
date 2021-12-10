___ram_for_bootloader_api = 8;

MEMORY
{
  FLASH      (rx)  : ORIGIN = 0x08000000, LENGTH = 240K
  EEPROMEMU  (r)   : ORIGIN = 0x0803C000, LENGTH = 8K
  BOOTLOADER (rx)  : ORIGIN = 0x0803E000, LENGTH = 8K
  RAM        (rwx) : ORIGIN = 0x20000000,
                     LENGTH = 32K - ___ram_for_bootloader_api
  BOOTLOADERAPI (rw)  : ORIGIN = 0x20000000 + 32K - ___ram_for_bootloader_api,
                        LENGTH = ___ram_for_bootloader_api
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);
__app_header_offset = 0x270;
__app_header_address = ORIGIN(FLASH) + __app_header_offset;
__bootloader_magic_ptr = ORIGIN(RAM);
__application_node_id = ORIGIN(BOOTLOADERAPI);
