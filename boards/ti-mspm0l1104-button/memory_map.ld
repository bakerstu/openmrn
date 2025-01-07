___ram_for_bootloader_api = 8;
___hw_flash = 16K - 8;
___hw_ram = 2K;


MEMORY
{
  FLASH      (rx)   : ORIGIN = 0x00000000, LENGTH = ___hw_flash
/*  EEPROMEMU  (r)   : ORIGIN = 0x0803C000, LENGTH = 8K*/
/*  BOOTLOADER (rx)  : ORIGIN = 0x0803E000, LENGTH = 8K*/
  RAM        (rwx)  : ORIGIN = 0x20000000,
                     LENGTH = ___hw_ram - ___ram_for_bootloader_api
  BOOTLOADERAPI (rw): ORIGIN = 0x20000000 + ___hw_ram - ___ram_for_bootloader_api,
                        LENGTH = ___ram_for_bootloader_api
  BCR_CONFIG (r)    : ORIGIN = 0x41C00000, LENGTH = 0x00000080
  BSL_CONFIG (r)    : ORIGIN = 0x41C00100, LENGTH = 0x00000080
}

__flash_start = ORIGIN(FLASH);
__flash_end = ORIGIN(FLASH) + LENGTH(FLASH);
/*__eeprom_start = ORIGIN(EEPROMEMU);
__eeprom_end = ORIGIN(EEPROMEMU) + LENGTH(EEPROMEMU);
__bootloader_start = ORIGIN(BOOTLOADER);*/
__app_header_offset = 0x270;
__app_header_address = ORIGIN(FLASH) + __app_header_offset;
__bootloader_magic_ptr = ORIGIN(RAM);
__application_node_id = ORIGIN(BOOTLOADERAPI);

/*
SECTIONS {
    .BCRConfig :
    {
        KEEP(*(.BCRConfig))
    } > BCR_CONFIG

    .BSLConfig :
    {
        KEEP(*(.BSLConfig))
    } > BSL_CONFIG
}
*/
