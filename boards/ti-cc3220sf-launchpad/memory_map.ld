___main_stack_length = 0x1000;
___total_ram = 256K - 0x4000;

MEMORY
{
  RAM2 (rw)  : ORIGIN = 0x20000000, LENGTH = 0x4000
  RAM  (rwx) : ORIGIN = 0x20004000, LENGTH = 0x35000 /* 212K */
  BOOTLOADER (rx) : ORIGIN = 0x20039000, LENGTH = 0x7000 - ___main_stack_length
  STACK (rw) : ORIGIN = 0x20004000 + ___total_ram - ___main_stack_length, LENGTH = ___main_stack_length - 4

    /* This is where the ROM bootlaoder puts the SPIflash image of the copyloader. */
  TRAMPOLINE (rwx) : ORIGIN = 0x20004000, LENGTH = 40K
}



__application_code_start = ORIGIN(RAM);
__application_code_end = ORIGIN(RAM) + LENGTH(RAM);
__app_header_address = ORIGIN(RAM) + 0x270;
__bootloader_magic_ptr = ORIGIN(RAM) + ___total_ram - 4;
/* This pointer will be written to the SP register at reset. */
PROVIDE(__stack = ORIGIN(STACK) + LENGTH(STACK));

ASSERT(ORIGIN(RAM) + LENGTH(RAM) == ORIGIN(BOOTLOADER), "RAM should touch bootloader.");
ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(STACK), "BOOTLOADER should touch stack.");
ASSERT(ORIGIN(STACK) + LENGTH(STACK) == __bootloader_magic_ptr, "STACK should be at the top of RAM - 4.");


/*__ram_start = ORIGIN(RAM);
__bootloader_start = ORIGIN(BOOTLOADER);
__bootloader_end = ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER);
__app_header_offset = 0x270;*/

