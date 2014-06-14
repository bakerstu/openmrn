/*
 * libmaple linker script for "Flash" builds.
 *
 * A Flash build puts .text (and .rodata) in Flash, and
 * .data/.bss/heap (of course) in SRAM, but offsets the sections by
 * enough space to store the Maple bootloader, which lives in low
 * Flash and uses low memory.
 */

/*
 * This pulls in the appropriate MEMORY declaration from the right
 * subdirectory of stm32/mem/ (the environment must call ld with the
 * right include directory flags to make this happen). Boards can also
 * use this file to use any of libmaple's memory-related hooks (like
 * where the heap should live).
 */
INCLUDE mem-flash.inc

/* Provide memory region aliases for common.inc */
REGION_ALIAS("REGION_TEXT", rom);
REGION_ALIAS("REGION_DATA", ram);
REGION_ALIAS("REGION_BSS", ram);
REGION_ALIAS("REGION_RODATA", rom);

EXTERN(ignore_fn);

/* Let common.inc handle the real work. */
INCLUDE common.inc

PROVIDE(__cs3_heap_start = __bss_end__);
__StackTop = __msp_init;
__StackLimit = __StackTop - 0x200;
PROVIDE(__cs3_heap_end = __StackLimit);
