#include "freertos/bootloader_hal.h"

#include "inc/hw_memmap.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/prcm.h"

#include "utils/Crc.hxx"

constexpr auto UART_PERIPH = PRCM_UARTA0;
constexpr auto UART_BASE = UARTA0_BASE;
constexpr auto UART_BAUD_RATE = 115200;


void serial_print(const char *s)
{
    if (s != NULL)
    {
        while (*s != '\0')
        {
            MAP_UARTCharPut(UART_BASE, *s++);
        }
    }
}

void serial_println(const char *s)
{
    serial_print(s);
    serial_print("\r\n");
}

static const char HEX_CHARS[] = "0123456789ABCDEF";
void serial_printhex(unsigned long data)
{
    for (int i = 7; i >= 0; --i) {
        unsigned nib = (data >> (i * 4)) & 0xf;
        MAP_UARTCharPut(UART_BASE, HEX_CHARS[nib]);
    }
}

void serial_init()
{
    /* UART0 initialization */
    MAP_PRCMPeripheralClkEnable(UART_PERIPH, PRCM_RUN_MODE_CLK);
    MAP_UARTConfigSetExpClk(UART_BASE, MAP_PRCMPeripheralClockGet(UART_PERIPH),
        UART_BAUD_RATE,
        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    serial_println("\nBooting.");
}

extern "C" {

/** @return the maximum size of the application binary (i.e., the amount of
 * flash space allocated for the application to occupy. */
uint32_t application_size()
{
    extern char __application_code_start;
    extern char __application_code_end;
    size_t sz = &__application_code_end - &__application_code_start;
    return sz;
}

void application_entry(void)
{
    serial_println("Starting application.");
    sl_Stop(0xFF);
    
    extern char __application_code_start;
    // We store the application reset in interrupt vecor 13, which is reserved
    // / unused on all Cortex_M3 processors.
    asm volatile(" cpsid i\n"
                 " mov   r3, %[flash_addr] \n"
                 :
                 : [flash_addr] "r"(&__application_code_start));
    asm volatile(" ldr r0, [r3]\n"
                 " mov sp, r0\n"
                 " ldr r0, [r3, #4]\n"
                 " bx  r0\n");
}

void checksum_data(const void *data, uint32_t size, uint32_t *checksum)
{
#ifdef BOOTLOADER_DEBUG
    serial_print("csum(");
    serial_printhex((unsigned) data);
    serial_print(", ");
    serial_printhex(size);
    serial_print(")=");
#endif
    memset(checksum, 0, 16);
    // TODO: use the hardware crc engine of the CC3200 instead of this function.
    crc3_crc16_ibm(data, size, (uint16_t *)checksum);

    // We put some magic into the checksum as well.
    checksum[2] = 0x73a92bd1;
    checksum[3] = 0x5a5a55aa;

#ifdef BOOTLOADER_DEBUG
    for (int i = 0; i < 4; ++i) {
        serial_print(" ");
        serial_printhex(checksum[i]);
    }
    serial_println("");
#endif
}

void get_flash_boundaries(const void **flash_min, const void **flash_max,
    const struct app_header **app_header)
{
    extern char __application_code_start;
    extern char __application_code_end;
    extern char __app_header_address;
    *flash_min = (void *)(&__application_code_start);
    *flash_max = (void *)(&__application_code_end);
    *app_header = (struct app_header *)(&__app_header_address);
}

}
