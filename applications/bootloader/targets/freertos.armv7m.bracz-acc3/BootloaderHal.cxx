#include <string.h>

#define TARGET_IS_TM4C123_RB1

#include "bootloader_hal.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"

extern "C" {

void bootloader_hw_set_to_safe(void)
{
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_5);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    ROM_GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
    ROM_GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_5, 0);
    ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);

    ROM_GPIOPinWrite(GPIO_PORTE_BASE,
                     GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, 0);
    ROM_GPIOPinWrite(GPIO_PORTD_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
}

void get_flash_boundaries(const void **flash_min, const void **flash_max,
                          const struct app_header **app_header)
{
    extern char __flash_start;
    extern char __flash_end;
    *flash_min = &__flash_start;
    *flash_max = &__flash_end;
    uint32_t *resetptr = reinterpret_cast<uint32_t *>(&__flash_start);
    *app_header = reinterpret_cast<const struct app_header *>(resetptr + 134);
}

void checksum_data(const void *data, uint32_t size, uint32_t *checksum)
{
    memset(checksum, 0, 16);
    ROM_Crc16Array3(size / 4, (uint32_t *)data,
                    reinterpret_cast<uint16_t *>(checksum));
}

extern void bootloader_reset_segments(void);


void bootloader_hw_init() {
    bootloader_reset_segments();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_20MHZ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIOPinConfigure(GPIO_PB4_CAN0RX);
    ROM_GPIOPinConfigure(GPIO_PB5_CAN0TX);
    ROM_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    
}

bool request_bootloader() {
    return false;
}

uint16_t nmranet_alias() {
    /// TODO(balazs.racz):  fix this
    return 0x428;
}

bool read_can_frame(struct can_frame *frame) {
    return false;
}

bool try_send_can_frame(const struct can_frame &frame) {
    return false;
}

void bootloader_reboot(void) {
    ROM_SysCtlReset();
}

void application_entry(void) {
    typedef void fun_t(void);
    fun_t *fun;
    fun = (fun_t*)(0x180);
    fun();
}

void ignore_fn(void) {}


} // extern "C"
