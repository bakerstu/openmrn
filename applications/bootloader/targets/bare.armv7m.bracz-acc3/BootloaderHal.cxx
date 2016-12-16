#include <string.h>

#define TARGET_IS_TM4C123_RB1

#include "../boards/ti-tm4c123-generic/BootloaderHal.hxx"
#include "bootloader_hal.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"

#include "nmranet_config.h"
#include "openlcb/Defs.hxx"
#include "TivaGPIO.hxx"

extern "C" {

GPIO_PIN(GOLD_SW, GpioInputPU, C, 7);

GPIO_PIN(GOLD_LED, LedPin, B, 6);
GPIO_PIN(BLUE_LED, LedPin, B, 7);

GPIO_PIN(GREEN, LedPin, D, 5);
GPIO_PIN(YELLOW, LedPin, B, 0);
GPIO_PIN(BLUE, LedPin, G, 1);

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
    GOLD_SW_Pin::hw_set_to_safe();
}

extern void bootloader_reset_segments(void);
extern unsigned long cm3_cpu_clock_hz;

void bootloader_hw_init()
{
    bootloader_reset_segments();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_20MHZ);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    ROM_GPIOPinConfigure(GPIO_PB4_CAN0RX);
    ROM_GPIOPinConfigure(GPIO_PB5_CAN0TX);
    ROM_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    ROM_CANInit(CAN0_BASE);
    ROM_CANBitRateSet(CAN0_BASE, cm3_cpu_clock_hz,
                      config_nmranet_can_bitrate());

    // Sets up CAN message receiving object.
    tCANMsgObject can_message;
    can_message.ui32MsgID = 0;
    can_message.ui32MsgIDMask = 0;
    can_message.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    can_message.ui32MsgLen = 8;
    ROM_CANMessageSet(CAN0_BASE, 1, &can_message, MSG_OBJ_TYPE_RX);

    ROM_CANEnable(CAN0_BASE);

    GOLD_SW_Pin::hw_init();
    // Sets up LEDs.
    GOLD_LED_Pin::hw_init();
    BLUE_LED_Pin::hw_init();
    GREEN_Pin::hw_init();
    YELLOW_Pin::hw_init();
    BLUE_Pin::hw_init();
}

void bootloader_led(enum BootloaderLed id, bool value) {
  switch(id) {
  case LED_ACTIVE: GOLD_LED_Pin::set(value); return;
  case LED_WRITING: BLUE_LED_Pin::set(value); return;
  case LED_CSUM_ERROR: BLUE_Pin::set(value); return;
  case LED_REQUEST: YELLOW_Pin::set(value); return;
  default: ; /* ignore */
  }
}

bool request_bootloader()
{
    extern uint32_t __bootloader_magic_ptr;
    if (__bootloader_magic_ptr == REQUEST_BOOTLOADER) {
        __bootloader_magic_ptr = 0;
        GREEN_Pin::set(true);
        return true;
    }
    GREEN_Pin::set(GOLD_SW_Pin::get());
    return !GOLD_SW_Pin::get();
}

} // extern "C"
