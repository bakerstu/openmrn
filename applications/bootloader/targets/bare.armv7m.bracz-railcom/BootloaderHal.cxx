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
#define PINDEFS_ONLY
#include "hardware.hxx"

//OVERRIDE_CONST(nmranet_can_bitrate, 1000000);

extern "C" {

void bootloader_hw_set_to_safe(void)
{
    BootloaderGpioInit::hw_set_to_safe();
}

extern void bootloader_reset_segments(void);
extern unsigned long cm3_cpu_clock_hz;

void bootloader_hw_init()
{
    bootloader_reset_segments();
    BootloaderGpioInit::hw_init();
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
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
}

void bootloader_led(enum BootloaderLed id, bool value) {
  switch(id) {
  case LED_ACTIVE: LED_RED_RAW_Pin::set(value); return;
  case LED_WRITING: LED_BLUE_Pin::set(value); return;
  case LED_CSUM_ERROR: LED_RED_RAW_Pin::set(value); return;
  case LED_REQUEST: LED_RED_RAW_Pin::set(value); LED_BLUE_Pin::set(value); return;
  case LED_FRAME_LOST: LED_GREEN_Pin::set(1); LED_BLUE_Pin::set(0);
      LED_RED_RAW_Pin::set(0); return;
  default: ; /* ignore */
  }
}

bool request_bootloader()
{
    extern uint32_t __bootloader_magic_ptr;
    if (__bootloader_magic_ptr == REQUEST_BOOTLOADER) {
        __bootloader_magic_ptr = 0;
        LED_GREEN_Pin::set(true);
        return true;
    }
    LED_GREEN_Pin::set(SW1_Pin::get());
    return !SW1_Pin::get();
}

} // extern "C"
