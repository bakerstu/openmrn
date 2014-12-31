#include <string.h>

#define TARGET_IS_TM4C123_RB1

#include "bootloader_hal.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"

#include "nmranet_config.h"
#include "nmranet/Defs.hxx"

extern "C" {

#define LED_GOLD GPIO_PORTB_BASE, GPIO_PIN_6
#define LED_BLUE GPIO_PORTB_BASE, GPIO_PIN_7

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
    extern struct app_header __app_header_offset;
    *flash_min = &__flash_start;
    *flash_max = &__flash_end;
    *app_header = &__app_header_offset;
}

void checksum_data(const void *data, uint32_t size, uint32_t *checksum)
{
    extern uint8_t __flash_start;
    if (static_cast<const uint8_t*>(data) == &__flash_start) {
        data = static_cast<const uint8_t*>(data) + 8; // ignores the reset vector for checksum calculations.
        size -= 8;
    }
    memset(checksum, 0, 16);
    ROM_Crc16Array3(size / 4, (uint32_t *)data,
                    reinterpret_cast<uint16_t *>(checksum));
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

    // Sets up LEDs.
    MAP_GPIOPinTypeGPIOOutput(LED_GOLD); 
    MAP_GPIOPadConfigSet(LED_GOLD, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(LED_GOLD, 0);
    MAP_GPIOPinTypeGPIOOutput(LED_BLUE); 
    MAP_GPIOPadConfigSet(LED_BLUE, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(LED_BLUE, 0);
}

bool request_bootloader()
{
    return false;
}

extern const uint16_t DEFAULT_ALIAS;

uint16_t nmranet_alias()
{
    /// TODO(balazs.racz):  fix this
    return DEFAULT_ALIAS;
}

extern const nmranet::NodeID NODE_ID;

uint64_t nmranet_nodeid()
{
    /// TODO(balazs.racz):  read some form of EEPROM instead.
    return NODE_ID;
}

bool read_can_frame(struct can_frame *frame)
{
    uint32_t regbits = ROM_CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    if (!(regbits & 1))
    {
        MAP_GPIOPinWrite(LED_GOLD, 0);
        return false;
    }
    MAP_GPIOPinWrite(LED_GOLD, 0xff);

    tCANMsgObject can_message;
    can_message.pui8MsgData = frame->data;

    /* Read a message from CAN and clear the interrupt source */
    MAP_CANMessageGet(CAN0_BASE, 1, &can_message, 1 /* clear interrupt */);

    frame->can_id = can_message.ui32MsgID;
    frame->can_rtr = (can_message.ui32Flags & MSG_OBJ_REMOTE_FRAME) ? 1 : 0;
    frame->can_eff = (can_message.ui32Flags & MSG_OBJ_EXTENDED_ID) ? 1 : 0;
    frame->can_err = 0;
    frame->can_dlc = can_message.ui32MsgLen;
    return true;
}

bool try_send_can_frame(const struct can_frame &frame)
{
    // Checks if previous frame is out yet.
    uint32_t regbits = ROM_CANStatusGet(CAN0_BASE, CAN_STS_TXREQUEST);
    if (regbits & 2)
    {
        return false;
    }

    /* load the next message to transmit */
    tCANMsgObject can_message;
    can_message.ui32MsgID = frame.can_id;
    can_message.ui32MsgIDMask = 0;
    can_message.ui32Flags = 0;
    if (frame.can_eff)
    {
        can_message.ui32Flags |= MSG_OBJ_EXTENDED_ID;
    }
    if (frame.can_rtr)
    {
        can_message.ui32Flags |= MSG_OBJ_REMOTE_FRAME;
    }
    can_message.ui32MsgLen = frame.can_dlc;
    can_message.pui8MsgData = (uint8_t *)frame.data;
    ROM_CANMessageSet(CAN0_BASE, 2, &can_message, MSG_OBJ_TYPE_TX);
    return true;
}

void bootloader_reboot(void)
{
    ROM_SysCtlReset();
}

void application_entry(void)
{
    extern char __flash_start;
    // We store the application reset in interrupt vecor 13, which is reserved
    // / unused on all Cortex_M3 processors.
    asm volatile(" mov   r3, %[flash_addr] \n"
                 :
                 : [flash_addr] "r"(&__flash_start));
    asm volatile(" ldr r0, [r3]\n"
                 " mov sp, r0\n"
                 " ldr r0, [r3, #52]\n"
                 " bx  r0\n");
}

void erase_flash_page(const void *address)
{
    MAP_GPIOPinWrite(LED_GOLD, 0);
    MAP_GPIOPinWrite(LED_BLUE, 0xff);
    ROM_FlashErase((uint32_t)address);
    extern char __flash_start;
    if (static_cast<const char*>(address) == &__flash_start) {
        // If we erased page zero, we ensure to write back the reset pointer
        // immiediately or we brick the bootloader.
        extern unsigned long *__stack;
        extern void reset_handler(void);
        uint32_t bootdata[2];
        bootdata[0] = reinterpret_cast<uint32_t>(&__stack);
        bootdata[1] = reinterpret_cast<uint32_t>(&reset_handler);
        ROM_FlashProgram(bootdata, (uint32_t)address, sizeof(bootdata));
    }
    MAP_GPIOPinWrite(LED_BLUE, 0);
}

void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    MAP_GPIOPinWrite(LED_GOLD, 0);
    MAP_GPIOPinWrite(LED_BLUE, 0xff);
    extern char __flash_start;
    if (address == &__flash_start) {
        address = static_cast<const uint8_t*>(address) + 8;
        data = static_cast<const uint8_t*>(data) + 8;
        size_bytes -= 8;
    }
    ROM_FlashProgram((uint32_t*)data, (uint32_t)address, (size_bytes + 3) & ~3);
    MAP_GPIOPinWrite(LED_BLUE, 0);
}

void get_flash_page_info(const void *address, const void **page_start,
                         uint32_t *page_length_bytes)
{
    // Tiva has 1 KB flash pages.
    uint32_t value = (uint32_t)address;
    value &= ~1023;
    *page_start = (const void *)value;
    *page_length_bytes = 1024;
}

void ignore_fn(void)
{
}

} // extern "C"
