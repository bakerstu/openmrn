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
}

bool request_bootloader()
{
    return false;
}

uint16_t nmranet_alias()
{
    /// TODO(balazs.racz):  fix this
    return 0x428;
}

extern const nmranet::NodeID NODE_ID;

uint64_t nmranet_nodeid()
{
    /// TODO(balazs.racz):  fix this
    return NODE_ID;
}

bool read_can_frame(struct can_frame *frame)
{
    uint32_t regbits = ROM_CANStatusGet(CAN0_BASE, CAN_STS_NEWDAT);
    if (!(regbits & 1))
    {
        return false;
    }

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

void ignore_fn(void)
{
}

} // extern "C"
