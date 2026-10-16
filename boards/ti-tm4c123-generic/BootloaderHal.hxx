/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \file BootloaderHal.hxx
 *
 * Tiva-specific implementation of the HAL (Hardware Abstraction Layer) used by
 * the OpenLCB bootloader.
 *
 * Usage: You have to define in your makefile a symbol like
 *   CXXFLAGSEXTRA+= -DTARGET_IS_TM4C123_RB1
 * 129_RA1 
 * @author Balazs Racz
 * @date 31 January 2015
 */

#ifndef _BOARDS_TI_TIVA_BOOTLOADERHAL_HXX_
#define _BOARDS_TI_TIVA_BOOTLOADERHAL_HXX_

#include <string.h>

#include "openlcb/bootloader_hal.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/can.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"

#include "nmranet_config.h"
#include "openlcb/Defs.hxx"
#include "freertos_drivers/ti/TivaGPIO.hxx"

extern "C" {

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

extern const uint16_t DEFAULT_ALIAS;

uint16_t nmranet_alias()
{
    /// TODO(balazs.racz):  fix this
    return DEFAULT_ALIAS;
}

extern const openlcb::NodeID NODE_ID;

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
        //MAP_GPIOPinWrite(LED_GOLD, 0);
        return false;
    }

    tCANMsgObject can_message;
    can_message.pui8MsgData = frame->data;

    /* Read a message from CAN and clear the interrupt source */
    MAP_CANMessageGet(CAN0_BASE, 1, &can_message, 1 /* clear interrupt */);
    if (can_message.ui32Flags & MSG_OBJ_DATA_LOST)
    {
        bootloader_led(LED_FRAME_LOST, 1);
    }
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
    extern uint64_t __application_node_id;
    __application_node_id = nmranet_nodeid();
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

void raw_erase_flash_page(const void *address)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);
    ROM_FlashErase((uint32_t)address);
    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void erase_flash_page(const void *address)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);
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
    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void raw_write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);
    ROM_FlashProgram((uint32_t*)data, (uint32_t)address, (size_bytes + 3) & ~3);
    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);
    extern char __flash_start;
    if (address == &__flash_start) {
        address = static_cast<const uint8_t*>(address) + 8;
        data = static_cast<const uint8_t*>(data) + 8;
        size_bytes -= 8;
    }
    ROM_FlashProgram((uint32_t*)data, (uint32_t)address, (size_bytes + 3) & ~3);
    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
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

uint16_t flash_complete(void)
{
    return 0;
}

void ignore_fn(void)
{
}

} // extern "C"


#endif // _BOARDS_TI_TIVA_BOOTLOADERHAL_HXX_
