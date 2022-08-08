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
 * STM-specific implementation of the HAL (Hardware Abstraction Layer) used by
 * the OpenLCB bootloader.
 *
 * Usage: You have to define in your makefile a symbol like
 *   CXXFLAGSEXTRA+= -DSTM32F091xC
 *
 * @author Balazs Racz
 * @date 31 January 2015
 */

#ifndef _BOARDS_STM32F0_BOOTLOADERHAL_HXX_
#define _BOARDS_STM32F0_BOOTLOADERHAL_HXX_

#include <string.h>

#include "bootloader_hal.h"
#include "stm32f0xx_hal_conf.h"

#include "nmranet_config.h"
#include "openlcb/Defs.hxx"
#include "utils/Crc.hxx"
#include "Stm32Gpio.hxx"

extern "C" {

void get_flash_boundaries(const void **flash_min, const void **flash_max,
                          const struct app_header **app_header)
{
    extern char __flash_start;
    extern char __flash_end;
    extern struct app_header __app_header_address;
    *flash_min = &__flash_start;
    *flash_max = &__flash_end;
    *app_header = &__app_header_address;
}

void checksum_data(const void *data, uint32_t size, uint32_t *checksum)
{
    extern uint8_t __flash_start;
    if (static_cast<const uint8_t*>(data) == &__flash_start)
    {
        // ignores the reset vector for checksum calculations.
        data = static_cast<const uint8_t*>(data) + 8;
        size -= 8;
    }
    memset(checksum, 0, 16);
    crc3_crc16_ibm(data, size, reinterpret_cast<uint16_t *>(checksum));
}

extern const openlcb::NodeID NODE_ID;

uint16_t nmranet_alias()
{
    /// TODO: we should probably read this from someplace else
    return 0x400 ^ (NODE_ID & 0xFFF);
}

uint64_t nmranet_nodeid()
{
    /// TODO(balazs.racz):  read some form of EEPROM instead.
    return NODE_ID;
}

bool read_can_frame(struct can_frame *can_frame)
{
    if (!(CAN->RF0R & CAN_RF0R_FMP0))
    {
        return false;
    }

    /* Read a message from CAN and clear the interrupt source */
    if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_IDE)
    {
        /* extended frame */
        can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 3;
        can_frame->can_eff = 1;
    }
    else
    {
        /* standard frame */
        can_frame->can_id = CAN->sFIFOMailBox[0].RIR >> 21;
        can_frame->can_eff = 0;
    }
    if (CAN->sFIFOMailBox[0].RIR & CAN_RI0R_RTR)
    {
        /* remote frame */
        can_frame->can_rtr = 1;
        can_frame->can_dlc = 0;
    }
    else
    {
        /* data frame */
        can_frame->can_rtr = 0;
        can_frame->can_dlc = CAN->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC;
        can_frame->data[0] = (CAN->sFIFOMailBox[0].RDLR >>  0) & 0xFF;
        can_frame->data[1] = (CAN->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
        can_frame->data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
        can_frame->data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;
        can_frame->data[4] = (CAN->sFIFOMailBox[0].RDHR >>  0) & 0xFF;
        can_frame->data[5] = (CAN->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
        can_frame->data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
        can_frame->data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;
    }
    /* release FIFO */;
    CAN->RF0R |= CAN_RF0R_RFOM0;
    return true;
}

bool try_send_can_frame(const struct can_frame &can_frame)
{
    /* load the next message to transmit */
    volatile CAN_TxMailBox_TypeDef *mailbox;
    if (CAN->TSR & CAN_TSR_TME0)
    {
        mailbox = CAN->sTxMailBox + 0;
    }
    else if (CAN->TSR & CAN_TSR_TME1)
    {
        mailbox = CAN->sTxMailBox + 1;
    }
    else if (CAN->TSR & CAN_TSR_TME2)
    {
        mailbox = CAN->sTxMailBox + 2;
    }
    else
    {
        // no buffer available
        return false;
    }

    /* setup frame */
    if (can_frame.can_eff)
    {
        mailbox->TIR = (can_frame.can_id << 3) | CAN_TI0R_IDE;
    }
    else
    {
        mailbox->TIR = can_frame.can_id << 21;
    }
    if (can_frame.can_rtr)
    {
        mailbox->TIR |= CAN_TI0R_RTR;
    }
    else
    {
        mailbox->TDTR = can_frame.can_dlc;
        mailbox->TDLR = (can_frame.data[0] <<  0) |
                        (can_frame.data[1] <<  8) |
                        (can_frame.data[2] << 16) |
                        (can_frame.data[3] << 24);
        mailbox->TDHR = (can_frame.data[4] <<  0) |
                        (can_frame.data[5] <<  8) |
                        (can_frame.data[6] << 16) |
                        (can_frame.data[7] << 24);
    }

    /* request transmission */
    mailbox->TIR |= CAN_TI0R_TXRQ;

    return true;
}

void bootloader_reboot(void)
{
    /* wait for TX messages to all go out */
    while (!(CAN->TSR & CAN_TSR_TME0));
    while (!(CAN->TSR & CAN_TSR_TME1));
    while (!(CAN->TSR & CAN_TSR_TME2));

    bootloader_hw_set_to_safe();

    HAL_NVIC_SystemReset();
}

void application_entry(void)
{
    bootloader_hw_set_to_safe();
    /* Globally disables interrupts. */
    asm("cpsid i\n");
    extern uint64_t __application_node_id;
    __application_node_id = nmranet_nodeid();
    extern char __flash_start;
    // We store the application reset in interrupt vecor 13, which is reserved
    // / unused on all Cortex-M0 processors.
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

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = (uint32_t)address;
    erase_init.NbPages = 1;

    uint32_t page_error;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    HAL_FLASH_Lock();

    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void erase_flash_page(const void *address)
{
    raw_erase_flash_page(address);

    extern char __flash_start;
    if (static_cast<const char*>(address) == &__flash_start) {
        // If we erased page zero, we ensure to write back the reset pointer
        // immiediately or we brick the bootloader.
        extern unsigned long *__stack;
        extern void reset_handler(void);
        uint32_t bootdata[2];
        bootdata[0] = reinterpret_cast<uint32_t>(&__stack);
        bootdata[1] = reinterpret_cast<uint32_t>(&reset_handler);
        raw_write_flash(address, bootdata, sizeof(bootdata));
    }
}

void raw_write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    uint32_t *data_ptr = (uint32_t*)data;
    uint32_t  addr_ptr = (uintptr_t)address;
    bootloader_led(LED_ACTIVE, 0);
    bootloader_led(LED_WRITING, 1);

    HAL_FLASH_Unlock();
    while (size_bytes)
    {
        HAL_FLASH_Program((uint32_t)FLASH_TYPEPROGRAM_WORD,
                          (uint32_t)addr_ptr, *data_ptr);
        size_bytes -= sizeof(uint32_t);
        addr_ptr += sizeof(uint32_t);
        ++data_ptr;
    }
    HAL_FLASH_Lock();

    bootloader_led(LED_WRITING, 0);
    bootloader_led(LED_ACTIVE, 1);
}

void write_flash(const void *address, const void *data, uint32_t size_bytes)
{
    extern char __flash_start;
    if (address == &__flash_start) {
        address = static_cast<const uint8_t*>(address) + 8;
        data = static_cast<const uint8_t*>(data) + 8;
        size_bytes -= 8;
    }
    raw_write_flash(address, (uint32_t*)data, (size_bytes + 3) & ~3);
}

void get_flash_page_info(const void *address, const void **page_start,
                         uint32_t *page_length_bytes)
{
    // STM32F091 has 2 KB flash pages.
    uint32_t value = (uint32_t)address;
    value &= ~(FLASH_PAGE_SIZE - 1);
    *page_start = (const void *)value;
    *page_length_bytes = FLASH_PAGE_SIZE;
}

uint16_t flash_complete(void)
{
    return 0;
}

void ignore_fn(void)
{
}

} // extern "C"


#endif // _BOARDS_STM32F0_BOOTLOADERHAL_HXX_
