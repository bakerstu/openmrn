/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file TivaUsbCdcDevice.cxx
 * This file implements a USB CDC device driver layer specific to TivaWare.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#ifndef gcc
#define gcc
#endif

#include <stdint.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "TivaDev.hxx"

#include "TivaGPIO.hxx"
GPIO_PIN(LED_B4, LedPin, F, 0);
GPIO_PIN(LED_B3, LedPin, F, 4);


/** The languages supported by this device.
 */
const uint8_t langDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

/** The manufacturur string.
 */
const uint8_t manufacturerString[] =
{
    2 + (7 * 2),
    USB_DTYPE_STRING,
    'O', 0, 'p', 0, 'e', 0, 'n', 0, 'M', 0, 'R', 0, 'N', 0
};

/** The product string.
 */
const uint8_t productString[] =
{
    2 + (16 * 2),
    USB_DTYPE_STRING,
    'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0,
    'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0

};

/** The serial number string.
 */
const uint8_t serialNumberString[] =
{
    2 + (8 * 2),
    USB_DTYPE_STRING,
    '2', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/** The configuration interface description string.
 */
const uint8_t controlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

/** The configuration description string.
 */
const uint8_t configString[] =
{
    2 + (26 * 2),
    USB_DTYPE_STRING,
    'S', 0, 'e', 0, 'l', 0, 'f', 0, ' ', 0, 'P', 0, 'o', 0, 'w', 0,
    'e', 0, 'r', 0, 'e', 0, 'd', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0,
    'f', 0, 'i', 0, 'g', 0, 'u', 0, 'r', 0, 'a', 0, 't', 0, 'i', 0,
    'o', 0, 'n', 0
};

/** The descriptor string table.
 */
const uint8_t *const stringDescriptors[] =
{
    langDescriptor,
    manufacturerString,
    productString,
    serialNumberString,
    controlInterfaceString,
    configString
};

#define NUM_STRING_DESCRIPTORS (sizeof(stringDescriptors) / sizeof(uint8_t *))

/** Instance pointers help us get context from the interrupt handler(s) */
static TivaCdc *instances[1] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param interrupt interrupt number used by the device
 */
TivaCdc::TivaCdc(const char *name, uint32_t interrupt)
    : USBSerialNode(name)
    , usbdcdcDevice{USB_VID_TI_1CBE, USB_PID_SERIAL, 0, USB_CONF_ATTR_SELF_PWR, control_callback, this, rx_callback, this, tx_callback, this, stringDescriptors, NUM_STRING_DESCRIPTORS}
    , interrupt(interrupt)
    , connected(false)
    , enabled(false)
    , woken(false)
{
    instances[0] = this;
    log_.log(0x71);
    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDCDCInit(0, &usbdcdcDevice);
}

/** Enable use of the device interrupts.
 */
void TivaCdc::enable()
{
    log_.log(0xD1);
    enabled = true;
}

/** Disable use of the device interrupts.
 */
void TivaCdc::disable()
{
    log_.log(0xD0);
    enabled = false;
}

/* Try and transmit a message.
 */
bool TivaCdc::tx_packet_irqlocked(const void* data, size_t len)
{
    if (connected) {
        uint32_t r = USBDCDCPacketWrite(&usbdcdcDevice, (uint8_t*)data, len, 1);
        if (r == len) {
            log_.log(0x30);
        } else if (r > 0) {
            log_.log(0x31);
        } else {
            log_.log(0x32);
            return false;
        }
        return true;
    } else {
        log_.log(0x38);
        return false;
    }
}

bool TivaCdc::tx_packet_from_isr(const void* data, size_t len)
{
    if (connected) {
        uint32_t r = USBDCDCPacketWrite(&usbdcdcDevice, (uint8_t*)data, len, 1);
        if (r == len) {
            log_.log(0x10);
        } else if (r > 0) {
            log_.log(0x11);
        } else {
            log_.log(0x12);
            return false;
        }
        return true;
    } else {
        log_.log(0x18);
        return false;
    }
}

/** Handles CDC driver notifications related to control and setup of the device.
 * This is called from within interrupt context.
 * @param data private data
 * @param event identifies the event we are being notified about
 * @param msg_value event-specific value
 * @param msg_data event-specific data
 * @return return value is event specific
 */
unsigned long TivaCdc::control_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;

    serial->log_.log(0xC0);
    switch(event)
    {
        case USB_EVENT_CONNECTED:
            serial->log_.log(0xC1);
            serial->connected = true;
            // starts sending data.
            serial->tx_finished_from_isr();
            break;
        case USB_EVENT_DISCONNECTED:
            serial->log_.log(0xC2);
            serial->connected = false;
            break;
        case USBD_CDC_EVENT_GET_LINE_CODING:
            break;
        case USBD_CDC_EVENT_SET_LINE_CODING:
            break;
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            break;
        case USBD_CDC_EVENT_SEND_BREAK:
            break;
        case USBD_CDC_EVENT_CLEAR_BREAK:
            break;
        case USB_EVENT_SUSPEND:
            break;
        case USB_EVENT_RESUME:
            break;
        default:
            break;
    }
    return 0;
}

unsigned long TivaCdc::rx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;

    switch (event)
    {
        default:
            break;
        case USB_EVENT_RX_AVAILABLE:
        {
            void* b = serial->try_read_packet_from_isr();
            if (b) {
                uint8_t count = USBDCDCPacketRead(&serial->usbdcdcDevice,
                                                  (uint8_t*)b,
                                                  USB_SERIAL_PACKET_SIZE,
                                                  true);
                serial->set_rx_finished_from_isr(count);
                LED_B4_Pin::set(false);
                return count;
            } else {
                LED_B4_Pin::set(true);
                return 0;
            }
        }
        case USB_EVENT_DATA_REMAINING:
        case USB_EVENT_ERROR:
            break;
    }
    return 0;
}

unsigned long TivaCdc::tx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;
    
    switch (event)
    {
        default:
            break;
        case USB_EVENT_TX_COMPLETE:
        {
            serial->tx_finished_from_isr();
        }
    }
    return 0;
}

/** Interrupt Handler in context.
 */
void TivaCdc::interrupt_handler(void)
{
    woken = true;
    USB0DeviceIntHandler();
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** Handle interrupts for USB0.
 */
void usb0_interrupt_handler(void)
{
    
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

} // extern "C"
