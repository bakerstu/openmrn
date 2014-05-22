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
    : Serial(name)
    , usbdcdcDevice{USB_VID_TI_1CBE, USB_PID_SERIAL, 0, USB_CONF_ATTR_SELF_PWR, control_callback, this, rx_callback, this, tx_callback, this, stringDescriptors, NUM_STRING_DESCRIPTORS}
    , interrupt(interrupt)
    , connected(false)
    , enabled(false)
    , woken(false)
{
    USBStackModeSet(0, eUSBModeDevice, 0);
    USBDCDCInit(0, &usbdcdcDevice);
    instances[0] = this;
}

/** Enable use of the device interrupts.
 */
void TivaCdc::enable()
{
    enabled = true;
}

/** Disable use of the device interrupts.
 */
void TivaCdc::disable()
{
    enabled = false;
}

/* Try and transmit a message.
 * @param dev device to transmit message on
 */
void TivaCdc::tx_char()
{
    #define CDC_SERIAL_STATE  USB_CDC_SERIAL_STATE_TXCARRIER | \
                              USB_CDC_SERIAL_STATE_RXCARRIER
    
    MAP_IntDisable(interrupt);
    if (connected)
    {
        unsigned long available;
        unsigned long count;

        /* we don't do handshakes, data is always available */
        USBDCDCSerialStateChange(&usbdcdcDevice, CDC_SERIAL_STATE);

        /* do this if we have data to send */
        available = USBDCDCTxPacketAvailable(&usbdcdcDevice);
        
        for (count = 0; count < USB_CDC_TX_DATA_SIZE && count < available; count++)
        {
            if (os_mq_timedreceive(txQ, &txData[count], 0) != OS_MQ_NONE)
            {
                /* no more data left to transmit */
                break;
            }
        }
        if (count)
        {
            /* we have some data to send */
            USBDCDCPacketWrite(&usbdcdcDevice, txData, count, 1);
        }
        MAP_IntEnable(interrupt);
    }
    else
    {
        MAP_IntEnable(interrupt);
        /* we are not connected, flush away the data */
        int result;
        do
        {
            unsigned char data;
            result = os_mq_timedreceive(txQ, &data, 0);
        } while (result == OS_MQ_NONE);
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

    switch(event)
    {
        case USB_EVENT_CONNECTED:
            serial->connected = true;
            break;
        case USB_EVENT_DISCONNECTED:
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
            if (msg_data)
            {
                unsigned char *data = (unsigned char*)msg_data;
                unsigned long count = 0;
                if (serial->enabled)
                {
                    for (count = 0; count < msg_param; count++, data++)
                    {
                        if (os_mq_send_from_isr(serial->rxQ, data, &serial->woken) != OS_MQ_NONE)
                        {
                            /* no more room left */
                            break;
                        }
                    }
                }
                return count;
            }
            else
            {
                //unsigned long available;
                unsigned long space;
                unsigned long count;
                
                //available = USBDCDCRxPacketAvailable(&serial->usbdcdcDevice);
                space = config_serial_rx_buffer_size() - os_mq_num_pending_from_isr(serial->rxQ);

                count = USBDCDCPacketRead(&serial->usbdcdcDevice,
                                          serial->rxData,
                                          space,
                                          true);
                if (serial->enabled)
                {
                    /* transfer data up */
                    for (unsigned long i = 0; i < count; i++)
                    {
                        os_mq_send_from_isr(serial->rxQ, &serial->rxData[i], &serial->woken);
                    }
                }
                
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
            unsigned long available;
            unsigned long count;

            /* do this if we have data to send */
            available = USBDCDCTxPacketAvailable(&serial->usbdcdcDevice);
            
            for (count = 0; count < USB_CDC_TX_DATA_SIZE && count < available; count++)
            {
                if (os_mq_receive_from_isr(serial->txQ, &serial->txData[count], &serial->woken) != OS_MQ_NONE)
                {
                    /* no more data left to transmit */
                    break;
                }
            }
            if (count)
            {
                /* we have some data to send */
                USBDCDCPacketWrite(&serial->usbdcdcDevice, serial->txData, count, 1);
            }
            break;
        }
    }
    return 0;
}

/** Interrupt Handler in context.
 */
void TivaCdc::interrupt_handler(void)
{
    woken = false;
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
