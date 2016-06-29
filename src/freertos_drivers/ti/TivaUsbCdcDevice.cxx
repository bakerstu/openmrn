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
/// not sure why this is needed
#define gcc
#endif

#include <algorithm>
#include <cstdint>
#include <fcntl.h>
#include <sys/select.h>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "TivaDev.hxx"

/** This is fixed and equals the USB packet size that the CDC device will
 * advertise to be able to receive. This is a performance parameter, 64 is the
 * largest packet size permitted by USB for virtual serial ports.
 */
#define TIVA_USB_PACKET_SIZE 64

/** This is the size of the RX buffer in bytes.  This is a performance
 * parameter and larger values will result in higher max throughput.
 */
#define TIVA_USB_BUFFER_SIZE (TIVA_USB_PACKET_SIZE * 4)

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

/** number of string descriptors */
#define NUM_STRING_DESCRIPTORS (sizeof(stringDescriptors) / sizeof(uint8_t *))

/** Instance pointers help us get context from the interrupt handler(s) */
static TivaCdc *instances[1] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param interrupt interrupt number used by the device
 */
TivaCdc::TivaCdc(const char *name, uint32_t interrupt)
    : Serial(name, 0, TIVA_USB_BUFFER_SIZE)
    , usbdcdcDevice{USB_VID_TI_1CBE, USB_PID_SERIAL, 0, USB_CONF_ATTR_SELF_PWR,
                    control_callback, this, rx_callback, this, tx_callback,
                    this, stringDescriptors, NUM_STRING_DESCRIPTORS}
    , interrupt(interrupt)
    , connected(false)
    , enabled(false)
    , woken(false)
    , txPending(false)
    , lineCoding{115200, USB_CDC_STOP_BITS_1, USB_CDC_PARITY_NONE, 8}
    , selInfoWr()
{
    instances[0] = this;

    /* USB interrupt low priority.  We make this a low priority because
     * USB interrupts perform a decent amount of processing and we want
     * to maintain the overall determinism of our system.
     */
    MAP_IntPrioritySet(interrupt, 0xff);

    USBStackModeSet(0, eUSBModeForceDevice, 0);
    USBDCDCInit(0, &usbdcdcDevice);
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

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t TivaCdc::write(File *file, const void *buf, size_t count)
{
    unsigned char *data = (unsigned char*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        if (!connected)
        {
            if (file->flags & O_NONBLOCK)
            {
                return -EAGAIN;
            }
            fd_set fds;
            FD_ZERO(&fds);
            int fd = Device::fd_lookup(file);
            FD_SET(fd, &fds);

            ::select(fd + 1, NULL, &fds, NULL, NULL);
        }
        portENTER_CRITICAL();
        size_t size = std::min((size_t)TIVA_USB_PACKET_SIZE, count);
        uint32_t sent = USBDCDCPacketWrite(&usbdcdcDevice, data,
                                           size, true);
        if (sent == 0)
        {
            portEXIT_CRITICAL();
            if (result > 0)
            {
                break;
            }
            if (file->flags & O_NONBLOCK)
            {
                return -EAGAIN;
            }
            else
            {
                fd_set fds;
                FD_ZERO(&fds);
                int fd = Device::fd_lookup(file);
                FD_SET(fd, &fds);

                ::select(fd + 1, NULL, &fds, NULL, NULL);
            }
        }
        else
        {
            txPending = true;
            portEXIT_CRITICAL();
            count -= sent;
            result += sent;
            data += sent;
        }
    }

    return result;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool TivaCdc::select(File* file, int mode)
{
    bool retval = false;

    portENTER_CRITICAL();
    switch (mode)
    {
        case FREAD:
            if (rxBuf->pending() > 0)
            {
                retval = true;
            }
            else
            {
                rxBuf->select_insert();
            }
            break;
        case FWRITE:
            if (connected)
            {
                if (txPending == false)
                {
                    retval = true;
                    break;
                }
            }
            select_insert(&selInfoWr);
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    portEXIT_CRITICAL();

    return retval;
}

/** Handles CDC driver notifications related to control and setup of the device.
 * This is called from within interrupt context.
 * @param data private data
 * @param event identifies the event we are being notified about
 * @param msg_value event-specific value
 * @param msg_data event-specific data
 * @return return value is event specific
 */
uint32_t TivaCdc::control_callback(void *data, unsigned long event,
                                   unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;

    switch(event)
    {
        case USB_EVENT_CONNECTED:
            serial->connected = true;
            // starts sending data.
            serial->select_wakeup_from_isr(&serial->selInfoWr, &serial->woken);
            break;
        case USB_EVENT_DISCONNECTED:
            serial->connected = false;
            break;
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            tLineCoding *line_coding = (tLineCoding*)msg_data;
            *line_coding = serial->lineCoding;
            break;
        }
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
            tLineCoding *line_coding = (tLineCoding*)msg_data;
            serial->lineCoding = *line_coding;
            break;
        }
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

/** Handles CDC driver notifications related to reception.
 * This is called from within interrupt context.
 * @param data private data
 * @param event identifies the event we are being notified about
 * @param msg_value event-specific value
 * @param msg_data event-specific data
 * @return return value is event specific
 */
uint32_t TivaCdc::rx_callback(void *data, unsigned long event,
                              unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;

    switch (event)
    {
        default:
            break;
        case USB_EVENT_RX_AVAILABLE:
        {
            HASSERT(msg_data == NULL);
            uint8_t *data;
            size_t space = serial->rxBuf->data_write_pointer(&data);
            uint32_t count = USBDCDCPacketRead(&serial->usbdcdcDevice,
                                               data, space, true);
            if (serial->enabled && count)
            {
                /* we only commit the data if the device is open */
                serial->rxBuf->advance(count);
                serial->rxBuf->signal_condition_from_isr();
            }
            return count;
        }
        case USB_EVENT_DATA_REMAINING:
            return serial->rxBuf->pending();
        case USB_EVENT_ERROR:
            break;
    }
    return 0;
}

/** Handles CDC driver notifications related to transmission.
 * This is called from within interrupt context.
 * @param data private data
 * @param event identifies the event we are being notified about
 * @param msg_value event-specific value
 * @param msg_data event-specific data
 * @return return value is event specific
 */
uint32_t TivaCdc::tx_callback(void *data, unsigned long event,
                              unsigned long msg_param, void *msg_data)
{
    TivaCdc *serial = (TivaCdc*)data;
    
    switch (event)
    {
        default:
            break;
        case USB_EVENT_TX_COMPLETE:
            serial->txPending = false;
            serial->select_wakeup_from_isr(&serial->selInfoWr, &serial->woken);
            break;
    }
    return 0;
}

/** Interrupt Handler in context.
 */
void TivaCdc::interrupt_handler(void)
{
    woken = 0;
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
