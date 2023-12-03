/** \copyright
 * Copyright (c) 2023, Balazs Racz
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
 * \file TinyUsbCdcImpl.hxx
 * Base class for implementing a CDC Device driver using the TinyUsb stack.
 *
 * @author Balazs Racz
 * @date 13 Nov 2023
 */

// This file does not compile standalone, instead it has to be included in the
// board-speciofic Cdc driver implementation cxx file.
#ifndef _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDCIMPL_HXX_
#define _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDCIMPL_HXX_

#include "freertos_drivers/tinyusb/TinyUsbCdc.hxx"

#include "freertos_drivers/common/DeviceBuffer.hxx"
#include "os/OS.hxx"
#include <fcntl.h>

#include "tusb.h"

#ifndef USBD_STACK_SIZE
#define USBD_STACK_SIZE 768
#endif

#ifndef USBD_TASK_PRIO
#define USBD_TASK_PRIO 3
#endif

TinyUsbCdc::~TinyUsbCdc()
{
}

void TinyUsbCdc::hw_postinit()
{
    usbdThread_.start("tinyusb_device", USBD_TASK_PRIO, USBD_STACK_SIZE);
}

void *TinyUsbCdc::UsbDeviceThread::entry()
{
    // init device stack on configured roothub port
    // This should be called after scheduler/kernel is started.
    // Otherwise it could cause kernel issue since USB IRQ handler does use RTOS
    // queue API.
    tud_init(BOARD_TUD_RHPORT);

    // RTOS forever loop
    while (1)
    {
        // put this thread to waiting state until there is new events
        tud_task();

        // following code only run if tud_task() process at least 1 event
        tud_cdc_write_flush();
    }
}

ssize_t TinyUsbCdc::read(File *file, void *buf, size_t count)
{
    unsigned char *data = (unsigned char *)buf;
    ssize_t result = 0;

    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we read with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t bytes_read = tud_cdc_read(data, count < 64 ? count : 64);
        portEXIT_CRITICAL();

        if (bytes_read == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                /* wait for data to come in, this call will release the
                 * critical section lock.
                 */
                DeviceBufferBase::block_until_condition(file, true);
            }
        }

        count -= bytes_read;
        result += bytes_read;
        data += bytes_read;
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result;
}

ssize_t TinyUsbCdc::write(File *file, const void *buf, size_t count)
{
    const unsigned char *data = (const unsigned char *)buf;
    ssize_t result = 0;

    while (count)
    {
        portENTER_CRITICAL();
        /* We limit the amount of bytes we write with each iteration in order
         * to limit the amount of time that interrupts are disabled and
         * preserve our real-time performance.
         */
        size_t bytes_written = tud_cdc_write(data, count < 64 ? count : 64);
        portEXIT_CRITICAL();

        if (bytes_written == 0)
        {
            /* no more space to send data */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                /* wait for space to be available, this call will release the
                 * critical section lock.
                 */
                DeviceBufferBase::block_until_condition(file, false);
            }
        }

        count -= bytes_written;
        result += bytes_written;
        data += bytes_written;
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    if (result)
    {
        tud_cdc_write_flush();
    }

    return result;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool TinyUsbCdc::select(File *file, int mode)
{
    bool retval = false;
    switch (mode)
    {
        case FREAD:
            portENTER_CRITICAL();
            if (tud_cdc_available() > 0)
            {
                retval = true;
            }
            else
            {
                Device::select_insert(&selectInfoRead_);
            }
            portEXIT_CRITICAL();
            break;
        case FWRITE:
            portENTER_CRITICAL();
            if (tud_cdc_write_available() > 0)
            {
                retval = true;
            }
            else
            {
                Device::select_insert(&selectInfoWrite_);
            }
            portEXIT_CRITICAL();
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    return retval;
}

inline void TinyUsbCdc::rx_available()
{
    Device::select_wakeup(&selectInfoRead_);
}

inline void TinyUsbCdc::tx_complete()
{
    Device::select_wakeup(&selectInfoWrite_);
}

extern "C" {

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
    (void)itf;
    Singleton<TinyUsbCdc>::instance()->rx_available();
}

// Invoked when a TX is complete and therefore space becomes available in TX
// buffer
void tud_cdc_tx_complete_cb(uint8_t itf)
{
    (void)itf;
    Singleton<TinyUsbCdc>::instance()->tx_complete();
}

// ===================== USB DESCRIPTORS =============================

/* A combination of interfaces must have a unique product id, since PC will save
 * device driver after the first plug. Same VID/PID with different interface e.g
 * MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]         HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n) ((CFG_TUD_##itf) << (n))
#define USB_PID                                                                \
    (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) |         \
        _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4))

#define USB_VID 0xCAFE
#define USB_BCD 0x0200

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void)
{
    static tusb_desc_device_t const desc_device = {
        .bLength = sizeof(tusb_desc_device_t),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = USB_BCD,

        // Use Interface Association Descriptor (IAD) for CDC
        // As required by USB Specs IAD's subclass must be common class (2) and
        // protocol must be IAD (1)
        .bDeviceClass = TUSB_CLASS_MISC,
        .bDeviceSubClass = MISC_SUBCLASS_COMMON,
        .bDeviceProtocol = MISC_PROTOCOL_IAD,

        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = USB_VID,
        .idProduct = USB_PID,
        .bcdDevice = 0x0100,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01};

    return (uint8_t const *)&desc_device;
}

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

// full speed configuration
uint8_t const desc_fs_configuration[] = {
    // Config number, interface count, string index, total length, attribute,
    // power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Interface number, string index, EP notification address and size, EP data
    // address (out, in) and size.
    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

#if TUD_OPT_HIGH_SPEED
// Per USB specs: high speed capable device must report device_qualifier and
// other_speed_configuration

// high speed configuration
uint8_t const desc_hs_configuration[] = {
    // Config number, interface count, string index, total length, attribute,
    // power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),

    // Interface number, string index, EP notification address and size, EP data
    // address (out, in) and size.
    TUD_CDC_DESCRIPTOR(
        ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 512),
};

// other speed configuration
uint8_t desc_other_speed_config[CONFIG_TOTAL_LEN];

// device qualifier is mostly similar to device descriptor since we don't change
// configuration based on speed
tusb_desc_device_qualifier_t const desc_device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = USB_BCD,

    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0x00};

// Invoked when received GET DEVICE QUALIFIER DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete. device_qualifier descriptor describes
// information about a high-speed capable device that would change if the device
// were operating at the other speed. If not highspeed capable stall this
// request.
uint8_t const *tud_descriptor_device_qualifier_cb(void)
{
    return (uint8_t const *)&desc_device_qualifier;
}

// Invoked when received GET OTHER SEED CONFIGURATION DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete Configuration descriptor in the other speed
// e.g if high speed then this is for full speed and vice versa
uint8_t const *tud_descriptor_other_speed_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations

    // if link speed is high return fullspeed config, and vice versa
    // Note: the descriptor type is OHER_SPEED_CONFIG instead of CONFIG
    memcpy(desc_other_speed_config,
        (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_fs_configuration
                                             : desc_hs_configuration,
        CONFIG_TOTAL_LEN);

    desc_other_speed_config[1] = TUSB_DESC_OTHER_SPEED_CONFIG;

    return desc_other_speed_config;
}

#endif // highspeed

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index; // for multiple configurations

#if TUD_OPT_HIGH_SPEED
    // Although we are highspeed, host may be fullspeed.
    return (tud_speed_get() == TUSB_SPEED_HIGH) ? desc_hs_configuration
                                                : desc_fs_configuration;
#else
    return desc_fs_configuration;
#endif
}

// array of pointer to string descriptors
char const *string_desc_arr[] = {
    (const char[]) {0x09, 0x04}, // 0: is supported language is English (0x0409)
    "TinyUSB",                   // 1: Manufacturer
    "TinyUSB Device",            // 2: Product
    "123456789012",              // 3: Serials, should use chip ID
    "TinyUSB CDC",               // 4: CDC Interface
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == 0)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        if (!(index < sizeof(string_desc_arr) / sizeof(string_desc_arr[0])))
            return NULL;

        const char *str = string_desc_arr[index];

        // Cap at max char
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31)
            chr_count = 31;

        // Convert ASCII string into UTF-16
        for (uint8_t i = 0; i < chr_count; i++)
        {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}

} // extern "C"

#endif // _FREERTOS_DRIVERS_TINYUSB_TINYUSBCDCIMPL_HXX_
