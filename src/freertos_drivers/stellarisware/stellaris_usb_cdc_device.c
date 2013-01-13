/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file stellaris_can.c
 * This file implements a USB CDC device driver layer specific to stellarisware.
 *
 * @author Stuart W. Baker
 * @date 3 January 2013
 */

#define gcc

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

#include "serial.h"

/* prototypes */
static int stellaris_cdc_init(devtab_t *dev);
static void stellaris_cdc_enable(devtab_t *dev);
static void stellaris_cdc_disable(devtab_t *dev);
static void stellaris_cdc_tx_char(devtab_t *dev);
static unsigned long control_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);
static unsigned long rx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);
static unsigned long tx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);


/** The languages supported by this device.
 */
const unsigned char langDescriptor[] =
{
    4,
    USB_DTYPE_STRING,
    USBShort(USB_LANG_EN_US)
};

/** The manufacturur string.
 */
const unsigned char manufacturerString[] =
{
    2 + (7 * 2),
    USB_DTYPE_STRING,
    'O', 0, 'p', 0, 'e', 0, 'n', 0, 'M', 0, 'R', 0, 'N', 0
};

/** The product string.
 */
const unsigned char productString[] =
{
    2 + (16 * 2),
    USB_DTYPE_STRING,
    'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0, 'a', 0, 'l', 0, ' ', 0,
    'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0, 'o', 0, 'r', 0, 't', 0

};

/** The serial number string.
 */
const unsigned char serialNumberString[] =
{
    2 + (8 * 2),
    USB_DTYPE_STRING,
    '2', 0, '2', 0, '3', 0, '4', 0, '5', 0, '6', 0, '7', 0, '8', 0
};

/** The configuration interface description string.
 */
const unsigned char controlInterfaceString[] =
{
    2 + (21 * 2),
    USB_DTYPE_STRING,
    'A', 0, 'C', 0, 'M', 0, ' ', 0, 'C', 0, 'o', 0, 'n', 0, 't', 0,
    'r', 0, 'o', 0, 'l', 0, ' ', 0, 'I', 0, 'n', 0, 't', 0, 'e', 0,
    'r', 0, 'f', 0, 'a', 0, 'c', 0, 'e', 0
};

/** The configuration description string.
 */
const unsigned char configString[] =
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
const unsigned char *const stringDescriptors[] =
{
    langDescriptor,
    manufacturerString,
    productString,
    serialNumberString,
    controlInterfaceString,
    configString
};

#define NUM_STRING_DESCRIPTORS (sizeof(stringDescriptors) / sizeof(unsigned char *))
#define TX_DATA_SIZE 64
#define RX_DATA_SIZE 64

/** Private data for this implementation of serial.
 */
typedef struct stellaris_cdc_priv
{
    SerialPriv serialPriv; /**< common private data */
    tUSBDCDCDevice usbdcdcDevice; /**< CDC serial device instance */
    tCDCSerInstance serialInstance; /**< CDC serial device private data */
    unsigned char txData[TX_DATA_SIZE]; /**< buffer for pending tx data */
    unsigned char rxData[RX_DATA_SIZE]; /**< buffer for pending tx data */
    char connected; /**< connection status */
    char enabled; /**< enabled status */
} StellarisCdcPriv;

static StellarisCdcPriv cdc_priv[1];

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(serUSB0, "/dev/serUSB0", stellaris_cdc_init, &cdc_priv[0]);

/** intitailize the device 
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int stellaris_cdc_init(devtab_t *dev)
{
    StellarisCdcPriv *priv = dev->priv;
    
    priv->usbdcdcDevice.usVID = USB_VID_STELLARIS;
    priv->usbdcdcDevice.usPID = USB_PID_SERIAL;
    priv->usbdcdcDevice.usMaxPowermA = 0;
    priv->usbdcdcDevice.ucPwrAttributes = USB_CONF_ATTR_SELF_PWR;
    priv->usbdcdcDevice.pfnControlCallback = control_callback;
    priv->usbdcdcDevice.pvControlCBData = dev;
    priv->usbdcdcDevice.pfnRxCallback = rx_callback;
    priv->usbdcdcDevice.pvRxCBData = dev;
    priv->usbdcdcDevice.pfnTxCallback = tx_callback;
    priv->usbdcdcDevice.pvTxCBData = dev;
    priv->usbdcdcDevice.ppStringDescriptors = stringDescriptors;
    priv->usbdcdcDevice.ulNumStringDescriptors = NUM_STRING_DESCRIPTORS;
    priv->usbdcdcDevice.psPrivateCDCSerData = &priv->serialInstance;
    priv->connected = 0;
    priv->enabled = 0;
    
    priv->serialPriv.enable = stellaris_cdc_enable;
    priv->serialPriv.disable = stellaris_cdc_disable;
    priv->serialPriv.tx_char = stellaris_cdc_tx_char;
    
    int result = serial_init(dev);
    
    USBStackModeSet(0, USB_MODE_DEVICE, 0);
    USBDCDCInit(0, &priv->usbdcdcDevice);
    
    return result;
}

/** Enable use of the device interrupts.
 * @param dev device to enable
 */
static void stellaris_cdc_enable(devtab_t *dev)
{
    StellarisCdcPriv *priv = dev->priv;
    priv->enabled = 1;
}

/** Disable use of the device interrupts.
 * @param dev device to disable
 */
static void stellaris_cdc_disable(devtab_t *dev)
{
    StellarisCdcPriv *priv = dev->priv;
    priv->enabled = 0;
}

/* Try and transmit a message.
 * @param dev device to transmit message on
 */
static void stellaris_cdc_tx_char(devtab_t *dev)
{
    StellarisCdcPriv *priv = dev->priv;
    
    #define CDC_SERIAL_STATE  USB_CDC_SERIAL_STATE_TXCARRIER | \
                              USB_CDC_SERIAL_STATE_RXCARRIER
    
    MAP_IntDisable(INT_USB0);
    if (priv->connected)
    {
        unsigned long available;
        unsigned long count;

        /* we don't do handshakes, data is always available */
        USBDCDCSerialStateChange(&priv->usbdcdcDevice, CDC_SERIAL_STATE);

        /* do this if we have data to send */
        available = USBDCDCTxPacketAvailable(&priv->usbdcdcDevice);
        
        for (count = 0; count < TX_DATA_SIZE && count < available; count++)
        {
            if (os_mq_timedreceive(priv->serialPriv.txQ, &priv->txData[count], 0) != OS_MQ_NONE)
            {
                /* no more data left to transmit */
                break;
            }
        }
        if (count)
        {
            /* we have some data to send */
            USBDCDCPacketWrite(&priv->usbdcdcDevice, priv->txData, count, 1);
        }
        MAP_IntEnable(INT_USB0);
    }
    else
    {
        MAP_IntEnable(INT_USB0);
        /* we are not connected, flush away the data */
        int result;
        do
        {
            unsigned char data;
            result = os_mq_timedreceive(priv->serialPriv.txQ, &data, 0);
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
static unsigned long control_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    devtab_t *dev = data;
    StellarisCdcPriv *priv = dev->priv;
    switch(event)
    {
        case USB_EVENT_CONNECTED:
            priv->connected = 1;
            break;
        case USB_EVENT_DISCONNECTED:
            priv->connected = 0;
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

static unsigned long rx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    devtab_t *dev = data;
    StellarisCdcPriv *priv = dev->priv;

    switch (event)
    {
        default:
            break;
        case USB_EVENT_RX_AVAILABLE:
        {
            if (msg_data)
            {
                unsigned char *data = msg_data;
                unsigned long count;
                if (priv->enabled)
                {
                    for (count = 0; count < msg_param; count++, data++)
                    {
                        if (os_mq_send_from_isr(priv->serialPriv.rxQ, data) != OS_MQ_NONE)
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
                
                //available = USBDCDCRxPacketAvailable(&priv->usbdcdcDevice);
                space = SERIAL_RX_BUFFER_SIZE - os_mq_num_pending_from_isr(priv->serialPriv.rxQ);

                count = USBDCDCPacketRead(&priv->usbdcdcDevice,
                                            priv->rxData,
                                            space,
                                            true);
                if (priv->enabled)
                {
                    /* transfer data up */
                    for (unsigned long i; i < count; i++)
                    {
                        os_mq_send_from_isr(priv->serialPriv.rxQ, &priv->rxData[i]);
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

static unsigned long tx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data)
{
    devtab_t *dev = data;
    StellarisCdcPriv *priv = dev->priv;
    
    switch (event)
    {
        default:
            break;
        case USB_EVENT_TX_COMPLETE:
        {
            unsigned long available;
            unsigned long count;

            /* do this if we have data to send */
            available = USBDCDCTxPacketAvailable(&priv->usbdcdcDevice);
            
            for (count = 0; count < TX_DATA_SIZE && count < available; count++)
            {
                if (os_mq_receive_from_isr(priv->serialPriv.txQ, &priv->txData[count]) != OS_MQ_NONE)
                {
                    /* no more data left to transmit */
                    break;
                }
            }
            if (count)
            {
                /* we have some data to send */
                USBDCDCPacketWrite(&priv->usbdcdcDevice, priv->txData, count, 1);
            }
            break;
        }
    }
    return 0;
}

/** Handle interrupts for USB0.
 */
void usb0_interrupt_handler(void)
{
    USB0DeviceIntHandler();
}
