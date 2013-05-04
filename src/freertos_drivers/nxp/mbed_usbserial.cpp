/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file mbed_usbserial.cpp
 * This file implements an USB-Serial driver on top of the mbed
 * library. Currently it is tested on the LPC23xx processors.
 *
 * @author Balazs Racz
 * @date 4 May 2013
 */

#include "mbed.h"
#include "USBSerial.h"
#include "serial.h"

#ifdef TARGET_LPC2368
#endif

#define TX_DATA_SIZE 64
#define RX_DATA_SIZE 64

/** Private data for this implementation of Serial port
 */
class MbedSerialPriv
{
public:
    MbedSerialPriv() : txPending(false), bytesLost(0) {}
    SerialPriv serialPriv; /**< common private data */
    bool txPending; /**< transmission currently pending */
    int bytesLost; /**< counts the number of bytes droipped due to error or buffer overruns. */
    USBSerial serial; /*< mbed USB implementation object */
    unsigned char txData[TX_DATA_SIZE]; /**< buffer for pending tx data */
    void RxCallback();
    void TxCallback();
    /** Transmits count bytes from the txData buffer. Sets txPending and bytesLost as needed. */
    void TxHelper(int count);
};

/** private data for the can device */
static MbedSerialPriv serial_private[1];

static int mbed_usbserial_init(devtab_t *dev);
static void ignore_dev_function(devtab_t *dev);
static void mbed_usbserial_tx_msg(devtab_t *dev);

/** initialize the device 
 * @param dev device to initialize
 * @return 0 upon success
 */
static int mbed_usbserial_init(devtab_t *dev)
{
    MbedSerialPriv *priv = (MbedSerialPriv*)dev->priv;
    
    priv->serialPriv.enable = ignore_dev_function;
    priv->serialPriv.disable = ignore_dev_function;
    priv->serialPriv.tx_char = mbed_usbserial_tx_msg;
    priv->serial.attach(priv, &MbedSerialPriv::RxCallback);
    priv->serial.txattach(priv, &MbedSerialPriv::TxCallback);
    return serial_init(dev);
}

/** Empty device function. Does nothing.
 * @param dev device
 */
static void ignore_dev_function(devtab_t *dev) {}

/** Try and transmit a message. Does nothing if there is no message to transmit
 *  or no write buffers to transmit via.
 * @param dev device to transmit message on
 */
static void mbed_usbserial_tx_msg(devtab_t *dev)
{
    MbedSerialPriv *priv = (MbedSerialPriv*)dev->priv;
    if (priv->txPending) return;
    priv->txPending = true;
    // At this point we know there is no outstading send, thus there can't be
    // an incoming TX interrupt either. Thus we don't need a critical section.
    int count;
    for (count = 0; count < TX_DATA_SIZE; count++)
    {
	if (os_mq_timedreceive(priv->serialPriv.txQ, &priv->txData[count], 0) != OS_MQ_NONE)
        {
	    /* no more data left to transmit */
	    break;
	}
    }
    priv->TxHelper(count);
}

void MbedSerialPriv::TxHelper(int count)
{
    if (!count) {
	txPending = false;
	return;
    }
    if (!serial.writeBlockAsync(txData, count)) {
	// An error occured, data was lost.
	txPending = false;
	bytesLost += count;
	return;
    }
}

/** Called by the USB driver in an ISR context when a pending transfer finished. */
void MbedSerialPriv::TxCallback()
{
    int count;
    for (count = 0; count < TX_DATA_SIZE; count++)
    {
	if (os_mq_receive_from_isr(serialPriv.txQ, &txData[count]) != OS_MQ_NONE)
        {
	    /* no more data left to transmit */
	    break;
	}
    }
    TxHelper(count);
}

/** Called by the USB driver in an ISR context when there is data received to read. */
void MbedSerialPriv::RxCallback()
{
    while (!os_mq_is_full_from_isr(serialPriv.rxQ) &&
	   serial.available()) {
	unsigned char c = serial.getc();
	if (os_mq_send_from_isr(serialPriv.rxQ, &c) != OS_MQ_NONE)
	{
	    ++bytesLost;
	    /* no more room left? we just made sure the queue is not full */
	    return;
	}
    }
}

/** Device table entry for serial device */
static SERIAL_DEVTAB_ENTRY(serUSB0, "/dev/serUSB0", mbed_usbserial_init, &serial_private[0]);
