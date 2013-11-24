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
 * \file StellarisDev.cxx
 * This file implements the device class prototypes for Stellaris.
 *
 * @author Stuart W. Baker
 * @date 31 October 2013
 */

#ifndef _StellarisDev_hxx_
#define _StellarisDev_hxx_

#ifndef gcc
#define gcc
#endif

#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"

#include "Serial.hxx"
#include "Can.hxx"

#define TX_DATA_SIZE 64
#define RX_DATA_SIZE 64

/** Private data for this implementation of serial.
 */
class StellarisCdc : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     */
    StellarisCdc(const char *name);
    
    /** Destructor.
     */
    ~StellarisCdc()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_char(); /**< function to try and transmit a character */

    static unsigned long control_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    static unsigned long rx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    static unsigned long tx_callback(void *data, unsigned long event, unsigned long msg_param, void *msg_data);

    tUSBDCDCDevice usbdcdcDevice; /**< CDC serial device instance */
    tCDCSerInstance serialInstance; /**< CDC serial device private data */
    unsigned char txData[TX_DATA_SIZE]; /**< buffer for pending tx data */
    unsigned char rxData[RX_DATA_SIZE]; /**< buffer for pending tx data */
    bool connected; /**< connection status */
    bool enabled; /**< enabled status */
    int woken; /**< task woken metadata for ISR */
    
    /** Default constructor.
     */
    StellarisCdc();
    
    DISALLOW_COPY_AND_ASSIGN(StellarisCdc);
};

/** Specialization of Serial driver for Stellaris UART.
 */
class StellarisUart : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     */
    StellarisUart(const char *name, unsigned long base);
    
    /** Destructor.
     */
    ~StellarisUart()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_char(); /**< function to try and transmit a character */

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    bool txPending; /**< transmission currently pending */

    /** Default constructor.
     */
    StellarisUart();
    
    DISALLOW_COPY_AND_ASSIGN(StellarisUart);
};

/** Specialization of CAN driver for Stellaris CAN.
 */
class StellarisCan : public Can
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     */
    StellarisCan(const char *name, unsigned long base);
    
    /** Destructor.
     */
    ~StellarisCan()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */
    void tx_msg(); /**< function to try and transmit a message */

    unsigned long base; /**< base address of this device */
    unsigned long interrupt; /**< interrupt of this device */
    uint8_t data[8]; /**< transmit data */
    bool txPending; /**< transmission currently pending */

    /** Default constructor.
     */
    StellarisCan();
    
    DISALLOW_COPY_AND_ASSIGN(StellarisCan);
};

#endif /* _StellarisDev_hxx_ */

