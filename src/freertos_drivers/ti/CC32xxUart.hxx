/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxUart.hxx
 * This file implements the UART class prototype for CC32xx.
 *
 * @author Stuart W. Baker
 * @date 15 March 2016
 */

#ifndef _FREERTOS_DRIVERS_TI_CC32XXUART_HXX_
#define _FREERTOS_DRIVERS_TI_CC32XXUART_HXX_

#ifndef gcc
#define gcc
#endif

#include <cstdint>

#include "Serial.hxx"

#include "driverlib/uart.h"

class Notifiable;

/** Specialization of Serial driver for CC32xx UART.
 */
class CC32xxUart : public Serial
{
public:
    enum Mode
    {
        CS5 = UART_CONFIG_WLEN_5, /**< 5-bits word length */
        CS6 = UART_CONFIG_WLEN_6, /**< 6-bits word length */
        CS7 = UART_CONFIG_WLEN_7, /**< 7-bits word length */
        CS8 = UART_CONFIG_WLEN_8, /**< 8-bits word length */
        CSTOPB = UART_CONFIG_STOP_TWO, /**< send two stop bits instead of 1 */
    };

    /** Function point for the tx enable assert and deassert methods */
    typedef void (*TxEnableMethod)();

    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     * @param baud desired baud rate
     * @param mode to configure the UART for
     * @param hw_fifo true if hardware fifo is to be enabled, else false.
     * @param tx_enable_assert callback to assert the transmit enable
     * @param tx_enable_deassert callback to deassert the transmit enable
     */
    CC32xxUart(const char *name, unsigned long base, uint32_t interrupt,
               uint32_t baud = 115200, uint32_t mode = CS8, bool hw_fifo = true,
               TxEnableMethod tx_enable_assert = nullptr,
               TxEnableMethod tx_enable_deassert = nullptr);

    /** Destructor.
     */
    ~CC32xxUart()
    {
    }

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

    /** Request an ioctl transaction. Supported ioctl is TCSBRK, TCDRAINNOTIFY,
     * TCSTOP*, TCBAUDRATE and TCPAR* from include/freertos/tc_ioctl.h */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */


    /** Try and transmit a message.
     */
    void tx_char() override;

    /** Send data until there is no more space left.
     */
    void send();

    /** Sets the port baud rate and mode from the class variables. */
    void set_mode();
    
    /** function pointer to a method that asserts the transmit enable. */
    TxEnableMethod txEnableAssert_;

    /** function pointer to a method that deasserts the transmit enable. */
    TxEnableMethod txEnableDeassert_;

    /** Notifiable to invoke when the transmit engine has finished operation. */
    Notifiable* txComplete_{nullptr};
    
    unsigned long base_; /**< base address of this device */
    uint32_t interrupt_ : 8; /**< interrupt of this device */
    uint32_t baud_ : 24; /**< desired baud rate */
    uint32_t uartMode_; /**< mode of the UART, 8 or 9 bit, 1 or 2 stop... */
    uint8_t txPending_; /**< transmission currently pending */
    uint8_t hwFIFO_; /**< true if hardware fifo is to be enabled, else false */
    uint8_t nineBit_; /**< true if using 9-bit reception */

    /** Default constructor.
     */
    CC32xxUart();

    DISALLOW_COPY_AND_ASSIGN(CC32xxUart);
};


#endif /* _FREERTOS_DRIVERS_TI_CC32XXUART_HXX_ */
