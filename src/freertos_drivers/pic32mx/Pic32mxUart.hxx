/** \copyright
 * Copyright (c) 2018, Balazss Racz
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
 * \file Pic32mxUart.hxx
 *
 * This file implements a UART device driver layer for the Pic32MX
 * microcontroller range.
 *
 * @author Balazs Racz
 * @date 7 Oct 2018
 */

#ifndef _FREERTOS_DRIVERS_PIC32MX_PIC32MXUART_HXX_
#define _FREERTOS_DRIVERS_PIC32MX_PIC32MXUART_HXX_

#include "Serial.hxx"
#include "peripheral/uart.h"

extern "C"
{
    extern const unsigned long pic32_periph_clock_hz;
}

class Pic32mxUart : public Serial
{
public:
    Pic32mxUart(const char *name, UART_MODULE periph, uint32_t int_vec,
        unsigned baud = 115200, bool enable_rts_cts = false)
        : Serial(name)
        , hw_(periph)
        , intVector_(int_vec)
    {
        unsigned rtscfg = UART_ENABLE_PINS_TX_RX_ONLY;
        if (enable_rts_cts)
        {
            rtscfg = UART_ENABLE_PINS_CTS_RTS | UART_RTS_WHEN_RX_NOT_FULL;
        }
        UARTConfigure(
            hw_, (UART_CONFIGURATION)(UART_ENABLE_HIGH_SPEED | rtscfg));
        UARTSetFifoMode(hw_,
            (UART_FIFO_MODE)(UART_INTERRUPT_ON_TX_NOT_FULL |
                UART_INTERRUPT_ON_RX_NOT_EMPTY));
        UARTSetLineControl(hw_,
            (UART_LINE_CONTROL_MODE)(
                UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1));
        UARTSetDataRate(hw_, get_pclk(), baud);

        INTSetVectorPriority(int_vector(), INT_PRIORITY_LEVEL_3);
        INTSetVectorSubPriority(int_vector(), INT_SUB_PRIORITY_LEVEL_0);
    }

    void enable() override
    {
        UARTEnable(hw_,
            (UART_ENABLE_MODE)(
                UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX)));

        // Configure UART2 RX Interrupt
        INTEnable(rx_int(), INT_ENABLED);
    }

    void disable() override
    {
        // We do not disable TX as we might still have some unsent data in our
        // buffer.
        UARTEnable(hw_, (UART_ENABLE_MODE)(UART_DISABLE_FLAGS(UART_RX)));
        INTEnable(rx_int(), INT_DISABLED);
    }

    inline void interrupt_handler()
    {
        if (INTGetFlag(rx_int()))
        {
            numIrqRx_++;
            if (receive_some_data())
            {
                rxBuf->signal_condition_from_isr();
            }
            INTClearFlag(rx_int());
        }
        if (INTGetFlag(tx_int()) && INTGetEnable(tx_int()))
        {
            numIrqTx_++;
            if (send_some_data())
            {
                txBuf->signal_condition_from_isr();
            }
            INTClearFlag(tx_int());
        }
        numIrq_++;
    }

    void tx_char() override
    {
        if (send_some_data())
        {
            txBuf->signal_condition();
        }
    }

private:
    /// Copies data from the RX fifo into the receive ring buffer. Must be
    /// called from critical section or interrupt. Throws away data if the
    /// receive buffer is full (marking overrunCount).
    /// @return true is some data showed up.
    bool receive_some_data()
    {
        bool has = false;
        while (UARTReceivedDataIsAvailable(hw_))
        {
            auto d = UARTGetDataByte(hw_);
            if (rxBuf->put(&d, 1) == 0)
            {
                ++overrunCount;
            }
            has = true;
        }
        return has;
    }

    /// Copies data from the tx buffer to the hardware send register /
    /// fifo. Updates the TX interrupt enable flag depending on why we stopped.
    /// Must be called from a critical section or interrupt.
    /// @return true if some data was sent.
    bool send_some_data()
    {
        bool has = false;
        while (true)
        {
            if (!txBuf->pending())
            {
                // Nothing to send. Do not wait for interrupts.
                INTEnable(tx_int(), INT_DISABLED);
                return has;
            }
            if (!UARTTransmitterIsReady(hw_))
            {
                // FIFO full. Wait with interrupt.
                INTEnable(tx_int(), INT_ENABLED);
                return has;
            }
            uint8_t d;
            txBuf->get(&d, 1);
            UARTSendDataByte(hw_, d);
            has = true;
        }
    }

    inline unsigned get_pclk()
    {
        return pic32_periph_clock_hz;
    }

    inline INT_SOURCE tx_int()
    {
        return (INT_SOURCE)INT_SOURCE_UART_TX(hw_);
    }

    inline INT_SOURCE rx_int()
    {
        return (INT_SOURCE)INT_SOURCE_UART_RX(hw_);
    }

    inline INT_VECTOR int_vector()
    {
        return (INT_VECTOR)(INT_VECTOR_UART(hw_));
    }

    /// Enum describing which uart module we are using. We need to pass this to
    /// the middleware calls.
    UART_MODULE hw_;
    /// This is the hardware vector number for the UART module. That is NOT the
    /// same as the INT_VECTOR enum.
    unsigned intVector_;
    /// Stats variable.
    unsigned numIrq_ = 0;
    /// Stats variable.
    unsigned numIrqTx_ = 0;
    /// Stats variable.
    unsigned numIrqRx_ = 0;
};

#endif // _FREERTOS_DRIVERS_PIC32MX_PIC32MXUART_HXX_
