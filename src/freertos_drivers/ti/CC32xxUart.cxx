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
 * \file CC32xxUart.cxx
 * This file implements the UART class prototype for CC32xx.
 *
 * @author Stuart W. Baker
 * @date 15 March 2016
 */

#include <algorithm>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/prcm.h"

#include "CC32xxUart.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static CC32xxUart *instances[2] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 * @param tx_enable_assert callback to assert the transmit enable
 * @param tx_enable_deassert callback to deassert the transmit enable
 */
CC32xxUart::CC32xxUart(const char *name, unsigned long base, uint32_t interrupt,
                       TxEnableMethod tx_enable_assert,
                       TxEnableMethod tx_enable_deassert)
    : Serial(name)
    , txEnableAssert(tx_enable_assert)
    , txEnableDeassert(tx_enable_deassert)
    , base(base)
    , interrupt(interrupt)
    , txPending(false)
{
    
    switch (base)
    {
        default:
            HASSERT(0);
        case UARTA0_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
            //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
            instances[0] = this;
            break;
        case UARTA1_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
            //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
            instances[1] = this;
            break;
    }
    
    /* We set the preliminary clock here, but it will be re-set when the device
     * gets enabled. The reason for re-setting is that the system clock is
     * switched in HwInit but that has not run yet at this point. */
    MAP_UARTConfigSetExpClk(base, cm3_cpu_clock_hz, 115200,
                            UART_CONFIG_WLEN_8 |
                            UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(base);
    MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_EOT);
    MAP_IntDisable(interrupt);
    /* We set the priority so that it is slightly lower than the highest needed
     * for FreeRTOS compatibility. This will ensure that CAN interrupts take
     * precedence over UART. */
    MAP_IntPrioritySet(interrupt,
                       std::min(0xff, configKERNEL_INTERRUPT_PRIORITY + 0x20));
    MAP_UARTIntEnable(base, UART_INT_RX | UART_INT_RT);
}

/** Enable use of the device.
 */
void CC32xxUart::enable()
{
    MAP_UARTConfigSetExpClk(base, cm3_cpu_clock_hz, 115200,
                            UART_CONFIG_WLEN_8 |
                            UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);
    MAP_IntEnable(interrupt);
    MAP_UARTEnable(base);
    MAP_UARTFIFOEnable(base);
}

/** Disable use of the device.
 */
void CC32xxUart::disable()
{
    MAP_IntDisable(interrupt);
    MAP_UARTDisable(base);
}

/** Try and transmit a message.
 */
void CC32xxUart::tx_char()
{
    if (txPending == false)
    {
        uint8_t data = 0;

        if (txBuf->get(&data, 1))
        {
            if (txEnableAssert)
            {
                txEnableAssert();
            }
            MAP_UARTCharPutNonBlocking(base, data);

            MAP_IntDisable(interrupt);
            txPending = true;
            MAP_UARTIntEnable(base, UART_INT_TX);
            MAP_IntEnable(interrupt);
            txBuf->signal_condition();
        }
    }
}

/** Common interrupt handler for all UART devices.
 */
void CC32xxUart::interrupt_handler()
{
    int woken = false;
    /* get and clear the interrupt status */
    unsigned long status = MAP_UARTIntStatus(base, true);    
    MAP_UARTIntClear(base, status);

    /** @todo (Stuart Baker) optimization opportunity by getting a write
     * pointer to fill the fifo and then advance the buffer when finished
     */
    /* receive charaters as long as we can */
    while (MAP_UARTCharsAvail(base))
    {
        long data = MAP_UARTCharGetNonBlocking(base);
        if (data >= 0 && data <= 0xff)
        {
            unsigned char c = data;
            if (rxBuf->put(&c, 1) == 0)
            {
                overrunCount++;
            }
            rxBuf->signal_condition_from_isr();
        }
    }
    /* transmit a character if we have pending tx data */
    if (txPending)
    {
        while (MAP_UARTSpaceAvail(base))
        {
            if (MAP_UARTTxIntModeGet(base) == UART_TXINT_MODE_EOT)
            {
                MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_FIFO);
                if (txBuf->pending() == 0)
                {
                    txEnableDeassert();
                    txPending = false;
                    MAP_UARTIntDisable(base, UART_INT_TX);
                    break;
                }
            }

            unsigned char data;
            if (txBuf->get(&data, 1) != 0)
            {
                MAP_UARTCharPutNonBlocking(base, data);
                txBuf->signal_condition_from_isr();
            }
            else
            {
                /* no more data pending */
                if (txEnableDeassert)
                {
                    MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_EOT);
                }
                else
                {
                    txPending = false;
                    MAP_UARTIntDisable(base, UART_INT_TX);
                }
                break;
            }
        }
    }
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** UART0 interrupt handler.
 */
void uart0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

/** UART1 interrupt handler.
 */
void __attribute__((__weak__)) uart1_interrupt_handler(void)
{
    if (instances[1])
    {
        instances[1]->interrupt_handler();
    }
}

} // extern C
