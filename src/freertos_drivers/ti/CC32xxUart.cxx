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
#include "driverlib/interrupt.h"
#include "driverlib/prcm.h"
#include "driverlib/utils.h"
#include "freertos/tc_ioctl.h"

#include "CC32xxUart.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static CC32xxUart *instances[2] = {NULL};

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
CC32xxUart::CC32xxUart(const char *name, unsigned long base, uint32_t interrupt,
                       uint32_t baud, uint32_t mode, bool hw_fifo,
                       TxEnableMethod tx_enable_assert,
                       TxEnableMethod tx_enable_deassert)
    : Serial(name)
    , txEnableAssert(tx_enable_assert)
    , txEnableDeassert(tx_enable_deassert)
    , base(base)
    , interrupt(interrupt)
    , txPending(false)
    , hwFIFO(hw_fifo)
{
    
    switch (base)
    {
        default:
            HASSERT(0);
        case UARTA0_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
            instances[0] = this;
            break;
        case UARTA1_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
            instances[1] = this;
            break;
    }
    
    MAP_UARTConfigSetExpClk(base, cm3_cpu_clock_hz, baud,
                            mode | UART_CONFIG_PAR_NONE);
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
    MAP_IntEnable(interrupt);
    MAP_UARTEnable(base);
    if (hwFIFO)
    {
        MAP_UARTFIFOEnable(base);
    }
    else
    {
        MAP_UARTFIFODisable(base);
    }
}

/** Disable use of the device.
 */
void CC32xxUart::disable()
{
    MAP_IntDisable(interrupt);
    MAP_UARTDisable(base);
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param key ioctl key
 * @param data key data
 * @return 0 upon success, -errno upon failure
 */
int CC32xxUart::ioctl(File *file, unsigned long int key, unsigned long data)
{
    switch (key)
    {
        default:
            return -EINVAL;
        case TCSBRK:
            MAP_UARTBreakCtl(base, true);
            // need to wait at least two frames here
            MAP_UtilsDelay(100 * 26);
            MAP_UARTBreakCtl(base, false);
            MAP_UtilsDelay(12 * 26);
            break;
    }

    return 0;
}

/** Send data until there is no more space left.
 */
void CC32xxUart::send()
{
    do
    {
        uint8_t data = 0;
        if (txBuf->get(&data, 1))
        {
            MAP_UARTCharPutNonBlocking(base, data);

        }
        else
        {
            break;
        }
    }
    while (MAP_UARTSpaceAvail(base));

    if (txBuf->pending())
    {
        /* more data to send later */
        MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_FIFO);
    }
    else
    {
        /* no more data left to send */
        MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_EOT);
        MAP_UARTIntClear(base, UART_INT_TX);
    }
}

/** Try and transmit a message.
 */
void CC32xxUart::tx_char()
{
    if (txPending == false)
    {
        if (txEnableAssert)
        {
            txEnableAssert();
        }

        send();
        txPending = true;

        MAP_UARTIntEnable(base, UART_INT_TX);
        txBuf->signal_condition();
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
                ++overrunCount;
            }
            rxBuf->signal_condition_from_isr();
        }
    }
    /* transmit a character if we have pending tx data */
    if (txPending && (status & UART_INT_TX))
    {
        if (txBuf->pending())
        {
            send();
            txBuf->signal_condition_from_isr();
        }
        else
        {
            /* no more data left to send */
            HASSERT(MAP_UARTTxIntModeGet(base) == UART_TXINT_MODE_EOT);
            if (txEnableDeassert)
            {
                txEnableDeassert();
            }
            txPending = false;
            MAP_UARTIntDisable(base, UART_INT_TX);
        }
    }
    os_isr_exit_yield_test(woken);
}

extern volatile unsigned current_interrupt;

class SetInterrupt {
public:
    SetInterrupt(unsigned new_value) {
        old_value = current_interrupt;
        current_interrupt = new_value;
    }

    ~SetInterrupt() {
        current_interrupt = old_value;
    }

private:
    unsigned old_value;
};




extern "C" {
/** UART0 interrupt handler.
 */
void uart0_interrupt_handler(void)
{
    SetInterrupt si(9);
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

/** UART1 interrupt handler.
 */
void __attribute__((__weak__)) uart1_interrupt_handler(void)
{
    SetInterrupt si(10);
    if (instances[1])
    {
        instances[1]->interrupt_handler();
    }
}

} // extern C
