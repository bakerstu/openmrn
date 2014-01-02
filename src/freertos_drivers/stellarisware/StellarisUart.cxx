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
 * \file StellarisUart.cxx
 * This file implements a USB CDC device driver layer specific to stellarisware.
 *
 * @author Stuart W. Baker
 * @date 11 January 2013
 */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "StellarisDev.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static StellarisUart *instances[8] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 */
StellarisUart::StellarisUart(const char *name, unsigned long base)
    : Serial(name),
      base(base),
      interrupt(0),
      txPending(false)
{
    
    switch (base)
    {
        default:
            HASSERT(0);
        case UART0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
            interrupt = INT_UART0;
            instances[0] = this;
            break;
        case UART1_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
            interrupt = INT_UART1;
            instances[1] = this;
            break;
        case UART2_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
            interrupt = INT_UART2;
            instances[2] = this;
            break;
        case UART3_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
            interrupt = INT_UART3;
            instances[3] = this;
            break;
        case UART4_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
            interrupt = INT_UART4;
            instances[4] = this;
            break;
        case UART5_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
            interrupt = INT_UART5;
            instances[5] = this;
            break;
        case UART6_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);
            interrupt = INT_UART6;
            instances[6] = this;
            break;
        case UART7_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
            interrupt = INT_UART7;
            instances[7] = this;
            break;
    }
    
    MAP_UARTConfigSetExpClk(base, MAP_SysCtlClockGet(), 115200,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
    MAP_UARTFIFOEnable(base);
    MAP_UARTTxIntModeSet(base, UART_TXINT_MODE_EOT);
    MAP_IntEnable(interrupt);
    MAP_UARTIntEnable(base, UART_INT_RX | UART_INT_RT);
}

/** Enable use of the device.
 * @param dev device to enable
 */
void StellarisUart::enable()
{
    MAP_IntEnable(interrupt);
    MAP_UARTEnable(base);
}

/** Disable use of the device.
 */
void StellarisUart::disable()
{
    MAP_IntDisable(interrupt);
    MAP_UARTDisable(base);
}

/* Try and transmit a message.
 */
void StellarisUart::tx_char()
{
    if (txPending == false)
    {
        unsigned char data;
        if (MAP_UARTSpaceAvail(base) &&
            os_mq_timedreceive(txQ, &data, 0) == OS_MQ_NONE)
        {
            MAP_IntDisable(interrupt);
            txPending = true;
            MAP_UARTCharPutNonBlocking(base, data);
            MAP_UARTIntEnable(base, UART_INT_TX);
        }
    }
}

/** Common interrupt handler for all UART devices.
 */
void StellarisUart::interrupt_handler()
{
    int woken = false;
    /* get and clear the interrupt status */
    unsigned long status = MAP_UARTIntStatus(base, true);    
    MAP_UARTIntClear(base, status);

    /* receive charaters as long as we can */
    while (MAP_UARTCharsAvail(base))
    {
        long data = MAP_UARTCharGetNonBlocking(base);
        if (data >= 0)
        {
            unsigned char c = data;
            if (os_mq_send_from_isr(rxQ, &c, &woken) == OS_MQ_FULL)
            {
                overrunCount++;
            }
        }
    }
    /* tranmit a character if we have pending tx data */
    if (txPending)
    {
        if (MAP_UARTSpaceAvail(base))
        {
            unsigned char data;
            if (os_mq_receive_from_isr(txQ, &data, &woken) == OS_MQ_NONE)
            {
                MAP_UARTCharPutNonBlocking(base, data);
            }
            else
            {
                /* no more data pending */
                txPending = false;
                MAP_UARTIntDisable(base, UART_INT_TX);
            }
        }
    }
    os_isr_exit_yield_test(woken);
}

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
void uart1_interrupt_handler(void)
{
    if (instances[1])
    {
        instances[1]->interrupt_handler();
    }
}

/** UART2 interrupt handler.
 */
void uart2_interrupt_handler(void)
{
    if (instances[2])
    {
        instances[2]->interrupt_handler();
    }
}

/** UART3 interrupt handler.
 */
void uart3_interrupt_handler(void)
{
    if (instances[3])
    {
        instances[3]->interrupt_handler();
    }
}
/** UART4 interrupt handler.
 */
void uart4_interrupt_handler(void)
{
    if (instances[4])
    {
        instances[4]->interrupt_handler();
    }
}

/** UART5 interrupt handler.
 */
void uart5_interrupt_handler(void)
{
    if (instances[5])
    {
        instances[5]->interrupt_handler();
    }
}

/** UART6 interrupt handler.
 */
void uart6_interrupt_handler(void)
{
    if (instances[6])
    {
        instances[6]->interrupt_handler();
    }
}

/** UART7 interrupt handler.
 */
void uart7_interrupt_handler(void)
{
    if (instances[7])
    {
        instances[7]->interrupt_handler();
    }
}

