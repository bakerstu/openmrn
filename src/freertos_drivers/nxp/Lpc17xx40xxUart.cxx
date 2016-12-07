/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Lpc17xx40xxUart.cxx
 * This file implements a UART device driver layer specific to LPC17xx and
 * LPC40xx devices.
 *
 * @author Stuart W. Baker
 * @date 9 April 2015
 */

#include "Lpc17xx40xxUart.hxx"

#include <algorithm>

#if  defined (CHIP_LPC177X_8X) || defined (CHIP_LPC407X_8X)
LpcUart *LpcUart::instances[5] = {NULL};
#else
LpcUart *LpcUart::instances[4] = {NULL};
#endif

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 */
LpcUart::LpcUart(const char *name, LPC_USART_T *base, IRQn_Type interrupt)
    : Serial(name)
    , base(base)
    , interrupt(interrupt)
{
    switch (interrupt)
    {
        default:
            HASSERT(0);
        case UART0_IRQn:
            instances[0] = this;
            break;
        case UART1_IRQn:
            instances[1] = this;
            break;
        case UART2_IRQn:
            instances[2] = this;
            break;
        case UART3_IRQn:
            instances[3] = this;
            break;
#if  defined (CHIP_LPC177X_8X) || defined (CHIP_LPC407X_8X)
        case UART4_IRQn:
            instances[4] = this;
            break;
#endif
    }

    /* should already be disabled, but just in case */
    NVIC_DisableIRQ(interrupt);
    /* We set the priority so that it is slightly lower than the highest needed
     * for FreeRTOS compatibility. This will ensure that CAN interrupts take
     * precedence over UART. */
    NVIC_SetPriority(interrupt,
                     std::min(0xff, configKERNEL_INTERRUPT_PRIORITY + 0x20));
}

/** Enable use of the device.
 */
void LpcUart::enable()
{
    Chip_UART_Init(base);
    Chip_UART_SetBaud(base, 115200);
    Chip_UART_ConfigData(base, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(base, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
                                UART_FCR_TX_RS | UART_FCR_TRG_LEV0));
    Chip_UART_TXEnable(base);
    Chip_UART_IntEnable(base, (UART_IER_RBRINT | UART_IER_RLSINT));
    NVIC_EnableIRQ(interrupt);
}

/** Disable use of the device.
 */
void LpcUart::disable()
{
    NVIC_DisableIRQ(interrupt);
    Chip_UART_IntDisable(base, (UART_IER_RBRINT | UART_IER_RLSINT));
    Chip_UART_TXDisable(base);
    Chip_UART_DeInit(base);
}

/** Try and transmit a message.
 */
void LpcUart::tx_char()
{
    if ((Chip_UART_GetIntsEnabled(base) & UART_IER_THREINT) == 0)
    {
        uint8_t data = 0;
        if (txBuf->get(&data, 1))
        {
            Chip_UART_SendByte(base, (uint8_t)data);
            Chip_UART_IntEnable(base, UART_IER_THREINT);
            txBuf->signal_condition();
        }
    }
}

/** Common interrupt handler for all UART devices.
 */
void LpcUart::interrupt_handler()
{
    /** @todo (Stuart Baker) optimization opportunity by getting a write
     * pointer to fill the fifo and then advance the buffer when finished
     */
    /* receive charaters as long as we can */
    while (Chip_UART_ReadLineStatus(base) & UART_LSR_RDR)
    {
        uint8_t data = Chip_UART_ReadByte(base);
        if (rxBuf->put(&data, 1) == 0)
        {
            overrunCount++;
        }
        rxBuf->signal_condition_from_isr();
    }

    /* tranmit a character if we have pending tx data */
    if (Chip_UART_GetIntsEnabled(base) & UART_IER_THREINT)
    {
        /** @todo (Stuart Baker) optimization opportunity by getting a read
         * pointer to fill the fifo and then consume the buffer when finished.
         */
        while (Chip_UART_ReadLineStatus(base) & UART_LSR_THRE)
        {
            uint8_t data;
            if (txBuf->get(&data, 1) != 0)
            {
                Chip_UART_SendByte(base, (uint8_t)data);
                txBuf->signal_condition_from_isr();
            }
            else
            {
                /* no more data pending */
                Chip_UART_IntDisable(base, UART_IER_THREINT);
                break;
            }
        }
    }
}

extern "C" {
/** UART0 interrupt handler.
 */
void uart0_interrupt_handler(void)
{
    LpcUart::interrupt_handler(0);
}

/** UART1 interrupt handler.
 */
void uart1_interrupt_handler(void)
{
    LpcUart::interrupt_handler(1);
}

/** UART2 interrupt handler.
 */
void uart2_interrupt_handler(void)
{
    LpcUart::interrupt_handler(2);
}

/** UART3 interrupt handler.
 */
void uart3_interrupt_handler(void)
{
    LpcUart::interrupt_handler(3);
}

#if  defined (CHIP_LPC177X_8X) || defined (CHIP_LPC407X_8X)
/** UART4 interrupt handler.
 */
void uart4_interrupt_handler(void)
{
    LpcUart::interrupt_handler(4);
}
#endif

} // extern C
