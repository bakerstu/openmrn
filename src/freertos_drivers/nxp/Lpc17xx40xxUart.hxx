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
 * \file Lpc17xx40xxUart.hxx
 * This file implements a UART device driver layer specific to LPC17xx and
 * LPC40xx devices.
 *
 * @author Stuart W. Baker
 * @date 5 April 2015
 */

#ifndef _FREERTOS_DRIVERS_NXP_LPC17XX40XXUART_HXX_
#define _FREERTOS_DRIVERS_NXP_LPC17XX40XXUART_HXX_

#include <cstdint>

#include "Serial.hxx"

#include "cmsis.h"
#if defined (CHIP_LPC175X_6X)
#include "cmsis_175x_6x.h"
#elif defined (CHIP_LPC177X_9X)
#include "cmsis_177x_8x.h"
#elif defined (CHIP_LPC407X_8X)
#include "cmsis_407x_8x.h"
#else
#error "LPC CHIP undefined"
#endif
#include "core_cm3.h"
#include "uart_17xx_40xx.h"

/** Specialization of Serial driver for LPC17xx and LPC40xx UART.
 */
class LpcUart : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    LpcUart(const char *name, LPC_USART_T *base, IRQn_Type interrupt);

    /** Destructor.
     */
    ~LpcUart()
    {
    }

    /** Translate an interrupt handler into C++ object context.
     * @param index UART index to translate
     */
    static void interrupt_handler(unsigned index)
    {
        instances[index]->interrupt_handler();
    }

private:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** handle an interrupt.
     */
    void interrupt_handler();

    /** Try and transmit a message.
     */
    void tx_char() override;

    LPC_USART_T *base; /**< base address of this device */
    IRQn_Type interrupt; /**< interrupt of this device */

#if  defined (CHIP_LPC177X_8X) || defined (CHIP_LPC407X_8X)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static LpcUart *instances[5];
#else
    /** Instance pointers help us get context from the interrupt handler(s) */
    static LpcUart *instances[4];
#endif

    /** Default constructor.
     */
    LpcUart();

    DISALLOW_COPY_AND_ASSIGN(LpcUart);
};


#endif /* _FREERTOS_DRIVERS_NXP_LPC17XX40XXUART_HXX_ */
