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
 * \file Stm32F0xxUart.hxx
 * This file implements a UART device driver layer specific to STM32F0xx MCUs.
 *
 * @author Stuart W. Baker
 * @date 26 April 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_

#include <cstdint>

#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_uart.h"

#include "Serial.hxx"

/** Specialization of Serial driver for STM32F0xx devices.
 */
class Stm32Uart : public Serial
{
public:
    /** Constructor.
     * @param name name of this device instance in the file system
     * @param base base address of this device
     * @param interrupt interrupt number of this device
     */
    Stm32Uart(const char *name, USART_TypeDef *base, IRQn_Type interrupt);

    /** Destructor.
     */
    ~Stm32Uart()
    {
    }

    /** Translate an interrupt handler into C++ object context.
     * @param index UART index to translate
     */
    static void interrupt_handler(unsigned index);

private:
    void enable(); /**< function to enable device */
    void disable(); /**< function to disable device */

    /** @todo (Stuart Baker) this should be made private */
    /** handle an interrupt.
     */
    void interrupt_handler();

    /** Try and transmit a message.
     */
    void tx_char();

    IRQn_Type interrupt; /**< interrupt of this device */

    /** Handle to the UART setup */
    UART_HandleTypeDef uartHandle;

    /** number of times interrupts have been enabled on these UART channels */
    unsigned int interrupt3_8EnableCnt;

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Uart *instances[1];
#elif defined (STM32F030x8) || defined (STM32F042x6) || defined (STM32F048xx) \
   || defined (STM32F051x8) || defined (STM32F058xx) || defined (STM32F070x6)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Uart *instances[2];
#elif defined (STM32F070xB) || defined (STM32F071xB) || defined (STM32F072xB) \
   || defined (STM32F078xx)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Uart *instances[4];
#elif defined (STM32F030xC)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Uart *instances[6];
#elif defined (STM32F091xC) || defined (STM32F098xx)
    /** Instance pointers help us get context from the interrupt handler(s) */
    static Stm32Uart *instances[8];
#endif

    /** Default constructor.
     */
    Stm32Uart();

    DISALLOW_COPY_AND_ASSIGN(Stm32Uart);
};

#endif /* _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_ */
