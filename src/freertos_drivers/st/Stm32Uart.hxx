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
 * \file Stm32Uart.hxx
 * This file implements a UART device driver layer specific to STM32F0xx MCUs.
 *
 * @author Stuart W. Baker
 * @date 26 April 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_

#include <cstdint>

#include "stm32f_hal_conf.hxx"

#include "Serial.hxx"

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx)
  #define NUM_USART 1
#elif defined (STM32F030x8) || defined (STM32F042x6) || defined (STM32F048xx) \
   || defined (STM32F051x8) || defined (STM32F058xx) || defined (STM32F070x6)
  #define NUM_USART 2
#elif defined (STM32L432xx)
  #define NUM_USART 3
  #define USART3 LPUART1
  #define LPUART_IDX 2
#elif defined (STM32L431xx)
  #define NUM_USART 4
  #define USART4 LPUART1
  #define LPUART_IDX 3
#elif defined (STM32F070xB) || defined (STM32F071xB) || defined (STM32F072xB) \
   || defined (STM32F078xx)
  #define NUM_USART 4
  #define SHARED_UART3_8_IRQn USART3_4_IRQn
#elif defined (STM32F303xC) || defined (STM32F303xE)
  #define NUM_USART 5
  #define USART4 UART4
  #define USART5 UART5
#elif defined (STM32F030xC)
  #define NUM_USART 6
  #define SHARED_UART3_8_IRQn USART3_6_IRQn
#elif defined (STM32F091xC) || defined (STM32F098xx)
  #define NUM_USART 8
  #define SHARED_UART3_8_IRQn USART3_8_IRQn
#elif defined (STM32F767xx)
  #define NUM_USART 8
  #define USART4 UART4
  #define USART5 UART5
  #define USART7 UART7
  #define USART8 UART8
#elif defined (STM32G0B1xx)
  #define NUM_USART 8
  #define SHARED_UART3_8_IRQn USART3_4_5_6_LPUART1_IRQn
  #define USART7 LPUART1
  #define USART8 LPUART2
#else
#error don_t know this STM32 MCU
#endif


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

    /** handle an interrupt.
     *
     * Call this function from HwInit.cxx from `void
     * u(s)artN_interrupt_handler(void)`. If there are multiple instances on
     * the same interrupt, call them all from the relevant interrupt handler.
     */
    void interrupt_handler();

    /** Request an ioctl transaction. Supported ioctl is TCBAUDRATE from
     * include/freertos/tc_ioctl.h */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;

protected:
    void enable() override; /**< function to enable device */
    void disable() override; /**< function to disable device */

    /** Try and transmit a message.
     */
    void tx_char() override;

    IRQn_Type interrupt; /**< interrupt of this device */

    /** Handle to the UART setup */
    UART_HandleTypeDef uartHandle;

    /** number of times interrupts have been enabled on these UART channels */
    static uint8_t interrupt3_to_8EnableCnt;
    static uint8_t interrupt2_8EnableCnt;

private:
    /** Default constructor.
     */
    Stm32Uart();

    DISALLOW_COPY_AND_ASSIGN(Stm32Uart);
};

#endif /* _FREERTOS_DRIVERS_ST_STM32F0XXUART_HXX_ */
