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
 * \file Stm32Uart.cxx
 * This file implements a UART device driver layer specific to STM32F0xx MCUs.
 *
 * @author Stuart W. Baker
 * @date 26 April 2015
 */

#include "Stm32Uart.hxx"

#if defined(STM32F072xB) || defined(STM32F091xC)
#include "stm32f0xx_hal_cortex.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_hal_cortex.h"
#elif defined(STM32F303xC)
#include "stm32f3xx_hal_cortex.h"
#else
#error Dont know what STM32 chip you have.
#endif

#if defined (STM32F030x6) || defined (STM32F031x6) || defined (STM32F038xx)
Stm32Uart *Stm32Uart::instances[1] = {NULL};
#elif defined (STM32F030x8) || defined (STM32F042x6) || defined (STM32F048xx) \
   || defined (STM32F051x8) || defined (STM32F058xx) || defined (STM32F070x6)
Stm32Uart *Stm32Uart::instances[2] = {NULL};
#elif defined (STM32F070xB) || defined (STM32F071xB) || defined (STM32F072xB) \
   || defined (STM32F078xx)
Stm32Uart *Stm32Uart::instances[4] = {NULL};
#elif defined (STM32F303xC)
Stm32Uart *Stm32Uart::instances[5] = {NULL};
#define USART4 UART4
#define USART5 UART5
#elif defined (STM32F030xC)
Stm32Uart *Stm32Uart::instances[6] = {NULL};
#elif defined (STM32F091xC) || defined (STM32F098xx)
Stm32Uart *Stm32Uart::instances[8] = {NULL};
#endif

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 */
Stm32Uart::Stm32Uart(const char *name, USART_TypeDef *base, IRQn_Type interrupt)
    : Serial(name)
    , interrupt(interrupt)
    , interrupt3_8EnableCnt(0)
{
    uartHandle.Instance = base;
    HAL_UART_DeInit(&uartHandle); 

    if (base == USART1)
    {
        instances[0] = this;
    }
#if !defined (STM32F030x6) && !defined (STM32F031x6) && !defined (STM32F038xx)
    else if (base == USART2)
    {
        instances[1] = this;
    }
#if !defined (STM32F030x8) && !defined (STM32F042x6) && !defined (STM32F048xx) \
 && !defined (STM32F051x8) && !defined (STM32F058xx) && !defined (STM32F070x6)
    else if (base == USART3)
    {
        instances[2] = this;
    }
    else if (base == USART4)
    {
        instances[3] = this;
    }
#if !defined (STM32F070xB) && !defined (STM32F071xB) && !defined (STM32F072xB) \
 && !defined (STM32F078xx)
    else if (base == USART5)
    {
        instances[4] = this;
    }
#if !defined (STM32F303xC)
    else if (base == USART6)
    {
        instances[5] = this;
    }
#if !defined (STM32F030xC)
    else if (base == USART7)
    {
        instances[6] = this;
    }
    else if (base == USART8)
    {
        instances[7] = this;
    }
#if !defined (STM32F091xC) && !defined (STM32F098xx)
	/* room for future devices with more UARTs */
#endif
#endif
#endif
#endif
#endif
#endif
    else
    {
        HASSERT(0);
    }

    HAL_NVIC_DisableIRQ(interrupt);
    HAL_NVIC_SetPriority(interrupt, 3, 0);
}

/** Enable use of the device.
 */
void Stm32Uart::enable()
{
    uartHandle.Init.BaudRate   = 115200;
    uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    uartHandle.Init.StopBits   = UART_STOPBITS_1;
    uartHandle.Init.Parity     = UART_PARITY_NONE;
    uartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uartHandle.Init.Mode       = UART_MODE_TX_RX;
    uartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    
    HAL_UART_Init(&uartHandle);
    __HAL_UART_ENABLE_IT(&uartHandle, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&uartHandle,  UART_IT_ERR);

    switch (interrupt)
    {
        case USART1_IRQn:
        case USART2_IRQn:
            HAL_NVIC_EnableIRQ(interrupt);
            break;
        default:
            if (interrupt3_8EnableCnt++ == 0)
            {
                HAL_NVIC_EnableIRQ(interrupt);
            }
            break;
    }
}

/** Disable use of the device.
 */
void Stm32Uart::disable()
{
    switch (interrupt)
    {
        case USART1_IRQn:
        case USART2_IRQn:
            HAL_NVIC_DisableIRQ(interrupt);
            break;
        default:
            if (--interrupt3_8EnableCnt == 0)
            {
                HAL_NVIC_DisableIRQ(interrupt);
            }
            break;
    }

    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_RXNE);
    HAL_UART_DeInit(&uartHandle); 
}

/** Try and transmit a message.
 */
void Stm32Uart::tx_char()
{
    if (!__HAL_UART_GET_IT_SOURCE(&uartHandle, UART_IT_TXE))
    {
        uint8_t data = 0;

        if (txBuf->get(&data, 1))
        {
            uartHandle.Instance->TDR = data;
            __HAL_UART_ENABLE_IT(&uartHandle, UART_IT_TXE);
            txBuf->signal_condition();
        }
    }
}

/** Common interrupt handler for all UART devices.
 */
void Stm32Uart::interrupt_handler()
{
    if (__HAL_UART_GET_IT(&uartHandle, UART_IT_ORE))
    {
        /* overrun error */
        ++overrunCount;
        __HAL_UART_CLEAR_IT(&uartHandle, UART_CLEAR_OREF);
    }
    if (__HAL_UART_GET_IT(&uartHandle, UART_IT_NE))
    {
        /* noise error */
        __HAL_UART_CLEAR_IT(&uartHandle, UART_CLEAR_NEF);
    }
    if (__HAL_UART_GET_IT(&uartHandle, UART_IT_FE))
    {
        /* framing error */
        __HAL_UART_CLEAR_IT(&uartHandle, UART_CLEAR_FEF);
    }
    while (__HAL_UART_GET_IT(&uartHandle, UART_IT_RXNE))
    {
        /** @todo (Stuart Baker) optimization opportunity by getting a write
         * pointer to fill the fifo and then advance the buffer when finished
         */
        uint8_t data = uartHandle.Instance->RDR;
        if (rxBuf->put(&data, 1) == 0)
        {
            ++overrunCount;
        }
        rxBuf->signal_condition_from_isr();
    }
    while (__HAL_UART_GET_IT(&uartHandle, UART_IT_TXE) &&
           __HAL_UART_GET_IT_SOURCE(&uartHandle, UART_IT_TXE))
    {
        /** @todo (Stuart Baker) optimization opportunity by getting a read
         * pointer to fill the fifo and then consume the buffer when finished.
         */
        uint8_t data;
        if (txBuf->get(&data, 1) != 0)
        {
            uartHandle.Instance->TDR = data;
            txBuf->signal_condition_from_isr();
        }
        else
        {
            /* no more data pending */
            __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_TXE);
            break;
        }
    }
}

/** Translate an interrupt handler into C++ object context.
* @param index UART index to translate
*/
void Stm32Uart::interrupt_handler(unsigned index)
{
#if !defined (STM32F030x6) && !defined (STM32F031x6) && !defined (STM32F038xx) \
 && !defined (STM32F030x8) && !defined (STM32F042x6) && !defined (STM32F048xx) \
 && !defined (STM32F051x8) && !defined (STM32F058xx) && !defined (STM32F070x6)
    if (index >= 2)
    {
        for (unsigned i = 2; i < (sizeof(instances)/sizeof(Stm32Uart*)); ++i)
        {
            UART_HandleTypeDef *uart_handle = &instances[i]->uartHandle;
            if ((uart_handle->Instance->CR3 & USART_CR3_EIE) ||
                (uart_handle->Instance->CR1 & USART_CR1_RXNEIE))
            {
                instances[i]->interrupt_handler();
            }
        }
    }
    else
#endif
    {
        instances[index]->interrupt_handler();
    }
}

extern "C" {
/** UART1 interrupt handler.
 */
void uart1_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(0);
}

#if !defined (STM32F030x6) && !defined (STM32F031x6) && !defined (STM32F038xx)
/** UART2 interrupt handler.
 */
void uart2_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(1);
}

#if defined (STM32F303xC)
/** UART3 interrupt handler.
 */
void uart3_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(2);
}

/** UART4 interrupt handler.
 */
void uart4_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(3);
}

/** UART5 interrupt handler.
 */
void uart5_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(4);
}

#else
#if !defined (STM32F030x8) && !defined (STM32F042x6) && !defined (STM32F048xx) \
 && !defined (STM32F051x8) && !defined (STM32F058xx) && !defined (STM32F070x6)
/** UART3 interrupt handler.
 */
void uart3_4_5_6_7_8_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(2);
}
#if !defined (STM32F070xB) && !defined (STM32F071xB) && !defined (STM32F072xB) \
 && !defined (STM32F078xx) && !defined (STM32F030xC) && !defined (STM32F091xC) \
 && !defined (STM32F098xx)
/* room for future devices with more Interrupt vectors */
#endif
#endif
#endif
#endif
} // extern C
