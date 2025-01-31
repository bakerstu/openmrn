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
 * @author Brian Barnt
 * @date August 27, 2023
 */

#include "Stm32Uart.hxx"

#include "freertos/tc_ioctl.h"
#include "stm32f_hal_conf.hxx"

#include "FreeRTOSConfig.h"

// static
uint8_t Stm32Uart::interrupt3_to_8EnableCnt = 0;
// static
uint8_t Stm32Uart::interrupt2_8EnableCnt = 0;

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 */
Stm32Uart::Stm32Uart(const char *name, USART_TypeDef *base, IRQn_Type interrupt)
    : Serial(name)
    , interrupt(interrupt)
{
    memset(&uartHandle, 0, sizeof(uartHandle));
    uartHandle.Instance = base;
    HAL_UART_DeInit(&uartHandle); 

    HAL_NVIC_DisableIRQ(interrupt);
#if defined(GCC_ARMCM0)    
    HAL_NVIC_SetPriority(interrupt, 3, 0);
#elif defined(GCC_ARMCM3)    
    // Below kernel-compatible interrupt priority.
    SetInterruptPriority(interrupt, configMAX_SYSCALL_INTERRUPT_PRIORITY + 0x20);
#else
#error not defined how to set interrupt priority
#endif    
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

    volatile auto ret = HAL_UART_Init(&uartHandle);
    HASSERT(HAL_OK == ret);
    __HAL_UART_ENABLE_IT(&uartHandle, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&uartHandle,  UART_IT_ERR);

    // Verifies shared interrupts.
    if (false)
    {
    }
#if defined(SHARED_UART3_8_IRQn)
    else if (interrupt == SHARED_UART3_8_IRQn)
    {
        if (interrupt3_to_8EnableCnt++ == 0)
        {
            HAL_NVIC_EnableIRQ(interrupt);
        }
    }
#endif    
#if defined(STM32G0B1xx)
    // This is the only device we support with more than one shared interrupt
    // line.
    else if (interrupt == USART2_LPUART2_IRQn)
    {
        if (interrupt2_8EnableCnt++ == 0)
        {
            HAL_NVIC_EnableIRQ(interrupt);
        }
    }
#endif
    else
    {
        // Individual interrupt.
        HAL_NVIC_EnableIRQ(interrupt);
    }
}

/** Disable use of the device.
 */
void Stm32Uart::disable()
{
    // Verifies shared interrupts.
    if (false) { }
#if defined(SHARED_UART3_8_IRQn)
    else if (interrupt == SHARED_UART3_8_IRQn)
    {
        if (--interrupt3_to_8EnableCnt == 0)
        {
            HAL_NVIC_DisableIRQ(interrupt);
        }
    }
#endif    
#if defined(STM32G0B1xx)
    // This is the only device we support with more than one shared interrupt
    // lines.
    else if (interrupt == USART2_LPUART2_IRQn)
    {
        if (--interrupt2_8EnableCnt == 0)
        {
            HAL_NVIC_DisableIRQ(interrupt);
        }
    }
#endif
    else
    {
        // Individual interrupt.
        HAL_NVIC_DisableIRQ(interrupt);
    }
    
    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_TXE);
    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&uartHandle, UART_IT_RXNE);
    HAL_UART_DeInit(&uartHandle);
}

int Stm32Uart::ioctl(File *file, unsigned long int key, unsigned long data)
{
    switch (key)
    {
        default:
            return -EINVAL;
        case TCBAUDRATE:
            uartHandle.Init.BaudRate = data;
            volatile auto ret = HAL_UART_Init(&uartHandle);
            HASSERT(HAL_OK == ret);
            break;
    }
    return 0;
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
