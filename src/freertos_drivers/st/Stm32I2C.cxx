/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file Stm32I2C.cxx
 *
 * This file implements an I2C device driver layer on top of the STM32 Cube
 * middleware.
 *
 * @author Balazs Racz
 * @date 28 Oct 2018
 */

#include "Stm32I2C.hxx"

#if defined(STM32F072xB) || defined(STM32F091xC)
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_i2c.h"

// This timing is assuming 48 MHz main clock, the I2C module being clocked from
// the main clock, and gives 400 kHz clock (fast mode).
#define I2C_TIMING (__LL_I2C_CONVERT_TIMINGS(5, 0x3, 0x3, 0x3, 0x9))

#elif defined(STM32F103xB)
#include "stm32f1xx_ll_rcc.h"
#elif defined(STM32F303xC) || defined(STM32F303xE)
#include "stm32f3xx_ll_rcc.h"

// This timing is assuming 72 MHz main clock, the I2C module being clocked from
// the main clock, and gives 400 kHz clock (fast mode).
#define I2C_TIMING (__LL_I2C_CONVERT_TIMINGS(8, 0x3, 0x3, 0x3, 0x9))

#elif defined(STM32L431xx) || defined(STM32L432xx)
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_i2c.h"

// This timing is assuming 80 MHz main clock, the I2C module being clocked from
// the main clock, and gives 400 kHz clock (fast mode).
#define I2C_TIMING (__LL_I2C_CONVERT_TIMINGS(9, 0x3, 0x3, 0x3, 0x9))

#elif defined(STM32F767xx)
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_i2c.h"

// This timing is assuming 216 MHz main clock, the I2C module being clocked
// from the main clock, and gives 400 kHz clock (fast mode).
#define I2C_TIMING (__LL_I2C_CONVERT_TIMINGS(8, 9, 9, 9, 27))

#elif defined(STM32G0B1xx)
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_i2c.h"

// This timing is assuming 64 MHz main clock, the I2C module being clocked
// from the main clock, and gives 400 kHz clock (fast mode).
#define I2C_TIMING (__LL_I2C_CONVERT_TIMINGS(7, 0x3, 0x3, 0x3, 0x9))

#else
#error Dont know what STM32 chip you have.
#endif

// Enables the clock and resets the I2C peripheral.
static void i2c_reset(I2C_TypeDef *port)
{
    switch ((unsigned)port)
    {
        default:
            DIE("Unknown I2C port requested.");
#ifdef I2C1
        case I2C1_BASE:
            LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C1_CLK_ENABLE();
            __HAL_RCC_I2C1_FORCE_RESET();
            __HAL_RCC_I2C1_RELEASE_RESET();
            break;
#endif
#ifdef I2C2
        case I2C2_BASE:
            #ifdef LL_RCC_I2C2_CLKSOURCE_SYSCLK
            LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_SYSCLK);
            #endif
            __HAL_RCC_I2C2_CLK_ENABLE();
            __HAL_RCC_I2C2_FORCE_RESET();
            __HAL_RCC_I2C2_RELEASE_RESET();
            break;
#endif
#ifdef I2C3
        case I2C3_BASE:
            #ifdef LL_RCC_I2C3_CLKSOURCE_SYSCLK
            LL_RCC_SetI2CClockSource(LL_RCC_I2C3_CLKSOURCE_SYSCLK);
            #endif
            __HAL_RCC_I2C3_CLK_ENABLE();
            __HAL_RCC_I2C3_FORCE_RESET();
            __HAL_RCC_I2C3_RELEASE_RESET();
            break;
#endif
#ifdef I2C4
        case I2C4_BASE:
            __HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C4_CLK_ENABLE();
            __HAL_RCC_I2C4_FORCE_RESET();
            __HAL_RCC_I2C4_RELEASE_RESET();
            break;
#endif
#ifdef I2C5
        case I2C5_BASE:
            __HAL_RCC_I2C5_CONFIG(RCC_I2C5CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C5_CLK_ENABLE();
            __HAL_RCC_I2C5_FORCE_RESET();
            __HAL_RCC_I2C5_RELEASE_RESET();
            break;
#endif
#ifdef I2C6
        case I2C6_BASE:
            __HAL_RCC_I2C6_CONFIG(RCC_I2C6CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C6_CLK_ENABLE();
            __HAL_RCC_I2C6_FORCE_RESET();
            __HAL_RCC_I2C6_RELEASE_RESET();
            break;
#endif
#ifdef I2C7
        case I2C7_BASE:
            __HAL_RCC_I2C7_CONFIG(RCC_I2C7CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C7_CLK_ENABLE();
            __HAL_RCC_I2C7_FORCE_RESET();
            __HAL_RCC_I2C7_RELEASE_RESET();
            break;
#endif
#ifdef I2C8
        case I2C8_BASE:
            __HAL_RCC_I2C8_CONFIG(RCC_I2C8CLKSOURCE_SYSCLK);
            __HAL_RCC_I2C8_CLK_ENABLE();
            __HAL_RCC_I2C8_FORCE_RESET();
            __HAL_RCC_I2C8_RELEASE_RESET();
            break;
#endif
    }
}

/** Constructor.
 * @param name name of this device instance in the file system
 * @param port hardware instance of this device, e.g. I2C1
 * @param ev_interrupt event interrupt vector number
 * @param er_interrupt error interrupt vector number
 */
Stm32I2C::Stm32I2C(const char *name, I2C_TypeDef *port, uint32_t ev_interrupt,
    uint32_t er_interrupt)
    : I2C(name)
{
    i2c_reset(port);
    memset(&i2cHandle_, 0, sizeof(i2cHandle_));

    i2cHandle_.Instance = port;

    i2cHandle_.Init.Timing = I2C_TIMING;
    i2cHandle_.Init.OwnAddress1 = 0;
    i2cHandle_.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    i2cHandle_.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    i2cHandle_.Init.OwnAddress2 = 0xFF;
    i2cHandle_.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    i2cHandle_.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HASSERT(HAL_I2C_Init(&i2cHandle_) == HAL_OK);

    /* Enable the Analog I2C Filter */
    HAL_I2CEx_ConfigAnalogFilter(&i2cHandle_, I2C_ANALOGFILTER_ENABLE);

    // We save the object pointer in order to get back to *this in the callback
    // routines. This field of the Init structure is not used after the Init
    // call above.
    i2cHandle_.Init.Timing = (uint32_t) this;

#ifdef configKERNEL_INTERRUPT_PRIORITY  // cortex-m3 or more
    SetInterruptPriority((IRQn_Type)ev_interrupt, configKERNEL_INTERRUPT_PRIORITY);
    SetInterruptPriority((IRQn_Type)er_interrupt, configKERNEL_INTERRUPT_PRIORITY);
#endif    
    HAL_NVIC_EnableIRQ((IRQn_Type)ev_interrupt);
    HAL_NVIC_EnableIRQ((IRQn_Type)er_interrupt);
}

int Stm32I2C::transfer(struct i2c_msg *msg, bool stop)
{
    int bytes = msg->len;
    // The slave address is set with an 8-bit value where bit 0 (R/nW) is
    // ignored. We shift the address left for this reason.

    // We kill the previous state information because we don't want the driver
    // to optimize away the restart condition.
    i2cHandle_.PreviousState = 0;
    error_ = 0;

    if (msg->flags & I2C_M_RD)
    {
        /* this is a read transfer */
        uint32_t xfer_options;
        if (stop)
        {
            xfer_options = I2C_FIRST_AND_LAST_FRAME;
        }
        else
        {
            xfer_options = I2C_FIRST_FRAME;
        }

        if (HAL_I2C_Master_Seq_Receive_IT(&i2cHandle_,
                (uint16_t)msg->addr << 1, (uint8_t *)msg->buf, bytes,
                xfer_options) != HAL_OK)
        {
            return -EIO;
        }
    }
    else
    {
        /* this is a write transfer */
        uint32_t xfer_options;
        if (stop)
        {
            xfer_options = I2C_FIRST_AND_LAST_FRAME;
        }
        else
        {
            xfer_options = I2C_FIRST_FRAME;
        }
        if (HAL_I2C_Master_Seq_Transmit_IT(&i2cHandle_,
                (uint16_t)msg->addr << 1, (uint8_t *)msg->buf, bytes,
                xfer_options) != HAL_OK)
        {
            return -EIO;
        }
    }

    sem.wait();

    return error_ < 0 ? error_ : bytes;
}

Stm32I2C *dev_from_handle(I2C_HandleTypeDef *hi2c)
{
    return (Stm32I2C *)hi2c->Init.Timing;
}

void Stm32I2C::complete_from_isr()
{
    int woken = 0;
    sem.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}

void Stm32I2C::error_from_isr()
{
    auto error = i2cHandle_.ErrorCode;

    if (error & HAL_I2C_ERROR_ARLO)
    {
        error_ = -EAGAIN;
    }
    else if (error & HAL_I2C_ERROR_AF)
    {
        error_ = -ENOENT;
    }
    else if (error & HAL_I2C_ERROR_TIMEOUT)
    {
        error_ = -ETIMEDOUT;
    }
    else
    {
        error_ = -EIO;
    }

    complete_from_isr();
}

// =============== Callbacks from driver ===============

extern "C" {

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    dev_from_handle(hi2c)->complete_from_isr();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    dev_from_handle(hi2c)->complete_from_isr();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    dev_from_handle(hi2c)->error_from_isr();
}
}
