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
 * \file Stm32SPI.hxx
 *
 * This file implements a SPI device driver layer on top of the STM32 Cube
 * middleware.
 *
 * @author Balazs Racz
 * @date 29 April 2018
 */

#include "Stm32SPI.hxx"

#if defined(STM32F072xB) || defined(STM32F091xC)
#include "stm32f0xx_ll_rcc.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_ll_rcc.h"
#elif defined(STM32F303xC) || defined(STM32F303xE)
#include "stm32f3xx_ll_rcc.h"
#elif defined(STM32L432xx) || defined(STM32L431xx)
#include "stm32l4xx_ll_rcc.h"
#elif defined(STM32F767xx)
#include "stm32f7xx_ll_rcc.h"
#elif defined(STM32G0B1xx)
#include "stm32g0xx_ll_rcc.h"
#else
#error Dont know what STM32 chip you have.
#endif

#define SPI_DEFAULT_TIMEOUT 100u

// Enables the clock and resets the SPI peripheral.
static void spi_reset(SPI_TypeDef *port)
{
    switch ((unsigned)port)
    {
        default:
            DIE("Unknown SPI port requested.");
#ifdef SPI1
        case SPI1_BASE:
            __HAL_RCC_SPI1_CLK_ENABLE();
            __HAL_RCC_SPI1_FORCE_RESET();
            __HAL_RCC_SPI1_RELEASE_RESET();
            break;
#endif
#ifdef SPI2
        case SPI2_BASE:
            __HAL_RCC_SPI2_CLK_ENABLE();
            __HAL_RCC_SPI2_FORCE_RESET();
            __HAL_RCC_SPI2_RELEASE_RESET();
            break;
#endif
#ifdef SPI3
        case SPI3_BASE:
            __HAL_RCC_SPI3_CLK_ENABLE();
            __HAL_RCC_SPI3_FORCE_RESET();
            __HAL_RCC_SPI3_RELEASE_RESET();
            break;
#endif
#ifdef SPI4
        case SPI4_BASE:
            __HAL_RCC_SPI4_CLK_ENABLE();
            __HAL_RCC_SPI4_FORCE_RESET();
            __HAL_RCC_SPI4_RELEASE_RESET();
            break;
#endif
#ifdef SPI5
        case SPI5_BASE:
            __HAL_RCC_SPI5_CLK_ENABLE();
            __HAL_RCC_SPI5_FORCE_RESET();
            __HAL_RCC_SPI5_RELEASE_RESET();
            break;
#endif
#ifdef SPI6
        case SPI6_BASE:
            __HAL_RCC_SPI6_CLK_ENABLE();
            __HAL_RCC_SPI6_FORCE_RESET();
            __HAL_RCC_SPI6_RELEASE_RESET();
            break;
#endif
#ifdef SPI7
        case SPI7_BASE:
            __HAL_RCC_SPI7_CLK_ENABLE();
            __HAL_RCC_SPI7_FORCE_RESET();
            __HAL_RCC_SPI7_RELEASE_RESET();
            break;
#endif
#ifdef SPI8
        case SPI8_BASE:
            __HAL_RCC_SPI8_CLK_ENABLE();
            __HAL_RCC_SPI8_FORCE_RESET();
            __HAL_RCC_SPI8_RELEASE_RESET();
            break;
#endif
    }
}

Stm32SPI::Stm32SPI(const char *name, SPI_TypeDef *port, uint32_t interrupt,
    ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
    OSMutex *bus_lock)
    : SPI(name, cs_assert, cs_deassert, bus_lock)
{
    spi_reset(port);
    memset(&spiHandle_, 0, sizeof(spiHandle_));

    spiHandle_.Instance = port;

    HASSERT(update_configuration() == 0);
}

static const uint32_t baud_rate_table[] =
{
    2, SPI_BAUDRATEPRESCALER_2,     //
    4, SPI_BAUDRATEPRESCALER_4,     //
    8, SPI_BAUDRATEPRESCALER_8,     //
    16, SPI_BAUDRATEPRESCALER_16,   //
    32, SPI_BAUDRATEPRESCALER_32,   //
    64, SPI_BAUDRATEPRESCALER_64,   //
    128, SPI_BAUDRATEPRESCALER_128, //
    256, SPI_BAUDRATEPRESCALER_256, //
    0, 0                            //
};

int Stm32SPI::update_configuration()
{
    // Deinit if needed.
    if (spiHandle_.State != HAL_SPI_STATE_RESET)
    {
        HAL_SPI_DeInit(&spiHandle_);
    }

    // Computes the lowest divisor that gets us under the desired max speed Hz.
    uint32_t pclock = 0; //cm3_cpu_clock_hz;  //HAL_RCC_GetPCLK1Freq();
    /// @todo this might not be the correct clock value for the F303
    pclock = __LL_RCC_CALC_PCLK1_FREQ(configCPU_CLOCK_HZ, LL_RCC_GetAPB1Prescaler());
    unsigned ofs = 0;
    while (baud_rate_table[ofs] && ((pclock / baud_rate_table[ofs]) > speedHz))
    {
        ofs += 2;
    }
    if (baud_rate_table[ofs] == 0)
    {
        return -EINVAL;
        // DIE("Could not find an appropriate SPI clock divider.");
    }
    spiHandle_.Init.BaudRatePrescaler = baud_rate_table[ofs + 1];

    spiHandle_.Init.Direction = SPI_DIRECTION_2LINES;

    if (mode & SPI_CPOL)
    {
        // clk one when idle   CPOL = 1
        spiHandle_.Init.CLKPolarity = SPI_POLARITY_HIGH;
    }
    else
    {
        // clk zero when idle   CPOL = 0
        spiHandle_.Init.CLKPolarity = SPI_POLARITY_LOW;
    }

    if (mode & SPI_CPHA)
    {
        // sample on trailing edge  CPHA = 1
        spiHandle_.Init.CLKPhase = SPI_PHASE_2EDGE;
    }
    else
    {
        // sample on leading edge  CPHA = 0
        spiHandle_.Init.CLKPhase = SPI_PHASE_1EDGE;
    }

    switch (bitsPerWord)
    {
        default:
            return -EINVAL;
            // DIE("Unknown data size.");
            // Note there are more data sizes available, just to keep the code
            // size reasonable we don't add all options.
        case 7:
            spiHandle_.Init.DataSize = SPI_DATASIZE_7BIT;
            break;
        case 0:
        case 8:
            spiHandle_.Init.DataSize = SPI_DATASIZE_8BIT;
            break;
        case 9:
            spiHandle_.Init.DataSize = SPI_DATASIZE_9BIT;
            break;
        case 16:
            spiHandle_.Init.DataSize = SPI_DATASIZE_16BIT;
            break;
    }

    if (lsbFirst)
    {
        spiHandle_.Init.FirstBit = SPI_FIRSTBIT_LSB;
    }
    else
    {
        spiHandle_.Init.FirstBit = SPI_FIRSTBIT_MSB;
    }
    spiHandle_.Init.NSS = SPI_NSS_SOFT;
    spiHandle_.Init.TIMode = SPI_TIMODE_DISABLE;
    spiHandle_.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    spiHandle_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spiHandle_.Init.CRCPolynomial = 7;
    spiHandle_.Init.CRCLength = SPI_CRC_LENGTH_8BIT;
    spiHandle_.Init.Mode = SPI_MODE_MASTER;

    if (HAL_SPI_Init(&spiHandle_) != HAL_OK)
    {
        /* Initialization Error -- invalid arguments*/
        return -EINVAL;
    }

    return 0;
}

/** Method to transmit/receive the data.
 * @param msg message(s) to transact.
 * @return bytes transfered upon success, -errno upon failure
 */
int Stm32SPI::transfer(struct spi_ioc_transfer *msg)
{
    if (!msg->len)
    {
        return 0;
    }
    HAL_StatusTypeDef ret = HAL_OK;
    unsigned bytes = msg->len;
    if (!msg->tx_buf)
    {
        // doing receive
        if (msg->rx_buf & 1)
        {
            ret = HAL_SPI_Receive(
                &spiHandle_, (uint8_t *)msg->rx_buf, 1, SPI_DEFAULT_TIMEOUT);
            ++msg->rx_buf;
            --msg->len;
        }
        if (ret == HAL_OK && msg->len)
        {
            ret = HAL_SPI_Receive(&spiHandle_, (uint8_t *)msg->rx_buf, msg->len,
                SPI_DEFAULT_TIMEOUT);
        }
    }
    else if (!msg->rx_buf)
    {
        // doing transmit
        if (msg->tx_buf & 1)
        {
            ret = HAL_SPI_Transmit(
                &spiHandle_, (uint8_t *)msg->tx_buf, 1, SPI_DEFAULT_TIMEOUT);
            ++msg->tx_buf;
            --msg->len;
        }
        if (ret == HAL_OK && msg->len)
        {
            ret = HAL_SPI_Transmit(&spiHandle_, (uint8_t *)msg->tx_buf,
                msg->len, SPI_DEFAULT_TIMEOUT);
        }
        // Wait for transmit to complete
        while (__HAL_SPI_GET_FLAG(&spiHandle_, SPI_FLAG_BSY))
        {
        }
        // Flush receive buffer otherwise an upcoming receive will get weird
        // stuff.
        while (__HAL_SPI_GET_FLAG(&spiHandle_, SPI_FLAG_RXNE))
        {
            *(__IO uint8_t *)&spiHandle_.Instance->DR;
        }

    }
    else
    {
        // doing both TX and RX
        if ((msg->rx_buf & 1) && (msg->tx_buf & 1))
        {
            ret = HAL_SPI_TransmitReceive(&spiHandle_, (uint8_t *)msg->tx_buf,
                (uint8_t *)msg->rx_buf, 1, SPI_DEFAULT_TIMEOUT);
            ++msg->tx_buf;
            ++msg->rx_buf;
            --msg->len;
        }
        if ((msg->rx_buf & 1) || (msg->tx_buf & 1))
        {
            // Cannot have differently aligned TX and RX buffer.
            return -EINVAL;
        }
        if (ret == HAL_OK && msg->len)
        {
            ret = HAL_SPI_TransmitReceive(&spiHandle_, (uint8_t *)msg->tx_buf,
                (uint8_t *)msg->rx_buf, msg->len, SPI_DEFAULT_TIMEOUT);
        }
    }
    if (ret != HAL_OK)
    {
        return -EIO;
    }
    return bytes;
}
