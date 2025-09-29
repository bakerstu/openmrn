/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file Stm32SpiPixelStrip.hxx
 *
 * Implements an RGB pixel strip using the SPI peripheral of a STM32 device.
 *
 * @author Balazs Racz
 * @date 18 Jan 2025
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32PIXELSTRIP_HXX_
#define _FREERTOS_DRIVERS_ST_STM32PIXELSTRIP_HXX_

#include <cstdint>


#include "stm32f_hal_conf.hxx"

#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_hal_dma.h"
#include "stm32g0xx_hal_spi.h"

class Stm32SpiPixelStrip
{
public:
    /// Initializes the SPI peripheral.
    /// @param spi the SPI peripheral instance, e.g. SPI0.
    /// @param num_pixels the number of RGB pixels to drive.
    /// @param backing_data array of 3*num_pixels which stores the RGB data to
    /// be sent to the devices. Note that the byte order is G R B in the
    /// storage.
    /// @param invert true if the bits output should be inverted.
    Stm32SpiPixelStrip(SPI_TypeDef *spi, DMA_Channel_TypeDef *dma_ch,
        unsigned dma_request, unsigned num_pixels, uint8_t *backing_data,
        bool invert = false)
        : spi_(spi)
        , dmaCh_(dma_ch)
        , dmaRequest_(dma_request)
        , numPixels_(num_pixels)
        , data_(backing_data)
        , invert_(invert)
    {
        memset(&spiHandle_, 0, sizeof(spiHandle_));
    }

    ~Stm32SpiPixelStrip()
    {
        delete[] dmaBuf_;
    }

    /// Opens and initializes the SPI hardware.
    /// @param spi_prescaler SPI_BAUDRATEPRESCALER_32 or a similar constant to
    /// match the necessary output timing.
    void hw_init(uint32_t spi_prescaler)
    {
        // Deinit if needed.
        if (spiHandle_.State != HAL_SPI_STATE_RESET)
        {
            HAL_SPI_DeInit(&spiHandle_);
        }
        spiHandle_.Instance = spi_;
        
        spiHandle_.Init.BaudRatePrescaler = spi_prescaler;
        spiHandle_.Init.Direction = SPI_DIRECTION_1LINE;
        spiHandle_.Init.CLKPolarity = SPI_POLARITY_LOW;
        spiHandle_.Init.CLKPhase = SPI_PHASE_2EDGE;
        spiHandle_.Init.DataSize = SPI_DATASIZE_16BIT;
        spiHandle_.Init.FirstBit = SPI_FIRSTBIT_LSB;
        spiHandle_.Init.NSS = SPI_NSS_SOFT;
        spiHandle_.Init.TIMode = SPI_TIMODE_DISABLE;
        spiHandle_.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
        spiHandle_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
        spiHandle_.Init.CRCPolynomial = 7;
        spiHandle_.Init.CRCLength = SPI_CRC_LENGTH_8BIT;
        spiHandle_.Init.Mode = SPI_MODE_MASTER;
        HASSERT(HAL_SPI_Init(&spiHandle_) == HAL_OK);
        LL_SPI_SetTransferDirection(spi_, LL_SPI_HALF_DUPLEX_TX);
    }

    /// Sets up a strip for a custom pattern.
    ///
    /// @param one_value These bits will be shifted out (LSB-first) for a one
    /// bit in the data. Typical value is 0b011
    /// @param one_bit_count Number of bits to shift out. Typical value is 3.
    /// @param zero_value These bits will be shifted out (LSB-first) for a zero
    /// bit in the data. Typical value is 0b001
    /// @param zero_bit_count  Number of bits to shift out. Typical value is 3.
    ///
    void set_pattern(uint8_t one_value, uint8_t one_bit_count,
        uint8_t zero_value, uint8_t zero_bit_count)
    {
        oneValue_ = one_value;
        oneBitCount_ = one_bit_count;
        zeroValue_ = zero_value;
        zeroBitCount_ = zero_bit_count;
    }

    /// Updates the hardware from the backing data. The update is synchronous,
    /// this call returns when all bytes are sent, or at least enqueued in the
    /// SPI peripheral's TX FIFO.
    void update_sync()
    {
        alloc_buffer();
        clear_iteration();
        uint16_t polarity = invert_ ? 0xffff : 0;
        unsigned word_ofs = 0;
        // Generates the output data.
        while (!eof())
        {
            uint32_t next_word = 0;
            uint16_t ofs = 0;
            // Computes 16 SPI bits with 5 pulses.
            while (!eof())
            {
                // Appends an 110 or 100 depending on what the next bit should
                // be. Generalized for other patterns too.
                if (next_bit())
                {
                    next_word |= uint32_t(oneValue_) << ofs;
                    ofs += oneBitCount_;
                }
                else
                {
                    next_word |= uint32_t(zeroValue_) << ofs;
                    ofs += zeroBitCount_;
                }
                if (ofs >= 16)
                {
                    dmaBuf_[word_ofs++] = (next_word & 0xffffu) ^ polarity;
                    next_word >>= 16;
                    ofs -= 16;
                }
            }
            // last / partial word
            dmaBuf_[word_ofs++] = (next_word & 0xffffu) ^ polarity;
        }
        // The last bit output is a zero, and leaving the line there is
        // reasonable, because it matches the latch level.

        // Triggers the actual transfer.
        dma_transfer(word_ofs);
    }

private:
    /// Allocates the DMA data buffer.
    void alloc_buffer()
    {
        if (dmaBuf_)
        {
            return;
        }
        unsigned need = std::max(zeroBitCount_, oneBitCount_) * 24 * numPixels_;
        dmaBuf_ = new uint16_t[need / 16];
    }

    /// Initiates the DMA transfer via SPI TX line.
    /// @param num_words the number of 16-bit SPI words that are written into
    /// the dmaBuf_.
    void dma_transfer(unsigned num_words)
    {
        dmaCh_->CCR &= ~DMA_CCR_EN;
        DMA_HandleTypeDef hdl = {0};
        memset(&hdl, 0, sizeof(hdl));
        hdl.Instance = dmaCh_;
        // Makes sure the channel is not running right now.
        HASSERT(READ_BIT(dmaCh_->CCR, DMA_CCR_EN) == 0);

        hdl.Init.Request = dmaRequest_;
        hdl.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdl.Init.PeriphInc = LL_DMA_PERIPH_NOINCREMENT;
        hdl.Init.MemInc = LL_DMA_MEMORY_INCREMENT;
        hdl.Init.PeriphDataAlignment = LL_DMA_PDATAALIGN_HALFWORD;
        hdl.Init.MemDataAlignment = LL_DMA_MDATAALIGN_HALFWORD;
        hdl.Init.Mode = LL_DMA_MODE_NORMAL;
        hdl.Init.Priority = LL_DMA_PRIORITY_HIGH;
        HASSERT(HAL_OK == HAL_DMA_Init(&hdl));
        HASSERT(HAL_OK ==
            HAL_DMA_Start(
                &hdl, (uint32_t)dmaBuf_, (uint32_t)&spi_->DR, num_words));
        // Enable error, transfer complete interrupt
        //__HAL_DMA_ENABLE_IT(&hdl, (DMA_IT_TC | DMA_IT_TE));

        // Enable SPI
        LL_SPI_Enable(spi_);
        // Enable Tx DMA Request. This will start the transfer by invoking the
        // DMA, which will then copy the first data byte.
        LL_SPI_EnableDMAReq_TX(spi_);
    }

    /// Starts a new iteration over the strip. Call next_bit() repeatedly to
    /// get the bits in transmission order.
    void clear_iteration()
    {
        currentByte_ = 0;
        nextBit_ = 0x80;
    }
    /// @return true if the next bit in the iteration should be 1.
    bool next_bit()
    {
        bool ret = data_[currentByte_] & nextBit_;
        nextBit_ >>= 1;
        if (!nextBit_)
        {
            nextBit_ = 0x80;
            currentByte_++;
        }
        return ret;
    }
    /// @return true when the iteration is at eof of the string.
    bool eof()
    {
        return currentByte_ >= numPixels_ * 3;
    }

    /// SPI peripheral pointer.
    SPI_TypeDef *spi_;
    /// Stm32 HAL device structure.
    SPI_HandleTypeDef spiHandle_;
    /// Hardware register set for the DMA channel.
    DMA_Channel_TypeDef* dmaCh_;
    /// DMAMUX request number.
    unsigned dmaRequest_;
    /// Number of pixels to drive.
    unsigned numPixels_;
    /// Backing framebuffer to use.
    uint8_t *data_;
    /// DMA buffer for output bit stream.
    uint16_t* dmaBuf_ = nullptr;

    /// Controls iteration over the data sequence when producing the output.
    /// This is the index of the current byte.
    unsigned currentByte_;
    /// Controls iteration over the data sequence when producing the output.
    /// This is the mask of the next bit to use.
    uint8_t nextBit_;
    /// If true, the bits are output inverted, false if normal.
    bool invert_;
    /// Right-aligned bit pattern to shift out for a zero. Will be shifted
    /// LSB-first.
    uint8_t zeroValue_ = 0b001;
    /// Number of bits in zeroValue_.
    uint8_t zeroBitCount_ = 3;
    /// Right-aligned bit pattern to shift out for a one. Will be shifted
    /// LSB-first.
    uint8_t oneValue_ = 0b011;
    /// Number of bits in oneValue_.
    uint8_t oneBitCount_ = 3;
};

#endif // _FREERTOS_DRIVERS_ST_STM32PIXELSTRIP_HXX_
