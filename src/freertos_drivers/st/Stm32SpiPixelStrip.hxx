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
    Stm32SpiPixelStrip(SPI_TypeDef *spi, unsigned num_pixels,
        uint8_t *backing_data, bool invert = false)
        : spi_(spi)
        , numPixels_(num_pixels)
        , data_(backing_data)
        , invert_(invert)
    {
        memset(&spiHandle_, 0, sizeof(spiHandle_));
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
        spiHandle_.Init.DataSize = SPI_DATASIZE_15BIT;
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

    /// Updates the hardware from the backing data. The update is synchronous,
    /// this call returns when all bytes are sent, or at least enqueued in the
    /// SPI peripheral's TX FIFO.
    void update_sync()
    {
        LL_SPI_Enable(spi_);
        clear_iteration();
        uint16_t polarity = invert_ ? 0xffff : 0;
        /// @todo use DMA instead of a critical section here.
        portENTER_CRITICAL();
        while (!eof())
        {
            uint16_t next_word = 0;
            uint16_t ofs = 1u;
            // Computes 16 SPI bits with 5 pulses.
            while (!eof() && ofs < (1u << 13))
            {
                // Appends an 110 or 100 depending opn what the next bit should
                // be.
                next_word |= ofs;
                ofs <<= 1;
                if (next_bit())
                    next_word |= ofs;
                ofs <<= 2;
            }
            // We leave an extra bit as zero at the end of the word. We hope
            // that this will not confuse the pixel. They care mostly about the
            // length of the HIGH pulse.

            // Waits for space in tx buffer.
            while (!LL_SPI_IsActiveFlag_TXE(spi_)) { }
            LL_SPI_TransmitData16(spi_, next_word ^ polarity);
            // Note that there is no wait here for the transfer to complete.
            // This is super important, because we want to be computing the
            // next word while the previous word is emitted by SPI.
        }
        // The last bit output is a zero, and leaving the line there is
        // reasonable, because it matches the latch level.
        portEXIT_CRITICAL();
    }

private:
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
    /// Number of pixels to drive.
    unsigned numPixels_;
    /// Backing framebuffer to use.
    uint8_t *data_;

    /// Controls iteration over the data sequence when producing the output.
    /// This is the index of the current byte.
    unsigned currentByte_;
    /// Controls iteration over the data sequence when producing the output.
    /// This is the mask of the next bit to use.
    uint8_t nextBit_;
    /// If true, the bits are output inverted, false if normal.
    bool invert_;
};

#endif // _FREERTOS_DRIVERS_ST_STM32PIXELSTRIP_HXX_
