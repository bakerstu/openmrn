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
 * \file MSPM0SpiPixelStrip.hxx
 *
 * Implements an RGB pixel strip using the SPI peripheral of a TI MSPM0 device.
 *
 * @author Balazs Racz
 * @date 12 Jan 2025
 */

#ifndef _FREERTOS_DRIVERS_TI_MSPM0PIXELSTRIP_HXX_
#define _FREERTOS_DRIVERS_TI_MSPM0PIXELSTRIP_HXX_

#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

class SpiPixelStrip
{
public:
    /// Initializes the SPI peripheral.
    /// @param spi the SPI peripheral instance, e.g. SPI0.
    /// @param num_pixels the number of RGB pixels to drive.
    /// @param backing_data array of 3*num_pixels which stores the RGB data to
    /// be sent to the devices. Note that the byte order is G R B in the
    /// storage.
    SpiPixelStrip(SPI_Regs *spi, unsigned num_pixels, uint8_t *backing_data)
        : spi_(spi)
        , numPixels_(num_pixels)
        , data_(backing_data)
    { }

    /// Opens and initializes the SPI hardware.
    void hw_init();

    /// Updates the hardware from the backing data. The update is synchronous,
    /// this call returns when all bytes are sent, or at least enqueued in the
    /// SPI peripheral's TX FIFO.
    void update_sync()
    {
        clear_iteration();
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
            DL_SPI_transmitDataBlocking16(spi_, next_word);
            // Note that there is no wait here for the transfer to complete.
            // This is super important, because we want to be computing the next
            // word while the previous word is emitted by SPI.
        }
        // The last bit output is a zero, and leaving the line there is
        // reasonable, because it matches the latch level.
    }

private:
    /// Starts a new iteration over the strip. Call next_bit() repeatedly to get
    /// the bits in transmission order.
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
    SPI_Regs *spi_;
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
};

#endif // _FREERTOS_DRIVERS_TI_MSPM0PIXELSTRIP_HXX_
