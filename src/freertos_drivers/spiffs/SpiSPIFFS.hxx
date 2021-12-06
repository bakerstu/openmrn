/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file SpiSPIFFS.hxx
 *
 * Hardware-independent SPIFFS implementation that uses a SPI connected flash
 * chip.
 *
 * @author Balazs Racz
 * @date 5 Dec 2021
 */

#ifndef _FREERTOS_DRIVERS_SPIFFS_SPISPIFFS_HXX_
#define _FREERTOS_DRIVERS_SPIFFS_SPISPIFFS_HXX_

#include "freertos_drivers/spiffs/SPIFFS.hxx"

class SPIFlash;

class SpiSPIFFS : public SPIFFS
{
public:
    /// Constructor.
    /// @param flash is the spi flash driver. This must be initialized before
    /// calling any operation (such as mount) on this file system. This object
    /// must stay alive as long as the filesystem is in use (ownership is not
    /// transferred).
    /// @param physical_address address relative to the beginning of the flash
    /// device.
    /// @param size_on_disk how long the filesystem is
    /// @param logical_block_size see SPIFFS documentation
    /// @param logical_page_size see SPIFFS documentation
    /// @param max_num_open_descriptors how many fds should we allocate memory
    /// for
    /// @param cache_pages how many pages SPIFFS should store in RAM
    /// @param post_format_hook method to be called after a clean format of the
    /// file system. This allows the user to prime a clean or factory reset
    /// file system with an initial set files.
    SpiSPIFFS(SPIFlash *flash, size_t physical_address, size_t size_on_disk,
        size_t logical_block_size, size_t logical_page_size,
        size_t max_num_open_descriptors = 16, size_t cache_pages = 8,
        std::function<void()> post_format_hook = nullptr);

    /// Destructor.
    ~SpiSPIFFS();

private:
    /// SPIFFS callback to read flash, in context.
    /// @param addr adddress location to read
    /// @param size size of read in bytes
    /// @param dst destination buffer for read
    int32_t flash_read(uint32_t addr, uint32_t size, uint8_t *dst) override;

    /// SPIFFS callback to write flash, in context.
    /// @param addr adddress location to write
    /// @param size size of write in bytes
    /// @param src source buffer for write
    int32_t flash_write(uint32_t addr, uint32_t size, uint8_t *src) override;

    /// SPIFFS callback to erase flash, in context.
    /// @param addr adddress location to erase
    /// @param size size of erase region in bytes
    int32_t flash_erase(uint32_t addr, uint32_t size) override;

    /// Flash access helper.
    SPIFlash *flash_;

    DISALLOW_COPY_AND_ASSIGN(SpiSPIFFS);
};

#endif // _FREERTOS_DRIVERS_SPIFFS_SPISPIFFS_HXX_
