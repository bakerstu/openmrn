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
 * \file SpiSPIFFS.cxx
 *
 * Hardware-independent SPIFFS implementation that uses a SPI connected flash
 * chip.
 *
 * @author Balazs Racz
 * @date 5 Dec 2021
 */

#include "freertos_drivers/spiffs/SpiSPIFFS.hxx"

#include "freertos_drivers/common/SPIFlash.hxx"
#include "utils/logging.h"

SpiSPIFFS::SpiSPIFFS(SPIFlash *flash, size_t physical_address,
    size_t size_on_disk, size_t logical_block_size, size_t logical_page_size,
    size_t max_num_open_descriptors, size_t cache_pages,
    std::function<void()> post_format_hook)
    : SPIFFS(physical_address, size_on_disk, flash->cfg().sectorSize_,
          logical_block_size, logical_page_size, max_num_open_descriptors,
          cache_pages, post_format_hook)
    , flash_(flash)
{
}

SpiSPIFFS::~SpiSPIFFS()
{
    unmount();
}

int32_t SpiSPIFFS::flash_read(uint32_t addr, uint32_t size, uint8_t *dst)
{
    flash_->read(addr, dst, size);
    return 0;
}

int32_t SpiSPIFFS::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    flash_->write(addr, src, size);

#if (LOGLEVEL >= VERBOSE)
    uint8_t db[4];
    flash_read(addr, 4, db);
    LOG(VERBOSE, "check [%x]=%02x%02x%02x%02x", (unsigned)addr, db[0], db[1],
        db[2], db[3]);
#endif
    return 0;
}

int32_t SpiSPIFFS::flash_erase(uint32_t addr, uint32_t size)
{
    flash_->erase(addr, size);
    return 0;
}
