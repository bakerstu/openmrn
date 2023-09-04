/** @copyright
 * Copyright (c) 2019, Balazs Racz
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
 * @file Stm32SPIFFS.hxx
 *
 * This file implements a SPIFFS FLASH driver specific to STM32 F7 flash
 * API.
 *
 * @author Balazs Racz
 * @date 13 July 2019
 */

#ifndef _FREERTOS_DRIVERS_SPIFFS_STM32F0_F3_STM32SPIFFS_HXX_
#define _FREERTOS_DRIVERS_SPIFFS_STM32F0_F3_STM32SPIFFS_HXX_

#include <cstdint>

#include "freertos_drivers/spiffs/SPIFFS.hxx"
#include "freertos_drivers/st/Stm32Flash.hxx"

/// Specialization of Serial SPIFFS driver for CC32xx devices.
class Stm32SPIFFS : public SPIFFS
{
public:
    /// Constructor.
    Stm32SPIFFS(size_t physical_address, size_t size_on_disk,
                size_t logical_block_size, size_t logical_page_size,
                size_t max_num_open_descriptors = 16, size_t cache_pages = 8,
                std::function<void()> post_format_hook = nullptr)
        : SPIFFS(physical_address, size_on_disk, ERASE_PAGE_SIZE,
                 logical_block_size, logical_page_size,
                 max_num_open_descriptors, cache_pages, post_format_hook)
    {
    }

    /// Destructor.
    ~Stm32SPIFFS()
    {
        unmount();
    }

private:
    /// size of an erase page in FLASH
    static constexpr size_t ERASE_PAGE_SIZE = 256 * 1024;
    
    /// Implementation of the flash read/write routines.
    Stm32Flash<FlashVariableSectors> flash_{STM32F7_SINGLE_BANK_2M_FLASH};
    
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

    DISALLOW_COPY_AND_ASSIGN(Stm32SPIFFS);
};

#endif // _FREERTOS_DRIVERS_SPIFFS_STM32F0_F3_STM32SPIFFS_HXX_
