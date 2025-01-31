/** @copyright
 * Copyright (c) 2018, Stuart W Baker
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
 * @file CC32x0SFSPIFFS.cxx
 * This file implements a SPIFFS FLASH driver specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 1 January 2018
 */

// #define LOGLEVEL INFO

#include "utils/logging.h"

#include "TiSPIFFS.hxx"
#include "freertos_drivers/ti/TiFlash.hxx"
#include "spiffs.h"



//
// TiSPIFFS::flash_read()
//
template<unsigned ERASE_PAGE_SIZE>
int32_t TiSPIFFS<ERASE_PAGE_SIZE>::flash_read(uint32_t addr, uint32_t size, uint8_t *dst)
{
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));

    TiFlash<ERASE_PAGE_SIZE>::read(addr, dst, size);

    return 0;
}

//
// TiSPIFFS::flash_write()
//
template<unsigned ERASE_PAGE_SIZE>
int32_t TiSPIFFS<ERASE_PAGE_SIZE>::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    LOG(INFO, "Write %x sz %d", (unsigned)addr, (unsigned)size);

    TiFlash<ERASE_PAGE_SIZE>::write(addr, src, size);
    return 0;
}

//
// TiSPIFFS::flash_erase()
//
template<unsigned ERASE_PAGE_SIZE>
int32_t TiSPIFFS<ERASE_PAGE_SIZE>::flash_erase(uint32_t addr, uint32_t size)
{
    LOG(INFO, "Erasing %x sz %d", (unsigned)addr, (unsigned)size);
    HASSERT(addr >= fs_->cfg.phys_addr &&
            (addr + size) <= (fs_->cfg.phys_addr  + fs_->cfg.phys_size));
    HASSERT((size % ERASE_PAGE_SIZE) == 0);

    TiFlash<ERASE_PAGE_SIZE>::erase(addr, size);

    LOG(INFO, "Erasing %x done", (unsigned)addr);
    return 0;
}

