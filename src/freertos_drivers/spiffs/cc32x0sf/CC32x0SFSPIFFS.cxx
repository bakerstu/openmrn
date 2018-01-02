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

#include "CC32x0SFSPIFFS.hxx"

#include "inc/hw_types.h"
#include "driverlib/flash.h"

//
// CC32x0SFSPIFFS::flash_read()
//
int32_t CC32x0SFSPIFFS::flash_read(uint32_t addr, uint32_t size, uint8_t *dst)
{
    HASSERT(addr >= config_.phys_addr &&
            (addr + size) < (config_.phys_addr  + config_.phys_size));

    memcpy(dst, (void*)addr, size);

    return 0;
}

//
// CC32x0SFSPIFFS::flash_write()
//
int32_t CC32x0SFSPIFFS::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    HASSERT(addr >= config_.phys_addr &&
            (addr + size) < (config_.phys_addr  + config_.phys_size));

    int misaligned = (addr + size) % 4;
    if (misaligned != 0)
    {
        // last write unaligned data
        uint32_t data = 0xFFFFFFFF;

        memcpy(&data, src + size - misaligned, misaligned);
        HASSERT(FlashProgram(&data, (addr + size) & (~0x3), 4) == 0);
        size -= misaligned;
    }

    misaligned = addr % 4;
    if (misaligned != 0)
    {
        // first write unaligned data
        uint32_t data = 0xFFFFFFFF;
        memcpy(&data + misaligned, src, 4 - misaligned);
        HASSERT(FlashProgram(&data, addr & (~0x3), 4) == 0);
        addr += 4 - misaligned;
        size -= 4 - misaligned;
        src  += 4 - misaligned;
    }

    HASSERT((addr % 4) == 0);
    HASSERT((size % 4) == 0);

    if (size)
    {
        // the rest of the aligned data
        HASSERT(FlashProgram((unsigned long*)src, addr, size) == 0);
    }


    return 0;
}

//
// CC32x0SFSPIFFS::flash_erase()
//
int32_t CC32x0SFSPIFFS::flash_erase(uint32_t addr, uint32_t size)
{
    HASSERT(addr >= config_.phys_addr &&
            (addr + size) < (config_.phys_addr  + config_.phys_size));

    while (size)
    {
        HASSERT(FlashErase(addr) == 0);
        addr += ERASE_PAGE_SIZE;
        size -= ERASE_PAGE_SIZE;
    }

    return 0;
}

