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
            (addr + size) <= (config_.phys_addr  + config_.phys_size));

    memcpy(dst, (void*)addr, size);

    return 0;
}

//
// CC32x0SFSPIFFS::flash_write()
//
int32_t CC32x0SFSPIFFS::flash_write(uint32_t addr, uint32_t size, uint8_t *src)
{
    union WriteWord
    {
        uint8_t  data[4];
        uint32_t data_word;
    };

    HASSERT(addr >= config_.phys_addr &&
            (addr + size) <= (config_.phys_addr  + config_.phys_size));

    if ((addr % 4) && ((addr % 4) + size) < 4)
    {
        // single unaligned write in the middle of a word.
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(ww.data + (addr % 4), src, size);
        ww.data_word &= *((uint32_t*)(addr & (~0x3)));
        HASSERT(FlashProgram(&ww.data_word, addr & (~0x3), 4) == 0);

        return 0;
    }

    int misaligned = (addr + size) % 4;
    if (misaligned != 0)
    {
        // last write unaligned data
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(&ww.data_word, src + size - misaligned, misaligned);
        ww.data_word &= *((uint32_t*)((addr + size) & (~0x3)));
        HASSERT(FlashProgram(&ww.data_word, (addr + size) & (~0x3), 4) == 0);

        size -= misaligned;
    }

    misaligned = addr % 4;
    if (size && misaligned != 0)
    {
        // first write unaligned data
        WriteWord ww;
        ww.data_word = 0xFFFFFFFF;

        memcpy(ww.data + misaligned, src, 4 - misaligned);
        ww.data_word &= *((uint32_t*)(addr & (~0x3)));
        HASSERT(FlashProgram(&ww.data_word, addr & (~0x3), 4) == 0);
        addr += 4 - misaligned;
        size -= 4 - misaligned;
        src  += 4 - misaligned;
    }

    HASSERT((addr % 4) == 0);
    HASSERT((size % 4) == 0);

    if (size)
    {
        // the rest of the aligned data
        uint8_t *flash = (uint8_t*)addr;
        for (uint32_t i = 0; i < size; i += 4)
        {
            src[i + 0] &= flash[i + 0];
            src[i + 1] &= flash[i + 1];
            src[i + 2] &= flash[i + 2];
            src[i + 3] &= flash[i + 3];
        }

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
            (addr + size) <= (config_.phys_addr  + config_.phys_size));

    while (size)
    {
        HASSERT(FlashErase(addr) == 0);
        addr += ERASE_PAGE_SIZE;
        size -= ERASE_PAGE_SIZE;
    }

    return 0;
}

