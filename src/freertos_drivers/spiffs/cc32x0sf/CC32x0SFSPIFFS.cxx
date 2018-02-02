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
    HASSERT(addr >= config_.phys_addr &&
            (addr + size) <= (config_.phys_addr  + config_.phys_size));

    // This is the number of bytes of 0xFF we need to add to the beginning.
    const unsigned pre_misalign = addr & 3;
    // This is the number of bytes of 0xFF we need to add to the end.
    const unsigned post_misalign = (4 - ((addr+size) & 3)) & 3;

    uint32_t buf[4];
    uint8_t b8 = (uint8_t*)buf;
    if ((size <= 10) && (pre_misalign || post_misalign)) {
        memset(buf, 0xff, sizeof(buf));
        memcpy(b8 + pre_misalign, src, size);
        HASSERT(FlashProgram(buf, addr - pre_misalign,
                    size + pre_misalign + post_misalign) == 0);
        return 0;
    }

    unsigned len;
    if (pre_misalign) {
        buf[0] = 0xFFFFFFFF;
        len = 4-pre_misalign;
        memcpy(b8[pre_misalign], src, len);
        HASSERT(FlashProgram(buf, addr - pre_misalign, 4) == 0);
        addr += len;
        src += len;
        size -= len;
    }

    HASSERT((addr % 4) == 0);

    len = size & ~3u;
    HASSERT(FlashProgram(src, addr, len) == 0);
    addr += len;
    src += len;
    size -= len;

    HASSERT((addr % 4) == 0);
    HASSERT(size < 4);

    if (post_misalign) {
        buf[0] = 0xFFFFFFFF;
        memcpy(buf, src, size);
        HASSERT(FlashProgram(buf, addr, 4) == 0);
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

