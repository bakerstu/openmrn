/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file TivaFlash.hxx
 * This file implements a Flash-backed file on the TI Tiva controllers.
 *
 * @author Balazs Racz
 * @date 2 May 2015
 */

#include "TivaFlash.hxx"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/flash.h"

ssize_t TivaFlash::read(File *f, void *buf, size_t size)
{
    if ((uint32_t)f->offset >= len_)
    {
        size = 0;
    }
    else
    {
        size = std::min(size, len_ - (size_t)f->offset);
    }
    memcpy(buf, base_ + f->offset, size);
    f->offset += size;
    return size;
}

static uint32_t write_buf[32];

ssize_t TivaFlash::write(File *f, const void *buf, size_t size)
{
    // Do not write beyond the end of the file.
    if ((uint32_t)f->offset >= len_)
    {
        return 0;
    }
    else
    {
        size = std::min(size, len_ - (size_t)f->offset);
    }
    uint32_t write_address = reinterpret_cast<uint32_t>(base_) + f->offset;
    const uint8_t *read_buf = static_cast<const uint8_t *>(buf);
    uint32_t end_address = write_address + size;
    ssize_t ret = 0;
    while (write_address < end_address)
    {
        HASSERT((write_address & 3) == 0);
        // If we are at the start of a page, let's erase the page.
        if (!(write_address & subpage_mask()))
        {
            MAP_FlashErase(reinterpret_cast<uint32_t>(write_address));
        }
        uint32_t current_end_address = end_address;
        // Checks if we are trying to write more than one page.
        if ((current_end_address & write_page_mask()) !=
            (write_address & write_page_mask()))
        {
            // Stops writing at the end of the current page.
            current_end_address =
                (write_address + write_page_size() - 1) & write_page_mask();
        }
        uint32_t current_len = current_end_address - write_address;
        HASSERT(current_len <= sizeof(write_buf));
        memcpy(write_buf, read_buf, current_len);
        if (MAP_FlashProgram(write_buf, write_address, (current_len + 3) & ~3) < 0)
        {
            return -EIO;
        }
        write_address += current_len;
        read_buf += current_len;
        f->offset += current_len;
        ret += current_len;
    }
    return ret;
}
