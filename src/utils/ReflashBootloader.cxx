/** \copyright
 * Copyright (c) 2015, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file ReflashBootloader.cxx
 *
 * A standalone application with the sole purpose of replacing the bootloader
 * in a production MCU.
 *
 * @author Balazs Racz
 * @date 27 Oct 2015
 */

#include "utils/ReflashBootloader.hxx"

#include "openlcb/bootloader_hal.h"

/// The list of segments to copy from the reflash-bootloader program code to
/// the flash space occupied by the bootloader (and thus overwriting the
/// bootloader itself). The last entry of this table must have a length = 0.
extern const SegmentTable table[];

extern "C" {
/// This function needs to be called from reset_handler (instead of main etc.)
void reflash_bootloader_entry()
{
    bootloader_hw_set_to_safe();
    bootloader_hw_init();

    for (unsigned i = 0; table[i].length; i++)
    {
        uint8_t *dst_address = table[i].dst_address;
        const uint8_t *src_address = table[i].src_address;
        uint8_t *const end_address = table[i].dst_address + table[i].length;
        while (dst_address < end_address)
        {
            const void *page_start;
            uint32_t page_length_bytes;
            get_flash_page_info(dst_address, &page_start, &page_length_bytes);
            raw_erase_flash_page(page_start);
            uint32_t write_length = end_address - dst_address;
            uint32_t max_length = ((const uint8_t*)page_start) + page_length_bytes - dst_address;
            if (write_length > max_length)
            {
                write_length = max_length;
            }
            raw_write_flash(dst_address, src_address, write_length);
            dst_address += write_length;
            src_address += write_length;
        }
    }

    bootloader_reboot();
}
}
