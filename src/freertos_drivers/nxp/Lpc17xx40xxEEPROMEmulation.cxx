/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Lpc17xx40xxEEPROMEmulation.cxx
 * This file implements Lpc compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 15 June 2015
 */

#include "Lpc17xx40xxEEPROMEmulation.hxx"

#include <cstring>

#include "chip.h"

const size_t __attribute__((weak)) EEPROMEmulation::SECTOR_SIZE = 0x8000;
const size_t EEPROMEmulation::BLOCK_SIZE = 16;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 8;

/** Constructor.
 * @param name device name
 * @param file_size maximum file size that we can grow to.
 */
LpcEEPROMEmulation::LpcEEPROMEmulation(const char *name, size_t file_size)
    : EEPROMEmulation(name, file_size)
{
    /* IAP cannot access RAM below 0x10000200) */
    HASSERT(((uintptr_t)scratch) >= 0x10000200);

    mount();
}

/** Simple LpcWare abstraction for Chip_IAP_CopyRamToFlash() API.
 * @param data a pointer to the data to be programmed
 * @param address the starting address in flash to be programmed.
 *                Must be a multiple of BLOCK_SIZE.
 * @param count the number of bytes to be programmed.
 *              Must be a multiple of BLOCK_SIZE and WRITE_SIZE or less
 */
void LpcEEPROMEmulation::flash_program(uint32_t *data, void *address,
                                       uint32_t count)
{
    HASSERT(((uintptr_t)address % BLOCK_SIZE) == 0);
    HASSERT((uintptr_t)address >= (uintptr_t)&__eeprom_start);
    HASSERT((uintptr_t)address < (uintptr_t)(&__eeprom_start + (FLASH_SIZE >> 1)));
    HASSERT((count % BLOCK_SIZE) == 0);
    HASSERT(count <= WRITE_SIZE);

    /* make sure we have the correct frequency */
    SystemCoreClockUpdate();

    uint32_t start_address = (uintptr_t)address & ~(WRITE_SIZE - 1);
    uint32_t sector = address_to_sector((void*)start_address);

    HASSERT(((uintptr_t)address + count) <= (start_address + WRITE_SIZE));

    memset(scratch, 0xFF, sizeof(scratch));
    memcpy(scratch + ((uintptr_t)address - start_address), data, count);

    portENTER_CRITICAL();
    Chip_IAP_PreSectorForReadWrite(sector, sector);
    Chip_IAP_CopyRamToFlash(start_address, (uint32_t*)scratch, WRITE_SIZE);
    portEXIT_CRITICAL();
}


/** Lookup sector number from address.
 * @param address sector address;
 * @return sector number.
 */
int LpcEEPROMEmulation::address_to_sector(const void *address)
{
    const uintptr_t uint_address = (const uintptr_t)address;

    if (uint_address < 0x00010000)
    {
        return (uint_address / 0x1000);
    }
    else if (uint_address < 0x00080000)
    {
        return (14 + (uint_address / 0x8000));
    }
    else
    {
        HASSERT(0);
    }

    return 0;
}

/** Lookup address number from sector number.
 * @param sector sector number;
 * @return sector address.
 */
uint32_t *LpcEEPROMEmulation::sector_to_address(const int sector)
{
    if (sector < 16)
    {
        return (uint32_t*)(0x1000 * sector);
    }
    else if (sector < 30)
    {
        return (uint32_t*)(0x8000 * (sector - 14));
    }
    else
    {
        HASSERT(0);
    }

    return NULL;
}
