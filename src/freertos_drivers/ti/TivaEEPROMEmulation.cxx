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
 * \file TivaEEPROMEmulation.cxx
 * This file implements Tiva compatible EEPROM emulation in FLASH.
 *
 * @author Stuart W. Baker
 * @date 21 January 2015
 */

#include "TivaEEPROMEmulation.hxx"

#include <cstring>

const size_t EEPROMEmulation::BLOCK_SIZE = 4;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 2;

/** Constructor.
 * @param name device name
 * @param file_size_bytes maximum file size that we can grow to.
 */
TivaEEPROMEmulation::TivaEEPROMEmulation(const char *name, size_t file_size_bytes)
    : EEPROMEmulation(name, file_size_bytes)
{
    HASSERT(SECTOR_SIZE >= (file_size() * 2));
    mount();
}

/** Simple hardware abstraction for FLASH erase API.
 * @param address the start address of the flash block to be erased
 */
void TivaEEPROMEmulation::flash_erase(void *address)
{

    HASSERT(((uintptr_t)address % SECTOR_SIZE) == 0);
    HASSERT((uintptr_t)address >= (uintptr_t)&__eeprom_start);
    HASSERT((uintptr_t)address < (uintptr_t)(&__eeprom_start + FLASH_SIZE));

    /* because the TM4C123 device family has small flash sectors, we allow
     * multiple physical sectors to be ganged together to form one bigger
     * virtual sector for larger EEPROM file storage.
     */
    size_t sectors_per_sector = ((file_size() - 1) / (FAMILY >> 1)) + 1;

    /* because the first "real" sector is always used to check block integrity,
     * we need to be sure that we erase that sector last. We assume that the
     * driver never tries to erase a sector that is marked as intact.
     */
    while (sectors_per_sector)
    {
        portENTER_CRITICAL();
        MAP_FlashErase((uint32_t)address + (FAMILY * --sectors_per_sector));
        portEXIT_CRITICAL();
    }
}

/** Simple hardware abstraction for FLASH program API.
 * @param data a pointer to the data to be programmed
 * @param address the starting address in flash to be programmed.
 *                Must be a multiple of BLOCK_SIZE
 * @param count the number of bytes to be programmed.
 *              Must be a multiple of BLOCK_SIZE
 */
void TivaEEPROMEmulation::flash_program(uint32_t *data, void *address,
                                        uint32_t count)
{
    HASSERT(((uintptr_t)address % BLOCK_SIZE) == 0);
    HASSERT((uintptr_t)address >= (uintptr_t)&__eeprom_start);
    HASSERT((uintptr_t)address < (uintptr_t)(&__eeprom_start + FLASH_SIZE));
    HASSERT((count % BLOCK_SIZE) == 0);

    portENTER_CRITICAL();
    MAP_FlashProgram(data, (uint32_t)address, count);
    portEXIT_CRITICAL();
}

/** Lookup sector number from address.
 * @param address sector address;
 * @return sector number.
 */
int TivaEEPROMEmulation::address_to_sector(const void *address)
{
    const uintptr_t uint_address = (const uintptr_t)address;

    int sector = uint_address / SECTOR_SIZE;

    HASSERT((FAMILY == TM4C129 &&
             (sector < (int)((FAMILY *  64) / SECTOR_SIZE))) ||
            (FAMILY == TM4C123 &&
             (sector < (int)((FAMILY * 256) / SECTOR_SIZE))));

    return sector;
}

/** Lookup address number from sector number.
 * @param sector sector number;
 * @return sector address.
 */
uint32_t *TivaEEPROMEmulation::sector_to_address(const int sector)
{
    HASSERT((FAMILY == TM4C129 &&
             (sector < (int)((FAMILY *  64) / SECTOR_SIZE))) ||
            (FAMILY == TM4C123 &&
             (sector < (int)((FAMILY * 256) / SECTOR_SIZE))));

    return (uint32_t*)(sector * SECTOR_SIZE);
}
