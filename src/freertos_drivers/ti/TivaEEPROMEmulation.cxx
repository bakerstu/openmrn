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

inline const uint32_t *TivaEEPROMEmulation::get_block(
    unsigned sector, unsigned offset)
{
    return (uint32_t*)(&__eeprom_start + sector * EEPROMEmulation::SECTOR_SIZE + offset * EEPROMEmulation::BLOCK_SIZE);
}

/**
 * Computes the pointer to load the data stored in a specific block from.
 * @param sector sector number [0..sectorCount_ - 1]
 * @param offset block index within sector, [0..rawBlockCount_ - 1]
 * @return pointer to the beginning of the data in the block. Must be alive until the next call to this function.
 */
const uint32_t* TivaEEPROMEmulation::block(unsigned sector, unsigned offset) {
    return get_block(sector, offset);
}


/** Simple hardware abstraction for FLASH erase API.
 * @param address the start address of the flash block to be erased
 */
void TivaEEPROMEmulation::flash_erase(unsigned sector)
{
    HASSERT(sector < sectorCount_);

    /* because the TM4C123 device family has small flash sectors, we allow
     * multiple physical sectors to be ganged together to form one bigger
     * virtual sector for larger EEPROM file storage.
     */
    size_t sectors_per_sector = (SECTOR_SIZE /  FAMILY);

    auto* address = get_block(sector, 0);
    
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
     * @param sector the sector to write to
     * @param start_block the block index to start writing to
     * @param data a pointer to the data to be programmed
     * @param byte_count the number of bytes to be programmed.
     *              Must be a multiple of BLOCK_SIZE
     */
void TivaEEPROMEmulation::flash_program(
    unsigned sector, unsigned start_block, uint32_t *data, uint32_t byte_count)
{
    HASSERT(sector < sectorCount_);
    HASSERT((byte_count % BLOCK_SIZE) == 0);
    HASSERT(start_block + (byte_count / BLOCK_SIZE) <= rawBlockCount_);
    auto* address = get_block(sector, start_block);

    portENTER_CRITICAL();
    MAP_FlashProgram(data, (uint32_t)address, byte_count);
    portEXIT_CRITICAL();
}
