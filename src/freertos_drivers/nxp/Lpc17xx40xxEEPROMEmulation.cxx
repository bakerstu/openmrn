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

#include "iap.h"
#include "chip.h"

// We can't put a weak definition of a constant, because GCC, due to a bug,
// will inline the definition that appears here even if it is overridden at link
// time.
//
// const size_t __attribute__((weak)) EEPROMEmulation::SECTOR_SIZE = 0x8000;
const size_t EEPROMEmulation::BLOCK_SIZE = 16;
const size_t EEPROMEmulation::BYTES_PER_BLOCK = 8;

static uint8_t get_first_sector() {
    unsigned estart = (unsigned)(&__eeprom_start);
    if (estart < 0x10000) {
        return estart / 0x1000;
    } else {
        return 16 - 2 + estart / 0x8000; // estart==0x10000  -> sector==16.
    }
}

/** Constructor.
 * @param name device name
 * @param file_size maximum file size that we can grow to.
 */
LpcEEPROMEmulation::LpcEEPROMEmulation(const char *name, size_t file_size)
    : EEPROMEmulation(name, file_size), firstSector_(get_first_sector())
{
    /* IAP cannot access RAM below 0x10000200) */
    HASSERT(((uintptr_t)scratch) >= 0x10000200);

    unsigned eend = (unsigned)(&__eeprom_end);
    unsigned estart = (unsigned)(&__eeprom_start);
    if (eend <= 0x10000) {
        HASSERT(SECTOR_SIZE == 0x1000);
    } else if (estart >= 0x10000) {
        HASSERT(SECTOR_SIZE == 0x8000);
    } else {
        DIE("LPC17xx cannot have the EEPROM range cross the boundary of "
            "address of 0x10000");
    }
    HASSERT(estart % SECTOR_SIZE == 0);
    HASSERT(eend % SECTOR_SIZE == 0);
    
    mount();
}

inline const uint32_t *LpcEEPROMEmulation::get_block(
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
const uint32_t* LpcEEPROMEmulation::block(unsigned sector, unsigned offset) {
    return get_block(sector, offset);
}

/** Simple hardware abstraction for FLASH erase API.
 * @param sector Number of sector [0.. sectorCount_ - 1] to erase
 */
void LpcEEPROMEmulation::flash_erase(unsigned sector)
{
    HASSERT(sector < sectorCount_);

    unsigned s = sector + firstSector_;
    portENTER_CRITICAL();
    Chip_IAP_PreSectorForReadWrite(s, s);
    Chip_IAP_EraseSector(s, s);
    portEXIT_CRITICAL();
}

/** Simple hardware abstraction for FLASH program API.
 * @param sector the sector to write to [0..sectorCount_ - 1]
 * @param start_block the block index to start writing to [0..rawBlockCount_ -
 * 1]
 * @param data a pointer to the data to be programmed
 * @param byte_count the number of bytes to be programmed.
 *              Must be a multiple of BLOCK_SIZE
 */
void LpcEEPROMEmulation::flash_program(
    unsigned relative_sector, unsigned start_block, uint32_t *data, uint32_t byte_count)
{
    HASSERT(relative_sector < sectorCount_);
    HASSERT((byte_count % BLOCK_SIZE) == 0);
    HASSERT(start_block + (byte_count / BLOCK_SIZE) <= rawBlockCount_);
    auto* address = get_block(relative_sector, start_block);

    /* make sure we have the correct frequency */
    SystemCoreClockUpdate();

    uint32_t start_address = (uintptr_t)address & ~(WRITE_SIZE - 1);
    uint32_t sector = firstSector_ + relative_sector;

    HASSERT(((uintptr_t)address + byte_count) <= (start_address + WRITE_SIZE));

    memset(scratch, 0xFF, sizeof(scratch));
    memcpy(scratch + ((uintptr_t)address - start_address), data, byte_count);

    portENTER_CRITICAL();
    Chip_IAP_PreSectorForReadWrite(sector, sector);
    Chip_IAP_CopyRamToFlash(start_address, (uint32_t*)scratch, WRITE_SIZE);
    portEXIT_CRITICAL();
}
