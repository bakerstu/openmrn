/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file CC32xxEEPROMEmulation.cxx
 * This file implements EEPROM emulation on the CC32xx SPI-flash filesystem.
 *
 * @author Balazs Racz
 * @date 30 December 2016
 */

#include "freertos_drivers/ti/CC32xxEEPROMEmulation.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "fs.h"

const size_t EEPROMEmulation::BLOCK_SIZE =
    CC32xxEEPROMEmulation::INT_BLOCK_SIZE;
const size_t EEPROMEmulation::BYTES_PER_BLOCK =
    CC32xxEEPROMEmulation::INT_BLOCK_SIZE / 2;

CC32xxEEPROMEmulation::CC32xxEEPROMEmulation(
    const char *name, size_t file_size_bytes)
    : EEPROMEmulation(name, file_size_bytes)
{
    bool have_rot = false;
    // First we need to find what was the last written segment.
    for (unsigned sector = 0; sector < sectorCount_; ++sector)
    {
        // Creates all files if they don't exist yet.
        int handle =
            open_file(sector, FS_MODE_OPEN_CREATE(SECTOR_SIZE + 4, 0), true);
        if (handle >= 0)
        {
            unsigned b = 0xaa55aaaa;
            sl_FsWrite(handle, SECTOR_SIZE, (uint8_t *)&b, 4);
            sl_FsClose(handle, nullptr, nullptr, 0);
            continue;
        }

        handle = open_file(sector, FS_MODE_OPEN_READ);
        unsigned count = MAGIC_COUNT * INT_BLOCK_SIZE;
        SlCheckResult(sl_FsRead(handle, MAGIC_FIRST_INDEX * INT_BLOCK_SIZE,
                          (uint8_t *)cache_, count),
            count);
        SlCheckResult(sl_FsClose(handle, nullptr, nullptr, 0));
        if (cache_[MAGIC_INTACT_INDEX * CACHE_BLOCK_MULT] != MAGIC_INTACT)
        {
            // Messed up or empty block.
            continue;
        }
        uint32_t current_rot = cache_[MAGIC_USED_INDEX * CACHE_BLOCK_MULT];
        if (current_rot > 0xff)
        {
            // maybe erased. We didn't write this, that's sure.
            continue;
        }
        if (!have_rot)
        {
            have_rot = true;
            fileRotation_ = current_rot;
            intactSector_ = sector;
        }
        else if (fileRotation_ == current_rot)
        {
            intactSector_ = sector;
        }
    }
    if (!have_rot)
        fileRotation_ = 0;
    mount();
    // This forces the first write to copy over everything to a new sector.
    availableSlots_ = 0;
}

CC32xxEEPROMEmulation::~CC32xxEEPROMEmulation()
{
    if (roSLFileHandle_ >= 0)
    {
        sl_FsClose(roSLFileHandle_, nullptr, nullptr, 0);
    }
    if (rwSLFileHandle_ >= 0)
    {
        sl_FsClose(rwSLFileHandle_, nullptr, nullptr, 0);
    }
}

void CC32xxEEPROMEmulation::flash_erase(unsigned sector)
{
    if (roSLFileHandle_ >= 0)
    {
        sl_FsClose(roSLFileHandle_, nullptr, nullptr, 0);
        roSLFileHandle_ = -1;
    }
    if (rwSLFileHandle_ >= 0)
    {
        roSLFileHandle_ = rwSLFileHandle_;
        roSector_ = rwSector_;
    }
    if (roSLFileHandle_ >= 0 && roSector_ == sector)
    {
        sl_FsClose(roSLFileHandle_, nullptr, nullptr, 0);
        roSLFileHandle_ = -1;
    }
    rwSector_ = sector;
    rwSLFileHandle_ = open_file(sector, FS_MODE_OPEN_WRITE);
    if (sector == 0)
    {
        fileRotation_++;
    }
    uint32_t rot = fileRotation_;
    SlCheckResult(
        sl_FsWrite(rwSLFileHandle_, get_rotation_offset(), (uint8_t *)&rot, 4), 4);
    intactSector_ = sector;
}

void CC32xxEEPROMEmulation::flash_program(
    unsigned sector, unsigned start_block, uint32_t *data, uint32_t byte_count)
{
    if (start_block == MAGIC_USED_INDEX)
    {
        HASSERT(byte_count == BLOCK_SIZE);
        // the used magic is handled internally.
        return;
    }
    HASSERT(sector == rwSector_);
    HASSERT(rwSLFileHandle_ >= 0);
    SlCheckResult(sl_FsWrite(rwSLFileHandle_, start_block * BLOCK_SIZE,
                             (uint8_t *)data, byte_count), byte_count);
    // Shadow write to the cache.
    if (sector == cacheSector_ && start_block >= cacheStartBlock_ &&
        start_block < (cacheStartBlock_ + CACHE_SIZE / INT_BLOCK_SIZE))
    {
        unsigned cache_offset =
            (start_block - cacheStartBlock_) * INT_BLOCK_SIZE;
        HASSERT(byte_count + cache_offset <= CACHE_SIZE);
        memcpy(((uint8_t *)cache_) + cache_offset, data, byte_count);
    }
}

int CC32xxEEPROMEmulation::open_file(
    unsigned sector, uint32_t open_mode, bool ignore_error)
{
    string filename(name);
    filename.push_back('.');
    filename.push_back('0' + sector);
    int32_t handle = -1;
    int ret = sl_FsOpen((uint8_t *)filename.c_str(), open_mode, NULL, &handle);
    if (!ignore_error)
    {
        SlCheckResult(ret);
        HASSERT(handle >= 0);
    }
    return handle;
}

const uint32_t *CC32xxEEPROMEmulation::block(unsigned sector, unsigned offset)
{
    if (sector == cacheSector_ && offset >= cacheStartBlock_ &&
        offset < (cacheStartBlock_ + CACHE_SIZE / INT_BLOCK_SIZE))
    {
        // We have a cache hit.
        return cache_ + (offset - cacheStartBlock_) / CACHE_BLOCK_MULT;
    }
    cacheSector_ = sector;
    cacheStartBlock_ = offset & CACHE_BLOCK_MASK;
    // Finds the correct file handle.
    int handle = -1;
    if (sector == rwSector_ && rwSLFileHandle_ >= 0)
    {
        handle = rwSLFileHandle_;
    }
    else if (sector == roSector_ && roSLFileHandle_ >= 0)
    {
        handle = roSLFileHandle_;
    }
    else
    {
        // Need to open file.
        if (roSLFileHandle_ >= 0)
        {
            sl_FsClose(roSLFileHandle_, nullptr, nullptr, 0);
            roSLFileHandle_ = -1;
        }
        roSLFileHandle_ = open_file(sector, FS_MODE_OPEN_READ);
        roSector_ = sector;
        handle = roSLFileHandle_;
    }
    // Refills the cache.
    SlCheckResult(sl_FsRead(handle, cacheStartBlock_ * INT_BLOCK_SIZE,
                            (uint8_t *)cache_, CACHE_SIZE), CACHE_SIZE);
    // Check if have the used magic in the cache. If yes, we need to fake it.
    if (cacheStartBlock_ == MAGIC_FIRST_INDEX)
    {
        if (sector != intactSector_)
        {
            cache_[MAGIC_USED_INDEX * CACHE_BLOCK_MULT] = MAGIC_USED;
        }
        else
        {
            cache_[MAGIC_USED_INDEX * CACHE_BLOCK_MULT] = MAGIC_ERASED;
        }
    }
    // Now we have the data in the cache.
    return cache_ + (offset - cacheStartBlock_) / CACHE_BLOCK_MULT;
}
