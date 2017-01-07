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

const uint8_t CC32xxEEPROMEmulation::SECTOR_COUNT = 8;

extern "C" {
void eeprom_updated_notification();
}

CC32xxEEPROMEmulation::CC32xxEEPROMEmulation(
    const char *name, size_t file_size_bytes)
    : EEPROMEmulation(name, file_size_bytes)
{
    data_ = malloc(file_size_bytes);
    HASSERT(data_);
    memset(data_, 0xff, file_size_bytes);
    bool have_rot = false;
    // First we need to find what was the last written segment.
    for (unsigned sector = 0; sector < sectorCount_; ++sector)
    {
        int handle = open_file(sector, FS_MODE_OPEN_READ, true);
        if (handle < 0)
        {
            continue;
        }
        unsigned b = 0xaa55aaaa;
        SlCheckResult(sl_FsRead(handle, 0, &b, 4), 4);
        sl_FsClose(handle, nullptr, nullptr, 0);
        if (b >= fileVersion_)
        {
            readSector_ = sector;
            have_rot = true;
        }
    }

    if (have_rot)
    {
        int handle = open_file(readSector_, FS_MODE_OPEN_READ);
        int ret = sl_FsRead(handle, 4, data, file_size_bytes);
        if (ret < 0)
        {
            SlCheckResult(ret);
        }
    }
}

CC32xxEEPROMEmulation::~CC32xxEEPROMEmulation()
{
    flush_buffers();
    free(data_);
}

void CC32xxEEPROMEmulation::flush_buffers()
{
    ++readSector_;
    ++fileVersion_;
    if (readSector_ >= SECTOR_COUNT)
        readSector_ = 0;
    int handle = open_file(readSector_, FS_MODE_OPEN_WRITE, true);
    if (handle < 0)
    {
        handle = open_file(sector, FS_MODE_OPEN_CREATE(file_size() + 4, 0));
    }
    SlCheckResult(sl_FsWrite(handle, 0, (uint8_t *)&fileVersion_, 4), 4);
    SlCheckResult(sl_FsWrite(handle, 4, data_, file_size()), 4);
    SlCheckResult(sl_FsClose(handle));
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

void CC32xxEEPROMEmulation::write(
    unsigned int index, const void *buf, size_t len)
{
    // Boundary checks are performed by the EEPROM class.
    memcpy(data_ + index, buf, len);
    eeprom_updated_notification();
}

void CC32xxEEPROMEmulation::read(unsigned int index, void *buf, size_t len)
{
    // Boundary checks are performed by the EEPROM class.
    memcpy(buf, data_ + index, len);
}
