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

//#define LOGLEVEL VERBOSE
#define SUPPORT_SL_R1_API

#include "utils/logging.h"

#include "freertos_drivers/ti/CC32xxEEPROMEmulation.hxx"
#include "freertos_drivers/ti/CC32xxHelper.hxx"
#include "simplelink.h"
#include "fs.h"

const uint8_t CC32xxEEPROMEmulation::SECTOR_COUNT = 8;

extern "C" {
void eeprom_updated_notification();
}

CC32xxEEPROMEmulation::CC32xxEEPROMEmulation(
    const char *name, size_t file_size_bytes)
    : EEPROM(name, file_size_bytes)
{
    data_ = (uint8_t *)malloc(file_size_bytes);
    HASSERT(data_);
    memset(data_, 0xff, file_size_bytes);
}

void CC32xxEEPROMEmulation::mount()
{
    bool have_rot = false;
    // First we need to find what was the last written segment.
    for (unsigned sector = 0; sector < SECTOR_COUNT; ++sector)
    {
        int handle = open_file(sector, SL_FS_READ, true);
        if (handle < 0)
        {
            LOG(VERBOSE, "EEPROM: sector %u: could not open.", sector);
            continue;
        }
        unsigned b = 0xaa55aaaa;
        SlCheckResult(sl_FsRead(handle, 0, (uint8_t *)&b, 4), 4);
        sl_FsClose(handle, nullptr, nullptr, 0);
        LOG(VERBOSE, "EEPROM: sector %u: version %u.", sector, b);
        if (b >= fileVersion_)
        {
            readSector_ = sector;
            fileVersion_ = b;
            have_rot = true;
        }
    }

    if (have_rot)
    {
        LOG(VERBOSE, "EEPROM: read sector %u:", readSector_);
        int handle = open_file(readSector_, SL_FS_READ);
        int ret = sl_FsRead(handle, 4, data_, file_size());
        if (ret < 0)
        {
            SlCheckResult(ret);
        }
        SlCheckResult(sl_FsClose(handle, nullptr, nullptr, 0));
    } else {
        LOG(VERBOSE, "EEPROM: no read sector");
    }
}

CC32xxEEPROMEmulation::~CC32xxEEPROMEmulation()
{
    flush();
    free(data_);
}

void CC32xxEEPROMEmulation::flush_buffers()
{
    if (!isDirty_)
        return;
    ++fileVersion_;
    ++readSector_;
    if (readSector_ >= SECTOR_COUNT)
        readSector_ = 0;
    LOG(VERBOSE, "EEPROM: write sector %u version %u", readSector_, fileVersion_);
    int handle = open_file(readSector_, SL_FS_WRITE, true);
    if (handle < 0)
    {
        handle =
            open_file(readSector_,
                      SL_FS_CREATE | SL_FS_CREATE_MAX_SIZE(file_size() + 4));
    }
    SlCheckResult(sl_FsWrite(handle, 0, (uint8_t *)&fileVersion_, 4), 4);
    SlCheckResult(sl_FsWrite(handle, 4, data_, file_size()), file_size());
    SlCheckResult(sl_FsClose(handle, nullptr, nullptr, 0));
    isDirty_ = 0;
}

int CC32xxEEPROMEmulation::open_file(
    unsigned sector, uint32_t open_mode, bool ignore_error)
{
    string filename(name);
    filename.push_back('.');
    filename.push_back('0' + sector);
    int ret = sl_FsOpen((uint8_t *)filename.c_str(), open_mode, NULL);
    if (!ignore_error && ret < 0)
    {
        SlCheckResult(ret);
    }
    return ret;
}

void CC32xxEEPROMEmulation::write(
    unsigned int index, const void *buf, size_t len)
{
    // Boundary checks are performed by the EEPROM class.
    if (memcmp(data_ + index, buf, len) == 0) {
        return;
    }
    memcpy(data_ + index, buf, len);
    isDirty_ = 1;
    eeprom_updated_notification();
}

void CC32xxEEPROMEmulation::read(unsigned int index, void *buf, size_t len)
{
    // Boundary checks are performed by the EEPROM class.
    memcpy(buf, data_ + index, len);
}
