/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file TivaEEPROMFile.hxx
 *
 * Implementation for persistent storage that uses Tiva EEPROM.
 *
 * @author Balazs Racz
 * @date 9 Dec 2022
 */

#ifndef _FREERTOS_DRIVER_TI_TIVAEEPROMFILE_HXX_
#define _FREERTOS_DRIVER_TI_TIVAEEPROMFILE_HXX_

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"

#include "freertos_drivers/common/DeviceFile.hxx"

class TivaEEPROMFile : public DeviceFile
{
public:
    TivaEEPROMFile(const char *name, unsigned byte_offset, unsigned byte_size)
        : DeviceFile(name)
        , byteOffset_(byte_offset)
        , byteSize_(byte_size)
    {
        MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
        // Due to Erratum MEM#11 we must never use ROM_EEPROMInit.
        EEPROMInit();
        // This is for Tiva 129. For 123 it would be okay to have 2*64.
        HASSERT(byte_offset % (8 * 64) == 0);
        HASSERT(byte_size % (8 * 64) == 0);
        HASSERT(byteOffset_ + byteSize_ <= MAP_EEPROMSizeGet());
    }

    /// Implements querying the file size.
    int fstat(File *file, struct stat *stat) override
    {
        DeviceFile::fstat(file, stat);
        stat->st_size = byteSize_;
        return 0;
    }

    /// Write to the eeprom.
    /// @param index index within the file address space to start write
    /// @param buf data to write
    /// @param len length in bytes of data to write
    /// @return number of bytes written upon success, -errno upon failure
    ssize_t write(unsigned int index, const void *buf, size_t len) override
    {
        size_t count = 0;
        uint8_t *b = (uint8_t *)buf;
        // Partial write at the beginning.
        if (index & 0x3)
        {
            uint32_t rd = 0;
            MAP_EEPROMRead(&rd, (index + byteOffset_) & ~0x3, 4);
            size_t actual_len = 4 - (index & 0x3);
            actual_len = std::min(len, actual_len);
            memcpy(((uint8_t*)&rd) + (index & 0x3), b, actual_len);
            MAP_EEPROMProgram(&rd, (index + byteOffset_) & ~0x3, 4);
            len -= actual_len;
            index += actual_len;
            b += actual_len;
            count += actual_len;
        }
        // Full writes in the middle.
        size_t word_len = len & ~0x3;
        if (word_len)
        {
            HASSERT((index & 0x3) == 0);
            MAP_EEPROMProgram((uint32_t *)b, index + byteOffset_, word_len);
            index += word_len;
            b += word_len;
            len -= word_len;
            count += word_len;
        }
        // Partial write at the end.
        if (len & 0x3)
        {
            HASSERT((index & 0x3) == 0);
            uint32_t rd = 0;
            MAP_EEPROMRead(&rd, index + byteOffset_, 4);
            memcpy(&rd, b, len);
            MAP_EEPROMProgram(&rd, (index + byteOffset_), 4);
            count += len;
        }
        return count;
    }

    /// Read from the eeprom.
    /// @param index index within DeviceFile address space to start read
    /// @param buf location to post read data
    /// @param len length in bytes of data to read
    /// @return number of bytes read upon success, -errno upon failure
    ssize_t read(unsigned int index, void *buf, size_t len) override
    {
        size_t count = 0;
        uint8_t *b = (uint8_t *)buf;
        // Partial read at the beginning.
        if (index & 0x3)
        {
            uint32_t rd = 0;
            MAP_EEPROMRead(&rd, (index + byteOffset_) & ~0x3, 4);
            size_t actual_len = 4 - (index & 0x3);
            actual_len = std::min(len, actual_len);
            memcpy(b, ((uint8_t *)&rd) + (index & 0x3), actual_len);
            len -= actual_len;
            index += actual_len;
            b += actual_len;
            count += actual_len;
        }
        // Full reads in the middle.
        size_t word_len = len & ~0x3;
        if (word_len)
        {
            HASSERT((index & 0x3) == 0);
            MAP_EEPROMRead((uint32_t *)b, index + byteOffset_, word_len);
            index += word_len;
            b += word_len;
            len -= word_len;
            count += word_len;
        }
        // Partial read at the end.
        if (len & 0x3)
        {
            HASSERT((index & 0x3) == 0);
            uint32_t rd = 0;
            MAP_EEPROMRead(&rd, index + byteOffset_, 4);
            memcpy(b, &rd, len);
            count += len;
        }
        return count;
    }

private:
    /// Address in the EEPROM of the first byte of this file.
    uint16_t byteOffset_;
    /// Total number of bytes in this file.
    uint16_t byteSize_;
};

#endif // _FREERTOS_DRIVER_TI_TIVAEEPROMFILE_HXX_
