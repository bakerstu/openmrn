/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file FlashFile.hxx
 *
 * File with backing data in flash (on-chip or SPI) in direct layout. This file
 * has restrictions on how it can be written.
 *
 * @author Balazs Racz
 * @date 30 Dec 2021
 */

#ifndef _FREERTOS_DRIVERS_COMMON_FLASHFILE_HXX_
#define _FREERTOS_DRIVERS_COMMON_FLASHFILE_HXX_

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "freertos_drivers/common/DeviceFile.hxx"

/// FlashFile is a driver for a single file that is backed by flash.
/// Instantiations may use serial flash (using SPIFlash) or internal flash.
///
/// There are limitations on writes.
///
/// - A sequential write of the file with no seeks from the beginning to the
///   end will work. Whenever the first byte of a sector is written, the entire
///   sector will be erased.
///
/// - If the file is opened with O_TRUNC, then all sectors of the file are
///   erased.
///
/// - The file does not remember its size. fstat always returns the maximum
///   size.
template <class FLASH> class FlashFile : public DeviceFile
{
public:
    /// Constructor.
    /// @param name what should be the name of this file be (to pass to ::open).
    /// @param flash accessor object to the backing flash. One such object can
    /// be used for multiple FlashFiles. The flash object shall do locking
    /// internally.
    /// @param address where does the data of this file start. Must be aligned
    /// on a sector boundary on the flash device. The unit is whatever address
    /// the flash driver understands.
    /// @param size maximum size of this file on flash.
    FlashFile(const char *name, FLASH *flash, size_t address, size_t size)
        : DeviceFile(name)
        , flash_(flash)
        , flashStart_(address)
        , size_(size)
    {
        auto start_sec = flash_->next_sector_address(flashStart_);
        // Beginning of the file must be on a sector boundary.
        HASSERT(start_sec == flashStart_);
    }

    /// Overrides behavior of open for O_TRUNC.
    int open(File *file, const char *path, int flags, int mode) override
    {
        if ((flags & O_TRUNC) && ((flags & O_ACCMODE) != O_RDONLY))
        {
            // erase entire file.
            flash_->erase(flashStart_, size_);
            flags &= ~O_TRUNC;
        }
        return DeviceFile::open(file, path, flags, mode);
    }

    /// Implements querying the file size.
    int fstat(File *file, struct stat *stat) override
    {
        DeviceFile::fstat(file, stat);
        stat->st_size = size_;
        return 0;
    }

    /// Write to the flash.
    /// @param index index within the file address space to start write
    /// @param buf data to write
    /// @param len length in bytes of data to write
    /// @return number of bytes written upon success, -errno upon failure
    ssize_t write(unsigned int index, const void *buf, size_t len) override
    {
        if (index >= size_)
        {
            return 0; // EOF
        }
        if (len > size_ - index)
        {
            len = size_ - index;
        }
        size_t addr = flashStart_ + index;
        size_t sec_addr = flash_->next_sector_address(addr);
        if (addr == sec_addr)
        {
            /// Writing at the beginning of a sector. Need an erase.
            size_t next_sec = flash_->next_sector_address(sec_addr + 1);
            flash_->erase(sec_addr, next_sec - sec_addr);
            sec_addr = next_sec;
        }
        if ((sec_addr - addr) < len)
        {
            len = sec_addr - addr;
        }
        flash_->write(addr, buf, len);
        return len;
    }

    /// Read from the flash.
    /// @param index index within DeviceFile address space to start read
    /// @param buf location to post read data
    /// @param len length in bytes of data to read
    /// @return number of bytes read upon success, -errno upon failure
    ssize_t read(unsigned int index, void *buf, size_t len) override
    {
        if (index >= size_)
        {
            return 0; // EOF
        }
        if (len > size_ - index)
        {
            len = size_ - index;
        }
        size_t addr = flashStart_ + index;
        flash_->read(addr, buf, len);
        return len;
    }

private:
    /// Accessor to the flash device.
    FLASH *flash_;
    /// Offset where our file start on flash.
    size_t flashStart_;
    /// How many bytes our file is on flash.
    size_t size_;
};

#endif
