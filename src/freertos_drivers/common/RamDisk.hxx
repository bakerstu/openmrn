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
 * \file RamDisk.hxx
 * Implements a simple file node that stores all data in RAM.
 *
 * @author Balazs Racz
 * @date 22 March 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_RAMDISK_HXX_
#define _FREERTOS_DRIVERS_COMMON_RAMDISK_HXX_

#include <fcntl.h>
#include "Devtab.hxx"

/// A simple device driver that reads/write data from a block of memory in RAM.
///
/// Example:
///   extern uint8_t __flash_config_block_start[];
///   extern uint8_t __flash_config_block_end[];
///   static const size_t flash_config_block_length = __flash_config_block_end
///       - __flash_config_block_start;
///   RamDiskBase flashdisk("/dev/rdonly_config, __flash_config_block_start,
///                         flash_config_block_length, true);
///
///   then add appropriate linker symbols in the memory_map.ld for the project.
class RamDiskBase : public Node
{
public:
    /// Constructor.
    ///
    /// @param path device node name (e.g. "/etc/ramdisk_nodeid");
    /// @param data Blcok of RAM assigned to the file.
    /// @param size How many bytes should be exported.
    /// @param read_only if true, writes will be ignored.
    RamDiskBase(const char *path, void *data, unsigned size, bool read_only)
        : Node(path)
        , data_((uint8_t *)data)
        , size_(size)
        , readOnly_(read_only ? 1 : 0)
        , owned_(0)
    {
    }

private:
    void enable() OVERRIDE {}
    void disable() OVERRIDE {}
    void flush_buffers() OVERRIDE {}

    /** Open method */
    int open(File *file, const char * name, int flags, int mode) OVERRIDE
    {
        Node::open(file, name, flags, mode);
        if ((flags & O_TRUNC) && !readOnly_)
        {
            actualSize_ = 0;
        }
        return 0;
    }

    ssize_t read(File *file, void *buf, size_t count) OVERRIDE
    {
        if (file->offset >= file_size())
        {
            return 0;
        }
        size_t left = file_size() - file->offset;
        count = std::min(count, left);
        memcpy(buf, data_ + file->offset, count);
        file->offset += count;
        return count;
    }

    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE
    {
        if (readOnly_)
        {
            return -EPERM;
        }
        if (file->offset >= size_)
        {
            return 0;
        }
        size_t left = size_ - file->offset;
        count = std::min(count, left);
        memcpy(data_ + file->offset, buf, count);
        file->offset += count;
        if (file->offset > (off_t)actualSize_)
        {
            actualSize_ = file->offset;
        }
        return count;
    }

protected:
    /// @return the file end offset. This is either the size presented at the
    /// constructor for read-only files (that are filled with data when this
    /// device gets instantiated), or the actual bytes written.
    off_t file_size()
    {
        if (readOnly_)
        {
            return size_;
        }
        else
        {
            return actualSize_;
        }
    }

    /// Pointer to data content.
    uint8_t *data_;
    /// What's the larget file offset that we received an actual write for.
    unsigned actualSize_ = 0;
    /// How many bytes we are exporting.
    unsigned size_ : 30;
    /// 1 ifreadonly file.
    unsigned readOnly_ : 1;
    /// 1 if we own the data bytes and need to free()them upon exit.
    unsigned owned_ : 1;
};

/// A simple device driver that reads/write data from a block of memory in a
/// typed array either statically or dynamically allocated.
///
/// Example:
///   RamDisk rdisk("/dev/volatile_config", 256);
class RamDisk : public RamDiskBase
{
public:
    /// Allocates space on the heap for the ramdisk of size `size'.
    RamDisk(const char *path, size_t size)
        : RamDiskBase(path, new uint8_t[size], size, false)
    {
        memset(data_, 0, size_);
        owned_ = 1;
    }

    /// Uses an existing variable for backing the ramdisk structure. The
    /// variable is usually of a struct type, such as @ref
    /// openlcb::SimpleNodeDynamicValues. The variable may also be in flash.
    /// @param path is the device node name (e.g "/etc/node_config")
    /// @param data is the variable to export
    /// @param read_only if set, writes will be ignored.
    template<class T>
    RamDisk(const char* path, T* data, bool read_only = false)
        : RamDiskBase(path, data, sizeof(T), read_only) {
        owned_ = 0;
        actualSize_ = sizeof(T);
    }

    ~RamDisk()
    {
        if (owned_) {
            delete[] data_;
        }
    }
};

#endif // _FREERTOS_DRIVERS_COMMON_RAMDISK_HXX_
