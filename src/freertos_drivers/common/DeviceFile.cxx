/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file DeviceFile.cxx
 * This implements a common device file abstraction.
 *
 * @author Stuart W. Baker
 * @date 16 July 2016
 */

#include "DeviceFile.hxx"

#include <fcntl.h>

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
int DeviceFile::open(File* file, const char *path, int flags, int mode)
{
    if (flags & O_APPEND || flags & O_TRUNC)
    {
        return -EINVAL;
    }

    if ((flags & O_CREAT) && (flags & O_EXCL))
    {
        return -EEXIST;
    }

    file->offset = 0;

    return Node::open(file, path, flags, mode);
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -errno upon failure
 */
ssize_t DeviceFile::read(File *file, void *buf, size_t count)
{
    ssize_t result = 0;

    if ((file->flags & O_ACCMODE) == O_WRONLY)
    {
        return -EBADF;
    }

    lock_.lock();
    if (count > 0)
    {
        /* if there is anything left to read */
        result = read(file->offset, buf, count);
        if (result > 0)
        {
            file->offset += result;
        }
    }
    lock_.unlock();

    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -errno upon failure
 */
ssize_t DeviceFile::write(File *file, const void *buf, size_t count)
{
    ssize_t result = 0;

    if ((file->flags & O_ACCMODE) == O_RDONLY)
    {
        return -EBADF;
    }

    lock_.lock();
    if (count > 0)
    {
        /* if there is anything left to write */
        result = write(file->offset, buf, count);
        if (result > 0)
        {
            file->offset += result;
        }
    }
    lock_.unlock();

    return result;
}


