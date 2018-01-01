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
 * \file Device.cxx
 * This file imlements Device level methods of the device file system.
 *
 * @author Stuart W. Baker
 * @date 31 January 2015
 */

#include "Devtab.hxx"

#include <fcntl.h>
#include <unistd.h>

FileSystem *FileSystem::first = NULL;

/** Constructor.
 * @param name name of device in file system. Pointer must be valid
 * throughout the entire lifetime.
 */
FileSystem::FileSystem(const char *name)
    : FileIO(name)
{
    mutex.lock();
    next = first;
    first = this;
    prev = NULL;
    if (next)
    {
        next->prev = this;
    }
    mutex.unlock();
}

/** Destructor.
 */
FileSystem::~FileSystem()
{
    mutex.lock();
    if (first == this)
    {
        first = next;
    }
    else
    {
        prev->next = next;
    }
    if (next)
    {
        next->prev = prev;
    }
    mutex.unlock();
}

/** Open a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::open(struct _reent *reent, const char *path, int flags, int mode)
{
    mutex.lock();
    int fd = fd_alloc();
    mutex.unlock();
    if (fd < 0)
    {
        errno = EMFILE;
        return -1;
    }
    files[fd].flags = flags;
    for (FileSystem *fs = first; fs != NULL; fs = fs->next)
    {
        if (fs->name == nullptr)        
        {
            /* mount path has no name */
            continue;
        }
        if (strlen(fs->name) > strlen(path))
        {
            /* path less than mount path */
            continue;
        }
        if (strncmp(fs->name, path, strlen(fs->name)))
        {
            /* no basename match */
            continue;
        }
        if (path[strlen(fs->name)] != '/')
        {
            /* not a directory break */
            continue;
        }

        files[fd].dev = fs;
        files[fd].device = false;
        int result = files[fd].dev->open(&files[fd], path, flags, mode);
        if (result < 0)
        {
            fd_free(fd);
            errno = -result;
            return -1;
        }
        return fd;
    }
    // No device found.
    fd_free(fd);
    errno = ENODEV;
    return -1;
}

/** Close a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int FileSystem::close(struct _reent *reent, int fd)
{
    File* f = file_lookup(fd);
    if (!f) 
    {
        /* errno should already be set appropriately */
        return -1;
    }
    if (fd >=0 && fd <= 2)
    {
        // stdin, stdout, and stderr never get closed
        return 0;
    }
    int result = f->dev->close(f);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    fd_free(fd);
    return 0;
}

