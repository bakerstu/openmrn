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

#ifdef TARGET_LPC11Cxx
#define NUM_OPEN_FILES     4
#else
#define NUM_OPEN_FILES     12
#endif

Device *Device::first = NULL;
OSMutex Device::mutex;
File Device::files[NUM_OPEN_FILES];

/** Constructor.
 * @param name name of device in file system. Pointer must be valid
 * throughout the entire lifetime.
 */
Device::Device(const char *name)
    : name(name)
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
Device::~Device()
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
int Device::open(struct _reent *reent, const char *path, int flags, int mode)
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
    for (Device *dev = first; dev != NULL; dev = dev->next)
    {
        if (!strcmp(dev->name, path))
        {
            files[fd].dev = dev;
            int result = dev->open(&files[fd], path, flags, mode);
            if (result < 0)
            {
                fd_free(fd);
                errno = -result;
                return -1;
            }
            return fd;
        }
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
int Device::close(struct _reent *reent, int fd)
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

/** Read from a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to read
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Device::read(struct _reent *reent, int fd, void *buf, size_t count)
{
    File* f = file_lookup(fd);
    if (!f) 
    {
        /* errno should already be set appropriately */
        return -1;
    }
    ssize_t result = f->dev->read(f, buf, count);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;
}

/** Write to a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to write
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Device::write(struct _reent *reent, int fd, const void *buf, size_t count)
{
    File* f = file_lookup(fd);
    if (!f) 
    {
        /* errno should already be set appropriately */
        return -1;
    }
    ssize_t result = f->dev->write(f, buf, count);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;
}

/** Change the offset index of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to seek
 * @param offset offset within file
 * @param whence type of seek to complete
 * @return resulting offset from beginning of file, -1 upon failure with errno containing the cause
 */
_off_t Device::lseek(struct _reent *reent, int fd, _off_t offset, int whence)
{
    File* f = file_lookup(fd);
    if (!f)
    {
        /* errno should already be set appropriately */
        return (_off_t) -1;
    }
    off_t result = f->dev->lseek(f, offset, whence);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;    
}

/** Get the status information of a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to get status of
 * @param stat structure to fill status info into
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int Device::fstat(struct _reent *reent, int fd, struct stat *stat)
{
    File* f = file_lookup(fd);
    if (!f)
    {
        /* errno should already be set appropriately */
        return -1;
    }
    ssize_t result = f->dev->fstat(f, stat);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;
}

/** Request and ioctl transaction.
 * @param fd file descriptor
 * @param key ioctl key
 * @param data key data
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int Device::ioctl(int fd, unsigned long int key, unsigned long data)
{
    File* f = file_lookup(fd);
    if (!f)
    {
        /* errno should already be set appropriately */
        return -1;
    }
    int result = f->dev->ioctl(f, key, data);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;    
}

/** Manipulate a file descriptor.
 * @param fd file descriptor
 * @param cmd operation to perform
 * @param data parameter to the cmd operation
 * @return dependent on the operation (POSIX compliant where applicable) or
 *         -1 on error with errno set appropriately
 */
int Device::fcntl(int fd, int cmd, unsigned long data)
{
    File *file = file_lookup(fd);

    if (!file) 
    {
        errno = EBADF;
        return -1;
    }

    switch (cmd)
    {
        case F_SETFL:
            /* on this platform, we ignore O_ASYNC, O_DIRECT, and O_NOATIME */
            data &= (O_APPEND | O_NONBLOCK);
            file->flags &= ~(O_APPEND | O_NONBLOCK);
            file->flags |= data;
            /* fall through */
        default:
        {
            File* f = file_lookup(fd);
            if (!f) 
            {
                /* errno should already be set appropriately */
                return -1;
            }
            int result = f->dev->fcntl(f, cmd, data);
            if (result < 0)
            {
                errno = -result;
                return -1;
            }
            return result;  
        }  
        case F_GETFL:
            return file->flags;
    }
}

/** Seek method.
 * @param file file reference for this device
 * @param offset offset in bytes from whence directive
 * @param whence SEEK_SET if to set the file offset to an abosolute position,
 *               SEEK_CUR if to set the file offset from current position
 * @return current offest, or -1 with errno set upon error.
 */
off_t Device::lseek(File* f, off_t offset, int whence)
{
    switch (whence)
    {
        case SEEK_SET:
            f->offset = offset;
            return offset;
        case SEEK_CUR:
            f->offset += offset;
            return f->offset;
    }
    return (off_t)-EINVAL;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int Device::ioctl(File *, unsigned long int, unsigned long) {
    return -EINVAL;
}

/** Manipulate a file descriptor.
 * @param file file reference for this device
 * @param cmd operation to perform
 * @param data parameter to the cmd operation
 * @return dependent on the operation (POSIX compliant where applicable)
 *         or negative error number upon error.
 */
int Device::fcntl(File *file, int cmd, unsigned long data)
{
    if (cmd == F_SETFL)
    {
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}

/** Allocate a free file descriptor.
 * @return file number on success, else -1 on failure
 */
int Device::fd_alloc(void)
{
    for (unsigned int i = 0; i < NUM_OPEN_FILES; i++)
    {
        if (files[i].inuse == false)
        {
            files[i].inuse = true;
            files[i].priv = nullptr;
            files[i].dev = nullptr;
            files[i].offset = 0;
            files[i].flags = 0;
            return i;
        }
    }
    return -1;
}

/** Free up a file descriptor.
 * @param fd number to free up
 */
void Device::fd_free(int fd)
{
    files[fd].inuse = false;
}

/** Looks up a reference to a File corresponding to a given file descriptor.
 * @param fd is a file descriptor as supplied to the read-write-close-ioctl
 *        commands.
 * @returns NULL and sets errno if fd is invalid, otherwise the File
 *          reference.
 */
File* Device::file_lookup(int fd)
{
    if (fd < 0 || fd >= NUM_OPEN_FILES)
    {
        errno = EBADF;
        return nullptr;
    }
    if (files[fd].inuse == 0)
    {
        errno = EBADF;
        return nullptr;
    }
    return &files[fd];
}

/** Looks up a file descriptor corresponding to a given File reference.
 * @param file is a reference to a File structure.
 * @returns file descriptor (assert on error).
 */
int Device::fd_lookup(File *file)
{
    HASSERT(file >= files && file <= (files + NUM_OPEN_FILES) && file->inuse);

    return (file - files);
}
