/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file Fileio.cxx
 * This file implements the generic fileio.
 *
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#include "Devtab.hxx"

#include <cstdarg>
#include <fcntl.h>
#include <reent.h>
#include <unistd.h>
#include <sys/select.h>

OSMutex FileIO::mutex;

/** Allocate a free file descriptor.
 * @return file number on success, else -1 on failure
 */
int FileIO::fd_alloc(void)
{
    for (unsigned int i = 0; i < numOpenFiles; i++)
    {
        if (files[i].inuse == false)
        {
            files[i].inuse = true;
            files[i].inshdn = false;
            files[i].device = true;
            files[i].dir = false;
            files[i].dirty = false;
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
void FileIO::fd_free(int fd)
{
    files[fd].inuse = false;
}

/** Looks up a reference to a File corresponding to a given file descriptor.
 * @param fd is a file descriptor as supplied to the read-write-close-ioctl
 *        commands.
 * @returns NULL and sets errno if fd is invalid, otherwise the File
 *          reference.
 */
File* FileIO::file_lookup(int fd)
{
    if (fd < 0 || fd >= (int)numOpenFiles)
    {
        errno = EBADF;
        return nullptr;
    }
    if (files[fd].inuse == 0 || files[fd].inshdn == 1)
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
int FileIO::fd_lookup(File *file)
{
    HASSERT(file >= files && file <= (files + numOpenFiles) && file->inuse);

    return (file - files);
}

/** Read from a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to read
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t FileIO::read(struct _reent *reent, int fd, void *buf, size_t count)
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
ssize_t FileIO::write(struct _reent *reent, int fd, const void *buf, size_t count)
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
_off_t FileIO::lseek(struct _reent *reent, int fd, _off_t offset, int whence)
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
int FileIO::fstat(struct _reent *reent, int fd, struct stat *stat)
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
int FileIO::ioctl(int fd, unsigned long int key, unsigned long data)
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
int FileIO::fcntl(int fd, int cmd, unsigned long data)
{
    File *file = file_lookup(fd);

    if (!file)
    {
        errno = EBADF;
        return -1;
    }

    switch (cmd)
    {
        case F_GETFL:
            return file->flags;
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
    }
}

/** Seek method.
 * @param f file reference for this device
 * @param offset offset in bytes from whence directive
 * @param whence SEEK_SET if to set the file offset to an abosolute position,
 *               SEEK_CUR if to set the file offset from current position
 * @return current offset, or -1 with errno set upon error.
 */
off_t FileIO::lseek(File* f, off_t offset, int whence)
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
 * @param key ioctl key
 * @param data key data
 * @return 0 upon success or negative error number upon error.
 */
int FileIO::ioctl(File *, unsigned long int, unsigned long) {
    return -EINVAL;
}

/** Manipulate a file descriptor.
 * @param file file reference for this device
 * @param cmd operation to perform
 * @param data parameter to the cmd operation
 * @return dependent on the operation (POSIX compliant where applicable)
 *         or negative error number upon error.
 */
int FileIO::fcntl(File *file, int cmd, unsigned long data)
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

extern "C" {

/** Open a file or device.
 * @param reent thread safe reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _open_r(struct _reent *reent, const char *path, int flags, int mode)
{
    int result = Device::open(reent, path, flags, mode);
    if (result < 0 && errno == ENODEV)
    {
        return FileSystem::open(reent, path, flags, mode);
    }

    return result;
}

/** Close a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _close_r(struct _reent *reent, int fd)
{
    return FileIO::is_device(fd) ?     Device::close(reent, fd) :
                                   FileSystem::close(reent, fd);
}

/** Read from a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to read
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t _read_r(struct _reent *reent, int fd, void *buf, size_t count)
{
    return FileIO::read(reent, fd, buf, count);
}

/** Write to a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to write
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t _write_r(struct _reent *reent, int fd, const void *buf, size_t count)
{
    return FileIO::write(reent, fd, buf, count);
}

/** Get the status information of a file or device.
 * @param reent thread safe reentrant structure
 * @param path file or device name
 * @param stat structure to fill status info into
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _stat_r(struct _reent *reent, const char *path, struct stat *stat)
{
    int result = Device::stat(reent, path, stat);
    if (result < 0 && errno == ENOENT)
    {
        return FileSystem::stat(reent, path, stat);
    }

    return result;
}

/** Get the status information of a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to get status of
 * @param stat structure to fill status info into
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _fstat_r(struct _reent *reent, int fd, struct stat *stat)
{
    return FileIO::fstat(reent, fd, stat);
}

/** Get the tty information of a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor determine if it is a tty
 * @return 1 if a tty, else 0
 */
int _isatty_r(struct _reent *reent, int fd)
{
    return 1;
}

/** Get the tty information of a file or device.
 * @param fd file descriptor determine if it is a tty
 * @return 1 if a tty, else 0
 */
int _isatty(int fd)
{
    return 1;
}

/** remove a file.
 * @param reent thread safe reentrant structure
 * @param path file name
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _unlink_r(struct _reent *reent, const char *path)
{
    return FileSystem::unlink(reent, path);
}

/** Change the offset index of a file or device.
 * @param reent thread safe reentrant structure
 * @param fd file descriptor to seek
 * @param offset offset within file
 * @param whence type of seek to complete
 * @return resulting offset from beginning of file, -1 upon failure with errno containing the cause
 */
_off_t _lseek_r(struct _reent *reent, int fd, _off_t offset, int whence)
{
    return FileIO::lseek(reent, fd, offset, whence);
}

/** Synchronize (flush) a file to disk.
 * @param fd file descriptor to synch
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int fsync(int fd)
{
    return FileSystem::fsync(fd);
}

/** Request and ioctl transaction.
 * @param fd file descriptor
 * @param key ioctl key
 * @param ... key data
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int ioctl(int fd, unsigned long int key, ...)
{
    va_list ap;
    va_start(ap, key);

    int result = FileIO::ioctl(fd, key, va_arg(ap, unsigned long));
    
    va_end(ap);
    return result;
}

/** POSIX select().
 * @param nfds highest numbered file descriptor in any of the three, sets plus 1
 * @param readfds fd_set of file descritpors to pend on read active
 * @param writefds fd_set of file descritpors to pend on write active
 * @param exceptfds fd_set of file descritpors to pend on error active
 * @param timeout timeout value to wait, if 0, return immediately, if NULL
 *                wait forever
 * @return on success, number of file descriptors in the three sets that are
 *         active, 0 on timeout, -1 with errno set appropriately upon error.
 */
int select(int nfds, fd_set *readfds, fd_set *writefds,
           fd_set *exceptfds, struct timeval *timeout)
{
    long long time_value = -1;
    if (timeout) {
        time_value = timeout->tv_sec;
        time_value *= 1000000;
        time_value += timeout->tv_usec;
        time_value *= 1000;
    }
    Device::select_clear();
    return Device::select(nfds, readfds, writefds, exceptfds, time_value);
}

/** Manipulate a file descriptor.
 * @param fd file descriptor
 * @param cmd operation to perform
 * @param ... parameter to the cmd operation
 * @return dependent on the operation (POSIX compliant where applicable) or
 *         -1 on error with errno set appropriately
 */
int fcntl(int fd, int cmd, ...)
{
    va_list ap;
    va_start(ap, cmd);

    int result = FileIO::fcntl(fd, cmd, va_arg(ap, unsigned long));

    va_end(ap);
    return result;
}

}
