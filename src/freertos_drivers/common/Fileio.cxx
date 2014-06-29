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

#include <sys/stat.h>
#include <cstring>
#include <cerrno>
#include <cstdarg>
#include <fcntl.h>
#include "reent.h"
#include "Devtab.hxx"
#include "os/OS.hxx"

Device *Device::first = NULL;
OSMutex Device::mutex;

#ifdef TARGET_LPC11Cxx
#define NUM_OPEN_FILES     4
#else
#define NUM_OPEN_FILES     8
#endif


/** Null device instance */
class Null : public Device
{
public:
    /** Constructor */
    Null(const char* path) : Device(path)
    {
    }

    /** Open method */
    int open(File *, const char *, int, int) OVERRIDE;
    /** Close method */
    int close(File *) OVERRIDE;
    /** Read method */
    ssize_t read(File *, void *, size_t) OVERRIDE;
    /** Write method */
    ssize_t write(File *, const void *, size_t) OVERRIDE;
};

Null null("/dev/null");

/** default stdin */
const char *STDIN_DEVICE __attribute__ ((weak)) = "/dev/null";

/** default stdout */
const char *STDOUT_DEVICE __attribute__ ((weak)) = "/dev/null";

/** default stderr */
const char *STDERR_DEVICE __attribute__ ((weak)) = "/dev/null";

/** File descriptor array. */
static File files[NUM_OPEN_FILES];

/** Allocate a free file descriptor.
 * @return file number on success, else -1 on failure
 */
int fd_alloc(void)
{
    for (unsigned int i = 0; i < NUM_OPEN_FILES; i++)
    {
        if (files[i].inuse == 0)
        {
            files[i].inuse = 1;
            return i;
        }
    }
    return -1;
}

/** Free up a file descriptor.
 * @param fd number to free up
 */
void fd_free(int fd)
{
    files[fd].inuse = 0;
}

/** Looks up an fd.
 *
 * @param fd is a file descriptor as supplied to the read-write-close-ioctl
 * commands.
 * @returns NULL and sets errno if fd is invalid, otherwise the file structure
 * pointer. */
File* fd_find(int fd) {
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

/** Open a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _open_r(struct _reent *reent, const char *path, int flags, int mode)
{
    return Devtab::open(reent, path, flags, mode);
}

/** Close a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _close_r(struct _reent *reent, int fd)
{
    File* f = fd_find(fd);
    if (!f) 
    {
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
ssize_t _read_r(struct _reent *reent, int fd, void *buf, size_t count)
{
    File* f = fd_find(fd);
    if (!f) 
    {
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
ssize_t _write_r(struct _reent *reent, int fd, const void *buf, size_t count)
{
    File* f = fd_find(fd);
    if (!f) 
    {
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

/** Get the status information of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to get status of
 * @param stat structure to fill status info into
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _fstat_r(struct _reent *reent, int fd, struct stat *stat)
{
    return 0;
}

/** Get the tty information of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor determine if it is a tty
 * @return 1 if a tty, else 0
 */
int _isatty_r(struct _reent *reent, int fd)
{
    return 1;
}

/** Change the offset index of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to seek
 * @param offset offset within file
 * @param whence type of seek to complete
 * @return resulting offset from beginning of file, -1 upon failure with errno containing the cause
 */
_off_t _lseek_r(struct _reent *reent, int fd, _off_t offset, int whence)
{
    return 0;
}

/** Request and ioctl transaction.
 * @param fd file descriptor
 * @param key ioctl key
 * @param ... key data
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int ioctl(int fd, unsigned long int key, ...)
{
    File* f = fd_find(fd);
    if (!f) 
    {
        return -1;
    }

    va_list ap;
    va_start(ap, key);

    int result = f->ioctl(f, key, va_arg(ap, unsigned long));
    
    va_end(ap);
    return result;
}

Device::Device(const char *name)
    : name(name)
{
    mutex.lock();
    next = first;
    first = this;
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
    // Sets the dev to a safe default in case we don't find the file later.
    files[fd].dev = &null;
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

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
int Null::open(File* file, const char *path, int flags, int mode)
{    
    return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
int Null::close(File *file, Node*node)
{    
    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Null::read(File *file, void *buf, size_t count)
{
    // /dev/null returns EOF when trying to read from it.
    return 0;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Null::write(File *file, const void *buf, size_t count)
{    
    return count;
}

int Device::ioctl(File *, unsigned long int, unsigned long) {
    return -EINVAL;
}

/** Open method */
int Node::open(File *, const char *, int, int) OVERRIDE {
    OSMutexLock l(&lock_);
    if (references_++ == 0) {
        enable();
    }
}

/** Close method */
int Node::close(File *) OVERRIDE {
    OSMutexLock l(&lock_);
    if (--references_ <= 0) {
        disable();
        references_ = 0;
    }
}
