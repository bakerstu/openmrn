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
#include <sys/select.h>

extern "C" {

/** Open a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _open_r(struct _reent *reent, const char *path, int flags, int mode)
{
    return Device::open(reent, path, flags, mode);
}

/** Close a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to close
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _close_r(struct _reent *reent, int fd)
{
    return Device::close(reent, fd);
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
    return Device::read(reent, fd, buf, count);
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
    return Device::write(reent, fd, buf, count);
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
    return Device::lseek(reent, fd, offset, whence);
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

    int result = Device::ioctl(fd, key, va_arg(ap, unsigned long));
    
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

    int result = Device::fcntl(fd, cmd, va_arg(ap, unsigned long));

    va_end(ap);
    return result;
}

}
