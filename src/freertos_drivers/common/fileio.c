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
 * \file devtab.h
 * This file implements the generic filio.
 *
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include "devtab.h"
#include "os/os.h"

#define NUM_DEVTAB_ENTRIES 4
#define NUM_OPEN_FILES     8

static devtab_t devices[NUM_DEVTAB_ENTRIES];
static file_t files[NUM_OPEN_FILES];
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

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
 * @param file number to free up
 */
void fd_free(int fd)
{
    files[fd].inuse = 0;
}

int _open_r(struct _reent *reent, const char *path, int flags, int mode)
{
    os_mutex_lock(&mutex);
    int fd = fd_alloc();
    os_mutex_unlock(&mutex);
    if (fd < 0)
    {
        errno = EMFILE;
        return -1;
    }
    for (unsigned int i = 0; i < NUM_DEVTAB_ENTRIES; i++)
    {
        if (!strcmp(devices[i].name, path))
        {
            int result = devices[i].fops.open(&files[i], path, flags, mode);
            if (result < 0)
            {
                errno = -result;
                return -1;
            }
            return 0;
        }
    }
    
    errno = ENODEV;
    return 0;
}

int _close_r(struct _reent *reent, int fd)
{
    return 0;
}

ssize_t _read_r(struct _reent *reent, int fd, void *buf, size_t count)
{
    return 0;
}

ssize_t _write_r(struct _reent *reent, int fd, const void *buf, size_t count)
{
    return 0;
}

int _fstat_r(struct _reent *reent, int fd, struct stat *stat)
{
    return 0;
}

int _isatty_r(struct _reent *reent, int fd)
{
    return 1;
}

_off_t _lseek_r(struct _reent *reent, int fd, _off_t offset, int whence)
{
    return 0;
}

