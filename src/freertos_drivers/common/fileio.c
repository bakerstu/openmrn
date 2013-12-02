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
 * \file fileio.c
 * This file implements the generic fileio.
 *
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include "devtab.h"
#include "os/os.h"

#ifdef TARGET_LPC11Cxx
#define NUM_OPEN_FILES 4
#else
#define NUM_OPEN_FILES 8
#endif

/* prototypes */
static int null_init(devtab_t *dev);
static int null_open(file_t *file, const char *path, int flags, int mode);
static int null_close(file_t *file, node_t *node);
static ssize_t null_read(file_t *file, void *buf, size_t count);
static ssize_t null_write(file_t *file, const void *buf, size_t count);
static int null_ioctl(file_t *file, node_t *node, int key, void *data);

/** default stdin */
const char *STDIN_DEVICE __attribute__((weak)) = "/dev/null";

/** default stdout */
const char *STDOUT_DEVICE __attribute__((weak)) = "/dev/null";

/** default stderr */
const char *STDERR_DEVICE __attribute__((weak)) = "/dev/null";

/** device operations for can. */
static DEVOPS(null_ops, null_open, null_close, null_read, null_write,
              null_ioctl);

/** Null device entry. */
static DEVTAB_ENTRY(nullDevice, "/dev/null", null_init, &null_ops, NULL);

/** File descriptor array. */
static file_t files[NUM_OPEN_FILES];

/** Mutual exclusion */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** Allocate a free file descriptor.
 * @return file number on success, else -1 on failure
 */
int fd_alloc(void) {
  for (unsigned int i = 0; i < NUM_OPEN_FILES; i++) {
    if (files[i].inuse == 0) {
      files[i].inuse = 1;
      return i;
    }
  }
  return -1;
}

/** Free up a file descriptor.
 * @param file number to free up
 */
void fd_free(int fd) { files[fd].inuse = 0; }

/** Open a file or device.
 * @param reent thread save reentrant structure
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode, ignored in this implementation
 * @return 0 upon success, -1 upon failure with errno containing the cause
 */
int _open_r(struct _reent *reent, const char *path, int flags, int mode) {
  os_mutex_lock(&mutex);
  int fd = fd_alloc();
  os_mutex_unlock(&mutex);
  if (fd < 0) {
    errno = EMFILE;
    return -1;
  }
  // Sets the dev to a safe default in case we don't find the file later.
  files[fd].dev = &nullDevice;
  files[fd].flags = flags;
  for (devtab_t *dev = &DEVTAB[0]; dev != &DEVTAB_END; dev++) {
    if (!strcmp(dev->name, path)) {
      files[fd].dev = dev;
      int result = dev->devops->open(&files[fd], path, flags, mode);
      if (result < 0) {
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
int _close_r(struct _reent *reent, int fd) {
  if (fd >= 0 && fd <= 2) {
    // stdin, stdout, and stderr never get closed
    return 0;
  }
  if (fd < 0 || fd >= NUM_OPEN_FILES) {
    errno = EBADF;
    return -1;
  }
  if (files[fd].inuse == 0) {
    errno = EBADF;
    return -1;
  }
  int result = files[fd].dev->devops->close(&files[fd], files[fd].node);
  if (result < 0) {
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
 * @return number of bytes read upon success, -1 upon failure with errno
 * containing the cause
 */
ssize_t _read_r(struct _reent *reent, int fd, void *buf, size_t count) {
  if (fd < 0 || fd >= NUM_OPEN_FILES) {
    errno = EBADF;
    return -1;
  }
  if (files[fd].inuse == 0) {
    errno = EBADF;
    return -1;
  }
  int result = files[fd].dev->devops->read(&files[fd], buf, count);
  if (result < 0) {
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
 * @return number of bytes written upon success, -1 upon failure with errno
 * containing the cause
 */
ssize_t _write_r(struct _reent *reent, int fd, const void *buf, size_t count) {
  if (fd < 0 || fd >= NUM_OPEN_FILES) {
    errno = EBADF;
    return -1;
  }
  if (files[fd].inuse == 0) {
    errno = EBADF;
    return -1;
  }
  int result = files[fd].dev->devops->write(&files[fd], buf, count);
  if (result < 0) {
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
int _fstat_r(struct _reent *reent, int fd, struct stat *stat) { return 0; }

/** Get the tty information of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor determine if it is a tty
 * @return 1 if a tty, else 0
 */
int _isatty_r(struct _reent *reent, int fd) { return 1; }

/** Change the offset index of a file or device.
 * @param reent thread save reentrant structure
 * @param fd file descriptor to seek
 * @param offset offset within file
 * @param whence type of seek to complete
 * @return resulting offset from beginning of file, -1 upon failure with errno
 * containing the cause
 */
_off_t _lseek_r(struct _reent *reent, int fd, _off_t offset, int whence) {
  return 0;
}

/** Request and ioctl transaction
 * @param fd file descriptor
 * @param key ioctl key
 * @param ... key data
 */
int ioctl(int fd, int key, ...) {
  if (fd < 0 || fd >= NUM_OPEN_FILES) {
    errno = EBADF;
    return -1;
  }
  if (files[fd].inuse == 0) {
    errno = EBADF;
    return -1;
  }
  int result = files[fd].dev->devops->ioctl(&files[fd], files[fd].node, key, 0);
  if (result < 0) {
    errno = -result;
    return -1;
  }
  return 0;
}

/** intitailize the device
 * @parem dev device to initialize
 * @return 0 upon success
 */
static int null_init(devtab_t *dev) { return 0; }

/** Open a device.
 * @param file new file reference to this device
 * @param path file or device name
 * @param flags open flags
 * @param mode open mode
 * @return 0 upon success, negative errno upon failure
 */
static int null_open(file_t *file, const char *path, int flags, int mode) {
  return 0;
}

/** Close a device.
 * @param file file reference for this device
 * @param node node reference for this device
 * @return 0 upon success, negative errno upon failure
 */
static int null_close(file_t *file, node_t *node) { return 0; }

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno
 * containing the cause
 */
static ssize_t null_read(file_t *file, void *buf, size_t count) {
  if ((file->flags & O_NONBLOCK) == 0) {
    for (; /* forever */;) {
    }
  }

  return 0;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno
 * containing the cause
 */
static ssize_t null_write(file_t *file, const void *buf, size_t count) {
  return count;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param ... key data
 */
static int null_ioctl(file_t *file, node_t *node, int key, void *data) {
  return 0;
}

/* setup the device table for use in the linker */
__asm__(".section \".device.table." __xstring(devtab) ".begin\",\"aw\"\n"
        ".globl   " __xstring(DEVTAB)"\n"
        ".type    " __xstring(DEVTAB) ",object\n"
        ".p2align " __xstring(3) "\n"
        __xstring(DEVTAB) ":\n"
        ".previous\n");
__asm__(".section \".device.table." __xstring(devtab) ".finish\",\"aw\"\n"
        ".globl   " __xstring(DEVTAB_END)"\n"
        ".type    " __xstring(DEVTAB_END) ",object\n"
        ".p2align " __xstring(3) "\n"
        __xstring(DEVTAB_END) ":\n"
        ".previous\n");
