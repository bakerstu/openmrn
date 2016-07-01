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
 * This file represents a newlib stub for device drivers.
 *
 * @todo: is this dead code?
 *
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#ifndef _FREERTOS_DRIVERS_COMMON_DEVTAB_H_
#define _FREERTOS_DRIVERS_COMMON_DEVTAB_H_

#include <sys/types.h>

/** Information about a currently open file.
 */
typedef struct file file_t;
/** Information about an entry in the filesystem (usually a device node).
 */
typedef struct node node_t;

/** Device operations pointer structure.
 */
typedef struct devops
{
    /** Open method */
    int (*open)(file_t *, const char *, int, int);
    /** Close method */
    int (*close)(file_t *, node_t*);
    /** Read method */
    ssize_t (*read)(file_t *, void *, size_t);
    /** Write method */
    ssize_t (*write)(file_t *, const void *, size_t);
    /** Ioctl method */
    int (*ioctl)(file_t *, node_t *, int, void *);
} devops_t;

/** Device tab structure.
 */
typedef struct devtab
{
    const char *name; /**< device name */
    int (*init)(struct devtab *); /**< initialization method */
    devops_t *devops; /**< device operations */
    void *priv; /**< device private data */
} devtab_t;

/** Node information.
 */
typedef struct node
{
    void *priv; /**< node private data */
    unsigned int references; /**< number of open references */
} node_t;

/** File information.
 */
typedef struct file
{
    devtab_t *dev; /**< file operations */
    node_t *node; /**< node this file information refers to */
    off_t offset; /**< current offset within file */
    int flags; /**< open flags */
    char inuse; /**< is this file in use */
} file_t;

/** Linker generated device table */
extern devtab_t DEVTAB[];
/** Linker generated device table end */
extern devtab_t DEVTAB_END;

/// Helper macro to turn a symbol into a string
#define __string(_x) #_x
/// Helper macro to turn a symbol into a string
#define __xstring(_x) __string(_x)

/** Device operations instance
 * @param _label unique label for instance
 * @param _open open method
 * @param _close close method
 * @param _read read method
 * @param _write write method
 * @param _ioctl ioctl method
 */
#define DEVOPS(_label, _open, _close, _read, _write, _ioctl) \
    devops_t _label =                                        \
    {                                                        \
        _open,                                               \
        _close,                                              \
        _read,                                               \
        _write,                                              \
        _ioctl                                               \
    };

/** Table entry.
 * @param _name table name for the entry
 */
#define TABLE_ENTRY(_name) \
    __attribute__((section((".device.table." __xstring(_name) ".data")))) \
    __attribute__((used))

/** Device Table entry instance.
 * @param _label unique label for entry
 * @param _name _name name of device
 * @param _init device initialization method
 * @param _devops device operations
 * @param _priv private data for use by device
 */
#define DEVTAB_ENTRY(_label, _name, _init, _devops, _priv) \
    devtab_t _label TABLE_ENTRY(devtab) =           \
    {                                               \
        _name,                                      \
        _init,                                      \
        _devops,                                    \
        _priv                                       \
    };

#endif /* _FREERTOS_DRIVERS_COMMON_DEVTAB_H_ */
