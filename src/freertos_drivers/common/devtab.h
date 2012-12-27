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
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#ifndef _devtab_h_
#define _devtab_h_

#include <sys/types.h>

typedef struct file file_t;
typedef struct node node_t;

/** Device operations pointer structure.
 */
typedef struct devops
{
    int (*open)(file_t*, const char *, int, int);
    int (*close)(file_t*, node_t*);
    ssize_t (*read)(file_t*, void *, size_t);
    ssize_t (*write)(file_t*, const void *, size_t);
    int (*ioctl)(file_t*, node_t*, int, void *);
} devops_t;

/** Device tab structure.
 */
typedef struct
{
    const char *name; /**< device name */
    devops_t fops; /**< device operations */
} devtab_t;

/** Node information.
 */
typedef struct node
{
    unsigned int references; /**< number of open references */
    void *priv; /**< node private data */
} node_t;

/** File information.
 */
typedef struct file
{
    int inuse; /**< is this file in use */
    node_t *node; /**< node this file information refers to */
    off_t offset; /**< current offset within file */
    devops_t *devops; /**< file operations */
} file_t;

#endif /* _devtab_h_ */
