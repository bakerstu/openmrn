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
 * \file Devtab.hxx
 * This file represents a newlib stub for device drivers.
 *
 * @author Stuart W. Baker
 * @date 27 December 2012
 */

#ifndef _Devtab_hxx_
#define _Devtab_hxx_

#include <stropts.h>
#include <sys/types.h>
#include "os/OS.hxx"

struct File;
struct Node;

/** Device operations pointer structure.
 */
struct Devops
{
    /** Open method */
    int (*open)(File*, const char*, int, int);
    /** Close method */
    int (*close)(File*, Node*);
    /** Read method */
    ssize_t (*read)(File*, void*, size_t);
    /** Write method */
    ssize_t (*write)(File*, const void*, size_t);
    /** Ioctl method */
    int (*ioctl)(File*, Node*, unsigned long int, unsigned long);
};

/** Device tab structure.
 */
struct Devtab
{
public:
    /** Constructor.
     * @param name name of device in file system
     * @param devops reference to device operations
     * @param priv private data pointer for device to recall for later use
     */
    Devtab(const char* name, Devops* devops, void* priv)
        : name(name), devops(devops), priv(priv)
    {
        next = first;
        first = this;
    }

    /** Destructor */
    ~Devtab()
    {
    }

    /** Open a file or device.
     * @param reent thread save reentrant structure
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode, ignored in this implementation
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int open(struct _reent* reent, const char* path, int flags,
                    int mode);

    /** Close a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to close
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    int close(struct _reent* reent, int fd);

    /** Read from a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to read
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t read(struct _reent* reent, int fd, void* buf, size_t count);

    /** Write to a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to write
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno
     * containing the cause
     */
    ssize_t write(struct _reent* reent, int fd, const void* buf, size_t count);

    /** Request and ioctl transaction
     * @param fd file descriptor
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(int fd, unsigned long int key, unsigned long data);

    /** Get the private data pointer.
     * @return private data pointer
     */
    void* get_priv()
    {
        return priv;
    }

private:
    const char* name; /**< device name */
    Devops* devops;   /**< device operations */
    void* priv;       /**< device private data */

    /** first device in linked list */
    static Devtab* first;

    /** mutual exclusion for fileio */
    static OSMutex mutex;

    /** next device in linked list */
    Devtab* next;

    /** Default constructor */
    Devtab();

    DISALLOW_COPY_AND_ASSIGN(Devtab);
};

/** Node information.
 */
class Node
{
protected:
    /** Constructor.
     */
    Node() : references(0)
    {
    }

    /** Destructor */
    ~Node()
    {
    }

    unsigned int references; /**< number of open references */

private:
    DISALLOW_COPY_AND_ASSIGN(Node);
};

/** File information.
 */
struct File
{
    Devtab* dev;  /**< file operations */
    Node* node;   /**< node this file information refers to */
    off_t offset; /**< current offset within file */
    int flags;    /**< open flags */
    char inuse;   /**< is this file in use */
};

#endif /* _Devtab_hxx_ */
