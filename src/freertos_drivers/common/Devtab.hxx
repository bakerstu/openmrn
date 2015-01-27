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

#ifndef _FREERTOS_DRIVERS_COMMON_DEVTAB_HXX_
#define _FREERTOS_DRIVERS_COMMON_DEVTAB_HXX_

#include <stropts.h>
#include <sys/types.h>
#include "os/OS.hxx"

struct File;
struct Node;
class Notifiable;

/** Device tab structure.
 */
struct Device
{
public:
    /** Constructor.
     * @param name name of device in file system. Pointer must be valid
     * throughout the entire lifetime.
     */
    Device(const char *name);

    /** Open method. Returns negative errno on failure. */
    virtual int open(File *, const char *, int, int) = 0;
    /** Close method. Returns negative errno on failure. */
    virtual int close(File *) = 0;
    /** Read method. Returns negative errno on failure. */
    virtual ssize_t read(File *, void *, size_t) = 0;
    /** Write method. Returns negative errno on failure. */
    virtual ssize_t write(File *, const void *, size_t) = 0;
    /** Seek method. Errors shall be written into errno and returns -1 on
     * error. The default implementation updates the offset in the File
     * structure. */
    virtual off_t lseek(File*, off_t offset, int whence);
    /** Ioctl method. Default implementation returns error. */
    virtual int ioctl(File *, unsigned long int, unsigned long);

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    virtual bool select(File* file, int mode);

    /** Open a file or device.
     * @param reent thread save reentrant structure
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode, ignored in this implementation
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int open(struct _reent *reent, const char *path, int flags,
                    int mode);

protected:
    /** Select wakeup information.
     */
    struct SelectInfo
    {
        /** Default constructor.
         */
        SelectInfo()
            : event(0)
        {
        }

        /** bit mask of clients that need woken */
        OSEventType event;
    };

    /** Add client to list of clients needing woken.
     * @param info wakeup event instance
     */
    static void select_insert(SelectInfo *info);

    /** Wakeup the list of clients needing woken.
     * @param info wakeup event instance
     */
    static void select_wakeup(SelectInfo *info);

    /** Wakeup the list of clients needing woken. from ISR context.
     * @param info wakeup event instance
     * @param woken is the task woken up
     */
    static void select_wakeup_from_isr(SelectInfo *info, int *woken);

private:
    const char *name; /**< device name */

    /** first device in linked list */
    static Device *first;

    /** mutual exclusion for fileio */
    static OSMutex mutex;

    /** next device in linked list */
    Device *next;

    DISALLOW_COPY_AND_ASSIGN(Device);
};

/** Node information.
 */
class Node : public Device
{
protected:
    /** Constructor.
     */
    Node(const char *name)
        : Device(name)
        , references_(0)
    {
    }

    /** Destructor */
    ~Node()
    {
    }

    /** This will be called once when reference-count goes from 0 to
     * positive. Called with lock_ held. */
    virtual void enable() = 0;
    /** This will be called when reference count goes from non-zero to
     * 0. Called with lock_ held. */
    virtual void disable() = 0;

    /** Instructs the device driver to drop all TX and RX queues. This is
     * called after disable() still under the device lock. */
    virtual void flush_buffers() = 0;

    /** Open method */
    int open(File *, const char *, int, int) OVERRIDE;
    /** Close method */
    int close(File *) OVERRIDE;

protected:
    OSMutex lock_;

private:
    unsigned int references_; /**< number of open references */

    DISALLOW_COPY_AND_ASSIGN(Node);
};


/** Node information.
 */
class NonBlockNode : public Node
{
protected:
    NonBlockNode(const char *name)
        : Node(name)
        , readableNotify_(NULL)
        , writableNotify_(NULL) {}

    /** Called under a critical section. @returns true if a read would not block
     * right now. */
    virtual bool has_rx_buffer_data() = 0;
    /** Called under a critical section. @returns true if a write would not
     * block right now. */
    virtual bool has_tx_buffer_space() = 0;

    /** Request an ioctl transaction
    * @param file file reference for this device
    * @param node node reference for this device
    * @param key ioctl key
    * @param data key data
    */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    /** This will be notified if the device has data avilable for read. */
    Notifiable* readableNotify_;
    /** This will be notified if the device has buffer avilable for write. */
    Notifiable* writableNotify_;
};

/** File information.
 */
struct File
{
    Device *dev; /**< file operations */
    /** Data that the device driver wants to store about this fd. */
    void *priv;
    off_t offset; /**< current offset within file */
    int flags;    /**< open flags */
    char inuse;   /**< non-zero if this is an open fd. */
};

#endif /* _FREERTOS_DRIVERS_COMMON_DEVTAB_HXX_ */
