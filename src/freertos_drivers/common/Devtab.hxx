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

#include <dirent.h>
#include <stropts.h>
#include <sys/types.h>
#include <sys/select.h>
#include "os/OS.hxx"

class FileIO;
class Device;
class FileSystem;
class Notifiable;
class DeviceBufferBase;

/** File information.
 */
struct File
{
    FileIO *dev; /**< file operations */
    /** Data that the device driver wants to store about this fd. */
    union
    {
        void *priv; /**< file reference specific data "pointer" */
        void *privPtr; /**< file reference specific data "pointer" */
        unsigned privUint; /**< file reference specific data "unsigned" */
        int privInt; /**< file reference specific data "int" */
    };
    off_t offset; /**< current offset within file */
    int flags;    /**< open flags */
    uint8_t inuse  : 1; /**< true if this is an open fd. */
    uint8_t inshdn : 1; /**< true if this fd is in shutdown. */
    uint8_t device : 1; /**< true if this is a device, false if file system */
    uint8_t dir    : 1; /**< true if this is a directory, else false */
    uint8_t dirty  : 1; /**< true if this file is dirty and needs flush */
};

/** Base class for both Device and FileSystem objects */
class FileIO
{
public:
    /** Read from a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to read
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     *         containing the cause
     */
    static ssize_t read(struct _reent *reent, int fd, void *buf, size_t count);

    /** Write to a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to write
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno
     *         containing the cause
     */
    static ssize_t write(struct _reent *reent, int fd, const void *buf, size_t count);

    /** Change the offset index of a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to seek
     * @param offset offset within file
     * @param whence type of seek to complete
     * @return resulting offset from beginning of file, -1 upon failure with
     *         errno containing the cause
     */
    static _off_t lseek(struct _reent *reent, int fd, _off_t offset, int whence);

    /** Get the status information of a file or device.
     * @param reent thread safe reentrant structure
     * @param fd file descriptor to get status of
     * @param stat structure to fill status info into
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int fstat(struct _reent *reent, int fd, struct stat *stat);

    /** Request and ioctl transaction.
     * @param fd file descriptor
     * @param key ioctl key
     * @param data key data
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int ioctl(int fd, unsigned long int key, unsigned long data);

    /** Manipulate a file descriptor.
     * @param fd file descriptor
     * @param cmd operation to perform
     * @param data parameter to the cmd operation
     * @return dependent on the operation (POSIX compliant where applicable) or
     *         -1 on error with errno set appropriately
     */
    static int fcntl(int fd, int cmd, unsigned long data);

    /** Test if the file descriptor belongs to a device.
     * @param fd file descriptor
     * @return true if fd belongs to a device, false if belongs to a file system
     */
    static bool is_device(int fd)
    {
        return file_lookup(fd)->device;
    }

protected:
    /** Constructor.
     * @param name name of mount point in the root file system.
     *             Pointer must be valid throughout the entire lifetime.
     */
    FileIO(const char *name)
        : name(name)
    {
    }

    /** Destructor */
    virtual ~FileIO()
    {
    }

    /** Open a file or device.
     * @param file file reference for this device
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode, ignored in this implementation
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    virtual int open(File *file, const char *path, int flags, int mode) = 0;

    /** Close a file or device.
     * @param file file reference for this device
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    virtual int close(File *file) = 0;

    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, -1 upon failure with errno
     *         containing the cause
     */
    virtual ssize_t read(File *file, void *buf, size_t count) = 0;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, -1 upon failure with errno
     *         containing the cause
     */
    virtual ssize_t write(File *file, const void *buf, size_t count) = 0;

    /** Seek method.
     * @param f file reference for this device
     * @param offset offset in bytes from whence directive
     * @param whence SEEK_SET if to set the file offset to an abosolute position,
     *               SEEK_CUR if to set the file offset from current position
     * @return current offest or negative error number upon error.
     */
    virtual off_t lseek(File* f, off_t offset, int whence);

    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    virtual int fstat(File* file, struct stat *stat) = 0;

    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param key ioctl key
     * @param data key data
     * @return 0 upon success or negative error number upon error.
     */
    virtual int ioctl(File *file, unsigned long int key, unsigned long data);

    /** Manipulate a file descriptor.
     * @param file file reference for this device
     * @param cmd operation to perform
     * @param data parameter to the cmd operation
     * @return dependent on the operation (POSIX compliant where applicable)
     *         or negative error number upon error.
     */
    virtual int fcntl(File *file, int cmd, unsigned long data);

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    virtual bool select(File* file, int mode)
    {
        return true;
    }

    /** Allocate a free file descriptor.  This call must be made with the
     * static Device::mutex locked.
     * @return file number on success, else -1 on failure
     */
    static int fd_alloc(void);

    /** Free up a file descriptor.
     * @param fd number to free up
     */
    static void fd_free(int fd);

    /** Looks up a reference to a File corresponding to a given file descriptor.
     * @param fd is a file descriptor as supplied to the read-write-close-ioctl
     *        commands.
     * @returns NULL and sets errno if fd is invalid, otherwise the File
     *          reference.
     */
    static File* file_lookup(int fd);

    /** Looks up a file descriptor corresponding to a given File reference.
     * @param file is a reference to a File structure.
     * @returns file descriptor (assert on error).
     */
    static int fd_lookup(File *file);

    /** @return the maximum number of open file descriptors possible (the size
     * of the files[] array. */
    static const unsigned int numOpenFiles;
    
    /** File descriptor pool */
    static File files[];

    /** mutual exclusion for fileio */
    static OSMutex mutex;

    const char *name; /**< device name */

    /** Allow access from Device class */
    friend class Device;

    /** Allow access from FileSystem class */
    friend class FileSystem;

private:
    DISALLOW_COPY_AND_ASSIGN(FileIO);
};

/** Base class for all File systems. */
class FileSystem : public FileIO
{
public:
    /** Constructor.
     */
    FileSystem();

    /** Destructor */
    virtual ~FileSystem();

    /** Mount the file system.
     * @param mount_point path in the root file system for the mount point
     *                    Must not contain any trailing '/' characters, e.g.
     *                    "/usr", not "/usr/".  Unlike in linux, each mount
     *                    must be entirely unique point in the system.  For
     *                    example, mounting of both "/usr" and "/usr/bin" is
     *                    is not supported.  However, "/usr" and "/usr1" is
     *                    supported.  This saves on file open logic.
     */
    virtual void mount(const char *mount_point) = 0;

    /** Format the file system, all data will be lost.  The file system must
     * not be mounted at the time of calling this.
     */
    virtual void format() = 0;

    /** Open a file or device.
     * @param reent thread save reentrant structure
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode, ignored in this implementation
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int open(struct _reent *reent, const char *path, int flags,
                    int mode);

    /** Close a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to close
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int close(struct _reent *reent, int fd);

    /** Remove a file.
     * @param reent thread safe reentrant structure
     * @param path file name
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int unlink(struct _reent *reent, const char *path);

    /** Get the status information of a file or device.
     * @param reent thread safe reentrant structure
     * @param path file or device name
     * @param stat structure to fill status info into
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int stat(struct _reent *reent, const char *path, struct stat *stat);

    /** Synchronize (flush) a file to disk.
     * @param fd file descriptor to sync
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int fsync(int fd);

    /** Close a directory.
     * @param @dirp directory pointer to close
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int closedir(DIR *dirp);

    /** Open a directory.
     * @param name directory path
     * @return pointer to the open directory on success, NULL on error
     */
    static DIR *opendir(const char *name);

    /** Read the next entry in a directory.
     * @param dirp directory pointer to read.
     * @return pointer to a struct dirent representing the next directectory
     *         entry
     */
    static struct dirent *readdir(DIR *dirp);

protected:
    /** Remove a file.
     * @param path file name
     * @return 0 upon successor or negative error number upon error.
     */
    virtual int unlink(const char *path) = 0;

    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    virtual int fstat(File* file, struct stat *stat) override;

    /** Get the status information of a file or device.
     * @param path file or device name
     * @param stat structure to fill status info into
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    virtual int stat(const char *path, struct stat *stat) = 0;

    /** Synchronize (flush) a file to disk.
     * @param file file reference for this device
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    virtual int fsync(File *file) = 0;

    /** Close a directory.
     * @param file file reference for this device
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    virtual int closedir(File *file) = 0;

    /** Open a directory.
     * @param file file reference for this device
     * @param name directory path
     * @return pointer to the open directory on success, NULL on error
     */
    virtual File *opendir(File *file, const char *name) = 0;

    /** Read the next entry in a directory.
     * @param file file reference for this device
     * @return pointer to a struct dirent representing the next directectory
     *         entry
     */
    virtual struct dirent *readdir(File *file) = 0;

private:
    /** Locate the file system for a given path.
     * @param path full path to file/directory
     * @return reference to file system on success, else nullptr
     */
    static FileSystem *fs_lookup(const char *path);

    /** first device in linked list */
    static FileSystem *first;

    /** next device in linked list */
    FileSystem *next;

    /** previous device in linked list */
    FileSystem *prev;

    DISALLOW_COPY_AND_ASSIGN(FileSystem);
};


/** Device tab structure.
 */
struct Device : public FileIO
{
public:
    /** Constructor.
     * @param name name of device in file system. Pointer must be valid
     * throughout the entire lifetime.
     */
    Device(const char *name);

    /** Destructor */
    virtual ~Device();

    /** Open a file or device.
     * @param reent thread save reentrant structure
     * @param path file or device name
     * @param flags open flags
     * @param mode open mode, ignored in this implementation
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int open(struct _reent *reent, const char *path, int flags,
                    int mode);

    /** Close a file or device.
     * @param reent thread save reentrant structure
     * @param fd file descriptor to close
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int close(struct _reent *reent, int fd);

    /** Get the status information of a file or device.
     * @param reent thread save reentrant structure
     * @param path file or device name
     * @param stat structure to fill status info into
     * @return 0 upon success, -1 upon failure with errno containing the cause
     */
    static int stat(struct _reent *reent, const char *path, struct stat *stat);

    /** POSIX select().
     * @param nfds highest numbered file descriptor in any of the three,
     *             sets plus 1
     * @param readfds fd_set of file descritpors to pend on read active
     * @param writefds fd_set of file descritpors to pend on write active
     * @param exceptfds fd_set of file descritpors to pend on error active
     * @param timeout timeout in nsec to wait, if 0, return immediately, if < 0
     *                wait forever
     * @return on success, number of file descriptors in the three sets that are
     *         active, 0 on timeout, -1 with errno set appropriately upon error.
     */
    static int select(int nfds, fd_set *readfds, fd_set *writefds,
                      fd_set *exceptfds, long long timeout);

    /** Clears the current thread's select bits. This is used by ::select and
     * ::pselect to ensure the necessary atomicity.
     */
    static void select_clear();

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

    /** Get the mode of the device.
     * @return mode of device.
     */
    virtual mode_t get_mode()
    {
        return 0;
    }

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

    /** allow class DeviceBuffer access to select() related members. */
    friend class DeviceBufferBase;

    /** allow class OSSelectWakeup access to select() related members. */
    friend class OSSelectWakeup;

private:
    /** first device in linked list */
    static Device *first;

    /** next device in linked list */
    Device *next;

    /** previous device in linked list */
    Device *prev;

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
        , mode_(0)
        , references_(0)
    {
    }

    /** Destructor */
    virtual ~Node()
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

    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    virtual int fstat(File* file, struct stat *stat) override;

    OSMutex lock_; ///< protects internal structures.
    /// File open mode, such as O_NONBLOCK.
    mode_t mode_;

    unsigned int references_; /**< number of open references */

private:
    /** Get the mode of the device.
     * @return mode of device.
     */
    mode_t get_mode() override
    {
        return mode_;
    }

    DISALLOW_COPY_AND_ASSIGN(Node);
};


/** Node information for a device node in the filesystem that has support for
 * nonblocking mode via Notifiable pointers for reading and writing.
 */
class NonBlockNode : public Node
{
protected:
    /// Constructor. @param name is the name of this device node in the
    /// filesystem.
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
    * @param key ioctl key
    * @param data key data
    */
    int ioctl(File *file, unsigned long int key, unsigned long data) OVERRIDE;

    /** This will be notified if the device has data avilable for read. */
    Notifiable* readableNotify_;
    /** This will be notified if the device has buffer avilable for write. */
    Notifiable* writableNotify_;
};

#endif /* _FREERTOS_DRIVERS_COMMON_DEVTAB_HXX_ */
