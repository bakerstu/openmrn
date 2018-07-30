/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * \file Pipe.cxx
 * This file implements POSIX pipe().
 *
 * @author Stuart W. Baker
 * @date 27 January 2015
 */

#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>

#include "Devtab.hxx"
#include "utils/constants.hxx"
#include "utils/RingBuffer.hxx"

/// Default value of the buffer size in the pipe implementation for FreeRTOS.
const size_t DEFAULT_PIPE_SIZE = 256;

/** Private data for a can device */
class Pipe : public Node
{
public:
    /** Constructor
     * @param name device name in file system
     */
    Pipe(const char *name)
        : Node(name)
        , selInfoRd()
        , selInfoWr()
        , ring(NULL)
        , size(DEFAULT_PIPE_SIZE)
    {
        mode_ = S_IFIFO;
    }    

    /** Destructor.
     */
    ~Pipe()
    {
        if (ring)
        {
            ring->destroy();
        }
    }

    /** Create a Unix style pipe.
     * @param pipefds index 0, file descriptor open for reading.
     *                index 1, file descriptor open for writing.
     * @return 0 upon success, -1 on errer with errno set appropriately
     */
    static int pipe(int pipefds[2]);

private:
    /** Close method. Returns negative errno on failure.
     * @param file reference to close
     * @return 0 upon success or negative error number upon error.
     */
    int close(File *file) OVERRIDE;

    /** Read from a file or device.
    * @param file file reference for this device
    * @param buf location to place read data
    * @param count number of bytes to read
    * @return number of bytes read upon success, -1 upon failure with errno containing the cause
    */
    ssize_t read(File *file, void *buf, size_t count) OVERRIDE;

    /** Write to a file or device.
    * @param file file reference for this device
    * @param buf location to find write data
    * @param count number of bytes to write
    * @return number of bytes written upon success, -1 upon failure with errno containing the cause
    */
    ssize_t write(File *file, const void *buf, size_t count) OVERRIDE;

    /** Seek method.  Not valid for a socket.
     * @param f file reference for this device
     * @param offset offset in bytes from whence directive
     * @param whence SEEK_SET if to set the file offset to an abosolute position,
     *               SEEK_CUR if to set the file offset from current position
     * @return -ESPIPE
     */
    off_t lseek(File* f, off_t offset, int whence) override
    {
        return (off_t)-ESPIPE;
    }

    /** Manipulate a file descriptor.
     * @param file file reference for this device
     * @param cmd operation to perform
     * @param data parameter to the cmd operation
     * @return dependent on the operation (POSIX compliant where applicable)
     *         or negative error number upon error.
     */
    int fcntl(File *file, int cmd, unsigned long data) OVERRIDE;

    /** Device select method. Default impementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) OVERRIDE;

    void enable() OVERRIDE {} /**< function to enable device */
    void disable() OVERRIDE {}; /**< function to disable device */
    void flush_buffers() OVERRIDE {}; /**< called after disable */

    SelectInfo selInfoRd; /**< select wakeup metadata for read active */
    SelectInfo selInfoWr; /**< select wakeup metadata for write active */

    RingBuffer<uint8_t> *ring; /**< ring buffer for storing the data */

    size_t size; /**< pipe size */

    DISALLOW_COPY_AND_ASSIGN(Pipe);
};

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, -1 upon failure with errno containing the cause
 */
ssize_t Pipe::read(File *file, void *buf, size_t count)
{
    if ((file->flags & O_ACCMODE) == O_WRONLY)
    {
        return -EBADF;
    }

    uint8_t *data = (uint8_t*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        size_t bytes;
        {
            OSMutexLock locker(&lock_);
            if (ring == NULL)
            {
                ring = RingBuffer<uint8_t>::create(size);
            }
            bytes = ring->get(data, count);

            count -= bytes;
            result += bytes;
            data += bytes;
        }

        if (bytes)
        {
            select_wakeup(&selInfoWr);
        }

        if (count)
        {
            /* no more data to receive */
            if (file->flags & O_NONBLOCK || result > 0)
            {
                break;
            }
            else
            {
                /* blocking mode, wait for writer */
                fd_set rdfds;
                FD_ZERO(&rdfds);
                int fd = fd_lookup(file);
                FD_SET(fd, &rdfds);
                ::select(fd + 1, &rdfds, NULL, NULL, NULL);
            }
        }
    }
    
    return result;
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, -1 upon failure with errno containing the cause
 */
ssize_t Pipe::write(File *file, const void *buf, size_t count)
{
    if ((file->flags & O_ACCMODE) == O_RDONLY)
    {
        return -EBADF;
    }

    const uint8_t *data = (const uint8_t*)buf;
    ssize_t result = 0;
    
    while (count)
    {
        size_t bytes;
        {
            OSMutexLock locker(&lock_);
            if (ring == NULL)
            {
                ring = RingBuffer<uint8_t>::create(size);
            }
            bytes = ring->put(data, count);

            count -= bytes;
            result += bytes;
            data += bytes;
        }

        if (bytes)
        {
            select_wakeup(&selInfoRd);
        }

        if (count)
        {
            /* no more room left */
            if (file->flags & O_NONBLOCK || result > 0)
            {
                break;
            }
            else
            {
                /* blocking mode, wait for reader */
                fd_set wrfds;
                FD_ZERO(&wrfds);
                int fd = fd_lookup(file);
                FD_SET(fd, &wrfds);
                ::select(fd + 1, NULL, &wrfds, NULL, NULL);
            }
        }
    }
    
    return result;
}

/** Close method. Returns negative errno on failure.
 * @param file reference to close
 * @return 0 upon success or negative error number upon error.
 */
int Pipe::close(File *file)
{
    mutex.lock();
    if (--references_ == 0)
    {
        mutex.unlock();
        HASSERT(file->device);
        delete static_cast<Device *>(file->dev);
    }
    else
    {
        mutex.unlock();
    }

    return 0;
}

/** Manipulate a file descriptor.
 * @param file file reference for this device
 * @param cmd operation to perform
 * @param data parameter to the cmd operation
 * @return dependent on the operation (POSIX compliant where applicable)
 *         or negative error number upon error.
 */
int Pipe::fcntl(File *file, int cmd, unsigned long data)
{
    switch (cmd)
    {
        default:
            return 0;
        case F_SETPIPE_SZ:
            size = data;
            return 0;
    }
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Pipe::select(File* file, int mode)
{
    bool retval = false;
    switch (mode)
    {
        case FREAD:
            portENTER_CRITICAL();
            if (ring->items())
            {
                retval = true;
            }
            else
            {
                select_insert(&selInfoRd);
            }
            portEXIT_CRITICAL();
            break;
        case FWRITE:
            portENTER_CRITICAL();
            if (ring->space())
            {
                retval = true;
            }
            else
            {
                select_insert(&selInfoWr);
            }
            portEXIT_CRITICAL();
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    return retval;
}


/** Create a Unix style pipe.
 * @param pipefds index 0, file descriptor open for reading.
 *                index 1, file descriptor open for writing.
 * @return 0 upon success, -1 on errer with errno set appropriately
 */
int Pipe::pipe(int pipefds[2])
{
    Pipe *new_pipe = new Pipe(NULL);

    mutex.lock();
    pipefds[0] = fd_alloc();
    if (pipefds[0] < 0)
    {
        mutex.unlock();
        errno = EMFILE;
        return -1;
    }
    pipefds[1] = fd_alloc();
    mutex.unlock();
    if (pipefds[1] < 0)
    {
        fd_free(pipefds[0]);
        errno = EMFILE;
        return -1;
    }

    File *files[2] = {file_lookup(pipefds[0]), file_lookup(pipefds[1])};

    files[0]->dev = new_pipe;
    files[1]->dev = new_pipe;
    files[0]->flags = O_RDONLY;
    files[1]->flags = O_WRONLY;

    new_pipe->references_ = 2;
    new_pipe->enable();

    return 0;
}

/** Create a Unix style pipe.
 * @param pipefds index 0, file descriptor open for reading.
 *                index 1, file descriptor open for writing.
 * @return 0 upon success, -1 on errer with errno set appropriately
 */
int pipe(int pipefds[2])
{
    return Pipe::pipe(pipefds);
}
