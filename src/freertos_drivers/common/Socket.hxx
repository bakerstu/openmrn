/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file Socket.hxx
 * This file implements a generic socket device driver layer.
 *
 * @author Stuart W. Baker
 * @date 17 March 2016
 */

#ifndef _FREERTOS_DRIVERS_COMMON_SOCKET_HXX_
#define _FREERTOS_DRIVERS_COMMON_SOCKET_HXX_

#include "Devtab.hxx"
#include "os/OS.hxx"
#include "DeviceBuffer.hxx"

/** Private data for a serial device */
class Socket : public Node
{
public:
    /** Create an unbound socket in a communications domain.
     * @param domain specifies the communications domain in which a socket is
     *               to be created
     * @param type specifies the type of socket to be created
     * @param protocol specifies a particular protocol to be used with the
     *                 socket, specifying a protocol of 0 causes socket() to
     *                 use an unspecified default protocol appropriate for the
     *                 requested socket type
     * @return a non-negative integer on success, the socket file descriptor,
     *         otherwise, a value of -1 shall be returned and errno set to
     *         indicate the error
     */
    static int socket(int domain, int type, int protocol);

    /** Receive a message from a connection-mode or connectionless-mode socket.
     * @param socket the socket file descriptor
     * @param buffer buffer where the message should be stored
     * @param length length in bytes of the buffer pointed to by the buffer
     *               argument
     * @param flags Specifies the type of message reception
     * @return the length of the message in bytes, if no messages are available
     *         to be received and the peer has performed an orderly shutdown,
     *         recv() shall return 0, otherwise, -1 shall be returned and errno
     *         set to indicate the error
     */
    static ssize_t recv(int socket, void *buffer, size_t length, int flags);

    /** Initiate transmission of a message from the specified socket.
     * @param socket the socket file descriptor
     * @param buffer buffer containing the message to send
     * @param length length of the message in bytes
     * @param flags the type of message transmission
     * @return the number of bytes sent, otherwise, -1 shall be returned and
     *         errno set to indicate the error
     */
    static ssize_t send(int socket, const void *buffer, size_t length, int flags);

protected:
    /** Request an ioctl transaction
     * @param file file reference for this device
     * @param node node reference for this device
     * @param key ioctl key
     * @param data key data
     */
    int ioctl(File *file, unsigned long int key, unsigned long data) override;
    
    /** Device select method. Default implementation returns true.
     * @param file reference to the file
     * @param mode FREAD for read active, FWRITE for write active, 0 for
     *        exceptions
     * @return true if active, false if inactive
     */
    bool select(File* file, int mode) override;

    SelectInfo selInfoRd; /**< select wakeup metadata for read active */
    SelectInfo selInfoWr; /**< select wakeup metadata for write active */

private:
    /** Close method. Returns negative errno on failure.
     * @param file reference to close
     * @return 0 upon success or negative error number upon error.
     */
    int close(File *file) override;

    /** Read from a file or device.
     * @param file file reference for this device
     * @param buf location to place read data
     * @param count number of bytes to read
     * @return number of bytes read upon success, negative errno containing
     *         the cause
     */
    ssize_t read(File *file, void *buf, size_t count) override;

    /** Write to a file or device.
     * @param file file reference for this device
     * @param buf location to find write data
     * @param count number of bytes to write
     * @return number of bytes written upon success, negative errno containing
     *         the cause
     */
    ssize_t write(File *file, const void *buf, size_t count) override;

    /** Seek method.  Not valid for a socket.
     * @param file file reference for this device
     * @param offset offset in bytes from whence directive
     * @param whence SEEK_SET if to set the file offset to an abosolute position,
     *               SEEK_CUR if to set the file offset from current position
     * @return -ESPIPE
     */
    off_t lseek(File* f, off_t offset, int whence) override
    {
        return (off_t)-ESPIPE;
    }

    /** Receive a message from a connection-mode or connectionless-mode socket.
     * @param file file reference for this device
     * @param buffer buffer where the message should be stored
     * @param length length in bytes of the buffer pointed to by the buffer
     *               argument
     * @param flags Specifies the type of message reception
     * @return the length of the message in bytes, if no messages are available
     *         to be received and the peer has performed an orderly shutdown,
     *         recv() shall return 0. Otherwise, -1 shall be returned and errno
     *         set to indicate the error
     */
    ssize_t recv(File *file, void *buffer, size_t length, int flags);

    /** Initiate transmission of a message from the specified socket.
     * @param file file reference for this device
     * @param buffer buffer containing the message to send
     * @param length length of the message in bytes
     * @param flags the type of message transmission
     * @return the number of bytes sent, otherwise, -1 shall be returned and
     *         errno set to indicate the error
     */
    ssize_t send(File *file, const void *buffer, size_t length, int flags);

    /** Get the status information of a file or device.
     * @param file file reference for this device
     * @param stat structure to fill status info into
     * @return 0 upon successor or negative error number upon error.
     */
    int fstat(File* file, struct stat *stat) override;

    /** Constructor
     * @param name device name in file system
     */
    Socket(const char *name)
        : Node(name)
        , selInfoRd()
        , selInfoWr()
    {
    }    

    /** Destructor.
     */
    ~Socket()
    {
    }

    void enable() override {} /**< function to enable device */
    void disable() override {}; /**< function to disable device */

    /** Discards all pending buffers. Called after disable(). */
    void flush_buffers() override {};

    DISALLOW_COPY_AND_ASSIGN(Socket);
};

#endif /* _FREERTOS_DRIVERS_COMMON_SOCKET_HXX_ */
