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
 * \file Socket.cxx
 * This file implements a generic socket device driver layer.
 *
 * @author Stuart W. Baker
 * @date 17 March 2016
 */

#include <cstdint>
#include <fcntl.h>
#include "Devtab.hxx"
#include "Socket.hxx"
#include "can_ioctl.h"

/** Close method. Returns negative errno on failure.
 * @param file reference to close
 * @return 0 upon success or negative error number upon error.
 */
int Socket::close(File *file)
{
    mutex.lock();
    if (--references_ == 0)
    {
        mutex.unlock();
        delete file->dev;
    }
    else
    {
        mutex.unlock();
    }

    return 0;
}

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, negative errno containing
 *         the cause
 */
ssize_t Socket::read(File *file, void *buf, size_t count)
{
    return recv(file, buf, count, 0);
}

/** Write to a file or device.
 * @param file file reference for this device
 * @param buf location to find write data
 * @param count number of bytes to write
 * @return number of bytes written upon success, negative errno containing
 *         the cause
 */
ssize_t Socket::write(File *file, const void *buf, size_t count)
{
    ssize_t result = 0;

    return result;
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
ssize_t Socket::recv(File *file, void *buffer, size_t length, int flags)
{
    return 0;
}

/** Initiate transmission of a message from the specified socket.
 * @param file file reference for this device
 * @param buffer buffer containing the message to send
 * @param length length of the message in bytes
 * @param flags the type of message transmission
 * @return the number of bytes sent, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
ssize_t send(File *file, const void *buffer, size_t length, int flags)
{
    return 0;
}

/** Get the status information of a file or device.
 * @param file file reference for this device
 * @param stat structure to fill status info into
 * @return 0 upon successor or negative error number upon error.
 */
int Socket::fstat(File* file, struct stat *stat)
{
    memset(stat, 0, sizeof(stat));
    stat->st_mode = S_IFSOCK;
    return 0;
}

/** Request an ioctl transaction
 * @param file file reference for this device
 * @param node node reference for this device
 * @param key ioctl key
 * @param data key data
 */
int Socket::ioctl(File *file, unsigned long int key, unsigned long data)
{
    return -1;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool Socket::select(File *file, int mode)
{
    bool retval = false;

    return retval;
}

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
int Socket::socket(int domain, int type, int protocol)
{
    Socket *new_socket = new Socket(NULL);

    mutex.lock();
    int fd = fd_alloc();
    mutex.unlock();
    if (fd < 0)
    {
        errno = EMFILE;
        return -1;
    }

    File *file = file_lookup(fd);

    file->dev = new_socket;
    file->flags = O_RDWR;

    new_socket->references_ = 1;

    return fd;
}

/** Receive a message from a connection-mode or connectionless-mode socket.
 * @param socket the socket file descriptor
 * @param buffer buffer where the message should be stored
 * @param length length in bytes of the buffer pointed to by the buffer
 *               argument
 * @param flags Specifies the type of message reception
 * @return the length of the message in bytes, if no messages are available
 *         to be received and the peer has performed an orderly shutdown,
 *         recv() shall return 0. Otherwise, -1 shall be returned and errno
 *         set to indicate the error
 */
ssize_t Socket::recv(int socket, void *buffer, size_t length, int flags)
{
    File* f = file_lookup(socket);
    Socket *s = static_cast<Socket *>(f->dev);
    if (!f) 
    {
        errno = EBADF;
        return -1;
    }
    int result = s->recv(f, buffer, length, flags);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;  
}

/** Initiate transmission of a message from the specified socket.
 * @param socket the socket file descriptor
 * @param buffer buffer containing the message to send
 * @param length length of the message in bytes
 * @param flags the type of message transmission
 * @return the number of bytes sent, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
ssize_t Socket::send(int socket, const void *buffer, size_t length, int flags)
{
    File* f = file_lookup(socket);
    Socket *s = static_cast<Socket *>(f->dev);
    if (!f) 
    {
        errno = EBADF;
        return -1;
    }
    int result = s->send(f, buffer, length, flags);
    if (result < 0)
    {
        errno = -result;
        return -1;
    }
    return result;  
}

extern "C"
{
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
int socket(int domain, int type, int protocol)
{
    return Socket::socket(domain, type, protocol);
}

/** Receive a message from a connection-mode or connectionless-mode socket.
 * @param socket the socket file descriptor
 * @param buffer buffer where the message should be stored
 * @param length length in bytes of the buffer pointed to by the buffer
 *               argument
 * @param flags Specifies the type of message reception
 * @return the length of the message in bytes, if no messages are available
 *         to be received and the peer has performed an orderly shutdown,
 *         recv() shall return 0. Otherwise, -1 shall be returned and errno
 *         set to indicate the error
 */
ssize_t recv(int socket, void *buffer, size_t length, int flags)
{
    return Socket::recv(socket, buffer, length, flags);
}

/** Initiate transmission of a message from the specified socket.
 * @param socket the socket file descriptor
 * @param buffer buffer containing the message to send
 * @param length length of the message in bytes
 * @param flags the type of message transmission
 * @return the number of bytes sent, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
ssize_t send(int socket, const void *buffer, size_t length, int flags)
{
    return Socket::send(socket, buffer, length, flags);
}

}
