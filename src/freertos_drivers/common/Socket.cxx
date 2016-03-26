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

#include <cstdlib>
#include <cstdint>
#include <sys/socket.h>
#include <sys/stat.h>

#include "Devtab.hxx"
#include "Socket.hxx"
#include "can_ioctl.h"

/** Read from a file or device.
 * @param file file reference for this device
 * @param buf location to place read data
 * @param count number of bytes to read
 * @return number of bytes read upon success, negative errno containing
 *         the cause
 */
ssize_t Socket::read(File *file, void *buf, size_t count)
{
    ssize_t result = ::recv(fd_lookup(file), buf, count, 0);
    if (result < 0)
    {
        return -errno;
    }
    return result;
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
    ssize_t result = ::send(fd_lookup(file), buf, count, 0);
    if (result < 0)
    {
        return -errno;
    }
    return result;
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

extern "C"
{
int socket(int domain, int type, int protocol)
    __attribute__ ((weak));

int bind(int socket, const struct sockaddr *address, socklen_t address_len)
    __attribute__ ((weak));

int listen(int socket, int backlog)
    __attribute__ ((weak));

int accept(int socket, struct sockaddr *address, socklen_t *address_len)
    __attribute__ ((weak));

int connect(int socket, const struct sockaddr *address, socklen_t address_len)
    __attribute__ ((weak));

ssize_t recv(int socket, void *buffer, size_t length, int flags)
    __attribute__ ((weak));

ssize_t send(int socket, const void *buffer, size_t length, int flags)
    __attribute__ ((weak));

int setsockopt(int socket, int level, int option_name,
               const void *option_value, socklen_t option_len)
    __attribute__ ((weak));

int getsockopt(int socket, int level, int option_name,
               void *option_value, socklen_t *option_len)
    __attribute__ ((weak));

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
    errno = EINVAL;
    return -1;
}

/** Bind a name to a socket.
 * @param socket file descriptor of the socket to be bound
 * @param address points to a sockaddr structure containing the address to
 *                be bound to the socket
 * @param address_len specifies the length of the sockaddr structure pointed
 *                    to by the address argument
 * @return shall return on success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int bind(int socket, const struct sockaddr *address, socklen_t address_len)
{
    errno = EINVAL;
    return -1;
}

/** Mark a connection-mode socket, specified by the socket argument, as
 * accepting connections.
 * @param socket the socket file descriptor
 * @param backlog provides a hint to the implementation which the
 *                implementation shall use to limit the number of
 *                outstanding connections in the socket's listen queue
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int listen(int socket, int backlog)
{
    errno = EINVAL;
    return -1;
}

/** Accept a new connection on a socket.
 * @param socket the socket file descriptor
 * @param address either a null pointer, or a pointer to a sockaddr
 *                structure where the address of the connecting socket
 *                shall be returned
 * @param address_len either a null pointer, if address is a null pointer,
 *                    or a pointer to a socklen_t object which on input
 *                    specifies the length of the supplied sockaddr
 *                    structure, and on output specifies the length of the
 *                    stored address
 * @return shall return the non-negative file descriptor of the accepted
 *         socket upon success, otherwise, -1 shall be returned and errno
 *         set to indicate the error
 */
int accept(int socket, struct sockaddr *address, socklen_t *address_len)
{
    errno = EINVAL;
    return -1;
}

/** Connect a socket.
 * @param socket the socket file descriptor
 * @param address points to a sockaddr structure containing the peer address
 * @param address_len specifies the length of the sockaddr structure pointed
 *                    to by the address argument
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int connect(int socket, const struct sockaddr *address, socklen_t address_len)
{
    errno = EINVAL;
    return -1;
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
    errno = EINVAL;
    return -1;
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
    errno = EINVAL;
    return -1;
}

/** Set the socket options.
 * @param socket the socket file descriptor
 * @param level specifies the protocol level at which the option resides
 * @param option_name specifies a single option to set
 * @param option_value the metadata that belongs to the option_name
 * @param option_len the length of the metadata that belongs to the
 *                   option_name
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int setsockopt(int socket, int level, int option_name,
               const void *option_value, socklen_t option_len)
{
    errno = EINVAL;
    return -1;
}

/** Get the socket options.
 * @param socket the socket file descriptor
 * @param level specifies the protocol level at which the option resides
 * @param option_name specifies a single option to get
 * @param option_value the metadata that belongs to the option_name
 * @param option_len the length of the metadata that belongs to the
 *                   option_name
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int getsockopt(int socket, int level, int option_name,
               void *option_value, socklen_t *option_len)
{
    errno = EINVAL;
    return -1;
}
} /* extern "C" */
