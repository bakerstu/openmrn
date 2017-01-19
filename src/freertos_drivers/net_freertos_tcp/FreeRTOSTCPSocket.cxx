/** \copyright
 * Copyright (c) 2016, Sidney McHarg
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
 * \file FreeRTOSTCPSocket.cxx
 * This file provides the sockets interface to the FreeRTOSPlus TCP stack.
 * Based on work by Stuart W. Baker
 *
 * @author Sidney McHarg
 * @date 21 March 2016
 */

#include <fcntl.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// FreeRTOSPTCP includes
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

#include "FreeRTOSTCPSocket.hxx"
#include "FreeRTOSTCP.hxx"

static FreeRTOSTCPSocket *FreeRTOSTCPSockets[MAX_SOCKETS];

/*
 * FreeRTOSTCPSocket::socket()
 */
int FreeRTOSTCPSocket::socket(int domain, int type, int protocol)
{
    FreeRTOSTCPSocket *new_socket = new FreeRTOSTCPSocket();
    if (new_socket == nullptr)
    {
        errno = ENOMEM;
        return -1;
    }

    mutex.lock();
    int fd = fd_alloc();
    mutex.unlock();
    if (fd < 0)
    {
        delete new_socket;
        errno = EMFILE;
        return -1;
    }

    switch (domain)
    {
        case AF_INET:
            domain = FREERTOS_AF_INET;
            break;
        default:
            fd_free(fd);
            errno = EAFNOSUPPORT;
            return -1;
    }

    switch (type)
    {
        case SOCK_STREAM:
            type = FREERTOS_SOCK_STREAM;
            break;
        case SOCK_DGRAM:
            type = FREERTOS_SOCK_DGRAM;
            break;
        default:
            fd_free(fd);
            errno = EINVAL;
            return -1;
    }

    switch (protocol)
    {
        case 0:
            break;
        case IPPROTO_TCP:
            protocol = FREERTOS_IPPROTO_TCP;
            break;
        case IPPROTO_UDP:
            protocol = FREERTOS_IPPROTO_UDP;
            break;
        default:
            fd_free(fd);
            errno = EINVAL;
            return -1;
    }

    Socket_t sd = FreeRTOS_socket(domain, type, protocol);

    if (sd == FREERTOS_INVALID_SOCKET)
    {
        // unable to allocate socket due to no memory
        errno = ENOMEM;
        fd_free(fd);
        return -1;
    }

    File *file = file_lookup(fd);

    file->dev = new_socket;
    file->flags = O_RDWR;
    file->priv = new_socket;

    new_socket->sd = sd;
    new_socket->references_ = 1;

    portENTER_CRITICAL();
    for (int i = 1; i < MAX_SOCKETS; ++i)
    {
        if (FreeRTOSTCPSockets[i] == NULL)
        {
            FreeRTOSTCPSockets[i] = new_socket;
            break;
        }
    }
    portEXIT_CRITICAL();

    return fd;
}

/*
 * FreeRTOSTCPSocket::bind()
 */
int FreeRTOSTCPSocket::bind(
    int socket, const struct sockaddr *address, socklen_t address_len)
{
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    struct freertos_sockaddr fr_address;
    fr_address.sin_len = sizeof(fr_address);

    switch (address->sa_family)
    {
        case AF_INET:
        	fr_address.sin_family = FREERTOS_AF_INET;
            break;
        default:
            errno = EINVAL;
            return -1;
    }

    struct sockaddr_in *sin = (struct sockaddr_in *)(address);
    fr_address.sin_addr = sin->sin_addr.s_addr;
    fr_address.sin_port = sin->sin_port;

    int result = FreeRTOS_bind(s->sd, &fr_address, sizeof(fr_address));

    if (result < 0)
    {
        switch (result)
        {
            case -FREERTOS_EINVAL:
                errno = EINVAL;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }

    return result;
}

/*
 * FreeRTOSTCPSocket::listen()
 */
int FreeRTOSTCPSocket::listen(int socket, int backlog)
{
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    int result = FreeRTOS_listen(s->sd, backlog);

    if (result < 0)
    {
        switch (result)
        {
            case -pdFREERTOS_ERRNO_EOPNOTSUPP:
                errno = EBADF;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }
    s->readActive = false;
    s->listenActive = true;

    return result;
}

/*
 * FreeRTOSTCPSocket::accept()
 */
int FreeRTOSTCPSocket::accept(
    int socket, struct sockaddr *address, socklen_t *address_len)
{
    int result;
    struct sockaddr_in *sin = (struct sockaddr_in *)(address);
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    if (!s->listenActive)
    {
        errno = EINVAL;
        return -1;
    }

    freertos_sockaddr fr_address;
    socklen_t fr_address_len = sizeof(fr_address);

    Socket_t sd = FreeRTOS_accept(s->sd, &fr_address, &fr_address_len);

    if (address && address_len && (*address_len >= sizeof(sockaddr_in)))
    {
        // copy the address across and set the address family
        sin->sin_port = fr_address.sin_port;
        sin->sin_addr.s_addr = fr_address.sin_addr;
        sin->sin_family = AF_INET;
        *address_len = sizeof(struct sockaddr_in);
    }

    if (sd == NULL)
    {
        // no queued connections
        errno = EAGAIN;
        return -1;
    }
    if (sd == FREERTOS_INVALID_SOCKET)
    {
        // bad socket or not listening
        errno = EBADF;
        return -1;
    }

    // for compatibility reset any timeout values inherited from listening
    // socket
    const TickType_t timeout = portMAX_DELAY;
    result = FreeRTOS_setsockopt(sd, 0, FREERTOS_SO_RCVTIMEO, &timeout, 0);
    if (result != 0)
    {
        // error detected
    }
    result = FreeRTOS_setsockopt(sd, 0, FREERTOS_SO_SNDTIMEO, &timeout, 0);
    if (result != 0)
    {
        // error detected
    }

    // set listening socket state
    s->readActive = false;

    return alloc_instance(sd);
}

/*
 * FreeRTOSTCPSocket::connect()
 */
int FreeRTOSTCPSocket::connect(
    int socket, const struct sockaddr *address, socklen_t address_len)
{
    const struct sockaddr_in *sin = (const struct sockaddr_in *)(address);
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    struct freertos_sockaddr fr_address;
    socklen_t fr_address_len = sizeof(fr_address);
    fr_address.sin_len = sizeof(fr_address);

    switch (sin->sin_family)
    {
        case AF_INET:
        	fr_address.sin_family = FREERTOS_AF_INET;
            break;
        default:
            errno = EINVAL;
            return -1;
    }

    fr_address.sin_addr = sin->sin_addr.s_addr;
    fr_address.sin_port = sin->sin_port;

    int result = FreeRTOS_connect(s->sd, &fr_address, fr_address_len);

    if (result)
    {
        switch (result)
        {
            case -pdFREERTOS_ERRNO_EBADF:
                errno = EBADF;
                break;
            case -pdFREERTOS_ERRNO_EISCONN:
                errno = EALREADY;
                break;
            case -pdFREERTOS_ERRNO_EINPROGRESS:
            case -pdFREERTOS_ERRNO_EAGAIN:
                errno = EAGAIN;
                break;
            case -FREERTOS_EWOULDBLOCK:
                errno = EWOULDBLOCK;
                break;
            case -pdFREERTOS_ERRNO_ETIMEDOUT:
                errno = ETIMEDOUT;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }
    return result;
}

/*
 * FreeRTOSTCPSocket::recv()
 */
ssize_t FreeRTOSTCPSocket::recv(
    int socket, void *buffer, size_t length, int flags)
{
    /* flags are not supported in FreeRTOSTCPSocket */
    HASSERT(flags == 0);

    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    int result = FreeRTOS_recv(s->sd, buffer, length, flags);

    if (result == 0)
    {
        s->readActive = false;
        errno = EAGAIN;
        return -1;
    }

    if (result < 0)
    {
        switch (result)
        {
            case -pdFREERTOS_ERRNO_ENOMEM:
                errno = ENOMEM;
                break;
            case -pdFREERTOS_ERRNO_ENOTCONN:
                errno = ENOTCONN;
                break;
            case -pdFREERTOS_ERRNO_EINTR:
                errno = EINTR;
                break;
            case -pdFREERTOS_ERRNO_EINVAL:
                errno = EINVAL;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }
    if ((size_t)result < length)
    {
        s->readActive = false;
    }
    else
    {
        s->readActive = true;
    }

    return result;
}

/*
 * FreeRTOSTCPSocket::send()
 */
ssize_t FreeRTOSTCPSocket::send(
    int socket, const void *buffer, size_t length, int flags)
{
    /* flags are not supported in FreeRTOSTCPSocket */
    HASSERT(flags == 0);

    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    int result = FreeRTOS_send(s->sd, buffer, length, flags);

    if (result < 0)
    {
        switch (result)
        {
            case -pdFREERTOS_ERRNO_ENOTCONN:
                errno = ENOTCONN;
                break;
            case -pdFREERTOS_ERRNO_ENOMEM:
                errno = ENOMEM;
                break;
            case -pdFREERTOS_ERRNO_EINVAL:
                errno = EINVAL;
                break;
            case -pdFREERTOS_ERRNO_ENOSPC:
                errno = ENOSPC;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }
    if ((size_t)result < length)
    {
        s->writeActive = false;
    }
    else
    {
        s->writeActive = true;
    }

    return result;
}

/*
 * FreeRTOSTCPSocket::setsockopt()
 */
int FreeRTOSTCPSocket::setsockopt(int socket, int level, int option_name,
    const void *option_value, socklen_t option_len)
{
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    int result;
    TickType_t timeout;
    const struct timeval *tm;

    switch (level)
    {
        default:
            errno = EINVAL;
            return -1;
        case SOL_SOCKET:
            switch (option_name)
            {
                case SO_REUSEADDR:
                    // ignore as FreeRTOS semantics different from BSD
                    return 0;
                case SO_RCVTIMEO:
                    tm = static_cast<const struct timeval *>(option_value);
                    timeout = pdMS_TO_TICKS(
                        (tm->tv_sec * 1000000 + tm->tv_usec) / 1000);
                    result = FreeRTOS_setsockopt(
                        s->sd, 0, FREERTOS_SO_RCVTIMEO, &timeout, 0);
                    if (result == -FREERTOS_EINVAL)
                    {
                        errno = EINVAL;
                        return -1;
                    }
                    return 0;
                case SO_SNDTIMEO:
                    tm = static_cast<const struct timeval *>(option_value);
                    timeout = pdMS_TO_TICKS(
                        (tm->tv_sec * 1000000 + tm->tv_usec) / 1000);
                    result = FreeRTOS_setsockopt(
                        s->sd, 0, FREERTOS_SO_SNDTIMEO, &timeout, 0);
                    if (result == -FREERTOS_EINVAL)
                    {
                        errno = EINVAL;
                        return -1;
                    }
                    return 0;
                default:
                    errno = EINVAL;
                    return -1;
            }
            break;
        case IPPROTO_TCP:
            switch (option_name)
            {
                case TCP_NODELAY:
                    // not implemented, return no error
                    return 0;
                default:
                    errno = EINVAL;
                    return -1;
            }
    }

    result = FreeRTOS_setsockopt(
        s->sd, level, option_name, option_value, option_len);

    if (result < 0)
    {
        switch (result)
        {
            case -FREERTOS_EINVAL:
                errno = EINVAL;
                break;
            default:
                HASSERT(0);
                break;
        }
        return -1;
    }

    return result;
}

/*
 * FreeRTOSTCPSocket::getsockopt()
 */
int FreeRTOSTCPSocket::getsockopt(int socket, int level, int option_name,
    void *option_value, socklen_t *option_len)
{
    File *f = file_lookup(socket);
    if (!f)
    {
        errno = EBADF;
        return -1;
    }

    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(f->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    return 0;

    // FreeRTOSTCPSocket does not support getsockopt
    errno = EINVAL;
    return -1;
}

/*
 * FreeRTOSTCPSocket::close()
 */
int FreeRTOSTCPSocket::close(File *file)
{
    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(file->priv);

    mutex.lock();
    if (--references_ == 0)
    {
        mutex.unlock();
        /* request that the socket be closed */
        FreeRTOSTCP::instance()->select_wakeup(s->sd);
    }
    else
    {
        mutex.unlock();
    }

    return 0;
}

/** Device select method. Default impementation returns true.
 * @param file reference to the file
 * @param mode FREAD for read active, FWRITE for write active, 0 for
 *        exceptions
 * @return true if active, false if inactive
 */
bool FreeRTOSTCPSocket::select(File *file, int mode)
{
    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(file->priv);
    bool retval = false;

    switch (mode)
    {
        case FREAD:
            portENTER_CRITICAL();
            if (readActive)
            {
                retval = true;
                portEXIT_CRITICAL();
            }
            else
            {
                select_insert(&selInfoRd);
                portEXIT_CRITICAL();
                FreeRTOSTCP::instance()->fd_set_read(s->sd);
            }
            break;
        case FWRITE:
            portENTER_CRITICAL();
            HASSERT(listenActive == false);
            if (writeActive)
            {
                retval = true;
                portEXIT_CRITICAL();
            }
            else
            {
                select_insert(&selInfoWr);
                portEXIT_CRITICAL();
                FreeRTOSTCP::instance()->fd_set_write(s->sd);
            }
            break;
        default:
        case 0:
            /* we don't support any exceptions */
            break;
    }
    return retval;
}

/*
 * FreeRTOSTCPSocket::get_sd_by_index()
 */

FreeRTOSTCPSocket *FreeRTOSTCPSocket::get_sd_by_index(int inx)
{
    if ((inx < 0) || (inx > MAX_SOCKETS))
    {
        HASSERT(0);
    }
    return FreeRTOSTCPSockets[inx];
}

/*
 * FreeRTOSTCPSocket::alloc_instance()
 */
int FreeRTOSTCPSocket::alloc_instance(Socket_t sd)
{
    FreeRTOSTCPSocket *new_socket = new FreeRTOSTCPSocket();

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
    file->priv = new_socket;

    new_socket->sd = sd;
    new_socket->references_ = 1;

    portENTER_CRITICAL();
    for (int i = 1; i < MAX_SOCKETS; ++i)
    {
        if (FreeRTOSTCPSockets[i] == nullptr)
        {
            FreeRTOSTCPSockets[i] = new_socket;
            break;
        }
    }
    portEXIT_CRITICAL();

    return fd;
}
/*
 * FreeRTOSTCPSocket::get_instance_from_sd()
 */
FreeRTOSTCPSocket *FreeRTOSTCPSocket::get_instance_from_sd(Socket_t sd)
{
    for (int i = 0; i < MAX_SOCKETS; ++i)
    {
        if (sd == FreeRTOSTCPSockets[i]->sd)
        {
            return FreeRTOSTCPSockets[i];
        }
    }
    HASSERT(0);
    return nullptr;
}

/*
 * FreeRTOSTCPSocket::remove_instance_from_sd()
 */
void FreeRTOSTCPSocket::remove_instance_from_sd(Socket_t sd)
{
    for (int i = 0; i < MAX_SOCKETS; ++i)
    {
        if (sd == FreeRTOSTCPSockets[i]->sd)
        {
            FreeRTOSTCPSockets[i] = NULL;
            return;
        }
    }
    HASSERT(0);
}

/*
 * FreeRTOSTCPSocket::fcntl()
 */
int FreeRTOSTCPSocket::fcntl(File *file, int cmd, unsigned long data)
{
    FreeRTOSTCPSocket *s = static_cast<FreeRTOSTCPSocket *>(file->priv);
    if (!S_ISSOCK(s->mode_))
    {
        errno = ENOTSOCK;
        return -1;
    }

    switch (cmd)
    {
        case F_SETFL:
            if (data & O_NONBLOCK)
            {
                // translate set non blocking into zero Recv and Send timeouts
                static const TickType_t timeout = 0;
                FreeRTOS_setsockopt(sd, 0, FREERTOS_SO_RCVTIMEO,
                    (void *)&timeout, sizeof(timeout));
                FreeRTOS_setsockopt(sd, 0, FREERTOS_SO_SNDTIMEO,
                    (void *)&timeout, sizeof(timeout));
            }
            return 0;
        case F_GETFL:
            return 0;
        default:
        {
            return -EINVAL;
        }
    }
}

extern "C" {
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
    return FreeRTOSTCPSocket::socket(domain, type, protocol);
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
    return FreeRTOSTCPSocket::bind(socket, address, address_len);
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
    return FreeRTOSTCPSocket::listen(socket, backlog);
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
    return FreeRTOSTCPSocket::accept(socket, address, address_len);
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
    return FreeRTOSTCPSocket::connect(socket, address, address_len);
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
    return FreeRTOSTCPSocket::recv(socket, buffer, length, flags);
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
    return FreeRTOSTCPSocket::send(socket, buffer, length, flags);
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
int setsockopt(int socket, int level, int option_name, const void *option_value,
    socklen_t option_len)
{
    return FreeRTOSTCPSocket::setsockopt(
        socket, level, option_name, option_value, option_len);
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
int getsockopt(int socket, int level, int option_name, void *option_value,
    socklen_t *option_len)
{
    return FreeRTOSTCPSocket::getsockopt(
        socket, level, option_name, option_value, option_len);
}

/** Get the socket name.
 * @param socket the socket file descriptor
 * @param addr points to a sockaddr structure to receive ane name
 * @param namelen points to a socklen_t to receive the lenght of the name
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int getsockname(int socket, struct sockaddr &addr, socklen_t &namelen)
{
    return 0;
}

static char str[32];

/** Converts internet address into a dotted decimal string
 * @param addr an in_addr structure containing the IP address
 * @return returns a string in dotted decimal representation
 */
const char *inet_ntoa(struct in_addr addr)
{
    // static char str[32];
    FreeRTOS_inet_ntoa(addr.s_addr, str);
    return str;
}

/** Converts the dotted decmail internet address string into a binary
 * representation
 * @param name string containing dotted decimal representation
 * @return binary address representation
 */
uint32_t inet_addr(const char *name)
{
    return FreeRTOS_inet_addr(name);
}
} /* extern "C" */
