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
 * \file select.h
 * This file implements POSIX sys/socket.h prototypes.
 *
 * @author Stuart W. Baker
 * @date 18 March 2016
 */

#ifndef _SYS_SOCKET_H_
#define _SYS_SOCKET_H_

#include <sys/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** TCP Socket */
#define SOCK_STREAM (1)

/** UDP Socket */
#define SOCK_DGRAM  (2)

/** Raw Socket */
#define SOCK_RAW    (3)

/** IPv4 Socket (UDP, TCP, etc...) */
#define AF_INET     (2)

/** IPv6 Socket (UDP, TCP, etc...) */
#define AF_INET6    (3)

#define AF_PACKET   (17)
#define PF_INET     AF_INET
#define PF_INET6    AF_INET6

/** socket option category */
#define SOL_SOCKET (1)

/** socket option to reuse address */
#define SO_REUSEADDR (2)

/** IPv4 socket address */
struct sockaddr
{
    uint16_t sa_family; /**< address family (e.g. AF_INET) */
    uint8_t  sa_data[14]; /**< protocol specific address information */
};

/** type of sockaddr lenth */
typedef uint32_t socklen_t;

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
int socket(int domain, int type, int protocol);

/** Bind a name to a socket.
 * @param socket file descriptor of the socket to be bound
 * @param address points to a sockaddr structure containing the address to
 *                be bound to the socket
 * @param address_len specifies the length of the sockaddr structure pointed
 *                    to by the address argument
 * @return shall return on success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int bind(int socket, const struct sockaddr *address, socklen_t address_len);

/** Mark a connection-mode socket, specified by the socket argument, as
 * accepting connections.
 * @param socket the socket file descriptor
 * @param backlog provides a hint to the implementation which the
 *                implementation shall use to limit the number of
 *                outstanding connections in the socket's listen queue
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int listen(int socket, int backlog);

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
int accept(int socket, struct sockaddr *address, socklen_t *address_len);

/** Connect a socket.
 * @param socket the socket file descriptor
 * @param address points to a sockaddr structure containing the peer address
 * @param address_len specifies the length of the sockaddr structure pointed
 *                    to by the address argument
 * @return shall return 0 upon success, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
int connect(int socket, const struct sockaddr *address, socklen_t address_len);

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
ssize_t recv(int socket, void *buffer, size_t length, int flags);

/** Initiate transmission of a message from the specified socket.
 * @param socket the socket file descriptor
 * @param buffer buffer containing the message to send
 * @param length length of the message in bytes
 * @param flags the type of message transmission
 * @return the number of bytes sent, otherwise, -1 shall be returned and
 *         errno set to indicate the error
 */
ssize_t send(int socket, const void *buffer, size_t length, int flags);

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
               const void *option_value, socklen_t option_len);

#ifdef __cplusplus
}
#endif

#endif /* _SYS_SOCKET_H_ */
