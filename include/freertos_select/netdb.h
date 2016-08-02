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
 * \file netdb.h
 * This file implements POSIX netdb.h prototypes.
 *
 * @author Stuart W. Baker
 * @date 2 July 2016
 */

#ifndef _NETDB_H_
#define _NETDB_H_

#include <sys/types.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Structure to contain information about address of a service provider.
 */
struct addrinfo
{
    int ai_flags;             /**< Input flags */
    int ai_family;            /**< Protocol family for socket */
    int ai_socktype;          /**< Socket type */
    int ai_protocol;          /**< Protocol for socket */
    socklen_t ai_addrlen;     /**< Length of socket address */
    struct sockaddr *ai_addr; /**< Socket address for socket */
    char *ai_canonname;       /**< Canonical name for service location */
    struct addrinfo *ai_next; /**< Pointer to next in list */
};

const char *gai_strerror (int __ecode);

void freeaddrinfo(struct addrinfo *ai);

int getaddrinfo(const char *nodename, const char *servname,
                const struct addrinfo *hints,
                struct addrinfo **res);

# define EAI_AGAIN    -3    /**< Temporary failure in name resolution */
# define EAI_FAIL     -4    /**<, Non-recoverable failure in name res */
# define EAI_MEMORY   -10   /**< Memory allocation failure */


#ifdef __cplusplus
}
#endif

#endif /* _NETDB_H_ */
