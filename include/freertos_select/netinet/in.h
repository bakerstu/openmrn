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
 * This file implements POSIX netinet/in.h prototypes.
 *
 * @author Stuart W. Baker
 * @date 18 March 2016
 */

#ifndef _NETINET_IN_H_
#define _NETINET_IN_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef uint32_t in_addr_t;
struct in_addr
{
    in_addr_t s_addr;
};

/** Structure describing an Internet socket address.  */
struct sockaddr_in
{
    uint16_t sin_family;     /**< protocol family (AF_INET) */
    uint16_t sin_port;       /**< port number */
    struct in_addr sin_addr; /**< internet address */

    /* Pad to size of `struct sockaddr'.  */
    unsigned char sin_zero[sizeof (struct sockaddr) - sizeof(uint16_t) -
                           sizeof (uint16_t) - sizeof (struct in_addr)];
};

#define INADDR_ANY  (0)

/** TCP Raw Socket */
#define IPPROTO_TCP (6)

/** UDP Raw Socket */
#define IPPROTO_UDP (17)

/** Raw Socket */
#define IPPROTO_RAW (255)

#include <endian.h>

#ifdef CONFIG_ENDIAN_BIG
#define ntohl(x)   (x)
#define ntohs(x)   (x)
#define htonl(x)   (x)
#define htons(x)   (x)
#else
#define ntohl(x) __bswap_32 (x)
#define ntohs(x) __bswap_16 (x)
#define htonl(x) __bswap_32 (x)
#define htons(x) __bswap_16 (x)
#endif

#ifdef __cplusplus
}
#endif

#endif /* _NETINET_IN_H_ */
