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
 * \file inet.h
 * This file implements POSIX arpa/inet.h prototypes.
 *
 * @author Stuart W. Baker
 * @date 19 March 2016
 */

#ifndef _ARPA_INET_H_
#define _ARPA_INET_H_

#include <netinet/in.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Converts a number-and-dot notation internet address to binary form.
///
/// @param name standard argument
/// @param addr standard argument
///
/// @return error code
///
int inet_aton (const char *name, struct in_addr *addr);

/// Converts a number-and-dot notation internet address to network byte order
/// form. @param name is the text representation. @return the 32-bit address in
/// ntwork byte order, or -1 if there is an error.
uint32_t inet_addr (const char *name);
    
/// Converts an address to textual representation. Not reentrant. @param addr
/// is the address. @return the textual form of the address (statically
/// allocated buffer).
char *inet_ntoa (struct in_addr addr);

/** Convert the network address in src to a character string in src.
 * @param af address family, AF_INET or AF_INET6
 * @param src source address in network byte order
 * @param dst resulting address string
 * @param size max number characters allowed for the result
 * @return on success non-null pointer to dst, else NULL with errno set on error
 */
const char *inet_ntop(int af, const void *src, char *dst, socklen_t size);

#ifdef __cplusplus
}
#endif

#endif /* _ARPA_INET_H_ */
