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
 * \file nmranet_types.h
 * Interesting NMRAnet types.
 *
 * @author Stuart W. Baker
 * @date 19 September 2012
 */

#ifndef _nmranet_types_h_
#define _nmranet_types_h_

#include <stdint.h>

#define NUM_ALIAS_IF 2

typedef uint64_t node_id_t;    /**< 48-bit node id type */
typedef uint16_t node_alias_t; /**< 12-bit node alias */
typedef void* node_t;          /**< handle to an NMRAnet node */
typedef void* alias_cache_t;   /**< alias cache handle type */
typedef void* datagram_t;      /**< handle to a datagram */

/** Handle as a 48-bit node id, 12-bit node alias, or both.
 */
typedef struct
{
    node_id_t id;       /**< 48-bit node id */
    node_alias_t alias; /**< 12-bit node alias */
} node_handle_t;

#endif /* _nmranet_types_h_ */
