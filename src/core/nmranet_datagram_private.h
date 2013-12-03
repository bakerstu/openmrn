/** \copyright
 * Copyright (c) 2013, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file nmranet_datagram_private.h
 * This file defines the handling of NMRAnet datagrams.
 *
 * @author Stuart W. Baker
 * @date 16 February 2013
 */

#ifndef _nmranet_datagram_private_h_
#define _nmranet_datagram_private_h_

#include "core/nmranet_datagram.h"

#ifdef __cplusplus
extern "C" {
#endif

/** We can try to resend the datagram.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_RESEND_OK(_err) (_err & 0x2000)

/** Transport error occurred.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_TRANSPORT_ERROR(_err) (_err & 0x6000)

/** Buffer unavailable error occurred.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_BUFFER_UNAVAILABLE(_err) (_err & 0x2020)

/** Out of order error occurred.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_OUT_OF_ORDER(_err) (_err & 0x2040)

/** Permanent error occurred.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_PERMANENT_ERROR(_err) (_err & 0x1000)

/** Source not permitted error occurred.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_SRC_NOT_PERMITTED(_err) (_err & 0x1020)

/** Destination node does not accept datagrams of any kind.
 * @param _err error number
 * @return true or false
 */
#define IS_DATAGRAM_NOT_ACCEPTED(_err) (_err & 0x1040)

/** We can try to resend the datagram.
 */
#define DATAGRAM_RESEND_OK 0x2000

/** Transport error occurred.
 */
#define DATAGRAM_TRANSPORT_ERROR 0x6000

/** Buffer unavailable error occurred.
 */
#define DATAGRAM_BUFFER_UNAVAILABLE 0x2020

/** Out of order error occurred.
 */
#define DATAGRAM_OUT_OF_ORDER 0x2040

/** Permanent error occurred.
 */
#define DATAGRAM_PERMANENT_ERROR 0x1000

/** Source not permitted error occurred.
 */
#define DATAGRAM_SRC_NOT_PERMITTED 0x1020

/** Destination node does not accept datagrams of any kind.
 */
#define DATAGRAM_NOT_ACCEPTED 0x1040

/** Datagram information. */
typedef struct datagram
{
    node_t to; /**< the node that the datagram is to, only used in automatic
                  processing */
    node_handle_t from; /**< node id or alias this datagram is from */
    size_t size;        /**< size of datagram in bytes */
    uint8_t data[DATAGRAM_MAX_SIZE]; /**< datagram payload */
} Datagram;

/** Allocate a datagram from the pool.
 * @return datagram allocated, or NULL if there are no more datagrams available.
 */
Datagram* nmranet_datagram_alloc(void);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_datagram_private_h_ */
