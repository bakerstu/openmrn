/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file nmranet_datagram.h
 * This file defines the handling of NMRAnet datagrams.
 *
 * @author Stuart W. Baker
 * @date 4 November 2012
 */

#ifndef _nmranet_datagram_h_
#define _nmranet_datagram_h_

#include <stdint.h>
#include "nmranet_types.h"
#include "nmranet_configuration.h"

#ifdef __cplusplus
extern "C" {
#endif

/** maximum size in bytes of a datagram */
#define DATAGRAM_MAX_SIZE 72

#define DATAGRAM_LOG_REQUEST 0x01   /**< request a placement into the log */
#define DATAGRAM_LOG_REPLY 0x02     /**< reply to a @ref DATAGRAM_LOG_REQUEST */
#define DATAGRAM_CONFIGURATION 0x20 /**< configuration message */
#define DATAGRAM_REMOTE_BUTTON 0x21 /**< remote button input */
#define DATAGRAM_DISPLAY 0x28       /**< place on display */
#define DATAGRAM_TRAIN_CONTROL 0x30 /**< operation of mobile nodes */

#define DATAGRAM_PROTOCOL_SIZE_2 \
  0xE0 /**< possible return value for @ref DATAGRAM_PROTOCOL_SIZE */
#define DATAGRAM_PROTOCOL_SIZE_6 \
  0xF0 /**< possible return value for @ref DATAGRAM_PROTOCOL_SIZE */
#define DATAGRAM_PROTOCOL_SIZE_MASK \
  0xF0 /**< mask used when determining protocol size */

/** Determine if the protocol ID is represented by one, two, or six bytes.
 * @param _protocol protocol ID to interrogate
 * @return number of bytes representing the protocol
 */
#define DATAGRAM_PROTOCOL_SIZE(_protocol)                                    \
  ((((_protocol) & DATAGRAM_PROTOCOL_SIZE_MASK) == DATAGRAM_PROTOCOL_SIZE_6) \
       ? 6                                                                   \
       : ((((_protocol) & DATAGRAM_PROTOCOL_SIZE_MASK) ==                    \
           DATAGRAM_PROTOCOL_SIZE_2)                                         \
              ? 2                                                            \
              : 1))

/** Grab a datagram from the datagram queue of the node.
 * @param node to grab datagram from
 * @return NULL if queue is empty, else pointer to the datagram
 */
datagram_t nmranet_datagram_consume(node_t node);

/** Consumed datagrams must be released so that their memory can be freed.
 * The application may hold onto the datagram for an appropriate amount of time
 * to process it before releasing it.
 * @param datagram datagram to release
 */
void nmranet_datagram_release(datagram_t datagram);

/** Determine the datagram protocol (could be 1, 2, or 6 bytes).
 * @param datagram pointer to the beginning of the datagram
 * @return protocol type
 */
uint64_t nmranet_datagram_protocol(datagram_t datagram);

/** Determine the datagram payload
 * @param datagram pointer to the beginning of the datagram
 * @return pointer to payload an a uint8_t array, assume byte alignment
 */
uint8_t *nmranet_datagram_payload(datagram_t datagram);

/** Allocate and prepare a datagram buffer.
 * @param protocol datagram protocol to use
 * @param size max length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait,
 * OS_WAIT_FOREVER = blocking
 * @return datagram handle upon success, else NULL on error with errno set
 */
datagram_t nmranet_datagram_buffer_get(uint64_t protocol, size_t size,
                                       long long timeout);

/** Allocate and prepare a datagram buffer.
 * @param protocol datagram protocol to use
 * @param data datagram to fill into datagram
 * @param offset offset within datagram payload to start filling
 * @param size length of data in bytes to fill
 */
void nmranet_datagram_buffer_fill(datagram_t datagram, uint64_t protocol,
                                  const void *data, size_t offset, size_t size);

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param dst destination node id or alias
 * @param datagram datagram to produce
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait,
 * OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int nmranet_datagram_buffer_produce(node_t node, node_handle_t dst,
                                    datagram_t datagram, long long timeout);

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param dst destination node id or alias
 * @param protocol datagram protocol to use
 * @param data datagram to produce
 * @param size length of data in bytes
 * @param timeout time in nanoseconds to keep trying, 0 = do not wait,
 * OS_WAIT_FOREVER = blocking
 * @return 0 upon success, else -1 on error with errno set
 */
int nmranet_datagram_produce(node_t node, node_handle_t dst, uint64_t protocol,
                             const void *data, size_t size, long long timeout);

/** Number of datagrams pending in the datagram queue of the node.
 * @param node node to query
 * @return number of datagrams pending
 */
size_t nmranet_datagram_pending(node_t node);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_datagram_h_ */
