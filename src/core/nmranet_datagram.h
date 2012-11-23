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

#ifdef __cplusplus
extern "C" {
#endif

#define DATAGRAM_LOG_REQUEST   0x01 /**< request a placement into the log */
#define DATAGRAM_LOG_REPLY     0x02 /**< reply to a @ref DATAGRAM_LOG_REQUEST */
#define DATAGRAM_CONFIGURATION 0x20 /**< configuration message */
#define DATAGRAM_REMOTE_BUTTON 0x21 /**< remote button input */
#define DATAGRAM_DISPLAY       0x28 /**< place on display */
#define DATAGRAM_TRAIN_CONTROL 0x30 /**< operation of mobile nodes */

/** Datagram information. */
typedef struct datagram
{
    node_id_t from; /**< node id this datagram is from */
    const void *data; /**< datagram payload */
} Datagram;

/** Process a datagram packet.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_datagram_packet(uint16_t mti, node_id_t src, node_id_t dst, const void *data);

/** Register for the consumption of a datagram type with a given node.
 * @param node to register datagram to
 * @param datagram datagram identifier to register
 */
void nmranet_datagram_consumer(node_t node, uint64_t datagram);

/** Grab a datagram from the datagram queue of the node.
 * @param node to grab datagram from
 * @return NULL if queue is empty, else pointer to the datagram
 */
Datagram *nmranet_datagram_consume(node_t node);

/** Consumed datagrams must be released so that their memory can be freed.
 * The application may hold onto the datagram for an appropriate amount of time
 * to process it before releasing it.
 * @param datagram datagram to release
 */
void nmranet_datagram_release(Datagram *datagram);

/** Process the datagram automatically.  @ref nmranet_datagram_release will be
 * called on the datagram prior to the return of this method.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process, this pointer is stale upon return
 */
void nmranet_datagram_process(node_t node, Datagram *datagram);

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param datagram datagram to produce (in the form of an nmranet buffer)
 */
void nmranet_datagram_produce(node_t node, const void *datagram);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_datagram_h_ */

