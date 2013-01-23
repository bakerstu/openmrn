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
 * \file nmranet_node.h
 * This file defines NMRAnet nodes.
 *
 * @author Stuart W. Baker
 * @date 19 September 2012
 */

#ifndef _nmranet_node_h_
#define _nmranet_node_h_

#include "nmranet_types.h"
#include "if/nmranet_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Well known events Node ID */
#define NODE_ID_WELL_KNOWN_EVENTS 0x010100000000LL
/** CBUS events base */
#define NODE_ID_CBUS_EVENTS_BASE  0x010101000000LL
/** CBUS events mask */
#define NODE_ID_CBUS_EVENTS_MASK  0xffffff000000LL
/** XpressNet translation base */
#define NODE_ID_XPRESSNET_BASE    0x016300000000LL
/** XpressNet translation mask */
#define NODE_ID_XPRESSNET_MASK    0xffff00000000LL
/** NocoNet translation base */
#define NODE_ID_LOCONET_BASE      0x018100000000LL
/** LocoNet translation mask */
#define NODE_ID_LOCONET_MASK      0xffff00000000LL
/** DCC translation base */
#define NODE_ID_DCC_BASE          0x01ee00000000LL
/** DCC translation mask */
#define NODE_ID_DCC_MASK          0xffff00000000LL
/** Mask to get all messages */
#define NODE_ID_ALL_MASK          0x000000000000LL
/** Mask for exact node ID */
#define NODE_ID_EXACT_MASK        0xffffffffffffLL


/** Create a new virtual node.
 * @param node_id 48-bit unique Node ID
 * @param node_id_mask mask used for routing messages to this node
 * @param nmranet_if interface to bind the node to
 * @param model node decription
 * @param priv private data to store with the the node for later retrieveal
 * @return upon success, handle to the newly created node, else NULL
 */
node_t nmranet_node_create(node_id_t node_id, node_id_t node_id_mask, NMRAnetIF *nmranet_if, const char* model, void *priv);

/** Lookup the node handle for a given node ID.
 * @param node_id 48-bit unique Node ID
 * @return if it exists, handle to the node id, else NULL
 */
node_t nmranet_node(node_id_t node_id);

/** Move node into the intitialized state.
 * @param node node instance to act on
 */
void nmranet_node_initialized(node_t node);

/** Wait for data to come in from the network.
 * @param node node to wait on
 */
void nmranet_node_wait(node_t node);

/** Post the reception of an event with to given node.
 * @param node to post event to
 * @param event event number to post
 */
void nmranet_node_post_event(node_t node, uint64_t event);

/** Post the reception of a datagram with to given node.
 * @param node to post event to
 * @param datagram datagram to post
 */
void nmranet_node_post_datagram(node_t node, const void *datagram);

/** Obtain the Node ID of a node handle
 * @param node node to get a the Node ID from
 * @return 48-bit NMRAnet Node ID
 */
node_id_t nmranet_node_id(node_t node);

/** Lookup the private data pointer for a given handle.
 * @param node node to get a the Node ID from
 * @return if it exists, handle to the node id, else NULL
 */
void *nmranet_node_private(node_t node);

/** Write a message from a node.
 * @param node node to write message from
 * @param mti Message Type Indicator
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_node_write(node_t node, uint16_t mti, node_handle_t dst, const void *data);


#ifdef __cplusplus
}
#endif

#endif /* _nmranet_node_h_ */

