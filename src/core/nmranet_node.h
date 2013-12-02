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
#define NODE_ID_CBUS_EVENTS_BASE 0x010101000000LL
/** CBUS events mask */
#define NODE_ID_CBUS_EVENTS_MASK 0xffffff000000LL
/** XpressNet translation base */
#define NODE_ID_XPRESSNET_BASE 0x016300000000LL
/** XpressNet translation mask */
#define NODE_ID_XPRESSNET_MASK 0xffff00000000LL
/** NocoNet translation base */
#define NODE_ID_LOCONET_BASE 0x018100000000LL
/** LocoNet translation mask */
#define NODE_ID_LOCONET_MASK 0xffff00000000LL
/** DCC translation base */
#define NODE_ID_DCC_BASE 0x01ee00000000LL
/** DCC translation mask */
#define NODE_ID_DCC_MASK 0xffff00000000LL
/** Mask to get all messages */
#define NODE_ID_ALL_MASK 0x000000000000LL
/** Mask for exact node ID */
#define NODE_ID_EXACT_MASK 0xffffffffffffLL

#define PROTOCOL_PROTOCOL_IDENTIFICATION 0x800000000000
#define PROTOCOL_DATAGRAM 0x400000000000
#define PROTOCOL_STREAM 0x200000000000
#define PROTOCOL_MEMORY_CONFIGURATION 0x100000000000
#define PROTOCOL_RESERVATION 0x080000000000
#define PROTOCOL_EVENT_EXCHANGE 0x040000000000
#define PROTOCOL_IDENTIFICATION 0x020000000000
#define PROTOCOL_LEARN_CONFIGURATION 0x010000000000
#define PROTOCOL_REMOTE_BUTTON 0x008000000000
#define PROTOCOL_ABBREVIATED_DEFAULT_CDI 0x004000000000
#define PROTOCOL_DISPLAY 0x002000000000
#define PROTOCOL_SIMPLE_NODE_INFORMATION 0x001000000000
#define PROTOCOL_CDI 0x000800000000
#define PROTOCOL_RESERVED_MASK 0x0007FFFFFFFF

/** Create a new virtual node.
 * @param node_id 48-bit unique Node ID
 * @param nmranet_if interface to bind the node to
 * @param model node decription
 * @param priv private data to store with the the node for later retrieval
 * @return upon success, handle to the newly created node, else NULL
 */
node_t nmranet_node_create(node_id_t node_id, NMRAnetIF *nmranet_if,
                           const char *model, void *priv);

/** Lookup the node handle for a given node ID.
 * @param node_id 48-bit unique Node ID
 * @return if it exists, handle to the node id, else NULL
 */
node_t nmranet_node(node_id_t node_id);

/** Move node into the initialized state.
 * @param node node instance to act on
 */
void nmranet_node_initialized(node_t node);

/** Obtain the Node ID of a node handle
 * @param node node to get a the Node ID from
 * @return 48-bit NMRAnet Node ID
 */
node_id_t nmranet_node_id(node_t node);

/** Get the 48-bit Node ID from a handle.
 * @param node node to get a the Node ID for
 * @param h handle to grab node_id from
 * @return 48-bit NMRAnet Node ID, 0 if not found.
 */
node_id_t nmranet_node_id_from_handle(node_t node, node_handle_t h);

/** Lookup the private data pointer for a given handle.
 * @param node node to get a the Node ID from
 * @return if it exists, handle to the node id, else NULL
 */
void *nmranet_node_private(node_t node);

/** Write a message from a node.  Though this is a public facing API, it is
 * highly recommended that this API not be used directly.  It is only present
 * for very special circumstances.
 * @param node node to write message from
 * @param mti Message Type Indicator
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_node_write(node_t node, uint16_t mti, node_handle_t dst,
                       const void *data);

/** Wait for data to come in from the network.
 * @param node node to wait on
 * @param timeout timeout in nanoseconds, 0 to return right way, OS_WAIT_FOREVER
          to wait forever.
   @return number of messages pending, else 0 on timeout
 */
int nmranet_node_wait(node_t node, long long timeout);

/** Set the user name of the node for simple ident protocol.
 * @param node node to set attribute on
 * @param user_name string to use for user name
 */
void nmranet_node_user_name(node_t node, const char *user_name);

/** Set the user description of the node for simple ident protocol.
 * @param node node to set attribute on
 * @param user_description string to use for user description
 */
void nmranet_node_user_description(node_t node, const char *user_description);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_node_h_ */
