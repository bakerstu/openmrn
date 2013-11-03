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
 * \file nmranet_node_private.h
 * This file defines NMRAnet node private data.
 *
 * @author Stuart W. Baker
 * @date 9 February 2013
 */

#ifndef _nmranet_node_private_h_
#define _nmranet_node_priavate_h_

#include "sys/tree.h"

#include "core/nmranet_node.h"
#include "core/nmranet_buf.h"
#include "core/nmranet_datagram_private.h"
#include "os/os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SIMPLE_NODE_IDENT_VERSION_A 0x01
#define SIMPLE_NODE_IDENT_VERSION_B 0x01

/** Operational states of the node */
typedef enum node_state
{
    NODE_UNINITIALIZED = 0, /**< uninitialized node state */
    NODE_INITIALIZED /**< initialized node state */
} NodeState;

/** List for storing a nodes interesting events */
typedef struct event_list
{
    uint64_t events[8]; /**< up to 8 active events */
    struct event_list *next; /**< next link in list */
    uint16_t state; /**< consumer state: 0 valid, 1 invalid, 2 reserved, 3 unknown */
    uint16_t count; /**< count of events used */
} EventList;

/** Private data about the node */
typedef struct node_priv
{
    NMRAnetIF *nmranetIF; /**< interface this node is bound to */
    nmranet_queue_t eventQueue; /**< queue for delivering events */    
    nmranet_queue_t datagramQueue; /**< queue for delivering datagrams */
    EventList *consumerEvents; /**< events that this node consumes */
    EventList *producerEvents; /**< events that this node produces */
    os_sem_t *wait; /**< semaphore used to wakeup thread waiting on node */
    const char *model; /**< model number for the ident info */
    const char *userName; /**< user provided name */
    const char *userDescription; /**< user provided description */
    void *priv; /**< Private data from the application */
    NodeState state; /**< node's current operational state */
    Datagram *txDatagram; /**< node's datagram in transmission */
    os_timer_t datagramTimer; /**< timer for traking datagram transmissions */
} NodePriv;

/** Red Black tree node for sorting by node ID.
 */
struct id_node
{
    /** entry metadata */
    RB_ENTRY(id_node) entry;
    union
    {
        node_id_t id;
        int64_t key;
    };
    NodePriv *priv; /**< Private data for the node ID */
};

/** Mutual exclusion for nodes */
extern os_mutex_t nodeMutex;

/** Datagram timeout handler.
 * @param data1 struct node_private pointer for node
 * @param data2 NULL
 * @return timer restart value
 */
long long nmranet_datagram_timeout(void *data1, void* data2);

/** Post the reception of a datagram with to given node.
 * @param node to post event to
 * @param mti Message Type Indeicator
 * @param src source Node ID
 * @param data datagram to post
 */
void nmranet_datagram_packet(node_t node, uint16_t mti, node_handle_t src, const uint8_t *data);

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param src source Node ID
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_global(uint16_t mti, node_handle_t src, const void *data);

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param node node that the packet is addressed to
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_addressed(uint16_t mti, node_handle_t src, node_t node, const void *data);

/** Identify all consumer events.
 * @param node node instance to act on
 * @param consumer event(s) to identify
 * @param mask to determine if we interested
 */
void nmranet_identify_consumers(node_t node, uint64_t event, uint64_t mask);

/** Identify all producer events.
 * @param node node instance to act on
 * @param producer event(s) to identify
 * @param mask to determine if we interested
 */
void nmranet_identify_producers(node_t node, uint64_t event, uint64_t mask);

/** Get a handle to the next node in the list of nodes.  This should be called
 * only with the @ref nodeMutex locked.
 * @param node previous node in the list, NULL to start from beginning
 * @return handle to next node in the list, NULL if at end of list
 */
node_t nmranet_node_next(node_t node);

#ifdef __cplusplus
}
#endif

#endif /* _nmranet_node_private_h_ */
