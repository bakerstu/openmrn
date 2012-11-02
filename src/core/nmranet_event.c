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
 * \file nmranet_event.c
 * This file defines the handling of NMRAnet producer/consumer events.
 *
 * @author Stuart W. Baker
 * @date 29 October 2012
 */

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/tree.h>
#include "core/nmranet_event.h"
#include "core/nmranet_node.h"
#include "os/os.h"

/** Mutual exclusion for socket library */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** Event metadata. */
typedef struct event
{
    node_t node; /**< event id associated with this chain */
    struct event *next; /**< next entry in the chain */
} EventPriv;

/** Red Black tree node for sorting by event ID.
 */
struct event_node
{
    /** entry metadata */
    RB_ENTRY(event_node) entry;
    union
    {
        uint64_t event;
        int64_t key;
    };
    EventPriv *priv; /**< socket associated with event */
};
    
/** Mathematically compare two event ID's.
 * @param a first event to compare
 * @param b second event to compare
 * @return (a - b)
 */
static int64_t event_compare(struct event_node *a, struct event_node *b)
{
    return a->key - b->key;
}

/** The event tree type */
RB_HEAD(event_tree, event_node);
/** The event tree methods */
RB_GENERATE_STATIC(event_tree, event_node, entry, event_compare);
/** The event tree head. */
struct event_tree eventHead = RB_INITIALIZER(&eventHead);

/** Register for the consumption of an event with a given node.
 * @param node to register event to
 * @param event event number to register
 */
void nmranet_event_consumer(node_t node, uint64_t event)
{    
    struct event_node *event_node;
    struct event_node  event_lookup;
    
    event_lookup.event = event;
    
    os_mutex_lock(&mutex);
    event_node = RB_FIND(event_tree, &eventHead, &event_lookup);
    if (event_node)
    {
        EventPriv *priv = malloc(sizeof(EventPriv));
        priv->next = event_node->priv;
        event_node->priv = priv;
    }
    else
    {
        event_node = malloc(sizeof(struct event_node));
        event_node->event = event;
        event_node->priv = malloc(sizeof(EventPriv));
        event_node->priv->node = node;
        event_node->priv->next = NULL;
        RB_INSERT(event_tree, &eventHead, event_node);
    }
    os_mutex_unlock(&mutex);
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_event_packet(uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    struct event_node *event_node;
    struct event_node  event_lookup;
    
    uint64_t event;
    memcpy(&event, data, sizeof(uint64_t));
    
    event = be64toh(event);
    event_lookup.event = event;

    os_mutex_lock(&mutex);
    event_node = RB_FIND(event_tree, &eventHead, &event_lookup);
    if (event_node)
    {
        for (EventPriv *current = event_node->priv;
             current != NULL;
             current = current->next)
        {
            nmranet_node_post_event(current->node, event);
        }
    }
    os_mutex_unlock(&mutex);
    
    return 0;
}

