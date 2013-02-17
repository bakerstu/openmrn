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
#include <endian.h>
#include <sys/tree.h>
#include "core/nmranet_event.h"
#include "core/nmranet_node_private.h"
#include "core/nmranet_buf.h"
#include "os/os.h"

//void nmranet_node_consumer_add(node_t node, uint64_t event, int state);
//void nmranet_node_producer_add(node_t node, uint64_t event, int state);
void nmranet_node_event_producer_state(node_t node, uint64_t event, int state);

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
    EventPriv *priv; /**< node associated with event */
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

/** Send a consumer identify message from a given node.
 * @param node node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
static void event_consumer_identify(node_t node, uint64_t event, int state)
{
    node_handle_t dst = {0, 0};
    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    *buffer = htobe64(event);
    nmranet_buffer_advance(buffer, sizeof(uint64_t));
    switch (state)
    {
        default:
            /* fall through */
        case EVENT_STATE_UNKNOWN:
            nmranet_node_write(node, MTI_CONSUMER_IDENTIFY_UNKNOWN, dst, buffer);
            break;
        case EVENT_STATE_VALID:
            nmranet_node_write(node, MTI_CONSUMER_IDENTIFY_VALID, dst, buffer);
            break;
        case EVENT_STATE_INVALID:
            nmranet_node_write(node, MTI_CONSUMER_IDENTIFY_INVALID, dst, buffer);
            break;
        case EVENT_STATE_RESERVED:
            nmranet_node_write(node, MTI_CONSUMER_IDENTIFY_RESERVED, dst, buffer);
            break;
    }
}

/** Send a producer identify message from a given node.
 * @param node node event is registered to
 * @param event event number to register
 * @param state initial state of the event
 */
static void event_producer_identify(node_t node, uint64_t event, int state)
{
    node_handle_t dst = {0, 0};
    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    *buffer = htobe64(event);
    nmranet_buffer_advance(buffer, sizeof(uint64_t));
    switch (state)
    {
        default:
            /* fall through */
        case EVENT_STATE_UNKNOWN:
            nmranet_node_write(node, MTI_PRODUCER_IDENTIFY_UNKNOWN, dst, buffer);
            break;
        case EVENT_STATE_VALID:
            nmranet_node_write(node, MTI_PRODUCER_IDENTIFY_VALID, dst, buffer);
            break;
        case EVENT_STATE_INVALID:
            nmranet_node_write(node, MTI_PRODUCER_IDENTIFY_INVALID, dst, buffer);
            break;
        case EVENT_STATE_RESERVED:
            nmranet_node_write(node, MTI_PRODUCER_IDENTIFY_RESERVED, dst, buffer);
            break;
    }
}

/** Add the event to the conumser list within a given node.
 * @param node node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
static void event_consumer_add(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;
    
    /* just to be safe, mask off upper bits */
    state &= 0x3;
    
    os_mutex_lock(&nodeMutex);
    if (n->priv->consumerEvents == NULL)
    {
        n->priv->consumerEvents = malloc(sizeof(EventList));
        n->priv->consumerEvents->next = NULL;
        n->priv->consumerEvents->count = 0;
    }
    EventList *link = n->priv->consumerEvents;
    while (link->next != NULL)
    {
        link = link->next;
    }
    
    if (link->count >= 8)
    {
        link->next = malloc(sizeof(EventList));
        link = link->next;
        link->next = NULL;
        link->count = 0;
    }
    link->events[link->count] = event;
    link->state &= ~(0x3 << (link->count << 1));
    link->state |= (state << (link->count << 1));
    link->count++;
    
    if (n->priv->state == NODE_INITIALIZED)
    {
        event_consumer_identify(node, event, state);
    }
    os_mutex_unlock(&nodeMutex);
}

/** Add the event to the producer list within a given node.
 * @param node node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
static void event_producer_add(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;
    
    /* just to be safe, mask off upper bits */
    state &= 0x3;
    
    os_mutex_lock(&nodeMutex);
    if (n->priv->producerEvents == NULL)
    {
        n->priv->producerEvents = malloc(sizeof(EventList));
        n->priv->producerEvents->next = NULL;
        n->priv->producerEvents->count = 0;
    }
    EventList *link = n->priv->producerEvents;
    while (link->next != NULL)
    {
        link = link->next;
    }
    
    if (link->count >= 8)
    {
        link->next = malloc(sizeof(EventList));
        link = link->next;
        link->next = NULL;
        link->count = 0;
    }
    link->events[link->count] = event;
    link->state &= ~(0x3 << (link->count << 1));
    link->state |= (state << (link->count << 1));
    link->count++;
    
    if (n->priv->state == NODE_INITIALIZED)
    {
        event_producer_identify(node, event, state);
    }
    os_mutex_unlock(&nodeMutex);
}

/** Register for the consumption of an event with a given node.
 * @param node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
void nmranet_event_consumer(node_t node, uint64_t event, int state)
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
    event_consumer_add(node, event, state);
    os_mutex_unlock(&mutex);
}

/** Register for the production of an event with from given node.
 * @param node to register event from
 * @param event event number to register
 * @param state initial state of the event
 */
void nmranet_event_producer(node_t node, uint64_t event, int state)
{    
    event_producer_add(node, event, state);
}

/** Produce an event from.
 * @param node node to produce event from
 * @param event event to produce
 * @param state state of the event
 */
void nmranet_event_produce(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&nodeMutex);
    /* change the state of this event */
    for (EventList *l = n->priv->producerEvents; l != NULL; l = l->next)
    {
        for (unsigned int count = 0; count < l->count; count++)
        {
            if (l->events[count] == event)
            {
                l->state &= ~(0x3 << (l->count << 1));
                l->state |= (state << (l->count << 1));
                if (state == EVENT_STATE_VALID && n->priv->state == NODE_INITIALIZED)
                {
                    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
                    *buffer = htobe64(event);
                    nmranet_buffer_advance(buffer, sizeof(uint64_t));

                    node_handle_t dst = {0, 0};
                    nmranet_node_write(node, MTI_EVENT_REPORT, dst, buffer);
                }
                os_mutex_unlock(&nodeMutex);
                return;
            }
        }
    }
    os_mutex_unlock(&nodeMutex);
}

/** Post the reception of an event to given node.
 * @param node to post event to
 * @param event event number to post
 */
static void event_post(node_t node, uint64_t event)
{
    struct id_node *n = (struct id_node*)node;
    
    os_mutex_lock(&nodeMutex);

    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    if (buffer == NULL)
    {
        /** @todo increment statistics counter here */
        return;
    }
    *buffer = event;
    nmranet_buffer_advance(buffer, sizeof(uint64_t));
    nmranet_queue_insert(n->priv->eventQueue, buffer);

    if (n->priv->wait != NULL)
    {
        /* wakeup whoever is waiting */
        os_sem_post(n->priv->wait);
    }
    os_mutex_unlock(&nodeMutex);
}

/** Identify all consumer events.
 * @param node node instance to act on
 * @param consumer event(s) to identify
 * @param mask to determine if we interested
 */
void nmranet_identify_consumers(node_t node, uint64_t event, uint64_t mask)
{
    struct id_node *n = (struct id_node*)node;

    /* identify all of the events this node consumes */
    for (EventList *l = n->priv->consumerEvents; l != NULL; l = l->next)
    {
        for (unsigned int count = 0; count < l->count; count++)
        {
            /* apply mask */
            if ((l->events[count] & mask) == (event & mask))
            {
                event_consumer_identify(node, l->events[count], (l->state >> l->count) & 0x3);
            }
        }
    }
}

/** Identify all producer events.
 * @param node node instance to act on
 * @param producer event(s) to identify
 * @param mask to determine if we interested
 */
void nmranet_identify_producers(node_t node, uint64_t event, uint64_t mask)
{
    struct id_node *n = (struct id_node*)node;

    /* identify all of the events this node produces */
    for (EventList *l = n->priv->producerEvents; l != NULL; l = l->next)
    {
        for (unsigned int count = 0; count < l->count; count++)
        {
            /* apply mask */
            if ((l->events[count] & mask) == (event & mask))
            {
                event_producer_identify(node, l->events[count], (l->state >> l->count) & 0x3);
            }
        }
    }
}

/** Get the mask used to identify a range of producers/consumers.
 * @param event event id w/mask
 * @return resulting decoded mask
 */
static uint64_t identify_range_mask(uint64_t event)
{
    uint64_t mask = 0x0000000000000001;

    if (event & 0x0000000000000001)
    {
        for (uint64_t i = 0x0000000000000002; event & i; i <<= 1)
        {
            mask |= i;
        }
    }
    else
    {
        for (uint64_t i = 0x0000000000000002; (event & i) == 0; i <<= 1)
        {
            mask |= i;
        }
    }
    return ~mask;
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param node node that the packet is addressed to
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_addressed(uint16_t mti, node_t node, const void *data)
{
    struct id_node *id_node = node;
    if (id_node->priv->state == NODE_UNINITIALIZED)
    {
        return;
    }

    uint64_t event;
    memcpy(&event, data, sizeof(uint64_t));
    event = be64toh(event);

    switch (mti)
    {
        default:
            break;
        case MTI_CONSUMER_IDENTIFY:
            nmranet_identify_consumers(node, event, EVENT_EXACT_MASK);
            break;
        case MTI_CONSUMER_IDENTIFY_RANGE:
            nmranet_identify_consumers(node, event, identify_range_mask(event));
            break;
        case MTI_CONSUMER_IDENTIFY_UNKNOWN:  /* fall through */
        case MTI_CONSUMER_IDENTIFY_VALID:    /* fall through */
        case MTI_CONSUMER_IDENTIFY_INVALID:  /* fall through */
        case MTI_CONSUMER_IDENTIFY_RESERVED:
            break;
        case MTI_PRODUCER_IDENTIFY:
            nmranet_identify_consumers(node, event, EVENT_EXACT_MASK);
            break;
        case MTI_PRODUCER_IDENTIFY_RANGE:
            nmranet_identify_producers(node, event, identify_range_mask(event));
            break;
        case MTI_PRODUCER_IDENTIFY_UNKNOWN:  /* fall through */
        case MTI_PRODUCER_IDENTIFY_VALID:    /* fall through */
        case MTI_PRODUCER_IDENTIFY_INVALID:  /* fall through */
        case MTI_PRODUCER_IDENTIFY_RESERVED:
            break;
        case MTI_EVENTS_IDENTIFY_GLOBAL:     /* fall through */
        case MTI_EVENTS_IDENTIFY_ADDRESSED:
            nmranet_identify_consumers(node, 0, EVENT_ALL_MASK);
            nmranet_identify_producers(node, 0, EVENT_ALL_MASK);
            break;
    }
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param data NMRAnet packet data
 */
void nmranet_event_packet_global(uint16_t mti, const void *data)
{
    switch (mti)
    {
        default:
            break;
        case MTI_EVENT_REPORT:
        {
            /* to save processing time in instantiations that include a large
             * number of nodes, consumers are sorted at the event level and
             * not at the node level.
             */
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
                    event_post(current->node, event);
                }
            }
            os_mutex_unlock(&mutex);
            break;
        }
        case MTI_CONSUMER_IDENTIFY:
            /* fall through */
        case MTI_CONSUMER_IDENTIFY_RANGE:
            /* fall through */
        case MTI_CONSUMER_IDENTIFY_UNKNOWN:
            /* fall through */
        case MTI_CONSUMER_IDENTIFY_VALID:
            /* fall through */
        case MTI_CONSUMER_IDENTIFY_INVALID:
            /* fall through */
        case MTI_CONSUMER_IDENTIFY_RESERVED:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY_RANGE:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY_UNKNOWN:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY_VALID:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY_INVALID:
            /* fall through */
        case MTI_PRODUCER_IDENTIFY_RESERVED:
            /* fall through */
        case MTI_EVENTS_IDENTIFY_GLOBAL:
            os_mutex_lock(&nodeMutex);
            /* global message, deliver all, non-subscribe */
            for (node_t node = nmranet_node_next(NULL);
                 node != NULL;
                 node = nmranet_node_next(node))
            {
                nmranet_event_packet_addressed(mti, node, data);
            }
            os_mutex_unlock(&nodeMutex);
            break;
    }
}

/** Process an event packet.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_event_packet(uint16_t mti, node_handle_t src, node_id_t dst, const void *data)
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
            event_post(current->node, event);
        }
        os_mutex_unlock(&mutex);
        return 0;
    }
    os_mutex_unlock(&mutex);
    
    return 1;
}

/** Grab an event from the event queue of the node.
 * @param node to grab event from
 * @return 0 if the queue is empty, else return the event number
 */
uint64_t nmranet_event_consume(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&nodeMutex);
    uint64_t *event = nmranet_queue_next(n->priv->eventQueue);
    if (event != NULL)
    {
        os_mutex_unlock(&nodeMutex);
        uint64_t result = *event;
        nmranet_buffer_free(event);
        return result;
    }
    os_mutex_unlock(&nodeMutex);
    
    return 0;
}

/** Number of events pending in the event queue of the node
 * @param node node to query
 * @return number of events pending.
 */
size_t nmranet_event_pending(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&nodeMutex);
    size_t pending = nmranet_queue_pending(n->priv->eventQueue);
    os_mutex_unlock(&nodeMutex);

    return pending; 
}

