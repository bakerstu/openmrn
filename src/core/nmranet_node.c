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
 * \file nmranet_node.c
 * This file defines NMRAnet nodes.
 *
 * @author Stuart W. Baker
 * @date 19 September 2012
 */

#include <stdlib.h>
#include <endian.h>
#include <sys/tree.h>
#include "core/nmranet_node.h"
#include "core/nmranet_buf.h"
#include "core/nmranet_datagram.h"
#include "core/nmranet_event.h"
#include "if/nmranet_if.h"
#include "os/os.h"

static void verify_node_id_number(node_t node);

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
    void *priv; /**< Private data from the application */
    NodeState state; /**< node's current operational state */
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
    node_id_t mask; /**< node id mask */
    NodePriv *priv; /**< Private data for the node ID */
};
    
/** Mathematically compare two Node ID's.
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int64_t id_compare(struct id_node *a, struct id_node *b)
{
    if (a->mask != 0 && b->mask != 0)
    {
        return (a->key & a->mask) - (b->key & b->mask);
    }
    else if (a->mask != 0)
    {
        return (a->key & a->mask) - (b->key & a->mask);
    }
    else if (b->mask != 0)
    {
        return (a->key & b->mask) - (b->key & b->mask);
    }
    return a->key - b->key;
}

/** The id tree type. */
RB_HEAD(id_tree, id_node);
/** The id tree methods. */
RB_GENERATE_STATIC(id_tree, id_node, entry, id_compare);

/** id tree head. */
static struct id_tree idHead = RB_INITIALIZER(&idHead);

/** Mutual exclusion for nodes */
static os_mutex_t mutex = OS_RECURSIVE_MUTEX_INITIALIZER;

/** Create a new virtual node.
 * @param node_id 48-bit unique Node ID
 * @param node_id_mask mask used for routing messages to this node
 * @param nmranet_if interface to bind the node to
 * @param model node decription
 * @param priv private data to store with the the node for later retrieveal
 * @return upon success, handle to the newly created node, else NULL
 */
node_t nmranet_node_create(node_id_t node_id, node_id_t node_id_mask, NMRAnetIF *nmranet_if, const char* model, void *priv)
{
    struct id_node  id_lookup;
    struct id_node *id_node;

    if (node_id == 0)
    {
        /* invalid Node ID */
        return NULL;
    }

    id_lookup.id = node_id;
    id_lookup.mask = 0;

    os_mutex_lock(&mutex);
    id_node = RB_FIND(id_tree, &idHead, &id_lookup);
    if (id_node == NULL)
    {
        id_node = malloc(sizeof(struct id_node));
        id_node->id = node_id;
        id_node->mask = node_id_mask;
        id_node->priv = malloc(sizeof(NodePriv));
        id_node->priv->nmranetIF = nmranet_if;
        id_node->priv->priv = priv;
        id_node->priv->state = NODE_UNINITIALIZED;
        id_node->priv->eventQueue = nmranet_queue_create();
        id_node->priv->datagramQueue = nmranet_queue_create();
        id_node->priv->consumerEvents = NULL;
        id_node->priv->producerEvents = NULL;
        id_node->priv->wait = NULL;
        id_node->priv->model = model;
        
        RB_INSERT(id_tree, &idHead, id_node);
    }
    os_mutex_unlock(&mutex);
    return id_node;
}

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

/** Change the state of a producer event and send an event report if required.
 * @param node node to register event to
 * @param event event number to register
 * @param state new state of the event
 */
void nmranet_node_event_producer_state(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&mutex);
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
                os_mutex_unlock(&mutex);
                return;
            }
        }
    }
    os_mutex_unlock(&mutex);
}

/** Add the event to the conumser list within a given node.
 * @param node node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
void nmranet_node_consumer_add(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;
    
    /* just to be safe, mask off upper bits */
    state &= 0x3;
    
    os_mutex_lock(&mutex);
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
    os_mutex_unlock(&mutex);
}

/** Add the event to the producer list within a given node.
 * @param node node to register event to
 * @param event event number to register
 * @param state initial state of the event
 */
void nmranet_node_producer_add(node_t node, uint64_t event, int state)
{
    struct id_node *n = (struct id_node*)node;
    
    /* just to be safe, mask off upper bits */
    state &= 0x3;
    
    os_mutex_lock(&mutex);
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
    os_mutex_unlock(&mutex);
}

/** Lookup the node handle for a given node ID.
 * @param node_id 48-bit unique Node ID
 * @return if it exists, handle to the node id, else NULL
 */
node_t nmranet_node(node_id_t node_id)
{
    struct id_node  id_lookup;
    struct id_node *id_node;

    id_lookup.id = node_id;

    os_mutex_lock(&mutex);
    id_node = RB_FIND(id_tree, &idHead, &id_lookup);
    os_mutex_unlock(&mutex);
    return id_node;
}

/** Move node into the initialized state.
 * @param node node instance to act on
 */
void nmranet_node_initialized(node_t node)
{
    if (node == NULL)
    {
        abort();
    }

    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&mutex);
    if (n->priv->state != NODE_INITIALIZED)
    {
        n->priv->state = NODE_INITIALIZED;
        verify_node_id_number(node);
        /* identify all of the events this node consumes */
        for (EventList *l = n->priv->consumerEvents; l != NULL; l = l->next)
        {
            for (unsigned int count = 0; count < l->count; count++)
            {
                event_consumer_identify(node, l->events[count], (l->state >> l->count) & 0x3);
            }
        }
        /* identify all of the events this node produces */
        for (EventList *l = n->priv->producerEvents; l != NULL; l = l->next)
        {
            for (unsigned int count = 0; count < l->count; count++)
            {
                event_producer_identify(node, l->events[count], (l->state >> l->count) & 0x3);
            }
        }
    }
    os_mutex_unlock(&mutex);
}

/** Obtain the Node ID of a node handle
 * @param node node to get a the Node ID from
 * @return 48-bit NMRAnet Node ID
 */
node_id_t nmranet_node_id(node_t node)
{
    struct id_node *n = (struct id_node*)node;
    
    return n->id;
}

/** Lookup the private data pointer for a given handle.
 * @param node node to get a the Node ID from
 * @return if it exists, handle to the node id, else NULL
 */
void *nmranet_node_private(node_t node)
{
    struct id_node *n = (struct id_node*)node;
    
    return n->priv->priv;
}

/** Wait for data to come in from the network.
 * @param node node to wait on
 */
void nmranet_node_wait(node_t node)
{
    struct id_node *n = (struct id_node*)node;
    os_sem_t        sem;

    os_mutex_lock(&mutex);
    if (nmranet_queue_empty(n->priv->eventQueue) &&
        nmranet_queue_empty(n->priv->datagramQueue))
    {
        /* nothing ready to be received */
        os_sem_init(&sem, 0);
        n->priv->wait = &sem;
        os_mutex_unlock(&mutex);

        /* wait for something interesting to happen */
        os_sem_wait(&sem);
        
        os_mutex_lock(&mutex);
        n->priv->wait = NULL;
    }
    os_mutex_unlock(&mutex);
}

/** Send a verify node id number message.
 * @param node Node to send message from
 */
static void verify_node_id_number(node_t node)
{
    struct id_node *n = (struct id_node*)node;
    char *buffer = nmranet_buffer_alloc(6);
    
    buffer[0] = (n->id >> 40) & 0xff;
    buffer[1] = (n->id >> 32) & 0xff;
    buffer[2] = (n->id >> 24) & 0xff;
    buffer[3] = (n->id >> 16) & 0xff;
    buffer[4] = (n->id >>  8) & 0xff;
    buffer[5] = (n->id >>  0) & 0xff;
    
    nmranet_buffer_advance(buffer, 6);
    
    node_handle_t dst = {0, 0};
    nmranet_node_write(node, MTI_VERIFIED_NODE_ID_NUMBER, dst, buffer);
}

/** Send an ident info reply message.
 * @param node Node to send message from
 * @param dst destination Node ID to respond to
 * @param description index 0 name of manufacturer
 *                    index 1 name of model
 *                    index 2 hardware revision
 *                    index 3 software revision
 */
static void ident_info_reply(node_t node, node_handle_t dst, const char *description[4])
{
    size_t  size = 4;
    char   *buffer;
    char   *pos;

    for (int i = 0; i < 4; i++)
    {
        size += strlen(description[i]) + 1;
    }

    buffer = nmranet_buffer_alloc(size);
    
    buffer[0] = 0x01;
    pos = nmranet_buffer_advance(buffer, 1);
    for (int i = 0; i < 4; i++)
    {
        size_t len = strlen(description[i]) + 1;
        memcpy(pos, description[i], len);
        pos = nmranet_buffer_advance(buffer, len);
    }
    pos[0] = 0x01;
    pos[1] = 0x00;
    pos[2] = 0x00;
    pos = nmranet_buffer_advance(buffer, 3);
    
    nmranet_node_write(node, MTI_IDENT_INFO_REPLY, dst, buffer);
}

/** This method can be called by any interface to indicate it has incoming data.
 * It is for interfaces that use full 48-bit Node ID's.
 * @param nmranet_if interface that the message came in on.
 * @param mti Message Type Indeicator
 * @param src source Node ID
 * @param dst destination Node ID
 * @param data data payload
 */
void nmranet_if_rx_data(struct nmranet_if *nmranet_if, uint16_t mti, node_handle_t src, node_id_t dst, const void *data)
{
    os_mutex_lock(&mutex);
    if (dst != 0)
    {
        struct id_node  id_lookup;
        struct id_node *id_node;
        id_lookup.id = dst;
        id_lookup.mask = 0;
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        if (id_node)
        {
            /* we own this id */
            if (id_node->priv->state != NODE_UNINITIALIZED)
            {
                /* the node is initialized */
                switch (mti)
                {
                    case MTI_VERIFY_NODE_ID_ADDRESSED:
                        verify_node_id_number(id_node);
                        break;
                    case MTI_IDENT_INFO_REQUEST:
                    {
                        const char *description[4] = {nmranet_manufacturer,
                                                      id_node->priv->model,
                                                      nmranet_hardware_rev,
                                                      nmranet_software_rev};
                        ident_info_reply(id_node, src, description);
                        break;
                    }
                    case MTI_DATAGRAM:
                        //nmranet_datagram_packet(mti, src, dst, data);
                        break;
                }
            }
        }
    }
    else
    {
        /* global message, handle subscribe based protocols first */
        switch (mti)
        {
            case MTI_EVENT_REPORT:
                nmranet_event_packet(mti, src, 0, data);
                break;
            default:
                /* global message, deliver all, non-subscribe */
                for (struct id_node * id_node = RB_MIN(id_tree, &idHead);
                     id_node != NULL;
                     id_node = RB_NEXT(id_tree, &idHead, id_node))
                {
                    if (id_node->priv->state != NODE_UNINITIALIZED)
                    {
                        /* the node is initialized */
                        switch (mti)
                        {
                            case MTI_VERIFY_NODE_ID_GLOBAL:
                                /* we own this id, it is initialized, respond */
                                verify_node_id_number(id_node);
                                break;
                        }
                    }
            }
        }
        if (data)
        {
            nmranet_buffer_free(data);
        }
    }
    os_mutex_unlock(&mutex);
}

/** Write a message from a node.
 * @param node node to write message from
 * @param mti Message Type Indicator
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_node_write(node_t node, uint16_t mti, node_handle_t dst, const void *data)
{
    struct id_node *n = (struct id_node*)node;
    NMRAnetIF *nmranet_if  = n->priv->nmranetIF;

    if (dst.id == 0 && dst.alias == 0)
    {
        /* broacast message */
        node_handle_t src = {n->id, 0};
        nmranet_if_rx_data(nmranet_if, mti, src, 0, data);
        (*nmranet_if->write)(nmranet_if, mti, n->id, dst, data);
    }
    else
    {
        struct id_node  id_lookup;
        struct id_node *id_node;
        id_lookup.id = dst.id;
        id_lookup.mask = 0;
        os_mutex_lock(&mutex);
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        os_mutex_unlock(&mutex);
        if (id_node)
        {
            /* loop-back */
            node_handle_t src = {n->id, 0};
            nmranet_if_rx_data(nmranet_if, mti, src, dst.id, data);
        }
        else
        {
            (*nmranet_if->write)(nmranet_if, mti, n->id, dst, data);
        }
    }
    return 0;
}

/** Post the reception of an event with to given node.
 * @param node to post event to
 * @param event event number to post
 */
void nmranet_node_post_event(node_t node, uint64_t event)
{
    struct id_node *n = (struct id_node*)node;
    
    os_mutex_lock(&mutex);

    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    *buffer = event;
    nmranet_buffer_advance(buffer, sizeof(uint64_t));
    nmranet_queue_insert(n->priv->eventQueue, buffer);

    if (n->priv->wait != NULL)
    {
        /* wakeup whoever is waiting */
        os_sem_post(n->priv->wait);
    }
    os_mutex_unlock(&mutex);
}

/** Grab an event from the event queue of the node.
 * @param node to grab event from
 * @return 0 if the queue is empty, else return the event number
 */
uint64_t nmranet_event_consume(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&mutex);
    uint64_t *event = nmranet_queue_next(n->priv->eventQueue);
    if (event != NULL)
    {
        os_mutex_unlock(&mutex);
        uint64_t result = *event;
        nmranet_buffer_free(event);
        return result;
    }
    os_mutex_unlock(&mutex);
    
    return 0;
}

/** Post the reception of a datagram with to given node.
 * @param node to post event to
 * @param datagram datagram to post
 */
void nmranet_node_post_datagram(node_t node, const void *datagram)
{
    struct id_node *n = (struct id_node*)node;
    
    os_mutex_lock(&mutex);

    nmranet_queue_insert(n->priv->eventQueue, datagram);

    if (n->priv->wait != NULL)
    {
        /* wakeup whoever is waiting */
        os_sem_post(n->priv->wait);
    }
    os_mutex_unlock(&mutex);
}

/** Grab a datagram from the datagram queue of the node.
 * @param node to grab datagram from
 * @return NULL if queue is empty, else pointer to the datagram
 */
Datagram *nmranet_datagram_consume(node_t node)
{
    struct id_node *n = (struct id_node*)node;

    os_mutex_lock(&mutex);
    Datagram *datagram = nmranet_queue_next(n->priv->eventQueue);
    os_mutex_unlock(&mutex);
    
    return datagram;
}

