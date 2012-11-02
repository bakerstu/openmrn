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
#include <sys/tree.h>
#include "core/nmranet_node.h"
#include "core/nmranet_buf.h"
#include "os/os.h"
#include "endian.h"

/** Operational states of the node */
typedef enum node_state
{
    NODE_UNINITIALIZED = 0, /**< uninitialized node state */
    NODE_INITIALIZED /**< initialized node state */
} NodeState;

/** Private data about the node */
typedef struct node_priv
{
    NodeState state; /**< node's current operational state */
    nmranet_queue_t eventQueue; /**< queue for delivering events */
    os_sem_t *wait; /**< semaphore used to wakeup thread waiting on node */
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
    NodePriv priv; /**< Private data for the node ID */
};
    
/** Mathematically compare two Node ID's.
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int64_t id_compare(struct id_node *a, struct id_node *b)
{
    return a->key - b->key;
}

/** The id tree type. */
RB_HEAD(id_tree, id_node);
/** The id tree methods. */
RB_GENERATE_STATIC(id_tree, id_node, entry, id_compare);

/** id tree head. */
static struct id_tree idHead = RB_INITIALIZER(&idHead);

/** Mutual exclusion for nodes */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** Create a new virtual node.
 * @param node_id 48-bit unique Node ID
 * @return upon success, handle to the newly created node, else NULL
 */
node_t nmranet_node_create(node_id_t node_id)
{
    struct id_node  id_lookup;
    struct id_node *id_node;

    if (node_id == 0)
    {
        /* invalid Node ID */
        return NULL;
    }

    id_lookup.id = node_id;

    os_mutex_lock(&mutex);
    id_node = RB_FIND(id_tree, &idHead, &id_lookup);
    if (id_node == NULL)
    {
        id_node = malloc(sizeof(struct id_node));
        id_node->id = node_id;
        id_node->priv.state = NODE_UNINITIALIZED;
        id_node->priv.eventQueue = NULL;
        id_node->priv.wait = NULL;
        
        RB_INSERT(id_tree, &idHead, id_node);
    }
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
    if (n->priv.state != NODE_INITIALIZED)
    {
        n->priv.state = NODE_INITIALIZED;
        nmranet_if_verify_node_id_number(n->id);
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

/** Wait for data to come in from the network.
 * @param node node to wait on
 */
void nmranet_node_wait(node_t node)
{
    struct id_node *n = (struct id_node*)node;
    os_sem_t        sem;

    os_mutex_lock(&mutex);
    if (n->priv.eventQueue)
    {
        if (!nmranet_queue_empty(n->priv.eventQueue))
        {
            /* we already have an event ready to be received */
            return;
        }
    }

    os_sem_init(&sem, 0);
    n->priv.wait = &sem;
    os_mutex_unlock(&mutex);

    /* wait for something interesting to happen */
    os_sem_wait(&sem);
    
    os_mutex_lock(&mutex);
    n->priv.wait = NULL;
    os_mutex_unlock(&mutex);
}

/** Process a node management packet.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_node_packet(uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    struct id_node  id_lookup;
    struct id_node *id_node;

    switch (mti)
    {
        default:
            /* we don't care about these MTI's */
            break;
        case MTI_VERIFY_NODE_ID_ADDRESSED:
            if (dst != 0)
            {
                /* valid Node ID */
                id_lookup.id = dst;

                os_mutex_lock(&mutex);
                id_node = RB_FIND(id_tree, &idHead, &id_lookup);
                if (id_node != NULL)
                {
                    if (id_node->priv.state != NODE_UNINITIALIZED)
                    {
                        /* we own this id, it is initialized, respond */
                        nmranet_if_verify_node_id_number(dst);
                    }
                }
                os_mutex_unlock(&mutex);
            }
            break;
        case MTI_VERIFY_NODE_ID_GLOBAL:
            os_mutex_lock(&mutex);
            for (id_node = RB_MIN(id_tree, &idHead);
                 id_node != NULL;
                 id_node = RB_NEXT(id_tree, &idHead, id_node))
            {
                if (id_node->priv.state != NODE_UNINITIALIZED)
                {
                    /* we own this id, it is initialized, respond */
                    nmranet_if_verify_node_id_number(id_node->id);
                }
            }
            os_mutex_unlock(&mutex);
            break;
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
    if (n->priv.eventQueue == NULL)
    {
        n->priv.eventQueue = nmranet_queue_create();
    }

    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    *buffer = event;
    nmranet_buffer_advance(buffer, sizeof(uint64_t));
    nmranet_queue_insert(n->priv.eventQueue, buffer);

    if (n->priv.wait != NULL)
    {
        /* wakeup whoever is waiting */
        os_sem_post(n->priv.wait);
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
    if (n->priv.eventQueue)
    {
        uint64_t *event = nmranet_queue_next(n->priv.eventQueue);
        if (event != NULL)
        {
            os_mutex_unlock(&mutex);
            uint64_t result = *event;
            nmranet_buffer_free(event);
            return result;
        }
    }
    os_mutex_unlock(&mutex);
    
    return 0;
}

/** Produce an event from.
 * @param node node to produce event from
 * @param event event to produce
 */
void nmranet_event_produce(node_t node, uint64_t event)
{
    struct id_node *n = (struct id_node*)node;

    uint64_t *buffer = nmranet_buffer_alloc(sizeof(uint64_t));
    *buffer = htobe64(event);
    nmranet_buffer_advance(buffer, sizeof(uint64_t));

    nmranet_if_rx_data(nmranet_lo_if(), MTI_EVENT_REPORT, n->id, 0, buffer);
}

