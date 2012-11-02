/** \copyright
 * Copyright (c) 2012, Stuart W Baker
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file nmranet_if.c
 * This file defines a generic NMRAnet interface.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#include <sys/tree.h>
#include "if/nmranet_if.h"
#include "core/nmranet_buf.h"
#include "os/os.h"

/* Interface init prototypes */
NMRAnetIF *nmranet_lo_if_init(void);

/** One time initialization for NMRAnet interfaces. */
static os_thread_once_t if_once = OS_THREAD_ONCE_INIT;

/** Head of interface list. */
static NMRAnetIF *head = NULL;

/** Mutual exclusion for interfaces. */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

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
    NMRAnetIF *nmranetIF; /**< interface that owns this ID */
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

/** One time initialization for the NMRAnet interfaces.
 */
static void if_init(void)
{
    /* we should always have a local/loopback interface */
    head = nmranet_lo_if_init();
    head->next = NULL;
}

/** Get the local interface instance.
 * @return local interface instance
 */
NMRAnetIF *nmranet_lo_if(void)
{
    return head;
}

/** This method can be called by any interface to indicate it has incoming data.
 */
void nmranet_if_rx_data(struct nmranet_if *nmranet_if, uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    os_mutex_lock(&mutex);
    if (head == NULL)
    {
        /* the local interface is not initialized yet */
        if (data)
        {
            nmranet_buffer_free(data);
        }
        os_mutex_unlock(&mutex);
        return;
    }
    if (src != 0)
    {
        /* update the node id routing table */
        struct id_node  id_lookup;
        struct id_node *id_node;
        id_lookup.id = src;
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        if (id_node == NULL)
        {
            id_node = malloc(sizeof(struct id_node));
            id_node->id = src;
            RB_INSERT(id_tree, &idHead, id_node);
        }
        id_node->nmranetIF = nmranet_if;
    }
    if (dst != 0)
    {
        struct id_node  id_lookup;
        struct id_node *id_node;
        id_lookup.id = dst;
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        if (id_node)
        {
            /* we have a route for this Node ID so forward it on */
            if (id_node->nmranetIF && id_node->nmranetIF != nmranet_if)
            {
                (*id_node->nmranetIF->write)(id_node->nmranetIF, mti, src, dst, data);
            }
        }
    }
    else
    {
        /* global message */
        for (NMRAnetIF *now = head; now != NULL; now = now->next)
        {
            /** @todo should we always forward to our self or not */
            if (now != nmranet_if || now == head)
            {
                (*now->write)(now, mti, src, dst, data);
            }
        }
    }

    if (data)
    {
        nmranet_buffer_free(data);
    }
    os_mutex_unlock(&mutex);
}

/** Initialize a NMRAnet interface.
 * @param nmranet_if instance of this interface
 * @param node_id Node ID to associate with this interface
 */
void nmranet_if_init(NMRAnetIF *nmranet_if, node_id_t node_id)
{
    os_mutex_lock(&mutex);
    os_thread_once(&if_once, if_init);
    if (nmranet_if != NULL)
    {
        /* add our newly created interface to our list of interfaces */
        NMRAnetIF * current = head;
        while (current->next != NULL)
        {
            current = current->next;
        }
        current->next = nmranet_if;
        nmranet_if->next = NULL;
        /* identify everyone on this segment */
        (*nmranet_if->write)(nmranet_if, MTI_VERIFY_NODE_ID_GLOBAL, node_id, 0, NULL);
    }
    os_mutex_unlock(&mutex);
}

/** Send a verify node id number message.
 * @param node_id Node ID to place in message payload
 */
void nmranet_if_verify_node_id_number(node_id_t node_id)
{
    char *buffer = nmranet_buffer_alloc(6);
    
    buffer[0] = (node_id >> 40) & 0xff;
    buffer[1] = (node_id >> 32) & 0xff;
    buffer[2] = (node_id >> 24) & 0xff;
    buffer[3] = (node_id >> 16) & 0xff;
    buffer[4] = (node_id >>  8) & 0xff;
    buffer[5] = (node_id >>  0) & 0xff;
    
    nmranet_buffer_advance(buffer, 6);
    
    nmranet_if_rx_data(head, MTI_VERIFIED_NODE_ID_NUMBER, node_id, 0, buffer);
}
