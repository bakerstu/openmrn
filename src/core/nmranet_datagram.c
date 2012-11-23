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
 * \file nmranet_datagram.c
 * This file defines the handling of NMRAnet datagrams.
 *
 * @author Stuart W. Baker
 * @date 4 November 2012
 */

#include <stdlib.h>
#include <sys/tree.h>
#include "core/nmranet_datagram.h"
#include "core/nmranet_node.h"
#include "core/nmranet_buf.h"
#include "core/nmranet_train_control.h"
#include "os/os.h"

/** Mutual exclusion for socket library */
static os_mutex_t mutex = OS_MUTEX_INITIALIZER;

/** Red Black tree node for sorting by datagram type.
 */
struct datagram_node
{
    /** entry metadata */
    RB_ENTRY(datagram_node) entry;
    union
    {
        uint64_t datagram;
        int64_t key;
    };
};

/** Mathematically compare two datagram types.
 * @param a first event to compare
 * @param b second event to compare
 * @return (a - b)
 */
static int64_t datagram_compare(struct datagram_node *a, struct datagram_node *b)
{
    return a->key - b->key;
}

/** The datagram tree type */
RB_HEAD(datagram_tree, datagram_node);
/** The datagram  tree methods */
RB_GENERATE_STATIC(datagram_tree, datagram_node, entry, datagram_compare);
/** The node handle tree type */

/** Red Black tree node for sorting by node handle.
 */
struct node_node
{
    /** entry metadata */
    RB_ENTRY(node_node) entry;
    union
    {
        node_t node;
        intptr_t key;
    };
    struct datagram_tree datagramHead; /**< tree of registered datagrams */
};
    
/** Mathematically compare two node handles.
 * @param a first event to compare
 * @param b second event to compare
 * @return (a - b)
 */
static intptr_t node_compare(struct node_node *a, struct node_node *b)
{
    return a->key - b->key;
}

RB_HEAD(node_tree, node_node);
/** The node handle tree methods */
RB_GENERATE_STATIC(node_tree, node_node, entry, node_compare);
/** The node handle tree head. */
struct node_tree nodeHead = RB_INITIALIZER(&nodeHead);

/** Register for the consumption of a datagram type with a given node.
 * @param node node handle to register datagram to
 * @param datagram datagram identifier to register
 */
void nmranet_datagram_consumer(node_t node, uint64_t datagram)
{
    struct node_node     *node_node;
    struct node_node      node_lookup;
    struct datagram_node *datagram_node;
    struct datagram_node  datagram_lookup;
    
    /* initialize search criteria */
    node_lookup.node = node;
    datagram_lookup.datagram = datagram;

    os_mutex_lock(&mutex);
    /* look for an existing node handle entry */
    node_node = RB_FIND(node_tree, &nodeHead, &node_lookup);
    if (node_node == NULL)
    {
        /* a node handle entry does not exist, create one */
        node_node = malloc(sizeof(struct node_node));
        node_node->node = node;
        RB_INIT(&node_node->datagramHead);
        RB_INSERT(node_tree, &nodeHead, node_node);
    }

    /* look for an existing datagram entry */
    datagram_node = RB_FIND(datagram_tree, &node_node->datagramHead, &datagram_lookup);
    if (datagram_node == NULL)
    {
        /* a datagram entry does not exist, create one */
        datagram_node = malloc(sizeof(struct datagram_node));
        datagram_node->datagram = datagram;
        RB_INSERT(datagram_tree, &node_node->datagramHead, datagram_node);
    }
    os_mutex_unlock(&mutex);
}

/** Determine the datagram protocol (could be 1, 2, or 6 bytes).
 * @param datagram pointer to the beginning of the datagram
 */
static uint64_t nmranet_datagram_protocol(const void *datagram)
{
    const unsigned char *bytes = datagram;

    switch (bytes[0] & 0xF0)
    {
        default:
            return bytes[0];
        case 0xE0:
            return ((uint64_t)bytes[1] << 8) + ((uint64_t)bytes[2] << 0);
        case 0xF0:
            return ((uint64_t)bytes[5] <<  8) + ((uint64_t)bytes[6] <<  0) +
                   ((uint64_t)bytes[3] << 24) + ((uint64_t)bytes[4] << 16) +
                   ((uint64_t)bytes[1] << 40) + ((uint64_t)bytes[2] << 32);
    }
}

/** Produce a Datagram from a given node.
 * @param node node to produce datagram from
 * @param datagram datagram to produce (in the form of an nmranet buffer)
 */
void nmranet_datagram_produce(node_t node, const void *datagram)
{
    nmranet_if_rx_data(nmranet_lo_if(), MTI_DATAGRAM, nmranet_node_id(node), 0, datagram);
}

/** Process a datagram packet.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int nmranet_datagram_packet(uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    node_t node = nmranet_node(dst);
    
    struct node_node     *node_node;
    struct node_node      node_lookup;
    struct datagram_node *datagram_node;
    struct datagram_node  datagram_lookup;
    
    /* initialize search criteria */
    node_lookup.node = node;
    datagram_lookup.datagram = nmranet_datagram_protocol(data);

    os_mutex_lock(&mutex);
    /* look for an existing node handle entry */
    node_node = RB_FIND(node_tree, &nodeHead, &node_lookup);
    if (node_node)
    {
        datagram_node = RB_FIND(datagram_tree, &node_node->datagramHead, &datagram_lookup);
        if (datagram_node)
        {
            /* we are setup to consume this datagram */
            Datagram * datagram = nmranet_buffer_alloc(sizeof(Datagram));
            datagram->from = src;
            datagram->data = data;
            nmranet_buffer_advance(datagram, sizeof(Datagram));
            nmranet_node_post_datagram(node, datagram);
        }
        os_mutex_unlock(&mutex);
        return 0;
    }
    os_mutex_unlock(&mutex);
    
    return 1;
}

/** Consumed datagrams must be released so that their memory can be freed.
 * The application may hold onto the datagram for an appropriate amount of time
 * to process it before releasing it.  After being released, the memory holding
 * the datagram is no longer available for use by the application.
 * @param datagram datagram to release, this pointer is stale upon return
 */
void nmranet_datagram_release(Datagram *datagram)
{
    nmranet_buffer_free(datagram->data);
    nmranet_buffer_free(datagram);
}

/** Process the datagram automatically.  @ref nmranet_datagram_release will be
 * called on the datagram prior to the return of this method.
 * @param node node handle the datagram was received from
 * @param datagram datagram to process, this pointer is stale upon return
 */
void nmranet_datagram_process(node_t node, Datagram *datagram)
{
    switch (nmranet_datagram_protocol(datagram->data))
    {
        default:
            /* unknown protocol */
            break;
        case DATAGRAM_TRAIN_CONTROL:
            nmranet_train_control_process(node, datagram);
            break;
    }
    nmranet_datagram_release(datagram);
}

