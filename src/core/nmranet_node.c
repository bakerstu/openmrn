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
#include "core/nmranet_node_private.h"
#include "core/nmranet_datagram.h"
#include "core/nmranet_event.h"
#include "if/nmranet_if.h"
#include "nmranet_config.h"

static void verify_node_id_number(node_t node);

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
os_mutex_t nodeMutex = OS_RECURSIVE_MUTEX_INITIALIZER;

/** Create a new virtual node.
 * @param node_id 48-bit unique Node ID
 * @param nmranet_if interface to bind the node to
 * @param model node decription
 * @param priv private data to store with the the node for later retrieveal
 * @return upon success, handle to the newly created node, else NULL
 */
node_t nmranet_node_create(node_id_t node_id, NMRAnetIF *nmranet_if, const char* model, void *priv)
{
    struct id_node  id_lookup;
    struct id_node *id_node;

    if (node_id == 0)
    {
        /* invalid Node ID */
        return NULL;
    }

    id_lookup.id = node_id;

    os_mutex_lock(&nodeMutex);
    id_node = RB_FIND(id_tree, &idHead, &id_lookup);
    if (id_node == NULL)
    {
        id_node = malloc(sizeof(struct id_node));
        id_node->id = node_id;
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
        id_node->priv->userName = NULL;
        id_node->priv->userDescription = NULL;
        id_node->priv->txDatagram = NULL;
        id_node->priv->datagramTimer = os_timer_create(nmranet_datagram_timeout, id_node, NULL);
        RB_INSERT(id_tree, &idHead, id_node);
    }
    os_mutex_unlock(&nodeMutex);
    return id_node;
}

/** Set the user name of the node for simple ident protocol.
 * @param node node to set attribute on
 * @param user_name string to use for user name
 */
void nmranet_node_user_name(node_t node, const char *user_name)
{
    struct id_node *id_node = node;
    id_node->priv->userName = user_name;
}

/** Set the user description of the node for simple ident protocol.
 * @param node node to set attribute on
 * @param user_description string to use for user description
 */
void nmranet_node_user_description(node_t node, const char *user_description)
{
    struct id_node *id_node = node;
    id_node->priv->userName = user_description;
}

/** Get a handle to the next node in the list of nodes.
 * @param node previous node in the list, NULL to start from beginning
 * @return handle to next node in the list, NULL if at end of list
 */
node_t nmranet_node_next(node_t node)
{
    struct id_node *id_node = node;

    if (node == NULL)
    {
        return RB_MIN(id_tree, &idHead);
    }
    
    return RB_NEXT(id_tree, &idHead, id_node);
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

    os_mutex_lock(&nodeMutex);
    id_node = RB_FIND(id_tree, &idHead, &id_lookup);
    os_mutex_unlock(&nodeMutex);
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

    os_mutex_lock(&nodeMutex);
    if (n->priv->state != NODE_INITIALIZED)
    {
        n->priv->state = NODE_INITIALIZED;
        verify_node_id_number(node);
        /* identify all of the events this node produces and consumes */
        nmranet_identify_consumers(node, 0, EVENT_ALL_MASK);
        nmranet_identify_producers(node, 0, EVENT_ALL_MASK);
    }
    os_mutex_unlock(&nodeMutex);
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

/** Get the 48-bit Node ID from a handle.
 * @param node node to get a the Node ID for
 * @param h handle to grab node_id from
 * @return 48-bit NMRAnet Node ID, 0 if not found.
 */
node_id_t nmranet_node_id_from_handle(node_t node, node_handle_t h)
{
    struct id_node *n = (struct id_node*)node;

    if (h.id)
    {
        return h.id;
    }
    else if (h.alias)
    {
        if (n->priv->nmranetIF->lookup_id)
        {
            return n->priv->nmranetIF->lookup_id(n->priv->nmranetIF, n->id, h.alias);
        }
    }
    return 0;
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
 * @param timeout timeout in nanoseconds, 0 to return right way, OS_WAIT_FOREVER
          to wait forever.
   @return number of messages pending, else 0 on timeout
 */
int nmranet_node_wait(node_t node, long long timeout)
{
    struct id_node *n = (struct id_node*)node;
    os_sem_t        sem;
    int             pending;

    os_mutex_lock(&nodeMutex);
    
    if (nmranet_queue_empty(n->priv->eventQueue) &&
        nmranet_queue_empty(n->priv->datagramQueue))
    {
        /* nothing ready to be received */
        os_sem_init(&sem, 0);
        n->priv->wait = &sem;
        os_mutex_unlock(&nodeMutex);

        /* wait for something interesting to happen */
        os_sem_timedwait(&sem, timeout);
        
        os_mutex_lock(&nodeMutex);
        n->priv->wait = NULL;
        os_sem_destroy(&sem);
    }

    /* check for number of pending messages */
    pending = nmranet_queue_pending(n->priv->eventQueue) +
              nmranet_queue_pending(n->priv->datagramQueue);
    
    /** @todo don't forget to add streams once supported */

    os_mutex_unlock(&nodeMutex);

    return pending;
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
 */
static void protocol_support_reply(node_t node, node_handle_t dst)
{
    uint64_t protocols = PROTOCOL_PROTOCOL_IDENTIFICATION |
                         PROTOCOL_DATAGRAM |
                         PROTOCOL_EVENT_EXCHANGE |
                         PROTOCOL_SIMPLE_NODE_INFORMATION;

    uint8_t *buffer = nmranet_buffer_alloc(6);
    
    buffer[0] = (protocols >> 40) & 0xff;
    buffer[1] = (protocols >> 32) & 0xff;
    buffer[2] = (protocols >> 24) & 0xff;
    buffer[3] = (protocols >> 16) & 0xff;
    buffer[4] = (protocols >>  8) & 0xff;
    buffer[5] = (protocols >>  0) & 0xff;
    
    nmranet_buffer_advance(buffer, 6);

    nmranet_node_write(node, MTI_PROTOCOL_SUPPORT_REPLY, dst, buffer);
}

/** Send an ident info reply message.
 * @param node Node to send message from
 * @param dst destination Node ID to respond to
 */
static void ident_info_reply(node_t node, node_handle_t dst)
{
    struct id_node *n = (struct id_node*)node;
    size_t  size = 8;
    char   *buffer;
    char   *pos;

    /* macro for condensing the size calculation code */
    #define ADD_STRING_SIZE(_str, _max)          \
    {                                            \
        if ((_str))                              \
        {                                        \
            size_t len = strlen((_str));         \
            size += len > (_max) ? (_max) : len; \
        }                                        \
    }

    /* macro for condensing the string insertion  code */
    #define INSERT_STRING(_str, _max)                      \
    {                                                      \
        if ((_str))                                        \
        {                                                  \
            size_t len = strlen((_str));                   \
            len = len > (_max) ? (_max) : len;             \
            memcpy(pos, (_str), len);                      \
            pos[len] = '\0';                               \
            pos = nmranet_buffer_advance(buffer, len + 1); \
        }                                                  \
        else                                               \
        {                                                  \
            pos[0] = '\0';                                 \
            pos = nmranet_buffer_advance(buffer, 1);       \
        }                                                  \
    }
    
    ADD_STRING_SIZE(nmranet_manufacturer, 40);
    ADD_STRING_SIZE(n->priv->model, 40);
    ADD_STRING_SIZE(nmranet_hardware_rev, 20);
    ADD_STRING_SIZE(nmranet_software_rev, 20);
    ADD_STRING_SIZE(n->priv->userName, 62);
    ADD_STRING_SIZE(n->priv->userDescription, 63);

    buffer = nmranet_buffer_alloc(size);
    
    buffer[0] = SIMPLE_NODE_IDENT_VERSION_A;
    pos = nmranet_buffer_advance(buffer, 1);
    
    INSERT_STRING(nmranet_manufacturer, 40);
    INSERT_STRING(n->priv->model, 40);
    INSERT_STRING(nmranet_hardware_rev, 20);
    INSERT_STRING(nmranet_software_rev, 20);

    pos[0] = SIMPLE_NODE_IDENT_VERSION_B;
    pos = nmranet_buffer_advance(buffer, 1);

    INSERT_STRING(n->priv->userName, 62);
    INSERT_STRING(n->priv->userDescription, 63);
    
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
    const uint8_t *bytes = data;

    os_mutex_lock(&nodeMutex);
    if (dst != 0)
    {
        /* !!! Because the message is addressed, the downstream protocol is
         * !!! responsible for freeing any data buffers.
         */
        struct id_node  id_lookup;
        struct id_node *id_node;
        id_lookup.id = dst;
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        if (id_node)
        {
            /* we own this id */
            if (id_node->priv->state != NODE_UNINITIALIZED)
            {
                /* the node is initialized */
                switch (mti)
                {
                    default:
                        if (data)
                        {
                            nmranet_buffer_free(data);
                        }
                        break;
                    case MTI_PROTOCOL_SUPPORT_INQUIRY:
                        protocol_support_reply(id_node, src);
                        break;
                    case MTI_VERIFY_NODE_ID_ADDRESSED:
                        verify_node_id_number(id_node);
                        break;
                    case MTI_IDENT_INFO_REQUEST:
                        ident_info_reply(id_node, src);
                        break;
                    case MTI_DATAGRAM_REJECTED:
                        /* fall through */
                    case MTI_DATAGRAM_OK:
                        /* fall through */
                    case MTI_DATAGRAM:
                        nmranet_datagram_packet(id_node, mti, src, data);
                        break;
                    case MTI_EVENTS_IDENTIFY_ADDRESSED:
                        if (data != NULL)
                        {
                            /* something went wrong or another node is
                             * behaving badly.
                             */
                            abort();
                        }
                        nmranet_event_packet_addressed(mti, src, id_node, data);
                        break;
                }
            }
        }
        else
        {
            if (data)
            {
                nmranet_buffer_free(data);
            }
        }
    }
    else
    {
        /* global message, handle subscribe based protocols first */
        switch (mti)
        {
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
                    /* fall through */
            case MTI_EVENT_REPORT:
                nmranet_event_packet_global(mti, src, data);
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
                            default:
                                break;
                            case MTI_VERIFY_NODE_ID_GLOBAL:
                                if (data != NULL)
                                {
                                    node_id_t id = ((node_id_t)bytes[5] <<  0) +
                                                   ((node_id_t)bytes[4] <<  8) +
                                                   ((node_id_t)bytes[3] << 16) +
                                                   ((node_id_t)bytes[2] << 24) +
                                                   ((node_id_t)bytes[1] << 32) +
                                                   ((node_id_t)bytes[0] << 40);
                                    if (id != id_node->id)
                                    {
                                        /* not a match, keep looking */
                                        continue;
                                    }
                                }
                                /* we own this id, it is initialized, respond */
                                verify_node_id_number(id_node);
                                break;
                        }
                    }
                }
                break;
        }
        /* global messages don't take possession of and free their data */
        if (data)
        {
            nmranet_buffer_free(data);
        }
    }
    os_mutex_unlock(&nodeMutex);
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
        os_mutex_lock(&nodeMutex);
        id_node = RB_FIND(id_tree, &idHead, &id_lookup);
        os_mutex_unlock(&nodeMutex);
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
