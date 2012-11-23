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
 * \file nmranet_can_if.c
 * This file defines an NMRAnet interface for generic CAN.
 *
 * @author Stuart W. Baker
 * @date 13 August 2012
 */

#include <endian.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/tree.h>
#include "if/nmranet_can_if.h"
#include "os/os.h"
#include "core/nmranet_buf.h"
#include "nmranet_can.h"

/** Status values for an alias.
 */
typedef enum alias_status
{
    ALIAS_DOWNSTREAM, /**< this is a downstream node */
    ALIAS_UPSTREAM, /**< this is an upstream node */
    ALIAS_UNDER_TEST, /**< this is an alias we are trying to claim */
    ALIAS_DISCOVERY, /**< alias is in the process of being discovered */
    ALIAS_CONFLICT /**< a conflict has been detected, still working out ID mapping */
} AliasStatus;

/** State values for an alias.
 */
typedef enum alias_state
{
    ALIAS_INHIBITED, /**< inhibited from non-control frame messages */
    ALIAS_PERMITTED, /**< permitted to transmit non-control frame messages */
} AliasState;

/** Metadata about each alias
 */
typedef struct alias_metadata
{
    node_id_t id; /**< Node ID belonging to alias */
    os_timer_t timer; /**< timer used for establishing the connection */
    AliasStatus status; /**< status of node */
    AliasState state; /**< state of node */
    void *datagramBuffer; /**< buffer for holding datagrams */
} AliasMetadata;

/** Red Black tree node for sorting by alias.
 */
struct alias_node
{
    /** entry metadata */
    RB_ENTRY(alias_node) entry;
    union
    {
        node_alias_t alias;
        int16_t key;
    };
    AliasMetadata *data; /**< the metadata for the node's alias */
};

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
    node_alias_t alias; /**< the alias for the node's ID */
};
    
/** Mathematically compare two aliases
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int16_t alias_compare(struct alias_node *a, struct alias_node *b)
{
    return a->key - b->key;
}

/** Mathematically compare two Node ID's.
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int64_t id_compare(struct id_node *a, struct id_node *b)
{
    return a->key - b->key;
}

/** The alias tree type */
RB_HEAD(alias_tree, alias_node);
/** The alias tree methods */
RB_GENERATE_STATIC(alias_tree, alias_node, entry, alias_compare);
/** The id tree type */
RB_HEAD(id_tree, id_node);
/** The id tree methods */
RB_GENERATE_STATIC(id_tree, id_node, entry, id_compare);

/** Information about the interface. */
typedef struct nmranet_can_if
{
    NMRAnetIF nmranetIF; /**< generic NMRAnet interface info */
#if defined (__linux__) || defined (__nuttx__)
    int fd; /**< CAN hardware handle */
#endif
    node_id_t id; /**< node id of interface */
    node_id_t seed; /**< the seed for the next alias generated */
    node_alias_t alias; /**< node alias of this interface */
    struct alias_node *pool[ALIAS_POOL_SIZE]; /**< preallocated alias pool */
    int poolWrIndex; /**< current index within the pool for getting an alias */
    int poolRdIndex; /**< current index within the pool for inserting an alias */
    int poolCount; /**< current number of aliases in the pool */
    struct alias_tree aliasHead; /**< alias tree head */
    struct id_tree idHead; /**< id tree head */
    os_mutex_t aliasMutex; /**< Mutex for managing aliases to Node IDs. */
    ssize_t (*read)(int, void*, size_t); /**< read method for device */
    ssize_t (*write)(int, const void*, size_t); /**< write method for device */
} NMRAnetCanIF;

/** Generate a 12-bit pseudo-random alias for a given node id or seed.
 * @param seed node id for the initial alias, contains next seed on return
 * @return pseudo-random 12-bit alias, an alias of zero is invalid
 */
static node_alias_t generate_alias(node_id_t *seed)
{
    node_alias_t alias;

    do
    {
        /* calculate the alias given the current seed */
        alias = (*seed ^ (*seed >> 12) ^ (*seed >> 24) ^ (*seed >> 36)) & 0xfff;

        /* calculate the next seed */
        *seed = ((((1 << 9) + 1) * (*seed) + constant)) & 0xffffffffffff;
    } while (alias == 0);

    return alias;
}

/** This it the timeout for claiming an alias.  At this point, the alias will
 * either be claimed as a downstream node, or we can start using it.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a @ref alias_node typecast to a void*
 * @return OS_TIMER_NONE
 */
static long long claim_alias_timeout(void *data1, void *data2)
{
    NMRAnetCanIF      *can_if     = (NMRAnetCanIF*)data1;
    struct alias_node *alias_node = (struct alias_node*)data2;

    os_mutex_lock(&can_if->aliasMutex);
    if (alias_node->data->status != ALIAS_UNDER_TEST)
    {
        /* try again with a new alias */
    }
    else
    {
        alias_node->data->status = ALIAS_UPSTREAM;
        /* Reserve the ID */
        if (alias_node->data->id == 0)
        {
            struct can_frame frame;
            CAN_CONTROL_FRAME_INIT(frame, alias_node->alias, RID_FRAME, 0);
            SET_CAN_FRAME_EFF(frame);
            CLR_CAN_FRAME_RTR(frame);
            CLR_CAN_FRAME_ERR(frame);
            int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
            if (result != (sizeof(struct can_frame)))
            {
                abort();
            }
            /* This is a pre-allocated node for the pool */            
            can_if->pool[can_if->poolWrIndex++] = alias_node;
            can_if->poolCount++;
        }
        else
        {
            abort();
        }
    }
    os_mutex_unlock(&can_if->aliasMutex);

    /* do not restart the timer */
    return OS_TIMER_NONE;
}

/** Put out a claim on an alias.  The alias mutex should be locked during this
 * call.
 * @param can_if interface to send claim to
 * @param node_id node id that is making the claim
 * @param alias alias that node is claiming
 */
static void claim_alias(NMRAnetCanIF *can_if, node_id_t node_id, node_alias_t alias)
{
    struct can_frame frame[4];

    struct alias_node *alias_node = malloc(sizeof(struct alias_node));
    
    alias_node->alias = alias;
    alias_node->data = malloc(sizeof(AliasMetadata));
    alias_node->data->id = node_id;
    alias_node->data->timer = os_timer_create(claim_alias_timeout, can_if, alias_node);
    alias_node->data->status = ALIAS_UNDER_TEST;
    alias_node->data->state = ALIAS_INHIBITED;
    alias_node->data->datagramBuffer = NULL;
    RB_INSERT(alias_tree, &can_if->aliasHead, alias_node);

    if (node_id == 0)
    {
        /* this is an anonymous request, use the interface Node ID */
        node_id = can_if->id;
    }
    
    CAN_CONTROL_FRAME_INIT(frame[0], alias, (node_id >> 36) & 0xfff, 7);
    CAN_CONTROL_FRAME_INIT(frame[1], alias, (node_id >> 24) & 0xfff, 6);
    CAN_CONTROL_FRAME_INIT(frame[2], alias, (node_id >> 12) & 0xfff, 5);
    CAN_CONTROL_FRAME_INIT(frame[3], alias, (node_id >>  0) & 0xfff, 4);
    SET_CAN_FRAME_EFF(frame[0]);
    SET_CAN_FRAME_EFF(frame[1]);
    SET_CAN_FRAME_EFF(frame[2]);
    SET_CAN_FRAME_EFF(frame[3]);
    CLR_CAN_FRAME_RTR(frame[0]);
    CLR_CAN_FRAME_RTR(frame[1]);
    CLR_CAN_FRAME_RTR(frame[2]);
    CLR_CAN_FRAME_RTR(frame[3]);
    CLR_CAN_FRAME_ERR(frame[0]);
    CLR_CAN_FRAME_ERR(frame[1]);
    CLR_CAN_FRAME_ERR(frame[2]);
    CLR_CAN_FRAME_ERR(frame[3]);
    int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame) * 4);
    if (result != (sizeof(struct can_frame) * 4))
    {
        abort();
    }

    /* wait 200+ msec */
    os_timer_start(alias_node->data->timer, MSEC_TO_NSEC(200));
}

/** Lookup a node ID from an existing alias.
 * @param can_if interface to locate from
 * @param alias alias associated with Node ID
 * @return 0 if Node ID does not exist or Node ID on success.
 */
static node_id_t lookup_id(NMRAnetCanIF *can_if, node_alias_t alias)
{
    struct alias_node  alias_lookup;
    struct alias_node *alias_node;

    alias_lookup.alias = alias;
    os_mutex_lock(&can_if->aliasMutex);
    alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &alias_lookup);
    os_mutex_unlock(&can_if->aliasMutex);
    
    if (alias_node != NULL)
    {
        return alias_node->data->id;
    }
    else
    {
        /* this little trick allows us to use aliases directly without a
         * mapping.  It allows for reduced memory footprint. */ 
        return 0xffff000000000000 | alias;
    }
}

/** Lookup existing alias, or allocate an alias from our pool.
 * @param can_if interface to allocate from
 * @param node_id node ID for the new alias
 * @param create create the alias if it does not exist
 * @return 0 if alias does not exist or alias on success.
 */
static node_alias_t lookup_alias(NMRAnetCanIF *can_if, node_id_t node_id, int create)
{
    struct id_node  id_lookup;
    struct id_node *id_node;
    node_alias_t    alias = 0;

    os_mutex_lock(&can_if->aliasMutex);
    /* check for an existing alias for our node ID */
    id_lookup.id = node_id;
    id_node = RB_FIND(id_tree, &can_if->idHead, &id_lookup);

    if (id_node)
    {
        alias = id_node->alias;
    }
    else if (create)
    {
        /* there is no existing alias, create one */
        while (can_if->poolCount <= 0)
        {
            /* the pool is empty, wait and try again */
            os_mutex_unlock(&can_if->aliasMutex);
            sleep(1);
            os_mutex_lock(&can_if->aliasMutex);
        }
        /* grab the next available alias from the pool */
        struct alias_node *alias_node = can_if->pool[can_if->poolRdIndex++];
        can_if->poolCount--;

        alias_node->data->id = node_id;
        alias = alias_node->alias;
        
        /* associate the alias with a node ID in the node ID tree */
        id_node = malloc(sizeof(struct id_node));
        id_node->id = node_id;
        id_node->alias = alias;
        RB_INSERT(id_tree, &can_if->idHead, id_node);
        
        /* Tell the segment who maps to this alias */
        struct can_frame frame;
        CAN_CONTROL_FRAME_INIT(frame, alias, AMD_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        frame.can_dlc = 6;
        frame.data[0] = (node_id >> 40) & 0xff;
        frame.data[1] = (node_id >> 32) & 0xff;
        frame.data[2] = (node_id >> 24) & 0xff;
        frame.data[3] = (node_id >> 16) & 0xff;
        frame.data[4] = (node_id >>  8) & 0xff;
        frame.data[5] = (node_id >>  0) & 0xff;
        
        int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
        if (result != sizeof(struct can_frame))
        {
            abort();
        }
        alias_node->data->state = ALIAS_PERMITTED;

        /* start looking for a new unique alias to fill the vacant spot left
         * in the pool
         */
        node_alias_t new_alias;
        do
        {
            struct alias_node alias_lookup;

            new_alias = generate_alias(&can_if->seed);
            alias_lookup.alias = new_alias;
            alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &alias_lookup);
        }
        while (alias_node != NULL);
        claim_alias(can_if, 0, new_alias);
    }
    else if ((node_id & 0xffff000000000000) == 0xffff000000000000)
    {
        /* this little trick allows us to use aliases directly without a
         * mapping.  It allows for reduced memory footprint. */ 
        alias = node_id & 0xffff;
    }

 /* if (id_node == NULL) */
    os_mutex_unlock(&can_if->aliasMutex);

    return alias;
}

/** Get the NMRAnet MTI from a can identifier.
 * @param can_id CAN identifider
 * @return NMRAnet MTI
 */
static uint16_t nmranet_mti(uint32_t can_id)
{
    switch (GET_CAN_ID_CAN_FRAME_TYPE(can_id))
    {
        default:
            return GET_CAN_ID_MTI(can_id);
        case TYPE_DATAGRAM_ONE_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_FIRST_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_MIDDLE_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_FINAL_FRAME:
            return MTI_DATAGRAM;
    }
}

/** Get the CAN identifier from an NMRAnet mti and source.
 * @param mti NMRAnet MTI
 * @param src Source node alias
 * @return CAN identifier
 */
static uint32_t can_identifier(uint16_t mti, node_alias_t src)
{
    return ((src << CAN_ID_SOURCE_SHIFT        )                  ) +
           ((mti << CAN_ID_MTI_SHIFT           ) & CAN_ID_MTI_MASK) +
           ((1   << CAN_ID_CAN_FRAME_TYPE_SHIFT)                  ) +
           ((1   << CAN_ID_FRAME_TYPE_SHIFT    )                  ) +
           ((1   << CAN_ID_PRIORITY_SHIFT      )                  );
}

/** Write a message onto the CAN bus.
 * @param nmranet_if interface to write message to
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
static int can_write(NMRAnetIF *nmranet_if, uint16_t mti, node_id_t src, node_id_t dst, const void *data)
{
    NMRAnetCanIF *can_if = (NMRAnetCanIF*)nmranet_if;

    if (dst)
    {
        node_alias_t alias_dst = lookup_alias(can_if, dst, 0);
        node_alias_t alias_src = lookup_alias(can_if, src, 0);
        if (alias_dst)
        {
            switch (mti)
            {
                default:
                    break;
                case MTI_IDENT_INFO_REPLY:
                {
                    const char *buffer = data;
                    size_t len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                    while (len > 0)
                    {
                        struct can_frame frame;
                        frame.can_id = can_identifier(mti, alias_src);
                        SET_CAN_FRAME_EFF(frame);
                        CLR_CAN_FRAME_RTR(frame);
                        CLR_CAN_FRAME_ERR(frame);
                        frame.data[0] = alias_dst >> 8;
                        frame.data[1] = alias_dst & 0xff;
                        size_t seg_size = len < 6 ? len : 6;
                        memcpy(&frame.data[2], buffer, seg_size);
                        frame.can_dlc = 2 + seg_size;
                        int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
                        if (result < 0)
                        {
                            abort();
                        }

                        len -= seg_size;
                        buffer += seg_size;
                        
                    }
                    nmranet_buffer_free(data);
                    break;
                }
            }
        }
        else
        {
            /* There is nobody here by that node ID */
        }
    }
    else if (src)
    {
        /* lookup alias */
        node_alias_t alias = lookup_alias(can_if, src, 1);
        if (alias)
        {
            struct can_frame frame;
            frame.can_id = can_identifier(mti, alias);
            SET_CAN_FRAME_EFF(frame);
            CLR_CAN_FRAME_RTR(frame);
            CLR_CAN_FRAME_ERR(frame);
            if (data != NULL)
            {
                size_t len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                memcpy(frame.data, data, len);
                frame.can_dlc = len;
            }
            else
            {
                frame.can_dlc = 0;
            }
            int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
            if (result < 0)
            {
                abort();
            }
        } /* if (alias) */
    }        
    return 0;
}

/** Setup the relationship between an alias and a downstream node.
 * @param can_if interface received on
 * @param alias node alias
 * @param node_id Node ID
 */
static void downstream_alias_setup(NMRAnetCanIF *can_if, node_alias_t alias, node_id_t node_id)
{
    struct alias_node *alias_node;
    struct alias_node  alias_lookup;
    struct id_node    *id_node;
    struct id_node     id_lookup;
    
    os_mutex_lock(&can_if->aliasMutex);
    /* check for an existing record of this alias */
    alias_lookup.alias = alias;
    alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &alias_lookup);
    if (alias_node == NULL)
    {
        /* no record of this alias, let's create a new one */
        alias_node = malloc(sizeof(struct alias_node));
        alias_node->data = malloc(sizeof(AliasMetadata));
        alias_node->alias = alias;
        RB_INSERT(alias_tree, &can_if->aliasHead, alias_node);
    }
    else
    {
        alias_node->alias = alias;
    }
        
    alias_node->data->id = node_id;
    alias_node->data->timer = NULL;
    alias_node->data->status = ALIAS_DOWNSTREAM;
    alias_node->data->state = ALIAS_PERMITTED;
    alias_node->data->datagramBuffer = NULL;
    
    /* check for an existing record of this Node ID */
    id_lookup.id = node_id;
    id_node = RB_FIND(id_tree, &can_if->idHead, &id_lookup);
    if (id_node == NULL)
    {
        /* no record of this Node ID, let's create a new one */
        id_node = malloc(sizeof(struct id_node));
        id_node->id = node_id;
        RB_INSERT(id_tree, &can_if->idHead, id_node);
    }
    else
    {
        id_node->id = node_id;
    }

    id_node->alias = alias_node->alias;
    
    os_mutex_unlock(&can_if->aliasMutex);
}

/** Decode global or addressed can frame.
 * @param can_if interface message is from
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
static void type_global_addressed(NMRAnetCanIF *can_if, uint32_t can_id, uint8_t dlc, uint8_t *data)
{
    node_id_t src = lookup_id(can_if, GET_CAN_ID_SOURCE(can_id));
    if (GET_MTI_ADDRESS(nmranet_mti(can_id)))
    {
        /* addressed message */
        uint16_t address = (data[0] << 0) +
                           (data[1] << 8);
        address = be16toh(address);
        node_id_t dst = lookup_id(can_if, GET_ADDRESSED_FIELD_DESTINATION(address));
        if (dst)
        {
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst, NULL);
        }
    }
    else
    {
        /* global message */
        if (nmranet_mti(can_id) == MTI_VERIFIED_NODE_ID_NUMBER)
        {
            /* lets update our alias table */
            src = data[5];
            src |= (node_id_t)data[4] << 8;
            src |= (node_id_t)data[3] << 16;
            src |= (node_id_t)data[2] << 24;
            src |= (node_id_t)data[1] << 32;
            src |= (node_id_t)data[0] << 40;
            
            downstream_alias_setup(can_if, GET_CAN_ID_SOURCE(can_id), src);
        }
        void *buffer = nmranet_buffer_alloc(dlc);
        memcpy(buffer, data, dlc);
        nmranet_buffer_advance(buffer, dlc);
        nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, 0, buffer);
    }
}

/** Decode datagram can frame.
 * @param can_if interface message is from
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
static void type_datagram(NMRAnetCanIF *can_if, uint32_t can_id, uint8_t dlc, uint8_t *data)
{
    void *buffer;

    switch(GET_CAN_ID_CAN_FRAME_TYPE(can_id))
    {
        default:
            break;
        case TYPE_DATAGRAM_ONE_FRAME:
        {
            buffer = nmranet_buffer_alloc(dlc);
            memcpy(buffer, data, dlc);
            nmranet_buffer_advance(buffer, dlc);
            node_id_t src = lookup_id(can_if, GET_CAN_ID_SOURCE(can_id));
            node_id_t dst = lookup_id(can_if, GET_CAN_ID_DST(can_id));
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst, buffer);
            break;
        }
        case TYPE_DATAGRAM_FIRST_FRAME:
        {
            struct alias_node  lookup_node;
            struct alias_node *alias_node;
            
            lookup_node.alias = GET_CAN_ID_SOURCE(can_id);
            
            os_mutex_lock(&can_if->aliasMutex);
            alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &lookup_node);
            /** @todo handle case where we are already receiving a datagram */
            alias_node->data->datagramBuffer = nmranet_buffer_alloc(72);
            memcpy(alias_node->data->datagramBuffer, data, dlc);
            nmranet_buffer_advance(alias_node->data->datagramBuffer, dlc);
            os_mutex_unlock(&can_if->aliasMutex);
            break;
        }
        case TYPE_DATAGRAM_MIDDLE_FRAME:
        {
            struct alias_node  lookup_node;
            struct alias_node *alias_node;
            
            lookup_node.alias = GET_CAN_ID_SOURCE(can_id);
            
            os_mutex_lock(&can_if->aliasMutex);
            alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &lookup_node);
            if (nmranet_buffer_available(alias_node->data->datagramBuffer) < 16)
            {
                /** @todo Not enough room in buffer, handle this error */
                break;
            }
            buffer = nmranet_buffer_position(alias_node->data->datagramBuffer);
            memcpy(buffer, data, dlc);
            nmranet_buffer_advance(alias_node->data->datagramBuffer, dlc);
            os_mutex_unlock(&can_if->aliasMutex);
            break;
        }
        case TYPE_DATAGRAM_FINAL_FRAME:
        {
            struct alias_node  lookup_node;
            struct alias_node *alias_node;
            
            lookup_node.alias = GET_CAN_ID_SOURCE(can_id);
            
            os_mutex_lock(&can_if->aliasMutex);
            alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &lookup_node);
            if (nmranet_buffer_available(alias_node->data->datagramBuffer) < 8)
            {
                /** @todo Not enough room in buffer, handle this error */
                break;
            }
            buffer = nmranet_buffer_position(alias_node->data->datagramBuffer);
            memcpy(buffer, data, dlc);
            nmranet_buffer_advance(alias_node->data->datagramBuffer, dlc);
            buffer = alias_node->data->datagramBuffer;
            alias_node->data->datagramBuffer = NULL;
            os_mutex_unlock(&can_if->aliasMutex);
            node_id_t src = lookup_id(can_if, GET_CAN_ID_SOURCE(can_id));
            node_id_t dst = lookup_id(can_if, GET_CAN_ID_DST(can_id));
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst, buffer);
            break;
        }
    }
}

/** Decode Check ID CAN control frame.
 * @param ccr CAN control frame
 */
static void ccr_cid_frame(uint32_t ccr)
{
}

/** Decode Reserve ID CAN control frame.
 * @param ccr CAN control frame
 */
static void ccr_rid_frame(uint32_t ccr)
{
}

/** Decode Alias Map Definition CAN control frame.
 * @param can_if interface AMD received on
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
static void ccr_amd_frame(NMRAnetCanIF *can_if, uint32_t ccr, uint8_t data[])
{
    node_id_t node_id;

    node_id = data[5];
    node_id |= (node_id_t)data[4] << 8;
    node_id |= (node_id_t)data[3] << 16;
    node_id |= (node_id_t)data[2] << 24;
    node_id |= (node_id_t)data[1] << 32;
    node_id |= (node_id_t)data[0] << 40;
    
    downstream_alias_setup(can_if, GET_CAN_CONTROL_FRAME_SOURCE(ccr), node_id);
}

/** Thread for reading the data from the interface.
 * @param data pointer to an NMRAnetCanIF for this interface
 * @return NULL, should never return
 */
static void *read_thread(void *data)
{
    NMRAnetCanIF *can_if = (NMRAnetCanIF*)data;

    for ( ; /* forever */ ; )
    {
        struct can_frame frame;
        int result = (*can_if->read)(can_if->fd, &frame, sizeof(struct can_frame));
        if (result < 0)
        {
            abort();
        }
        
        /* address any abnormalities */
        if (IS_CAN_FRAME_ERR(frame) ||
            IS_CAN_FRAME_RTR(frame) ||
            !IS_CAN_FRAME_EFF(frame))
        {
            /** @todo need to handle these conditions properly */
            abort();
        }
        
        os_mutex_lock(&can_if->aliasMutex);
        /* check the metadata for the source alias */
        struct alias_node  alias_lookup;
        struct alias_node *alias_node;
        alias_lookup.alias = GET_CAN_ID_SOURCE(frame.can_id);
        alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &alias_lookup);
        if (alias_node)
        {
            switch (alias_node->data->status)
            {
                case ALIAS_UPSTREAM:
                    /** @todo this case is bad, alias conflict, need to address */
                    break;
                case ALIAS_UNDER_TEST:
                {
                    /* we are trying to claim this alias, oops, mark as taken */
                    struct can_frame send_frame;
                    alias_node->data->status = ALIAS_CONFLICT;
                    
                    /* lets find out who this alias belongs to */
                    SET_CAN_ID_FIELDS(send_frame.can_id, can_if->alias,
                                      MTI_VERIFY_NODE_ID_ADDRESSED,
                                      TYPE_GLOBAL_ADDRESSED, 1, 1);
                    SET_CAN_FRAME_EFF(send_frame);
                    CLR_CAN_FRAME_RTR(send_frame);
                    CLR_CAN_FRAME_ERR(send_frame);
                    uint16_t data = GET_CAN_ID_SOURCE(frame.can_id);
                    data = htobe16(data);
                    memcpy(send_frame.data, &data, sizeof(uint16_t));
                    send_frame.can_dlc = sizeof(uint16_t);
                    int result = (*can_if->write)(can_if->fd, &send_frame, sizeof(struct can_frame));
                    if (result != sizeof(struct can_frame))
                    {
                        abort();
                    }
                    break;
                }
                case ALIAS_DOWNSTREAM:
                    /* fall through */
                case ALIAS_DISCOVERY:
                    /* fall through */
                case ALIAS_CONFLICT:
                    /* totally normal, let's move on */
                    break;
            } /* switch (alias_node->data->state) */
        } /* if (alias_node) */
        os_mutex_unlock(&can_if->aliasMutex);

        if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0)
        {
            switch (GET_CAN_CONTROL_FRAME_SEQUENCE(frame.can_id))
            {
                default:
                    /* this is another protocol, let's grab the next frame */
                    continue;
                case 0x4:
                    /* fall through */
                case 0x5:
                    /* fall through */
                case 0x6:
                    /* fall through */
                case 0x7:
                    ccr_cid_frame(frame.can_id);
                    /* we are done decoding, let's grab the next frame */
                    continue;
                case 0x0:
                    switch (GET_CAN_CONTROL_FRAME_FIELD(frame.can_id))
                    {
                        default:
                            /* unknown field, let's grab the next frame */
                        case RID_FRAME:
                            ccr_rid_frame(frame.can_id);
                            break;
                        case AMD_FRAME:
                            ccr_amd_frame(can_if, frame.can_id, frame.data);
                            break;
                        case AMI_FRAME:
                        case AMR_FRAME:
                            break;
                    } /* switch (GET_CAN_CONTROL_FRAME_FIELD(frame.can_id)) */
                    break;
            } /* switch (GET_CAN_CONTROL_FRAME_SEQUENCE(frame.can_id)) */
        } /* if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0) */
        else
        {
            /** find the proper packet decoder */
            switch(GET_CAN_ID_CAN_FRAME_TYPE(frame.can_id))
            {
                default:
                    break;
                case TYPE_GLOBAL_ADDRESSED:
                    type_global_addressed(can_if, frame.can_id, frame.can_dlc, frame.data);
                    break;
                case TYPE_DATAGRAM_ONE_FRAME:
                    /* fall through */
                case TYPE_DATAGRAM_FIRST_FRAME:
                    /* fall through */
                case TYPE_DATAGRAM_MIDDLE_FRAME:
                    /* fall through */
                case TYPE_DATAGRAM_FINAL_FRAME:
                    type_datagram(can_if, frame.can_id, frame.can_dlc, frame.data);
                    break;
                case TYPE_STREAM_DATA:
                    break;
            } /* switch(GET_CAN_ID_CAN_FRAME_TYPE(frame.can_id) */
        } /* if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0), else */
    } /* for ( ; forever ; ) */
    
    return NULL;
}

/** Initialize a can interface.
 * @param node_id node ID of interface
 * @param device description for this instance
 * @param if_read read method for this interface
 * @param if_write write method for this interface
 */
void nmranet_can_if_init(node_id_t node_id, const char *device,
                         ssize_t (*if_read)(int, void*, size_t),
                         ssize_t (*if_write)(int, const void*, size_t))
{
    NMRAnetCanIF *can_if = malloc(sizeof(NMRAnetCanIF));

    os_thread_t thread_handle;
    
    can_if->nmranetIF.write = can_write;
    can_if->read = if_read;
    can_if->write = if_write;
    can_if->fd = open(device, O_RDWR);
    if (can_if->fd < 0)
    {
        abort();
    }
    can_if->id = node_id;
    can_if->seed = node_id;
    can_if->poolWrIndex = 0;
    can_if->poolRdIndex = 0;
    can_if->poolCount = 0;
    RB_INIT(&can_if->aliasHead);
    RB_INIT(&can_if->idHead);
    os_mutex_init(&can_if->aliasMutex);

    /* start the thread that will process received packets */
    os_thread_create(&thread_handle, 0, 2048, read_thread, &can_if->nmranetIF);

    struct alias_node *alias_node;
    node_alias_t alias;

    /* prime the alias pool */
    for (int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        os_mutex_lock(&can_if->aliasMutex);
        /* get a new unique alias */
        do
        {
            struct alias_node alias_lookup;

            alias = generate_alias(&can_if->seed);
            alias_lookup.alias = alias;
            alias_node = RB_FIND(alias_tree, &can_if->aliasHead, &alias_lookup);
        }
        while (alias_node != NULL);

        claim_alias(can_if, 0, alias);
        os_mutex_unlock(&can_if->aliasMutex);
    }

    /* initialize the upper layer */
    nmranet_if_init(&can_if->nmranetIF);
    
    return;
}

