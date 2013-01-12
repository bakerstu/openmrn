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
#include "core/nmranet_alias.h"
#include "nmranet_can.h"

/** Status values for an alias.
 */
typedef enum alias_status
{
    ALIAS_UNDER_TEST, /**< this is an alias we are trying to claim */
    ALIAS_DISCOVERY, /**< alias is in the process of being discovered */
    ALIAS_CONFLICT, /**< a conflict has been detected, still working out ID mapping */
    ALIAS_RESERVED, /**< The alias has been reserved for use */
    ALIAS_FREE /**< the alias is free for another request */
} AliasStatus;

/** Metadata about each alias
 */
typedef struct alias_metadata
{
    node_alias_t alias; /**< alias */
    os_timer_t timer; /**< timer used for establishing the connection */
    AliasStatus status; /**< status of node */
    //void *datagramBuffer; /**< buffer for holding datagrams */
} AliasMetadata;

/** Information about the interface. */
typedef struct nmranet_can_if
{
    NMRAnetIF nmranetIF; /**< generic NMRAnet interface info */
    int fd; /**< CAN hardware handle */
    node_id_t id; /**< node id of interface */
    alias_cache_t aliasCache; /**< list of downstream aliases */
    alias_cache_t upstreamCache; /**< list of upstream aliases */
    AliasMetadata *pool; /**< preallocated alias pool */
#if 0
    int poolWrIndex; /**< current index within the pool for getting an alias */
    int poolRdIndex; /**< current index within the pool for inserting an alias */
    int poolCount; /**< current number of aliases in the pool */
#endif
    os_mutex_t aliasMutex; /**< Mutex for managing aliases to Node IDs. */
    ssize_t (*read)(int, void*, size_t); /**< read method for device */
    ssize_t (*write)(int, const void*, size_t); /**< write method for device */
} NMRAnetCanIF;

/* prototypes */
static void claim_alias(NMRAnetCanIF *can_if, node_id_t node_id, node_alias_t alias);
static long long claim_alias_timeout(void *data1, void *data2);

/** This it the timeout for claiming an alias.  At this point, the alias will
 * either be claimed as a downstream node, or we can start using it.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a @ref alias_node typecast to a void*
 * @return OS_TIMER_NONE
 */
static long long claim_alias_timeout(void *data1, void *data2)
{
    NMRAnetCanIF  *can_if     = (NMRAnetCanIF*)data1;
    AliasMetadata *alias_meta = (AliasMetadata*)data2;

    os_mutex_lock(&can_if->aliasMutex);
    if (alias_meta->status != ALIAS_UNDER_TEST)
    {
        /* try again with a new alias */
        /** @todo should we do this in a background thread? */
        alias_meta->status = ALIAS_FREE;
        claim_alias(can_if, 0, nmranet_alias_generate(can_if->aliasCache));
    }
    else
    {
        alias_meta->status = ALIAS_RESERVED;
        /* Reserve the ID */
        struct can_frame frame;
        CAN_CONTROL_FRAME_INIT(frame, alias_meta->alias, RID_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
        if (result != (sizeof(struct can_frame)))
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
    for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        if (can_if->pool[i].status == ALIAS_FREE)
        {
            /* found an alias slot to use for the claim */
            can_if->pool[i].alias = alias;
            can_if->pool[i].status = ALIAS_UNDER_TEST;

            if (node_id == 0)
            {
                /* this is an anonymous request, use the interface Node ID */
                node_id = can_if->id;
            }
            
            struct can_frame frame[4];

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
            os_timer_start(can_if->pool[i].timer, MSEC_TO_NSEC(200));
            return;
        }
    }
}

/** Setup the relationship between an alias and a downstream node.
 * @param can_if interface received on
 * @param alias node alias
 * @param node_id Node ID
 */
static void downstream_alias_setup(NMRAnetCanIF *can_if, node_alias_t alias, node_id_t node_id)
{
    os_mutex_lock(&can_if->aliasMutex);
    nmranet_alias_add(can_if->aliasCache, node_id, alias);
    os_mutex_unlock(&can_if->aliasMutex);
}

/** Setup the relationship between an alias and a downstream node.
 * @param can_if interface received on
 * @param node_id Node ID
 * @return assigned alias
 */
static node_alias_t upstream_alias_setup(NMRAnetCanIF *can_if, node_id_t node_id)
{

    os_mutex_lock(&can_if->aliasMutex);
    /* we need to loop here until we find an alias to assign */
    for ( ; /* forever */ ; )
    {
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
        {
            if (can_if->pool[i].status == ALIAS_RESERVED)
            {
                node_alias_t alias = can_if->pool[i].alias;
                struct can_frame frame;

                /* Tell the segment who maps to this alias */
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
                
                /** @todo should we do this in a background thread? */
                can_if->pool[i].status = ALIAS_FREE;
                /* One might expect that we generate an alias from the upstream
                 * cache, however, since both the upstream and downstream cache
                 * uses the same seed, they would produce overlapping aliases.
                 */
                claim_alias(can_if, 0, nmranet_alias_generate(can_if->aliasCache));
                nmranet_alias_add(can_if->upstreamCache, node_id, alias);

                os_mutex_unlock(&can_if->aliasMutex);
                return alias;
            }
        }

        /* the pool is empty, wait and try again */
        os_mutex_unlock(&can_if->aliasMutex);
        sleep(1);
        os_mutex_lock(&can_if->aliasMutex);
    }
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
static int can_write(NMRAnetIF *nmranet_if, uint16_t mti, node_id_t src, node_handle_t dst, const void *data)
{
    NMRAnetCanIF *can_if = (NMRAnetCanIF*)nmranet_if;

    node_alias_t alias = nmranet_alias_lookup(can_if->upstreamCache, src);
    if (alias == 0)
    {
        /* we have never seen this node before, let's claim an alias for it */
        alias = upstream_alias_setup(can_if, src);
    }
    
    if (dst.alias != 0 || dst.id != 0)
    {
        if (dst.alias == 0)
        {
            /* look for a downstream match */
            dst.alias = nmranet_alias_lookup(can_if->aliasCache, dst.id);
        }
        
        if (dst.alias)
        {
            switch (mti)
            {
                default:
                {
                    struct can_frame frame;
                    size_t len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                    if (len > 8)
                    {
                        abort();
                    }
                    frame.can_id = can_identifier(mti, alias);
                    SET_CAN_FRAME_EFF(frame);
                    CLR_CAN_FRAME_RTR(frame);
                    CLR_CAN_FRAME_ERR(frame);
                    memcpy(frame.data, data, len);
                    frame.can_dlc = len;
                    int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
                    if (result < 0)
                    {
                        abort();
                    }
                    break;
                }
                case MTI_IDENT_INFO_REPLY:
                {
                    const char *buffer = data;
                    size_t len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                    while (len > 0)
                    {
                        struct can_frame frame;
                        frame.can_id = can_identifier(mti, alias);
                        SET_CAN_FRAME_EFF(frame);
                        CLR_CAN_FRAME_RTR(frame);
                        CLR_CAN_FRAME_ERR(frame);
                        frame.data[0] = dst.alias >> 8;
                        frame.data[1] = dst.alias & 0xff;
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
    else if (alias)
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
    }        
    return 0;
}

/** Decode global or addressed can frame.
 * @param can_if interface message is from
 * @param can_id can identifier
 * @param dlc data length code
 * @param data pointer to up to 8 bytes of data
 */
static void type_global_addressed(NMRAnetCanIF *can_if, uint32_t can_id, uint8_t dlc, uint8_t *data)
{
    node_handle_t src;
    src.alias = GET_CAN_ID_SOURCE(can_id);
    src.id = nmranet_alias_lookup_id(can_if->aliasCache, src.alias);

    if (GET_MTI_ADDRESS(nmranet_mti(can_id)))
    {
        /* addressed message */
        uint16_t address = (data[0] << 0) +
                           (data[1] << 8);
        address = be16toh(address);
        node_id_t dst = nmranet_alias_lookup_id(can_if->upstreamCache, GET_ADDRESSED_FIELD_DESTINATION(address));
        if (dst != 0)
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
            /** @todo rather than update our table, we need to do something else */
#if 0
            src = data[5];
            src |= (node_id_t)data[4] << 8;
            src |= (node_id_t)data[3] << 16;
            src |= (node_id_t)data[2] << 24;
            src |= (node_id_t)data[1] << 32;
            src |= (node_id_t)data[0] << 40;
            
            downstream_alias_setup(can_if, GET_CAN_ID_SOURCE(can_id), src);
#endif
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
#if 0
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
            node_handle_t src;
            node_handle_t dst;
            src.alias = GET_CAN_ID_SOURCE(can_id);
            dst.alias = GET_CAN_ID_SOURCE(can_id);
            src.id = nmranet_alias_lookup_id(can_if->aliasCache, src.alias);
            dst.id = nmranet_alias_lookup_id(can_if->aliasCache, dst.alias);
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
#endif
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
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
        {
            /** @todo need to look at upstream aliases, is this the right place? */
            if (can_if->pool[i].alias == GET_CAN_ID_SOURCE(frame.can_id))
            {
                switch (can_if->pool[i].alias)
                {
                    default:
                        /* fall through */
                    case ALIAS_FREE:
                        /* fall through */
                    case ALIAS_DISCOVERY:
                        /* fall through */
                    case ALIAS_CONFLICT:
                        /* totally normal, let's move on */
                        break;
                    case ALIAS_RESERVED:
                        /** @todo should we do this in a background thread? */
                        can_if->pool[i].status = ALIAS_FREE;
                        claim_alias(can_if, 0, nmranet_alias_generate(can_if->aliasCache));
                        break;
                    case ALIAS_UNDER_TEST:
                        /* we are trying to claim this alias, oops, mark as taken */
                        can_if->pool[i].status = ALIAS_CONFLICT;
                        break;
                } /* switch (can_if->pool[i].alias) */
            } /* if (can_if->pool[i].alias == GET_CAN_ID_SOURCE(frame.can_id)) */
        } /* for (unsigned int i = 0; i < MAX_POOL_SIZE; i++) */
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
 * @return handle to the NMRAnet interface
 */
NMRAnetIF *nmranet_can_if_init(node_id_t node_id, const char *device,
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
    can_if->pool = malloc(sizeof(AliasMetadata)*ALIAS_POOL_SIZE);
    for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        can_if->pool[i].status = ALIAS_FREE;
        can_if->pool[i].timer = os_timer_create(claim_alias_timeout, can_if, &can_if->pool[i]);
    }
    can_if->aliasCache = nmranet_alias_cache_create(node_id, DOWNSTREAM_ALIAS_CACHE_SIZE);
    can_if->upstreamCache = nmranet_alias_cache_create(node_id, UPSTREAM_ALIAS_CACHE_SIZE);
    os_mutex_init(&can_if->aliasMutex);

    /* prime the alias pool */
    for (int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        os_mutex_lock(&can_if->aliasMutex);
        /* initiate a claim on a new unique alias */
        claim_alias(can_if, 0, nmranet_alias_generate(can_if->aliasCache));
        os_mutex_unlock(&can_if->aliasMutex);
    }

    /* start the thread that will process received packets */
    os_thread_create(&thread_handle, 0, 1024, read_thread, &can_if->nmranetIF);

    return &can_if->nmranetIF;
}

