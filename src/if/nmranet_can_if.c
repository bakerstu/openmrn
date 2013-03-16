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

#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <endian.h>
#include <sys/tree.h>
#include "if/nmranet_can_if.h"
#include "os/os.h"
#include "core/nmranet_buf.h"
#include "core/nmranet_alias.h"
#include "core/nmranet_datagram_private.h"
#include "nmranet_can.h"
#include "nmranet_config.h"

static int can_write(NMRAnetIF *nmranet_if, uint16_t mti, node_id_t src, node_handle_t dst, const void *data);

/** This is how long we should buffer a write pending a Node ID to alias mapping
 * request.
 */
#define WRITE_BUFFER_TIMEOUT 3000000000LL

/** This is how long we should wait for a Node ID lookup from an alias.
 */
#define LOOKUP_ID_TIMEOUT 3000000000LL

/** Status values for an alias.
 */
typedef enum alias_status
{
    ALIAS_UNDER_TEST, /**< this is an alias we are trying to claim */
    ALIAS_RESERVED, /**< the alias has been reserved for use */
    ALIAS_CONFLICT, /**< we discovered someone else already is using this alias */
    ALIAS_FREE /**< the alias is free for another request */
} AliasStatus;

/** Metadata about each alias
 */
typedef struct alias_metadata
{
    os_timer_t timer; /**< timer used for establishing the connection */
    AliasStatus status; /**< status of node */
    node_alias_t alias; /**< alias */
} AliasMetadata;

/** Structure for buffering an addressed write until we can lookup its alias.
 */
typedef struct write_buffer
{
    uint16_t mti; /**< MTI value, 0 if buffer not in use */
    node_id_t src; /**< source node ID */
    node_handle_t dst; /**< destination node ID and/or alias */
    const void *data; /**< message data */
    os_timer_t timer; /**< timeout on error */
} WriteBuffer;

/** Structure for looking up a node's ID based on its alias.
 */
typedef struct lookup_id
{
    os_mutex_t mutex; /**< mutual exclusion for looking up a Node ID */
    os_sem_t sem; /**< semaphore to pop when we have our answer */
    os_timer_t timer; /**< timeout on error */   
    node_id_t id; /**< id that was last looked up by alias */
    node_alias_t alias; /**< alias that is actively being looked up */
} LookupID;

struct datagram_node
{
    /** entry metadata */
    RB_ENTRY(datagram_node) entry;
    union
    {
        uint32_t alias;
        int32_t key;
    };
    Datagram *datagram; /**< datagram to track */
};

/** Mathematically compare two Node ID's.
 * @param a first node to compare
 * @param b second node to compare
 * @return (a - b)
 */
static int32_t datagram_compare(struct datagram_node *a, struct datagram_node *b)
{
    return a->key - b->key;
}

/** The datagram tree type. */
RB_HEAD(datagram_tree, datagram_node);
/** The datagram tree methods. */
RB_GENERATE_STATIC(datagram_tree, datagram_node, entry, datagram_compare);

/** Information about the interface. */
typedef struct nmranet_can_if
{
    NMRAnetIF nmranetIF; /**< generic NMRAnet interface info */
    WriteBuffer writeBuffer; /**< this buffer is used to wait on an Node ID to alias mapping */
    LookupID lookup_id; /**< this is used to lookup a 48-bit node ID based on a given alias */
    struct datagram_tree datagramHead; /**< tree for keeping track of receive datagrams */
    node_id_t id; /**< node id of interface */
    alias_cache_t aliasCache; /**< list of downstream aliases */
    alias_cache_t upstreamCache; /**< list of upstream aliases */
    os_mutex_t aliasMutex; /**< Mutex for managing aliases to Node IDs. */
    AliasMetadata *pool; /**< preallocated alias pool */
    ssize_t (*read)(int, void*, size_t); /**< read method for device */
    ssize_t (*write)(int, const void*, size_t); /**< write method for device */
    int fd; /**< CAN hardware handle */
} NMRAnetCanIF;

/* prototypes */
static void claim_alias(NMRAnetCanIF *can_if, node_id_t node_id, node_alias_t alias);
static long long claim_alias_timeout(void *data1, void *data2);

/* Release the write buffer from being in use.
 * @param can_if interface to act on
 */
static void write_buffer_release(NMRAnetCanIF *can_if)
{
    os_timer_stop(can_if->writeBuffer.timer);
    can_if->writeBuffer.mti = 0;
}

/* Determine if the write buffer is currently in use.
 * @param can_if interface to act on
 * @return true if in use, else false;
 */
static int write_buffer_in_use(NMRAnetCanIF *can_if) 
{
    return (can_if->writeBuffer.mti != 0);
}

/* Interrogate the write buffer for its destination Node ID.
 * @param can_if interface to act on
 * @return destination Node ID
 */
static node_id_t write_buffer_dst_id(NMRAnetCanIF *can_if)
{
    return can_if->writeBuffer.dst.id;
}

/** This is the timeout for giving up on an outstanding Node ID to alias
 * mapping request.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a unused
 * @return OS_TIMER_NONE
 */
long long write_buffer_timeout(void *data1, void *data2)
{
    NMRAnetCanIF *can_if = data1;

    /** @todo currently we fail silently, should we throw an error? */
    os_mutex_lock(&can_if->aliasMutex);
    can_if->writeBuffer.mti = 0;
    os_mutex_unlock(&can_if->aliasMutex);
    return OS_TIMER_NONE;
}

/** Buffer a rite message on the CAN bus waiting for its alias mapping.
 * @param nmranet_if interface to write message to
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
static void write_buffer_setup(NMRAnetCanIF *can_if, uint16_t mti, node_id_t src, node_handle_t dst, const void *data)
{
    can_if->writeBuffer.mti = mti;
    can_if->writeBuffer.src = src;
    can_if->writeBuffer.dst = dst;
    can_if->writeBuffer.data = data;
    
    os_timer_start(can_if->writeBuffer.timer, WRITE_BUFFER_TIMEOUT);
}

/** This is the timeout for giving up on a Node ID lookup from an alias.
 * mapping request.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a unused
 * @return OS_TIMER_NONE
 */
long long lookup_id_timeout(void *data1, void *data2)
{
    NMRAnetCanIF *can_if = data1;

    /** @todo currently we fail silently, should we throw an error? */
    os_mutex_lock(&can_if->aliasMutex);
    if (can_if->lookup_id.alias != 0)
    {
        can_if->lookup_id.id = 0;
        can_if->lookup_id.alias = 0;
        os_sem_post(&can_if->lookup_id.sem);
    }
    os_mutex_unlock(&can_if->aliasMutex);
    return OS_TIMER_NONE;
}

/** Lookup a 48-bit Node ID from a given alias.
 * @param nmranet_if interface to perform the lookup on
 * @param alias alias to lookup
 * @return 48-bit Node ID that maps to the alias
 */
static node_id_t lookup_id(NMRAnetIF *nmranet_if, node_id_t src, node_alias_t alias)
{
    NMRAnetCanIF *can_if = (NMRAnetCanIF*)nmranet_if;
    node_id_t id;

    os_mutex_lock(&can_if->lookup_id.mutex);
    os_mutex_lock(&can_if->aliasMutex);
    id = nmranet_alias_lookup_id(can_if->aliasCache, alias);
    if (id == 0)
    {
        id = nmranet_alias_lookup_id(can_if->upstreamCache, alias);
        if (id == 0)
        {
            can_if->lookup_id.id = 0;
            can_if->lookup_id.alias = alias;
            os_mutex_unlock(&can_if->aliasMutex);

            node_handle_t dst = {0, alias};
            can_write(nmranet_if, MTI_VERIFY_NODE_ID_ADDRESSED, src, dst, NULL);
            os_timer_start(can_if->lookup_id.timer, LOOKUP_ID_TIMEOUT);
            os_sem_wait(&can_if->lookup_id.sem);

            id = can_if->lookup_id.id;
        }
        else
        {
            os_mutex_unlock(&can_if->aliasMutex);
        }
    }
    else
    {
        os_mutex_unlock(&can_if->aliasMutex);
    }
    os_mutex_unlock(&can_if->lookup_id.mutex);

    return id;
}

/** This is the timeout for claiming an alias.  At this point, the alias will
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
        node_alias_t new_alias;
        do
        {
            new_alias = nmranet_alias_generate(can_if->upstreamCache);
        }
        while (nmranet_alias_lookup(can_if->aliasCache, new_alias) != 0);
        claim_alias(can_if, 0, new_alias);
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
                node_alias_t new_alias;
                do
                {
                    new_alias = nmranet_alias_generate(can_if->upstreamCache);
                }
                while (nmranet_alias_lookup(can_if->aliasCache, new_alias) != 0);
                claim_alias(can_if, 0, new_alias);
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
            return 0;
        case TYPE_GLOBAL_ADDRESSED:
            return GET_CAN_ID_MTI(can_id);
        case TYPE_DATAGRAM_ONE_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_FIRST_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_MIDDLE_FRAME:
            /* fall through */
        case TYPE_DATAGRAM_FINAL_FRAME:
            return MTI_DATAGRAM;
        case TYPE_STREAM_DATA:
            return MTI_STREAM_DATA;
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
            const uint8_t *buffer = data;
            size_t len;
            if (GET_MTI_DATAGRAM(mti))
            {
                const Datagram *datagram = data;
                /* datagrams and streams are special because the CAN ID
                 * also contains the destination alias.  These may also span
                 * multiple CAN frames.
                 */
                if (mti == MTI_STREAM_DATA)
                {
                    len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                }
                else
                {
                    len = datagram->size;
                    buffer = datagram->data;
                }
                
                for (int i = 0; len > 0; i++)
                {
                    int type;
                    if (mti == MTI_STREAM_DATA)
                    {
                        type = TYPE_STREAM_DATA;
                    }
                    else if (i == 0 && len <= 8)
                    {
                        type = TYPE_DATAGRAM_ONE_FRAME;
                    }
                    else if (i == 0)
                    {
                        type = TYPE_DATAGRAM_FIRST_FRAME;
                    }
                    else if (len <= 8)
                    {
                        type = TYPE_DATAGRAM_FINAL_FRAME;
                    }
                    else
                    {
                        type = TYPE_DATAGRAM_MIDDLE_FRAME;
                    }
                    struct can_frame frame;
                    SET_CAN_ID_FIELDS(frame.can_id, alias, dst.alias, type, 1, 1);
                    SET_CAN_FRAME_EFF(frame);
                    CLR_CAN_FRAME_RTR(frame);
                    CLR_CAN_FRAME_ERR(frame);
                    size_t seg_size = len < 8 ? len : 8;
                    memcpy(frame.data, buffer, seg_size);
                    frame.can_dlc = seg_size;
                    int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
                    if (result < 0)
                    {
                        abort();
                    }

                    len -= seg_size;
                    buffer += seg_size;
                }
            }
            else
            {
                if (data)
                {
                    len = nmranet_buffer_size(data) - nmranet_buffer_available(data);
                }
                else
                {
                    len = 0;
                }
                /* typically, only the simple node ident info reply will require
                 * the sending of more than one CAN frame.
                 */
                do
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
                    
                } while (len > 0);
            }
            /* release the buffer if we can */
            if (data)
            {
                nmranet_buffer_free(data);
            }
        }
        else
        {
            if (mti & MTI_ADDRESS_MASK)
            {
                /* this is an addressed message, so we need to buffer while
                 * we determine what alias this node ID belongs to.
                 */
                os_mutex_lock(&can_if->aliasMutex);
                while (write_buffer_in_use(can_if))
                {
                    os_mutex_unlock(&can_if->aliasMutex);
                    usleep(300);
                    os_mutex_lock(&can_if->aliasMutex);
                }
                write_buffer_setup(can_if, mti, src, dst, data);
                os_mutex_unlock(&can_if->aliasMutex);

                /* Alias Map Enquiry */
                struct can_frame frame;
                CAN_CONTROL_FRAME_INIT(frame, alias, AME_FRAME, 0);
                SET_CAN_FRAME_EFF(frame);
                CLR_CAN_FRAME_RTR(frame);
                CLR_CAN_FRAME_ERR(frame);
                frame.can_dlc = 6;
                frame.data[0] = (dst.id >> 40) & 0xff;
                frame.data[1] = (dst.id >> 32) & 0xff;
                frame.data[2] = (dst.id >> 24) & 0xff;
                frame.data[3] = (dst.id >> 16) & 0xff;
                frame.data[4] = (dst.id >>  8) & 0xff;
                frame.data[5] = (dst.id >>  0) & 0xff;
                int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
                if (result != (sizeof(struct can_frame)))
                {
                    abort();
                }
            }
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
        if (dlc < 2)
        {
            /* soft error, we throw this one away */
            return;
        }
        /* addressed message */
        uint16_t address = (data[0] << 0) +
                           (data[1] << 8);
        address = be16toh(address);
        node_id_t dst = nmranet_alias_lookup_id(can_if->upstreamCache, GET_ADDRESSED_FIELD_DESTINATION(address));
        if (dst != 0)
        {
            void *buffer;
            if (dlc > 2)
            {
                /* collect the data */
                buffer = nmranet_buffer_alloc(dlc - 2);
                memcpy(buffer, data, (dlc - 2));
                nmranet_buffer_advance(buffer, (dlc - 2));
            }
            else
            {
                buffer = NULL;
            }
                
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst, buffer);
        }
    }
    else
    {
        /* global message */
        if (nmranet_mti(can_id) == MTI_VERIFIED_NODE_ID_NUMBER)
        {
            int mapped = 0;
            node_id_t node_id;
            node_id = data[5];
            node_id |= (node_id_t)data[4] << 8;
            node_id |= (node_id_t)data[3] << 16;
            node_id |= (node_id_t)data[2] << 24;
            node_id |= (node_id_t)data[1] << 32;
            node_id |= (node_id_t)data[0] << 40;
            
            os_mutex_lock(&can_if->aliasMutex);
            if (src.id)
            {
                if (src.id != node_id)
                {
                    /* Looks like we have an existing mapping conflict.
                     * Lets remove existing mapping.
                     */
                    nmranet_alias_remove(can_if->aliasCache, src.alias);
                }
                else
                {
                    mapped = 1;
                }
            }
            /* Normally, with a buffered write, we are looking for a CCR AMD
             * frame.  However, this message has what we need, so if we get it
             * first, let's go ahead and use it.
             */
            if (write_buffer_in_use(can_if))
            {
                /* we have buffered a write for this node */
                if (node_id == write_buffer_dst_id(can_if))
                {
                    can_if->writeBuffer.dst.alias = src.alias;
                    can_if->nmranetIF.write(&can_if->nmranetIF,
                                            can_if->writeBuffer.mti,
                                            can_if->writeBuffer.src,
                                            can_if->writeBuffer.dst,
                                            can_if->writeBuffer.data);
                    if (mapped == 0)
                    {
                        /* We obviously use this alias, so let's cache it
                         * for later use.
                         */
                        nmranet_alias_add(can_if->aliasCache, node_id, src.alias);
                        write_buffer_release(can_if);
                        mapped = 1;
                    }
                }
            }
            else if (src.id && src.id != node_id)
            {
                /* we already had this mapping, we need to replace it */
                nmranet_alias_add(can_if->aliasCache, node_id, src.alias);
            }
            if (can_if->lookup_id.alias == src.alias)
            {
                /* we are performing a Node ID lookup based on an alias 
                 * and we found a match
                 */
                can_if->lookup_id.id = node_id;
                can_if->lookup_id.alias = 0;
                os_timer_stop(&can_if->lookup_id.timer);
                os_sem_post(&can_if->lookup_id.sem);
                if (mapped == 0)
                {
                    /* We obviously use this alias, so let's cache it
                     * for later use.
                     */
                    nmranet_alias_add(can_if->aliasCache, node_id, src.alias);
                    write_buffer_release(can_if);
                    mapped = 1;
                }
            }
            os_mutex_unlock(&can_if->aliasMutex);
            /* update the source Node ID */
            src.id = node_id;
        }
        void *buffer;
        if (dlc)
        {
            /* collect the data */
            buffer = nmranet_buffer_alloc(dlc);
            memcpy(buffer, data, dlc);
            nmranet_buffer_advance(buffer, dlc);
        }
        else
        {
            buffer = NULL;
        }
        nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, 0, buffer);
    }
}

/** Send a datagram error from the receiver to the sender.
 * @param can_if interface message is from
 * @param src source alias to send message from
 * @param dst destination alias to send message to
 * @param error_code error value to send
 */
static void datagram_rejected(NMRAnetCanIF *can_if, node_alias_t src, node_alias_t dst, uint16_t error_code)
{
    /* no buffer available, let the sender know */
    struct can_frame frame;
    frame.can_id = can_identifier(MTI_DATAGRAM_REJECTED, src);
    SET_CAN_FRAME_EFF(frame);
    CLR_CAN_FRAME_RTR(frame);
    CLR_CAN_FRAME_ERR(frame);
    frame.data[0] = dst >> 8;
    frame.data[1] = dst & 0xff;
    frame.data[2] = error_code >> 8;
    frame.data[3] = error_code & 0xff;
    frame.can_dlc = 4;
    int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
    if (result < 0)
    {
        abort();
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
    /* There is no need to use a mutex to access the datagram tree.  This is
     * because this function can only be called from a single thread, the
     * receive thread.  If this changes in the future, a mutex must be added
     * to protect this tree.
     */
    node_handle_t src;
    src.alias = GET_CAN_ID_SOURCE(can_id);
    src.id = nmranet_alias_lookup_id(can_if->aliasCache, src.alias);
    uint16_t dst_alias = GET_CAN_ID_DST(can_id);
    node_id_t dst_id = nmranet_alias_lookup_id(can_if->upstreamCache, dst_alias);

    if (dst_id == 0)
    {
        /* nobody here by that ID, not for us */
        return;
    }

    switch(GET_CAN_ID_CAN_FRAME_TYPE(can_id))
    {
        default:
            break;
        case TYPE_DATAGRAM_ONE_FRAME:
        {
            Datagram *datagram = nmranet_datagram_alloc();
            if (datagram == NULL)
            {
                /* no buffer available, let the sender know */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_BUFFER_UNAVAILABLE);
                break;
            }
            memcpy(datagram->data, data, dlc);
            datagram->from.id = src.id;
            datagram->from.alias = src.alias;
            datagram->size = dlc;
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst_id, datagram);
            break;
        }
        case TYPE_DATAGRAM_FIRST_FRAME:
        {
            Datagram *datagram = nmranet_datagram_alloc();
            if (datagram == NULL)
            {
                /* no buffer available, let the sender know */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_BUFFER_UNAVAILABLE);
                break;
            }

            struct datagram_node  lookup_node;
            struct datagram_node *datagram_node;
            
            lookup_node.alias = ((uint32_t)dst_alias << 12) + src.alias;
            
            datagram_node = RB_FIND(datagram_tree, &can_if->datagramHead, &lookup_node);
            if (datagram_node)
            {
                /* we are already receiving a datagram, this is an error */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_OUT_OF_ORDER);
                RB_REMOVE(datagram_tree, &can_if->datagramHead, datagram_node);
                free(datagram_node);
                nmranet_datagram_release(datagram);
                break;
            }
            datagram_node = malloc(sizeof(struct datagram_node));
            datagram_node->alias = lookup_node.alias;
            datagram_node->datagram = datagram;
            RB_INSERT(datagram_tree, &can_if->datagramHead, datagram_node);

            memcpy(datagram->data, data, dlc);
            datagram->from.id = src.id;
            datagram->from.alias = src.alias;
            datagram->size = dlc;
            break;
        }
        case TYPE_DATAGRAM_MIDDLE_FRAME:
        {
            struct datagram_node  lookup_node;
            struct datagram_node *datagram_node;
            
            lookup_node.alias = ((uint32_t)dst_alias << 12) + src.alias;
            
            datagram_node = RB_FIND(datagram_tree, &can_if->datagramHead, &lookup_node);
            if (datagram_node == NULL)
            {
                /* we have no record of this datagram, this is an error */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_OUT_OF_ORDER);
                break;
            }
            Datagram *datagram = datagram_node->datagram;
            if ((datagram->size + dlc) > DATAGRAM_MAX_SIZE)
            {
                /* we have too much data for a datagram, this is an error */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_OUT_OF_ORDER);
                RB_REMOVE(datagram_tree, &can_if->datagramHead, datagram_node);
                free(datagram_node);
                nmranet_datagram_release(datagram);
                break;
            }
            memcpy(datagram->data + datagram->size, data, dlc);
            datagram->size += dlc;
            break;
        }
        case TYPE_DATAGRAM_FINAL_FRAME:
        {
            struct datagram_node  lookup_node;
            struct datagram_node *datagram_node;
            
            lookup_node.alias = ((uint32_t)dst_alias << 12) + src.alias;
            
            datagram_node = RB_FIND(datagram_tree, &can_if->datagramHead, &lookup_node);
            if (datagram_node == NULL)
            {
                /* we have no record of this datagram, this is an error */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_OUT_OF_ORDER);
                break;
            }
            Datagram *datagram = datagram_node->datagram;
            if ((datagram->size + dlc) > DATAGRAM_MAX_SIZE)
            {
                /* we have too much data for a datagram, this is an error */
                datagram_rejected(can_if, dst_alias, src.alias, DATAGRAM_OUT_OF_ORDER);
                RB_REMOVE(datagram_tree, &can_if->datagramHead, datagram_node);
                free(datagram_node);
                nmranet_datagram_release(datagram);
                break;
            }
            memcpy(datagram->data + datagram->size, data, dlc);
            datagram->size += dlc;
            RB_REMOVE(datagram_tree, &can_if->datagramHead, datagram_node);
            free(datagram_node);
            nmranet_if_rx_data(&can_if->nmranetIF, nmranet_mti(can_id), src, dst_id, datagram);
            break;
        }
    }
}

/** Test to see if the alias is in conflict with an alias we are using.
 * @param can_if interface to look for conflict on
 * @param alias alias to look for conflict with
 * @param release we should release the alias if we have it reserved
 * @return 0 if no conflict found, else 1
 */
static int alias_conflict(NMRAnetCanIF *can_if, node_alias_t alias, int release)
{
    int conflict = 0;
    
    os_mutex_lock(&can_if->aliasMutex);
    node_id_t id = nmranet_alias_lookup_id(can_if->upstreamCache, alias);
    if (id)
    {
        /* we have this alias reserved, prevent a collision */
        if (release)
        {
            struct can_frame frame;
            /* tell everyone we to un-map our alias */
            CAN_CONTROL_FRAME_INIT(frame, alias, AMR_FRAME, 0);
            SET_CAN_FRAME_EFF(frame);
            CLR_CAN_FRAME_RTR(frame);
            CLR_CAN_FRAME_ERR(frame);
            frame.can_dlc = 6;
            frame.data[0] = (id >> 40) & 0xff;
            frame.data[1] = (id >> 32) & 0xff;
            frame.data[2] = (id >> 24) & 0xff;
            frame.data[3] = (id >> 16) & 0xff;
            frame.data[4] = (id >>  8) & 0xff;
            frame.data[5] = (id >>  0) & 0xff;
            int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
            if (result != sizeof(struct can_frame))
            {
                abort();
            }
            nmranet_alias_remove(can_if->upstreamCache, alias);
        }
        conflict = 1;
    }
    else
    {
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
        {
            switch (can_if->pool[i].status)
            {
                case ALIAS_FREE:
                    /* fall through */
                case ALIAS_CONFLICT:
                    /* keep looking */
                    continue;
                case ALIAS_UNDER_TEST:
                    if (can_if->pool[i].alias == alias)
                    {
                        can_if->pool[i].status = ALIAS_CONFLICT;
                        conflict = 1;
                    }
                    break;
                case ALIAS_RESERVED:
                    if (can_if->pool[i].alias == alias)
                    {
                        if (release)
                        {
                            can_if->pool[i].status = ALIAS_CONFLICT;
                            node_alias_t new_alias;
                            do
                            {
                                new_alias = nmranet_alias_generate(can_if->upstreamCache);
                            }
                            while (nmranet_alias_lookup(can_if->aliasCache, new_alias) != 0);
                            claim_alias(can_if, 0, new_alias);
                        }
                        conflict = 1;
                    }
                    break;
            }
            break;
        }
    }
    os_mutex_unlock(&can_if->aliasMutex);
    
    return conflict;
}

/** Decode Check ID CAN control frame.
 * @param can_if interface CID received on
 * @param ccr CAN control frame
 */
static void ccr_cid_frame(NMRAnetCanIF *can_if, uint32_t ccr)
{
    uint16_t alias = GET_CAN_CONTROL_FRAME_SOURCE(ccr);
    
    if (alias_conflict(can_if, alias, 0))
    {
        /* remind everyone we own this alias with a Reserve ID frame */
        struct can_frame frame;
        CAN_CONTROL_FRAME_INIT(frame, alias, RID_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
        if (result != (sizeof(struct can_frame)))
        {
            abort();
        }
    }
}

/** Decode Reserve ID CAN control frame.
 * @param ccr CAN control frame
 */
static void ccr_rid_frame(uint32_t ccr)
{
    /* If we got this far we have nothing to do.  If there was a conflict,
     * it should have been caught by now.
     */
}

static void send_amd_frame(void *data, node_id_t id, node_alias_t alias)
{
    NMRAnetCanIF *can_if = (NMRAnetCanIF*)data;

    if (alias && id)
    {
        /* Tell the segment who maps to this alias */
        struct can_frame frame;
        CAN_CONTROL_FRAME_INIT(frame, alias, AMD_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        frame.can_dlc = 6;
        frame.data[0] = (id >> 40) & 0xff;
        frame.data[1] = (id >> 32) & 0xff;
        frame.data[2] = (id >> 24) & 0xff;
        frame.data[3] = (id >> 16) & 0xff;
        frame.data[4] = (id >>  8) & 0xff;
        frame.data[5] = (id >>  0) & 0xff;
        int result = (*can_if->write)(can_if->fd, &frame, sizeof(struct can_frame));
        if (result != sizeof(struct can_frame))
        {
            abort();
        }
    }
}

/** Decode Alias Map Enquiry CAN control frame.
 * @param can_if interface AME received on
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
static void ccr_ame_frame(NMRAnetCanIF *can_if, uint32_t ccr, uint8_t data[])
{
    if (data)
    {
        node_id_t node_id;

        node_id = data[5];
        node_id |= (node_id_t)data[4] << 8;
        node_id |= (node_id_t)data[3] << 16;
        node_id |= (node_id_t)data[2] << 24;
        node_id |= (node_id_t)data[1] << 32;
        node_id |= (node_id_t)data[0] << 40;
        
        os_mutex_lock(&can_if->aliasMutex);
        node_alias_t alias = nmranet_alias_lookup(can_if->upstreamCache, node_id);
        if (alias)
        {
            send_amd_frame(can_if, node_id, alias);
        }
    }
    else
    {
        os_mutex_lock(&can_if->aliasMutex);
        nmranet_alias_for_each(can_if->upstreamCache, send_amd_frame, can_if);
    }
    os_mutex_unlock(&can_if->aliasMutex);
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
    
    os_mutex_lock(&can_if->aliasMutex);
    if (write_buffer_in_use(can_if))
    {
        if (node_id == write_buffer_dst_id(can_if))
        {
            can_if->writeBuffer.dst.alias = GET_CAN_CONTROL_FRAME_SOURCE(ccr);
            can_if->nmranetIF.write(&can_if->nmranetIF,
                                    can_if->writeBuffer.mti,
                                    can_if->writeBuffer.src,
                                    can_if->writeBuffer.dst,
                                    can_if->writeBuffer.data);
            /* we obviously use this alias, so let's cache it for later use */
            nmranet_alias_add(can_if->aliasCache, node_id, can_if->writeBuffer.dst.alias);
            write_buffer_release(can_if);
        }
    }
    os_mutex_unlock(&can_if->aliasMutex);
}

/** Decode Alias Map Reset CAN control frame.
 * @param can_if interface AMR received on
 * @param ccr CAN control frame
 * @param data frame data representing the full 48-bit Node ID
 */
static void ccr_amr_frame(NMRAnetCanIF *can_if, uint32_t ccr, uint8_t data[])
{
    os_mutex_lock(&can_if->aliasMutex);
    nmranet_alias_remove(can_if->aliasCache, GET_CAN_CONTROL_FRAME_SOURCE(ccr));
    os_mutex_unlock(&can_if->aliasMutex);
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
        CLR_CAN_FRAME_ERR(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_EFF(frame);

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
                    ccr_cid_frame(can_if, frame.can_id);
                    /* we are done decoding, let's grab the next frame */
                    continue;
                case 0x0:
                    switch (GET_CAN_CONTROL_FRAME_FIELD(frame.can_id))
                    {
                        default:
                            /* unknown field, let's grab the next frame */
                            continue;
                        case RID_FRAME:
                            ccr_rid_frame(frame.can_id);
                            break;
                        case AMD_FRAME:
                            ccr_amd_frame(can_if, frame.can_id, frame.data);
                            break;
                        case AME_FRAME:
                            ccr_ame_frame(can_if, frame.can_id, (frame.can_dlc == 0) ? NULL : frame.data);
                            break;
                        case AMR_FRAME:
                            ccr_amr_frame(can_if, frame.can_id, frame.data);
                            break;
                    } /* switch (GET_CAN_CONTROL_FRAME_FIELD(frame.can_id)) */
                    alias_conflict(can_if, GET_CAN_CONTROL_FRAME_SOURCE(frame.can_id), 1);
                    break;
            } /* switch (GET_CAN_CONTROL_FRAME_SEQUENCE(frame.can_id)) */
        } /* if (GET_CAN_ID_FRAME_TYPE(frame.can_id) == 0) */
        else
        {
            if (alias_conflict(can_if, GET_CAN_ID_SOURCE(frame.can_id), 1))
            {
                /* there was a conflict in the alias mappings */
                continue;
            }
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
    int fd = open(device, O_RDWR);
    if (fd < 0)
    {
        return NULL;
    }
    NMRAnetCanIF *can_if = malloc(sizeof(NMRAnetCanIF));

    os_thread_t thread_handle;
    
    can_if->nmranetIF.write = can_write;
    can_if->nmranetIF.lookup_id = lookup_id;
    can_if->read = if_read;
    can_if->write = if_write;
    can_if->fd = fd;
    can_if->id = node_id;
    can_if->pool = malloc(sizeof(AliasMetadata)*ALIAS_POOL_SIZE);
    can_if->writeBuffer.mti = 0;
    can_if->writeBuffer.timer = os_timer_create(write_buffer_timeout, can_if, NULL);
    can_if->lookup_id.id = 0;
    can_if->lookup_id.alias = 0;
    can_if->lookup_id.timer = os_timer_create(lookup_id_timeout, can_if, NULL);
    os_mutex_init(&can_if->lookup_id.mutex);
    os_sem_init(&can_if->lookup_id.sem, 0);
    RB_INIT(&can_if->datagramHead);

    for (unsigned int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        can_if->pool[i].status = ALIAS_FREE;
        can_if->pool[i].timer = os_timer_create(claim_alias_timeout, can_if, &can_if->pool[i]);
    }
    can_if->aliasCache = nmranet_alias_cache_create(node_id, DOWNSTREAM_ALIAS_CACHE_SIZE);
    can_if->upstreamCache = nmranet_alias_cache_create(node_id, UPSTREAM_ALIAS_CACHE_SIZE);
    os_mutex_init(&can_if->aliasMutex);

    os_mutex_lock(&can_if->aliasMutex);
    /* prime the alias pool */
    for (int i = 0; i < ALIAS_POOL_SIZE; i++)
    {
        /* initiate a claim on a new unique alias */
        claim_alias(can_if, 0, nmranet_alias_generate(can_if->upstreamCache));
    }
    os_mutex_unlock(&can_if->aliasMutex);

    /* start the thread that will process received packets */
    os_thread_create(&thread_handle, device, 0, 1024, read_thread, &can_if->nmranetIF);

    return &can_if->nmranetIF;
}

