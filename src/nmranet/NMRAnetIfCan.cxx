/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file NMRAnetIfCan.cxx
 * This file provides an NMRAnet interface specific to CAN.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#include <unistd.h>

#include "nmranet/NMRAnetIfCan.hxx"

#include "core/nmranet_datagram_private.h"

namespace NMRAnet
{

/** Get the NMRAnet MTI from a can identifier.
 * @param can_id CAN identifider
 * @return NMRAnet MTI
 */
If::MTI IfCan::nmranet_mti(uint32_t can_id)
{
    switch (get_can_frame_type(can_id))
    {
        default:
            return MTI_NONE;
        case GLOBAL_ADDRESSED:
            return (MTI)get_mti(can_id);
        case DATAGRAM_ONE_FRAME:
            /* fall through */
        case DATAGRAM_FIRST_FRAME:
            /* fall through */
        case DATAGRAM_MIDDLE_FRAME:
            /* fall through */
        case DATAGRAM_FINAL_FRAME:
            return MTI_DATAGRAM;
        case STREAM_DATA:
            return MTI_STREAM_DATA;
    }
}

/** Get the CAN identifier from an NMRAnet mti and source alias.
 * @param mti NMRAnet MTI
 * @param src Source node alias
 * @return CAN identifier
 */
uint32_t IfCan::can_identifier(MTI mti, NodeAlias src)
{
    return ((src << SRC_SHIFT           ) & SRC_MASK) +
           ((mti << MTI_SHIFT           ) & MTI_MASK) +
           ((1   << CAN_FRAME_TYPE_SHIFT)           ) +
           ((1   << FRAME_TYPE_SHIFT    )           ) +
           ((1   << PRIORITY_SHIFT      )           );
}

/** Put out a claim on an alias.  This method must always be called with the
 * mutex locked.
 * @param node_id node id that is making the claim
 * @param alias alias that node is claiming
 * @param entry entry within the pool to use for claim.
 */
void IfCan::claim_alias(NodeID node_id, NodeAlias alias, Pool *entry)
{
    HASSERT(entry->status == FREE);
    entry->alias = alias;
    entry->status = UNDER_TEST;
    if (node_id == 0)
    {
        /* this is an anonymous request, use the interface Node ID */
        node_id = nodeID;
    }
    struct can_frame frame[4];
    
    control_init(frame[0], alias, (node_id >> 36) & 0xfff, 7);
    control_init(frame[0], alias, (node_id >> 24) & 0xfff, 6);
    control_init(frame[0], alias, (node_id >> 12) & 0xfff, 5);
    control_init(frame[0], alias, (node_id >>  0) & 0xfff, 4);
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

    int result = (*write)(fd, &frame, sizeof(struct can_frame) * 4);
    HASSERT(result == (sizeof(struct can_frame) * 4));

    /* wait 200+ msec */
    entry->timer.start(MSEC_TO_NSEC(200));
}

/** This is the timeout for claiming an alias.  At this point, the alias will
 * either be claimed as a downstream node, or we can start using it.
 * @param data1 a @ref NMRAnetCanIF typecast to a void*
 * @param data2 a @ref alias_node typecast to a void*
 * @return OS_TIMER_NONE
 */
long long IfCan::Pool::timeout(void *data1, void *data2)
{
    IfCan *if_can = (IfCan*)data1;
    Pool  *entry  = (Pool*)data2;

    if_can->mutex.lock();
    if (entry->status != UNDER_TEST)
    {
        /* try again with a new alias */
        /** @todo should we do this in a background thread? */
        entry->status = FREE;
        NodeAlias new_alias;
        do
        {
            new_alias = if_can->upstreamCache.generate();
        }
        while (if_can->downstreamCache.lookup(new_alias) != 0);
        
        /* note that this call will restart the timer.  Even though we don't
         * restart the timer on return from timeout, it will have already been
         * restarted, so we are good.
         */
        if_can->claim_alias(0, new_alias, entry);
    }
    else
    {
        entry->status = RESERVED;
        /* Reserve the ID */
        struct can_frame frame;
        control_init(frame, entry->alias, RID_FRAME, 0);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);

        int result = (*if_can->write)(if_can->fd, &frame, sizeof(struct can_frame));
        HASSERT(result == (sizeof(struct can_frame)));
    }
    if_can->mutex.unlock();

    /* do not restart the timer */
    return OS_TIMER_NONE;
}

/** Setup the relationship between an alias and a downstream node.  This method
 * must always be called with the mutex locked.
 * @param node_id Node ID
 * @return assigned alias
 */
NodeAlias IfCan::upstream_alias_setup(NodeID node_id)
{
    for ( ; /* forever */ ; )
    {
        for (unsigned int i = 0; i < ALIAS_POOL_SIZE; ++i)
        {
            if (pool[i].status == RESERVED)
            {
                NodeAlias alias = pool[i].alias;
                struct can_frame frame;

                /* Tell the segment who maps to this alias */
                control_init(frame, alias, AMD_FRAME, 0);
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

                int result = (*write)(fd, &frame, sizeof(struct can_frame));
                HASSERT (result == sizeof(struct can_frame));
                
                /** @todo should we do this in a background thread? */
                pool[i].status = FREE;
                NodeAlias new_alias;
                do
                {
                    new_alias = upstreamCache.generate();
                }
                while (downstreamCache.lookup(new_alias) != 0);
                claim_alias(0, new_alias, pool + i);
                upstreamCache.add(node_id, alias);

                return alias;
            }
        }
        /* the pool is empty, wait and try again */
        mutex.unlock();
        sleep(1);
        mutex.lock();
    }
}

/** Write a message onto the CAN bus.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int IfCan::if_write(MTI mti, NodeID src, NodeHandle dst, Buffer *data)
{
    mutex.lock();
    NodeAlias alias = upstreamCache.lookup(src);
    if (alias == 0)
    {
        /* we have never seen this node before, let's claim an alias for it */
        alias = upstream_alias_setup(src);
    }

    HASSERT(alias != 0);

    if (dst.alias != 0 || dst.id != 0)
    {
        /* we have an addressed message */
        if (dst.alias == 0)
        {
            /* look for a downstream match */
            dst.alias = downstreamCache.lookup(dst.id);
        }
        if (dst.alias)
        {
            HASSERT(data != NULL);
            Buffer *buffer = data;
            int index = 0;
            size_t len;
            /* check to see if we have a stream or datagram */
            if (get_mti_datagram(mti))
            {
                //const Datagram *datagram = data;
                /* datagrams and streams are special because the CAN ID
                 * also contains the destination alias.  These may also span
                 * multiple CAN frames.
                 */
                if (mti == MTI_STREAM_DATA)
                {
                    len = data->size() - data->available();
                }
                else
                {
                    len = 0;//datagram->size;
                    //buffer = datagram->data;
                }
                for (int i = 0; len > 0; ++i)
                {
                    CanFrameType type;
                    if (mti == MTI_STREAM_DATA)
                    {
                        type = STREAM_DATA;
                    }
                    else if (i == 0 && len <= 8)
                    {
                        type = DATAGRAM_ONE_FRAME;
                    }
                    else if (i == 0)
                    {
                        type = DATAGRAM_FIRST_FRAME;
                    }
                    else if (len <= 8)
                    {
                        type = DATAGRAM_FINAL_FRAME;
                    }
                    else
                    {
                        type = DATAGRAM_MIDDLE_FRAME;
                    }
                    struct can_frame frame;
                    set_fields(frame.can_id, alias, (MTI)dst.alias, type, NMRANET_MSG, NORMAL_PRIORITY);
                    SET_CAN_FRAME_EFF(frame);
                    CLR_CAN_FRAME_RTR(frame);
                    CLR_CAN_FRAME_ERR(frame);
                    size_t seg_size = len < 8 ? len : 8;
                    memcpy(frame.data, (char*)buffer->start() + i, seg_size);
                    frame.can_dlc = seg_size;

                    int result = (*write)(fd, &frame, sizeof(struct can_frame));
                    HASSERT (result == sizeof(struct can_frame));
                    
                    len-= seg_size;
                    index += seg_size;
                }
            }
            else
            {
                if (data)
                {
                    len = data->size() - data->available();
                }
                else
                {
                    len = 0;
                }
                /* typically, only the simple node ident info reply will require
                 * the sending of more than one CAN frame, but who knows what
                 * the future might hold?
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
                    memcpy(&frame.data[2], (char*)buffer->start() + index, seg_size);
                    frame.can_dlc = 2 + seg_size;
                    int result = (*write)(fd, &frame, sizeof(struct can_frame));
                    HASSERT(result == sizeof(struct can_frame));

                    len -= seg_size;
                    index += seg_size;
                    
                } while (len > 0);

                /* release the buffer if we can,
                 * we don't release datagrams and streams
                 */
                if (data)
                {
                    data->free();
                }
            }
        }
        else
        {
            if (get_mti_address(mti))
            {
                /* this is an addressed message, so we need to buffer while
                 * we determine what alias this node ID belongs to.  This
                 * should happen infrequently because we should have the most
                 * often addressed aliases cached.
                 */
                while (writeBuffer.in_use())
                {
                    mutex.unlock();
                    usleep(300);
                    mutex.lock();
                }
                writeBuffer.setup(mti, src, dst, data);

                /* Alias Map Enquiry */
                struct can_frame frame;
                control_init(frame, alias, AME_FRAME, 0);
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
                int result = (*write)(fd, &frame, sizeof(struct can_frame));
                HASSERT(result == (sizeof(struct can_frame)));
            }
        }
    }
    else
    {
        /* we have an unaddressed message */
        struct can_frame frame;
        frame.can_id = can_identifier(mti, alias);
        SET_CAN_FRAME_EFF(frame);
        CLR_CAN_FRAME_RTR(frame);
        CLR_CAN_FRAME_ERR(frame);
        if (data != NULL)
        {
            frame.can_dlc = data->size() - data->available();
            memcpy(frame.data, data->start(), frame.can_dlc);
        }
        else
        {
            frame.can_dlc = 0;
        }
        int result = (write)(fd, &frame, sizeof(struct can_frame));
        HASSERT(result == (sizeof(struct can_frame)));
    }        
    mutex.unlock();

    return 0;
}

}; /* namespace NMRAnet */
