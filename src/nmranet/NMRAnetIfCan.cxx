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

#include "nmranet/NMRAnetIfCan.hxx"

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

/** Write a message onto the CAN bus.
 * @param mti Message Type Indicator
 * @param src source node ID, 0 if unavailable
 * @param dst destination node ID, 0 if unavailable
 * @param data NMRAnet packet data
 * @return 0 upon success
 */
int IfCan::write(uint16_t mti, NodeID src, NodeHandle dst, const void *data)
{
#if 0
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
                    int result = (*can_if->write)(can_if->write_fd, &frame, sizeof(struct can_frame));
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
                    int result = (*can_if->write)(can_if->write_fd, &frame, sizeof(struct can_frame));
                    if (result < 0)
                    {
                        abort();
                    }

                    len -= seg_size;
                    buffer += seg_size;
                    
                } while (len > 0);

                /* release the buffer if we can,
                 * we don't release datagrams and streams
                 */
                if (data)
                {
                    nmranet_buffer_free(data);
                }
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
                int result = (*can_if->write)(can_if->write_fd, &frame, sizeof(struct can_frame));
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
        int result = (*can_if->write)(can_if->write_fd, &frame, sizeof(struct can_frame));
        if (result < 0)
        {
            abort();
        }
    }        
#endif
    return 0;
}

}; /* namespace NMRAnet */
