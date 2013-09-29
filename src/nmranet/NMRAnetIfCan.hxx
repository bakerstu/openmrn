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
 * \file NMRAnetIfCan.hxx
 * This file provides an NMRAnet interface specific to CAN.
 *
 * @author Stuart W. Baker
 * @date 18 September 2013
 */

#ifndef _NMRAnetIfCan_hxx_
#define _NMRAnetIfCan_hxx_

#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetAliasCache.hxx"

namespace NMRAnet
{

/** The generic interface for NMRAnet network interfaces
 */
class IfCan : public If
{
public:
    /** Constructor.
     */
    IfCan(NodeID node_id)
        : If(node_id)
    {
    }

protected:
    /** Default destructor.
     */
    ~IfCan();

private:
    /** CAN ID bit fields for most CAN frames. */
    enum ID
    {
        SRC_MASK            = 0x00000fff, /**< mask for source field of CAN ID */
        MTI_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        DST_MASK            = 0x00fff000, /**< mask for MTI field of CAN ID */
        CAN_FRAME_TYPE_MASK = 0x07000000, /**< mask for can frame type field of CAN ID */
        FRAME_TYPE_MASK     = 0x08000000, /**< mask for frame type field of CAN ID */
        PRIORITY_MASK       = 0x10000000, /**< mask for priority field of CAN ID */
        PADDING_MASK        = 0xe0000000, /**< mask for padding field of CAN ID */

        SRC_SHIFT            =  0, /**< shift for source field of CAN ID */
        MTI_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        DST_SHIFT            = 12, /**< shift for MTI field of CAN ID */
        CAN_FRAME_TYPE_SHIFT = 24, /**< shift for can frame type field of CAN ID */
        FRAME_TYPE_SHIFT     = 27, /**< shift for frame type field of CAN ID */
        PRIORITY_SHIFT       = 28, /**< shift for priority field of CAN ID */
        PADDING_SHIFT        = 29  /**< shift for padding field of CAN ID */
    };

    typedef uint16_t CanMTI;

    /** CAN Frame Types. */
    enum CanFrameType
    {
        GLOBAL_ADDRESSED      = 1, /**< most CAN frame types fall in this category */
        DATAGRAM_ONE_FRAME    = 2, /**< a single frame datagram */
        DATAGRAM_FIRST_FRAME  = 3, /**< first frame of multi-frame datagram */
        DATAGRAM_MIDDLE_FRAME = 4, /**< middle frame of multi-frame datagram */
        DATAGRAM_FINAL_FRAME  = 5, /**< last frame of multi-frame datagram */
        STREAM_DATA           = 7, /**< stream data frame */
    };
    
    /** Frame Types, Control or normal NMRAnet message. */
    enum FrameType
    {
        CONTROL_MSG = 0, /**< CAN control frame message */
        NMRANET_MSG = 1  /**< normal NMRAnet message */
    };
    
    /** Highest order priority of a CAN message.  Most messages fall into
     * the NORMAL_PRIORITY category.
     */
    enum Priority
    {
        HIGH_PRIORITY   = 0, /**< high priority CAN message */
        NORMAL_PRIORITY = 1  /**< normal priority CAN message */
    };
    
    enum ControlFrameType
    {
        RID_FRAME = 0x0700, /**< Reserve ID Frame */
        AMD_FRAME = 0x0701, /**< Alias Map Definition frame */
        AME_FRAME = 0x0702, /**< Alias Mapping Inquery */
        AMR_FRAME = 0x0703  /**< Alias Map Reset */
    };
    
    /** Get the source field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return source field
     */
    NodeAlias get_src(uint32_t can_id)
    {
        return (can_id & SRC_MASK) >> SRC_SHIFT;
    }

    /** Get the MTI field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return MTI field value
     */
    CanMTI get_mti(uint32_t can_id)
    {
        return ((can_id & MTI_MASK) >> MTI_SHIFT);
    }

    /** Get the destination field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return destination field value
     */
    NodeAlias get_dst(uint32_t can_id)
    {
        return (can_id & DST_MASK) >> DST_SHIFT;
    }

    /** Get the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return CAN frame type field value
     */
    CanFrameType get_can_frame_type(uint32_t can_id)
    {
        return (CanFrameType)((can_id & CAN_FRAME_TYPE_MASK) >> CAN_FRAME_TYPE_SHIFT);
    }

    /** Get the frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return frame type field value
     */
    FrameType get_frame_type(uint32_t can_id)
    {
        return (FrameType)((can_id & FRAME_TYPE_MASK) >> FRAME_TYPE_SHIFT);
    }

    /** Get the priority field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return priority field value
     */
    Priority get_priority(uint32_t can_id)
    {
        return (Priority)((can_id & PRIORITY_MASK) >> PRIORITY_SHIFT);
    }

    /** Set the MTI field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param mti MTI field value
     */
    void set_mti(uint32_t &can_id, CanMTI mti)
    {
        can_id &= ~MTI_MASK;
        can_id |= mti << MTI_SHIFT;
    }

    /** Set the destination field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param dst destination field value
     */
    void set_dst(uint32_t &can_id, NodeAlias dst)
    {
        can_id &= ~DST_MASK;
        can_id |= dst << DST_SHIFT;
    }

    /** Set the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type CAN frame type field value
     */
    void set_can_frame_type(uint32_t can_id, CanFrameType type)
    {
        can_id &= ~CAN_FRAME_TYPE_MASK;
        can_id |= type << CAN_FRAME_TYPE_SHIFT;
    }

    /** Set the frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type frame type field value
     */
    void set_frame_type(uint32_t can_id, FrameType type)
    {
        can_id &= ~FRAME_TYPE_MASK;
        can_id |= type << FRAME_TYPE_SHIFT;
    }

    /** Set the priority field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param priority pryority field value
     */
    void SET_CAN_ID_PRIORITY(uint32_t can_id, Priority priority)
    {
        can_id &= ~PRIORITY_MASK;
        can_id |= priority << PRIORITY_SHIFT;
    }

    /** Get the NMRAnet MTI from a can identifier.
     * @param can_id CAN identifider
     * @return NMRAnet MTI
     */
    MTI nmranet_mti(uint32_t can_id);

    /** Get the CAN identifier from an NMRAnet mti and source alias.
     * @param mti NMRAnet MTI
     * @param src Source node alias
     * @return CAN identifier
     */
    uint32_t can_identifier(MTI mti, NodeAlias src);

    /** Write a message onto the CAN bus.
     * @param mti Message Type Indicator
     * @param src source node ID, 0 if unavailable
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int write(uint16_t mti, NodeID src, NodeHandle dst, const void *data);

    DISALLOW_COPY_AND_ASSIGN(IfCan);
};

}; /* namespace NMRAnet */

#endif /* _NMRAnetIfCan_hxx_ */

