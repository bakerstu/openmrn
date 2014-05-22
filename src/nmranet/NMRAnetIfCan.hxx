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

#include <fcntl.h>
#include <new>

#include "nmranet/NMRAnetAliasCache.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet_config.h"
#include "nmranet_can.h"

#define WRITE_BUFFER_TIMEOUT 3000000000LL

namespace NMRAnet
{

class IfCan;


/** The generic interface for NMRAnet network interfaces
 */
class IfCan : public If
{
public:
    /** Constructor.
     * @param node_id node ID of interface
     * @param device description for this instance
     * @param read read method for this interface
     * @param write write method for this interface
     */
    IfCan(NodeID node_id, const char *device,
          ssize_t (*read)(int, void*, size_t),
          ssize_t (*write)(int, const void*, size_t));

    /** Message ID's that we can receive */
    enum MessageId
    {
        READ_FRAME = NMRANET_IF_CAN_BASE,
        CONTROL_FRAME,
        WRITE_FRAME
    };

protected:
    /** Default destructor.
     */
    ~IfCan()
    {
    }

private:

    /** Status value for an alias pool item.
     */
    enum AliasStatus
    {
        UNDER_TEST, /**< this is an alias we are trying to claim */
        RESERVED,   /**< the alias has been reserved for use */
        CONFLICT,   /**< we discovered someone else already is using this alias */
        FREE        /**< the alias is free for another request */
    };
    
    /** Metadata for members of the alias pool.
     */
    class Pool
    {
    public:
        /** Constructor.
         */
        Pool(IfCan *if_can)
            : timer(timeout, if_can, this),
              status(FREE),
              alias(0)
        {
        }
        
        /** This is the timeout for claiming an alias.  At this point, the alias will
         * either be claimed as a downstream node, or we can start using it.
         * @param data1 a @ref IfCan instance typecast to a void*
         * @param data2 a @ref Pool instance typecast to a void*
         * @return OS_TIMER_NONE
         */
        static long long timeout(void *data1, void *data2);
        
        OSTimer timer;      /**< timer used for establishing the connection */
        AliasStatus status; /**< status of node */
        NodeAlias alias;    /**< alias */
    };

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
        PADDING_SHIFT        = 29, /**< shift for padding field of CAN ID */
        
        CONTROL_SRC_MASK      = 0x00000fff, /**< source alias mask */
        CONTROL_FIELD_MASK    = 0x00fff000, /**< control field data mask */
        CONTROL_SEQUENCE_MASK = 0x07000000, /**< frame sequence number mask */
        CONTROL_TYPE_MASK     = 0x08000000, /**< value of '0' means control frame mask */
        CONTROL_PRIORITY_MASK = 0x10000000, /**< priority mask */
        CONTROL_PADDING_MASK  = 0xe0000000, /**< pad out to a full 32-bit word */

        CONTROL_SRC_SHIFT      =  0, /**< source alias shift */
        CONTROL_FIELD_SHIFT    = 12, /**< control field data shift */
        CONTROL_SEQUENCE_SHIFT = 24, /**< frame sequence number shift */
        CONTROL_TYPE_SHIFT     = 27, /**< value of '0' means control frame shift */
        CONTROL_PRIORITY_SHIFT = 28, /**< priority shift */
        CONTROL_PADDING_SHIFT  = 29  /**< pad out to a full 32-bit word */
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
    
    enum ControlField
    {
        RID_FRAME = 0x0700, /**< Reserve ID Frame */
        AMD_FRAME = 0x0701, /**< Alias Map Definition frame */
        AME_FRAME = 0x0702, /**< Alias Mapping Inquery */
        AMR_FRAME = 0x0703  /**< Alias Map Reset */
    };
    
    /** Enumerations having to deal with addressed frame types. */
    enum Addressed
    {
        /** destination alias mask */
        DESTINATION_MASK  = 0x0fff,

        /** 0b00 = only frame, 0b01 = first frame,
         *  0b10 = last frame, 0b11 = middle frame (mask)
         */
        FRAME_MASK        = 0x3000, 

        /** reserved for future use */
        RESERVED_MASK     = 0xc000,

        /** destination alias shift */
        DESTINATION_SHIFT = 0, /**< destination alias shift */

        /** 0b00 = only frame, 0b01 = first frame,
         *  0b10 = last frame, 0b11 = middle frame (shift)
         */
        FRAME_SHIFT       = 12, 

        /** reserved for future use */
        RESERVED_SHIFT    = 14,
        
        /** single frame message */
        FRAME_ONLY = 0x0,

        /** First frame in multi-frame message */
        FRAME_FIRST = 0x1,
         
        /** Last frame in multi-frame message */
        FRAME_LAST = 0x2,
        
        /** Middle frame in multi-frame message */
        FRAME_MIDDLE = 0x3
    };
    
    /** Get the source field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return source field
     */
    static NodeAlias get_src(uint32_t can_id)
    {
        return (can_id & SRC_MASK) >> SRC_SHIFT;
    }

    /** Get the MTI field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return MTI field value
     */
    static CanMTI get_mti(uint32_t can_id)
    {
        return ((can_id & MTI_MASK) >> MTI_SHIFT);
    }

    /** Get the destination field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return destination field value
     */
    static NodeAlias get_dst(uint32_t can_id)
    {
        return (can_id & DST_MASK) >> DST_SHIFT;
    }

    /** Get the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return CAN frame type field value
     */
    static CanFrameType get_can_frame_type(uint32_t can_id)
    {
        return (CanFrameType)((can_id & CAN_FRAME_TYPE_MASK) >> CAN_FRAME_TYPE_SHIFT);
    }

    /** Get the frame type field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return frame type field value
     */
    static FrameType get_frame_type(uint32_t can_id)
    {
        return (FrameType)((can_id & FRAME_TYPE_MASK) >> FRAME_TYPE_SHIFT);
    }

    /** Get the priority field value of the CAN ID.
     * @param can_id identifier to act upon
     * @return priority field value
     */
    static Priority get_priority(uint32_t can_id)
    {
        return (Priority)((can_id & PRIORITY_MASK) >> PRIORITY_SHIFT);
    }

    /** Tests if the incoming frame is a CID frame.
     * @param can_id identifier to act upon
     * @return true for CID frame, false for any other frame.
     */
    static bool is_cid_frame(uint32_t can_id)
    {
        return ((can_id >> CAN_FRAME_TYPE_SHIFT) & 0x14) == 0x14;
    }


    /** Set the MTI field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param mti MTI field value
     */
    static void set_mti(uint32_t *can_id, CanMTI mti)
    {
        *can_id &= ~MTI_MASK;
        *can_id |= mti << MTI_SHIFT;
    }

    /** Set the source field value of the CAN ID.
     * @param can_id identifier to act upon, passed by pointer
     * @param src source field value
     */
    static void set_src(uint32_t *can_id, NodeAlias src)
    {
        *can_id &= ~SRC_MASK;
        *can_id |= src << SRC_SHIFT;
    }

    /** Set the destination field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param dst destination field value
     */
    static void set_dst(uint32_t *can_id, NodeAlias dst)
    {
        *can_id &= ~DST_MASK;
        *can_id |= dst << DST_SHIFT;
    }

    /** Set the CAN frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type CAN frame type field value
     */
    static void set_can_frame_type(uint32_t *can_id, CanFrameType type)
    {
        *can_id &= ~CAN_FRAME_TYPE_MASK;
        *can_id |= type << CAN_FRAME_TYPE_SHIFT;
    }

    /** Set the frame type field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param type frame type field value
     */
    static void set_frame_type(uint32_t *can_id, FrameType type)
    {
        *can_id &= ~FRAME_TYPE_MASK;
        *can_id |= type << FRAME_TYPE_SHIFT;
    }

    /** Set the priority field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param priority pryority field value
     */
    static void set_priority(uint32_t *can_id, Priority priority)
    {
        *can_id &= ~PRIORITY_MASK;
        *can_id |= priority << PRIORITY_SHIFT;
    }

    /** Set all the CAN ID fields.
     * @param can_id identifier to act upon
     * @param source source field value
     * @param mti MTI field value
     * @param can_type CAN frame type field value
     * @param type frame type field value
     * @param priority priority field value
     */
    static void set_fields(uint32_t *can_id, NodeAlias src, MTI mti, CanFrameType can_type, FrameType type, Priority priority)
    {
        *can_id = (src      << SRC_SHIFT           ) +
                  (mti      << MTI_SHIFT           ) +
                  (can_type << CAN_FRAME_TYPE_SHIFT) +
                  (type     << FRAME_TYPE_SHIFT    ) +
                  (priority << PRIORITY_SHIFT      );
    }
    
    /** Get the NMRAnet MTI from a can identifier.
     * @param can_id CAN identifider
     * @return NMRAnet MTI
     */
    static MTI nmranet_mti(uint32_t can_id);

    /** Get the CAN identifier from an NMRAnet mti and source alias.
     * @param mti NMRAnet MTI
     * @param src Source node alias
     * @return CAN identifier
     */
    static uint32_t can_identifier(MTI mti, NodeAlias src);

    /** Get the source field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the source field
     */
    static NodeAlias get_control_src(uint32_t can_id)
    {
        return (can_id & CONTROL_SRC_MASK) >> CONTROL_SRC_SHIFT;
    }

    /** Get the field field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the field field
     */
    static ControlField get_control_field(uint32_t can_id)
    {
        return (ControlField)((can_id & CONTROL_FIELD_MASK) >> CONTROL_FIELD_SHIFT);
    }

    /** Get the sequence field of the a can control frame.
     * @param can_id CAN ID of the control frame
     * @return value of the sequence field
     */
    static unsigned int get_control_sequence(uint32_t can_id)
    {
        return (can_id & CONTROL_SEQUENCE_MASK) >> CONTROL_SEQUENCE_SHIFT;
    }

    /** Initialize a control frame CAN ID and set DLC to 0.
     * @param _frame control frame to initialize
     * @param _source source data
     * @param _field field data
     * @param _sequence sequence data
     */
    static void control_init(struct can_frame &frame, NodeAlias src, uint16_t field, int sequence)
    {
        SET_CAN_FRAME_ID_EFF(frame,
                             (src      << CONTROL_SRC_SHIFT     ) |
                             (field    << CONTROL_FIELD_SHIFT   ) |
                             (sequence << CONTROL_SEQUENCE_SHIFT) |
                             ((0)      << CONTROL_TYPE_SHIFT    ) |
                             ((1)      << CONTROL_PRIORITY_SHIFT));
        frame.can_dlc = 0;
    }

    /** Get the frame field of addressed data.
     * @param _address addressed data
     * @return destination field value
     */
    static unsigned get_addressed_frame(uint16_t address)
    {
        return (address & FRAME_MASK) >> FRAME_SHIFT;
    }

    /** Get the destination field of addressed data.
     * @param _address addressed data
     * @return destination field value
     */
    static NodeAlias get_addressed_destination(uint16_t address)
    {
        return (address & DESTINATION_MASK) >> DESTINATION_SHIFT;
    }


    ssize_t (*read)(int, void*, size_t); /**< read method for device */
    ssize_t (*write)(int, const void*, size_t); /**< write method for device */

    /** Can be used by the application to determine if the link is up or down.
     * @return current link status
     */
    LinkStatus link_status()
    {
        return linkStatus;
    }
    
    /** Transition to link up state.
     */
    void link_up();
    
    /** Transition to link down state.
     */
    void link_down();
    
    // These classes use the private enums for message field parsing.
    friend class AddressedCanMessageWriteFlow;
    friend class AliasConflictHandler;
    friend class AsyncAliasAllocator;
    friend class CanDatagramClient;
    friend class CanDatagramParser;
    friend class CanMessageWriteFlow;
    friend class FrameToAddressedMessageParser;
    friend class FrameToGlobalMessageParser;

    /** Maximum number of multi-frame addressed messages we can track in flight */
    static const size_t MAX_IN_FLIGHT_MULTI_FRAME;

    /** file descriptor used for reading and writing data to and from physical
     * interface
     */
    int fd;

    /** Array of Pool entries for pre-allocated aliases */
    Pool *pool;

    /** Cache of Node ID to Alias mappings for downstream nodes */
    AliasCache downstreamCache;
    
    /** Cache of Node ID to alias mappings for upstream nodes */
    AliasCache upstreamCache;
    
    /** Mutual exclusion for an instance of this class */
    OSMutex mutex;
    
    /** current link status */
    LinkStatus linkStatus;

    /** Short hand for the uint64_t/void* Map type */
    typedef Map <uint64_t, BufferBase*> MultiFrameMap;

    /** Mapping for tracking multi-frame addressed messages that are in flight */
    MultiFrameMap multiFrameMap;
    
    /** number of multi-frame message in flight */
    size_t multiFrameInFlight;

    DISALLOW_COPY_AND_ASSIGN(IfCan);
};

}; /* namespace NMRAnet */

#endif /* _NMRAnetIfCan_hxx_ */
