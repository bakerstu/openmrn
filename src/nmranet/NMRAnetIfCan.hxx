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

#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetAliasCache.hxx"
#include "nmranet_config.h"
#include "nmranet_can.h"

#define WRITE_BUFFER_TIMEOUT 3000000000LL

namespace NMRAnet
{

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
        RESERVED_SHIFT    = 14
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

    /** Set the MTI field value of the CAN ID.
     * @param can_id identifier to act upon, passed by reference
     * @param mti MTI field value
     */
    static void set_mti(uint32_t *can_id, CanMTI mti)
    {
        *can_id &= ~MTI_MASK;
        *can_id |= mti << MTI_SHIFT;
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
        frame.can_id = (src      << CONTROL_SRC_SHIFT     ) +
                       (field    << CONTROL_FIELD_SHIFT   ) +
                       (sequence << CONTROL_SEQUENCE_SHIFT) +
                       ((0)      << CONTROL_TYPE_SHIFT    ) +
                       ((1)      << CONTROL_PRIORITY_SHIFT);
        frame.can_dlc = 0;
    }

    /** Get the destination field of addressed data.
     * @param _address addressed data
     * @return destination field value
     */
    NodeAlias get_addressed_destination(uint16_t address)
    {
        return (address & DESTINATION_MASK) >> DESTINATION_SHIFT;
    }

    /** Write a message onto the CAN bus.
     * @param mti Message Type Indicator
     * @param src source node ID, 0 if unavailable
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int if_write(MTI mti, NodeID src, NodeHandle dst, Buffer *data)
    {
        mutex.lock();
        int result = if_write_locked(mti, src, dst, data);
        mutex.unlock();
        return result;
    }

    /** Write a message onto the CAN bus.  The interface mutex should already
     * be locked.
     * @param mti Message Type Indicator
     * @param src source node ID, 0 if unavailable
     * @param dst destination node ID, 0 if unavailable
     * @param data NMRAnet packet data
     * @return 0 upon success
     */
    int if_write_locked(MTI mti, NodeID src, NodeHandle dst, Buffer *data);

    /** Entry point of thread for reading the data from the interface.
     * @param data pointer to an IfCan instance
     * @return NULL, should never return
     */
    static void *read_thread_entry(void *data)
    {
        IfCan *if_can = (IfCan*)data;
        
        return if_can->read_thread(data);
    }

    /** Thread for reading the data from the interface in proper context.
     * @param data pointer to an IfCan instance
     * @return NULL, should never return
     */
    void *read_thread(void *data);

    /** Setup the relationship between an alias and a downstream node.  This method
     * must always be called with the mutex locked.
     * @param node_id Node ID
     * @return assigned alias
     */
    NodeAlias upstream_alias_setup(NodeID node_id);
    
    /** Decode global or addressed can frame.
     * @param can_id can identifier
     * @param dlc data length code
     * @param data pointer to up to 8 bytes of data
     */
    void global_addressed(uint32_t can_id, uint8_t dlc, uint8_t *data);

    /** Test to see if the alias is in conflict with an alias we are using.
     * @param alias alias to look for conflict with
     * @param release true if we should release the alias if we have it reserved
     * @return false if no conflict found, else true
     */
    bool alias_conflict(NodeAlias alias, bool release);

    /** Put out a claim on an alias.  The alias mutex should be locked during this
     * call.
     * @param node_id node id that is making the claim
     * @param alias alias that node is claiming
     * @param entry entry within the pool to use for claim.
     */
    void claim_alias(NodeID node_id, NodeAlias alias, Pool *entry);

    /** Decode Check ID CAN control frame.
     * @param ccr CAN control frame
     */
    void ccr_cid_frame(uint32_t ccr);

    /** Decode Reserve ID CAN control frame.
     * @param ccr CAN control frame
     */
    void ccr_rid_frame(uint32_t ccr)
    {
        /* someone just reserved this alias.  Kick it out of our cache if we
         * reserved it in the past.  This should never happen, we should catch
         * it before now.
         */
        alias_conflict(get_control_src(ccr), 1);
    }

    /** Decode Alias Map Definition CAN control frame.
     * @param ccr CAN control frame
     * @param data frame data representing the full 48-bit Node ID
     */
    void ccr_amd_frame(uint32_t ccr, uint8_t data[]);

    /** Decode Alias Map Enquiry CAN control frame.
     * @param ccr CAN control frame
     * @param data frame data representing the full 48-bit Node ID
     */
    void ccr_ame_frame(uint32_t ccr, uint8_t data[]);

    /** Send an AMD frame for a given Node ID and Alias pair.
     * @param data context pointer
     * @param id Node ID
     * @param alias Node Alias
     */
    static void send_amd_frame(void *data, NodeID id, NodeAlias alias);

    /** Decode Alias Map Reset CAN control frame.
     * @param ccr CAN control frame
     * @param data frame data representing the full 48-bit Node ID
     */
    void ccr_amr_frame(uint32_t ccr, uint8_t data[]);

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

    /** Structure for buffering an addressed write until we can lookup its alias.
     */
    class WriteBuffer
    {
    public:
        /** Constructor.
         * @param if_can parent instance of IfCan
         */
        WriteBuffer(IfCan *if_can)
            : src(0),
              dst({0,0}),
              data(NULL),
              mti(MTI_NONE),
              timer(timeout, if_can, this)
        {
        }
        
        /** Buffer a rite message on the CAN bus waiting for its alias mapping.
         * @param m Message Type Indicator
         * @param s source node ID, 0 if unavailable
         * @param d destination node ID, 0 if unavailable
         * @param b NMRAnet packet data
         * @return 0 upon success
         */
        void setup(MTI m, NodeID s, NodeHandle d, Buffer *b)
        {
            src = s;
            dst = d;
            data = b;
            mti = m;
            timer.start(WRITE_BUFFER_TIMEOUT);
        }
        
        /** Release the buffer from use.
         */
        void release()
        {
            mti = MTI_NONE;
        }
        
        /** Test if the buffer is in use.
         * @return true if in use, else false
         */
        bool in_use()
        {
            return (mti != 0);
        }

        /** This is the timeout for giving up on an outstanding Node ID to alias
         * mapping request.
         * @param data1 a @ref IfCan* typecast to a void*
         * @param data2 a @ref WriteBuffer* typecast to a void*
         * @return OS_TIMER_NONE
         */
        static long long timeout(void *data1, void *data2)
        {
            IfCan       *if_can = (IfCan*)data1;
            WriteBuffer *me     = (WriteBuffer*)data2;

            /** @todo currently we fail silently, should we throw an error? */
            if_can->mutex.lock();
            me->release();
            if_can->mutex.unlock();
            return OS_TIMER_NONE;
        }

        NodeID src;     /**< source node ID */
        NodeHandle dst; /**< destination node ID and/or alias */
        Buffer *data;   /**< message data */
        MTI mti;        /**< MTI value, 0 if buffer not in use */
        OSTimer timer;  /**< timeout on error */

        /** we need to be friends with IfCan in order to access its mutex */
        friend class IfCan;
    };
    
    /** file descriptor used for reading and writing data to and from physical
     * interface
     */
    int fd;

    /** single buffer to hold write data while we determine the proper alias */
    WriteBuffer writeBuffer;

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

    /** Datagram pool */
    BufferPool datagramPool;

    /** Tree for tracking datagrams that are in flight */
    RBTree <uint32_t, Buffer*> datagramTree;

    /** Nodes that will be statically alicated for datagrams in flight */
    RBTree <uint32_t, Buffer*>::Node *datagramNode;

    DISALLOW_COPY_AND_ASSIGN(IfCan);
};

}; /* namespace NMRAnet */

#endif /* _NMRAnetIfCan_hxx_ */

