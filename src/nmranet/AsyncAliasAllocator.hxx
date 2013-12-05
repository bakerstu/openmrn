/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file AsyncAliasAllocator.hxx
 *
 * Asynchronous implementation of NMRAnet alias reservation algorithms.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#ifndef _NMRANET_ASYNC_ALIAS_ALLOCATOR_HXX_
#define _NMRANET_ASYNC_ALIAS_ALLOCATOR_HXX_

#include "nmranet/AsyncIfCan.hxx"
#include "nmranet/NMRAnetIf.hxx"

namespace NMRAnet
{

/** Information we know locally about an NMRAnet CAN alias. */
struct AliasInfo : public QueueMember
{
    AliasInfo() : alias(0), state(STATE_EMPTY)
    {
    }

    void Reset() {
        alias = 0;
        state = STATE_EMPTY;
    }

    /** The current alias. This is 0 if the alias needs to be generated. */
    unsigned alias : 12;
    unsigned state : 4;

    enum State {
        STATE_EMPTY = 0,
        STATE_CHECKING,
        STATE_RESERVED,
        STATE_ASSIGNED,
        STATE_CONFLICT
    };
};

/** This control flow is responsible for reserving node ID aliases.



 */
class AsyncAliasAllocator : public ControlFlow, private IncomingFrameHandler
{
public:
    /**
       Constructs a new AliasAllotator flow.

       @param if_id is a 48-bit NMRAnet NodeID. This node id will be used for
       reserving all aliases. This NodeID must be unique to the hardware on the
       bus; it will typically be the same as the NodeID of some virtual node
       that the current application will be creating.

       @param if_can is the interface to which this alias allocator should talk
       to.

       @param executor is the executor on which the allocation flow will
       run. Typically the same executor as the interface.
     */
    AsyncAliasAllocator(NodeID if_id, AsyncIfCan* if_can);

    virtual ~AsyncAliasAllocator();

    TypedAllocator<AliasInfo>* empty_aliases()
    {
        return &empty_alias_allocator_;
    }

    TypedAllocator<AliasInfo>* reserved_aliases()
    {
        return &reserved_alias_allocator_;
    }

private:
    //! Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* message, Notifiable* done);

    ControlFlowAction HandleGetMoreWork();
    ControlFlowAction HandleWorkArrived();
    ControlFlowAction HandleInitAliasCheck();
    ControlFlowAction HandleSendCidFrames();
    ControlFlowAction HandleWaitDone();
    ControlFlowAction HandleSendRidFrame();

    ControlFlowAction HandleAliasConflict();

    //! Generates the next alias to check in the seed_ variable.
    void NextSeed();

    friend class AsyncAliasAllocatorTest;
    friend class AsyncIfTest;

    //! 48-bit nodeID that we will use for alias reservations.
    NodeID if_id_;
    //! Physical interface for sending packets and assigning handlers to
    //! received packets.
    AsyncIfCan* if_can_;

    /** This allocator contains the info structures for aliases that we need to
        reserve. The AliasAllocatorFlow will automatically wake up if an alias
        info is posted to this allocator, and try to reserve that alias. */
    TypedAllocator<AliasInfo> empty_alias_allocator_;

    /** Freelist of reserved aliases that can be used by virtual nodes. The
        AliasAllocatorFlow will post successfully reserved aliases to this
        allocator. */
    TypedAllocator<AliasInfo> reserved_alias_allocator_;

    //! The alias currently being checked.
    AliasInfo* pending_alias_;
    //! Which CID frame are we trying to send out. Valid values: 7..4
    unsigned cid_frame_sequence_ : 3;
    //! Set to 1 if an incoming frame signals an alias conflict.
    unsigned conflict_detected_ : 1;

    //! Seed for generating random-looking alias numbers.
    unsigned seed_ : 12;

    //! Timer needed for sleeping the control flow.
    SleepData sleep_helper_;
};
}

#endif // _NMRANET_ASYNC_ALIAS_ALLOCATOR_HXX_
