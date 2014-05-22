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
#include "nmranet/Defs.hxx"
#include "executor/StateFlow.hxx"

namespace NMRAnet
{

/** Counts the number of aliases that were given up because a conflict has
 * arisen during the allocation. */
extern size_t g_alias_test_conflicts;

/** Information we know locally about an NMRAnet CAN alias. */
struct AliasInfo : public QMember
{
    AliasInfo()
        : alias(0)
        , state(STATE_EMPTY)
    {
    }

    void reset()
    {
        alias = 0;
        state = STATE_EMPTY;
    }

    /** The current alias. This is 0 if the alias needs to be generated. */
    unsigned alias : 12;
    unsigned state : 4;

    enum State
    {
        STATE_EMPTY = 0,
        STATE_CHECKING,
        STATE_RESERVED,
        STATE_ASSIGNED,
        STATE_CONFLICT
    };
};

/** This state flow is responsible for reserving node ID aliases.
 *
 * For every incoming Buffer<AliasInfo> it will run through the
 * standard-compliant flow of reserving an alias, and then push the alias into
 * the queue of reserved aliases.
 *
 * Users who need an allocated alias should get it from the queue in
 * reserved_aliases().
 */
class AliasAllocator : public StateFlow<Buffer<AliasInfo>, QList<1>>
{
public:
    /**
       Constructs a new AliasAllocator flow.

       @param if_id is a 48-bit NMRAnet NodeID. This node id will be used for
       reserving all aliases. This NodeID must be unique to the hardware on the
       bus; it will typically be the same as the NodeID of some virtual node
       that the current application will be creating.

       @param if_can is the interface to which this alias allocator should talk
       to.
     */
    AliasAllocator(NodeID if_id, IfCan *if_can);

    virtual ~AliasAllocator();

    /** "Allocate" a buffer from this pool (but without initialization) in
     * order to get a reserved alias. */
    QAsync *reserved_aliases()
    {
        return &reserved_alias_pool_;
    }

    /** If there is a pending alias allocation waiting for the timer to expire,
     * finishes it immediately. Needed in test destructors. */
    void TEST_finish_pending_allocation();

private:
    /** Listens to incoming CAN frames and handles alias conflicts. */
    class ConflictHandler : public IncomingFrameHandler
    {
    public:
        ConflictHandler(AliasAllocator *parent) : parent_(parent)
        {
        }
        virtual void send(Buffer<CanMessageData> *message, unsigned priority);

    private:
        AliasAllocator *parent_;
    } conflictHandler_;

    friend class ConflictHandler;

    AliasInfo *pending_alias()
    {
        return message()->data();
    }

    Action entry();
    Action handle_allocate_for_cid_frame();
    Action send_cid_frame();
    Action wait_done();
    Action send_rid_frame();

    Action handle_alias_conflict();

    /// Generates the next alias to check in the seed_ variable.
    void next_seed();

    friend class AsyncAliasAllocatorTest;
    friend class AsyncIfTest;

    StateFlowTimer timer_;

    /** Freelist of reserved aliases that can be used by virtual nodes. The
        AliasAllocatorFlow will post successfully reserved aliases to this
        allocator. */
    QAsync reserved_alias_pool_;

    /// 48-bit nodeID that we will use for alias reservations.
    NodeID if_id_;

    /** Physical interface for sending packets and assigning handlers to
     * received packets. */
    IfCan *if_can()
    {
        return static_cast<IfCan *>(service());
    }

    /// Which CID frame are we trying to send out. Valid values: 7..4
    unsigned cid_frame_sequence_ : 3;
    /// Set to 1 if an incoming frame signals an alias conflict.
    unsigned conflict_detected_ : 1;

    /// Seed for generating random-looking alias numbers.
    unsigned seed_ : 12;

    /// Notifiable used for tracking outgoing frames.
    BarrierNotifiable n_;

    /// Timer needed for sleeping the control flow.
    // SleepData sleep_helper_;
};
#if 0
    class AliasAllocator : public StateFlow<Buffer<AliasInfo>, QList<1>> {
    public:
        DynamicPool* reserved_aliases() { return mainBufferPool; }
    };
#endif

/** Create this object statically to add an alias allocator to an already
 * statically allocated interface. */
class AddAliasAllocator
{
public:
    AddAliasAllocator(NodeID if_id, IfCan *interface)
    {
        interface->set_alias_allocator(
            new AliasAllocator(if_id, interface));
    }
};

}  // namespace NMRAnet


#endif // _NMRANET_ASYNC_ALIAS_ALLOCATOR_HXX_
