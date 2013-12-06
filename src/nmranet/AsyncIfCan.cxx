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
 * \file AsyncIfCan.cxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#include "nmranet/AsyncIfCan.hxx"

#include "executor/allocator.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/AsyncIfImpl.hxx"
#include "nmranet/NMRAnetIfCan.hxx"
#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet_can.h"

namespace NMRAnet
{

size_t g_alias_use_conflicts = 0;

/**
   This is a control flow running in an infinite loop, reading from the device
   in an asynchronous way and sending packets to the dispatcher.

   Allocation semantics: none. Flow is allocated by the IfCan object, and never
   terminates.
 */
class AsyncIfCan::CanReadFlow : public ControlFlow
{
public:
    CanReadFlow(AsyncIfCan* if_can, Executor* e)
        : ControlFlow(e, nullptr), if_can_(if_can)
    {
        StartFlowAt(ST(GetDispatcher));
    }

    ~CanReadFlow()
    {
    }

private:
    ControlFlowAction GetDispatcher()
    {
        return Allocate(if_can_->frame_dispatcher()->allocator(),
                        ST(HandleDispatcher));
    }

    ControlFlowAction HandleDispatcher()
    {
        AsyncIfCan::FrameDispatchFlow* flow;
        GetAllocationResult(&flow);
        struct can_frame* frame = flow->mutable_params();
        if_can_->pipe_member()->ReceiveData(frame, sizeof(*frame), this);
        return WaitAndCall(ST(HandleFrameArrived));
    }

    ControlFlowAction HandleFrameArrived()
    {
        AsyncIfCan::FrameDispatchFlow* flow;
        GetAllocationResult(&flow);
        if (IS_CAN_FRAME_ERR(*flow->mutable_params()) ||
            IS_CAN_FRAME_RTR(*flow->mutable_params()) ||
            !IS_CAN_FRAME_EFF(*flow->mutable_params()))
        {
            // Ignore these frames, read another frame. We already have the
            // dispatcher object's ownership.
            return YieldAndCall(ST(HandleDispatcher));
        }
        uint32_t can_id = GET_CAN_FRAME_ID_EFF(*flow->mutable_params());
        flow->IncomingMessage(can_id);
        // Now we abandon the flow, which will come back into the allocator
        // when the message is handled properly.
        return YieldAndCall(ST(GetDispatcher));
    }

    //! Parent interface. Owned externlly.
    AsyncIfCan* if_can_;
};

class AsyncIfCan::CanWriteFlow : public CanFrameWriteFlow
{
public:
    CanWriteFlow(AsyncIfCan* parent, Executor* e)
        : CanFrameWriteFlow(e, nullptr), if_can_(parent)
    {
        if_can_->write_allocator()->Release(this);
    }

    virtual void Send(Notifiable* done)
    {
        Restart(done);
        StartFlowAt(ST(HandleSend));
    }

    virtual void Cancel()
    {
        ResetFrameEff();
        if_can_->write_allocator()->Release(this);
    }

private:
    ControlFlowAction HandleSend()
    {
        // @TODO(balazs.racz): implement true asynchronous sending.
        if_can_->pipe_member()->parent()->WriteToAll(if_can_->pipe_member(),
                                                     &frame_, sizeof(frame_));
        ResetFrameEff();
        //! @TODO(balazs.racz): there was a crash due ot spurious notification. Why?
        return YieldAndCall(ST(handle_release));
    }

    ControlFlowAction handle_release()
    {
        return ReleaseAndExit(if_can_->write_allocator(), this);
    }

    //! Parent interface. Owned externlly.
    AsyncIfCan* if_can_;
};

/** Implements the write-side conversion logic from generic messages to CAN
 * frames. */
class CanMessageWriteFlow : public WriteFlowBase
{
public:
    CanMessageWriteFlow(AsyncIfCan* if_can)
        : WriteFlowBase(if_can->frame_dispatcher()->executor(), nullptr),
          if_can_(if_can)
    {
    }

    virtual AsyncIf* async_if()
    {
        return if_can_;
    }

protected:
    /// @returns the allocator that this flow belongs to.
    virtual TypedAllocator<WriteFlow>* allocator() = 0;

    AsyncIfCan* if_can_; ///< The physical interface.

    unsigned src_alias_ : 12;  ///< Source node alias.
    unsigned dst_alias_ : 12;  ///< Destination node alias.
    unsigned data_offset_ : 8; /**< for continuation frames: which offset in
                                * the Buffer should we start the payload at. */

private:
    virtual ControlFlowAction send_to_hardware()
    {
        data_offset_ = 0;
        src_alias_ = 0;
        dst_alias_ = 0;
        if (data_)
        {
            // We have limited space for counting offsets. In practice this
            // valaue will be max 10 for certain traction control protocol
            // values. Longer data usually travels via datagrams or streams.
            HASSERT(data_->used() < 256);
        }
        // @TODO(balazs.racz): need to add alias lookups.
        return CallImmediately(ST(find_local_alias));
    }

    ControlFlowAction find_local_alias()
    {
        // We are on the IF's executor, so we can access the alias caches.
        src_alias_ = if_can_->local_aliases()->lookup(src_);
        if (!src_alias_)
        {
            return CallImmediately(ST(allocate_new_alias));
        }
        return CallImmediately(ST(src_alias_lookup_done));
    }

    ControlFlowAction allocate_new_alias()
    {
        /** At this point we assume that there will always be at least one
         * alias that is reserved but not yet used.
         *
         * @TODO(balazs.racz): implement proper local alias reclaim/reuse
         * mechanism. */
        HASSERT(if_can_->alias_allocator());
        return Allocate(if_can_->alias_allocator()->reserved_aliases(),
                        ST(take_new_alias));
    }

    ControlFlowAction take_new_alias()
    {
        /* In this function we do a number of queries to the local alias
         * cache. It is important that these queries show a consistent state;
         * we do not need to hold any mutex though because only the current
         * executor is allowed to access that object. */
        NodeAlias alias = 0;
        {
            AliasInfo* new_alias;
            GetAllocationResult(&new_alias);
            HASSERT(new_alias->alias);
            alias = new_alias->alias;
            // Releases the alias back for reallocating. This will trigger the
            // alias allocator flow.
            new_alias->Reset();
            if_can_->alias_allocator()->empty_aliases()->ReleaseBack(new_alias);
        }
        LOG(INFO, "Allocating new alias %03X", alias);

        // Checks that there was no conflict on this alias.
        if (if_can_->local_aliases()->lookup(alias) !=
            AliasCache::RESERVED_ALIAS_NODE_ID)
        {
            LOG(INFO, "Alias has seen conflict: %03X", alias);
            // Problem. Let's take another alias.
            return CallImmediately(ST(allocate_new_alias));
        }

        src_alias_ = alias;
        /** @TODO(balazs.racz): We leak aliases here in case of eviction by the
         * AliasCache object. */
        if_can_->local_aliases()->add(src_, alias);
        // Take a CAN frame to send off the AMD frame.
        return Allocate(if_can_->write_allocator(), ST(send_amd_frame));
    }

    ControlFlowAction send_amd_frame()
    {
        CanFrameWriteFlow* write_flow;
        GetAllocationResult(&write_flow);
        struct can_frame* f = write_flow->mutable_frame();
        IfCan::control_init(*f, src_alias_, IfCan::AMD_FRAME, 0);
        f->can_dlc = 6;
        uint64_t rd = htobe64(src_);
        memcpy(f->data, reinterpret_cast<uint8_t*>(&rd) + 2, 6);
        write_flow->Send(nullptr);
        // Source alias is settled.
        return CallImmediately(ST(src_alias_lookup_done));
    }

    ControlFlowAction src_alias_lookup_done()
    {
        return CallImmediately(ST(get_can_frame_buffer));
    }

    ControlFlowAction get_can_frame_buffer()
    {
        return Allocate(if_can_->write_allocator(), ST(fill_can_frame_buffer));
    }

    ControlFlowAction fill_can_frame_buffer()
    {
        CanFrameWriteFlow* write_flow;
        GetAllocationResult(&write_flow);
        struct can_frame* f = write_flow->mutable_frame();
        if (mti_ & (If::MTI_DATAGRAM_MASK | If::MTI_SPECIAL_MASK |
                    If::MTI_RESERVED_MASK))
        {
            // We don't know how to handle such an MTI in a generic way.
            write_flow->Cancel();
            return CallImmediately(ST(finalize));
        }
        HASSERT(!(mti_ & ~0xfff));

        // Sets the CAN id.
        uint32_t can_id = 0;
        IfCan::set_fields(&can_id, src_alias_, mti_, IfCan::GLOBAL_ADDRESSED,
                          IfCan::NMRANET_MSG, IfCan::NORMAL_PRIORITY);
        SET_CAN_FRAME_ID_EFF(*f, can_id);

        bool need_more_frames = false;
        // Sets the destination bytes if needed. Adds the payload.
        if (If::get_mti_address(mti_))
        {
            f->data[0] = dst_alias_ >> 8;
            f->data[1] = dst_alias_ & 0xff;
            if (data_)
            {
                if (data_offset_)
                {
                    // This is not the first frame.
                    f->data[0] |= 0x20;
                }
                uint8_t* b = static_cast<uint8_t*>(data_->start());
                unsigned len = data_->used() - data_offset_;
                if (len > 6)
                {
                    len = 6;
                    // This is not the last frame.
                    need_more_frames = true;
                    f->data[0] |= 0x10;
                }
                memcpy(f->data + 2, b + data_offset_, len);
                data_offset_ += len;
                f->can_dlc = 2 + len;
            }
        }
        else
        {
            if (data_)
            {
                HASSERT(data_->used() <= 8);
                memcpy(f->data, data_->start(), data_->used());
                f->can_dlc = data_->used();
            }
        }
        write_flow->Send(nullptr);
        if (need_more_frames)
        {
            return CallImmediately(ST(get_can_frame_buffer));
        }
        else
        {
            return CallImmediately(ST(finalize));
        }
    }

    ControlFlowAction finalize()
    {
        cleanup(); // Will release the buffer.
        return ReleaseAndExit(allocator(), this);
    }
};

namespace
{

class AddressedCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    AddressedCanMessageWriteFlow(AsyncIfCan* if_can)
        : CanMessageWriteFlow(if_can)
    {
    }

protected:
    virtual TypedAllocator<WriteFlow>* allocator()
    {
        return if_can_->addressed_write_allocator();
    }
};

class GlobalCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    GlobalCanMessageWriteFlow(AsyncIfCan* if_can) : CanMessageWriteFlow(if_can)
    {
        allocator()->ReleaseBack(this);
    }

protected:
    virtual TypedAllocator<WriteFlow>* allocator()
    {
        return if_can_->global_write_allocator();
    }
};

} // namespace

/** This class listens for incoming CAN messages, and if it sees a local alias
 * conflict, then takes the appropriate action:
 *
 * . if the conflict happened in alias check, it responds with an AMD frame.
 *
 * . if the conflict is with an allocated alias, kicks it out of the local
 * cache, forcing an alias reallocation for that node.
 *
 * . if the conflict is with a reserved but unused alias, kicks it out of the
 * cache and triggers allocating a new one instead.
 *
 * NOTE: this handler could be a control flow, but since the flow is very
 * simple, it is implemented on first principles. The resulting object will
 * take much less RAM. */
class AliasConflictHandler : public AllocationResult,
                             public IncomingFrameHandler
{
public:
    AliasConflictHandler(AsyncIfCan* if_can) : ifCan_(if_can)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->RegisterHandler(0, ~((1 << 30) - 1), this);
    }

    ~AliasConflictHandler()
    {
        ifCan_->frame_dispatcher()->UnregisterHandler(0, ~((1 << 30) - 1),
                                                      this);
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        done->Notify(); // We don't need the frame anymore.
        f = nullptr;
        done = nullptr;
        if (IfCan::get_priority(id) != IfCan::NORMAL_PRIORITY)
        {
            // Probably not an OpenLCB frame.
            lock_.TypedRelease(this);
            return;
        }
        NodeAlias alias = IfCan::get_src(id);
        NodeID node = ifCan_->local_aliases()->lookup(alias);
        if (!node)
        {
            // This is not a local alias of ours.
            lock_.TypedRelease(this);
            return;
        }
        if (IfCan::is_cid_frame(id))
        {
            // This is a CID frame. We own the alias, let them know.
            alias_ = alias;
            ifCan_->write_allocator()->AllocateEntry(this);
            // We keep the lock.
            return;
        }
        /* Removes the alias from the local alias cache.  If it was assigned to
         * a node, the node will grab a new alias next time it tries to send
         * something.  If it was reserved but not used, then whoever tries to
         * use it will realize due to the RESERVED_ALIAS_NODE_ID guard missing
         * from the cache.  We do not aggressively start looking for a new
         * alias in place of the missing one. If we want to do that, we would
         * have to find the entry in the list of
         * ifCan_->alias_allocator()->reserved_alias_allocator() and put it
         * into the empty_alias_allocator() instead to be picked up by the
         * alias allocator flow.  */
        g_alias_use_conflicts++;
        ifCan_->local_aliases()->remove(alias);
        lock_.TypedRelease(this);
    }

    virtual void AllocationCallback(QueueMember* entry)
    {
        frameWriteFlow_ = static_cast<CanFrameWriteFlow*>(entry);
        ifCan_->frame_dispatcher()->executor()->Add(this);
    }

    virtual void Run()
    {
        // We are sending an RID frame.
        struct can_frame* f = frameWriteFlow_->mutable_frame();
        IfCan::control_init(*f, alias_, IfCan::RID_FRAME, 0);
        frameWriteFlow_->Send(nullptr);
        lock_.TypedRelease(this);
    }

private:
    //! Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    //! Parent interface.
    AsyncIfCan* ifCan_;
    //! Alias being checked.
    unsigned alias_ : 12;
    //! The write flow we received from the allocator.
    CanFrameWriteFlow* frameWriteFlow_;
};

AsyncIfCan::AsyncIfCan(Executor* executor, Pipe* device,
                       int local_alias_cache_size, int remote_alias_cache_size)
    : AsyncIf(executor),
      frame_dispatcher_(executor),
      pipe_member_(device),
      localAliases_(0, local_alias_cache_size),
      remoteAliases_(0, remote_alias_cache_size)
{
    // Ensures that the underlying pipe will read and write whole frames.
    HASSERT(device->unit() == sizeof(struct can_frame));
    owned_flows_.push_back(
        std::unique_ptr<ControlFlow>(new CanReadFlow(this, executor)));
    for (int i = 0; i < 10; ++i)
    {
        owned_flows_.push_back(std::unique_ptr<ControlFlow>(
            new CanWriteFlow(this, DefaultWriteFlowExecutor())));
    }
    owned_flows_.push_back(
        std::unique_ptr<Executable>(new AliasConflictHandler(this)));
}

AsyncIfCan::~AsyncIfCan()
{
}

void AsyncIfCan::set_alias_allocator(AsyncAliasAllocator* a)
{
    aliasAllocator_.reset(a);
}

void AsyncIfCan::AddWriteFlows(int num_addressed, int num_global)
{
    for (int i = 0; i < num_global; ++i)
    {
        owned_flows_.push_back(
            std::unique_ptr<Executable>(new GlobalCanMessageWriteFlow(this)));
    }
    for (int i = 0; i < num_addressed; ++i)
    {
        owned_flows_.push_back(std::unique_ptr<Executable>(
            new AddressedCanMessageWriteFlow(this)));
    }
}

} // namespace NMRAnet
