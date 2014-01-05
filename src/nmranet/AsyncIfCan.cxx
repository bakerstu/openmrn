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
class AsyncIfCan::CanReadFlow : public ControlFlow, public PipeMember
{
public:
    CanReadFlow(Pipe* pipe, AsyncIfCan* if_can, Executor* e)
        : ControlFlow(e, nullptr), if_can_(if_can), pipe_(pipe)
    {
        StartFlowAt(ST(Terminated));
        // Ensures that the underlying pipe will read and write whole frames.
        HASSERT(pipe_->unit() == sizeof(struct can_frame));
        pipe_->RegisterMember(this);
    }

    ~CanReadFlow()
    {
        HASSERT(IsDone());
        pipe_->UnregisterMember(this);
    }

    Pipe* parent()
    {
        return pipe_;
    }

    // Callback from the pipe.
    virtual void write(const void* buf, size_t count)
    {
        HASSERT(0);
    }

    // Callback from the pipe.
    virtual void async_write(const void* buf, size_t count, Notifiable* done)
    {
        HASSERT(IsDone());
        buf_ = static_cast<const struct can_frame*>(buf);
        byte_count_ = count;
        Restart(done);
        StartFlowAt(ST(next_frame));
    };

private:
    ControlFlowAction next_frame()
    {
        if (byte_count_ < sizeof(struct can_frame))
        {
            return Exit();
        }

        if (IS_CAN_FRAME_ERR(*buf_) || IS_CAN_FRAME_RTR(*buf_) ||
            !IS_CAN_FRAME_EFF(*buf_))
        {
            // Ignore these frames, read another frame.
            ++buf_;
            byte_count_ -= sizeof(*buf_);
            return RetryCurrentState();
        }

        return Allocate(if_can_->frame_dispatcher()->allocator(),
                        ST(handle_dispatcher));
    }

    ControlFlowAction handle_dispatcher()
    {
        // At this point we know that the frame is an extended frame and we
        // have the dispatcher.
        AsyncIfCan::FrameDispatchFlow* flow =
            GetTypedAllocationResult(if_can_->frame_dispatcher()->allocator());
        struct can_frame* frame = flow->mutable_params();
        memcpy(frame, buf_, sizeof(*frame));
        buf_++;
        byte_count_ -= sizeof(*buf_);
        uint32_t can_id = GET_CAN_FRAME_ID_EFF(*frame);
        flow->IncomingMessage(can_id);
        // Now we abandon the dispatcher flow, which will come back into the
        // allocator when the message is handled properly.
        return CallImmediately(ST(next_frame));
    }

    const struct can_frame* buf_;
    size_t byte_count_;
    //! Parent interface. Owned externlly.
    AsyncIfCan* if_can_;
    Pipe* pipe_;
};

class AsyncIfCan::CanWriteFlow : public CanFrameWriteFlow
{
public:
    CanWriteFlow(AsyncIfCan* parent, Executor* e)
        : CanFrameWriteFlow(e, nullptr), if_can_(parent)
    {
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
        PipeBuffer* b = &pipeBuffer_;
        b->data = &frame_;
        b->size = sizeof(frame_);
        b->skipMember = if_can_->pipe_member();
        b->done = notifiable_.NewCallback(this);
        if_can_->pipe_member()->parent()->SendBuffer(b);
        return WaitAndCall(ST(wait_for_sent));
    }

    ControlFlowAction wait_for_sent()
    {
        if (!notifiable_.HasBeenNotified())
            return WaitForNotification();
        ResetFrameEff();
        return CallImmediately(ST(handle_release));
    }

    ControlFlowAction handle_release()
    {
        return ReleaseAndExit(if_can_->write_allocator(), this);
    }

    PipeBuffer pipeBuffer_;
    ProxyNotifiable notifiable_;
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
        return CallImmediately(ST(find_local_alias));
    }

protected:
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

private:
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
        LOG(INFO, "Allocating new alias %03X for node %012llx", alias, src_);

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

protected:
    ControlFlowAction finalize()
    {
        cleanup(); // Will release the buffer.
        return ReleaseAndExit(allocator(), this);
    }
};

/** Specifies how long to wait for a response to an alias mapping enquiry
 * message when trying to send an addressed message to a destination. The final
 * timeout will be twice this time, because after the first timeout a global
 * message for verify node id request will also be sent out.
 *
 * This value is writable for unittesting purposes. We might consider moving it
 * to flash with a weak definition instead. */
extern long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC;
long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC = SEC_TO_NSEC(1);

/* This write flow inherits all the business logic from the parent, just
 * maintains a separate allocation queue. This allows global messages to go out
 * even if addressed messages are waiting for destination address
 * resolution. */
class AddressedCanMessageWriteFlow : public CanMessageWriteFlow,
                                     private IncomingFrameHandler
{
public:
    AddressedCanMessageWriteFlow(AsyncIfCan* if_can)
        : CanMessageWriteFlow(if_can)
    {
        if_can_->add_addressed_write_flow(this);
    }

protected:
    virtual TypedAllocator<WriteFlow>* allocator()
    {
        return if_can_->addressed_write_allocator();
    }

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
        HASSERT(dst_.id || dst_.alias); // We must have some kind of address.
        if (dst_.id)
        {
            dst_alias_ = if_can_->remote_aliases()->lookup(dst_.id);
        }
        if (dst_.alias && dst_alias_ && dst_.alias != dst_alias_)
        {
            /** @TODO(balazs.racz) what should we do here? The caller said
             * something different than the cache. */
            LOG(INFO, "AddressedWriteFlow: Caller supplied outdated alias. "
                      "Node id %012llx, Result from cache %03x, "
                      "alias from caller %03x.",
                dst_.id, dst_alias_, dst_.alias);
        }
        else if (!dst_alias_)
        {
            dst_alias_ = dst_.alias;
        }
        if (dst_alias_)
        {
            return CallImmediately(ST(find_local_alias));
        }
        else
        {
            return CallImmediately(ST(find_remote_alias));
        }
    }

    ControlFlowAction find_remote_alias()
    {
        RegisterLocalHandler();
        src_alias_ = if_can_->local_aliases()->lookup(src_);
        if (src_alias_)
        {
            // We can try to send an AME (alias mapping enquiry) frame.
            return Allocate(if_can_->write_allocator(), ST(send_ame_frame));
        }
        // No local alias -- we need to jump straight to local nodeid
        // verify. That will allocate a new local alias.
        return CallImmediately(ST(send_verify_nodeid_global));
        // send inquiry frame
        // sleep until response shows up or timer expires.
        // if timer expired then try with the node id verify message
        return WaitForNotification();
    }

    ControlFlowAction send_ame_frame()
    {
        CanFrameWriteFlow* write_flow;
        GetAllocationResult(&write_flow);
        struct can_frame* f = write_flow->mutable_frame();
        IfCan::control_init(*f, src_alias_, IfCan::AME_FRAME, 0);
        f->can_dlc = 6;
        uint64_t rd = htobe64(dst_.id);
        memcpy(f->data, reinterpret_cast<uint8_t*>(&rd) + 2, 6);
        write_flow->Send(nullptr);
        // We wait for a rmeote response to come via the handler input. If it
        // doesn't come for 1 sec, go to more global and send a global
        // unaddressed message inquiring the node id.
        return Sleep(&sleep_data_, ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC,
                     ST(send_verify_nodeid_global));
    }

    ControlFlowAction send_verify_nodeid_global()
    {
        return Allocate(if_can_->global_write_allocator(),
                        ST(fill_verify_nodeid_global));
    }

    ControlFlowAction fill_verify_nodeid_global()
    {
        WriteFlow* f =
            GetTypedAllocationResult(if_can_->global_write_allocator());
        f->WriteGlobalMessage(If::MTI_VERIFY_NODE_ID_GLOBAL, src_,
                              node_id_to_buffer(dst_.id), nullptr);
        return Sleep(&sleep_data_, ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC,
                     ST(timeout_looking_for_dst));
    }

    ControlFlowAction timeout_looking_for_dst()
    {
        LOG(INFO, "AsyncIfCan: Could not resolve destination address %012llx "
                  "to an alias on the bus. Dropping packet.",
            dst_.id);
        UnregisterLocalHandler();
        return CallImmediately(ST(finalize));
    }

    ControlFlowAction remote_alias_found()
    {
        HASSERT(dst_alias_);
        return CallImmediately(ST(find_local_alias));
    }

    enum
    {
        // AMD frames
        CAN_FILTER1 = AsyncIfCan::CAN_EXT_FRAME_FILTER | 0x10701000,
        CAN_MASK1 = AsyncIfCan::CAN_EXT_FRAME_MASK | 0x1FFFF000,
        // Initialization complete
        CAN_FILTER2 = AsyncIfCan::CAN_EXT_FRAME_FILTER | 0x19100000,
        CAN_MASK2 = AsyncIfCan::CAN_EXT_FRAME_MASK | 0x1FFFF000,
        // Verified node ID number
        CAN_FILTER3 = AsyncIfCan::CAN_EXT_FRAME_FILTER | 0x19170000,
        CAN_MASK3 = AsyncIfCan::CAN_EXT_FRAME_MASK | 0x1FFFF000,
    };

    // Registers *this to the ifcan to receive alias resolution messages.
    void RegisterLocalHandler()
    {
        if_can_->frame_dispatcher()->RegisterHandler(CAN_FILTER1, CAN_MASK1,
                                                     this);
        if_can_->frame_dispatcher()->RegisterHandler(CAN_FILTER2, CAN_MASK2,
                                                     this);
        if_can_->frame_dispatcher()->RegisterHandler(CAN_FILTER3, CAN_MASK3,
                                                     this);
    }

    // Unregisters *this from the ifcan to not receive alias resolution
    // messages.
    void UnregisterLocalHandler()
    {
        if_can_->frame_dispatcher()->UnregisterHandler(CAN_FILTER1, CAN_MASK1,
                                                       this);
        if_can_->frame_dispatcher()->UnregisterHandler(CAN_FILTER2, CAN_MASK2,
                                                       this);
        if_can_->frame_dispatcher()->UnregisterHandler(CAN_FILTER3, CAN_MASK3,
                                                       this);
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        AutoNotify an(done);
        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        if (f->can_dlc != 6)
        {
            // Not sending a node ID.
            return;
        }
        uint64_t nodeid_be = htobe64(dst_.id);
        uint8_t* nodeid_start = reinterpret_cast<uint8_t*>(&nodeid_be) + 2;
        if (memcmp(nodeid_start, f->data, 6))
        {
            // Node id does not match.
            return;
        }
        // Now: we have an alias.
        dst_alias_ = id & IfCan::SRC_MASK;
        if (!dst_alias_)
        {
            LOG(ERROR, "Incoming alias definition message with zero alias. "
                       "CAN frame id %08x",
                (unsigned)id);
            return;
        }
        UnregisterLocalHandler();
        if_can_->remote_aliases()->add(dst_.id, dst_alias_);
        StopTimer(&sleep_data_);
        StartFlowAt(ST(remote_alias_found));
    }

    SleepData sleep_data_;
};

namespace
{

/* This write flow inherits all the business logic from the parent, just
 * maintains a separate allocation queue. This allows global messages to go out
 * even if addressed messages are waiting for destination address
 * resolution. */
class GlobalCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    GlobalCanMessageWriteFlow(AsyncIfCan* if_can) : CanMessageWriteFlow(if_can)
    {
        if_can_->add_global_write_flow(this);
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
 * cache. This condition will be detected when a new node tries using that
 * alias.
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
        // If the caller comes with alias 000, we ignore that.
        NodeID node = alias ? ifCan_->local_aliases()->lookup(alias) : 0;
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
        frameWriteFlow_ = ifCan_->write_allocator()->cast_result(entry);
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

/** This class listens for incoming CAN frames of regular unaddressed global
 * OpenLCB messages, then translates it in a generic way into a message,
 * computing its MTI. The resulting message is then passed to the generic If
 * for dispatching. */
class FrameToGlobalMessageParser : public IncomingFrameHandler,
                                   public AllocationResult
{
public:
    enum
    {
        CAN_FILTER = AsyncIfCan::CAN_EXT_FRAME_FILTER |
                     (IfCan::GLOBAL_ADDRESSED << IfCan::CAN_FRAME_TYPE_SHIFT) |
                     (IfCan::NMRANET_MSG << IfCan::FRAME_TYPE_SHIFT) |
                     (IfCan::NORMAL_PRIORITY << IfCan::PRIORITY_SHIFT),
        CAN_MASK = AsyncIfCan::CAN_EXT_FRAME_MASK | IfCan::CAN_FRAME_TYPE_MASK |
                   IfCan::FRAME_TYPE_MASK | IfCan::PRIORITY_MASK |
                   (If::MTI_ADDRESS_MASK << IfCan::MTI_SHIFT)
    };

    FrameToGlobalMessageParser(AsyncIfCan* if_can) : ifCan_(if_can)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
    }

    ~FrameToGlobalMessageParser()
    {
        ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK,
                                                      this);
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        if (f->can_dlc)
        {
            buf_ = buffer_alloc(f->can_dlc);
            memcpy(buf_->start(), f->data, f->can_dlc);
            buf_->advance(f->can_dlc);
        }
        else
        {
            buf_ = nullptr;
        }
        // We don't need the frame anymore.
        done->Notify();
        // Get the dispatch flow.
        ifCan_->dispatcher()->allocator()->AllocateEntry(this);
    }

    virtual void AllocationCallback(QueueMember* entry)
    {
        auto* f = ifCan_->dispatcher()->allocator()->cast_result(entry);
        IncomingMessage* m = f->mutable_params();
        m->mti =
            static_cast<If::MTI>((id_ & IfCan::MTI_MASK) >> IfCan::MTI_SHIFT);
        m->payload = buf_;
        m->dst = {0, 0};
        m->dst_node = nullptr;
        m->src.alias = id_ & IfCan::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? ifCan_->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = ifCan_->local_aliases()->lookup(m->src.alias);
        }
        f->IncomingMessage(m->mti);
        // Return ourselves to the pool.
        lock_.TypedRelease(this);
    }

    virtual void Run()
    {
        HASSERT(0);
    }

private:
    uint32_t id_;
    Buffer* buf_;
    //! Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    //! Parent interface.
    AsyncIfCan* ifCan_;
};

/** This class listens for incoming CAN frames of regular addressed OpenLCB
 * messages detined for local nodes, then translates them in a generic way into
 * a message, computing its MTI. The resulting message is then passed to the
 * generic If for dispatching. */
class FrameToAddressedMessageParser : public IncomingFrameHandler,
                                      public AllocationResult
{
public:
    enum
    {
        CAN_FILTER = AsyncIfCan::CAN_EXT_FRAME_FILTER |
                     (IfCan::GLOBAL_ADDRESSED << IfCan::CAN_FRAME_TYPE_SHIFT) |
                     (IfCan::NMRANET_MSG << IfCan::FRAME_TYPE_SHIFT) |
                     (IfCan::NORMAL_PRIORITY << IfCan::PRIORITY_SHIFT) |
                     (If::MTI_ADDRESS_MASK << IfCan::MTI_SHIFT),
        CAN_MASK = AsyncIfCan::CAN_EXT_FRAME_MASK | IfCan::CAN_FRAME_TYPE_MASK |
                   IfCan::FRAME_TYPE_MASK | IfCan::PRIORITY_MASK |
                   (If::MTI_ADDRESS_MASK << IfCan::MTI_SHIFT)
    };

    FrameToAddressedMessageParser(AsyncIfCan* if_can) : ifCan_(if_can)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
    }

    ~FrameToAddressedMessageParser()
    {
        ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK,
                                                      this);
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        AutoNotify an(done);
        TypedAutoRelease<IncomingFrameHandler> ar(&lock_, this);
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        // Do we have enough payloade for the destination address?
        if (f->can_dlc < 2)
        {
            LOG(WARNING, "Incoming can frame addressed message without payload."
                         " can ID %08x data length %d",
                (unsigned)id_, f->can_dlc);
            // Drop the frame.
            return;
        }
        // Gets the destination address and checks if it is our node.
        dstHandle_.alias = (((unsigned)f->data[0] & 0xf) << 8) | f->data[1];
        dstHandle_.id = ifCan_->local_aliases()->lookup(dstHandle_.alias);
        if (!dstHandle_.id) // Not destined for us.
        {
            LOG(VERBOSE, "Dropping addressed message not for local destination."
                         "id %08x Alias %03x",
                (unsigned)id_, dstHandle_.alias);
            // Drop the frame.
            return;
        }
        // Checks the continuation bits.
        if (f->data[0] & 0b00110000)
        {
            /// @TODO: If this is not an only frame, we are in trouble here.
            LOG(WARNING, "Received an addressed message with multiple frames. "
                         "OpenMRN does not support generic reassembly yet. "
                         "frame ID=%08x, fddd=%02x%02x",
                (unsigned)id_, f->data[0], f->data[1]);
            // Drop the frame.
            return;
        }
        // Saves the payload.
        if (f->can_dlc > 2)
        {
            buf_ = buffer_alloc(f->can_dlc - 2);
            memcpy(buf_->start(), f->data + 2, f->can_dlc - 2);
            buf_->advance(f->can_dlc - 2);
        }
        else
        {
            buf_ = nullptr;
        }
        // Keeps the lock on *this.
        ar.Transfer();
        // Gets the dispatch flow.
        ifCan_->dispatcher()->allocator()->AllocateEntry(this);
    }

    virtual void AllocationCallback(QueueMember* entry)
    {
        auto* f = ifCan_->dispatcher()->allocator()->cast_result(entry);
        IncomingMessage* m = f->mutable_params();
        m->mti =
            static_cast<If::MTI>((id_ & IfCan::MTI_MASK) >> IfCan::MTI_SHIFT);
        m->payload = buf_;
        m->dst = dstHandle_;
        // This might be NULL if dst is a proxied node in a router.
        m->dst_node = ifCan_->lookup_local_node(dstHandle_.id);
        m->src.alias = id_ & IfCan::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? ifCan_->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = ifCan_->local_aliases()->lookup(m->src.alias);
        }
        f->IncomingMessage(m->mti);
        // Return ourselves to the pool.
        lock_.TypedRelease(this);
    }

    virtual void Run()
    {
        HASSERT(0);
    }

private:
    uint32_t id_;
    Buffer* buf_;
    NodeHandle dstHandle_;
    //! Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    //! Parent interface.
    AsyncIfCan* ifCan_;
};

AsyncIfCan::AsyncIfCan(Executor* executor, Pipe* device,
                       int local_alias_cache_size, int remote_alias_cache_size,
                       int hw_write_flow_count, int global_can_write_flow_count)
    : AsyncIf(executor),
      frame_dispatcher_(executor),
      localAliases_(0, local_alias_cache_size),
      remoteAliases_(0, remote_alias_cache_size)
{
    pipe_member_.reset(new CanReadFlow(device, this, executor));
    for (int i = 0; i < hw_write_flow_count; ++i)
    {
        CanFrameWriteFlow* f = new CanWriteFlow(this, executor);
        write_allocator_.Release(f);
        owned_flows_.push_back(std::unique_ptr<ControlFlow>(f));
    }
    owned_flows_.push_back(
        std::unique_ptr<Executable>(new AliasConflictHandler(this)));
    owned_flows_.push_back(
        std::unique_ptr<Executable>(new FrameToGlobalMessageParser(this)));
    for (int i = 0; i < global_can_write_flow_count; ++i)
    {
        owned_flows_.push_back(
            std::unique_ptr<Executable>(new GlobalCanMessageWriteFlow(this)));
    }
}

AsyncIfCan::~AsyncIfCan()
{
}

void AsyncIfCan::add_owned_flow(Executable* e)
{
    owned_flows_.push_back(std::unique_ptr<Executable>(e));
}

void AsyncIfCan::set_alias_allocator(AsyncAliasAllocator* a)
{
    aliasAllocator_.reset(a);
}

void AsyncIfCan::add_addressed_message_support(int num_write_flows)
{
    owned_flows_.push_back(
        std::unique_ptr<Executable>(new FrameToAddressedMessageParser(this)));
    for (int i = 0; i < num_write_flows; ++i)
    {
        owned_flows_.push_back(std::unique_ptr<Executable>(
            new AddressedCanMessageWriteFlow(this)));
    }
}

} // namespace NMRAnet
