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

#include "nmranet/AsyncAliasAllocator.hxx"
#include "nmranet/AsyncIfImpl.hxx"
#include "nmranet/AsyncIfCanImpl.hxx"
#include "nmranet/NMRAnetIfCan.hxx"
//#include "nmranet/NMRAnetWriteFlow.hxx"
#include "nmranet_can.h"

namespace NMRAnet
{

size_t g_alias_use_conflicts = 0;

DynamicPool *CanFrameWriteFlow::pool()
{
    return ifCan_->device()->pool();
}

void CanFrameWriteFlow::send(Buffer<CanHubData> *message, unsigned priority)
{
    LOG(INFO, "outgoing message %" PRIx32 ".",
        GET_CAN_FRAME_ID_EFF(message->data()->frame()));
    message->data()->skipMember_ = ifCan_->hub_port();
    ifCan_->device()->send(message, priority);
}

DynamicPool *CanFrameReadFlow::pool()
{
    return ifCan_->dispatcher()->pool();
}

void CanFrameReadFlow::send(Buffer<CanHubData> *message, unsigned priority)
{
    const struct can_frame &frame = message->data()->frame();
    if (IS_CAN_FRAME_ERR(frame) || IS_CAN_FRAME_RTR(frame) ||
        !IS_CAN_FRAME_EFF(frame))
    {
        // Ignores these frames.
        message->unref();
        return;
    }

    // We typecast the incoming buffer to a different buffer type that should be
    // the subset of the data.
    Buffer<CanMessageData> *incoming_buffer;

    // Checks that it fits.
    HASSERT(sizeof(*incoming_buffer) <= sizeof(*message));
    // Does the cast.
    incoming_buffer = static_cast<Buffer<CanMessageData> *>(
        static_cast<BufferBase *>(message));
    // Checks that the frame is still in the same place (by pointer).
    HASSERT(incoming_buffer->data()->mutable_frame() ==
            message->data()->mutable_frame());

    /** @TODO(balazs.racz): Figure out what priority the new message should be
     * at. */
    ifCan_->frame_dispatcher()->send(incoming_buffer, priority);
}

/** Specifies how long to wait for a response to an alias mapping enquiry
 * message when trying to send an addressed message to a destination. The final
 * timeout will be twice this time, because after the first timeout a global
 * message for verify node id request will also be sent out.
 *
 * This value is writable for unittesting purposes. We might consider moving it
 * to flash with a weak definition instead. */
extern long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC;
long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC = SEC_TO_NSEC(1);

namespace
{

/* This write flow inherits all the business logic from the parent, just
 * maintains a separate allocation queue. This allows global messages to go out
 * even if addressed messages are waiting for destination address
 * resolution. */
class GlobalCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    GlobalCanMessageWriteFlow(AsyncIfCan *if_can) : CanMessageWriteFlow(if_can)
    {
    }

protected:
    virtual Action entry()
    {
        return call_immediately(STATE(send_to_hardware));
    }

    virtual Action send_finished()
    {
        return call_immediately(STATE(global_entry));
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
 */
class AliasConflictHandler : public StateFlow<Buffer<CanMessageData>, QList<1>>
{
public:
    AliasConflictHandler(AsyncIfCan *service)
        : StateFlow<Buffer<CanMessageData>, QList<1>>(service)
    {
        if_can()->frame_dispatcher()->register_handler(this, 0,
                                                       ~((1 << 30) - 1));
    }

    ~AliasConflictHandler()
    {
        if_can()->frame_dispatcher()->unregister_handler(this, 0,
                                                         ~((1 << 30) - 1));
    }

    AsyncIfCan *if_can()
    {
        return static_cast<AsyncIfCan *>(service());
    }

    /// Handler callback for incoming messages.
    virtual Action entry()
    {
        uint32_t id = GET_CAN_FRAME_ID_EFF(*message()->data());
        release();
        if (IfCan::get_priority(id) != IfCan::NORMAL_PRIORITY)
        {
            // Probably not an OpenLCB frame.
            return exit();
        }
        NodeAlias alias = IfCan::get_src(id);
        // If the caller comes with alias 000, we ignore that.
        NodeID node = alias ? if_can()->local_aliases()->lookup(alias) : 0;
        if (!node)
        {
            // This is not a local alias of ours.
            return exit();
        }
        if (IfCan::is_cid_frame(id))
        {
            // This is a CID frame. We own the alias, let them know.
            alias_ = alias;
            return allocate_and_call(if_can()->frame_write_flow(),
                                     STATE(send_reserved_alias));
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
        if_can()->local_aliases()->remove(alias);
        return exit();
    }

    Action send_reserved_alias()
    {
        auto *b = get_allocation_result(if_can()->frame_write_flow());
        // We are sending an RID frame.
        struct can_frame *f = b->data();
        IfCan::control_init(*f, alias_, IfCan::RID_FRAME, 0);
        if_can()->frame_write_flow()->send(b);
        return exit();
    }

private:
    /// Alias being checked.
    unsigned alias_ : 12;
};

#if 0
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

    FrameToGlobalMessageParser(AsyncIfCan *if_can) : ifCan_(if_can)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->register_handler(CAN_FILTER, CAN_MASK,
                                                     this);
    }

    ~FrameToGlobalMessageParser()
    {
        ifCan_->frame_dispatcher()->unregister_handler(CAN_FILTER, CAN_MASK,
                                                       this);
    }

    virtual AllocatorBase *get_allocator()
    {
        return &lock_;
    }

    /// Handler callback for incoming messages.
    virtual void handle_message(struct can_frame *f, Notifiable *done)
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
        done->notify();
        // Get the dispatch flow.
        ifCan_->dispatcher()->allocator()->AllocateEntry(this);
    }

    virtual void AllocationCallback(QueueMember *entry)
    {
        auto *f = ifCan_->dispatcher()->allocator()->cast_result(entry);
        IncomingMessage *m = f->mutable_params();
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
    Buffer *buf_;
    /// Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    /// Parent interface.
    AsyncIfCan *ifCan_;
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

    FrameToAddressedMessageParser(AsyncIfCan *if_can) : ifCan_(if_can)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->register_handler(CAN_FILTER, CAN_MASK,
                                                     this);
    }

    ~FrameToAddressedMessageParser()
    {
        ifCan_->frame_dispatcher()->unregister_handler(CAN_FILTER, CAN_MASK,
                                                       this);
    }

    virtual AllocatorBase *get_allocator()
    {
        return &lock_;
    }

    /// Handler callback for incoming messages.
    virtual void handle_message(struct can_frame *f, Notifiable *done)
    {
        AutoNotify an(done);
        TypedAutoRelease<IncomingFrameHandler> ar(&lock_, this);
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        // Do we have enough payload for the destination address?
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

    virtual void AllocationCallback(QueueMember *entry)
    {
        auto *f = ifCan_->dispatcher()->allocator()->cast_result(entry);
        IncomingMessage *m = f->mutable_params();
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
    Buffer *buf_;
    NodeHandle dstHandle_;
    /// Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    /// Parent interface.
    AsyncIfCan *ifCan_;
};
#endif

AsyncIfCan::AsyncIfCan(ExecutorBase *executor, CanHubFlow *device,
                       int local_alias_cache_size, int remote_alias_cache_size,
                       int local_nodes_count)
    : AsyncIf(executor, local_nodes_count)
    , device_(device)
    , frameWriteFlow_(this)
    , frameReadFlow_(this)
    , frameDispatcher_(this)
    , localAliases_(0, local_alias_cache_size)
    , remoteAliases_(0, remote_alias_cache_size)
{
    auto *gflow = new GlobalCanMessageWriteFlow(this);
    globalWriteFlow_ = gflow;
    add_owned_flow(gflow);
    /*add_owned_flow(new VerifyNodeIdHandler(this));
    pipe_member_.reset(new CanReadFlow(device, this, executor));
    for (int i = 0; i < hw_write_flow_count; ++i)
    {
        CanFrameWriteFlow *f = new CanWriteFlow(this, executor);
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
            }*/
    this->device()->register_port(hub_port());
}

AsyncIfCan::~AsyncIfCan()
{
    this->device()->unregister_port(hub_port());
}

void AsyncIfCan::add_owned_flow(Executable *e)
{
    ownedFlows_.push_back(std::unique_ptr<Executable>(e));
}

void AsyncIfCan::set_alias_allocator(AsyncAliasAllocator *a)
{
    aliasAllocator_.reset(a);
}

void AsyncIfCan::add_addressed_message_support()
{
    ///@TODO(balazs.racz): implement addressed message support.
    /*owned_flows_.push_back(
        std::unique_ptr<Executable>(new FrameToAddressedMessageParser(this)));
    for (int i = 0; i < num_write_flows; ++i)
    {
        auto *f = new AddressedCanMessageWriteFlow(this);
        add_addressed_write_flow(f);
        owned_flows_.push_back(std::unique_ptr<Executable>(f));
        }*/
}

} // namespace NMRAnet
