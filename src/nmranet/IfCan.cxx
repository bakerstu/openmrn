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
 * \file IfCan.cxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#include "nmranet/IfCan.hxx"

#include "nmranet/AliasAllocator.hxx"
#include "nmranet/IfImpl.hxx"
#include "nmranet/IfCanImpl.hxx"
#include "nmranet/CanDefs.hxx"
#include "can_frame.h"

namespace nmranet
{

size_t g_alias_use_conflicts = 0;

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
class GlobalCanMessageWriteFlow : public CanMessageWriteFlow
{
public:
    GlobalCanMessageWriteFlow(IfCan *if_can) : CanMessageWriteFlow(if_can)
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
class AliasConflictHandler : public CanFrameStateFlow
{
public:
    AliasConflictHandler(IfCan *service) : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(this, 0,
                                                       ~((1 << 30) - 1));
    }

    ~AliasConflictHandler()
    {
        if_can()->frame_dispatcher()->unregister_handler(this, 0,
                                                         ~((1 << 30) - 1));
    }

    /// Handler callback for incoming messages.
    virtual Action entry()
    {
        uint32_t id = GET_CAN_FRAME_ID_EFF(*message()->data());
        release();
        if (CanDefs::get_priority(id) != CanDefs::NORMAL_PRIORITY)
        {
            // Probably not an OpenLCB frame.
            return exit();
        }
        NodeAlias alias = CanDefs::get_src(id);
        // If the caller comes with alias 000, we ignore that.
        NodeID node = alias ? if_can()->local_aliases()->lookup(alias) : 0;
        if (!node)
        {
            // This is not a local alias of ours.
            return exit();
        }
        if (CanDefs::is_cid_frame(id))
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
        CanDefs::control_init(*f, alias_, CanDefs::RID_FRAME, 0);
        if_can()->frame_write_flow()->send(b);
        return exit();
    }

private:
    /// Alias being checked.
    unsigned alias_ : 12;
};

/** This class listens for incoming CAN frames of regular unaddressed global
 * OpenLCB messages, then translates it in a generic way into a message,
 * computing its MTI. The resulting message is then passed to the generic If
 * for dispatching. */
class FrameToGlobalMessageParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
                     (CanDefs::GLOBAL_ADDRESSED << CanDefs::CAN_FRAME_TYPE_SHIFT) |
                     (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
                     (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK | CanDefs::CAN_FRAME_TYPE_MASK |
                   CanDefs::FRAME_TYPE_MASK | CanDefs::PRIORITY_MASK |
                   (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT)
    };

    FrameToGlobalMessageParser(IfCan *service) : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(this, CAN_FILTER,
                                                       CAN_MASK);
    }

    ~FrameToGlobalMessageParser()
    {
        if_can()->frame_dispatcher()->unregister_handler(this, CAN_FILTER,
                                                         CAN_MASK);
    }

    /// Handler entry for incoming messages.
    virtual Action entry()
    {
        struct can_frame *f = message()->data();
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        if (f->can_dlc)
        {
            buf_.assign((const char*)(&f->data[0]), f->can_dlc);
        }
        else
        {
            buf_.clear();
        }
        release();
        // Get the dispatch flow.
        return allocate_and_call(if_can()->dispatcher(), STATE(send_to_if));
    }

    Action send_to_if()
    {
        auto *b = get_allocation_result(if_can()->dispatcher());
        NMRAnetMessage* m = b->data();
        m->mti =
            static_cast<Defs::MTI>((id_ & CanDefs::MTI_MASK) >> CanDefs::MTI_SHIFT);
        m->payload = buf_;
        m->dst = {0, 0};
        m->dstNode = nullptr;
        m->src.alias = id_ & CanDefs::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(b, b->data()->priority());
        return exit();
    }

private:
    /// CAN frame ID, saved from the incoming frame.
    uint32_t id_;
    /// Payload for the MTI message.
    string buf_;
};

/** This class listens for incoming CAN frames of regular addressed OpenLCB
 * messages detined for local nodes, then translates them in a generic way into
 * a message, computing its MTI. The resulting message is then passed to the
 * generic If for dispatching. */
class FrameToAddressedMessageParser : public CanFrameStateFlow
{
public:
    enum
    {
        CAN_FILTER = CanMessageData::CAN_EXT_FRAME_FILTER |
                     (CanDefs::GLOBAL_ADDRESSED << CanDefs::CAN_FRAME_TYPE_SHIFT) |
                     (CanDefs::NMRANET_MSG << CanDefs::FRAME_TYPE_SHIFT) |
                     (CanDefs::NORMAL_PRIORITY << CanDefs::PRIORITY_SHIFT) |
                     (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT),
        CAN_MASK = CanMessageData::CAN_EXT_FRAME_MASK | CanDefs::CAN_FRAME_TYPE_MASK |
                   CanDefs::FRAME_TYPE_MASK | CanDefs::PRIORITY_MASK |
                   (Defs::MTI_ADDRESS_MASK << CanDefs::MTI_SHIFT)
    };

    FrameToAddressedMessageParser(IfCan *service) : CanFrameStateFlow(service)
    {
        if_can()->frame_dispatcher()->register_handler(this, CAN_FILTER, CAN_MASK);
    }

    ~FrameToAddressedMessageParser()
    {
        if_can()->frame_dispatcher()->unregister_handler(this, CAN_FILTER, CAN_MASK);
    }

    /// Handler entry for incoming messages.
    virtual Action entry()
    {
        struct can_frame *f = message()->data();
        id_ = GET_CAN_FRAME_ID_EFF(*f);
        // Do we have enough payload for the destination address?
        if (f->can_dlc < 2)
        {
            LOG(WARNING, "Incoming can frame addressed message without payload."
                         " can ID %08x data length %d",
                (unsigned)id_, f->can_dlc);
            // Drop the frame.
            return release_and_exit();
        }
        // Gets the destination address and checks if it is our node.
        dstHandle_.alias = (((unsigned)f->data[0] & 0xf) << 8) | f->data[1];
        dstHandle_.id = if_can()->local_aliases()->lookup(dstHandle_.alias);
        if (!dstHandle_.id) // Not destined for us.
        {
            LOG(VERBOSE, "Dropping addressed message not for local destination."
                         "id %08x Alias %03x",
                (unsigned)id_, dstHandle_.alias);
            // Drop the frame.
            return release_and_exit();
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
            return release_and_exit();
        }
        // Saves the payload.
        if (f->can_dlc > 2)
        {
            buf_.assign((const char*)(f->data + 2), f->can_dlc - 2);
        }
        else
        {
            buf_.clear();
        }
        /** Frame not needed anymore. If we want to save the reserved bits from
         *  the first octet, we need to revise this. */
        release();
        return allocate_and_call(if_can()->dispatcher(), STATE(send_to_if));
    }

    Action send_to_if()
    {
        auto *b = get_allocation_result(if_can()->dispatcher());
        NMRAnetMessage* m = b->data();
        m->mti =
            static_cast<Defs::MTI>((id_ & CanDefs::MTI_MASK) >> CanDefs::MTI_SHIFT);
        m->payload = buf_;
        m->dst = dstHandle_;
        // This might be NULL if dst is a proxied node in a router.
        m->dstNode = if_can()->lookup_local_node(dstHandle_.id);
        m->src.alias = id_ & CanDefs::SRC_MASK;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? if_can()->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            m->src.id = if_can()->local_aliases()->lookup(m->src.alias);
        }
        if_can()->dispatcher()->send(b, b->data()->priority());
        return exit();
    }

private:
    uint32_t id_;
    string buf_;
    NodeHandle dstHandle_;
};

IfCan::IfCan(ExecutorBase *executor, CanHubFlow *device,
                       int local_alias_cache_size, int remote_alias_cache_size,
                       int local_nodes_count)
    : If(executor, local_nodes_count)
    , CanIf(this, device)
    , localAliases_(0, local_alias_cache_size)
    , remoteAliases_(0, remote_alias_cache_size)
{
    auto *gflow = new GlobalCanMessageWriteFlow(this);
    globalWriteFlow_ = gflow;
    add_owned_flow(gflow);

    add_owned_flow(new AliasConflictHandler(this));
    add_owned_flow(new FrameToGlobalMessageParser(this));
    add_owned_flow(new VerifyNodeIdHandler(this));
    /*pipe_member_.reset(new CanReadFlow(device, this, executor));
    for (int i = 0; i < hw_write_flow_count; ++i)
    {
        CanFrameWriteFlow *f = new CanWriteFlow(this, executor);
        write_allocator_.Release(f);
        owned_flows_.push_back(std::unique_ptr<ControlFlow>(f));
    }
    for (int i = 0; i < global_can_write_flow_count; ++i)
    {
        owned_flows_.push_back(
            std::unique_ptr<Executable>(new GlobalCanMessageWriteFlow(this)));
            }*/
}

IfCan::~IfCan()
{
}

void IfCan::add_owned_flow(Executable *e)
{
    ownedFlows_.push_back(std::unique_ptr<Executable>(e));
}

void IfCan::set_alias_allocator(AliasAllocator *a)
{
    aliasAllocator_.reset(a);
}

void IfCan::add_addressed_message_support()
{
    add_owned_flow(new FrameToAddressedMessageParser(this));
    auto* f = new AddressedCanMessageWriteFlow(this);
    addressedWriteFlow_ = f;
    add_owned_flow(f);
}

void IfCan::canonicalize_handle(NodeHandle* h) {
  if (!h->id & !h->alias) return;
  if (!h->id) {
    h->id = local_aliases()->lookup(h->alias);
  }
  if (!h->id) {
    h->id = remote_aliases()->lookup(h->alias);
  }
  if (!h->alias) {
    h->alias = local_aliases()->lookup(h->id);
  }
  if (!h->alias) {
    h->alias = remote_aliases()->lookup(h->id);
  }
}

bool IfCan::matching_node(NodeHandle expected, NodeHandle actual)
{
    canonicalize_handle(&expected);
    canonicalize_handle(&actual);
    if (expected.id && actual.id)
    {
        return expected.id == actual.id;
    }
    if (expected.alias && actual.alias)
    {
        return expected.alias == actual.alias;
    }
    // Cannot reconcile.
    LOG(VERBOSE, "Cannot reconcile expected and actual NodeHandles for "
                 "equality testing.");
    return false;
}

} // namespace nmranet
