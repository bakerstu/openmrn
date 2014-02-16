/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file AsyncIfCanImpl.hxx
 *
 * Implementation flows for writing to the CANbus.
 *
 * @author Balazs Racz
 * @date 27 Jan 2013
 */

#ifndef _NMRANET_ASYNC_IF_CAN_IMPL_HXX_
#define _NMRANET_ASYNC_IF_CAN_IMPL_HXX_

#include "nmranet/NMRAnetIfCan.hxx"
#include "executor/control_flow.hxx"
#include "nmranet/AsyncIfImpl.hxx"
#include "nmranet/AsyncAliasAllocator.hxx"

namespace NMRAnet {

extern long long ADDRESSED_MESSAGE_LOOKUP_TIMEOUT_NSEC;

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

protected:
    ControlFlowAction get_can_frame_buffer()
    {
        return Allocate(if_can_->write_allocator(), ST(fill_can_frame_buffer));
    }

private:
    virtual ControlFlowAction fill_can_frame_buffer()
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

/* The addressed write flow is responsible for sending addressed messages to
 * the CANbus. It uses some shared states from the generic CAN write flow base
 * class, and extends it with destination alias lookup states.  */
class AddressedCanMessageWriteFlow : public CanMessageWriteFlow,
                                     private IncomingFrameHandler
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

    virtual ControlFlowAction timeout_looking_for_dst()
    {
        LOG(INFO, "AddressedWriteFlow: Could not resolve destination "
                  "address %012llx to an alias on the bus. Dropping packet.",
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


}  // namespace NMRAnet

#endif //_NMRANET_ASYNC_IF_CAN_IMPL_HXX_
