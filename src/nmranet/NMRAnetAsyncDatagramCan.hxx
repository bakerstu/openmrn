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
 * \file NMRAnetAsyncDatagramCan.hxx
 *
 * CANbus datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 25 Jan 2013
 */

#ifndef _NMRAnetAsyncDatagramCan_hxx_
#define _NMRAnetAsyncDatagramCan_hxx_

#include "nmranet/NMRAnetIfCan.hxx"
#include "nmranet/AsyncIfCan.hxx"

namespace NMRAnet
{

/** Frame handler that assembles incoming datagram fragments into a single
 * datagram message. */
class CanDatagramParser : public IncomingFrameHandler, public AllocationResult
{
public:
    enum
    {
        CAN_FILTER = AsyncIfCan::CAN_EXT_FRAME_FILTER |
                     (0 << IfCan::CAN_FRAME_TYPE_SHIFT) |
                     (IfCan::NMRANET_MSG << IfCan::FRAME_TYPE_SHIFT) |
                     (IfCan::NORMAL_PRIORITY << IfCan::PRIORITY_SHIFT),
        CAN_MASK = AsyncIfCan::CAN_EXT_FRAME_MASK | IfCan::CAN_FRAME_TYPE_MASK |
                   IfCan::FRAME_TYPE_MASK | IfCan::PRIORITY_MASK,
    };

    CanDatagramParser(AsyncIfCan* interface) : ifCan_(interface)
    {
        lock_.TypedRelease(this);
        ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
    }

    ~CanDatagramParser()
    {
        ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK,
                                                      this);
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    /// Handler callback for incoming messages.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        AutoNotify an(done);
        TypedAutoRelease<IncomingFrameHandler> ar(&lock_, this);

        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        unsigned can_frame_type =
            (id & IfCan::CAN_FRAME_TYPE_MASK) >> IfCan::CAN_FRAME_TYPE_SHIFT;

        if (can_frame_type < 2 || can_frame_type > 5)
        {
            // Not datagram frame.
            return;
        }

        uint64_t buffer_key = id & (IfCan::DST_MASK | IfCan::SRC_MASK);

        dst_.alias = buffer_key >> (IfCan::DST_SHIFT);
        dstNode_ = nullptr;
        dst_.id = ifCan_->local_aliases()->lookup(NodeAlias(dst_.alias));
        if (dst_.id)
        {
            dstNode_ = ifCan_->lookup_local_node(dst_.id);
        }
        if (!dstNode_)
        {
            // Destination not local node.
            return;
        }

        buf_ = nullptr;
        bool last_frame = true;

        switch (can_frame_type)
        {
            case 2:
                // Single-frame datagram. Let's allocate one small buffer for
                // it.
                buf_ = buffer_alloc(f->can_dlc);
                break;
            case 3:
            {
                // Datagram first frame. Get a full buffer.
                buf_ = buffer_alloc(72);

                void*& map_entry = pendingBuffers_[buffer_key];
                if (!map_entry)
                {
                    /** @TODO(balazs.racz) maybe we should reject the datagram
                     * in this case. It looks as if there are multiple
                     * datagrams in parallel between the same src/dst pair. */
                    static_cast<Buffer*>(map_entry)->free();
                }
                map_entry = buf_;
                last_frame = false;
                break;
            }
            case 4:
                last_frame = false;
            // Fall through
            case 5:
            {
                auto it = pendingBuffers_.find(buffer_key);
                if (it != pendingBuffers_.end())
                {
                    buf_ = static_cast<Buffer*>(it->second);
                    if (last_frame)
                    {
                        pendingBuffers_.erase(it);
                    }
                }
                break;
            }
            default:
                // Not datagram frame.
                return;
        }

        if (!buf_)
        {
            //@TODO(balazs.racz) Reject datagram with temporary error resend OK.
            return;
        }

        // Copies new data into buf.
        HASSERT(buf_->available() >= f->can_dlc);
        memcpy(buf_->position(), f->data, f->can_dlc);
        buf_->advance(f->can_dlc);

        if (last_frame)
        {
            // Datagram is complete; let's send it to higher level If.
            srcAlias_ = (id & IfCan::SRC_MASK) >> IfCan::SRC_SHIFT;

            // Keeps the lock on *this.
            ar.Transfer();
            // Gets the dispatch flow.
            ifCan_->dispatcher()->allocator()->AllocateEntry(this);
        }
    }

    /// Callback when the dispatch flow is ours.
    virtual void AllocationCallback(QueueMember* entry)
    {
        auto* f = ifCan_->dispatcher()->allocator()->cast_result(entry);
        IncomingMessage* m = f->mutable_params();
        m->mti = If::MTI_DATAGRAM;
        m->payload = buf_;
        m->dst = dst_;
        m->dst_node = dstNode_;
        m->src.alias = srcAlias_;
        // This will be zero if the alias is not known.
        m->src.id =
            m->src.alias ? ifCan_->remote_aliases()->lookup(m->src.alias) : 0;
        if (!m->src.id && m->src.alias)
        {
            // It's unlikely to have a datagram coming in on the interface with
            // a local alias and still framed into CAN frames. But we still
            // handle it.
            m->src.id = ifCan_->local_aliases()->lookup(m->src.alias);
        }
        f->IncomingMessage(m->mti);
        // Return ourselves to the pool.
        lock_.TypedRelease(this);
    }

private:
    // Buffer of the pending datagram. Filled when allocation is needed.
    Buffer* buf_;
    AsyncNode* dstNode_;

    NodeHandle dst_;
    unsigned srcAlias_ : 12;

    /** Open datagram buffers. Keyed by (dstid | srcid), value is a Buffer*.
     * @TODO(balazs.racz) we need some kind of timeout-based release mechanism
     * in here. */
    StlMap<uint64_t, void*> pendingBuffers_;

    //! Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    //! Parent interface.
    AsyncIfCan* ifCan_;
};
} // namespace NMRAnet

#endif // _NMRAnetAsyncDatagramCan_hxx_
