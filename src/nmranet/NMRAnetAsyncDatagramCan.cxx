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
 * @date 27 Jan 2013
 */

#include "nmranet/NMRAnetAsyncDatagramCan.hxx"
#include "nmranet/AsyncIfCanImpl.hxx"

namespace NMRAnet
{

long long DATAGRAM_RESPONSE_TIMEOUT_NSEC = SEC_TO_NSEC(3);

class CanDatagramClient : public DatagramClient,
                          public AddressedCanMessageWriteFlow,
                          private IncomingMessageHandler
{
public:
    CanDatagramClient(AsyncIfCan* interface)
        : AddressedCanMessageWriteFlow(interface)
    {
    }

    virtual void write_datagram(NodeID src, NodeHandle dst, Buffer* payload,
                                Notifiable* done)
    {
        result_ = OPERATION_PENDING;
        if_can_->dispatcher()->RegisterHandler(MTI_1, MASK_1, this);
        if_can_->dispatcher()->RegisterHandler(MTI_2, MASK_2, this);
        WriteAddressedMessage(If::MTI_DATAGRAM, src, dst, payload, done);
    }

    /** Requests cancelling the datagram send operation. Will notify the done
     * callback when the canceling is completed. */
    virtual void cancel()
    {
        HASSERT(0);
    }

private:
    enum
    {
        MTI_1a = If::MTI_TERMINATE_DUE_TO_ERROR,
        MTI_1b = If::MTI_OPTIONAL_INTERACTION_REJECTED,
        MASK_1 = !(MTI_1a ^ MTI_1b),
        MTI_1 = MTI_1a,
        MTI_2a = If::MTI_DATAGRAM_OK,
        MTI_2b = If::MTI_DATAGRAM_REJECTED,
        MASK_2 = ~(MTI_2a ^ MTI_2b),
        MTI_2 = MTI_2a,
    };

    void RegisterHandlers()
    {
    }

    virtual TypedAllocator<WriteFlow>* allocator() {
        // A call to this funciton means we tried to release this flow as if it
        // was an addressed message write flow instead of a datagram write
        // flow.
        HASSERT(0);
    }

    virtual ControlFlowAction fill_can_frame_buffer()
    {
        LOG(VERBOSE, "fill can frame buffer");
        CanFrameWriteFlow* write_flow;
        GetAllocationResult(&write_flow);
        struct can_frame* f = write_flow->mutable_frame();
        HASSERT(mti_ == If::MTI_DATAGRAM);

        // Sets the CAN id.
        uint32_t can_id = 0x1A000000;
        IfCan::set_src(&can_id, src_alias_);
        LOG(VERBOSE, "dst alias %x", dst_alias_);
        IfCan::set_dst(&can_id, dst_alias_);

        HASSERT(data_);
        bool need_more_frames = false;
        unsigned len = data_->used() - data_offset_;
        if (len > 8)
        {
            len = 8;
            // This is not the last frame.
            need_more_frames = true;
            if (data_offset_)
            {
                IfCan::set_can_frame_type(&can_id,
                                          IfCan::DATAGRAM_MIDDLE_FRAME);
            }
            else
            {
                IfCan::set_can_frame_type(&can_id, IfCan::DATAGRAM_FIRST_FRAME);
            }
        }
        else
        {
            // No more data after this frame.
            if (data_offset_)
            {
                IfCan::set_can_frame_type(&can_id, IfCan::DATAGRAM_FINAL_FRAME);
            }
            else
            {
                IfCan::set_can_frame_type(&can_id, IfCan::DATAGRAM_ONE_FRAME);
            }
        }

        uint8_t* b = static_cast<uint8_t*>(data_->start());
        memcpy(f->data, b + data_offset_, len);
        data_offset_ += len;
        f->can_dlc = len;

        SET_CAN_FRAME_ID_EFF(*f, can_id);
        write_flow->Send(nullptr);

        if (need_more_frames)
        {
            return CallImmediately(ST(get_can_frame_buffer));
        }
        else
        {
            return Sleep(&sleep_data_, DATAGRAM_RESPONSE_TIMEOUT_NSEC,
                         ST(timeout_waiting_for_dg_response));
        }
    }

    // override.
    virtual ControlFlowAction timeout_looking_for_dst()
    {
        LOG(INFO, "CanDatagramWriteFlow: Could not resolve destination "
                  "address %012llx to an alias on the bus. Dropping packet.",
            dst_.id);
        UnregisterLocalHandler();
        result_ |= PERMANENT_ERROR | DST_NOT_FOUND;
        return CallImmediately(ST(datagram_finalize));
    }

    virtual ControlFlowAction timeout_waiting_for_dg_response()
    {
        LOG(INFO, "CanDatagramWriteFlow: No datagram response arrived from "
                  "destination %012llx.",
            dst_.id);
        result_ |= PERMANENT_ERROR | TIMEOUT;
        return CallImmediately(ST(datagram_finalize));
    }

    ControlFlowAction datagram_finalize()
    {
        if_can_->dispatcher()->UnregisterHandler(MTI_1, MASK_1, this);
        if_can_->dispatcher()->UnregisterHandler(MTI_2, MASK_2, this);
        cleanup(); // will release the buffer.
        HASSERT(result_ & OPERATION_PENDING);
        result_ &= ~OPERATION_PENDING;
        // Will notify the done_ closure.
        return Exit();
    }

    // Callback when a matching response comes in on the bus.
    virtual void handle_message(IncomingMessage* message, Notifiable* done)
    {
        LOG(INFO, "%p: Incoming response to datagram: mti %x from %x", this, (int)message->mti, (int) message->src.alias);
        // This will call done when the method returns.
        AutoNotify n(done);
        // First we check that the response is for this source node.
        if (message->dst.id)
        {
            if (message->dst.id != src_) 
            {
                LOG(VERBOSE, "wrong dst"); 
                return;
            }
        }
        else if (message->dst.alias != src_alias_)
        {
            LOG(VERBOSE, "wrong dst alias"); 
            /* Here we hope that the source alias was not released by the time
             * the response comes in. */
            return;
        }
        // We also check that the source of the response is our destination.
        if (message->src.id && dst_.id)
        {
            if (message->src.id != dst_.id) {
                LOG(VERBOSE, "wrong src"); 
                return;
            }
        }
        else if (message->src.alias)
        {
            // We hope the dst_alias_ has not changed yet.
            if (message->src.alias != dst_alias_) {
                LOG(VERBOSE, "wrong src alias %x %x",(int)message->src.alias, (int)dst_alias_); 
                return;
            }
        }
        else
        {
            /// @TODO(balazs.racz): we should initiate an alias lookup here.
            HASSERT(0); // Don't know how to match the response source.
        }

        uint16_t error_code = 0;
        uint8_t payload_length = 0;
        const uint8_t* payload = nullptr;
        if (message->payload)
        {
            payload = static_cast<const uint8_t*>(message->payload->start());
            payload_length = message->payload->used();
        }
        if (payload_length >= 2)
        {
            error_code = (((uint16_t)payload[0]) << 8) | payload[1];
        }

        switch (message->mti)
        {
            case If::MTI_TERMINATE_DUE_TO_ERROR:
            case If::MTI_OPTIONAL_INTERACTION_REJECTED:
            {
                if (payload_length >= 4)
                {
                    uint16_t return_mti = payload[2];
                    return_mti <<= 8;
                    return_mti |= payload[3];
                    if (return_mti != If::MTI_DATAGRAM)
                    {
                        // This must be a rejection of some other
                        // message. Ignore.
                        LOG(VERBOSE, "wrong rejection mti");
                        return;
                    }
                }
            } // fall through
            case If::MTI_DATAGRAM_REJECTED:
            {
                result_ &= ~0xffff;
                result_ |= error_code;
                // Ensures that an error response is visible in the flags.
                if (!(result_ & (PERMANENT_ERROR | RESEND_OK)))
                {
                    result_ |= PERMANENT_ERROR;
                }
                break;
            }
            case If::MTI_DATAGRAM_OK:
            {
                if (payload_length)
                {
                    result_ &= ~(0xff << RESPONSE_FLAGS_SHIFT);
                    result_ |= payload[0] << RESPONSE_FLAGS_SHIFT;
                }
                result_ |= OPERATION_SUCCESS;
                break;
            }
            default:
                // Ignore message.
                LOG(VERBOSE, "unknown mti");
                return;
        } // switch response MTI

        // Stops waiting for response.
        StopTimer(&sleep_data_);
        /// @TODO(balazs.racz) Here we might want to decide whether to start a
        /// retry.
        LOG(VERBOSE, "restarting at datagram finalize");
        StartFlowAt(ST(datagram_finalize));
    } // handle_message
};

/** Frame handler that assembles incoming datagram fragments into a single
 * datagram message. */
class CanDatagramParser : public IncomingFrameHandler, public AllocationResult
{
public:
    enum
    {
        CAN_FILTER = AsyncIfCan::CAN_EXT_FRAME_FILTER |
                     (IfCan::NMRANET_MSG << IfCan::FRAME_TYPE_SHIFT) |
                     (IfCan::NORMAL_PRIORITY << IfCan::PRIORITY_SHIFT),
        CAN_MASK = AsyncIfCan::CAN_EXT_FRAME_MASK | IfCan::FRAME_TYPE_MASK |
                   IfCan::PRIORITY_MASK,
    };

    /** @param num_clients tells how many datagram write flows (aka client
     * flows) to put into the client allocator. */
    CanDatagramParser(AsyncIfCan* interface);
    ~CanDatagramParser();

    /// Lock for the incoming CAN frames.
    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    /// Handler callback for incoming frames.
    virtual void handle_message(struct can_frame* f, Notifiable* done)
    {
        AutoNotify an(done);
        TypedAutoRelease<IncomingFrameHandler> ar(&lock_, this);
        errorCode_ = 0;

        uint32_t id = GET_CAN_FRAME_ID_EFF(*f);
        unsigned can_frame_type =
            (id & IfCan::CAN_FRAME_TYPE_MASK) >> IfCan::CAN_FRAME_TYPE_SHIFT;

        if (can_frame_type < 2 || can_frame_type > 5)
        {
            // Not datagram frame.
            return;
        }

        srcAlias_ = (id & IfCan::SRC_MASK) >> IfCan::SRC_SHIFT;

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
                void*& map_entry = pendingBuffers_[buffer_key];
                if (map_entry)
                {
                    static_cast<Buffer*>(map_entry)->free();
                    pendingBuffers_.erase(buffer_key);
                    /** Frames came out of order or more than one datagram is
                     * being sent to the same dst. */
                    errorCode_ = DatagramClient::RESEND_OK |
                                 DatagramClient::OUT_OF_ORDER;
                    break;
                }

                // Datagram first frame. Get a full buffer.
                buf_ = buffer_alloc(72);
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
            errorCode_ =
                DatagramClient::RESEND_OK | DatagramClient::OUT_OF_ORDER;
        }
        else if (buf_->available() < f->can_dlc)
        {
            // Too long datagram arrived.
            LOG(WARNING, "AsyncDatagramCan: too long incoming datagram arrived."
                         " Size: %d",
                (int)(buf_->used() + f->can_dlc));
            errorCode_ = DatagramClient::PERMANENT_ERROR;
        }

        if (errorCode_)
        {
            // Keeps the lock on *this.
            ar.Transfer();
            // Gets the send flow to send rejection.
            ifCan_->addressed_write_allocator()->AllocateEntry(this);
            return;
        }

        // Copies new data into buf.
        memcpy(buf_->position(), f->data, f->can_dlc);
        buf_->advance(f->can_dlc);

        if (last_frame)
        {
            // Datagram is complete; let's send it to higher level If.

            // Keeps the lock on *this.
            ar.Transfer();
            // Gets the dispatch flow.
            ifCan_->dispatcher()->allocator()->AllocateEntry(this);
        }
    }

    /// Callback when the dispatch flow is ours.
    virtual void AllocationCallback(QueueMember* entry)
    {
        if (errorCode_)
        {
            send_rejection(entry);
        }
        else
        {
            datagram_complete(entry);
        }
    }

    /** Sends a datagram rejection. The lock_ is held and must be
     * released. entry is an If::addressed write flow. errorCode_ != 0. */
    void send_rejection(QueueMember* entry)
    {
        HASSERT(errorCode_);
        HASSERT(dstNode_);
        auto* f = ifCan_->addressed_write_allocator()->cast_result(entry);
        Buffer* payload = buffer_alloc(2);
        uint8_t* w = static_cast<uint8_t*>(payload->start());
        w[0] = (errorCode_ >> 8) & 0xff;
        w[1] = errorCode_ & 0xff;
        payload->advance(2);
        f->WriteAddressedMessage(If::MTI_DATAGRAM_REJECTED, dst_.id,
                                 {0, srcAlias_}, payload, nullptr);
        // Return ourselves to the pool.
        lock_.TypedRelease(this);
    }

    /** Requests the datagram in buf_, dstNode_ etc... to be sent to the
     * AsyncIf for processing. The lock_ is held and must be released. entry is
     * the dispatcher. */
    void datagram_complete(QueueMember* entry)
    {
        HASSERT(!errorCode_);
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

    virtual void Run()
    {
        HASSERT(0);
    }

private:
    // Buffer of the pending datagram. Filled when allocation is needed.
    Buffer* buf_;
    AsyncNode* dstNode_;

    NodeHandle dst_;
    unsigned short srcAlias_ : 12;
    // If non-zero, contains a Rejection error code and the datagram should not
    // be forwarded to the upper layer in this case.
    uint16_t errorCode_;

    /** Open datagram buffers. Keyed by (dstid | srcid), value is a Buffer*.
     * @TODO(balazs.racz) we need some kind of timeout-based release mechanism
     * in here. */
    StlMap<uint64_t, void*> pendingBuffers_;

    //! Lock for ourselves.
    TypedAllocator<IncomingFrameHandler> lock_;
    //! Parent interface.
    AsyncIfCan* ifCan_;
};

CanDatagramSupport::CanDatagramSupport(AsyncIfCan* interface,
                                       int num_registry_entries,
                                       int num_clients)
    : DatagramSupport(interface, num_registry_entries)
{
    if_can()->add_owned_flow(new CanDatagramParser(if_can()));
    for (int i = 0; i < num_clients; ++i)
    {
        auto* client_flow = new CanDatagramClient(if_can());
        if_can()->add_owned_flow(client_flow);
        client_allocator()->TypedReleaseBack(client_flow);
    }
}

Executable* TEST_CreateCanDatagramParser(AsyncIfCan* if_can) {
    return new CanDatagramParser(if_can);
}

CanDatagramSupport::~CanDatagramSupport()
{
}

CanDatagramParser::CanDatagramParser(AsyncIfCan* interface) : ifCan_(interface)
{
    lock_.TypedRelease(this);
    ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
}

CanDatagramParser::~CanDatagramParser()
{
    ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK, this);
}

} // namespace NMRAnet
