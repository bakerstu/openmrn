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

class CanDatagramParser::CanDatagramClient
    : public DatagramClient,
      private AddressedCanMessageWriteFlow,
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

    virtual ControlFlowAction fill_can_frame_buffer()
    {
        CanFrameWriteFlow* write_flow;
        GetAllocationResult(&write_flow);
        struct can_frame* f = write_flow->mutable_frame();
        HASSERT(mti_ == If::MTI_DATAGRAM);

        // Sets the CAN id.
        uint32_t can_id = 0x1A000000;
        IfCan::set_src(&can_id, src_alias_);
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
            "destination %012llx.", dst_.id);
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
        // This will call done when the method returns.
        AutoNotify n(done);
        // First we check that the response is for this source node.
        if (message->dst.id)
        {
            if (message->dst.id != src_)
                return;
        }
        else if (message->dst.alias != src_alias_)
        {
            /* Here we hope that the source alias was not released by the time
             * the response comes in. */
            return;
        }
        // We also check that the source of the response is our destination.
        if (message->src.id && dst_.id)
        {
            if (message->src.id != dst_.id)
                return;
        }
        else if (message->src.alias)
        {
            // We hope the dst_alias_ has not changed yet.
            if (message->src.alias != dst_alias_)
                return;
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
                        return;
                    }
                }
            }  // fall through
            case If::MTI_DATAGRAM_REJECTED:
            {
                result_ &= ~0xffff;
                result_ |= error_code;
                // Ensures that an error response is visible in the flags.
                if (!(result_ & (PERMANENT_ERROR | RESEND_OK))) {
                    result_ |= PERMANENT_ERROR;
                }
                break;
            }
            case If::MTI_DATAGRAM_OK:
            {
                if (payload_length) {
                    result_ &= ~(0xff << RESPONSE_FLAGS_SHIFT);
                    result_ |= payload[0] << RESPONSE_FLAGS_SHIFT;
                }
                result_ |= OPERATION_SUCCESS;
                break;
            }
        default:
            // Ignore message.
            return;
        } // switch response MTI

        // Stops waiting for response.
        StopTimer(&sleep_data_);
        /// @TODO(balazs.racz) Here we might want to decide whether to start a
        /// retry.
        StartFlowAt(ST(datagram_finalize));
    } // handle_message
};

CanDatagramParser::CanDatagramParser(AsyncIfCan* interface, int num_clients)
    : ifCan_(interface)
{
    lock_.TypedRelease(this);
    ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
    for (int i = 0; i < num_clients; ++i)
    {
        /// @TODO(balazs.racz): need to deallocate these in the destructor.
        client_allocator()->TypedReleaseBack(new CanDatagramClient(interface));
    }
}

CanDatagramParser::~CanDatagramParser()
{
    ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK, this);
}

} // namespace NMRAnet
