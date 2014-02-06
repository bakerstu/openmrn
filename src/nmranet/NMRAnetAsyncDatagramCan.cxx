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

class CanDatagramParser::CanDatagramClient : public DatagramClient,
                                             public AddressedCanMessageWriteFlow
{
public:
    CanDatagramClient(AsyncIfCan* interface)
        : AddressedCanMessageWriteFlow(interface)
    {
    }

    virtual void write_datagram(NodeID src, NodeHandle dst, Buffer* payload, Notifiable* done) {
        result_ = OPERATION_PENDING;
        WriteAddressedMessage(If::MTI_DATAGRAM, src, dst, payload, done);
    }

    /** Requests cancelling the datagram send operation. Will notify the done
     * callback when the canceling is completed. */
    virtual void cancel() {
        HASSERT(0);
    }

private:
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
            return CallImmediately(ST(datagram_finalize));
        }
    }

    virtual ControlFlowAction timeout_looking_for_dst()
    {
        LOG(INFO, "CanDatagramWriteFlow: Could not resolve destination "
                  "address %012llx to an alias on the bus. Dropping packet.",
            dst_.id);
        UnregisterLocalHandler();
        return CallImmediately(ST(datagram_finalize));
    }

    ControlFlowAction datagram_finalize() {
        cleanup();  // will release the buffer.
        result_ &= ~OPERATION_PENDING;
        return Exit();
    }

};

CanDatagramParser::CanDatagramParser(AsyncIfCan* interface, int num_clients)
    : ifCan_(interface)
{
    lock_.TypedRelease(this);
    ifCan_->frame_dispatcher()->RegisterHandler(CAN_FILTER, CAN_MASK, this);
    for (int i = 0; i < num_clients; ++i) {
        /// @TODO(balazs.racz): need to deallocate these in the destructor.
        client_allocator()->TypedReleaseBack(new CanDatagramClient(interface));
    }
}

CanDatagramParser::~CanDatagramParser()
{
    ifCan_->frame_dispatcher()->UnregisterHandler(CAN_FILTER, CAN_MASK, this);
}

} // namespace NMRAnet
