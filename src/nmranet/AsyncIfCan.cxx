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

#include "nmranet/AsyncIfImpl.hxx"
#include "nmranet/NMRAnetIfCan.hxx"
#include "nmranet_can.h"

namespace NMRAnet
{

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
    CanMessageWriteFlow(AsyncIfCan* if_can, Executor* e, Notifiable* done)
        : WriteFlowBase(e, done), if_can_(if_can)
    {
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
        IfCan::set_fields(&can_id, src_alias_, mti_,
                                 IfCan::GLOBAL_ADDRESSED, IfCan::NMRANET_MSG,
                                 IfCan::NORMAL_PRIORITY);
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
                    f->data[1] |= 0x20;
                }
                uint8_t* b = static_cast<uint8_t*>(data_->start());
                unsigned len = data_->used() - data_offset_;
                if (len > 6)
                {
                    len = 6;
                    // This is not the last frame.
                    need_more_frames = true;
                    f->data[1] |= 0x10;
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

AsyncIfCan::AsyncIfCan(Executor* executor, Pipe* device)
    : AsyncIf(executor), frame_dispatcher_(executor), pipe_member_(device)
{
    // Ensures that the underlying pipe will read and write whole frames.
    HASSERT(device->unit() == sizeof(struct can_frame));
    owned_flows_.push_back(
        std::unique_ptr<ControlFlow>(new CanReadFlow(this, executor)));
    owned_flows_.push_back(
        std::unique_ptr<ControlFlow>(new CanWriteFlow(this, executor)));
}

AsyncIfCan::~AsyncIfCan()
{
}

} // namespace NMRAnet
