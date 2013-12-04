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
        parent->write_allocator()->Release(this);
    }

    void Send(Notifiable* done) {
        Restart(done);
        StartFlowAt(ST(HandleSend));
    }

private:
    ControlFlowAction HandleSend() {
        // @TODO(balazs.racz): implement true asynchronous sending.
        if_can_->pipe_member()->parent()->WriteToAll(if_can_->pipe_member(),
                                                     &frame_,
                                                     sizeof(frame_));
        return ReleaseAndExit(if_can_->write_allocator(), this);
    }

    //! Parent interface. Owned externlly.
    AsyncIfCan* if_can_;
};

AsyncIfCan::AsyncIfCan(Executor* executor, Pipe* device)
    : AsyncIf(executor), frame_dispatcher_(executor), pipe_member_(device)
{
    // Ensures that the underlying pipe will read and write whole frames.
    HASSERT(device->unit() == sizeof(struct can_frame));
    owned_flows_.push_back(std::unique_ptr<ControlFlow>(new CanReadFlow(this, executor)));
    owned_flows_.push_back(std::unique_ptr<ControlFlow>(new CanWriteFlow(this, executor)));
}

AsyncIfCan::~AsyncIfCan()
{
}

} // namespace NMRAnet
