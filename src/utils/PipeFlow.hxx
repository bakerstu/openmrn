/** \copyright
 * Copyright (c) 2013, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are  permitted provided that the following conditions are met:
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
 * \file PipeFlow.hxx
 * Implementation of the pipe dispatcher flow.
 *
 * @author Balazs Racz
 * @date 8 Dec 2013
 */

#include "utils/pipe.hxx"
#include "executor/Dispatcher.hxx"

/** Implementation of the core of an asynchronous pipe. Handles taking incoming
 * packets from the allocator queue, and passing them to pipe members. */
class PipeFlow : public DispatchFlow<uint32_t>
{
public:
    PipeFlow(Executor* e) : DispatchFlow<uint32_t>(e)
    {
        StartFlowAt(ST(wait_for_buffer));
    }

    TypedAllocator<PipeBuffer>* full_buffers()
    {
        return &fullBufferAllocator_;
    }

    TypedAllocator<PipeBuffer>* empty_buffers()
    {
        return &emptyBufferAllocator_;
    }

    ControlFlowAction wait_for_buffer()
    {
        return Allocate(&fullBufferAllocator_, ST(start_flow));
    }

    ControlFlowAction start_flow()
    {
        currentBuffer_ = GetTypedAllocationResult(&fullBufferAllocator_);
        IncomingMessage(0);
        return WaitForNotification();
    }

    virtual bool OnFlowFinished()
    {
        currentBuffer_->done->Notify();
        StartFlowAt(ST(wait_for_buffer));
        return false;
    }

    virtual void CallCurrentHandler(HandlerBase* b_handler, Notifiable* done)
    {
        PipeMember* member = static_cast<PipeMember*>(b_handler);
        if (member == currentBuffer_->skipMember)
        {
            done->Notify();
            return;
        }
        member->async_write(currentBuffer_->data, currentBuffer_->size, done);
    }

private:
    PipeBuffer* currentBuffer_;
    TypedAllocator<PipeBuffer> fullBufferAllocator_;
    TypedAllocator<PipeBuffer> emptyBufferAllocator_;
};
