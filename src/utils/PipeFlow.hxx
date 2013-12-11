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

#ifndef _utils_PipeFlow_hxx_
#define _utils_PipeFlow_hxx_

#include "utils/pipe.hxx"
#include "executor/Dispatcher.hxx"

class PipeBuffer;
class PipeMember;

//! A packet of data to be written to the pipe device.
struct PipeBuffer : public QueueMember
{
    // The data to write.
    const void* data;
    // Number of bytes to write.
    size_t size;
    // Pipe member to skip during this write.
    PipeMember* skipMember;
    // Notifies the caller when the buffer is no longer needed.
    Notifiable* done;
};

/**
   An interface class for channels where we forward data from a pipe.

   Various different pipe receivers will implement this interface.
 */
class PipeMember : public HandlerBase
{
public:
    virtual ~PipeMember()
    {
    }
    /**
       Writes bytes to the device.

       @param buf is the source buffer from whch to write bytes.

       @param count is the number of bytes to write. count is a multiple of the
    parent pipe's unit, otherwise implementations are allowed to drop data or
    die.

       Blocks until the write is complete (that is, all data is enqueued in a
    buffer which will drain as the output device's speed
    allows). Implementations may want to use a lock inside to avoid writes from
    multiple sources being interleaved.
    */
    virtual void write(const void* buf, size_t count) = 0;

    virtual void async_write(const void* buf, size_t count, Notifiable* done)
    {
        write(buf, count);
        done->Notify();
    };
};

/** Implementation of the core of an asynchronous pipe. Handles taking incoming
 * packets from the allocator queue, and passing them to pipe members. */
class PipeFlow : public DispatchFlow<uint32_t>
{
public:
    PipeFlow(Executor* e) : DispatchFlow<uint32_t>(e)
    {
        static const int PIPE_BUFFER_COUNT = 5;
        allBuffers_.reset(new PipeBuffer[PIPE_BUFFER_COUNT]);
        for (int i = 0; i < PIPE_BUFFER_COUNT; ++i)
        {
            empty_buffers()->TypedRelease(&allBuffers_[i]);
        }
        StartFlowAt(ST(wait_for_buffer));
    }

    ~PipeFlow()
    {
        StartFlowAt(ST(NotStarted));
        executor()->WaitUntilEmpty();
    }

    TypedAllocator<PipeBuffer>* full_buffers()
    {
        return &fullBufferAllocator_;
    }

    TypedAllocator<PipeBuffer>* empty_buffers()
    {
        return &emptyBufferAllocator_;
    }

    //! Adds a specific handler.
    void RegisterMember(PipeMember* handler)
    {
        RegisterHandler(0, 0, handler);
    }

    //! Removes a specific instance of a handler from this IF.
    void UnregisterMember(PipeMember* handler)
    {
        UnregisterHandler(0, 0, handler);
    }

protected:
    virtual void CheckNotStartedState()
    {
    }

private:
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
        empty_buffers()->TypedRelease(currentBuffer_);
        currentBuffer_ = nullptr;
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

    PipeBuffer* currentBuffer_;
    std::unique_ptr<PipeBuffer[]> allBuffers_;
    TypedAllocator<PipeBuffer> fullBufferAllocator_;
    TypedAllocator<PipeBuffer> emptyBufferAllocator_;
};

#endif // _utils_PipeFlow_hxx_
