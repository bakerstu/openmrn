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
 * \file WriteFlow.hxx
 *
 * Class that allows enqueing an outgoing message.
 *
 * @author Balazs Racz
 * @date 3 Nov 2013
 */

#ifndef _NMRAnetWriteFlow_hxx_
#define _NMRAnetWriteFlow_hxx_

#include "nmranet_config.h"
#include "executor/executor.hxx"
#include "executor/notifiable.hxx"

#include "utils/BufferQueue.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace NMRAnet
{

Executor* DefaultWriteFlowExecutor();

class WriteHelper : private Executable
{
public:
    static NodeHandle global()
    {
        return {0, 0};
    }

    WriteHelper(Executor* executor)
        : done_(nullptr),
          executor_(executor)
    {
    }

    /** Originales an NMRAnet message from a particular node.
     *
     * @param node is the originating node.
     * @param mti is the message to send
     * @param dst is the destination node to send to (may be Global())
     * @param buffer is the message payload.
     * @param done will be notified when the packet has been enqueued to the
     * physical layer. If done == nullptr, the sending is invoked synchronously.
     */
    void write_async(Node *node, If::MTI mti, NodeHandle dst,
                     Buffer *buffer, Notifiable* done)
    {
        HASSERT(!done_);
        node_ = node;
        mti_ = mti;
        dst_ = dst;
        buffer_ = buffer;
        if (done)
        {
            done_ = done;
            executor_->Add(this);
        }
        else
        {
            done_ = nullptr;
            Run();
        }
    }

private:
    /** Callback in the executor thread.
     */
    virtual void Run();

    Node *node_;
    If::MTI mti_;
    NodeHandle dst_;
    Buffer *buffer_;
    Notifiable* done_;
    Executor* executor_;
};

Buffer *EventIdToBuffer(uint64_t eventid);

}; /* namespace NMRAnet */

#endif  /* _NMRAnetWriteFlow_hxx_ */
