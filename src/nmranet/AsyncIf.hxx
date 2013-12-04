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
 * \file AsyncIf.hxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _NMRAnetAsyncIf_hxx_
#define _NMRAnetAsyncIf_hxx_

#include "nmranet/ReadDispatch.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "utils/BufferQueue.hxx"

namespace NMRAnet
{

struct IncomingMessage
{
    //! OpenLCB MTI of the incoming message.
    If::MTI mti;
    //! Source node.
    NodeHandle src;
    //! Destination node.
    NodeHandle dst;
    //! If the destination node is local, this value is non-NULL.
    Node* dst_node;
    //! Data content in the message body. Owned by the dispatcher.
    Buffer* payload;
};

typedef ParamHandler<IncomingMessage> IncomingMessageHandler;

class WriteFlow : public ControlFlow
{
public:
    WriteFlow(Executor* e, Notifiable* done) : ControlFlow(e, done)
    {
    }

    /** Initiates sending an addressed message onto the NMRAnet bus.
     *
     * Must only be called if *this is an addressed flow.
     *
     * @param mti of the message to send
     * @param src is the NodeID of the originating node
     * @param dst is the destination node (cannot be 0,0)
     * @param data is the message payload (may be null)
     * @param done will be notified when the message is enqueued for sending.
     *  May be set to nullptr.
     */
    virtual void WriteAddressedMessage(If::MTI mti, NodeID src, NodeHandle dst,
                                       Buffer* data, Notifiable* done) = 0;

    /** Initiates sending an unaddressed (global) message onto the NMRAnet bus.
     *
     * Must only be called if *this is a global flow.
     *
     * @param mti of the message to send
     * @param src is the NodeID of the originating node
     * @param data is the message payload (may be null)
     * @param done will be notified when the message is enqueued for sending.
     *  May be set to nullptr.
     */
    virtual void WriteGlobalMessage(If::MTI mti, NodeID src, Buffer* data,
                                    Notifiable* done) = 0;
};

class AsyncIf
{
public:
    typedef TypedDispatchFlow<uint32_t, IncomingMessage> MessageDispatchFlow;

    AsyncIf(Executor* executor);

    /// @returns the dispatcher of incoming messages.
    MessageDispatchFlow* dispatcher()
    {
        return &dispatcher_;
    }

    /// @returns an allocator for sending global messages to the bus.
    TypedAllocator<WriteFlow>* global_write_allocator()
    {
        return &globalWriteAllocator;
    }

    /// @returns an allocator for sending addressed messages to the bus.
    TypedAllocator<WriteFlow>* addressed_write_allocator()
    {
        return &addressedWriteAllocator;
    }

private:
    /// Flow responsible for routing incoming messages to handlers.
    MessageDispatchFlow dispatcher_;

    /// Allocator containing the global write flows.
    TypedAllocator<WriteFlow> globalWriteAllocator;
    /// Allocator containing the addressed write flows.
    TypedAllocator<WriteFlow> addressedWriteAllocator;

    DISALLOW_COPY_AND_ASSIGN(AsyncIf);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIf_hxx_
