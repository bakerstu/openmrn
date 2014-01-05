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
 * \file AsyncIfImpl.hxx
 *
 * Implementation details for the asynchronous NMRAnet interfaces. This file
 * should only be needed in hardware interface implementations.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#ifndef _NMRAnetAsyncIfImpl_hxx_
#define _NMRAnetAsyncIfImpl_hxx_

#include "nmranet/AsyncIf.hxx"

namespace NMRAnet
{

/** Implementation of the hardware-independent parts of the write flows. */
class WriteFlowBase : public WriteFlow
{
public:
    WriteFlowBase(Executor* e, Notifiable* done) : WriteFlow(e, done)
    {
    }

protected:
    /** This function will be called (on the main executor) to initiate sending
     * this message to the hardware. The flow will then execute the returned
     * action.
     *
     * NOTE: it is possible that this functon will never be called for a given
     * flow. */
    virtual ControlFlowAction send_to_hardware() = 0;

    /// @returns the interface that this flow is assigned to.
    virtual AsyncIf* async_if() = 0;

    /** Implementations shall call this function when they are done with
     * sending the packet.
     */
    void cleanup();

    If::MTI mti_;    ///< MTI of message to send.
    NodeID src_;     ///< Source node that wants to send this message.
    NodeHandle dst_; /**< Destination node to send message to, or 0 if global
                        message. */
    AsyncNode* dstNode_; ///< node pointer to dst node (for local addressed).
    Buffer* data_;       ///< Message payload.

protected:
    /// @returns the allocator that this flow belongs to.
    virtual TypedAllocator<WriteFlow>* allocator() = 0;

private:
    /// Entry point for external callers.
    virtual void WriteAddressedMessage(If::MTI mti, NodeID src, NodeHandle dst,
                                       Buffer* data, Notifiable* done)
    {
        HASSERT(IsNotStarted());
        Restart(done);
        mti_ = mti;
        src_ = src;
        dst_ = dst;
        dstNode_ = nullptr;
        data_ = data;
        StartFlowAt(ST(maybe_send_to_local_node));
    }

    /// Entry point for external callers.
    virtual void WriteGlobalMessage(If::MTI mti, NodeID src, Buffer* data,
                                    Notifiable* done)
    {
        HASSERT(IsNotStarted());
        Restart(done);
        mti_ = mti;
        src_ = src;
        dst_.id = 0;
        dst_.alias = 0;
        dstNode_ = nullptr;
        data_ = data;
        StartFlowAt(ST(send_to_local_nodes));
    }

    /// Copies the buffered message to the local asyncif->dispatcher.
    void
    send_message_to_local_dispatcher(AsyncIf::MessageDispatchFlow* dispatcher);

    ControlFlowAction maybe_send_to_local_node();
    ControlFlowAction send_to_local_nodes();
    ControlFlowAction unaddressed_with_local_dispatcher();
    ControlFlowAction addressed_with_local_dispatcher();
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfImpl_hxx_
