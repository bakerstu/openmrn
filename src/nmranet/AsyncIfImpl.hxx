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

private:
    /// Copies the buffered message to the local asyncif->dispatcher.
    void
    send_message_to_local_dispatcher(AsyncIf::MessageDispatchFlow* dispatcher);

    ControlFlowAction maybe_send_to_local_node();
    ControlFlowAction send_to_local_nodes();
    ControlFlowAction unaddressed_with_local_dispatcher();
    ControlFlowAction addressed_with_local_dispatcher();
};

/** This handler handles VerifyNodeId messages (both addressed and global) on
 * the interface level. Each interface implementation will want to create one
 * of these. */
class VerifyNodeIdHandler : private IncomingMessageHandler,
                            public AllocationResult
{
public:
    VerifyNodeIdHandler(AsyncIf* interface) : interface_(interface)
    {
        lock_.TypedRelease(this);
        interface_->dispatcher()->RegisterHandler(
            If::MTI_VERIFY_NODE_ID_GLOBAL & If::MTI_VERIFY_NODE_ID_ADDRESSED,
            0xffff & ~(If::MTI_VERIFY_NODE_ID_GLOBAL ^
                       If::MTI_VERIFY_NODE_ID_ADDRESSED),
            this);
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    //! Handler callback for incoming messages.
    virtual void handle_message(IncomingMessage* m, Notifiable* done)
    {
        AutoNotify an(done);
        TypedAutoRelease<IncomingMessageHandler> ar(&lock_, this);
        if (m->dst.id)
        {
            // Addressed message.
            srcNode_ = m->dst_node;
        }
        else if (m->payload && m->payload->used() == 6)
        {
            // Global message with a node id included
            NodeID id = buffer_to_node_id(m->payload);
            srcNode_ = interface_->lookup_local_node(id);
        }
        else
        {
// Global message. Everyone should respond.
#ifdef SIMPLE_NODE_ONLY
            // We assume there can be only one local node.
            AsyncIf::VNodeMap::Iterator it = interface_->localNodes_.begin();
            if (it == interface_->localNodes_.end())
            {
                // No local nodes.
                return;
            }
            srcNode_ = it->second;
            ++it;
            HASSERT(it == interface_->localNodes_.end());
#else
            // We need to do an iteration over all local nodes.
            it_ = interface_->localNodes_.begin();
            if (it_ == interface_->localNodes_.end())
            {
                // No local nodes.
                return;
            }
            srcNode_ = it_->second;
            ++it_;
#endif // not simple node.
        }
        if (srcNode_)
        {
            ar.Transfer();
            return interface_->global_write_allocator()->AllocateEntry(this);
        }
    }

#ifdef SIMPLE_NODE_ONLY
    virtual void AllocationCallback(QueueMember* entry)
    {
        WriteFlow* f = interface_->global_write_allocator()->cast_result(entry);
        NodeID id = srcNode_->node_id();
        f->WriteGlobalMessage(If::MTI_VERIFIED_NODE_ID_NUMBER, id,
                              node_id_to_buffer(id), nullptr);
        lock_.TypedRelease(this);
    }

    virtual void Run()
    {
        HASSERT(0);
    }
#else
    virtual void AllocationCallback(QueueMember* entry)
    {
        f_ = interface_->global_write_allocator()->cast_result(entry);
        // Need to jump to the main executor for two reasons:
        // . we need to constrain stack usage
        // . we need to access the node map which we can only do there.
        interface_->dispatcher()->executor()->Add(this);
    }

    virtual void Run()
    {
        NodeID id = srcNode_->node_id();
        LOG(VERBOSE, "Sending verified reply from node %012llx", id);
        f_->WriteGlobalMessage(If::MTI_VERIFIED_NODE_ID_NUMBER, id,
                               node_id_to_buffer(id), nullptr);
        // Continues the iteration over the nodes.
        if (it_ != interface_->localNodes_.end())
        {
            srcNode_ = it_->second;
            ++it_;
            return interface_->global_write_allocator()->AllocateEntry(this);
        }
        lock_.TypedRelease(this);
    }
#endif // not simple node

private:
    TypedAllocator<IncomingMessageHandler> lock_;
    AsyncIf* interface_;
    AsyncNode* srcNode_;

#ifndef SIMPLE_NODE_ONLY
    WriteFlow* f_;
    AsyncIf::VNodeMap::Iterator it_;
#endif
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfImpl_hxx_
