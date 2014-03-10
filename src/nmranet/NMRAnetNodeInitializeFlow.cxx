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
 * \file NMRAnetAsyncDefaultNode.hxx
 *
 * Default AsyncNode implementation for a fat virtual node.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#include "executor/control_flow.hxx"
#include "nmranet/AsyncIf.hxx"
#include "nmranet/NMRAnetAsyncDefaultNode.hxx"

namespace NMRAnet
{

namespace
{
class InitializeFlow : public ControlFlow
{
public:
    InitializeFlow(DefaultAsyncNode* node)
        : ControlFlow(node->interface()->dispatcher()->executor(), nullptr),
          node_(node)
    {
        StartFlowAt(ST(handle_start));
    }

private:
    ControlFlowAction handle_start()
    {
        return Allocate(node_->interface()->global_write_allocator(),
                        ST(send_initialized));
    }

    ControlFlowAction send_initialized()
    {
        WriteFlow* flow = GetTypedAllocationResult(
            node_->interface()->global_write_allocator());
        NodeID id = node_->node_id();
        flow->WriteGlobalMessage(If::MTI_INITIALIZATION_COMPLETE, id,
                                 node_id_to_buffer(id), this);
        return WaitAndCall(ST(initialization_complete));
    }

    ControlFlowAction initialization_complete()
    {
        node_->set_initialized();
        return CallImmediately(ST(identify_events));
    }

    ControlFlowAction identify_events()
    {
        // Get the dispatch flow.
        return Allocate(node_->interface()->dispatcher()->allocator(),
                        ST(initiate_local_identify));
    }

    ControlFlowAction initiate_local_identify()
    {
        auto* f = GetTypedAllocationResult(
            node_->interface()->dispatcher()->allocator());
        IncomingMessage* m = f->mutable_params();
        m->mti = If::MTI_EVENTS_IDENTIFY_ADDRESSED;
        m->payload = nullptr;
        m->dst.id = node_->node_id();
        m->dst_node = node_;
        m->src.alias = 0;
        m->src.id = node_->node_id();
        f->IncomingMessage(m->mti);
        return Exit(); // will delete *this.
    }

    DefaultAsyncNode* node_;
};

} // namespace

void StartInitializationFlow(DefaultAsyncNode* node)
{
    new InitializeFlow(node);
}

} // namespace NMRAnet
