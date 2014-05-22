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

#include "nmranet/If.hxx"
#include "nmranet/DefaultNode.hxx"

namespace nmranet
{

namespace
{
class InitializeFlow : public StateFlowBase
{
public:
    InitializeFlow(Node *node)
        : StateFlowBase(node->interface()->dispatcher()->service())
        , node_(node)
    {
        HASSERT(node);
        start_flow(STATE(handle_start));
    }

private:
    Action handle_start()
    {
        return allocate_and_call(
            node_->interface()->global_message_write_flow(),
            STATE(send_initialized));
    }

    Action send_initialized()
    {
        auto* b = get_allocation_result(node_->interface()->global_message_write_flow());
        done_.reset(this);
        NodeID id = node_->node_id();
        b->data()->reset(Defs::MTI_INITIALIZATION_COMPLETE, id,
                         node_id_to_buffer(id));
        b->set_done(&done_);
        node_->interface()->global_message_write_flow()->send(b);
        return wait_and_call(STATE(initialization_complete));
    }

    Action initialization_complete()
    {
        node_->set_initialized();
        return call_immediately(STATE(identify_events));
    }

    Action identify_events()
    {
        // Get the dispatch flow.
        return allocate_and_call(node_->interface()->dispatcher(),
                                 STATE(initiate_local_identify));
    }

    Action initiate_local_identify()
    {
        auto* b = get_allocation_result(node_->interface()->dispatcher());
        NMRAnetMessage *m = b->data();
        m->mti = Defs::MTI_EVENTS_IDENTIFY_ADDRESSED;
        m->payload.clear();
        m->dst.id = node_->node_id();
        m->dstNode = node_;
        m->src.alias = 0;
        m->src.id = node_->node_id();
        node_->interface()->dispatcher()->send(b);

        return delete_this();
    }

    Node *node_;
    BarrierNotifiable done_;
};

} // namespace

void StartInitializationFlow(Node *node)
{
    new InitializeFlow(node);
}

} // namespace nmranet
