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
 * \file NodeInitializeFlow.cxx
 *
 * Default AsyncNode implementation for a fat virtual node.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#include "utils/constants.hxx"
#include "nmranet/If.hxx"
#include "nmranet/DefaultNode.hxx"

extern Service g_service;

DECLARE_CONST(node_init_identify);

namespace nmranet
{

struct InitializeRequest
{
    InitializeRequest() : node(nullptr)
    {
    }
    Node *node;
};

typedef StateFlow<Buffer<InitializeRequest>, QList<1>> InitializeFlowBase;

class InitializeFlow : public InitializeFlowBase
{
public:
    InitializeFlow(Service *service) : InitializeFlowBase(service)
    {
    }

private:
    Node *node()
    {
        return message()->data()->node;
    }

    Action entry() OVERRIDE
    {
        HASSERT(message()->data()->node);
        return allocate_and_call(
            node()->interface()->global_message_write_flow(),
            STATE(send_initialized));
    }

    Action send_initialized()
    {
        auto *b = get_allocation_result(
            node()->interface()->global_message_write_flow());
        done_.reset(this);
        NodeID id = node()->node_id();
        b->data()->reset(Defs::MTI_INITIALIZATION_COMPLETE, id,
                         node_id_to_buffer(id));
        b->data()->set_flag_dst(NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK);
        b->set_done(&done_);
        node()->interface()->global_message_write_flow()->send(
            b, b->data()->priority());
        return wait_and_call(STATE(initialization_complete));
    }

    Action initialization_complete()
    {
        node()->set_initialized();
        return call_immediately(STATE(identify_events));
    }

    Action identify_events()
    {
        if (!config_node_init_identify())
        {
            return release_and_exit();
        }
        // Get the dispatch flow.
        return allocate_and_call(node()->interface()->dispatcher(),
                                 STATE(initiate_local_identify));
    }

    Action initiate_local_identify()
    {
        auto *b = get_allocation_result(node()->interface()->dispatcher());
        b->set_done(done_.reset(this));
        NMRAnetMessage *m = b->data();
        m->mti = Defs::MTI_EVENTS_IDENTIFY_ADDRESSED;
        m->payload.clear();
        m->dst.id = node()->node_id();
        m->dstNode = node();
        m->src.alias = 0;
        m->src.id = node()->node_id();
        node()->interface()->dispatcher()->send(b, b->data()->priority());
        return wait_and_call(STATE(wait_for_local_identify));
    }

    Action wait_for_local_identify()
    {
        return release_and_exit();
    }

    BarrierNotifiable done_;
};

void StartInitializationFlow(Node *node)
{
    static InitializeFlow g_initialize_flow(&g_service);
    auto *b = g_initialize_flow.alloc();
    b->data()->node = node;
    g_initialize_flow.send(b);
}

} // namespace nmranet
