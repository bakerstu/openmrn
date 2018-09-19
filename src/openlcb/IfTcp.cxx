/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file IfTcp.cxx
 *
 * OpenLCB interface implementation for native TCP connection.
 *
 * @author Balazs Racz
 * @date 16 Apr 2017
 */

#include "openlcb/IfTcp.hxx"
#include "openlcb/IfTcpImpl.hxx"
#include "openlcb/IfImpl.hxx"

namespace openlcb
{

void IfTcp::delete_local_node(Node *node)
{
    remove_local_node_from_map(node);
}

void IfTcp::add_owned_flow(Executable *e)
{
    ownedFlows_.emplace_back(e);
}

bool IfTcp::matching_node(NodeHandle expected, NodeHandle actual)
{
    if (expected.id && actual.id)
    {
        return expected.id == actual.id;
    }
    // Cannot reconcile.
    LOG(VERBOSE, "Cannot reconcile expected and actual NodeHandles for "
                 "equality testing.");
    return false;
}

IfTcp::IfTcp(NodeID gateway_node_id, HubFlow* device, int local_nodes_count)
    : If(device->service()->executor(), local_nodes_count)
{
    add_owned_flow(new VerifyNodeIdHandler(this));
    seq_ = new ClockBaseSequenceNumberGenerator;
    add_owned_flow(seq_);
    recvFlow_ = new TcpRecvFlow(dispatcher());
    add_owned_flow(recvFlow_);
    sendFlow_ = new TcpSendFlow(this, gateway_node_id, device, recvFlow_, seq_);
    add_owned_flow(sendFlow_);
    globalWriteFlow_ = sendFlow_;
    addressedWriteFlow_ = sendFlow_;
}

IfTcp::~IfTcp()
{
    device_->unregister_port(recvFlow_);
}

} // namespace openlcb
