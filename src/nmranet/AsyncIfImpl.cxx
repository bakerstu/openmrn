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
 * \file AsyncIfImpl.cxx
 *
 * Implementation details for the asynchronous NMRAnet interfaces. This file
 * should only be needed in hardware interface implementations.
 *
 * @author Balazs Racz
 * @date 4 Dec 2013
 */

#include "nmranet/AsyncIfImpl.hxx"

namespace NMRAnet
{

void WriteFlowBase::cleanup()
{
    if (data_)
    {
        data_->free();
    }
}

ControlFlow::ControlFlowAction WriteFlowBase::send_to_local_nodes()
{
    return Allocate(async_if()->dispatcher()->allocator(),
                    ST(unaddressed_with_local_dispatcher));
    ///@TODO(balazs.racz) actually send to local nodes.

    return send_to_hardware();
}

ControlFlow::ControlFlowAction WriteFlowBase::unaddressed_with_local_dispatcher()
{
    AsyncIf::MessageDispatchFlow* dispatcher;
    GetAllocationResult(&dispatcher);
    dispatcher->mutable_params()->mti = mti_;
    dispatcher->mutable_params()->src.id = src_;
    dispatcher->mutable_params()->src.alias = 0;
    dispatcher->mutable_params()->dst = dst_;  // should be 0,0.
    dispatcher->mutable_params()->dst_node = nullptr;

    dispatcher->mutable_params()->payload = nullptr;
    if (data_) {
        Buffer* copy = buffer_alloc(data_->used());
        memcpy(copy->start(), data_->start(), data_->used());
        copy->advance(data_->used());
        dispatcher->mutable_params()->payload = copy;
    }
    dispatcher->IncomingMessage(mti_);

    return send_to_hardware();
}

ControlFlow::ControlFlowAction WriteFlowBase::maybe_send_to_local_node()
{
    ///@TODO(balazs.racz) lookup local node and send.
    return send_to_hardware();
}

} // namespace NMRAnet
