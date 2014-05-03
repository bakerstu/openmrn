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

StateFlowBase::Action WriteFlowBase::addressed_entry()
{
    if (nmsg()->dst.id)
    {
        nmsg()->dstNode = async_if()->lookup_local_node(nmsg()->dst.id);
        if (nmsg()->dstNode)
        {
            async_if()->dispatcher()->send(transfer_message(), priority());
            return call_immediately(STATE(send_finished));
        }
    }
    return send_to_hardware();
}

StateFlowBase::Action WriteFlowBase::global_entry()
{
    if (!message()->data()->has_flag_dst(
            NMRAnetMessage::WAIT_FOR_LOCAL_LOOPBACK))
    {
        // We do not pass on the done notifiable with the loopbacked message.
        BarrierNotifiable *d = message()->new_child();
        if (d)
        {
            // This is abuse of the barriernotifiable code, because we assume
            // that notifying the child twice will cause the parent to be
            // notified once.
            d->notify();
            d->notify();
            message()->set_done(nullptr);
        }
    }
    async_if()->dispatcher()->send(transfer_message());
    return release_and_exit();
}

} // namespace NMRAnet
