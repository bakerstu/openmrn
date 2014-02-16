/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file NMRAnetAsyncDatagramDefaultHandler.hxx
 *
 * Control flow for incoming datagram handlers.
 *
 * @author Balazs Racz
 * @date 11 Feb 2014
 */

#ifndef _NMRAnetAsyncDatagramDefaultHandler_hxx_
#define _NMRAnetAsyncDatagramDefaultHandler_hxx_

#include "nmranet/NRMAnetAsyncDatagram.hxx"

namespace NMRAnet
{

class DefaultDatagramHandler : public DatagramHandler, public ControlFlow
{
public:
    DefaultDatagramHandler(Executor* executor) : ControlFlow(executor, nullptr)
    {
        StartFlowAt(ST(wait_for_datagram));
    }

    ControlFlowAction wait_for_datagram()
    {
        return Allocate(&queue_, ST(datagram_arrived_stub));
    }

    ControlFlowAction datagram_arrived_stub()
    {
        datagram_ = GetTypedAllocationResult(&queue_);
        return CallImmediately(ST(datagram_arrived));
    }

    ControlFlowAction respond_ok(uint8_t flags) {
        responseMti_ = If::MTI_DATAGRAM_OK;
        responseErrorCode_ = flags;
        return Allocate()
        return CallImmediately(ST(wait_for_datagram));
    }

    ControlFlowAction respond_reject(uint16_t error_code) {
        datagram_->free();
        return CallImmediately(ST(wait_for_datagram));
    }

    /** Handles a new incoming datagram. Implementation must eventually call
     * respond_ok or respond_reject. */
    virtual ControlFlowAction datagram_arrived() = 0;

protected:
    /// The current datagram being processed.
    IncomingDatagram* datagram_;

private:
    uint16_t responseErrorCode_;
    uint16_t responseMti_;
};

} // namespace

#endif // _NMRAnetAsyncDatagramDefaultHandler_hxx_
