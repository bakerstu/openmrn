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

#include "nmranet/NMRAnetAsyncDatagram.hxx"

namespace nmranet
{

class DefaultDatagramHandler : public DatagramHandler, public ControlFlow
{
public:
    DefaultDatagramHandler(DatagramSupport* if_datagram)
        : ControlFlow(if_datagram->interface()->dispatcher()->executor(), nullptr),
          ifDatagram_(if_datagram)
    {
        StartFlowAt(STATE(wait_for_datagram));
    }

    Action wait_for_datagram()
    {
        return Allocate(&queue_, ST(datagram_arrived_stub));
    }

    Action datagram_arrived_stub()
    {
        datagram_ = GetTypedAllocationResult(&queue_);
        return call_immediately(STATE(datagram_arrived));
    }

    /** Sends a DATAGRAM_OK response to the datagram originator node. Call this
     * from the user handler. The flow will end up in the ok_response_sent()
     * state.
     *
     * @param flags is the 1-byte payload of the DATAGRAM_OK message.*/
    Action respond_ok(uint8_t flags)
    {
        responseMti_ = Defs::MTI_DATAGRAM_OK;
        responseErrorCode_ = flags;
        return Allocate(ifDatagram_->interface()->addressed_write_allocator(),
                        ST(send_ok_response));
    }

    Action send_ok_response()
    {
        auto* flow = GetTypedAllocationResult(
            ifDatagram_->interface()->addressed_write_allocator());
        Buffer* payload = buffer_alloc(1);
        uint8_t* data = static_cast<uint8_t*>(payload->start());
        data[0] = responseErrorCode_ & 0xff;
        payload->advance(1);
        flow->WriteAddressedMessage(responseMti_, datagram_->dst->node_id(),
                                    datagram_->src, payload, nullptr);
        return call_immediately(STATE(ok_response_sent));
    }

    /** Sends a DATAGRAM_OK response to the datagram originator node. Call this
     * from the user handler. The flow will return to the
     *
     * @param flags is the 1-byte payload of the DATAGRAM_OK message.*/
    Action respond_reject(uint16_t error_code)
    {
        responseMti_ = Defs::MTI_DATAGRAM_REJECTED;
        responseErrorCode_ = error_code;
        return Allocate(ifDatagram_->interface()->addressed_write_allocator(),
                        ST(send_reject_response));
    }

    Action send_reject_response()
    {
        auto* flow = GetTypedAllocationResult(
            ifDatagram_->interface()->addressed_write_allocator());
        Buffer* payload = buffer_alloc(2);
        uint8_t* data = static_cast<uint8_t*>(payload->start());
        data[0] = (responseErrorCode_ >> 8) & 0xff;
        data[1] = responseErrorCode_ & 0xff;
        payload->advance(2);
        flow->WriteAddressedMessage(responseMti_, datagram_->dst->node_id(),
                                    datagram_->src, payload, nullptr);
        datagram_->free();
        return call_immediately(STATE(wait_for_datagram));
    }

    /** Handles a new incoming datagram. Implementation must eventually call
     * respond_ok or respond_reject. */
    virtual Action datagram_arrived() = 0;

    /** This state is where the handling will end up after a respond_ok
     * call. The user is responsible for free-ing the datagram object and
     * returning to the wait_for_datagram state. */
    virtual Action ok_response_sent()
    {
        datagram_->free();
        return call_immediately(STATE(wait_for_datagram));
    }

protected:
    /// The current datagram being processed.
    IncomingDatagram* datagram_;

    /// The interface's datagram support code.
    DatagramSupport* ifDatagram_;

private:
    uint16_t responseErrorCode_;
    nmranet::Defs::MTI responseMti_;
};

} // namespace

#endif // _NMRAnetAsyncDatagramDefaultHandler_hxx_
