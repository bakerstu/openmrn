/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file StreamReceiverInterface.hxx
 *
 * Transport-agnostic interface for receiving a stream from OpenLCB remote.
 *
 * @author Balazs Racz
 * @date 18 Dec 2022
 */

#ifndef _OPENLCB_STREAMRECEIVERINTERFACE_HXX_
#define _OPENLCB_STREAMRECEIVERINTERFACE_HXX_

#include "executor/CallableFlow.hxx"

namespace openlcb
{

struct StreamReceiveRequest : public CallableFlowRequestBase
{
    /// Gets a free local stream ID.
    void reset();

    /// Initiates a stream receive operation.
    void reset(ByteSink *target, Node* dst, NodeHandle src, uint8_t src_stream_id,
        uint8_t dst_stream_id, uint16_t max_window = 0)
    {
        reset_base();
        target_ = target;
        src_ = src;
        dst_ = dst;
        srcStreamId_ = src_stream_id;
        localStreamId_ = dst_stream_id;
        maxWindowSize_ = max_window;
    }

    /// Where to send the incoming stream data.
    ByteSink *target_;
    /// Remote node that will send us the stream.
    NodeHandle src_;
    /// Local node for receiving the stream.
    Node* dst_;
    /// Source (remote) stream ID. May be INVALID_STREAM_ID.
    uint8_t srcStreamId_;
    /// Local (target) stream ID. Must be valid.
    uint8_t localStreamId_;
    /// if non-zero, limits the maximum window size by the
    /// local side. If zero, the default max window size will be taken from a
    /// linker-time constant.
    uint16_t maxWindowSize_;
};

using StreamReceiverInterface = FlowInterface<Buffer<StreamReceiveRequest>>;

} // namespace openlcb

#endif // _OPENLCB_STREAMRECEIVERINTERFACE_HXX_
