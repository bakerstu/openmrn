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
#include "openlcb/Defs.hxx"
#include "openlcb/StreamDefs.hxx"

template <class T> class FlowInterface;
template <class T> class Buffer;
struct ByteChunk;
using ByteBuffer = Buffer<ByteChunk>;
using ByteSink = FlowInterface<ByteBuffer>;

namespace openlcb
{

class Node;

struct StreamReceiveRequest : public CallableFlowRequestBase
{
    enum
    {
        /// This bit in the resultCode is cleared when done is called.
        OPERATION_PENDING = 0x20000,
        /// The operation was canceled by the caller using `cancel_request()`
        ERROR_CANCELED = Defs::ERROR_OUT_OF_ORDER | 1,
    };

    /// Gets a local stream ID. This will be returning the assigned local
    /// stream ID from the stream receiver object.
    void reset()
    {
        reset_base();
        target_ = nullptr;
        localStreamId_ = StreamDefs::INVALID_STREAM_ID;
        resultCode = OPERATION_PENDING;
    }

    /// Starts the stream receiver and prepares for an announced stream. This
    /// is generally invoked by a handler of a higher level protocol where the
    /// stream connection is arranged, such as the Memory Config Protocol.
    ///
    /// This call is processed synchronously. It is expected that shortly after
    /// this call a stream init message will arrive to the local interface,
    /// originating from the stream source node.
    ///
    /// @param src node handle of the source node that announced the stream.
    /// @param src_stream_id stream ID on the source node side. It is possible
    /// that this is not yet known at the time of this call, in which case
    /// INVALID_STREAM_ID may be passed in.
    /// @param dst_stream_id allocated stream ID at the local node. If it is
    /// INVALID_STREAM_ID, then the assigned local ID is used by the stream
    /// receiver.
    /// @param max_window if non-zero, limits the maximum window size by the
    /// local side. If zero, the default max window size will be taken from a
    /// linker-time constant.
    void reset(ByteSink *target, Node *dst, NodeHandle src,
        uint8_t src_stream_id = StreamDefs::INVALID_STREAM_ID,
        uint8_t dst_stream_id = StreamDefs::INVALID_STREAM_ID,
        uint16_t max_window = 0)
    {
        reset_base();
        HASSERT(target);
        target_ = target;
        src_ = src;
        dst_ = dst;
        srcStreamId_ = src_stream_id;
        localStreamId_ = dst_stream_id;
        streamWindowSize_ = max_window;
        resultCode = OPERATION_PENDING;
    }

    /// Where to send the incoming stream data.
    ByteSink *target_ {nullptr};
    /// Remote node that will send us the stream.
    NodeHandle src_ {0, 0};
    /// Local node for receiving the stream.
    Node *dst_ {nullptr};
    /// Source (remote) stream ID. May be INVALID_STREAM_ID.
    uint8_t srcStreamId_ {StreamDefs::INVALID_STREAM_ID};
    /// Local (target) stream ID. Must be valid.
    uint8_t localStreamId_ {StreamDefs::INVALID_STREAM_ID};
    /// if non-zero, limits the maximum window size by the
    /// local side. If zero, the default max window size will be taken from a
    /// linker-time constant.
    uint16_t streamWindowSize_ {0};
};

class StreamReceiverInterface : public CallableFlow<StreamReceiveRequest>
{
public:
    StreamReceiverInterface(Service *s)
        : CallableFlow<StreamReceiveRequest>(s)
    { }

    /// Cancels the currently pending stream receive request. The message will
    /// then be asynchronously returned using the regular mechanism with a
    /// temporary error.
    virtual void cancel_request() = 0;
};

} // namespace openlcb

#endif // _OPENLCB_STREAMRECEIVERINTERFACE_HXX_
