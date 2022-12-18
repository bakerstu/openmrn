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
 * \file StreamReceiver.hxx
 *
 * Implementation flow for the Stream Service that receives data to a remote
 * source using the stream protocol.
 *
 * @author Balazs Racz
 * @date 3 May 2022
 */

#ifndef _OPENLCB_STREAMRECEIVER_HXX_
#define _OPENLCB_STREAMRECEIVER_HXX_

#include "openlcb/StreamReceiverInterface.hxx"

#include "openlcb/IfCan.hxx"
#include "openlcb/StreamDefs.hxx"
#include "utils/ByteBuffer.hxx"
#include "utils/LimitedPool.hxx"

namespace openlcb
{

class StreamReceiverCan : public CallableFlow<StreamReceiveRequest>
{
public:
    StreamReceiverCan(IfCan *interface)
        : CallableFlow<StreamReceiveRequest>(interface)
        , ifCan_(interface)
        , streamClosed_(0)
    { }

    /// Initializes the stream receiver and prepares for an announced
    /// stream. This is generally invoked by a handler of a higher level
    /// protocol where the stream connection is arranged, such as the Memory
    /// Config Protocol.
    ///
    /// This is a synchronous call. It is expected that shortly after this call
    /// a stream init message will arrive to the local interface, originating
    /// from the stream source node.
    ///
    /// @param src node handle of the source node that announced the stream.
    /// @param src_stream_id stream ID on the source node side. It is possible
    /// that this is not yet known at the time of this call, in which case
    /// INVALID_STREAM_ID may be passed in.
    /// @param dst_stream_id allocated stream ID at the local node. Must be a
    /// valid stream ID.
    /// @param max_window if non-zero, limits the maximum window size by the
    /// local side. If zero, the default max window size will be taken from a
    /// linker-time constant.
    ///
    void announced_stream(NodeHandle src, uint8_t src_stream_id,
        uint8_t dst_stream_id, uint16_t max_window = 0);

    /// Defines where to send the received stream data.
    ///
    /// @param target the business logic that will consume the data that arrived
    /// in the stream.
    /// @return the current object.
    StreamReceiverCan &set_sink(ByteSink *target)
    {
        target_ = target;
        return *this;
    }

private:
    Action start_stream() {
        return wait_for_wakeup();
    }

    Action wait_for_wakeup() {
        return wait_and_call(STATE(wait_for_wakeup));
    }
    
    Action wakeup() {
        // Check reason for wakeup.
        if (!streamWindowRemaining_) {
            // Need to send an ack.
            return call_immediately(STATE(window_reached));
        }
        return call_immediately(STATE(wait_for_wakeup));
    }

    /// Invoked when the stream window runs out. Maybe waits for the data to be
    /// consumed below the low-watermark.
    Action window_reached();
    /// Called when the allocation of the raw buffer is successful. Sends off
    /// the stream proceed message.
    Action have_raw_buffer();
    
    /// Invoked by the GenericHandler when a stream initiate message arrives.
    ///
    /// @param message buffer with stream initiate message.
    ///
    void handle_stream_initiate(Buffer<GenMessage> *message);

    /// Handles data arriving from the network.
    inline void handle_bytes_received(const uint8_t *data, size_t len);

    /// Invoked by the GenericHandler when a stream complete message arrives.
    ///
    /// @param message buffer with stream complete message.
    ///
    void handle_stream_complete(Buffer<GenMessage> *message);

    /// @return the local CAN interface.
    IfCan *if_can()
    {
        return ifCan_;
    }

    /// @return the local node pointer.
    Node *node()
    {
        return request()->dst_;
    }

    /// helper class for incoming message for stream initiate.
    MessageHandler::GenericHandler streamInitiateHandler_ {
        this, &StreamReceiverCan::handle_stream_initiate};

    class StreamDataHandler;
    friend class StreamDataHandler;

    /// helper class for incoming message for stream initiate.
    MessageHandler::GenericHandler streamCompleteHandler_ {
        this, &StreamReceiverCan::handle_stream_complete};
    
    /// CAN-bus interface.
    IfCan *ifCan_;

    /// This pool is used to allocate one raw buffer per stream window
    /// size. This pool therefore functions as a throttling for the data
    /// producer. We have a fixed size of 2, meaning that we are allowing
    /// ourselves to load 2x the stream window size into our RAM.
    LimitedPool lastBufferPool_ {sizeof(RawBuffer), 2, rawBufferPool};

    /// The buffer that we are currently filling with incoming data.
    ByteBufferPtr currentBuffer_;

    /// The buffer that will be the last one in this stream window. This buffer
    /// comes from the lastBufferPool_ to function as throttling signal.
    RawBufferPtr lastBuffer_;

    /// Helper object that receives the actual stream CAN frames.
    std::unique_ptr<StreamDataHandler> dataHandler_;

    /// Where to send the actually received data.
    ByteSink *target_ {nullptr};

    /// Source node that the data is coming from.
    NodeHandle src_;
    /// How many bytes we have transmitted in this stream so far.
    size_t totalByteCount_;

    /// Total stream window size. @todo fill in
    uint16_t streamWindowSize_;
    /// Remaining stream window size. @todo fill in
    uint16_t streamWindowRemaining_;

    /// Stream ID at the source node. @todo fill in
    uint8_t srcStreamId_;
    /// Stream ID at the destination (local) node. @todo fill in
    uint8_t localStreamId_;

    /// 1 if we received the stream complete message.
    uint8_t streamClosed_ : 1;

}; // class StreamReceiver

} // namespace openlcb

#endif // _OPENLCB_STREAMRECEIVER_HXX_
