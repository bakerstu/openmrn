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

#include "openlcb/StreamDefs.hxx"

namespace openlcb
{

class StreamReceiverCan
{
public:
    StreamReceiverCan(Service *service, IfCan *interface, Node *node)
        : ifCan_(interface)
        , node_(node)
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

private:
    /// Invoked by the GenericHandler when a stream initiate message arrives.
    ///
    /// @param message buffer with stream initiate message.
    ///
    void handle_stream_initiate(Buffer<GenMessage> *message);

    /// helper class for incoming message for stream initiate.
    MessageHandler::GenericHandler streamInitiateHandler_ {
        this, &StreamReceiverCan::handle_stream_initiate};

    /// CAN-bus interface.
    IfCan *ifCan_;
    /// Which node are we sending the outgoing data from. This is a local
    /// virtual node.
    Node *node_;

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

}; // class StreamReceiver

} // namespace openlcb

#endif // _OPENLCB_STREAMRECEIVER_HXX_
