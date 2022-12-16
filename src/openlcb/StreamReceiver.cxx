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
 * \file StreamReceiver.cxx
 *
 * Implementation flow for the Stream Service that receives data to a remote
 * source using the stream protocol.
 *
 * @author Balazs Racz
 * @date 3 May 2022
 */

#include "openlcb/StreamReceiver.hxx"

#include "openlcb/Defs.hxx"
#include "openlcb/CanDefs.hxx"
#include "nmranet_config.h"

namespace openlcb
{

void StreamReceiverCan::announced_stream(NodeHandle src, uint8_t src_stream_id,
    uint8_t dst_stream_id, uint16_t max_window)
{
    src_ = src;
    node_->iface()->canonicalize_handle(&src_);
    srcStreamId_ = src_stream_id;
    localStreamId_ = dst_stream_id;
    if (!max_window)
    {
        streamWindowSize_ = config_stream_receiver_default_window_size();
    }
    else
    {
        streamWindowSize_ = max_window;
    }
    streamWindowRemaining_ = 0;
    node_->iface()->dispatcher()->register_handler(
        &streamInitiateHandler_, Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);
}

void StreamReceiverCan::handle_stream_initiate(Buffer<GenMessage> *message)
{
    auto rb = get_buffer_deleter(message);

    if (message->data()->dstNode != node_ ||
        !node_->iface()->matching_node(src_, message->data()->src))
    {
        LOG(INFO, "stream init not for me");
        // Not for me.
        return;
    }
    const auto &payload = message->data()->payload;
    uint16_t proposed_window;
    uint8_t incoming_src_id = StreamDefs::INVALID_STREAM_ID;
    if (payload.size() >= 5)
    {
        incoming_src_id = payload[4];
        if (srcStreamId_ != StreamDefs::INVALID_STREAM_ID &&
            srcStreamId_ != incoming_src_id)
        {
            LOG(INFO, "stream init ID not for me");
            // Not for me.
            return;
        }
        srcStreamId_ = incoming_src_id;
    }
    if (payload.size() < 5 ||
        incoming_src_id == StreamDefs::INVALID_STREAM_ID ||
        ((proposed_window = data_to_error(&payload[0])) == 0))
    {
        // Invalid arguments. This will synchronously allocate a buffer and
        // send the message to the interface.
        send_message(node_, Defs::MTI_STREAM_INITIATE_REPLY,
            message->data()->src,
            StreamDefs::create_initiate_response(0, incoming_src_id,
                localStreamId_, StreamDefs::STREAM_ERROR_INVALID_ARGS));
        node_->iface()->dispatcher()->unregister_handler(
            &streamInitiateHandler_, Defs::MTI_STREAM_INITIATE_REQUEST,
            Defs::MTI_EXACT);
        return;
    }
    if (proposed_window < streamWindowSize_)
    {
        streamWindowSize_ = proposed_window;
    }

    streamWindowRemaining_ = streamWindowSize_;
    send_message(node_, Defs::MTI_STREAM_INITIATE_REPLY, message->data()->src,
        StreamDefs::create_initiate_response(
            streamWindowSize_, srcStreamId_, localStreamId_));

    node_->iface()->dispatcher()->unregister_handler(
        &streamInitiateHandler_, Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);
}

class StreamReceiverCan::StreamDataHandler : public IncomingFrameHandler
{
public:
    StreamDataHandler(StreamReceiverCan *parent)
        : parent_(parent)
    { }

    /// Starts registration for receiving stream data with the given aliases.
    void start(NodeAlias remote_alias, NodeAlias local_alias)
    {
        uint32_t frame_id = 0;
        CanDefs::set_datagram_fields(
            &frame_id, remote_alias, local_alias, CanDefs::STREAM_DATA);
        parent_->ifCan_->frame_dispatcher()->register_handler(
            this, frame_id, CanDefs::STREAM_DG_RECV_MASK);
    }

    /// Stops receiving stream data.
    void stop()
    {
        parent_->ifCan_->frame_dispatcher()->unregister_handler_all(this);
    }

    /// Handler callback for incoming messages.
    void send(Buffer<CanMessageData> *message, unsigned priority) override
    {
        auto rb = get_buffer_deleter(message);

        if (message->data()->can_dlc <= 0) {
            return; // no payload
        }
        if (message->data()->data[0] != parent_->localStreamId_) {
            return; // different stream
        }
        
    }

private:
    /// Owning stream receiver object.
    StreamReceiverCan *parent_;
};

} // namespace openlcb
