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

#include <endian.h>

#include "nmranet_config.h"
#include "openlcb/CanDefs.hxx"
#include "openlcb/Defs.hxx"

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
    node_->iface()->dispatcher()->register_handler(&streamInitiateHandler_,
        Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);
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
    // Initialize the last buffer for the first window.
    lastBufferPool_.alloc(&lastBuffer_);

    node_->iface()->dispatcher()->register_handler(
        &streamCompleteHandler_, Defs::MTI_STREAM_COMPLETE, Defs::MTI_EXACT);

    send_message(node_, Defs::MTI_STREAM_INITIATE_REPLY, message->data()->src,
        StreamDefs::create_initiate_response(
            streamWindowSize_, srcStreamId_, localStreamId_));

    node_->iface()->dispatcher()->unregister_handler(&streamInitiateHandler_,
        Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);
}

void StreamReceiverCan::handle_bytes_received(const uint8_t *data, size_t len)
{
    while (len > 0)
    {
        if (!currentBuffer_)
        {
            // Need to allocate a new chunk first.
            mainBufferPool->alloc(&currentBuffer_);
            // Add an empty raw buffer to it.
            RawBufferPtr rb;
            if (streamWindowRemaining_ <= RawData::MAX_SIZE)
            {
                // We need to use the last raw buffer.
                rb = std::move(lastBuffer_);
            }
            else
            {
                // We need a new (middle) raw buffer.
                rawBufferPool->alloc(&rb);
            }
            currentBuffer_->data()->set_from(std::move(rb), 0);
        }
        size_t copied = currentBuffer_->data()->append(data, len);
        data += copied;
        len -= copied;
        totalByteCount_ += copied;
        if (copied <= streamWindowRemaining_)
        {
            streamWindowRemaining_ -= copied;
        }
        else
        {
            LOG(WARNING, "Unexpected stream bytes, window is negative.");
            streamWindowRemaining_ = 0;
        }
        if (!currentBuffer_->data()->free_space() || !streamWindowRemaining_)
        {
            // Sends off the buffer and clears currentBuffer_.
            target_->send(currentBuffer_.release());
        }
    } // while len > 0
    if (!streamWindowRemaining_)
    {
        // wake up state flow to send ack to the stream
        notify();
    }
}

StateFlowBase::Action StreamReceiverCan::window_reached()
{
    return allocate_and_call<RawData>(
        nullptr, STATE(have_raw_buffer), &lastBufferPool_);
}

StateFlowBase::Action StreamReceiverCan::have_raw_buffer()
{
    lastBuffer_.reset(get_allocation_result<RawData>(nullptr));
    send_message(node_, Defs::MTI_STREAM_PROCEED, src_,
        StreamDefs::create_data_proceed(srcStreamId_, localStreamId_));
    return call_immediately(STATE(wait_for_wakeup));
}

void StreamReceiverCan::handle_stream_complete(Buffer<GenMessage> *message)
{
    auto rb = get_buffer_deleter(message);

    if (message->data()->dstNode != node_ ||
        !node_->iface()->matching_node(src_, message->data()->src))
    {
        LOG(INFO, "stream complete not for me");
        // Not for me.
        return;
    }

    if (message->data()->payload.size() < 2)
    {
        // Invalid arguments. Ignore.
        return;
    }

    if (message->data()->payload[0] != srcStreamId_ ||
        message->data()->payload[1] != localStreamId_)
    {
        // Different stream.
        LOG(INFO, "stream complete different stream");
        return;
    }

    uint32_t total_size = StreamDefs::INVALID_TOTAL_BYTE_COUNT;

    if (message->data()->payload.size() >= 5)
    {
        memcpy(&total_size, message->data()->payload.data() + 2, 4);
        total_size = be32toh(total_size);
    }

    streamClosed_ = true;

    if (total_size != StreamDefs::INVALID_TOTAL_BYTE_COUNT)
    {
        // We have to wait for the remaining bytes to show up.
        streamWindowRemaining_ = total_size - totalByteCount_;
    }
    else
    {
        streamWindowRemaining_ = 0;
        // wake up the flow.
        notify();
    }

    node_->iface()->dispatcher()->unregister_handler(
        &streamCompleteHandler_, Defs::MTI_STREAM_COMPLETE, Defs::MTI_EXACT);
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

        if (message->data()->can_dlc <= 0)
        {
            return; // no payload
        }
        if (message->data()->data[0] != parent_->localStreamId_)
        {
            return; // different stream
        }
        parent_->handle_bytes_received(
            message->data()->data + 1, message->data()->can_dlc - 1);
    }

private:
    /// Owning stream receiver object.
    StreamReceiverCan *parent_;
};

} // namespace openlcb
