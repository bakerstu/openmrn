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
#include "utils/format_utils.hxx"

namespace openlcb
{

void StreamReceiverCan::announced_stream()
{
    // Resets state bits.
    streamClosed_ = 0;
    pendingInit_ = 0;
    pendingCancel_ = 0;
    isWaiting_ = 0;

    if (!request()->streamWindowSize_)
    {
        request()->streamWindowSize_ =
            config_stream_receiver_default_window_size();
    }
    streamWindowRemaining_ = 0;
    node()->iface()->dispatcher()->register_handler(&streamInitiateHandler_,
        Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);
}

void StreamReceiverCan::send(Buffer<StreamReceiveRequest> *msg, unsigned prio)
{
    reset_message(msg, prio);

    if (request()->localStreamId_ == StreamDefs::INVALID_STREAM_ID)
    {
        request()->localStreamId_ = assignedStreamId_;
    }

    if (!request()->target_)
    {
        // asking for stream ID.
        request()->localStreamId_ = assignedStreamId_;
        return_buffer();
        return;
    }

    announced_stream();
    wait_for_wakeup();
}

void StreamReceiverCan::handle_stream_initiate(Buffer<GenMessage> *message)
{
    auto rb = get_buffer_deleter(message);

    if (message->data()->dstNode != node() ||
        !node()->iface()->matching_node(request()->src_, message->data()->src))
    {
        LOG(INFO, "stream init not for me");
        // Not for me.
        return;
    }
    // Saves alias as well.
    request()->src_ = message->data()->src;
    const auto &payload = message->data()->payload;
    uint16_t proposed_window;
    uint8_t incoming_src_id = StreamDefs::INVALID_STREAM_ID;
    if (payload.size() >= 5)
    {
        incoming_src_id = payload[4];
        if (request()->srcStreamId_ != StreamDefs::INVALID_STREAM_ID &&
            request()->srcStreamId_ != incoming_src_id)
        {
            LOG(INFO, "stream init ID not for me");
            // Not for me.
            return;
        }
        request()->srcStreamId_ = incoming_src_id;
    }
    if (payload.size() < 5 ||
        incoming_src_id == StreamDefs::INVALID_STREAM_ID ||
        ((proposed_window = data_to_error(&payload[0])) == 0))
    {
        LOG(INFO, "Incoming stream: invalid arguments.");
        // Invalid arguments. This will synchronously allocate a buffer and
        // send the message to the interface.
        send_message(node(), Defs::MTI_STREAM_INITIATE_REPLY,
            message->data()->src,
            StreamDefs::create_initiate_response(0, incoming_src_id,
                request()->localStreamId_,
                StreamDefs::STREAM_ERROR_INVALID_ARGS));
        node()->iface()->dispatcher()->unregister_handler(
            &streamInitiateHandler_, Defs::MTI_STREAM_INITIATE_REQUEST,
            Defs::MTI_EXACT);
        return;
    }
    if (proposed_window < request()->streamWindowSize_)
    {
        request()->streamWindowSize_ = proposed_window;
    }

    streamWindowRemaining_ = request()->streamWindowSize_;
    totalByteCount_ = 0;

    node()->iface()->dispatcher()->register_handler(
        &streamCompleteHandler_, Defs::MTI_STREAM_COMPLETE, Defs::MTI_EXACT);

    node()->iface()->dispatcher()->unregister_handler(&streamInitiateHandler_,
        Defs::MTI_STREAM_INITIATE_REQUEST, Defs::MTI_EXACT);

    pendingInit_ = 1;
    notify();
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
            request()->target_->send(currentBuffer_.release());
        }
    } // while len > 0
    if (!streamWindowRemaining_)
    {
        // wake up state flow to send ack to the stream
        notify();
    }
}

void StreamReceiverCan::handle_stream_complete(Buffer<GenMessage> *message)
{
    auto rb = get_buffer_deleter(message);

    if (message->data()->dstNode != node() ||
        !node()->iface()->matching_node(request()->src_, message->data()->src))
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

    if (((uint8_t)message->data()->payload[0]) != request()->srcStreamId_ ||
        ((uint8_t)message->data()->payload[1]) != request()->localStreamId_)
    {
        // Different stream.
        LOG(INFO, "stream complete different stream");
        return;
    }

    uint32_t total_size = StreamDefs::INVALID_TOTAL_BYTE_COUNT;

    if (message->data()->payload.size() >= 6)
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
    }

    if (!streamWindowRemaining_)
    {
        // wake up the flow.
        notify();
    }

    node()->iface()->dispatcher()->unregister_handler(
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
        HASSERT(remote_alias);
        HASSERT(local_alias);
        uint32_t frame_id = 0;
        CanDefs::set_datagram_fields(
            &frame_id, remote_alias, local_alias, CanDefs::STREAM_DATA);
        LOG(VERBOSE, "register frame ID %x", (unsigned)frame_id);
        parent_->if_can()->frame_dispatcher()->register_handler(
            this, frame_id, CanDefs::STREAM_DG_RECV_MASK);
    }

    /// Stops receiving stream data.
    void stop()
    {
        parent_->if_can()->frame_dispatcher()->unregister_handler_all(this);
    }

    /// Handler callback for incoming messages.
    void send(Buffer<CanMessageData> *message, unsigned priority) override
    {
        auto rb = get_buffer_deleter(message);

        if (message->data()->can_dlc <= 0)
        {
            return; // no payload
        }
        if (message->data()->data[0] != parent_->request()->localStreamId_)
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

StreamReceiverCan::StreamReceiverCan(IfCan *interface, uint8_t local_stream_id)
    : StreamReceiverInterface(interface)
    , dataHandler_(new StreamDataHandler(this))
    , assignedStreamId_(local_stream_id)
    , streamClosed_(0)
    , pendingInit_(0)
    , pendingCancel_(0)
    , isWaiting_(0)
{ }

StreamReceiverCan::~StreamReceiverCan()
{ }

void StreamReceiverCan::cancel_request()
{
    pendingCancel_ = 1;
    if (isWaiting_)
    {
        isWaiting_ = 0;
        notify();
    }
}

void StreamReceiverCan::unregister_handlers()
{
    dataHandler_->stop();
    node()->iface()->dispatcher()->unregister_handler_all(
        &streamInitiateHandler_);
    node()->iface()->dispatcher()->unregister_handler_all(
        &streamCompleteHandler_);
}

StateFlowBase::Action StreamReceiverCan::wakeup()
{
    isWaiting_ = 0;
    // Checks reason for wakeup.
    if (pendingCancel_)
    {
        unregister_handlers();
        if (currentBuffer_)
        {
            // Sends off the buffer and clears currentBuffer_.
            request()->target_->send(currentBuffer_.release());
        }
        return return_with_error(StreamReceiveRequest::ERROR_CANCELED);
    }
    if (pendingInit_)
    {
        pendingInit_ = 0;
        return call_immediately(STATE(init_reply));
    }
    if (!streamWindowRemaining_)
    {
        if (streamClosed_)
        {
            streamClosed_ = 0;
            dataHandler_->stop();
            if (currentBuffer_)
            {
                // Sends off the buffer and clears currentBuffer_.
                request()->target_->send(currentBuffer_.release());
            }
            return return_ok();
        }
        // Need to send an ack.
        return call_immediately(STATE(window_reached));
    }
    return wait();
}

StateFlowBase::Action StreamReceiverCan::init_reply()
{
    // Initialize the last buffer for the first window.
    return allocate_and_call<RawData>(
        nullptr, STATE(init_buffer_ready), &lastBufferPool_);
}

StateFlowBase::Action StreamReceiverCan::init_buffer_ready()
{
    lastBuffer_.reset(get_allocation_result<RawData>(nullptr));

    node()->iface()->canonicalize_handle(&request()->src_);
    NodeHandle local(node()->node_id());
    node()->iface()->canonicalize_handle(&local);
    dataHandler_->start(request()->src_.alias, local.alias);

    send_message(node(), Defs::MTI_STREAM_INITIATE_REPLY, request()->src_,
        StreamDefs::create_initiate_response(request()->streamWindowSize_,
            request()->srcStreamId_, request()->localStreamId_));

    return wait_for_wakeup();
}

StateFlowBase::Action StreamReceiverCan::window_reached()
{
    return allocate_and_call<RawData>(
        nullptr, STATE(have_raw_buffer), &lastBufferPool_);
}

StateFlowBase::Action StreamReceiverCan::have_raw_buffer()
{
    lastBuffer_.reset(get_allocation_result<RawData>(nullptr));
    streamWindowRemaining_ = request()->streamWindowSize_;
    send_message(node(), Defs::MTI_STREAM_PROCEED, request()->src_,
        StreamDefs::create_data_proceed(
            request()->srcStreamId_, request()->localStreamId_));
    return wait_for_wakeup();
}

} // namespace openlcb
