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
 * \file StreamSender.hxx
 *
 * Implementation flow for the Stream Service that sends data to a remote
 * destination using the stream protocol.
 *
 * @author Balazs Racz
 * @date 17 Apr 2022
 */

#ifndef _OPENLCB_STREAMSENDER_HXX_
#define _OPENLCB_STREAMSENDER_HXX_

#include "executor/StateFlow.hxx"
#include "openlcb/CanDefs.hxx"
#include "openlcb/DatagramDefs.hxx"
#include "openlcb/IfCan.hxx"
#include "openlcb/StreamDefs.hxx"
#include "utils/ByteBuffer.hxx"
#include "utils/LimitedPool.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

class StreamSender : public StateFlow<ByteBuffer, QList<1>>
{
public:
    StreamSender(Service *s)
        : StateFlow<ByteBuffer, QList<1>>(s)
    {
    }

    /// Describes the different states in the stream sender.
    enum StreamSenderState : uint8_t
    {
        /// This stream sender is not in use now
        IDLE,
        /// The local client has started using the stream sender (via API).
        STARTED,
        /// The stream initiate message was sent.
        INITIATING,
        /// Stream is open and data can be transferred.
        RUNNING,
        /// Stream buffer is full, waiting for proceed message.
        FULL,
        /// Stream close message was sent.
        CLOSING,
        /// An error occurred.
        STATE_ERROR
    };
};

/// Helper class for sending stream data to a CAN interface.
/// @todo add progress report API.
class StreamSenderCan : public StreamSender
{
public:
    StreamSenderCan(Service *service, IfCan *iface)
        : StreamSender(service)
        , ifCan_(iface)
        , sleeping_(false)
        , requestClose_(false)
        , requestInit_(false)
    {
    }

    /// Initiates using the stream sender. May be called only on idle stream
    /// senders.
    ///
    /// @param src Source virtual node on the local interface.
    /// @param dst Destination node ID to send the stream to.
    /// @param source_stream_id 8-bit stream ID to use on the this (the source)
    /// side.
    /// @param dst_stream_id 8-bit stream ID to use on the remote side (the
    /// destination).
    ///
    /// @return *this for calling optional settings API commands.
    ///
    StreamSenderCan &start_stream(Node *src, NodeHandle dst,
        uint8_t source_stream_id,
        uint8_t dst_stream_id = StreamDefs::INVALID_STREAM_ID)
    {
        DASSERT(state_ == IDLE);
        state_ = STARTED;
        node_ = src;
        dst_ = dst;
        totalByteCount_ = 0;
        localStreamId_ = source_stream_id;
        dstStreamId_ = dst_stream_id;
        HASSERT(sleeping_ == false);
        HASSERT(requestClose_ == 0);
        requestInit_ = true;
        trigger();
        streamFlags_ = 0;
        streamAdditionalFlags_ = 0;
        streamWindowSize_ = StreamDefs::MAX_PAYLOAD;
        streamWindowRemaining_ = 0;
        errorCode_ = 0;
        return *this;
    }

    /// Closes the stream when all the bytes are transferred.
    /// @param error_code 0 upon success. This code is intended to be
    /// transferred in the stream close message, but that is not yet
    /// implemented, because the draft protocol does not have provisions for
    /// an error code at close.
    void close_stream(uint16_t error_code = 0)
    {
        requestClose_ = true;
        trigger();
    }

    /// Specifies what the source should propose as window size to the
    /// destination. May be called only after start_stream.
    ///
    /// @param window_size in bytes, what should we propose in the stream
    /// initiate call
    ///
    StreamSenderCan &set_proposed_window_size(uint16_t window_size)
    {
        HASSERT(state_ == STARTED);
        streamWindowSize_ = window_size;
        return *this;
    }

    /// Specifies the Stream UID to send in the stream initiate request. May be
    /// called only after start_stream. This function must be used if opening
    /// an unannounced stream to a destination.
    ///
    /// @param stream_uid a valid 6-byte stream identifier.
    ///
    StreamSenderCan &set_stream_uid(NodeID stream_uid)
    {
        HASSERT(state_ == STARTED);
        /// @todo implement opening unannounced streams.
        return *this;
    }

    /// Sets the stream sender to be available for reuse after a stream has
    /// been closed or reached error.
    void clear()
    {
        if (state_ == STATE_ERROR || state_ == CLOSING)
        {
            state_ = IDLE;
        }
    }

#ifdef GTEST
    /// Requests to exit any timed operation.
    /// @return true if a timer was woken up.
    bool shutdown()
    {
        if (sleeping_)
        {
            timer_.trigger();
            sleeping_ = false;
            return true;
        }
        return false;
    }
#endif

    /// @return the state of this stream sender.
    StreamSenderState get_state()
    {
        return state_;
    }

    /// @return the error code if we got a rejection from the remote node.
    uint16_t get_error()
    {
        return errorCode_;
    }

    /// @return the stream SID (identifier on this node).
    uint8_t get_src_stream_id()
    {
        return localStreamId_;
    }

    /// @return the stream DID (identifier on the receiving node).
    uint8_t get_dst_stream_id()
    {
        return dstStreamId_;
    }

    /// Start of state machine, called when a buffer of data to send arrives
    /// from the application layer.
    Action entry() override
    {
        if (requestInit_)
        {
            requestInit_ = 0;
            return call_immediately(STATE(initiate_stream));
        }
        if (state_ == STATE_ERROR || state_ == CLOSING)
        {
            return release_and_exit();
        }
        DASSERT(state_ == RUNNING);
        if (!streamWindowRemaining_)
        {
            // We ran out of the current stream window size.
            return call_immediately(STATE(wait_for_stream_proceed));
        }
        if (!remaining())
        {
            // We ran out of the current chunk of stream payload from the
            // source.
            if (requestClose_ && queue_empty())
            {
                requestClose_ = false;
                return call_immediately(STATE(do_close_stream));
            }
            return release_and_exit();
        }
        return call_immediately(STATE(allocate_can_buffer));
    }

private:
    /// Sends an empty message to *this, thereby waking up the state machine.
    void trigger()
    {
        auto *b = alloc();
        this->send(b);
    }

    /// Allocates a GenMessage buffer and sends out the stream initiate message
    /// to the destination.
    Action initiate_stream()
    {
        // Grabs alias / node ID from the cache.
        node_->iface()->canonicalize_handle(&dst_);
        return allocate_and_call(node_->iface()->addressed_message_write_flow(),
            STATE(send_init_stream));
    }

    /// Sends the stream initiate message.
    Action send_init_stream()
    {
        auto *b = get_allocation_result(
            node_->iface()->addressed_message_write_flow());
        b->data()->reset(Defs::MTI_STREAM_INITIATE_REQUEST, node_->node_id(),
            dst_,
            StreamDefs::create_initiate_request(
                streamWindowSize_, false, localStreamId_, dstStreamId_));

        node_->iface()->dispatcher()->register_handler(
            &streamInitiateReplyHandler_, Defs::MTI_STREAM_INITIATE_REPLY,
            Defs::MTI_EXACT);

        node_->iface()->addressed_message_write_flow()->send(b);
        sleeping_ = true;
        state_ = INITIATING;
        LOG(VERBOSE, "wait for stream init reply");
        return sleep_and_call(&timer_, SEC_TO_NSEC(STREAM_INIT_TIMEOUT_SEC),
            STATE(received_init_stream));
    }

    /// Callback from GenHandler when a stream initiate reply message arrives
    /// at the local interface.
    void stream_initiate_replied(Buffer<GenMessage> *message)
    {
        LOG(VERBOSE, "stream init reply: %s",
            string_to_hex(message->data()->payload).c_str());
        auto rb = get_buffer_deleter(message);
        if (message->data()->dstNode != node_ ||
            !node_->iface()->matching_node(dst_, message->data()->src))
        {
            LOG(INFO, "stream reply not for me");
            // Not for me.
            return;
        }
        const auto &payload = message->data()->payload;
        if (payload.size() < 6 || (uint8_t)payload[4] != localStreamId_)
        {
            LOG(INFO, "wrong stream ID %x %x", payload[4], localStreamId_);
            // Talking about another stream or incorrect data.
            return;
        }
        dstStreamId_ = payload[5];
        streamFlags_ = payload[2];
        streamAdditionalFlags_ = payload[3];
        streamWindowSize_ = (payload[0] << 8) | payload[1];
        // Grabs alias / node ID from the cache.
        node_->iface()->canonicalize_handle(&dst_);

        // We save the remote alias here if we haven't got any yet.
        if (message->data()->src.alias)
        {
            dst_.alias = message->data()->src.alias;
        }

        isLoopbackStream_ =
            (node_->iface()->lookup_local_node_handle(dst_) != nullptr);

        sleeping_ = false;
        timer_.trigger();
    }

    /// State executed after wakeup from the stream initiate reply received
    /// handler.
    Action received_init_stream()
    {
        LOG(VERBOSE, "stream init reply wait done");
        node_->iface()->dispatcher()->unregister_handler(
            &streamInitiateReplyHandler_, Defs::MTI_STREAM_INITIATE_REPLY,
            Defs::MTI_EXACT);
        if (!(streamFlags_ & StreamDefs::FLAG_ACCEPT))
        {
            if (streamFlags_ & StreamDefs::FLAG_PERMANENT_ERROR)
            {
                return return_error(DatagramDefs::PERMANENT_ERROR |
                        (streamFlags_ << 8) | streamAdditionalFlags_,
                    "Stream initiate request was denied (permanent error).");
            }
            else
            {
                return return_error(Defs::ERROR_TEMPORARY |
                        (streamFlags_ << 8) | streamAdditionalFlags_,
                    "Stream initiate request was denied (temporary error).");
            }
        }
        if (!streamWindowSize_)
        {
            return return_error(DatagramDefs::PERMANENT_ERROR,
                "Inconsistency: zero buffer length but "
                "accepted stream request.");
        }
        streamWindowRemaining_ = streamWindowSize_;
        node_->iface()->dispatcher()->register_handler(
            &streamProceedHandler_, Defs::MTI_STREAM_PROCEED, Defs::MTI_EXACT);
        state_ = RUNNING;
        return entry();
    }

    /// Allocates a GenMessage buffer and sends out the stream close message
    /// to the destination.
    Action do_close_stream()
    {
        node_->iface()->dispatcher()->unregister_handler(
            &streamProceedHandler_, Defs::MTI_STREAM_PROCEED, Defs::MTI_EXACT);
        return allocate_and_call(node_->iface()->addressed_message_write_flow(),
            STATE(send_close_stream));
    }

    /// Sends the stream close message.
    Action send_close_stream()
    {
        auto *b = get_allocation_result(
            node_->iface()->addressed_message_write_flow());
        b->data()->reset(Defs::MTI_STREAM_COMPLETE, node_->node_id(), dst_,
            StreamDefs::create_close_request(
                localStreamId_, dstStreamId_, totalByteCount_));

        node_->iface()->addressed_message_write_flow()->send(b);
        state_ = CLOSING;
        return entry();
    }

    /// Allocates a buffer for a CAN frame (for payload send).
    Action allocate_can_buffer()
    {
        return allocate_and_call(
            ifCan_->frame_write_flow(), STATE(got_frame), &canFramePool_);
    }

    /// Got a buffer for an output frame (payload send).
    Action got_frame()
    {
        auto *b = get_allocation_result(ifCan_->frame_write_flow());

        uint32_t can_id;
        NodeAlias local_alias =
            ifCan_->local_aliases()->lookup(node_->node_id());
        NodeAlias remote_alias = dst_.alias;
        CanDefs::set_datagram_fields(
            &can_id, local_alias, remote_alias, CanDefs::STREAM_DATA);
        auto *frame = b->data()->mutable_frame();
        SET_CAN_FRAME_ID_EFF(*frame, can_id);

        size_t len = compute_next_can_length();

        frame->can_dlc = len + 1;
        frame->data[0] = dstStreamId_;
        memcpy(&frame->data[1], payload(), len);
        advance(len);

        if (!isLoopbackStream_)
        {
            ifCan_->frame_write_flow()->send(b);
        }
        else
        {
            ifCan_->loopback_frame_write_flow()->send(b);
        }
        return entry();
    }

    /// Starts sleeping until a proceed message arrives. Run this state when
    /// streamWindowRemaining_ == 0.
    Action wait_for_stream_proceed()
    {
        if (streamWindowRemaining_)
        {
            // received early stream_proceed response
            return call_immediately(STATE(stream_proceed_timeout));
        }
        sleeping_ = true;
        state_ = FULL;
        return sleep_and_call(&timer_, SEC_TO_NSEC(STREAM_PROCEED_TIMEOUT_SEC),
            STATE(stream_proceed_timeout));
    }

    /// Callback from the handler flow.
    void stream_proceed_received(Buffer<GenMessage> *message)
    {
        auto rb = get_buffer_deleter(message);
        if (message->data()->dstNode != node_ ||
            !node_->iface()->matching_node(dst_, message->data()->src))
        {
            // Not for me.
            return;
        }

        const auto &payload = message->data()->payload;
        if (payload.size() < 2 || (uint8_t)payload[0] != localStreamId_)
        {
            // Talking about another stream or incorrect data.
            return;
        }

        /// @todo add progress callback API

        streamWindowRemaining_ += streamWindowSize_;
        if (sleeping_)
        {
            sleeping_ = false;
            timer_.trigger();
        }
    }

    Action stream_proceed_timeout()
    {
        if (!streamWindowRemaining_) // no proceed arrived
        {
            /// @todo (balazs.racz) somehow merge these two actions: remember
            /// that we timed out and close the stream.
            return return_error(Defs::ERROR_TEMPORARY,
                "Timed out waiting for stream proceed message.");
            // return call_immediately(STATE(close_stream));
        }
        state_ = RUNNING;
        return entry();
    }

private:
    /// @return how many bytes of data we can put into the next CAN frame.
    size_t compute_next_can_length()
    {
        size_t ret = remaining();
        // Cannot exceed CAN frame max payload.
        if (ret > MAX_BYTES_PAYLOAD_PER_CAN_FRAME)
        {
            ret = MAX_BYTES_PAYLOAD_PER_CAN_FRAME;
        }
        // Cannot exceed remaining bytes in stream window.
        if (ret > streamWindowRemaining_)
        {
            ret = streamWindowRemaining_;
        }
        return ret;
    }

    /// @return the number of bytes available in the current chunk.
    size_t remaining()
    {
        return message()->data()->size_;
    }

    /// @return pointer to the beginning of the data to send.
    uint8_t *payload()
    {
        return message()->data()->data_;
    }

    /// Consumes a certain number of bytes from the beginning of the data to
    /// send.
    /// @param num_bytes how much data to consume.
    void advance(size_t num_bytes)
    {
        message()->data()->advance(num_bytes);
        totalByteCount_ += num_bytes;
        streamWindowRemaining_ -= num_bytes;
    }

    Action return_error(uint32_t code, string message)
    {
        LOG(INFO, "error %x: %s", (unsigned)code, message.c_str());
        errorCode_ = code;
        state_ = STATE_ERROR;
        return release_and_exit();
    }

    /// How many seconds for waiting for a stream proceed before we give up
    /// with a timeout.
    static constexpr size_t STREAM_PROCEED_TIMEOUT_SEC = 20;

    /// How many seconds for waiting for a stream init before we give up
    /// with a timeout.
    static constexpr size_t STREAM_INIT_TIMEOUT_SEC = 20;

    /// How many bytes payload we can copy into a single CAN frame.
    static constexpr size_t MAX_BYTES_PAYLOAD_PER_CAN_FRAME = 7;

    /// How many CAN frames should we allocate at a given time.
    static constexpr size_t MAX_FRAMES_IN_FLIGHT = 4;

    /// How many bytes the allocation of a single CAN frame should be.
    static constexpr size_t CAN_FRAME_ALLOC_SIZE =
        sizeof(CanFrameWriteFlow::message_type);

    /// Handles incoming stream proceed messages.
    MessageHandler::GenericHandler streamProceedHandler_ {
        this, &StreamSenderCan::stream_proceed_received};
    /// Handles incoming stream initiate reply messages.
    MessageHandler::GenericHandler streamInitiateReplyHandler_ {
        this, &StreamSenderCan::stream_initiate_replied};

    /// CAN-bus interface.
    IfCan *ifCan_;
    /// Which node are we sending the outgoing data from. This is a local
    /// virtual node.
    Node *node_ {nullptr};
    /// Destination node that we are sending to. It is important that the alias
    /// is filled in here.
    NodeHandle dst_;
    /// How many bytes we have transmitted in this stream so far.
    size_t totalByteCount_ {0};
    /// What state the current class is in.
    StreamSenderState state_ {IDLE};
    /// Stream ID at the source node. @todo fill in
    uint8_t localStreamId_ {StreamDefs::INVALID_STREAM_ID};
    /// Stream ID at the destination node. @todo fill in
    uint8_t dstStreamId_ {StreamDefs::INVALID_STREAM_ID};
    /// Determines whether the stream transmission is happening to
    /// localhost. Almost never true.
    uint8_t isLoopbackStream_ : 1;
    /// True if we are waiting for the timer.
    uint8_t sleeping_ : 1;
    /// 1 if there is a pending close request.
    uint8_t requestClose_ : 1;
    /// 1 if there is a pending initialize request.
    uint8_t requestInit_ : 1;
    /// Flags from the remote node that we got in stream initiate reply
    uint8_t streamFlags_ {0};
    /// More flags from the remote node that we got in stream initiate reply
    uint8_t streamAdditionalFlags_ {0};
    /// Total stream window size. @todo fill in
    uint16_t streamWindowSize_ {StreamDefs::MAX_PAYLOAD};
    /// Remaining stream window size. @todo fill in
    uint16_t streamWindowRemaining_ {0};
    /// When the stream process fails, this variable contains an error code.
    uint32_t errorCode_ {0};
    /// Source of buffers for outgoing CAN frames. Limtedpool is allocating and
    /// releasing to the mainBufferPool, but blocks when we exceed a certain
    /// number of allocations until some buffers get freed.
    LimitedPool canFramePool_ {CAN_FRAME_ALLOC_SIZE, MAX_FRAMES_IN_FLIGHT};
    /// Helper object for timeouts.
    StateFlowTimer timer_ {this};
};

class StreamRendererCan : public StateFlow<ByteBuffer, QList<1>>
{ };

} // namespace openlcb

#endif // _OPENLCB_STREAMSENDER_HXX_
