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
 * \file MemoryConfigStream.hxx
 *
 * Implementation of the stream support for Memory Config Protocol server
 *
 * @author Balazs Racz
 * @date 20 Dec 2022
 */

#ifndef _OPENLCB_MEMORYCONFIGSTREAM_HXX_
#define _OPENLCB_MEMORYCONFIGSTREAM_HXX_

#include "openlcb/If.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/StreamSender.hxx"
#include "openlcb/StreamTransport.hxx"

namespace openlcb
{

/// This is a self-owned flow which reads an memory space into a stream.
class MemorySpaceStreamReadFlow : public StateFlowBase
{
public:
    /// This callback function will be called with an error code, or 0 on
    /// success.
    using CallbackFn = std::function<void(uint16_t)>;

    /// Constructor. Does NOT transfer ownership; this class will delete
    /// itself.
    ///
    /// @param node virtual nde where to send the stream from.
    /// @param space memory space pointer
    /// @param dst target node to send data to
    /// @param dst_stream_id sream ID on the target node
    /// @param ofs where to start reading from in the memory space
    /// @param len how many bytes to send (0xFFFFFFFF to send until EOF)
    /// @param started_cb will be invoked when the stream initiate reply
    /// arrives and data can be sent to the stream.
    MemorySpaceStreamReadFlow(Node *node, MemorySpace *space, NodeHandle dst,
        uint8_t dst_stream_id, uint32_t ofs, uint32_t len,
        CallbackFn started_cb)
        : StateFlowBase(node->iface())
        , dst_(dst)
        , startedCb_(std::move(started_cb))
        , space_(space)
        , dstStreamId_(dst_stream_id)
        , node_(node)
        , ofs_(ofs)
        , len_(len)
    {
        LOG(INFO, "starting streamed read, dst %02x", dstStreamId_);
        start_flow(STATE(alloc_stream));
    }

    /// @return the currently running flow's source stream ID.
    uint8_t get_src_stream_id()
    {
        return srcStreamId_;
    }

    /// @return the currently running flow's destination stream ID.
    uint8_t get_dst_stream_id()
    {
        return dstStreamId_;
    }

private:
    Action alloc_stream()
    {
        return allocate_and_call(
            STATE(got_sender), stream_transport()->sender_allocator());
    }

    Action got_sender()
    {
        LOG(INFO, "got sender");
        sender_ =
            full_allocation_result(stream_transport()->sender_allocator());
        /// @todo the APIs are not on the right object in StreamSender, so we
        /// have to do this down cast.
        senderCan_ = static_cast<StreamSenderCan *>(sender_);
        HASSERT(senderCan_);
        return call_immediately(STATE(initiate_stream));
    }

    Action initiate_stream()
    {
        LOG(INFO, "initiate");
        srcStreamId_ = stream_transport()->get_send_stream_id();
        senderCan_->start_stream(node_, dst_, srcStreamId_, dstStreamId_);
        return call_immediately(STATE(wait_for_started));
    }

    Action wait_for_started()
    {
        auto state = senderCan_->get_state();
        if (state == StreamSender::RUNNING)
        {
            dstStreamId_ = senderCan_->get_dst_stream_id();
            startedCb_(0);
            return call_immediately(STATE(alloc_buffer));
        }
        if (state == StreamSender::STATE_ERROR)
        {
            auto err = senderCan_->get_error();
            LOG(INFO, "failed to start stream: 0x%04x", err);
            startedCb_(err);
            return call_immediately(STATE(done_stream));
        }
        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(3), STATE(wait_for_started));
    }

    Action alloc_buffer()
    {
        return allocate_and_call<RawData>(
            nullptr, STATE(have_raw_buffer), &sendBufferPool_);
    }

    Action have_raw_buffer()
    {
        LOG(INFO, "have raw buf len %u", (unsigned)len_);

        RawBufferPtr raw_buffer(get_allocation_result<RawData>(nullptr));
        sendBuffer_ = get_buffer_deleter(sender_->alloc());
        sendBuffer_->data()->set_from(std::move(raw_buffer), 0);
        return call_immediately(STATE(try_read));
    }

    Action try_read()
    {
        if (!len_)
        {
            sender_->send(sendBuffer_.release());
            senderCan_->close_stream();
            return call_immediately(STATE(wait_for_close));
        }
        size_t free = sendBuffer_->data()->free_space();
        if (!free)
        {
            sender_->send(sendBuffer_.release());
            return call_immediately(STATE(alloc_buffer));
        }
        size_t cnt = len_;
        if (free < cnt)
        {
            cnt = free;
        }
        uint8_t *ptr = sendBuffer_->data()->append_ptr();
        MemorySpace::errorcode_t err;
        size_t copied = space_->read(ofs_, ptr, cnt, &err, this);
        sendBuffer_->data()->append_complete(copied);
        ofs_ += copied;
        len_ -= copied;
        if (err == MemorySpace::ERROR_AGAIN)
        {
            return wait();
        }
        if (err == MemoryConfigDefs::ERROR_OUT_OF_BOUNDS)
        {
            sender_->send(sendBuffer_.release());
            senderCan_->close_stream();
            return call_immediately(STATE(wait_for_close));
        }
        if (!err)
        {
            return again();
        }
        else
        {
            LOG(INFO, "error reading input stream: %04x", err);
            sender_->send(sendBuffer_.release());
            senderCan_->close_stream(err);
            return call_immediately(STATE(wait_for_close));
        }
    }

    Action wait_for_close()
    {
        auto state = senderCan_->get_state();
        if (state == StreamSender::CLOSING && senderCan_->is_waiting())
        {
            // Sender is done and empty.
            return call_immediately(STATE(done_stream));
        }
        if (state == StreamSender::STATE_ERROR && senderCan_->is_waiting())
        {
            // Sender has errored and consumed / thrown away all data.
            // There is no place really to show the error.
            LOG(INFO, "Stream sender error: 0x%04x", senderCan_->get_error());
            return call_immediately(STATE(done_stream));
        }
        return sleep_and_call(&timer_, MSEC_TO_NSEC(3), STATE(wait_for_close));
    }

    Action done_stream()
    {
        senderCan_->clear();
        stream_transport()->sender_allocator()->typed_insert(sender_);
        sender_ = nullptr;
        return delete_this();
    }

    StreamTransport *stream_transport()
    {
        return node_->iface()->stream_transport();
    }

    /// This pool is used to allocate raw buffers to read data into from the
    /// memory space. By keeping the count limited we can ensure that we only
    /// carry 2 kbytes of data in RAM before it gets dumped into the CAN-bus
    /// packets.
    LimitedPool sendBufferPool_ {sizeof(RawBuffer), 2, rawBufferPool};
    /// We keep reading into this buffer from the memory space.
    ByteBufferPtr sendBuffer_;
    /// Helper object for waiting.
    StateFlowTimer timer_ {this};
    /// Address to which we are sending the stream.
    NodeHandle dst_;
    /// callback to invoke after start is successful.
    CallbackFn startedCb_;
    /// Memory space we are reading.
    MemorySpace *space_;
    /// Destination stream ID on the target node.
    uint8_t srcStreamId_ {StreamDefs::INVALID_STREAM_ID};
    /// Destination stream ID on the target node.
    uint8_t dstStreamId_;
    /// Node from which we are sending the stream.
    Node *node_;
    /// Next byte to read.
    uint32_t ofs_;
    /// How many bytes are left to read. 0xFFFFFFFF if all bytes until EOF need
    /// to be read.
    uint32_t len_;
    ///
    ///
    StreamSenderCan *senderCan_;
    StreamSender *sender_;
};

/// Handler for the stream read/write commands in the memory config protocol
/// (server side).
class MemoryConfigStreamHandler : public MemoryConfigHandlerBase
{
public:
    MemoryConfigStreamHandler(MemoryConfigHandler *parent)
        : MemoryConfigHandlerBase(parent->dg_service())
        , parent_(parent)
    {
        parent_->set_stream_handler(this);
    }

    Action entry() override
    {
        LOG(INFO, "stream req");
        // The verification of the incoming data is already done by the calling
        // MemoryConfigHandler.
        response_.clear();
        const uint8_t *bytes = in_bytes();
        uint8_t cmd = bytes[1];

        switch (cmd & MemoryConfigDefs::COMMAND_MASK)
        {
            case MemoryConfigDefs::COMMAND_READ_STREAM:
            {
                return call_immediately(STATE(handle_read_stream));
            }
                /// @todo handle write stream
        }
        return respond_reject(Defs::ERROR_UNIMPLEMENTED_SUBCMD);
    }

private:
    Action handle_read_stream()
    {
        size_t len = message()->data()->payload.size();
        const uint8_t *bytes = in_bytes();

        if (len < 8)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS);
        }
        MemorySpace *space = get_space();
        if (!space)
        {
            return respond_reject(MemoryConfigDefs::ERROR_SPACE_NOT_KNOWN);
        }

        size_t stream_data_offset = 6;
        if (has_custom_space())
        {
            ++stream_data_offset;
        }
        if (len < stream_data_offset + 2)
        {
            return respond_reject(Defs::ERROR_INVALID_ARGS);
        }

        uint8_t dst_stream_id = bytes[stream_data_offset + 1];
        uint32_t num_bytes_to_read = 0xFFFFFFFFu;
        LOG(INFO, "dst stream id %02x", dst_stream_id);
        if (len >= stream_data_offset + 6)
        {
            memcpy(&num_bytes_to_read, bytes + stream_data_offset + 2, 4);
            num_bytes_to_read = be32toh(num_bytes_to_read);
        }
        streamErrorCode_ = Defs::ERROR_TEMPORARY;
        // This object is self-owned, so it will run `delete this`.
        readFlow_ = new MemorySpaceStreamReadFlow(message()->data()->dst,
            get_space(), message()->data()->src, dst_stream_id, get_address(),
            num_bytes_to_read,
            std::bind(&MemoryConfigStreamHandler::stream_start_cb, this,
                std::placeholders::_1));
        return wait_and_call(STATE(read_started));
    }

    /// Callback from the stream flow that tells us whether the stream was
    /// successfully started (or not).
    void stream_start_cb(uint16_t error)
    {
        streamErrorCode_ = error;
        notify();
    }

    Action read_started()
    {
        uint16_t error = streamErrorCode_;
        size_t response_data_offset = 6;
        if (has_custom_space())
        {
            ++response_data_offset;
        }
        response_.reserve(response_data_offset + 6);
        response_.resize(response_data_offset + 2);
        uint8_t *response_bytes = out_bytes();
        response_bytes[0] = DATAGRAM_ID;
        response_bytes[1] = error ? MemoryConfigDefs::COMMAND_READ_STREAM_FAILED
                                  : MemoryConfigDefs::COMMAND_READ_STREAM_REPLY;
        set_address_and_space();
        if (error)
        {
            response_.resize(response_data_offset + 2);
            response_bytes[response_data_offset] = error >> 8;
            response_bytes[response_data_offset + 1] = error & 0xff;
        }
        else
        {
            response_.resize(response_data_offset + 2);
            response_bytes[response_data_offset] =
                readFlow_->get_src_stream_id();
            response_bytes[response_data_offset + 1] =
                readFlow_->get_dst_stream_id();
            if (message()->data()->payload.size() >= response_data_offset + 6)
            {
                response_.resize(response_data_offset + 6);
                memcpy(response_bytes + response_data_offset + 2,
                    in_bytes() + response_data_offset + 2, 4);
            }
        }
        return respond_ok(DatagramClient::REPLY_PENDING);
    }

    /** Looks up the memory space for the current datagram. Returns NULL if no
     * space was registered (for neither the current node, nor global). */
    MemorySpace *get_space()
    {
        int space_number = get_space_number();
        if (space_number < 0)
            return nullptr;
        MemorySpace *space =
            registry()->lookup(message()->data()->dst, space_number);
        if (!space)
        {
            LOG(WARNING,
                "MemoryConfig: asked node 0x%012" PRIx64 " for unknown space "
                "%d. Source {0x%012" PRIx64 ", %03x}",
                message()->data()->dst->node_id(), space_number,
                message()->data()->src.id, message()->data()->src.alias);
            return nullptr;
        }
        if (!space->set_node(message()->data()->dst))
        {
            LOG(WARNING, "MemoryConfig: Global space %d rejected node.",
                space_number);
            return nullptr;
        }
        return space;
    }

    Registry *registry()
    {
        return parent_->registry();
        ;
    }

    /// Parent object from which we are getting commands forwarded.
    MemoryConfigHandler *parent_;

    /// OpenLCB error code from the stream start.
    uint16_t streamErrorCode_;

    union
    {
        /// The flow that we created for reading the memory space into the
        /// stream.
        MemorySpaceStreamReadFlow *readFlow_;
    };
}; // class MemoryConfigStreamHandler

} // namespace openlcb

#endif // _OPENLCB_MEMORYCONFIGSTREAM_HXX_
