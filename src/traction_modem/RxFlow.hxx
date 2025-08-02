/** @copyright
 * Copyright (c) 2025, Stuart Baker
 * All rights reserved
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
 * @file RxFlow.hxx
 *
 * Implements the message flow for reception.
 *
 * @author Stuart Baker
 * @date 6 Feb 2025
 */

#ifndef _TRACTION_MODEM_RXFLOW_HXX_
#define _TRACTION_MODEM_RXFLOW_HXX_

#include "executor/Dispatcher.hxx"
#include "traction_modem/Message.hxx"
#include "utils/format_utils.hxx"

namespace traction_modem
{

/// Public interface to aid in testing.
class RxInterface
{
public:
    /// Start the flow using the given interface.
    /// @param fd interface to receive messages on
    virtual void start(int fd) = 0;

    /// Register a message handler.
    /// @param interface interface to dispatch the messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    virtual void register_handler(PacketFlowInterface *interface,
        Message::id_type id, Message::id_type mask = Message::EXACT_MASK) = 0;

    /// Unregister a message handler. Must be currently registered by a previous
    /// call to register handler.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    virtual void unregister_handler(PacketFlowInterface *interface,
        Message::id_type id, Message::id_type mask = Message::EXACT_MASK) = 0;

    /// Unregister all message handlers.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    virtual void unregister_handler_all(PacketFlowInterface *interface) = 0;

    /// Sets one handler to receive all messages that no other handler has
    /// matched. May be called only once in the lifetime of a dispatcher
    /// object.
    /// @param handler is the handler pointer for the fallback handler.
    virtual void register_fallback_handler(PacketFlowInterface *interface) = 0;

    /// Get the current resynchronization count value.
    /// @return resynchronization count
    unsigned get_resync_count()
    {
        return resyncCount_;
    }
protected:
    /// Track the number of times that we try to resync.
    unsigned resyncCount_{0};
};

/// Object responsible for reading in a stream of bytes over the modem
/// interface, forming the stream of bytes into complete messages, and
/// dispatching them to registered handlers.
class RxFlow : public RxInterface, public StateFlowBase
{
public:
    using BufferType = Buffer<Message>;
    using Receiver = PacketFlowInterface;

    /// Constructor.
    /// @param service service that the flow is bound to
    RxFlow(Service *service)
        : StateFlowBase(service)
        , dispatcher_(service)
    { }

    /// Start the flow using the given interface.
    /// @param fd interface to transmit the messages on, should be open with
    ///           all serial settings already applied
    void start(int fd) override
    {
        fd_ = fd;
        start_flow(STATE(reset));
    }

    /// Get the wire time for a single character.
    /// @param wire time for a single character in nanoseconds
    long long get_character_nsec()
    {
        /// @todo Once we support multi-baud, this should be modified to
        ///       provide the character nsec at the current baud rate.
        return CHARACTER_NSEC;
    }

#if defined(GTEST)
    bool exit_ = false;
#endif

    /// Register a message handler.
    /// @param interface interface to dispatch the messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    void register_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.register_handler(interface, id, mask);
    }

    /// Unregister a message handler. Must be currently registered by a previous
    /// call to register handler.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    /// @param id ID of the message
    /// @param mask bit mask of the message ID.
    void unregister_handler(PacketFlowInterface *interface, Message::id_type id,
        Message::id_type mask = Message::EXACT_MASK) override
    {
        dispatcher_.unregister_handler(interface, id, mask);
    }

    /// Unregister all message handlers.
    /// @param interface interface previously registered to dispatch the
    ///        messages to
    void unregister_handler_all(PacketFlowInterface *interface) override
    {
        dispatcher_.unregister_handler_all(interface);
    }

    /// Sets one handler to receive all messages that no other handler has
    /// matched. May be called only once in the lifetime of a dispatcher
    /// object.
    /// @param handler is the handler pointer for the fallback handler.
    void register_fallback_handler(PacketFlowInterface *interface) override
    {
        dispatcher_.register_fallback_handler(interface);
    }

private:
    /// Resets the message reception state machine.
    /// @return next state wait_for_base_data()
    Action reset()
    {
        // Looking for the start of a new message. The clear is needed to
        // "reset" the string from the std::move() operation that might have 
        // occurred on the prior message, leaving it in an unknown state.
        payload_.clear();
        // Note: We are relying on the fact that the default allocator is used,
        //       which is the heap allocator, and that the heap allocator will
        //       allocate memory on (at least) the native machine size boundary.
        //       This is important in order to be able to cast payload_.data()
        //       to a Defs::Message type for easier decoding. Otherwise if the
        //       architecture supports unaligned accesses, then we are fine for
        //       all allocators.
        payload_.resize(MIN_MESSAGE_SIZE);
        recvCnt_ = 0;
        return call_immediately(STATE(wait_for_base_data));
    }

    /// Wait until we have at least as much data as a minimum size message.
    /// @return next state base_data_received
    Action wait_for_base_data()
    {
#if defined(GTEST)
        if (exit_)
        {
            return exit();
        }
#endif
        // Every message is at least a minimum size. There is really no point
        // to waste any cycles processing until at least the minimum count is
        // received.
        if (recvCnt_ >= MIN_MESSAGE_SIZE)
        {
            // We already have enough data to start processing.
            return call_immediately(STATE(base_data_received));
        }

        HASSERT(payload_.size() >= MIN_MESSAGE_SIZE);

        // This is pre-emptive. We will have received this count by the time we
        // enter the next state because the read blocks until we have all the
        // base data.
        size_t offset = recvCnt_;
        recvCnt_ = MIN_MESSAGE_SIZE;

        return read_repeated(&helper_, fd_, &payload_[offset],
            MIN_MESSAGE_SIZE - offset, STATE(base_data_received));
    }

    /// Received at least as much data as the minimum message size.
    /// @return next state is header_complete if valid preamble found, else
    ///         next state is resync to look for a valid header.
    Action base_data_received()
    {
        const Defs::Message *m = (const Defs::Message*)payload_.data();

        /// @todo Handle read errors.

        // Look for the preamble.
        if (m->header_.preamble_ == htobe32(Defs::PREAMBLE))
        {
            // Valid preamble found where it is supposed to be. Assume for now
            // that we also have a valid header that follows.
            LOG(VERBOSE, "[ModemRx] recv cmd: 0x%04x, len: %u",
                be16toh(m->header_.command_), be16toh(m->header_.length_));
            return call_immediately(STATE(header_complete));
        }

        // Else valid preamble and/or header not found where it is supposed to
        // be.
        LOG(WARNING,
            "[ModemRx] Did not find an expected valid preamble and/or header.");
        return call_immediately(STATE(resync));
    }

    /// We think we have a complete header because we just validated a preamble.
    /// Validate that the header meta data (data length) is legit.
    /// @return next state is resync on error (invalid data length), else
    ///         next state is maybe_message_complete.
    Action header_complete()
    {
        const Defs::Message *m = (const Defs::Message*)payload_.data();
        size_t len = be16toh(m->header_.length_);

        if (len > MAX_DATA_LEN)
        {
            // Violated the maximum length data allowed by the protocol. We are
            // probably out of sync.
            LOG(WARNING, "[ModemRx] Maximum data length violation.");
            return call_immediately(STATE(resync));
        }

        size_t total_len = MIN_MESSAGE_SIZE + len;
        if (recvCnt_ >= total_len)
        {
            // We already have enough data for the complete message.
            return call_immediately(STATE(maybe_message_complete));
        }

        payload_.resize(total_len);
        size_t needed_len = total_len - recvCnt_;

        return read_repeated_with_timeout(&helper_,
            2 * get_character_nsec() * needed_len, fd_, &payload_[recvCnt_],
            needed_len, STATE(maybe_message_complete));
    }

    /// We might have a complete message if we have received enough data.
    /// @return next state is resync if a timeout occurred or a CRC error is
    ///         detected, else if we have the likely start to the next message
    ///         next state is wait_for_base_data, else next state is reset.
    Action maybe_message_complete()
    {
        const Defs::Message *m = (const Defs::Message*)payload_.data();
        size_t len = be16toh(m->header_.length_);

        if (recvCnt_ < (MIN_MESSAGE_SIZE + len) && helper_.remaining_)
        {
            // Timeout, we may be out ot sync. Check for a preamble in the data
            // we did receive.
            recvCnt_ += len - helper_.remaining_;
            LOG(WARNING, "[ModemRx] Timeout waiting for expected receive data, "
                "remaining: %u", helper_.remaining_);
            return call_immediately(STATE(resync));
        }

        Defs::CRC crc_calc;
        Defs::CRC crc_recv = Defs::get_crc(payload_, len);
        crc3_crc16_ccitt(
            payload_.data() + sizeof(uint32_t),
            sizeof(m->header_.command_) + sizeof(m->header_.length_) + len,
            &crc_calc.crc[0]);
        if (crc_calc != crc_recv)
        {
            LOG(WARNING, "[ModemRx] CRC Error, received: 0x%04X 0x%04X 0x%04X, "
                "calculated: 0x%04X 0x%04X 0x%04X",
                crc_recv.all_, crc_recv.even_, crc_recv.odd_,
                crc_calc.all_, crc_calc.even_, crc_calc.odd_);
            return call_immediately(STATE(resync));
        }

        size_t total_len = MIN_MESSAGE_SIZE + len;
        Defs::Payload payload_tmp;
        if (payload_.size() > total_len)
        {
            // We have the start of the next message, which we need to save off.
            payload_tmp =
                payload_.substr(total_len, payload_.size() - total_len);
            payload_.resize(total_len);
        }
        // A this point, we have a valid message.
        LOG(VERBOSE, "[ModemRx] %s", string_to_hex(payload_).c_str());

        auto *b = dispatcher_.alloc();
        b->data()->payload = std::move(payload_);
        dispatcher_.send(b);

        if (payload_tmp.size())
        {
            // Move the start of the next message into payload.
            LOG(VERBOSE, "[ModemRx] Remaining payload size: %zu",
                payload_tmp.size());
            payload_ = std::move(payload_tmp);
            recvCnt_ = payload_.size();
            return call_immediately(STATE(wait_for_base_data));
        }
        return call_immediately(STATE(reset));
    }

    /// Something went wrong in decoding the data stream. Try to resync on a
    /// preamble word.
    /// @return next state is wait_for_base_data if a valid preamble word is
    ///         found, else next state is reset to start over with new data.
    Action resync()
    {
        // Message parsing out of sync.
        ++resyncCount_;

        /// @todo Should we be sending out a framing error? I think so. How to
        ///       dispatch that? Maybe an error field in the Message? If we
        ///       send this to the dispatcher, be sure to add the appropriate
        ///       EXPECT_CALL(s) to the unit tests.

        for (unsigned idx = 1; idx < recvCnt_; ++idx)
        {
            if (payload_[idx] == Defs::PREAMBLE_FIRST)
            {
                if ((idx + 4) <= recvCnt_)
                {
                    uint32_t p;
                    memcpy(&p, payload_.data() + idx, 4);
                    if (p != htobe32(Defs::PREAMBLE))
                    {
                        continue;
                    }
                }
                // A memmove is more efficient than payload_.erase(0, idx)
                // because it will not shrink the size of the payload_, and it
                // maintains proper alignment by reusing the original heap
                // allocation.
                memmove(&payload_[0], &payload_[idx], recvCnt_ - idx);
                recvCnt_ -= idx;
                LOG(VERBOSE, "[ModemRx] Sync on preamble first, recvCnt_: %zu",
                    recvCnt_);
                return call_immediately(STATE(wait_for_base_data));
            }
        }
        // Now: we're out of sync and never found a viable first byte. Drop
        // all data.
        return call_immediately(STATE(reset));
    }

    /// Used to place a bounds on a timeout of a message.
    static constexpr long long CHARACTER_NSEC = 10 * SEC_TO_NSEC(1) / 250000;

    /// Minimum size of a message.
    static constexpr unsigned MIN_MESSAGE_SIZE = Defs::LEN_BASE;

    /// Maximum size of the data portion of a message.
    static constexpr unsigned MAX_DATA_LEN = Defs::MAX_LEN;

    /// Helper for reading in a select flow.
    StateFlowTimedSelectHelper helper_ {this};
    /// We assemble the message here.
    string payload_;
    /// Number of bytes that have been received into payload_, which may be
    /// less than payload_.size() since we reserve space ahead of time.
    size_t recvCnt_;
    /// Handles incoming messages from the RX Flow.
    DispatchFlow<Buffer<Message>, 2> dispatcher_;

    /// Interface fd.
    int fd_ = -1;
};

} // namespace traction_modem

#endif // _TRACTION_MODEM_RXFLOW_HXX_