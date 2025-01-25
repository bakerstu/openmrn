/** @copyright
 * Copyright (c) 2024, Balazs Racz
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
 * @file TractionModem.hxx
 *
 * Class for interacting with the decoder chip via the traction modem protocol.
 *
 * @author Balazs Racz
 * @date 9 Feb 2024
 */

#ifndef _TRACTIONMODEM_TRACTIONMODEM_HXX_
#define _TRACTIONMODEM_TRACTIONMODEM_HXX_

/// @todo Need to prune out the "hardware.hxx" dependencies from this file.
///       These need to be dispatched to hardware specific code somehow.

#include "traction_modem/TractionModemDefs.hxx"
#if !defined(GTEST)
#include "hardware.hxx"
#endif

#include "executor/StateFlow.hxx"
#include "openlcb/MemoryConfig.hxx"
#include "openlcb/TrainInterface.hxx"
#include "utils/format_utils.hxx"

namespace tractionmodem
{

struct TxMessage
{
    bool valid() const
    {
        return Defs::is_valid(payload);
    }

    uint16_t command() const
    {
        return Defs::get_uint16(payload, Defs::OFS_CMD);
    }

    uint16_t length() const
    {
        return Defs::get_uint16(payload, Defs::OFS_LEN);
    }

    /// Return the first two bytes of a response payload as an uint16_t. This is
    /// often a status / error code.
    uint16_t response_status()
    {
        return Defs::get_uint16(payload, Defs::OFS_DATA);
    }

    string payload;
};

using PacketFlowInterface = FlowInterface<Buffer<TxMessage>>;
using TxFlowBase = StateFlow<Buffer<TxMessage>, QList<2>>;

/// Object responsible for writing messages to the modem interface.
class TxFlow : public TxFlowBase
{
public:
    /// Constructor.
    /// @param service service that the flow is bound to
    TxFlow(Service *service)
        : TxFlowBase(service)
    {
        LOG(INFO, "[ModemTx] constructor");
    }

    /// Bind an interface to the flow to start transmitting to.
    /// @param fd interface to transmit the messages on
    void start(int fd)
    {
        LOG(INFO, "[ModemTx] fd");
        fd_ = fd;
    }

    /// Entry point to the state flow for incoming TxMessages to transmit.
    /// @return next state write_complete
    Action entry() override
    {
        if (fd_ < 0)
        {
            LOG(INFO, "[ModemTx] no uart");
            return release_and_exit();
        }
        LOG(INFO, "[ModemTx] msg len %d",
            (int)message()->data()->payload.size());
        return write_repeated(&helper_, fd_, message()->data()->payload.data(),
            message()->data()->payload.size(), STATE(write_complete));
    }

    /// Finish up the write and exit
    /// @return release_and_exit
    Action write_complete()
    {
        unsigned len = message()->data()->payload.size();
        unsigned num_sent = len - helper_.remaining_;
        const uint8_t *d = (const uint8_t *)message()->data()->payload.data();
        LOG(INFO, "[ModemTx] sent E%d len %u done %u %08x %04x...",
            helper_.hasError_, len, num_sent, *(unsigned *)(d + 0),
            (unsigned)be32toh(*(uint16_t *)(d + 4)));
        /// @TODO check for error
        return release_and_exit();
    }

private:
    /// Helper for performing the writes.
    StateFlowSelectHelper helper_ {this};
    /// Interface fd.
    int fd_ = -1;
};

/// Object responsible for reading in a stream of bytes over the modem interface
/// and forming the stream of bytes into complete messages.
class RxFlow : public StateFlowBase
{
public:
    using BufferType = Buffer<TxMessage>;
    using Receiver = PacketFlowInterface;

    /// Constructor.
    /// @param service service that the flow is bound to
    RxFlow(Service *service)
        : StateFlowBase(service)
    { }

    /// Start the flow using the given interface.
    /// @param fd interface to receive messages on
    void start(int fd)
    {
        fd_ = fd;
        start_flow(STATE(reset));
    }

    /// Register a listener to send incoming messages to.
    /// @param rcv lister that will receive incoming messages
    void set_listener(Receiver *rcv)
    {
        receiver_ = rcv;
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
        //       to a Defs::Message type for easier decoding.
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

        // Look for the preamble.
        if (m->header_.preamble_ == htobe32(Defs::PREAMBLE))
        {
            // Valid preamble found where it is supposed to be. Assume for now
            // that we also have a valid header that follows.
            LOG(INFO, "[ModemRx] recv cmd: 0x%04x, len: %u",
                be16toh(m->header_.command_), be16toh(m->header_.length_));
            return call_immediately(STATE(header_complete));
        }

        // Else valid preamble and/or header not found where it is supposed to
        // be.
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
            return call_immediately(STATE(resync));
        }

        size_t total_len = MIN_MESSAGE_SIZE + len;
        payload_.resize(total_len);

        if (recvCnt_ >= total_len)
        {
            // We already have enough data for the complete message.
            return call_immediately(STATE(maybe_message_complete));
        }

        size_t needed_len = total_len - recvCnt_;

        return read_repeated_with_timeout(&helper_,
            2 * get_character_nsec() * needed_len, fd_, &payload_[recvCnt_],
            needed_len, STATE(maybe_message_complete));
    }

    /// We might have a complete message if we have received enough data.
    /// @return next state is resync if a timeout occurred or a CRC error is
    ///         detected, else next state is reset.
    Action maybe_message_complete()
    {
        const Defs::Message *m = (const Defs::Message*)payload_.data();
        size_t len = be16toh(m->header_.length_);

        if (recvCnt_ < (MIN_MESSAGE_SIZE + len) && helper_.remaining_)
        {
            // Timeout, we may be out ot sync. Check for a preamble in the data
            // we did receive.
            recvCnt_ += len - helper_.remaining_;
            return call_immediately(STATE(resync));
        }

        // A this point, we have a valid message.
        LOG(INFO, "[ModemRx] %s", string_to_hex(payload_).c_str());

        Defs::CRC crc_calc;
        Defs::CRC crc_recv = Defs::get_crc(payload_);
        crc3_crc16_ccitt(
            payload_.data() + sizeof(uint32_t),
            payload_.size() - (sizeof(uint32_t) + sizeof(crc_calc)),
            crc_calc.crc);
        if (crc_calc != crc_recv)
        {
            LOG(INFO, "CRC Error, received: 0x%04X 0x%04X 0x%04X, "
                "calculated: 0x%04X 0x%04X 0x%04X",
                crc_recv.all_, crc_recv.even_, crc_recv.odd_,
                crc_calc.all_, crc_calc.even_, crc_calc.odd_);
            return call_immediately(STATE(resync));
        }

        if (receiver_)
        {
            auto *b = receiver_->alloc();
            b->data()->payload = std::move(payload_);
            receiver_->send(b);
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

        /// @todo Should we be sending out a framing error? I think so. How to
        ///       dispatch that? Maybe an error field in the TxMessage? If we
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
    /// Incoming messages get routed to this object.
    Receiver *receiver_;

    /// Interface fd.
    int fd_ = -1;
};

class CvSpace : public openlcb::MemorySpace, public PacketFlowInterface
{
public:
    CvSpace(PacketFlowInterface *tx)
        : pendingRead_(false)
        , doneRead_(false)
        , pendingWrite_(false)
        , doneWrite_(false)
        , txFlow_(tx)
    { }

    address_t max_address() override
    {
        return 1023;
    }

    /// @returns whether the memory space does not accept writes.
    bool read_only() override
    {
        return false;
    }

    size_t write(address_t destination, const uint8_t *data, size_t len,
        errorcode_t *error, Notifiable *again) override
    {
        if (doneWrite_)
        {
            doneWrite_ = false;
            *error = errorCode_;
            return actualLen_;
        }

        actualLen_ = len;
        pendingWrite_ = true;
        doneWrite_ = false;
        done_ = again;
        *error = ERROR_AGAIN;

        auto *b = txFlow_->alloc();
        b->data()->payload =
            Defs::get_memw_payload(proxySpace_, destination, data, len);
        txFlow_->send(b);
        return 0;
    }

    size_t read(address_t source, uint8_t *dst, size_t len, errorcode_t *error,
        Notifiable *again) override
    {
        if (doneRead_)
        {
            doneRead_ = false;
            *error = errorCode_;
            return actualLen_;
        }

        readBuf_ = dst;
        actualLen_ = len;
        pendingRead_ = true;
        doneRead_ = false;
        done_ = again;
        *error = ERROR_AGAIN;

        auto *b = txFlow_->alloc();
        b->data()->payload = Defs::get_memr_payload(proxySpace_, source, len);
        txFlow_->send(b);
        return 0;
    }

    /// Handles messages coming back from the decoder via the traction modem
    /// protocol.
    void send(Buffer<TxMessage>* buf, unsigned prio) override {
        auto rb = get_buffer_deleter(buf);
        auto& txm = *buf->data();
        if (!txm.valid()) {
            return;
        }
        switch (txm.command())
        {
            case Defs::RESP_MEM_R:
            {
                if (pendingRead_)
                {
                    handle_read_response(txm);
                }
                break;
            }
            case Defs::RESP_MEM_W:
            {
                if (pendingWrite_)
                {
                    handle_write_response(txm);
                }
                break;
            }
        }
    }

    void handle_read_response(TxMessage &txm)
    {
        doneRead_ = true;
        pendingRead_ = false;
        errorCode_ = txm.response_status();
        unsigned data_bytes = txm.length() - 2;
        if (data_bytes > actualLen_)
        {
            // We should not have received more bytes back than we requested,
            // but we still clip.
            data_bytes = actualLen_;
        }
        memcpy(readBuf_, txm.payload.data() + Defs::OFS_DATA + 2, data_bytes);
        actualLen_ = data_bytes;
        done_->notify();
    }

    void handle_write_response(TxMessage& txm) {
        pendingWrite_ = false;
        doneWrite_ = true;
        errorCode_ = txm.response_status();
        if (errorCode_)
        {
            actualLen_ = Defs::get_uint16(txm.payload, Defs::OFS_DATA + 2);
        }
        done_->notify();
    }

    /// This is the memory space we will be using on the decoder.
    uint8_t proxySpace_ = openlcb::MemoryConfigDefs::SPACE_DCC_CV;

    /// true if we are waiting for a read response
    bool pendingRead_ : 1;
    /// true if we the read response arrived
    bool doneRead_ : 1;
    /// true if we are waiting for a write response
    bool pendingWrite_ : 1;
    /// true if we the write response arrived
    bool doneWrite_ : 1;

    /// Returned error code from the backend.
    uint16_t errorCode_ = 0;

    /// Where to put the bytes read.
    uint8_t* readBuf_ = nullptr;
    /// How many bytes to put there. When doneRead_, then the number of bytes
    /// actually read.
    unsigned actualLen_ = 0;

    /// Notifiable to mark when the pending read/write completes.
    Notifiable* done_ = nullptr;

    /// We send outgoing packets to the decoder using this interface.
    PacketFlowInterface* txFlow_;
};

class ModemTrain : public openlcb::TrainImpl
{
public:
    ModemTrain(Service *service)
        : txFlow_(service)
        , rxFlow_(service)
        , isActive_(false)
    {
        rxFlow_.set_listener(&cvSpace_);
    }

    void start(int uart_fd)
    {
        txFlow_.start(uart_fd);
        rxFlow_.start(uart_fd);
    }

    /// Set the active state of the wireless control.
    /// @param is_active true if under wireless control, else false
    void set_is_active(bool is_active)
    {
        if (isActive_ != is_active)
        {
            isActive_ = is_active;
            send_packet(Defs::get_wireless_present_payload(isActive_));
        }
    }

    openlcb::MemorySpace *get_cv_space()
    {
        return &cvSpace_;
    }

    // ====== Train interface =======

    void set_speed(openlcb::SpeedType speed) override
    {
        set_is_active(true);
        inEStop_ = false;
        lastSpeed_ = speed;
        send_packet(Defs::get_speed_set_payload(speed));
    }

    /** Returns the last set speed of the locomotive. */
    openlcb::SpeedType get_speed() override
    {
        return lastSpeed_;
    }

    /** Sets the train to emergency stop. */
    void set_emergencystop() override
    {
        inEStop_ = true;
        lastSpeed_.set_mph(0); // keeps direction
        send_packet(Defs::get_estop_payload());
    }

    bool get_emergencystop() override
    {
        return inEStop_;
    }

    /** Sets the value of a function.
     * @param address is a 24-bit address of the function to set. For legacy DCC
     * locomotives, see @ref TractionDefs for the address definitions (0=light,
     * 1-28= traditional function buttons).
     * @param value is the function value. For binary functions, any non-zero
     * value sets the function to on, zero sets it to off.*/
    void set_fn(uint32_t address, uint16_t value) override
    {
        set_is_active(true);
        send_packet(Defs::get_fn_set_payload(address, value));
        /// @todo The following switch statement is for hardware testing only.
        ///       In production software, the main MCU should have the on/off
        ///       logical by "output" number, and not by function number. The
        ///       main MCU should instruct the modem of the activity state.
        switch (address)
        {
            default:
                break;
#if !defined(GTEST)
            case 0:
                if (lastSpeed_.direction() == openlcb::Velocity::REVERSE)
                {
                    LOGIC_F0R_Pin::set(value ? 1 : 0);
                }
                break;
            case 1:
                LOGIC_F1_Pin::set(value ? 1 : 0);
                break;
            case 2:
                LOGIC_F2_Pin::set(value ? 1 : 0);
                break;
            case 3:
                LOGIC_F3_Pin::set(value ? 1 : 0);
                break;
            case 4:
                LOGIC_F4_Pin::set(value ? 1 : 0);
                break;
            // F5 and F6 also overwritten with input1 and input2 values when
            // button 7 or 8 are pressed
            case 5:
                LOGIC_F5_Pin::set(value ? 1 : 0);
                break;
            case 6:
                LOGIC_F6_Pin::set(value ? 1 : 0);
                break;
#endif // !defined(GTEST)
// Commented out since these pins cannot be defined as both inputs
// and outputs.
#if 0
            case 7:
                LOGIC_F7_Pin::set(value ? 1 : 0);
                break;
            case 8:
                LOGIC_F8_Pin::set(value ? 1 : 0);
                break;
#endif
#if !defined(GTEST)
            case 7:
            {
                bool input1 = INPUT1_Pin::get();
                LOGIC_F5_Pin::set(INPUT2_Pin::get());
                LOG(ALWAYS, "INPUT1: %u", input1);
                break;
            }
            case 8:
            {
                bool input2 = INPUT2_Pin::get();
                LOGIC_F6_Pin::set(INPUT2_Pin::get());
                LOG(ALWAYS, "INPUT2: %u", input2);
                break;
            }
#endif // !defined(GTEST)
        }
    }

    /** @returns the value of a function. */
    uint16_t get_fn(uint32_t address) override
    {
        return 0;
    }

    uint32_t legacy_address() override
    {
        /// @todo what should this be?
        return 883;
    }

    /** @returns the type of legacy protocol in use. */
    dcc::TrainAddressType legacy_address_type() override
    {
        return dcc::TrainAddressType::DCC_LONG_ADDRESS;
    }

private:
    inline void send_packet(Defs::Payload p)
    {
        auto *b = txFlow_.alloc();
        b->data()->payload = std::move(p);
        txFlow_.send(b);
    }

    bool isRunning_ = false;
    /// UART fd to send traffic to the device.
    int fd_;

    openlcb::SpeedType lastSpeed_ = 0.0;
    /// True if the last set was estop, false if it was a speed.
    bool inEStop_ = false;

    TxFlow txFlow_;
    RxFlow rxFlow_;
    CvSpace cvSpace_{&txFlow_};
    bool isActive_;
};

} // namespace tractionmodem

#endif // _TRACTIONMODEM_TRACTIONMODEM_HXX_
