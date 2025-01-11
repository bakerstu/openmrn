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

#ifndef _WOWW_MAIN_TRACTIONMODEM_HXX_
#define _WOWW_MAIN_TRACTIONMODEM_HXX_

#include "TractionModemDefs.hxx"
#include "hardware.hxx"

#include "executor/StateFlow.hxx"

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

class TxFlow : public TxFlowBase
{
public:
    TxFlow(Service *service)
        : TxFlowBase(service)
    {
        LOG(ALWAYS, "[ModemTx] constructor");
    }

    void start(int uart_fd)
    {
        LOG(ALWAYS, "[ModemTx] fd");
        fd_ = uart_fd;
    }

    Action entry() override
    {
        if (fd_ < 0)
        {
            LOG(ALWAYS, "[ModemTx] no uart");
            return release_and_exit();
        }
        LOG(ALWAYS, "[ModemTx] msg len %d",
            (int)message()->data()->payload.size());
        return write_repeated(&helper_, fd_, message()->data()->payload.data(),
            message()->data()->payload.size(), STATE(write_complete));
    }

    Action write_complete()
    {
        unsigned len = message()->data()->payload.size();
        unsigned num_sent = len - helper_.remaining_;
        const uint8_t *d = (const uint8_t *)message()->data()->payload.data();
        LOG(ALWAYS, "[ModemTx] sent E%d len %u done %u %08x %04x...",
            helper_.hasError_, len, num_sent, *(unsigned *)(d + 0),
            (unsigned)be32toh(*(uint16_t *)(d + 4)));
        /// @TODO check for error
        return release_and_exit();
    }

private:
    StateFlowSelectHelper helper_ {this};
    /// UART fd.
    int fd_ = -1;
};

class RxFlow : public StateFlowBase
{
public:
    using BufferType = Buffer<TxMessage>;
    using Receiver = PacketFlowInterface;

    RxFlow(Service *service)
        : StateFlowBase(service)
    { }

    void start(int uart_fd)
    {
        fd_ = uart_fd;
        start_flow(STATE(wait_for_data));
    }

    void set_listener(Receiver *rcv)
    {
        receiver_ = rcv;
    }

    Action wait_for_data()
    {
        payload_.clear();
        if (!overflow_.empty())
        {
            payload_ = std::move(overflow_);
            overflow_.clear();
            return call_immediately(STATE(expand_header));
        }
        return read_single(
            &helper_, fd_, buf_, sizeof(buf_), STATE(data_received));
    }

    Action data_received()
    {
        unsigned num_received = sizeof(buf_) - helper_.remaining_;
        // Ignores received data.
        LOG(ALWAYS, "[ModemRx] recvd len=%u data=%08x cmd=%04x...",
            num_received, (unsigned)be32toh(*(unsigned *)buf_),
            be16toh((((uint16_t *)buf_)[2])));
        payload_.append((char *)buf_, num_received);
        return call_immediately(STATE(expand_header));
    }

    Action expand_header()
    {
        if (payload_.size() >= HEADER_SIZE)
        {
            return call_immediately(STATE(header_complete));
        }
        return read_repeated_with_timeout(&helper_,
            (MIN_PACKET_SIZE + 10) * CHARACTER_NSEC, fd_, buf_,
            HEADER_SIZE - payload_.size(), STATE(min_read_done));
    }

    Action min_read_done()
    {
        unsigned tried_read = HEADER_SIZE - payload_.size();
        unsigned num_received = tried_read - helper_.remaining_;
        payload_.append((char *)buf_, num_received);
        if (payload_.size() >= MIN_PACKET_SIZE)
        {
            return call_immediately(STATE(header_complete));
        }
        // else we have an error.
        return call_immediately(STATE(resync));
    }

    Action header_complete()
    {
        if ((*(uint32_t *)payload_.data()) != htobe32(Defs::PREAMBLE))
        {
            return call_immediately(STATE(resync));
        }
        uint16_t len = be16toh(*(uint16_t *)(&payload_[6]));
        if (len > 512)
        {
            // This is garbage. The protocol only allows length of 512.
            return call_immediately(STATE(resync));
        }
        unsigned total_length = Defs::LEN_BASE + len;
        if (payload_.size() > total_length)
        {
            // We read too much.
            overflow_ = payload_.substr(total_length);
            payload_.resize(total_length);
        }
        if (payload_.size() == total_length)
        {
            return call_immediately(STATE(packet_complete));
        }
        unsigned offset = payload_.size();
        payload_.resize(total_length);
        return read_repeated_with_timeout(&helper_,
            total_length * CHARACTER_NSEC * 2, fd_, &payload_[offset],
            total_length - offset, STATE(header_complete));
    }

    Action resync()
    {
        // Packet out of sync.
        for (unsigned idx = 1; idx < payload_.size(); ++idx)
        {
            if (payload_[idx] == Defs::PREAMBLE_FIRST)
            {
                if (idx + 4 < payload_.size())
                {
                    uint32_t p;
                    memcpy(&p, payload_.data() + idx, 4);
                    if (p != htobe32(Defs::PREAMBLE))
                    {
                        continue;
                    }
                }
                payload_.erase(0, idx);
                return call_immediately(STATE(expand_header));
            }
        }
        // Now: we're out of sync and never found a viable first byte. Drop
        // all data.
        return call_immediately(STATE(wait_for_data));
    }

    Action packet_complete()
    {
        LOG(ALWAYS, "[Rx] %s", string_to_hex(payload_).c_str());
        if (receiver_)
        {
            auto *b = receiver_->alloc();
            b->data()->payload = std::move(payload_);
            receiver_->send(b);
        }
        return call_immediately(STATE(wait_for_data));
    }

private:
    static constexpr unsigned CHARACTER_NSEC = 10 * SEC_TO_NSEC(1) / 250000;
    static constexpr unsigned HEADER_SIZE = Defs::LEN_HEADER;
    static constexpr unsigned MIN_PACKET_SIZE = Defs::LEN_BASE;

    /// Time when we received the start of this packet.
    long long packetStartNsec_ = 0;
    /// Helper for reading in a select flow.
    StateFlowTimedSelectHelper helper_ {this};
    /// Raw reads come into this buffer.
    uint8_t buf_[64];
    /// We assemble the packet here.
    string payload_;
    /// If we overread the packet, the remaining data is here.
    string overflow_;
    /// Incoming packets get routed to this object.
    Receiver *receiver_;

    /// UART fd.
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

#endif // _WOWW_MAIN_TRACTIONMODEM_HXX_
