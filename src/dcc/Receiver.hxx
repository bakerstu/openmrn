/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file Receiver.hxx
 *
 * State flow for DCC packet reception and decoding.
 *
 * @author Balazs Racz
 * @date 30 November 2014
 */

#ifndef _DCC_RECEIVER_HXX_
#define _DCC_RECEIVER_HXX_

#include <fcntl.h>
#include <unistd.h>

#include "executor/StateFlow.hxx"

#ifdef __FreeRTOS__
#include "freertos/can_ioctl.h"
#else
#include "can_ioctl.h"
#endif
#include "freertos_drivers/common/SimpleLog.hxx"
#include "dcc/packet.h"
#include "utils/Crc.hxx"

// If defined, collects samples of timing and state into a ring buffer.
//#define DCC_DECODER_DEBUG

namespace dcc
{

/// State machine for decoding a DCC packet flow. Supports both DCC and
/// Marklin-Motorola packets.
class DccDecoder
{
public:
    /// @param tick_per_usec specifies how many timer capture ticks happen per
    /// usec. The default value assumes the timer does not have a prescaler.
    DccDecoder(unsigned tick_per_usec)
    {
        timings_[DCC_ONE].set(tick_per_usec, 52, 64);
        timings_[DCC_ZERO].set(tick_per_usec, 95, 9900);
        timings_[MM_PREAMBLE].set(tick_per_usec, 1000, -1);
        timings_[MM_SHORT].set(tick_per_usec, 20, 32);
        timings_[MM_LONG].set(tick_per_usec, 200, 216);
    }

    /// Internal states of the decoding state machine.
    enum State : uint8_t
    {
        UNKNOWN,             // 0
        DCC_PREAMBLE,        // 1
        DCC_END_OF_PREAMBLE, // 2
        DCC_DATA,            // 3
        DCC_DATA_ONE,        // 4
        DCC_DATA_ZERO,       // 5
        DCC_MAYBE_CUTOUT,    // 6
        DCC_CUTOUT,          // 7
        DCC_PACKET_FINISHED, // 8
        MM_DATA,
        MM_ZERO,
        MM_ONE,
        MM_PACKET_FINISHED,
    };

    /// @return the current decoding state.
    State state()
    {
        return parseState_;
    }

    /// Sets where to write the decoded DCC data to. If this function is not
    /// called before a packet preamble starts, the packet will be discarded.
    /// @param pkt where to store the incoming payload bytes. This packet will
    /// be reset to an empty packet.
    void set_packet(DCCPacket *pkt)
    {
        pkt_ = pkt;
        if (pkt_)
        {
            clear_packet();
        }
    }

    /// Retrieves the last applied DCC packet pointer.
    /// @return DCC packet, or nullptr if there was no DCC packet pointer
    /// assigned yet.
    DCCPacket *pkt()
    {
        return pkt_;
    };

    /// Call this function for each time the polarity of the signal changes.
    /// @param value is the number of clock cycles since the last polarity
    /// change.
    void process_data(uint32_t value)
    {
#ifdef DCC_DECODER_DEBUG
        debugLog_.add(value);
        debugLog_.add(parseState_);
#endif
        switch (parseState_)
        {
            case DCC_PACKET_FINISHED:
            case MM_PACKET_FINISHED:
            case UNKNOWN:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseCount_ = 0;
                    parseState_ = DCC_PREAMBLE;
                    return;
                }
                if (timings_[MM_PREAMBLE].match(value) && pkt_)
                {
                    clear_packet();
                    pkt_->packet_header.is_marklin = 1;
                    parseCount_ = 1 << 2;
                    parseState_ = MM_DATA;
                    havePacket_ = true;
                }
                break;
            }
            case DCC_PREAMBLE:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseCount_++;
                    return;
                }
                if (timings_[DCC_ZERO].match(value) && (parseCount_ >= 20))
                {
                    parseState_ = DCC_END_OF_PREAMBLE;
                    return;
                }
                break;
            }
            case DCC_END_OF_PREAMBLE:
            {
                if (timings_[DCC_ZERO].match(value))
                {
                    parseState_ = DCC_DATA;
                    parseCount_ = 1 << 7;
                    xorState_ = 0;
                    crcState_.init();
                    if (pkt_)
                    {
                        clear_packet();
                        havePacket_ = true;
                        pkt_->packet_header.skip_ec = 1;
                    }
                    else
                    {
                        havePacket_ = false;
                    }
                    return;
                }
                break;
            }
            case DCC_DATA:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    parseState_ = DCC_DATA_ONE;
                    return;
                }
                if (timings_[DCC_ZERO].match(value))
                {
                    parseState_ = DCC_DATA_ZERO;
                    return;
                }
                break;
            }
            case DCC_DATA_ONE:
            {
                if (timings_[DCC_ONE].match(value))
                {
                    if (parseCount_)
                    {
                        if (havePacket_)
                        {
                            pkt_->payload[pkt_->dlc] |= parseCount_;
                        }
                        parseCount_ >>= 1;
                        parseState_ = DCC_DATA;
                        return;
                    }
                    else
                    {
                        // end of packet 1 bit.
                        if (havePacket_)
                        {
                            if (checkCRC_ && (pkt_->dlc > 6) &&
                                !crcState_.check_ok())
                            {
                                pkt_->packet_header.csum_error = 1;
                            }
                            xorState_ ^= pkt_->payload[pkt_->dlc];
                            if (xorState_)
                            {
                                pkt_->packet_header.csum_error = 1;
                            }
                            pkt_->dlc++;
                        }
                        parseState_ = DCC_MAYBE_CUTOUT;
                        return;
                    }
                    return;
                }
                break;
            }
            case DCC_DATA_ZERO:
            {
                if (timings_[DCC_ZERO].match(value))
                {
                    if (parseCount_)
                    {
                        // zero bit into data_.
                        parseCount_ >>= 1;
                    }
                    else
                    {
                        // end of byte zero bit. Packet is not finished yet.
                        if (havePacket_)
                        {
                            xorState_ ^= pkt_->payload[pkt_->dlc];
                            if ((pkt_->dlc == 0) &&
                                (pkt_->payload[0] == 254 ||
                                    pkt_->payload[0] == 253))
                            {
                                checkCRC_ = true;
                            }
                            if (checkCRC_)
                            {
                                crcState_.update16(pkt_->payload[pkt_->dlc]);
                            }
                            pkt_->dlc++;
                            if (pkt_->dlc >= DCC_PACKET_MAX_PAYLOAD)
                            {
                                havePacket_ = false;
                            }
                            else
                            {
                                pkt_->payload[pkt_->dlc] = 0;
                            }
                        }
                        parseCount_ = 1 << 7;
                    }
                    parseState_ = DCC_DATA;
                    return;
                }
                break;
            }
            case DCC_MAYBE_CUTOUT:
            {
                if (value < timings_[DCC_ZERO].min_value)
                {
                    parseState_ = DCC_CUTOUT;
                    return;
                }
                parseState_ = DCC_PACKET_FINISHED;
                return;
            }
            case DCC_CUTOUT:
            {
                parseState_ = DCC_PACKET_FINISHED;
                return;
            }
            case MM_DATA:
            {
                if (timings_[MM_LONG].match(value))
                {
                    parseState_ = MM_ZERO;
                    return;
                }
                if (timings_[MM_SHORT].match(value))
                {
                    parseState_ = MM_ONE;
                    return;
                }
                break;
            }
            case MM_ZERO:
            {
                if (timings_[MM_SHORT].match(value))
                {
                    // data_[ofs_] |= 0;
                    parseCount_ >>= 1;
                    if (!parseCount_)
                    {
                        if (pkt_->dlc == 2)
                        {
                            parseState_ = MM_PACKET_FINISHED;
                            return;
                        }
                        else
                        {
                            pkt_->dlc++;
                            parseCount_ = 1 << 7;
                            pkt_->payload[pkt_->dlc] = 0;
                        }
                    }
                    parseState_ = MM_DATA;
                    return;
                }
                break;
            }
            case MM_ONE:
            {
                if (timings_[MM_LONG].match(value))
                {
                    pkt_->payload[pkt_->dlc] |= parseCount_;
                    parseCount_ >>= 1;
                    if (!parseCount_)
                    {
                        if (pkt_->dlc == 2)
                        {
                            parseState_ = MM_PACKET_FINISHED;
                            return;
                        }
                        else
                        {
                            pkt_->dlc++;
                            parseCount_ = 1 << 7;
                            pkt_->payload[pkt_->dlc] = 0;
                        }
                    }
                    parseState_ = MM_DATA;
                    return;
                }
                break;
            }
        }
        parseState_ = UNKNOWN;
        return;
    }

    /// Returns true if we are close to the DCC cutout. This situation is
    /// recognized by having seen the first half of the end-of-packet one bit.
    bool before_dcc_cutout() {
        return (!parseCount_) &&           // end of byte
            (parseState_ == DCC_DATA_ONE); // one bit comes
    }

private:
    /// Counter that works through bit patterns.
    uint8_t parseCount_ = 0;
    /// True if we have storage in the right time for the current packet.
    uint8_t havePacket_ : 1;
    /// True if we need to check CRC
    uint8_t checkCRC_ : 1;
    /// Checksum state for XOR checksum.
    uint8_t xorState_;
    /// Checksum state for CRC8 checksum.
    Crc8DallasMaxim crcState_;
    /// State machine for parsing.
    State parseState_ = UNKNOWN;
    /// Storage for the current packet.
    DCCPacket* pkt_ = nullptr;

    /// Sets the input DCC packet to empty.
    void clear_packet()
    {
        pkt_->header_raw_data = 0;
        pkt_->dlc = 0;
        pkt_->feedback_key = 0;
        pkt_->payload[0] = 0;
        checkCRC_ = 0;
        xorState_ = 0;
        crcState_.init();
    }

    /// Represents the timing of a half-wave of the digital track signal.
    struct Timing
    {
        void set(uint32_t tick_per_usec, int min_usec, int max_usec)
        {
            if (min_usec < 0)
            {
                min_value = 0;
            }
            else
            {
                min_value = tick_per_usec * min_usec;
            }
            if (max_usec < 0)
            {
                max_usec = UINT_MAX;
            }
            else
            {
                max_value = tick_per_usec * max_usec;
            }
        }

        bool match(uint32_t value_clocks) const
        {
            return min_value <= value_clocks && value_clocks <= max_value;
        }

        uint32_t min_value;
        uint32_t max_value;
    };

    /// Indexes the timing array.
    enum TimingInfo
    {
        DCC_ONE = 0,
        DCC_ZERO,
        MM_PREAMBLE,
        MM_SHORT,
        MM_LONG,
        MAX_TIMINGS
    };
    /// The various timings by the standards.
    Timing timings_[MAX_TIMINGS];
#ifdef DCC_DECODER_DEBUG
    LogRing<uint16_t, 256> debugLog_;
#endif
};

/// User-space DCC decoding flow. This flow receives a sequence of numbers from
/// the DCC driver, where each number means a specific number of microseconds
/// for which the signal was of the same polarity (e.g. for dcc packet it would
/// be something like 56, 56, 56, 56, ..., 56, 56, 105, 105, 56, 56, ...). Then
/// calls for each packet the virtual function dcc_packet_finished() or
/// mm_packet_finished().
///
/// This flow is a pretty expensive way to decode DCC data.
class DccDecodeFlow : public StateFlowBase
{
public:
    DccDecodeFlow(Service *s, const char *dev)
        : StateFlowBase(s)
    {
        fd_ = ::open(dev, O_RDONLY | O_NONBLOCK);
        decoder_.set_packet(&pkt_);
        start_flow(STATE(register_and_sleep));
    }

private:
    Action register_and_sleep()
    {
        ::ioctl(fd_, CAN_IOC_READ_ACTIVE, this);
        return wait_and_call(STATE(data_arrived));
    }

    Action data_arrived()
    {
        while (true)
        {
            uint32_t value;
            int ret = ::read(fd_, &value, sizeof(value));
            if (ret != 4)
            {
                return call_immediately(STATE(register_and_sleep));
            }
            debug_data(value);
            decoder_.process_data(value);
            if (decoder_.state() == DccDecoder::DCC_PACKET_FINISHED)
            {
                dcc_packet_finished(pkt_.payload, pkt_.dlc);
            }
            else if (decoder_.state() == DccDecoder::MM_PACKET_FINISHED)
            {
                mm_packet_finished(pkt_.payload, pkt_.dlc);
            }

            static uint8_t x = 0;
            // MAP_GPIOPinWrite(LED_GREEN, 0);
            x = ~x;
        }
    }

    virtual void dcc_packet_finished(const uint8_t* payload, size_t len) = 0;
    virtual void mm_packet_finished(const uint8_t* payload, size_t len) = 0;
    virtual void debug_data(uint32_t value)
    {
    }

    int fd_;
    uint32_t lastValue_ = 0;

protected:
    /// Packet buffer.
    DCCPacket pkt_;
    /// State machine that does the DCC decoding. We have 1 usec per tick, as
    /// these are the numbers we receive from the driver.
    DccDecoder decoder_ {1};
};

} // namespace dcc

#endif // _DCC_RECEIVER_HXX_
