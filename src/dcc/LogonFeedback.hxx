/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file LogonFeedback.hxx
 * Parser for RailCom feedback that recognizes logon messages.
 *
 * @author Balazs Racz
 * @date 12 Aug 2021
 */

#ifndef _DCC_LOGONFEEDBACK_HXX_
#define _DCC_LOGONFEEDBACK_HXX_

#include "dcc/RailcomHub.hxx"
#include "utils/Crc.hxx"

namespace dcc
{

/// Abstract class to get callbacks for recognized feedback messages
class LogonFeedbackCallbacks
{
public:
    enum PacketType
    {
        /// Non-254 packet or not known (not relevant) 254 packet type
        UNKNOWN = 0,
        /// Get Data Start packet
        GET_DATA_START,
        /// Get Data Continue packet
        GET_DATA_CONT,
        /// Logon Enable packet
        LOGON_ENABLE,
        /// Select packet with Get Short Info command
        SELECT_SHORTINFO,
        /// Logon Assign packet
        LOGON_ASSIGN,
        /// Misc 254 packet (ID based responses)
        MISC_254
    };

    /// Determines based on feedback key what the given DCC packet was.
    /// @param feedback_key from the railcom packet.
    /// @return the packet classification wrt the logon feature.
    virtual PacketType classify_packet(uintptr_t feedback_key) = 0;

    /// Handles a Select ShortInfo feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload.
    virtual void process_select_shortinfo(
        uintptr_t feedback_key, bool error, uint64_t data) = 0;

    /// Handles a Logon Assign feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload.
    virtual void process_logon_assign(
        uintptr_t feedback_key, bool error, uint64_t data) = 0;

    /// Handles a Decoder ID feedback message.
    /// @param feedback_key refers to the packet it came from.
    /// @param error true if there was a transmission error or the data came in
    /// incorrect format.
    /// @param data 48 bits of payload. The low 44 bits of this is a decoder ID.
    virtual void process_decoder_id(
        uintptr_t feedback_key, bool error, uint64_t data) = 0;
};

/// Parser for RailCom feedback that recognizes logon messages.
class LogonFeedbackParser : public RailcomHubPortInterface
{
public:
    using PacketType = LogonFeedbackCallbacks::PacketType;
    /// Constructor.
    /// @param cb recognized packets get calls on this object.
    /// @param hub the railcom hub for gathering feedback.
    LogonFeedbackParser(LogonFeedbackCallbacks *cb, RailcomHubFlow *hub)
        : cb_(cb)
        , hub_(hub)
    {
        hub_->register_port(this);
    }

    ~LogonFeedbackParser()
    {
        hub_->unregister_port(this);
    }

    /// Receives railcom feedback.
    void send(Buffer<RailcomHubData> *b, unsigned priority) override
    {
        auto rb = get_buffer_deleter(b);
        PacketType type = cb_->classify_packet(b->data()->feedbackKey);
        if (type == PacketType::UNKNOWN)
        {
            return;
        }
        uint64_t data = parse_code(b->data());
        bool any_error = data & ERROR_MASK;
        auto key = b->data()->feedbackKey;
        switch (type)
        {
            case PacketType::SELECT_SHORTINFO:
                any_error |= has_crc_error(data);
                any_error |= (((data >> 47) & 1) != 1);
                cb_->process_select_shortinfo(
                    key, any_error, data & PAYLOAD_MASK);
                break;
            case PacketType::LOGON_ASSIGN:
                any_error |= has_crc_error(data);
                any_error |=
                    (((data >> 44) & 0xf) != RMOB_LOGON_ASSIGN_FEEDBACK);
                cb_->process_logon_assign(key, any_error, data & PAYLOAD_MASK);
                break;
            case PacketType::LOGON_ENABLE:
                any_error |=
                    (((data >> 44) & 0xf) != RMOB_LOGON_ENABLE_FEEDBACK);
                cb_->process_decoder_id(key, any_error, data & PAYLOAD_MASK);
                break;
            case PacketType::GET_DATA_START:
            case PacketType::GET_DATA_CONT:
            case PacketType::MISC_254:
            case PacketType::UNKNOWN:
            default:
                break;
        }
    }

#ifndef GTEST
private:
#endif
    enum Errors
    {
        /// Which bit offset do the error bits start.
        ERROR_SHIFT = 56,
        /// Mask where the error bits are.
        ERROR_MASK = 0xffULL << ERROR_SHIFT,
        /// Which bit offset do the error bits start.
        LENGTH_SHIFT = 48,
        /// Counts the number of valid 6-bit counts.
        LENGTH_OFFSET = 1ULL << LENGTH_SHIFT,
        /// Which bit offset do the error bits start.
        LENGTH_MASK = 0xffULL << LENGTH_SHIFT,
        /// Mask where the decoded payload is.
        PAYLOAD_MASK = LENGTH_OFFSET - 1,

        /// Not enough bytes in the feedback response.
        ERROR_MISSING_DATA = 1ULL << ERROR_SHIFT,
        /// Found an ACK byte.
        ERROR_ACK = 2ULL << ERROR_SHIFT,
        /// Found valid data past an ACK byte or a missing byte.
        ERROR_OUT_OF_ORDER = 4ULL << ERROR_SHIFT,
        /// Found invalid 6/8 code.
        ERROR_GARBAGE = 8ULL << ERROR_SHIFT,
        /// Found unknown codepoint (e.g. NACK or BUSY).
        ERROR_UNKNOWN = 16ULL << ERROR_SHIFT,
    };

    /// Appends 6 bits of incoming data from railcom.
    /// @param data the 48-bit aggregated data.
    /// @param shift where the next 6 bits should be at (0 to 42)
    /// @param next_byte the next byte from the uart.
    static void append_data(uint64_t &data, unsigned &shift, uint8_t next_byte)
    {
        uint8_t code = railcom_decode[next_byte];
        if (code < 64)
        {
            data |= ((uint64_t)code) << shift;
            if (data >> ERROR_SHIFT)
            {
                // Valid data beyond some error.
                data |= ERROR_OUT_OF_ORDER;
            }
            data += LENGTH_OFFSET;
        }
        else if (code == RailcomDefs::ACK)
        {
            data |= ERROR_ACK;
        }
        else if (code == RailcomDefs::INV)
        {
            data |= ERROR_GARBAGE;
        }
        else
        {
            data |= ERROR_UNKNOWN;
        }
        shift -= 6;
    }

    /// Parses a concatenated ch1+ch2 response into a single uint64_t. It
    /// contains error flags in the MSB, the number of valid 6-bit codepoints
    /// in the second MSB , and 6 bytes data in the LSB, entered with the first
    /// wire byte in the third MSB and last wire byte in the LSB.
    ///
    /// Any input byte that was missing or contained any other codepoint than a
    /// valid 6-bit data codepoint (including garbage or ACK) is translated
    /// into zero bits in the parse result.
    /// @param fb feedback to parse
    /// @return parse result.
    static uint64_t parse_code(const Feedback *fb)
    {
        uint64_t data = 0;
        unsigned shift = 48 - 6;
        for (unsigned i = 0; i < 2; i++)
        {
            if (fb->ch1Size > i)
            {
                append_data(data, shift, fb->ch1Data[i]);
            }
            else
            {
                data |= ERROR_MISSING_DATA;
            }
        }
        for (unsigned i = 0; i < 6; i++)
        {
            if (fb->ch2Size > i)
            {
                append_data(data, shift, fb->ch2Data[i]);
            }
            else
            {
                data |= ERROR_MISSING_DATA;
            }
        }
        return data;
    }

    /// Checks the CRC8 on a 6-byte payload in this feedback.
    /// @param data the feedback bytes (right aligned, first is MSB).
    /// @return true if there is a CRC error
    static bool has_crc_error(uint64_t data)
    {
        Crc8DallasMaxim m;
        for (int i = 48 - 8; i >= 0; i -= 8)
        {
            m.update16((data >> i) & 0xff);
        }
        return !m.check_ok();
    }

private:
    /// Callbacks object.
    LogonFeedbackCallbacks *cb_;
    /// The railcom hub we are registered to.
    RailcomHubFlow *hub_;
};

} // namespace dcc

#endif // _DCC_LOGONFEEDBACK_HXX_
