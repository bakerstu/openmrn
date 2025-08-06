/** @copyright
 * Copyright (c) 2016, Balazs Racz
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
 * @file Defs.hxx
 * Helper class for creating delayed initialized objects in static memory.
 *
 * @author Balazs Racz
 * @date 24 June 2017
 */

#ifndef _TRACTION_MODEM_DEFS_HXX_
#define _TRACTION_MODEM_DEFS_HXX_

#include <string>

#include "openlcb/Velocity.hxx"
#include "utils/Crc.hxx"

namespace traction_modem
{

/// Useful definitions for the traction modem.
struct Defs
{
    using Payload = std::string;

    /// Every command starts with these bytes.
    static constexpr uint32_t PREAMBLE = 0x41d2c37a;
    /// First byte of the preamble
    static constexpr char PREAMBLE_FIRST = 0x41;

    /// The most significant bit of each command means "response" to a previous
    /// message.
    static constexpr uint16_t RESPONSE = 0x8000;

    /// Normal (default) response timeout.
    static constexpr long long RESP_TIMEOUT = SEC_TO_NSEC(3);

    /// Shortened response timeout, typically used in the link establishment
    /// phase.
    static constexpr long long RESP_TIMEOUT_SHORT = SEC_TO_NSEC(1);

    /// Time to wait between sending the next ping. This must be comfortably
    /// below RESP_TIMEOUT, otherwise the ping could be sent too late resulting
    /// in link down.
    static constexpr long long PING_TIMEOUT = SEC_TO_NSEC(2);

    /// Command values.
    enum Command : uint16_t
    {
        CMD_PING               = 0x0000, ///< ping
        CMD_NOP                = 0x0001, ///< no-operation (do nothing)
        CMD_REBOOT             = 0x0002, ///< reboot request
        CMD_BAUD_RATE_QUERY    = 0x0003, ///< query the supported baud rates
        CMD_BAUD_RATE_REQUEST  = 0x0004, ///< request a specific baud rate
        CMD_SPEED_SET          = 0x0100, ///< set velocity
        CMD_FN_SET             = 0x0101, ///< set function
        CMD_ESTOP_SET          = 0x0102, ///< emergency stop request
        CMD_SPEED_QUERY        = 0x0110, ///< query current speed
        CMD_FN_QUERY           = 0x0111, ///< query function status
        CMD_DC_DCC_PRESENT     = 0x0200, ///< DC/DCC present
        CMD_WIRELESS_PRESENT   = 0x0201, ///< wireless present
        CMD_OUTPUT_STATE       = 0x0300, ///< command the output state
        CMD_OUTPUT_STATE_QUERY = 0x0301, ///< query the output state
        CMD_OUTPUT_RESTART     = 0x0302, ///< synchronize/restart the output
        CMD_MEM_R              = 0x1000, ///< memory read
        CMD_MEM_W              = 0x1001, ///< memory write

        //
        // response commands
        //
        /// ping response
        RESP_PING               = RESPONSE | CMD_PING,
        /// baud rate query response
        RESP_BAUD_RATE_QUERY    = RESPONSE | CMD_BAUD_RATE_QUERY,
        /// set velocity response
        RESP_SPEED_SET          = RESPONSE | CMD_SPEED_SET,
        /// set function response
        RESP_FN_SET             = RESPONSE | CMD_FN_SET,
        /// emergency stop response
        RESP_ESTOP_SET          = RESPONSE | CMD_ESTOP_SET,
        /// query current speed response
        RESP_SPEED_QUERY        = RESPONSE | CMD_SPEED_QUERY,
        /// query function status response
        RESP_FN_QUERY           = RESPONSE | CMD_FN_QUERY,
        /// DC/DCC present response
        RESP_DC_DCC_PRESENT     = RESPONSE | CMD_DC_DCC_PRESENT,
        /// wireless present response
        RESP_WIRELESS_PRESENT   = RESPONSE | CMD_WIRELESS_PRESENT,
        /// Output state query response
        RESP_OUTPUT_STATE_QUERY = RESPONSE | CMD_OUTPUT_STATE_QUERY,
        /// memory read response
        RESP_MEM_R              = RESPONSE | CMD_MEM_R,
        /// memory write response
        RESP_MEM_W              = RESPONSE | CMD_MEM_W,
    };

    /// Reboot command argument options.
    enum RebootArg : uint8_t
    {
        BOOT         = 0, ///< reboot into the bootloader
        APP          = 1, ///< reboot into the application
        APP_VALIDATE = 2, ///< reboot into the application after full validation
    };

    enum BaudRates : uint16_t
    {
        BAUD_NONE      = 0x0000, ///< no baud rate
        BAUD_5M_MASK   = 0x0200, ///< 5 Mbps
        BAUD_4M_MASK   = 0x0400, ///< 4 Mbps
        BAUD_3M_MASK   = 0x0800, ///< 3 Mbps
        BAUD_2M_MASK   = 0x1000, ///< 2 Mbps
        BAUD_1M_MASK   = 0x2000, ///< 1 Mbps
        BAUD_500K_MASK = 0x4000, ///< 500 Kbps
        BAUD_250K_MASK = 0x8000, ///< 250 Kbps
    };

    /// Length of a the header. 4 bytes preamble, 2 bytes cmd, 2 bytes length.
    static constexpr unsigned LEN_HEADER = 4+2+2;

    /// Length of a zero-payload packet. 4 bytes preamble, 2 bytes cmd, 2 bytes
    /// length, 6 bytes CRC.
    static constexpr unsigned LEN_BASE = 14;
    /// Length of the data payload of a ping packet.
    static constexpr unsigned LEN_PING = 0;
    /// Length of the data payload of a reboot packet.
    static constexpr unsigned LEN_REBOOT = 1;
    /// Length of the data payload of a buad rate query packet.
    static constexpr unsigned LEN_BAUD_RATE_QUERY = 0;
    /// Length of the data payload of a buad rate query response packet.
    static constexpr unsigned LEN_BAUD_RATE_QUERY_RESP = 2;
    /// Length of the data payload of a buad rate request packet.
    static constexpr unsigned LEN_BAUD_RATE_REQUEST = 2;
    /// Length of the data payload of a set function packet.
    static constexpr unsigned LEN_FN_SET = 6;
    /// Length of the data payload of a set speed packet.
    static constexpr unsigned LEN_SPEED_SET = 1;
    /// Length of the data payload of a set estop packet.
    static constexpr unsigned LEN_ESTOP_SET = 0;
    /// Length of the data payload of a wireless present packet.
    static constexpr unsigned LEN_WIRELESS_PRESENT = 1;
    /// Length of the data payload of an output state packet.
    static constexpr unsigned LEN_OUTPUT_STATE = 4;
    /// Length of the data payload of an output state queue packet.
    static constexpr unsigned LEN_OUTPUT_STATE_QUERY = 2;
    /// Length of the data payload of an output state queue response packet.
    static constexpr unsigned LEN_OUTPUT_STATE_QUERY_RESP = 6;
    /// Length of the data payload of an output restart packet.
    static constexpr unsigned LEN_OUTPUT_RESTART = 2;
    /// Length of the data payload of a set estop packet.
    static constexpr unsigned LEN_MEM_R = 6;
    /// Base length of the data, add the number of payload bytes.
    static constexpr unsigned LEN_MEM_W = 5;

    /// Maximum allowed len value.
    static constexpr unsigned MAX_LEN = 512;
    /// Maximum allowed length of the user data portion of a read.
    static constexpr size_t MAX_READ_DATA_LEN = 256;
    /// Maximum allowed length of the user data portion of a write.
    static constexpr size_t MAX_WRITE_DATA_LEN = 256;

    /// Offset of the command in the packet.
    static constexpr unsigned OFS_CMD = 4;
    /// Offset of the length in the packet.
    static constexpr unsigned OFS_LEN = 6;
    /// Offset of the first data byte in the packet.
    static constexpr unsigned OFS_DATA = 8;

    /// The header of a message.
    struct Header
    {
        uint32_t preamble_; ///< packet preamble
        uint16_t command_; ///< message command
        uint16_t length_; ///< message length
    };

    /// The definition of a message.
    struct Message
    {
        Header header_; ///< packet header
        uint8_t data[0]; ///< start of the message data
    };

    // CRC definition of a message.
    struct CRC
    {
        union
        {
            uint16_t crc[3];
            struct
            {
                uint16_t all_; ///< CRC of all bytes
                uint16_t even_; ///< CRC of even bytes
                uint16_t odd_; ///< CRC of odd bytes
            };
        };

        /// Overload == operator.
        /// @param c comparison value
        /// @return true if equal, else false
        bool operator==(const CRC &c)
        {
            return (all_ == c.all_ && even_ == c.even_ && odd_ == c.odd_);
        }

        /// Overload != operator.
        /// @param c comparison value
        /// @return true if not equal, else false
        bool operator!=(const CRC &c)
        {
            return !(all_ == c.all_ && even_ == c.even_ && odd_ == c.odd_);
        }
    };

    /// Structure of a baud rate query response.
    struct BaudRateQueryResponse
    {
        Header header_; ///< packet header
        uint16_t rates_; ///< supported baud rates
        uint8_t end; ///< used for alignment and length validation only
    };
    static_assert(
        offsetof(BaudRateQueryResponse, end) ==
            (LEN_HEADER + LEN_BAUD_RATE_QUERY_RESP),
        "BuadRateQueryResponse struct length or alignment mismatch.");
    static_assert(std::is_standard_layout<BaudRateQueryResponse>::value == true);

    /// Structure of an output state command
    struct OutputState
    {
        Header header_; ///< packet header
        uint16_t output_; ///< output number
        uint16_t effect_; ///< 0 = off, 0xFFFF = on, else effect
        uint8_t end; ///< used for alignment and length validation only
    };
    static_assert(offsetof(OutputState, end) == (LEN_HEADER + LEN_OUTPUT_STATE),
        "OutputState struct length or alignment mismatch.");
    static_assert(std::is_standard_layout<OutputState>::value == true);

    /// Structure of an output state query response
    struct OutputStateQueryResponse
    {
        Header header_; ///< packet header
        uint16_t error_; ///< error code
        uint16_t output_; ///< output number
        uint16_t effect_; ///< 0 = off, 0xFFFF = on, else effect
        uint8_t end; ///< used for alignment and length validation only
    };
    static_assert(
        offsetof(OutputStateQueryResponse, end) == (LEN_HEADER + LEN_OUTPUT_STATE_QUERY_RESP),
        "OutputStateQueryResponse struct length or alignment mismatch.");
    static_assert(
        std::is_standard_layout<OutputStateQueryResponse>::value == true);

    /// Structure of an output restart command (synchronize lighting effect)
    struct OutputRestart
    {
        Header header_; ///< packet header
        uint16_t output_; ///< output number
        uint8_t end; ///< used for alignment and length validation only
    };
    static_assert(offsetof(OutputRestart, end) == (LEN_HEADER + LEN_OUTPUT_RESTART),
        "OutputRestart struct length or alignment mismatch.");
    static_assert(std::is_standard_layout<OutputRestart>::value == true);

    /// Structure of a read reply packet
    struct ReadResponse
    {
        Header header_; ///< packet header
        uint16_t error_; ///< error code
        uint8_t data_[0];
    };
    static_assert(std::is_standard_layout<ReadResponse>::value == true);

    /// Structure of a read reply packet
    struct WriteResponse
    {
        Header header_; ///< packet header
        uint16_t error_; ///< error code
        uint16_t bytesWritten_; ///< length in number of bytes actually written
    };
    static_assert(std::is_standard_layout<WriteResponse>::value == true);

    /// Computes the payload for a ping message.
    static Payload get_ping_payload()
    {
        Payload p;
        prepare(&p, CMD_PING, LEN_PING);

        append_crc(&p);
        return p;
    }

    /// Computes the payload for a ping message.
    static Payload get_baud_rate_query_payload()
    {
        Payload p;
        prepare(&p, CMD_BAUD_RATE_QUERY, LEN_BAUD_RATE_QUERY);

        append_crc(&p);
        return p;
    }

    /// Computes the payload for a reboot message.
    /// @param arg type of reboot
    /// @return wire formatted payload
    static Payload get_reboot_payload(RebootArg arg)
    {
        Payload p;
        prepare(&p, CMD_REBOOT, LEN_REBOOT);
        append_uint8(&p, arg);

        append_crc(&p);
        return p;
    }

    /// Computes payload for the wireless present message.
    /// @param is_present true if "present", else false
    /// @return wire formatted payload
    static Payload get_wireless_present_payload(bool is_present)
    {
        Payload p;
        prepare(&p, CMD_WIRELESS_PRESENT, LEN_WIRELESS_PRESENT);
        append_uint8(&p, is_present ? 1 : 0);

        append_crc(&p);
        return p;
    }

    /// Computes the payload to query for the current output state
    /// @param output output number
    /// @return wire formatted payload
    static Payload get_output_state_query_payload(uint16_t output)
    {
        Payload p;
        prepare(&p, CMD_OUTPUT_STATE_QUERY, LEN_OUTPUT_STATE_QUERY);
        append_uint16(&p, output);

        append_crc(&p);
        return p;
    }

    /// Computes payload to set a function.
    /// @param fn function number
    /// @param value function value. For binary functions, 0 is off, 1 is on.
    /// @return wire formatted payload
    static Payload get_fn_set_payload(unsigned fn, uint16_t value)
    {
        Payload p;
        prepare(&p, CMD_FN_SET, LEN_FN_SET);
        append_uint32(&p, fn);
        append_uint16(&p, value);

        append_crc(&p);
        return p;
    }

    /// Computes payload to set speed and direction.
    /// @param v speed and direction data.
    /// @return wire formatted payload
    static Payload get_speed_set_payload(openlcb::Velocity v)
    {
        Payload p;
        prepare(&p, CMD_SPEED_SET, LEN_SPEED_SET);
        append_uint8(&p, v.get_dcc_128());

        append_crc(&p);
        return p;
    }

    /// Computes payload to set estop.
    /// @return wire formatted payload
    static Payload get_estop_payload()
    {
        Payload p;
        prepare(&p, CMD_ESTOP_SET, LEN_ESTOP_SET);
        // no data in the payload
        append_crc(&p);
        return p;
    }

    /// Computes payload to read some data.
    /// @param space address space
    /// @param address address offset within address space
    /// @param count size of read data requested in bytes
    /// @return wire formatted payload
    static Payload get_memr_payload(
        uint8_t space, uint32_t address, uint8_t count)
    {
        Payload p;
        prepare(&p, CMD_MEM_R, LEN_MEM_R);
        append_uint32(&p, address);
        append_uint8(&p, space);
        append_uint8(&p, count);

        append_crc(&p);
        return p;
    }

    /// Computes payload to write some data.
    /// @param space address space
    /// @param address address offset within address space
    /// @param data data to write
    /// @return wire formatted payload
    static Payload get_memw_payload(
        uint8_t space, uint32_t address, const std::string& data)
    {
        return get_memw_payload(space, address, (const uint8_t*)data.data(), data.size());
    }

    /// Computes payload to write some data.
    /// @param space address space
    /// @param address address offset within address space
    /// @param buf data to write
    /// @param count size of data to write in bytes
    /// @return wire formatted payload
    static Payload get_memw_payload(
        uint8_t space, uint32_t address, const uint8_t* buf, size_t count)
    {
        Payload p;
        prepare(&p, CMD_MEM_W, LEN_MEM_W + count);
        append_uint32(&p, address);
        append_uint8(&p, space);
        p.append((char*)buf, count);

        append_crc(&p);
        return p;
    }

    /// Appends an uint32_t in network byte order to the payload.
    /// @param p wire formatted payload to append to
    /// @param v value to append
    static void append_uint32(Payload *p, uint32_t v)
    {
        p->push_back(v >> 24);
        p->push_back((v >> 16) & 0xff);
        p->push_back((v >> 8) & 0xff);
        p->push_back((v)&0xff);
    }

    /// Appends an uint16_t in network byte order to the payload.
    /// @param p wire formatted payload to append to
    /// @param v value to append
    static void append_uint16(Payload *p, uint16_t v)
    {
        p->push_back((v >> 8) & 0xff);
        p->push_back((v)&0xff);
    }

    /// Appends an uint8_t to the payload.
    /// @param p wire formatted payload to append to
    /// @param v value to append
    static void append_uint8(Payload *p, uint8_t v)
    {
        p->push_back((v)&0xff);
    }

    /// Prepares the header of a packet.
    /// @param p packet buffer.
    /// @param cmd command to add to header.
    /// @param len length of data payload that will come after the header.
    static void prepare(Payload *p, Command cmd, uint16_t len)
    {
        p->reserve(LEN_BASE + len);
        append_uint32(p, PREAMBLE);
        append_uint16(p, cmd);
        append_uint16(p, len);
    }

    /// Computes and appends the CRC to the payload.
    /// @param p wire formatted payload, including prepended preamble, to
    ///        append to
    static void append_crc(Payload *p)
    {
        CRC crc;
        crc3_crc16_ccitt(p->data() + sizeof(uint32_t),
            p->size() - sizeof(uint32_t), crc.crc);
        append_uint16(p, crc.all_);
        append_uint16(p, crc.even_);
        append_uint16(p, crc.odd_);
    }

    /// Extract uint32_t value from a given offset in a wire formatted payload.
    /// @param p wire formatted payload
    /// @param ofs byte offset to start from for extraction
    /// @return extracted value
    static uint32_t get_uint32(const Payload &p, unsigned ofs)
    {
        uint32_t ret;
        if ((ofs + 4) <= p.size())
        {
            memcpy(&ret, p.data() + ofs, 4);
            return be32toh(ret);
        }
        return 0;
    }

    /// Extract uint16_t value from a given offset in a wire formatted payload.
    /// @param p wire formatted payload
    /// @param ofs byte offset to start from for extraction
    /// @return extracted value
    static uint16_t get_uint16(const Payload &p, unsigned ofs)
    {
        uint16_t ret;
        if ((ofs + 2) <= p.size())
        {
            memcpy(&ret, p.data() + ofs, 2);
            return be16toh(ret);
        }
        return 0;
    }

    /// Extract the CRC value(s) from a given payload. The assumption is that
    /// p contains a valid (correctly formatted) message. Additional partial
    /// or complete messages contained within p are ignored.
    /// @param p wire formatted payload
    /// @param length length field from the Header in host endianess (length
    ///        in bytes of the message data)
    static const CRC get_crc(const Payload &p, uint16_t length)
    {
        CRC result;
        result.all_ = get_uint16(p, length + sizeof(Header));
        result.even_ = get_uint16(p, length + sizeof(Header) + 2);
        result.odd_ = get_uint16(p, length + sizeof(Header) + 4);
        return result;
    }

    /// Verifies that a given payload is a valid packet. This checks for the
    /// preamble and the length matching what is needed for the packet
    /// size. Does not check the CRC.
    /// @param p wire formatted payload
    /// @return true if valid, else false.
    static bool is_valid(const Payload &p)
    {
        if (get_uint32(p, 0) != PREAMBLE)
        {
            return false;
        }
        uint16_t len = get_uint16(p, OFS_LEN);
        if ((p.size() != LEN_BASE + len) || len > MAX_LEN)
        {
            return false;
        }
        return true;
    }
}; // struct Defs

} // namespace traction_modem

#endif // _TRACTION_MODEM_DEFS_HXX_
