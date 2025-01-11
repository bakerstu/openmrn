/** \copyright
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
 * \file Uninitialized.hxx
 * Helper class for creating delayed initialized objects in static memory.
 *
 * @author Balazs Racz
 * @date 24 June 2017
 */

#ifndef _WOWW_MAIN_TRACTIONMODEMDEFS_HXX_
#define _WOWW_MAIN_TRACTIONMODEMDEFS_HXX_

#include <string>

#include "openlcb/Velocity.hxx"

namespace tractionmodem
{

struct Defs
{
    using Payload = std::string;

    /// Every command starts with these bytes.
    static constexpr uint32_t PREAMBLE = 0x41d2c37a;
    static constexpr char PREAMBLE_FIRST = 0x41;

    static constexpr uint16_t RESPONSE = 0x8000;

    enum Command : uint16_t
    {
        CMD_FN_SET = 0x0101,
        CMD_SPEED_SET = 0x0100,
        CMD_ESTOP_SET = 0x0102,
        CMD_WIRELESS_PRESENT = 0x0201,
        CMD_MEM_R = 0x1000,
        CMD_MEM_W = 0x1001,

        RESP_MEM_R = CMD_MEM_R | RESPONSE,
        RESP_MEM_W = CMD_MEM_W | RESPONSE,
    };

    static constexpr unsigned LEN_HEADER = 4+2+2;

    /// Length ofa zero-payload packet. 4 bytes preamble, 2 bytes cmd, 2 bytes
    /// length, 6 bytes CRC.
    static constexpr unsigned LEN_BASE = 14;
    /// Length of the data payload of a set function packet.
    static constexpr unsigned LEN_FN_SET = 6;
    /// Length of the data payload of a set speed packet.
    static constexpr unsigned LEN_SPEED_SET = 1;
    /// Length of the data payload of a set estop packet.
    static constexpr unsigned LEN_ESTOP_SET = 0;
    /// Length of the data payload of a wireless present packet.
    static constexpr unsigned LEN_WIRELESS_PRESENT = 1;
    /// Length of the data payload of a set estop packet.
    static constexpr unsigned LEN_MEM_R = 6;
    static constexpr unsigned LEN_MEM_W = 5; // add the numberof payload bytes

    /// Maximum allowed len value.
    static constexpr unsigned MAX_LEN = 512;

    /// Offset of the command in the packet.
    static constexpr unsigned OFS_CMD = 4;
    /// Offset of the length in the packet.
    static constexpr unsigned OFS_LEN = 6;
    /// Offset of the first data byte in the packet.
    static constexpr unsigned OFS_DATA = 8;


    /// Computes payload for the wirless present message.
    /// @apram is_present true if "present", else false
    static Payload get_wireless_present_payload(bool is_present)
    {
        Payload p;
        prepare(&p, CMD_WIRELESS_PRESENT, LEN_WIRELESS_PRESENT);
        append_uint8(&p, is_present ? 1 : 0);
        return p;
    }

    /// Computes payload to set a function.
    /// @param fn function number
    /// @param value function value. For binary functions, 0 is off, 1 is on.
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
    static Payload get_speed_set_payload(openlcb::Velocity v)
    {
        Payload p;
        prepare(&p, CMD_SPEED_SET, LEN_SPEED_SET);
        append_uint8(&p, v.get_dcc_128());

        append_crc(&p);
        return p;
    }

    /// Computes payload to set estop.
    static Payload get_estop_payload()
    {
        Payload p;
        prepare(&p, CMD_ESTOP_SET, LEN_ESTOP_SET);
        // no data in the payload
        append_crc(&p);
        return p;
    }

    /// Computes payload to read some data.
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
    static Payload get_memw_payload(
        uint8_t space, uint32_t address, const std::string& data)
    {
        return get_memw_payload(space, address, (const uint8_t*)data.data(), data.size());
    }

    /// Computes payload to write some data.
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
    static void append_uint32(Payload *p, uint32_t v)
    {
        p->push_back(v >> 24);
        p->push_back((v >> 16) & 0xff);
        p->push_back((v >> 8) & 0xff);
        p->push_back((v)&0xff);
    }
    /// Appends an uint16_t in network byte order to the payload.
    static void append_uint16(Payload *p, uint16_t v)
    {
        p->push_back((v >> 8) & 0xff);
        p->push_back((v)&0xff);
    }
    /// Appends an uint8_t to the payload.
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
    static void append_crc(Payload *p)
    {
        /// @todo
        append_uint16(p, 0);
        append_uint16(p, 0);
        append_uint16(p, 0);
    }

    static uint32_t get_uint32(const Payload &p, unsigned ofs)
    {
        uint32_t ret;
        if (ofs + 4 < p.size())
        {
            memcpy(&ret, p.data() + ofs, 4);
            return be32toh(ret);
        }
        return 0;
    }

    static uint16_t get_uint16(const Payload &p, unsigned ofs)
    {
        uint16_t ret;
        if (ofs + 2 < p.size())
        {
            memcpy(&ret, p.data() + ofs, 2);
            return be16toh(ret);
        }
        return 0;
    }

    /// Verifies that a given payload is a valid packet. This checks for the
    /// preamble and the length matching what is needed for the packet
    /// size. Does not check the CRC.
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

} // namespace tractionmodem

#endif // _WOWW_MAIN_TRACTIONMODEMDEFS_HXX_
