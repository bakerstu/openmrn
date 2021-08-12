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
 * \file RailCom.cxx
 *
 * Implementation of RailCom helper functions and tables.
 *
 * @author Balazs Racz
 * @date 12 Nov 2014
 */

#include <string.h>

#include "dcc/RailCom.hxx"
#include "utils/Crc.hxx"

namespace dcc {
static constexpr uint8_t INV = RailcomDefs::INV;
static constexpr uint8_t ACK = RailcomDefs::ACK;
static constexpr uint8_t NACK = RailcomDefs::NACK;
static constexpr uint8_t BUSY = RailcomDefs::BUSY;
static constexpr uint8_t RESVD1 = RailcomDefs::RESVD1;
static constexpr uint8_t RESVD2 = RailcomDefs::RESVD2;
const uint8_t railcom_decode[256] =
    // 0|8     1|9     2|a     3|b     4|c     5|d     6|e     7|f  
{      INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // 0
       INV,    INV,    INV,    INV,    INV,    INV,    INV,    ACK, // 0
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x33, // 1
       INV,    INV,    INV,   0x34,    INV,   0x35,   0x36,    INV, // 1
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x3A, // 2
       INV,    INV,    INV,   0x3B,    INV,   0x3C,   0x37,    INV, // 2
       INV,    INV,    INV,   0x3F,    INV,   0x3D,   0x38,    INV, // 3
       INV,   0x3E,   0x39,    INV,   NACK,    INV,    INV,    INV, // 3
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x24, // 4
       INV,    INV,    INV,   0x23,    INV,   0x22,   0x21,    INV, // 4
       INV,    INV,    INV,   0x1F,    INV,   0x1E,   0x20,    INV, // 5
       INV,   0x1D,   0x1C,    INV,   0x1B,    INV,    INV,    INV, // 5
       INV,    INV,    INV,   0x19,    INV,   0x18,   0x1A,    INV, // 6
       INV,   0x17,   0x16,    INV,   0x15,    INV,    INV,    INV, // 6
       INV,   0x25,   0x14,    INV,   0x13,    INV,    INV,    INV, // 7
      0x32,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // 7
       INV,    INV,    INV,    INV,    INV,    INV,    INV, RESVD2, // 8
       INV,    INV,    INV,   0x0E,    INV,   0x0D,   0x0C,    INV, // 8
       INV,    INV,    INV,   0x0A,    INV,   0x09,   0x0B,    INV, // 9
       INV,   0x08,   0x07,    INV,   0x06,    INV,    INV,    INV, // 9
       INV,    INV,    INV,   0x04,    INV,   0x03,   0x05,    INV, // a
       INV,   0x02,   0x01,    INV,   0x00,    INV,    INV,    INV, // a
       INV,   0x0F,   0x10,    INV,   0x11,    INV,    INV,    INV, // b
      0x12,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // b
       INV,    INV,    INV, RESVD1,    INV,   0x2B,   0x30,    INV, // c
       INV,   0x2A,   0x2F,    INV,   0x31,    INV,    INV,    INV, // c
       INV,   0x29,   0x2E,    INV,   0x2D,    INV,    INV,    INV, // d
      0x2C,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // d
       INV,   BUSY,   0x28,    INV,   0x27,    INV,    INV,    INV, // e
      0x26,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // e
       ACK,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // f
       INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV, // f       
};

const uint8_t railcom_encode[64] = {
    0b10101100,
    0b10101010,
    0b10101001,
    0b10100101,
    0b10100011,
    0b10100110,
    0b10011100,
    0b10011010,
    0b10011001,
    0b10010101,
    0b10010011,
    0b10010110,
    0b10001110,
    0b10001101,
    0b10001011,
    0b10110001,
    0b10110010,
    0b10110100,
    0b10111000,
    0b01110100,
    0b01110010,
    0b01101100,
    0b01101010,
    0b01101001,
    0b01100101,
    0b01100011,
    0b01100110,
    0b01011100,
    0b01011010,
    0b01011001,
    0b01010101,
    0b01010011,
    0b01010110,
    0b01001110,
    0b01001101,
    0b01001011,
    0b01000111,
    0b01110001,
    0b11101000,
    0b11100100,
    0b11100010,
    0b11010001,
    0b11001001,
    0b11000101,
    0b11011000,
    0b11010100,
    0b11010010,
    0b11001010,
    0b11000110,
    0b11001100,
    0b01111000,
    0b00010111,
    0b00011011,
    0b00011101,
    0b00011110,
    0b00101110,
    0b00110110,
    0b00111010,
    0b00100111,
    0b00101011,
    0b00101101,
    0b00110101,
    0b00111001,
    0b00110011,
};

/// Helper function to parse a part of a railcom packet.
///
/// @param fb_channel Which hardware channel did the railcom message arrive
/// at. typically from 0.. num channels - 1: for a command station always 0,
/// for a multi-channel railcom decoder it's as many as the number of ports.
/// @param railcom_channel 1 or 2 depending on which part of the cutout window
/// the data is from.
/// @param ptr raw railcom data read from the UART.
/// @param size how many bytes were read from the UART
/// @param output where to put the decoded packets (or GARBAGE packets if
/// decoding fails).
///
void parse_internal(uint8_t fb_channel, uint8_t railcom_channel,
    const uint8_t *ptr, unsigned size,
    std::vector<struct RailcomPacket> *output)
{
    if (!size)
        return;
    for (unsigned ofs = 0; ofs < size; ++ofs)
    {
        uint8_t decoded = railcom_decode[ptr[ofs]];
        uint8_t type = 0xff;
        uint32_t arg = 0;
        if (decoded == RailcomDefs::ACK)
        {
            type = RailcomPacket::ACK;
        }
        else if (decoded == RailcomDefs::NACK)
        {
            type = RailcomPacket::NACK;
        }
        else if (decoded == RailcomDefs::BUSY)
        {
            type = RailcomPacket::BUSY;
        }
        else if (decoded >= 64)
        {
            output->emplace_back(
                fb_channel, railcom_channel, RailcomPacket::GARBAGE, 0);
            break;
        }
        if (type != 0xff)
        {
            output->emplace_back(fb_channel, railcom_channel, type, 0);
            continue;
        }
        // Now: we have a packet.
        uint8_t packet_id = decoded >> 2;
        uint8_t len = 2;
        arg = decoded & 3;
        switch (packet_id)
        {
            case RMOB_ADRHIGH:
                type = RailcomPacket::MOB_ADRHIGH;
                break;
            case RMOB_ADRLOW:
                type = RailcomPacket::MOB_ADRLOW;
                break;
            case RMOB_EXT:
                type = RailcomPacket::MOB_EXT;
                // TODO: according to the standardthis should be a len==3
                // packet, but the ESU LokPilot 3 is sending it as 2-byte
                // packet.
                break;
            case RMOB_POM:
                type = RailcomPacket::MOB_POM;
                if (size == 6 && ofs == 0
                    // The ESU LokPilot V4 decoder fills the CV read (a 2-byte
                    // packet) with four NACK bytes, presumably to report that
                    // it is not actually giving back a 32-bit response but
                    // only an 8-bit response.
                    && railcom_decode[ptr[2]] < 64)
                {
                    len = 6;
                }
                else
                {
                    len = 2;
                }
                break;
            case RMOB_XPOM0:
            case RMOB_XPOM1:
            case RMOB_XPOM2:
            case RMOB_XPOM3:
                type = RailcomPacket::MOB_XPOM0 + (packet_id - RMOB_XPOM0);
                len = 6;
                break;
            case RMOB_DYN:
                type = RailcomPacket::MOB_DYN;
                len = 3;
                break;
            default:
                // Don't know the size of the fragment. Throws out response.
                len = 255;
                break;
        }
        if (ofs + len > size)
        {
            // The expected message size does not fit.
            output->emplace_back(
                fb_channel, railcom_channel, RailcomPacket::GARBAGE, 0);
            break;
        }
        for (int i = 1; i < len; ++i, ++ofs)
        {
            arg <<= 6;
            uint8_t decoded = railcom_decode[ptr[ofs + 1]];
            if (decoded >= 64)
            {
                type = RailcomPacket::GARBAGE;
            }
            arg |= decoded;
        }
        output->emplace_back(fb_channel, railcom_channel, type, arg);
    }
}

void parse_railcom_data(
    const dcc::Feedback &fb, std::vector<struct RailcomPacket> *output)
{
    output->clear();
    if (fb.channel == 0xff)
        return; // Occupancy feedback information
    if (fb.ch1Size == 1 && (railcom_decode[fb.ch1Data[0]] != RailcomDefs::INV) && fb.ch2Size >= 1)
    {
        // Railcom channel 1 should have 0 or 2 bytes according to the standard.
        //
        // There is probably a mistake in the placement of the second window
        // (i.e., a timing problem in the decoder). Let's concatenate the two
        // channels and parse them together.
        uint8_t data[8];
        memcpy(data, fb.ch1Data, fb.ch1Size);
        memcpy(data + fb.ch1Size, fb.ch2Data, fb.ch2Size);
        parse_internal(fb.channel, 2, data, fb.ch1Size + fb.ch2Size, output);
        return;
    }
    for (bool ch1 : {true, false})
    {
        uint8_t size;
        const uint8_t *ptr;
        if (ch1)
        {
            size = fb.ch1Size;
            ptr = fb.ch1Data;
        }
        else
        {
            size = fb.ch2Size;
            ptr = fb.ch2Data;
        }
        parse_internal(fb.channel, ch1 ? 1 : 2, ptr, size, output);
    }
}

// static
void RailcomDefs::add_did_feedback(uint64_t decoder_id, Feedback *fb)
{
    fb->ch1Size = 2;
    fb->ch2Size = 6;
    append12(
        RMOB_LOGON_ENABLE_FEEDBACK, (decoder_id >> 36) & 0xff, fb->ch1Data);
    append36((decoder_id >> 32) & 0xf, (decoder_id & 0xffffffffu), fb->ch2Data);
}

// static
void RailcomDefs::add_shortinfo_feedback(uint16_t requested_address,
    uint8_t max_fn, uint8_t psupp, uint8_t ssupp, Feedback *fb)
{
    Crc8DallasMaxim m;
    requested_address &= 0x3FFF;
    requested_address |= 0x8000;
    m.update16(requested_address >> 8);
    m.update16(requested_address & 0xff);
    m.update16(max_fn);
    m.update16(psupp);
    m.update16(ssupp);
    fb->ch1Size = 2;
    fb->ch2Size = 6;
    append12(
        requested_address >> 12, (requested_address >> 4) & 0xff, fb->ch1Data);
    uint32_t lp = (uint32_t(max_fn) << 24) | (uint32_t(psupp) << 16) |
        (uint32_t(ssupp) << 8) | m.get();
    append36(requested_address & 0xf, lp, fb->ch2Data);
}

// static
void RailcomDefs::add_assign_feedback(uint8_t changeflags, uint16_t changecount,
    uint8_t supp2, uint8_t supp3, Feedback *fb)
{
    changecount &= 0xFFF;
    fb->ch1Size = 2;
    fb->ch2Size = 6;

    Crc8DallasMaxim m;
    uint8_t h = (RMOB_LOGON_ASSIGN_FEEDBACK << 4) | (changeflags >> 4);
    m.update16(h);
    h = ((changeflags & 0xf) << 4) | (changecount >> 8);
    m.update16(h);
    append12(RMOB_LOGON_ASSIGN_FEEDBACK, changeflags, fb->ch1Data);

    h = changecount & 0xff;
    m.update16(h);
    m.update16(supp2);
    m.update16(supp3);

    uint32_t lp =
        ((changecount & 0xff) << 24) | (supp2 << 16) | (supp3 << 8) | m.get();
    append36(changecount >> 8, lp, fb->ch2Data);
}

}  // namespace dcc
