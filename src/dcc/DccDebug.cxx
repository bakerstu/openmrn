/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * \file DccDebug.cxx
 *
 * Defines helper functions for debugging DCC packets
 *
 * @author Balazs Racz
 * @date 28 Dec 2017
 */

#include "dcc/DccDebug.hxx"
#include "utils/StringPrintf.hxx"

namespace dcc
{

string packet_to_string(const DCCPacket &pkt)
{
    if (pkt.packet_header.is_pkt)
    {
        return StringPrintf("[cmd:%u]", pkt.command_header.cmd);
    }
    // First render the option bits
    string options;
    if (pkt.packet_header.is_marklin)
    {
        options += "[marklin]";
        return options;
    }
    else
    {
        options += "[dcc]";
    }
    if (pkt.packet_header.send_long_preamble)
    {
        options += "[long_preamble]";
    }
    if (pkt.packet_header.sense_ack)
    {
        options += "[sense_ack]";
    }
    if (pkt.packet_header.rept_count)
    {
        options +=
            StringPrintf("[repeat %u times]", pkt.packet_header.rept_count + 1);
    }
    if (!pkt.dlc)
    {
        return options + " no payload";
    }
    unsigned ofs = 0;
    bool is_idle_packet = false;
    if (pkt.payload[ofs] == 0xff)
    {
        options += " Idle packet";
        ofs++;
        if (pkt.payload[ofs] != 0)
        {
            options += StringPrintf(" unexpected[0x%02x]", pkt.payload[ofs]);
        }
        is_idle_packet = true;
    }
    else if (pkt.payload[ofs] == 0)
    {
        options += " Broadcast";
        ofs++;
    }
    else if ((pkt.payload[ofs] & 0x80) == 0)
    {
        options += StringPrintf(" Short Address %u", pkt.payload[ofs]);
        ofs++;
    }
    else if ((pkt.payload[ofs] & 0xC0) == 0X80)
    {
        // accessory decoder
    }
    else if (pkt.payload[ofs] >= 192 && pkt.payload[ofs] <= 231)
    {
        // long address
        unsigned addr = pkt.payload[ofs] & 0x3F;
        addr <<= 8;
        ofs++;
        addr |= pkt.payload[ofs];
        ofs++;
        options += StringPrintf(" Long Address %u", addr);
    }
    uint8_t cmd = pkt.payload[ofs];
    ofs++;
    if ((cmd & 0xC0) == 0x40)
    {
        // Speed and direction
        bool is_forward = (cmd & 0x20) != 0;
        options += " SPD ";
        options += is_forward ? 'F' : 'R';
        uint8_t speed = ((cmd & 0xF) << 1) | ((cmd & 0x10) >> 4);
        switch (speed)
        {
            case 0:
                options += " 0";
                break;
            case 1:
                options += " 0'";
                break;
            case 2:
                options += " E-STOP";
                break;
            case 3:
                options += " E-STOP'";
                break;
            default:
                options += StringPrintf(" %u", speed - 3);
        }
    }
    else if ((cmd >> 5) == 0b100)
    {
        // function group 0
        options += StringPrintf(" F[0-4]=%d%d%d%d%d", (cmd >> 4) & 1,
            (cmd >> 0) & 1, (cmd >> 1) & 1, (cmd >> 2) & 1, (cmd >> 3) & 1);
    }
    else if ((cmd >> 5) == 0b101)
    {
        // function group 1 or 2
        if (cmd & 0x10)
        {
            options += " F[5-8]=";
        }
        else
        {
            options += " F[9-12]=";
        }
        options += StringPrintf("%d%d%d%d", (cmd >> 0) & 1, (cmd >> 1) & 1,
            (cmd >> 2) & 1, (cmd >> 3) & 1);
    }
    else if ((cmd >> 5) == 0b110)
    {
        // expansion
        uint8_t c = cmd & 0x1F;
        if ((c & ~1) == 0b11110)
        {
            if (c & 1)
            {
                options += " F[21-28]=";
            }
            else
            {
                options += " F[13-20]=";
            }
            c = pkt.payload[ofs];
            ofs++;
            for (int i = 0; i < 8; ++i, c >>= 1)
                options += '0' + (c & 1);
        }
        else
        {
            /// @todo
        }
    }
    else if (cmd == 0 && is_idle_packet)
    {
    }
    // checksum of packet
    if (ofs == pkt.dlc && pkt.packet_header.skip_ec == 0)
    {
        // EC skipped.
    }
    else if (((ofs + 1) == pkt.dlc) && pkt.packet_header.skip_ec == 1)
    {
        uint8_t x = 0;
        for (unsigned i = 0; i + 1 < pkt.dlc; ++i)
        {
            x ^= pkt.payload[i];
        }
        if (x != pkt.payload[pkt.dlc - 1])
        {
            options += StringPrintf(" [bad EC expected 0x%02x actual 0x%02x]",
                x, pkt.payload[pkt.dlc - 1]);
        }
    }
    else
    {
        options += StringPrintf(" [bad dlc, exp %u, actual %u]", ofs, pkt.dlc);
    }
    return options;
}

} // namespace dcc
