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
 * \file Packet.hxx
 *
 * Defines a DCC Packet structure.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#include "dcc/Packet.hxx"

#include "utils/macros.h"

namespace dcc
{
enum
{
    MARKLIN_DEFAULT_CMD = 0b00100110,
    DCC_DEFAULT_CMD = 0,
    DCC_LONG_PREAMBLE_CMD = 0b00001100,
    DCC_SERVICE_MODE_5X_WITH_ACK_CMD = 0b00111000,
    // standard dcc packet with 5x repeat.
    DCC_SERVICE_MODE_5X_CMD = 0b00101000,
    DCC_SERVICE_MODE_1X_CMD = 0b00001000,

    DCC_LONG_ADDRESS_FIRST = 0b11000000,

    // Baseline packet: speed and direction.
    DCC_BASELINE_SPEED = 0b01000000,
    DCC_BASELINE_SPEED_FORWARD = 0b00100000,
    DCC_BASELINE_SPEED_LIGHT = 0b00010000,
    DCC_FUNCTION1 = 0b10000000,
    DCC_FUNCTION1_F0 = 0b00010000,
    DCC_FUNCTION2_F5 = 0b10110000,
    DCC_FUNCTION2_F9 = 0b10100000,
    DCC_FEATURE_EXP_F13 = 0b11011110,
    DCC_FEATURE_EXP_F21 = 0b11011111,
};

void Packet::add_dcc_checksum()
{
    HASSERT(dlc < MAX_PAYLOAD);
    // Protects against double call of add checksum.
    HASSERT(!packet_header.skip_ec);
    uint8_t cs = 0;
    for (int i = 0; i < dlc; ++i)
    {
        cs ^= payload[i];
    }
    payload[dlc++] = cs;
    packet_header.skip_ec = 1;
}

void Packet::set_dcc_idle()
{
    start_dcc_packet();
    dlc = 3;
    payload[0] = payload[2] = 0xFF;
    payload[1] = 0;
    packet_header.skip_ec = 1;
}

void Packet::set_dcc_reset_all_decoders()
{
    start_dcc_packet();
    dlc = 3;
    payload[0] = payload[1] = payload[2] = 0;
    packet_header.skip_ec = 1;
}

void Packet::add_dcc_address(DccShortAddress address)
{
    start_dcc_packet();
    payload[dlc++] = address.value & 0x7F;
}

void Packet::add_dcc_address(DccLongAddress address)
{
    start_dcc_packet();
    payload[dlc++] = DCC_LONG_ADDRESS_FIRST | (address.value >> 8);
    payload[dlc++] = address.value & 0xff;
}

void Packet::add_dcc_speed14(bool is_fwd, bool light, unsigned speed)
{
    /// TODO(balazs.racz) add unittests.
    uint8_t b1 = DCC_BASELINE_SPEED;
    if (is_fwd)
        b1 |= DCC_BASELINE_SPEED_FORWARD;
    if (light)
        b1 |= DCC_BASELINE_SPEED_LIGHT;
    if (speed == EMERGENCY_STOP)
    {
        b1 |= 1;
    }
    else if (speed == 0)
    {
    }
    else
    {
        HASSERT(speed <= 14);
        speed += 1; // avoids 01 (e-stop)
        b1 |= speed & 0xf;
    }
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_speed28(bool is_fwd, unsigned speed)
{
    uint8_t b1 = DCC_BASELINE_SPEED;
    if (is_fwd)
        b1 |= DCC_BASELINE_SPEED_FORWARD;
    if (speed == EMERGENCY_STOP)
    {
        b1 |= 1;
    }
    else if (speed == 0)
    {
    }
    else
    {
        HASSERT(speed <= 28);
        speed += 3; // avoids 00, 01 (stop) and 10 11 (e-stop)
        if (speed & 1)
            b1 |= DCC_BASELINE_SPEED_LIGHT;
        b1 |= (speed >> 1) & 0xf;
    }
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function0_4(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION1;
    if (values & 1)
        b1 |= DCC_FUNCTION1_F0;
    b1 |= (values >> 1) & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function5_8(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION2_F5;
    b1 |= values & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function9_12(unsigned values)
{
    uint8_t b1 = DCC_FUNCTION2_F9;
    b1 |= values & 0xf;
    payload[dlc++] = b1;
    add_dcc_checksum();
}

void Packet::add_dcc_function13_20(unsigned values)
{
    payload[dlc++] = DCC_FEATURE_EXP_F13;
    payload[dlc++] = values & 0xff;
    add_dcc_checksum();
}

void Packet::add_dcc_function21_28(unsigned values)
{
    payload[dlc++] = DCC_FEATURE_EXP_F21;
    payload[dlc++] = values & 0xff;
    add_dcc_checksum();
}

} // namespace dcc
