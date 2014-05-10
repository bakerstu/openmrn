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
enum {
    MARKLIN_DEFAULT_CMD = 0b00100110,
    DCC_DEFAULT_CMD = 0,
    DCC_LONG_PREAMBLE_CMD = 0b00001100,
    DCC_SERVICE_MODE_5X_WITH_ACK_CMD = 0b00111000,
    // standard dcc packet with 5x repeat.
    DCC_SERVICE_MODE_5X_CMD = 0b00101000,
    DCC_SERVICE_MODE_1X_CMD = 0b00001000,

    // Baseline packet: speed and direction.
    DCC_BASELINE_SPEED = 0b01000000,
    DCC_BASELINE_SPEED_FORWARD = 0b00100000,
    DCC_BASELINE_SPEED_LIGHT = 0b00010000,
};

void Packet::AddDccChecksum()
{
    HASSERT(dlc < MAX_PAYLOAD);
    uint8_t cs = 0;
    for (int i = 0; i < dlc; ++i)
    {
        cs ^= payload[i];
    }
    payload[dlc++] = cs;
}

void Packet::SetDccIdle() {
    dlc = 3;
    payload[0] = payload[2] = 0xFF;
    payload[1] = 0;
}

void Packet::SetDccResetAllDecoders() {
    dlc = 3;
    payload[0] = payload[1] = payload[2] = 0;
}

void Packet::SetDccSpeed14(DccShortAddress address, bool is_fwd, bool light, unsigned speed) {
    HASSERT(0);
}

void Packet::SetDccSpeed28(DccShortAddress address, bool is_fwd, unsigned speed) {
    header_raw_data = DCC_DEFAULT_CMD;
    dlc = 2;
    payload[0] = address.value & 0x7F;
    uint8_t b1 = DCC_BASELINE_SPEED;
    if (is_fwd) b1 |= DCC_BASELINE_SPEED_FORWARD;
    if (speed == EMERGENCY_STOP) {
        b1 |= 1;
    } else if (speed == 0) {
    } else {
        HASSERT(speed <= 28);
        speed += 3; // avoids 00, 01 (stop) and 10 11 (e-stop)
        if (speed & 1) b1 |= DCC_BASELINE_SPEED_LIGHT;
        b1 |= (speed >> 1) & 0xf;
    }
    payload[1] = b1;
    AddDccChecksum();
}

} // namespace dcc
