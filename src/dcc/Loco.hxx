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
 * \file Loco.hxx
 *
 * Defines a simple DCC locomotive.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_LOCO_HXX_
#define _DCC_LOCO_HXX_

#include "dcc/Packet.hxx"
#include "dcc/PacketSource.hxx"

namespace dcc
{

class Dcc28Train : public PacketSource
{
public:
    Dcc28Train(DccShortAddress a)
    {
        memset(this, 0, sizeof(*this));
        isShortAddress_ = 1;
        dccAddress_ = a.value;
    }

    Dcc28Train(DccLongAddress a)
    {
        memset(this, 0, sizeof(*this));
        isShortAddress_ = 0;
        dccAddress_ = a.value;
    }

    void set_speed(SpeedType speed) OVERRIDE;
    SpeedType get_speed() OVERRIDE;
    SpeedType get_commanded_speed() OVERRIDE {
        return get_speed();
    }
    void set_emergencystop() OVERRIDE;
    void set_fn(uint32_t address, uint16_t value) OVERRIDE;
    uint16_t get_fn(uint32_t address) OVERRIDE;
    uint32_t legacy_address() OVERRIDE;

    // Generates next outgoing packet.
    void get_next_packet(unsigned code, Packet* packet) OVERRIDE;

private:
    // largest address allowed is 10239.
    unsigned dccAddress_ : 14;
    unsigned isShortAddress_ : 1;
    // 0: forward, 1: reverse
    unsigned direction_ : 1;
    unsigned lastSetSpeed_ : 16;
    // functions f0-f28.
    unsigned fn_ : 29;
    // Which refresh packet should go out next.
    unsigned nextRefresh_ : 3;
    unsigned speed_ : 5;
    unsigned directionChanged_ : 1;
};

} // namespace dcc

#endif // _DCC_LOCO_HXX_
