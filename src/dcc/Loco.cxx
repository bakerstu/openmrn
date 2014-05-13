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
 * \file Loco.cxx
 *
 * Defines a simple DCC locomotive.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#include "dcc/Loco.hxx"

#include "utils/logging.h"
#include "dcc/UpdateLoop.hxx"

namespace dcc
{

Dcc28Train::Dcc28Train(DccShortAddress a)
{
    // If this fails, then we might be zeroing out the wrong place.
    HASSERT(sizeof(*this) == 4 + sizeof(data_));
    memset(data_, 0, sizeof(data_));

    isShortAddress_ = 1;
    dccAddress_ = a.value;
    packet_processor_add_refresh_source(this);
}

Dcc28Train::Dcc28Train(DccLongAddress a)
{
    // If this fails, then we might be zeroing out the wrong place.
    HASSERT(sizeof(*this) == 4 + sizeof(data_));
    memset(data_, 0, sizeof(data_));

    isShortAddress_ = 0;
    dccAddress_ = a.value;
    packet_processor_add_refresh_source(this);
}

Dcc28Train::~Dcc28Train()
{
    packet_processor_remove_refresh_source(this);
}

enum DccTrainUpdateCode
{
    REFRESH = 0,
    SPEED = 1,
    FUNCTION0 = 2,
    FUNCTION5 = 3,
    FUNCTION9 = 4,
    FUNCTION13 = 5,
    FUNCTION21 = 6,
    MIN_REFRESH = SPEED,
    /** @TODO(balazs.racz) choose adaptive max-refresh based on how many
     * functions are actually in use for the loco. */
    MAX_REFRESH = FUNCTION9,
    ESTOP = 16,
};

void Dcc28Train::set_speed(SpeedType speed)
{
    float16_t new_speed = speed.get_wire();
    if (lastSetSpeed_ == new_speed)
    {
        LOG(VERBOSE, "not updating speed: old speed %04x, new speed %04x",
            lastSetSpeed_, new_speed);
        return;
    }
    lastSetSpeed_ = new_speed;
    if (speed.direction() != direction_)
    {
        directionChanged_ = 1;
        direction_ = speed.direction();
    }
    float f_speed = speed.mph();
    if (f_speed > 0)
    {
        f_speed *= (28.0 / 128);
        unsigned sp = f_speed;
        sp++; // makes sure it is at least speed step 1.
        if (sp > 28)
            sp = 28;
        LOG(VERBOSE, "set speed to step %u", sp);
        speed_ = sp;
    }
    else
    {
        speed_ = 0;
    }
    packet_processor_notify_update(this, SPEED);
};

SpeedType Dcc28Train::get_speed()
{
    SpeedType v;
    v.set_wire(lastSetSpeed_);
    return v;
}

void Dcc28Train::set_emergencystop()
{
    speed_ = 0;
    SpeedType dir0;
    dir0.set_direction(direction_);
    lastSetSpeed_ = dir0.get_wire();
    directionChanged_ = 1;
    packet_processor_notify_update(this, ESTOP);
}

void Dcc28Train::set_fn(uint32_t address, uint16_t value)
{
    if (address <= 28)
    {
        unsigned bit = 1 << address;
        if (value)
        {
            fn_ |= bit;
        }
        else
        {
            fn_ &= ~bit;
        }
        if (address < 5)
        {
            packet_processor_notify_update(this, FUNCTION0);
        }
        else if (address < 9)
        {
            packet_processor_notify_update(this, FUNCTION5);
        }
        else if (address < 13)
        {
            packet_processor_notify_update(this, FUNCTION9);
        }
        else if (address < 21)
        {
            packet_processor_notify_update(this, FUNCTION13);
        }
        else
        {
            packet_processor_notify_update(this, FUNCTION21);
        }
    }
}

uint16_t Dcc28Train::get_fn(uint32_t address)
{
    if (address <= 28)
    {
        if (fn_ & (1 << address))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

uint32_t Dcc28Train::legacy_address()
{
    return dccAddress_;
}

// Generates next outgoing packet.
void Dcc28Train::get_next_packet(unsigned code, Packet *packet)
{
    packet->start_dcc_packet();
    if (isShortAddress_)
    {
        packet->add_dcc_address(DccShortAddress(dccAddress_));
    }
    else
    {
        packet->add_dcc_address(DccLongAddress(dccAddress_));
    }
    if (code == REFRESH)
    {
        code = MIN_REFRESH + nextRefresh_++;
        if (nextRefresh_ > MAX_REFRESH - MIN_REFRESH)
        {
            nextRefresh_ = 0;
        }
    }
    else
    {
        // User action. Up repeat count.
        packet->packet_header.rept_count = 1;
    }
    switch (code)
    {
        case FUNCTION0:
        {
            packet->add_dcc_function0_4(fn_ & 0x1F);
            return;
        }
        case FUNCTION5:
        {
            packet->add_dcc_function5_8(fn_ >> 5);
            return;
        }
        case FUNCTION9:
        {
            packet->add_dcc_function9_12(fn_ >> 9);
            return;
        }
        case FUNCTION13:
        {
            packet->add_dcc_function13_20(fn_ >> 13);
            return;
        }
        case FUNCTION21:
        {
            packet->add_dcc_function21_28(fn_ >> 21);
            return;
        }
        case ESTOP:
        {
            packet->add_dcc_speed28(!direction_, Packet::EMERGENCY_STOP);
            packet->packet_header.rept_count = 3;
            return;
        }
        default:
            LOG(WARNING, "Unknown packet generation code: %x", code);
        // fall through
        case SPEED:
        {
            if (directionChanged_)
            {
                packet->packet_header.rept_count = 2;
                directionChanged_ = 0;
            }
            packet->add_dcc_speed28(!direction_, speed_);
            return;
        }
    }
}

} // namespace dcc
