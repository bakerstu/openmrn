/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file Charlieplex.hxx
 * Helper classes for driving multiple LEDs using charlieplexing output lines.
 *
 * @author Balazs Racz
 * @date 8 Nov 2015
 */

#ifndef _UTILS_CHARLIEPLEXING_HXX_
#define _UTILS_CHARLIEPLEXING_HXX_

#include "os/Gpio.hxx"

template <unsigned N> struct CharlieplexHelper;

template <> struct CharlieplexHelper<3>
{
    static const uint8_t pinlist[];
    static unsigned num_bits()
    {
        return 6;
    }

    static unsigned pin_high(unsigned bit)
    {
        return pinlist[bit << 1];
    }
    static unsigned pin_low(unsigned bit)
    {
        return pinlist[(bit << 1) | 1];
    }
};

const uint8_t CharlieplexHelper<3>::pinlist[] = {
    0, 2, //
    0, 1, //
    1, 2, //
    1, 0, //
    2, 1, //
    2, 0, //
};

template <unsigned N, class helper = CharlieplexHelper<N>> class Charlieplex
{
public:
    Charlieplex(const Gpio * const pins[N] )
        : pins_(pins)
        , nextBit_(0)
        , bits_(0)
    {
        for (unsigned i = 0; i < N; ++i)
        {
            pins_[i]->set_direction(Gpio::Direction::INPUT);
        }
    }

    void tick()
    {
        pins_[helper::pin_high(nextBit_)]->set_direction(
            Gpio::Direction::INPUT);
        pins_[helper::pin_low(nextBit_)]->set_direction(Gpio::Direction::INPUT);
        nextBit_++;
        if (nextBit_ >= helper::num_bits()) {
            nextBit_ = 0;
        }
        if (bits_ & (1 << nextBit_))
        {
            pins_[helper::pin_high(nextBit_)]->set_direction(
                Gpio::Direction::OUTPUT);
            pins_[helper::pin_high(nextBit_)]->set();
            pins_[helper::pin_low(nextBit_)]->set_direction(
                Gpio::Direction::OUTPUT);
            pins_[helper::pin_low(nextBit_)]->clr();
        }
    }

    unsigned *payload()
    {
        return &bits_;
    }

private:
    const Gpio *const *pins_; //< array of all GPIO pins to use
    unsigned nextBit_;        //< LED that comes next
    unsigned bits_;
};

#endif // _UTILS_CHARLIEPLEXING_HXX_
