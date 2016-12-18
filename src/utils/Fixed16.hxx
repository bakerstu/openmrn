/** \copyright
 * Copyright (c) 2016, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file Fixed16.hxx
 *
 * A Fixed-point numeric type with 16.16 specification.
 *
 * @author Balazs Racz
 * @date 19 Dec 2016
 */

#ifndef _UTILS_FIXED16_HXX_
#define _UTILS_FIXED16_HXX_

#include <stdint.h>

class Fixed16
{
public:
    Fixed16(uint16_t integer, uint16_t frac = 0)
    {
        uint32_t v = integer;
        v <<= 16;
        v |= frac;
        value_ = v;
    }

    Fixed16(const Fixed16& o) = default;
    Fixed16 &operator=(const Fixed16& o) = default;

    Fixed16 &operator+=(Fixed16 o)
    {
        value_ += o.value_;
        return *this;
    }

    template<class T> Fixed16 operator+(T o)
    {
        Fixed16 ret(*this);
        ret += o;
        return ret;
    }
    
    Fixed16 &operator-=(Fixed16 o)
    {
        value_ -= o.value_;
        return *this;
    }

    template<typename T> Fixed16 operator-(T o)
    {
        Fixed16 ret(*this);
        ret -= o;
        return ret;
    }
    
    Fixed16 &operator*=(Fixed16 o)
    {
        uint64_t v = value_;
        v *= o.value_;
        v >>= 16;
        value_ = v;
        return *this;
    }

    template<typename T> Fixed16 operator*(T o)
    {
        Fixed16 ret(*this);
        ret *= o;
        return ret;
    }

    Fixed16 &operator/=(Fixed16 o)
    {
        uint64_t v = value_;
        v <<= 16;
        v /= o.value_;
        value_ = v;
        return *this;
    }

    template<typename T> Fixed16 operator/(T o)
    {
        Fixed16 ret(*this);
        ret /= o;
        return ret;
    }

    /// @return the rounded value to the nearest integer
    operator uint16_t() {
        return (value_ + 0x8000) >> 16;
    }

    /// @return the integer part, rounded down
    uint16_t trunc() {
        return value_ >> 16;
    }

    /// @return the fractional part, as an uint16 value between 0 and 0xffff
    uint16_t frac() {
        return value_ & 0xffff;
    }
    
private:
    uint32_t value_;
};

#endif // _UTILS_FIXED16_HXX_
