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
 * \file Address.hxx
 *
 * Defines structures for holding and identifying DCC addresses.
 *
 * @author Balazs Racz
 * @date 10 May 2014
 */

#ifndef _DCC_ADDRESS_HXX_
#define _DCC_ADDRESS_HXX_

#include <stdint.h>

#include "utils/macros.h"

namespace dcc
{

/// Strongly typed wrapper representing a short DCC address. This allows C++
/// type inference to decide whether a particular value is a long or short
/// address.
struct DccShortAddress
{
    /// Largest valid address.
    static constexpr unsigned ADDRESS_MAX = 127;
    /// Address value.
    uint8_t value;
    /// Constructor. @param v is the address value (0<=v<128);
    explicit DccShortAddress(unsigned v)
        : value(v)
    {
        HASSERT(value <= ADDRESS_MAX);
    }
};

/// Strongly typed wrapper representing a long DCC address. This allows C++
/// type inference to decide whether a particular value is a long or short
/// address.
struct DccLongAddress
{
    /// Largest valid address.
    static constexpr unsigned ADDRESS_MAX = 10239;
    /// Address value.
    uint16_t value;
    /// Constructor. @param v is the address value (0<=v<10239);
    explicit DccLongAddress(unsigned v)
        : value(v)
    {
        HASSERT(value <= ADDRESS_MAX);
    }
};

/// Strongly typed wrapper representing a marklin-motorola protocol
/// address. This address is between 0 and 80.
struct MMAddress
{
    /// Largest valid address.
    static constexpr unsigned ADDRESS_MAX = 80;
    /// Address value.
    uint8_t value;
    /// Constructor. @param v is the address value (0<=v<81);
    explicit MMAddress(unsigned v)
        : value(v)
    {
        HASSERT(v <= ADDRESS_MAX);
    }
};

} // namespace dcc

#endif // _DCC_ADDRESS_HXX_
