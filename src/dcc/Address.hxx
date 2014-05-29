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

#ifndef _DCC_AD
#define _DCC_AD

#include <stdint.h>

#include "utils/macros.h"

namespace dcc {

struct DccShortAddress {
    uint8_t value;
    explicit DccShortAddress(uint8_t v)
        : value(v) {
        HASSERT(value < 128);
    }
};

struct DccLongAddress {
    uint16_t value;
    explicit DccLongAddress(uint16_t v)
        : value(v) {
        HASSERT(value <= 10239);
    }
};

struct MMAddress {
    uint8_t value;
    explicit MMAddress(uint8_t v)
        : value(v) {
        HASSERT(v <= 80);
    }
};

}  // namespace dcc

#endif // _DCC_AD
