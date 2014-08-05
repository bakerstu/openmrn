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
 * \file TractionDefs.cxx
 *
 * Implementations for static functions related to traction protocol.
 *
 * @author Balazs Racz
 * @date 5 May 2014
 */

#include "nmranet/TractionDefs.hxx"

#include <string.h>
#include "utils/macros.h"

namespace nmranet {

SpeedType fp16_to_speed(const void *fp16) {
    Velocity speed;
    DASSERT(sizeof(speed) == 4);
    float16_t input;
    const uint8_t* in_p = static_cast<const uint8_t*>(fp16);
    // We assemble the input assuming it is big-endian.
    input = *in_p++;
    input <<= 8;
    input |= *in_p;
    speed.set_wire(input);
    return speed;
}

void speed_to_fp16(SpeedType speed, void *fp16) {
    DASSERT(sizeof(speed) == 4);
    float16_t output = speed.get_wire();
    uint8_t* o = static_cast<uint8_t*>(fp16);
    *o++ = output >> 8;
    *o = output & 0xff;
}

const uint64_t TractionDefs::IS_TRAIN_EVENT = 0x0101000000000303ULL;
const uint64_t TractionDefs::EMERGENCY_STOP_EVENT = 0x010100000000FFFFULL;
const uint64_t TractionDefs::CLEAR_EMERGENCY_STOP_EVENT = 0x010100000000FFFEULL;

const uint64_t TractionDefs::NODE_ID_DC_BLOCK = 0x060000000000ULL;
const uint64_t TractionDefs::NODE_ID_DCC = 0x060100000000ULL;
const uint64_t TractionDefs::NODE_ID_TMCC = 0x060200000000ULL;
const uint64_t TractionDefs::NODE_ID_MARKLIN_MOTOROLA = 0x060300000000ULL;
const uint64_t TractionDefs::NODE_ID_MTH_DCS = 0x060400000000ULL;

}  // namespace nmranet
