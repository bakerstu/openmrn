/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file Blinker.cxx
 *
 * Implementations of blinker routines
 *
 * @author Balazs Racz
 * @date 27 Feb 2021
 */

#include "utils/blinker.h"

extern "C"
{

unsigned parseblink(uint32_t pattern)
{
    unsigned ret = 0;
    if (!pattern)
    {
        return ret;
    }
    // Finds the top bit.
    uint32_t tb = 1U << 31;
    while ((tb & pattern) == 0)
    {
        tb >>= 1;
    }
    unsigned nibble_shift = 1;
    unsigned last_len = 1;
    unsigned curr_len = 1;
    pattern &= ~tb;
    while (true)
    {
        if (curr_len)
        {
            if (pattern & 1)
            {
                curr_len++;
            }
            else
            {
                // end of lit period
                if (last_len != curr_len)
                {
                    // new blink length
                    nibble_shift <<= 4;
                    last_len = curr_len;
                }
                ret += nibble_shift;
                curr_len = 0;
            }
        }
        else
        {
            // we are in unlit
            if (pattern & 1)
            {
                // start of a new lit period.
                curr_len = 1;
            }
        }
        if (!pattern)
        {
            break;
        }
        pattern >>= 1;
    }
    return ret;
}

}
