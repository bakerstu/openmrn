/** \copyright
 * Copyright (c) 2019, Balazs Racz
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
 * \file CC32xxSha.hxx
 *
 * Helper function to perform SHA calculation with hardware hash engine on the
 * CC32xx.
 *
 * @author Balazs Racz
 * @date 3 Nov 2019
 */

#ifndef _CC32xxSHA_HXX_
#define _CC32xxSHA_HXX_

#define USE_CC3220_ROM_DRV_API

#include "driverlib/prcm.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/shamd5.h"
#include "inc/hw_dthe.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include <string>

#include "utils/SyncStream.hxx"

class SHAHelper
{
public:
    /// Computes a SHA-256 hash of the given input data.
    /// @param data is the pointer to the input.
    /// @param len the size in bytes of the input. Must be a multiple of 64
    /// bytes.
    /// @return a 256-bit (32 byte) string containing the SHA256 hash.
    static std::string sha256(const void *data, size_t len)
    {
        MAP_SHAMD5ConfigSet(SHAMD5_BASE, SHAMD5_ALGO_SHA256, 1, 1, 0, 0);
        std::string ret(32, 0);
        // Note: this should really be SHAMD5DataProcess, but that has an
        // assert on len%64 == 0 for no reason. The below function does the
        // same thing but without the extra assert.
        ROM_SHAMD5DataProcess(
            SHAMD5_BASE, (uint8_t *)data, len, (uint8_t *)&ret[0]);
        return ret;
    }

private:
    /// This class cannot be instantiated.
    SHAHelper();
};

#endif // _CC32xxSHA_HXX_
