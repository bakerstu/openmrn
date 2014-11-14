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
 * \file RailCom.cxx
 *
 * Implementation of RailCom helper functions and tables.
 *
 * @author Balazs Racz
 * @date 12 Nov 2014
 */

#include "dcc/RailCom.hxx"

namespace dcc {

const uint8_t railcom_decode[256] =
{      INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   NACK,
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x33,
       INV,    INV,    INV,   0x34,    INV,   0x35,   0x36,    INV,
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x3A,
       INV,    INV,    INV,   0x3B,    INV,   0x3C,   0x37,    INV,
       INV,    INV,    INV,   0x3F,    INV,   0x3D,   0x38,    INV,
       INV,   0x3E,   0x39,    INV, RESVD3,    INV,    INV,    INV,
       INV,    INV,    INV,    INV,    INV,    INV,    INV,   0x24,
       INV,    INV,    INV,   0x23,    INV,   0x22,   0x21,    INV,
       INV,    INV,    INV,   0x1F,    INV,   0x1E,   0x20,    INV,
       INV,   0x1D,   0x1C,    INV,   0x1B,    INV,    INV,    INV,
       INV,    INV,    INV,   0x19,    INV,   0x18,   0x1A,    INV,
       INV,   0x17,   0x16,    INV,   0x15,    INV,    INV,    INV,
       INV,   0x25,   0x14,    INV,   0x13,    INV,    INV,    INV,
      0x32,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       INV,    INV,    INV,    INV,    INV,    INV,    INV, RESVD2,
       INV,    INV,    INV,   0x0E,    INV,   0x0D,   0x0C,    INV,
       INV,    INV,    INV,   0x0A,    INV,   0x09,   0x0B,    INV,
       INV,   0x08,   0x07,    INV,   0x06,    INV,    INV,    INV,
       INV,    INV,    INV,   0x04,    INV,   0x03,   0x05,    INV,
       INV,   0x02,   0x01,    INV,   0x00,    INV,    INV,    INV,
       INV,   0x0F,   0x10,    INV,   0x11,    INV,    INV,    INV,
      0x12,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       INV,    INV,    INV, RESVD1,    INV,   0x2B,   0x30,    INV,
       INV,   0x2A,   0x2F,    INV,   0x31,    INV,    INV,    INV,
       INV,   0x29,   0x2E,    INV,   0x2D,    INV,    INV,    INV,
      0x2C,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       INV,   BUSY,   0x28,    INV,   0x27,    INV,    INV,    INV,
      0x26,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       ACK,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
       INV,    INV,    INV,    INV,    INV,    INV,    INV,    INV,
};

}  // namespace dcc
