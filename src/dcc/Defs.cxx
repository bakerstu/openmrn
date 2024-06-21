/** \copyright
 * Copyright (c) 2024, Balazs Racz
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
 * \file dcc/Defs.cxx
 *
 * Helper functions for DCC concepts.
 *
 * @author Balazs Racz
 * @date 7 Jan 2024
 */

#include "dcc/Defs.hxx"

namespace dcc
{
namespace Defs
{

bool decode_address_partition(uint16_t addr14, uint16_t *addr,
    uint8_t *partition, dcc::TrainAddressType *atype)
{
    TrainAddressType _atype = TrainAddressType::UNSUPPORTED;
    uint8_t _partition = 0xff;
    uint16_t _addr = 0xffffu;

    addr14 &= (1u << 14) - 1;
    uint8_t hibyte = addr14 >> 8;
    if (hibyte <= MAX_MOBILE_LONG)
    {
        _atype = TrainAddressType::DCC_LONG_ADDRESS;
        _partition = 0;
        _addr = addr14;
    }
    else if ((hibyte & MASK_ACC_EXT) == ADR_ACC_EXT)
    {
        _atype = TrainAddressType::DCC_ACCY_EXT;
        _partition = ADR_ACC_EXT;
        _addr = addr14 & ~(MASK_ACC_EXT << 8);
    }
    else if ((hibyte & MASK_ACC_BASIC) == ADR_ACC_BASIC)
    {
        _atype = TrainAddressType::DCC_ACCY_BASIC_OUTPUT;
        _partition = ADR_ACC_BASIC;
        _addr = addr14 & ~(MASK_ACC_BASIC << 8);
    }
    else if (hibyte == ADR_MOBILE_SHORT)
    {
        _atype = TrainAddressType::DCC_SHORT_ADDRESS;
        _partition = hibyte;
        _addr = addr14 & 0x7f;
    }
    else
    {
        return false;
    }

    if (addr)
    {
        *addr = _addr;
    }
    if (partition)
    {
        *partition = _partition;
    }
    if (atype)
    {
        *atype = _atype;
    }
    return true;
}

} // namespace dcc::Defs

} // namespace dcc
