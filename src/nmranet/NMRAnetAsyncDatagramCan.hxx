/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file NMRAnetAsyncDatagramCan.hxx
 *
 * CANbus datagram parser and renderer flows.
 *
 * @author Balazs Racz
 * @date 25 Jan 2014
 */

#ifndef _NMRAnetAsyncDatagramCan_hxx_
#define _NMRAnetAsyncDatagramCan_hxx_

#include "nmranet/NMRAnetIfCan.hxx"
#include "nmranet/AsyncIfCan.hxx"
#include "nmranet/NMRAnetAsyncDatagram.hxx"

namespace nmranet
{

class CanDatagramSupport : public DatagramSupport {
public:
    /*
     * @param num_registry_entries is the size of the registry map (how
     * many datagram handlers can be registered)*/
    CanDatagramSupport(IfCan* interface, int num_registry_entries,
                       int num_clients);

    ~CanDatagramSupport();

    IfCan* if_can() {
        return static_cast<IfCan*>(interface());
    }
};

/// Creates a CAn datagram parser flow. Exposed for testing only.
Executable* TEST_CreateCanDatagramParser(IfCan* if_can);

} // namespace nmranet

#endif // _NMRAnetAsyncDatagramCan_hxx_
