/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file ReadDispatch.hxx
 *
 * Class for dispatching incoming messages to handlers.
 *
 * @author Balazs Racz
 * @date 2 Dec 2013
 */

#ifndef _NMRAnetReadDispatch_hxx_
#define _NMRAnetReadDispatch_hxx_

#include <vector>

#include "executor/Dispatcher.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetNode.hxx"

namespace NMRAnet
{

struct MtiParams
{
    Defs::MTI mti;          //< MTI of the incoming message.
    NodeHandle dst;       //< destination node, or {0,0} for global.
    Node* dst_node;       //< destination node pointer, or NULL for global.
    const Buffer* buffer; //< message payload.
};

typedef ParamHandler<MtiParams> MtiHandler;

} // namespace NMRAnet

#endif // _NMRAnetReadDispatch_hxx_
