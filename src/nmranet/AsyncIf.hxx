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
 * \file AsyncIf.hxx
 *
 * Asynchronous NMRAnet interface.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _NMRAnetAsyncIf_hxx_
#define _NMRAnetAsyncIf_hxx_

#include "nmranet/ReadDispatch.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "utils/BufferQueue.hxx"

namespace NMRAnet
{

struct IncomingMessage {
  //! OpenLCB MTI of the incoming message.
  If::MTI mti;
  //! Source node.
  NodeHandle src;
  //! Destination node.
  NodeHandle dst;
  //! If the destination node is local, this value is non-NULL.
  Node* dst_node;
  //! Data content in the message body. Owned by the dispatcher.
  Buffer* payload;
};

typedef ParamHandler<IncomingMessage> IncomingMessageHandler;

class AsyncIf
{
 public:
  typedef TypedDispatchFlow<uint32_t, IncomingMessage> MessageDispatchFlow;

  AsyncIf(Executor* executor);

  //! @returns the dispatcher of incoming messages.
  MessageDispatchFlow* dispatcher() { return &dispatcher_; }

 private:
  //! Flow responsible for routing incoming messages to handlers.
  MessageDispatchFlow dispatcher_;

  DISALLOW_COPY_AND_ASSIGN(AsyncIf);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIf_hxx_
