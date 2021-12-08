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
 * \file TcpHubPort.hxx
 * Establishes the connection for a single remote Tcp client to a Hub.
 *
 * @author Balazs Racz
 * @date 8 Dec 2021
 */

#include "openlcb/TcpHubPort.hxx"

namespace openlcb
{

TcpHubPort::TcpHubPort(TcpHubFlow *parent, NodeID gateway_node_id,
    SequenceNumberGenerator *sequence, int fd, Notifiable *on_error)
    : parent_(parent)
    , onError_(on_error)
    , stringHub_(parent_->service())
    , device_(&stringHub_, fd, this)
    , sendFlow_(parent_->service(), gateway_node_id, &stringHub_, &recvFlow_,
          sequence)
    , recvFlow_(parent_, this)
{
}

TcpHubPort::~TcpHubPort()
{
}

void TcpHubPort::send(Buffer<TcpHubData> *b, unsigned prio)
{
    sendFlow_.send((Buffer<openlcb::GenMessage> *)b, prio);
}

} // namespace openlcb
