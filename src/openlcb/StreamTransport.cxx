/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file StreamTransport.cxx
 *
 * Interface for stream functionality attached to an OpenLCB interface.
 *
 * @author Balazs Racz
 * @date 20 Dec 2022
 */

#include "openlcb/StreamTransport.hxx"

#include "openlcb/StreamSender.hxx"

namespace openlcb
{

StreamTransport::StreamTransport(If *iface)
    : inUseSendStreamIds_(0)
    , nextSendStreamId_(0)
{
    iface->set_stream_transport(this);
}

StreamTransport::~StreamTransport()
{
}

StreamTransportCan::StreamTransportCan(IfCan *iface, unsigned num_senders)
    : StreamTransport(iface)
{
    for (unsigned i = 0; i < num_senders; ++i)
    {
        senders_.typed_insert(new StreamSenderCan(iface, iface));
    }
}

StreamTransportCan::~StreamTransportCan()
{
}

} // namespace openlcb
