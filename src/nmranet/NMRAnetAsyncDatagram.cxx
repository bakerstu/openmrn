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
 * \file NMRAnetAsyncDatagram.cxx
 *
 * Implementation for the node-agnostic datagram functionality.
 *
 * @author Balazs Racz
 * @date 25 Jan 2013
 */

#include "nmranet/NMRAnetAsyncDatagram.hxx"

namespace NMRAnet
{

void DatagramDispatcher::handle_message(IncomingMessage* m, Notifiable* done)
{
    HASSERT(!m_);
    m_ = m;
    done_ = done;
    g_incoming_datagram_allocator.AllocateEntry(this);
}

void DatagramDispatcher::AllocationCallback(QueueMember* entry)
{
    IncomingDatagram* d = g_incoming_datagram_allocator.cast_result(entry);
    d->src = m_->src;
    d->dst = m_->dst_node;
    d->payload = m_->payload;
    // Takes over ownership of payload.
    /// @TODO(balazs.racz) Implement buffer refcounting.
    m_->payload = nullptr;

    unsigned datagram_id = -1;
    if (!d->payload || !d->payload->used())
    {
        /// @TODO(balazs.racz): reject datagram with invalid arguments.
        d->free();
        return;
    }
    datagram_id = *static_cast<uint8_t*>(d->payload->start());

    // Looks up the datagram handler.
    DatagramHandler* h = registry_.lookup(d->dst, datagram_id);

    if (!h)
    {
        /// @TODO(balazs.racz): reject datagram with permanent error no
        /// retries.
        d->free();
        return;
    }

    h->datagram_arrived(d);
    lock_.TypedRelease(this);
}

} // namespace NMRAnet
