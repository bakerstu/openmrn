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

DatagramSupport::DatagramSupport(AsyncIf* interface,
                                 size_t num_registry_entries)
    : interface_(interface), dispatcher_(interface_, num_registry_entries)
{
    interface_->dispatcher()->register_handler(If::MTI_DATAGRAM, 0xffff,
                                              &dispatcher_);
}

DatagramSupport::~DatagramSupport()
{
    interface_->dispatcher()->unregister_handler(If::MTI_DATAGRAM, 0xffff,
                                                &dispatcher_);
}

void DatagramSupport::DatagramDispatcher::handle_message(IncomingMessage* m,
                                                         Notifiable* done)
{
    HASSERT(!m_);
    HASSERT(IsNotStarted());
    if (!m->dst_node)
    {
        // Destination is not a local virtal node.
        lock_.TypedRelease(this);
        done->notify();
        return;
    }
    m_ = m;
    Restart(done);
    Allocate(&g_incoming_datagram_allocator, ST(incoming_datagram_allocated));
}

ControlFlow::Action
DatagramSupport::DatagramDispatcher::incoming_datagram_allocated()
{
    IncomingDatagram* d =
        GetTypedAllocationResult(&g_incoming_datagram_allocator);
    d->src = m_->src;
    d->dst = m_->dst_node;
    d->payload = m_->payload;
    // Takes over ownership of payload.
    /// @TODO(balazs.racz) Implement buffer refcounting.
    m_->payload = nullptr;

    // The incoming message is no longer needed; call the done callback. Since
    // we don't release ourselves to the lock yet, there will be no other
    // incoming message to handle coming yet.
    m_ = nullptr;
    Exit();

    // Saves the datagram pointer -- note that this is unioned over m_ which was
    // cleared above.
    d_ = d;

    unsigned datagram_id = -1;
    if (!d->payload || !d->payload->used())
    {
        LOG(WARNING, "Invalid arguments: incoming datagram from node %llx "
                     "alias %x has no payload.",
            d->src.id, d->src.alias);
        resultCode_ = DatagramClient::PERMANENT_ERROR;
        return Allocate(interface_->addressed_write_allocator(),
                        ST(respond_rejection));
    }
    datagram_id = *static_cast<uint8_t*>(d->payload->start());

    // Looks up the datagram handler.
    DatagramHandler* h = registry_.lookup(d->dst, datagram_id);

    if (!h)
    {
        LOG(VERBOSE, "No datagram handler found for node %p id %x", d->dst,
            datagram_id);
        resultCode_ = DatagramClient::PERMANENT_ERROR;
        return Allocate(interface_->addressed_write_allocator(),
                        ST(respond_rejection));
    }

    h->datagram_arrived(d_);
    d_ = nullptr;
    return ReleaseAndExit(&lock_, this);
}

ControlFlow::Action
DatagramSupport::DatagramDispatcher::respond_rejection()
{
    auto* f = GetTypedAllocationResult(interface_->addressed_write_allocator());

    Buffer* payload = buffer_alloc(2);
    uint8_t* w = static_cast<uint8_t*>(payload->start());
    w[0] = (resultCode_ >> 8) & 0xff;
    w[1] = resultCode_ & 0xff;
    payload->advance(2);
    f->WriteAddressedMessage(If::MTI_DATAGRAM_REJECTED, d_->dst->node_id(),
                             d_->src, payload, nullptr);
    d_->free();
    d_ = nullptr;
    return ReleaseAndExit(&lock_, this);
}

} // namespace NMRAnet
