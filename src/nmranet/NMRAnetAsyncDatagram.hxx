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
 * \file NMRAnetAsyncDatagram.hxx
 *
 * Interface for datagram functionality in Async NMRAnet implementation.
 *
 * @author Balazs Racz
 * @date 25 Jan 2013
 */

#ifndef _NMRAnetAsyncDatagram_hxx_
#define _NMRAnetAsyncDatagram_hxx_

#include "utils/NodeHandlerMap.hxx"
#include "executor/allocator.hxx"
#include "nmranet/AsyncIf.hxx"

namespace NMRAnet
{

struct IncomingDatagram;

/// Allocator for getting IncomingDatagram objects.
extern InitializedAllocator<IncomingDatagram> g_incoming_datagram_allocator;

struct IncomingDatagram : public QueueMember
{
    NodeHandle src;
    AsyncNode* dst;
    // Owned by the current IncomingDatagram object. Includes the datagram ID
    // as the first byte.
    Buffer* payload;

    void free()
    {
        if (payload)
        {
            payload->free();
            payload = nullptr;
        }
        g_incoming_datagram_allocator.Release(this);
    }
};

/** Base class for datagram handlers.
 *
 * The datagram handler needs to listen to the incoming queue for arriving
 * datagrams. */
class DatagramHandler
{
public:
    /** Sends a datagram to this handler. Takes ownership of datagram. */
    void datagram_arrived(IncomingDatagram* datagram)
    {
        queue_.ReleaseBack(datagram);
    }

protected:
    TypedAllocator<IncomingDatagram> queue_;
};

/** Transport-agnostic dispatcher of datagrams.
 *
 * There will be typically one instance of this for each interface with virtual
 * nodes. This class is responsible for maintaining the registered datagram
 * handlers, and taking the datagram MTI from the incoming messages and routing
 * them to the datagram handlers. */
class DatagramDispatcher : public IncomingMessageHandler,
                           private AllocationResult
{
    /// @TODO(balazs.racz) we have two QueueMember base classes in here.
public:
    typedef TypedNodeHandlerMap<AsyncNode, DatagramHandler> Registry;

    /** Creates a datagram dispatcher.
     *
     * @param interface is the async interface to which to bind.
     * @param num_registry_entries is the size of the registry map.
     */
    DatagramDispatcher(AsyncIf* interface, size_t num_registry_entries)
        : m_(nullptr), done_(nullptr), registry_(num_registry_entries)
    {
        lock_.TypedRelease(this);
    }

    /// @returns the registry of datagram handlers.
    Registry* registry()
    {
        return &registry_;
    }

    virtual AllocatorBase* get_allocator()
    {
        return &lock_;
    }

    /// Callback from the dispatcher.
    virtual void handle_message(IncomingMessage* m, Notifiable* done);
    /// Callback from the allocator.
    virtual void AllocationCallback(QueueMember* entry);

    virtual void Run()
    {
        HASSERT(0);
    }

protected:
    TypedAllocator<IncomingMessageHandler> lock_;

    /// Message called from the dispatcher.
    IncomingMessage* m_;
    Notifiable* done_;

    /// Maintains the registered datagram handlers.
    Registry registry_;
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncDatagram_hxx_
