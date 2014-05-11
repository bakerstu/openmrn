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
 * \file AsyncIfCan.hxx
 *
 * Asynchronous NMRAnet interface implementation for CANbus.
 *
 * @author Balazs Racz
 * @date 3 Dec 2013
 */

#ifndef _NMRAnetAsyncIfCan_hxx_
#define _NMRAnetAsyncIfCan_hxx_

#include <memory>

#include "nmranet/AsyncIf.hxx"
#include "nmranet/NMRAnetIf.hxx"
#include "nmranet/NMRAnetAliasCache.hxx"
#include "utils/BufferQueue.hxx"
#include "utils/CanIf.hxx"

namespace NMRAnet
{

class AsyncIfCan;

/** Counts the number of alias conflicts that we see for aliases that we
 * already reserved. */
extern size_t g_alias_use_conflicts;

class AsyncAliasAllocator;
class AsyncIfCan;


class AsyncIfCan : public AsyncIf, public CanIf
{
public:
    /**
     * Creates a CAN interface.
     *
     * @param executor will be used to process incoming (and outgoing) messages.
     *
     * @param device is a CanHub. The interface will add a member to this pipe
     * to handle incoming and outgoing traffic. The caller should add the
     * necessary hardware device, GridConnect bridge or mock interface to this
     * pipe (before this constructor or else outgoing packets might be lost).
     *
     * @param local_alias_cache_size tells the number of aliases to keep track
     * of for nocal virtual nodes and proxied nodes.
     *
     * @param remote_alias_cache_size tells the number of aliases to keep track
     * of for remote nodes on the bus.
     *
     * @param local_nodes_count is the maximum number of virtual nodes that
     * this interface will support. */
    AsyncIfCan(ExecutorBase *executor, CanHubFlow *device,
               int local_alias_cache_size, int remote_alias_cache_size,
               int local_nodes_count);

    ~AsyncIfCan();

    /** Adds support to this interface for addressed NMRAnet messages (both
     * sending and receiving). */
    void add_addressed_message_support();

    /// @returns the alias cache for local nodes (vnodes and proxies)
    AliasCache *local_aliases()
    {
        return &localAliases_;
    }

    /// @returns the alias cache for remote nodes on this IF
    AliasCache *remote_aliases()
    {
        return &remoteAliases_;
    }

    /// @returns the alias cache for remote nodes on this IF
    AsyncAliasAllocator *alias_allocator()
    {
        return aliasAllocator_.get();
    }

    /// Sets the alias allocator for this If. Takes ownership of pointer.
    void set_alias_allocator(AsyncAliasAllocator *a);

    virtual void add_owned_flow(Executable *e);

private:
    friend class CanFrameWriteFlow; // accesses the device and the hubport.

    /** Aliases we know are owned by local (virtual or proxied) nodes.
     *
     *  This member must only be accessed from the If's executor.
     */
    AliasCache localAliases_;
    /** Aliases we know are owned by remote nodes on this If.
     *
     *  This member must only be accessed from the If's executor.
     */
    AliasCache remoteAliases_;

    /// Various implementation control flows that this interface owns.
    std::vector<std::unique_ptr<Executable>> ownedFlows_;

    /// Owns the alias allocator module.
    std::unique_ptr<AsyncAliasAllocator> aliasAllocator_;

    DISALLOW_COPY_AND_ASSIGN(AsyncIfCan);
};

/** Base class for incoming CAN frame handlers. */
class CanFrameStateFlow : public StateFlow<Buffer<CanMessageData>, QList<1>>
{
public:
    CanFrameStateFlow(AsyncIfCan *service)
        : StateFlow<Buffer<CanMessageData>, QList<1>>(service)
    {
    }

    AsyncIfCan *if_can()
    {
        return static_cast<AsyncIfCan *>(service());
    }
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfCan_hxx_
