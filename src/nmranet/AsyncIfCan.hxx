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
#include "nmranet_can.h"
#include "utils/BufferQueue.hxx"
#include "utils/PipeFlow.hxx"

namespace NMRAnet
{

/** Thin wrapper around struct can_frame that will allow a dispatcher select
 * the frames by CAN ID and mask as desired by the handlers. */
struct CanMessageData : public can_frame
{
    /** Constructor. Resets the inlined frame to an empty extended frame. */
    CanMessageData()
    {
        CLR_CAN_FRAME_ERR(*this);
        CLR_CAN_FRAME_RTR(*this);
        SET_CAN_FRAME_EFF(*this);
        can_dlc = 0;
    }

    typedef uint32_t id_type;

    /** This bit will be set in standard CAN frames when they get to the
        dispatcher. */ static const id_type
    STANDARD_FRAME_BIT = (1U << 30);

    /** Filter to OR onto a can ID to tell the dispatcher to only consider
     * extended can frames. */
    static const uint32_t CAN_EXT_FRAME_FILTER = 0;
    /** Mask to OR onto a can mask to tell the dispatcher to only consider
     * extended can frames. */
    static const uint32_t CAN_EXT_FRAME_MASK = ~0x1FFFFFFFU;

    /** @returns the identifier for dispatching */
    id_type id()
    {
        if (IS_CAN_FRAME_EFF(*this))
        {
            return GET_CAN_FRAME_ID_EFF(*this);
        }
        else
        {
            return GET_CAN_FRAME_ID(*this) | STANDARD_FRAME_BIT;
        }
    }

    /** @Returns a mutable pointer to the embedded CAN frame. */
    struct can_frame *mutable_frame()
    {
        return this;
    }

    /** @Returns the embedded CAN frame. */
    const struct can_frame &frame() const
    {
        return *this;
    }
};

/** @todo(balazs.racz) make these two somehow compatible with each other. It's
 * not easy, because they use different ID functions and their size differs
 * a bit as well. */
typedef FlowInterface<Buffer<CanMessageData>> IncomingFrameHandler;
typedef FlowInterface<Buffer<CanHubData>> OutgoingFrameHandler;

/** Counts the number of alias conflicts that we see for aliases that we
 * already reserved. */
extern size_t g_alias_use_conflicts;

class AsyncAliasAllocator;
class AsyncIfCan;

/** Interface class for the asynchronous frame write flow. This flow allows you
    to write frames to the CAN bus.

    Usage:
    . allocate a buffer for this flow.
    . fill in buffer->data()->mutable_frame() [*]
    . call flow->send(buffer)
*/
class CanFrameWriteFlow : public OutgoingFrameHandler
{
public:
    CanFrameWriteFlow(AsyncIfCan *service) : ifCan_(service)
    {
    }

    virtual DynamicPool *pool();
    virtual void send(Buffer<CanHubData> *message,
                      unsigned priority = UINT_MAX);

private:
    AsyncIfCan *ifCan_;
};

class CanFrameReadFlow : public OutgoingFrameHandler
{
public:
    CanFrameReadFlow(AsyncIfCan *service) : ifCan_(service)
    {
    }

    virtual DynamicPool *pool();
    virtual void send(Buffer<CanHubData> *message,
                      unsigned priority = UINT_MAX);

private:
    AsyncIfCan *ifCan_;
};

class AsyncIfCan : public AsyncIf
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
    AsyncIfCan(Executor *executor, CanHubFlow *device,
               int local_alias_cache_size, int remote_alias_cache_size,
               int local_nodes_count);

    ~AsyncIfCan();

    typedef DispatchFlow<Buffer<CanMessageData>, 4> FrameDispatchFlow;

    /** Adds support to this interface for addressed NMRAnet messages (both
     * sending and receiving). */
    void add_addressed_message_support();

    /// @returns the dispatcher of incoming CAN frames.
    FrameDispatchFlow *frame_dispatcher()
    {
        return &frameDispatcher_;
    }

    /// @returns the flow for writing CAN frames to the bus.
    OutgoingFrameHandler *frame_write_flow()
    {
        return &frameWriteFlow_;
    }

    /// Implementation class for receiving frames from CAN.
    class CanReadFlow;
    /// Implementation class for sending frames to CAN.
    /// class CanWriteFlow;

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
    /// @returns the asynchronous read/write object.
    CanReadFlow *hub_port()
    {
        return hubPort_.get();
    }
    /// @returns the device (hub) that we need to send the packets to.
    CanHubFlow *device()
    {
        return device_;
    }

    /// The device we need to send packets to.
    CanHubFlow* device_;

    /** Flow responsible for translating from generic packets to CAN hub
     * packets. */
    CanFrameWriteFlow frameWriteFlow_;

    /// Flow responsible for routing incoming messages to handlers.
    FrameDispatchFlow frameDispatcher_;

    /// Handles asynchronous reading and writing from the device.
    std::unique_ptr<CanReadFlow> hubPort_;

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

    /** Allocator that holds (and mutex-controls) the frame write flow.
     *
     *  It is important that this allocator be destructed before the
     *  owned_flows_.
     */
    TypedAllocator<CanFrameWriteFlow> write_allocator_;

    /// Owns the alias allocator module.
    std::unique_ptr<AsyncAliasAllocator> aliasAllocator_;

    DISALLOW_COPY_AND_ASSIGN(AsyncIfCan);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfCan_hxx_
