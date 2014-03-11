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

/** Counts the number of alias conflicts that we see for aliases that we
 * already reserved. */
extern size_t g_alias_use_conflicts;

class AsyncAliasAllocator;

typedef FlowInterface<CanMessageData> IncomingFrameHandler;

/** Interface class for the asynchronous frame write flow. This flow allows you
    to write frames to the CAN bus.

    Usage:
    . allocate a flow instance through the write_allocator.
    . fill in flow->mutable_frame() [*]
    . call flow->Send()

    [*] When a write flow arrives from an allocator, it is guaranteed that it
    is a regular (non-err, non-remote) extended data frame.

    The flow will return itself to the allocator when done.
*/
class CanFrameWriteFlow : public ControlFlow
{
public:
    CanFrameWriteFlow(Executor *e, Notifiable *done) : ControlFlow(e, done)
    {
        ResetFrameEff();
    }

    /// @returns the frame buffer to be filled.
    struct can_frame *mutable_frame()
    {
        return &frame_;
    }

    /** Requests the frame buffer to be sent to the bus. Takes ownership of
     *  *this and returns it to the allocator.
     *
     *  @param done will be notified, when the frame was successfully
     *  enqueued. In most cases it can be set to NULL.
     */
    virtual void Send(Notifiable *done) = 0;

    /** Releases this frame buffer without sending of anything to the bus. Takes
     *  ownership of *this and returns it to the allocator.
     */
    virtual void Cancel() = 0;

    /** Resets the frame buffer to regular (non-err, non-remote) extended data
     * frame */
    void ResetFrameEff()
    {
        CLR_CAN_FRAME_ERR(frame_);
        CLR_CAN_FRAME_RTR(frame_);
        SET_CAN_FRAME_EFF(frame_);
    }

protected:
    struct can_frame frame_;
};

class AsyncIfCan : public AsyncIf
{
public:
    typedef TypedDispatchFlow<uint32_t, struct can_frame> FrameDispatchFlow;

    /**
     * Creates a CAN interface.
     *
     * @param executor will be used to process incoming (and outgoing) messages.
     *
     * @param device is a Pipe. The interface will add a member to this pipe to
     * handle incoming and outgoing traffic. The caller should add the
     * necessary hardware device, GridConnect bridge or mock interface to this
     * pipe (before this call or else outgoing packets might be lost).
     *
     * @param local_alias_cache_size tells the number of aliases to keep track
     * of for nocal virtual nodes and proxied nodes.
     *
     * @param remote_alias_cache_size tells the number of aliases to keep track
     * of for remote nodes on the bus.
     *
     * @param hw_write_flow_count tells how many concurrent write flows (each
     * with one CAN frame) should we have.
     *
     * @param global_can_write_flow_count tells how many concurrent write flows
     * for global unaddressed messages (each with one MTI/message) should we
     * have.
     */
    AsyncIfCan(Executor *executor, Pipe *device, int local_alias_cache_size,
               int remote_alias_cache_size, int hw_write_flow_count,
               int global_can_write_flow_count, int local_nodes_count);

    ~AsyncIfCan();

    /** Adds support to this interface for addressed NMRAnet messages (both
     * sending and receiving).
     *
     * @param num_write_flows sets how many concurrent addressed messages could
     * be in flight sending (ex. waiting for destination address resolution).*/
    void add_addressed_message_support(int num_write_flows);

    //! @returns the dispatcher of incoming CAN frames.
    FrameDispatchFlow *frame_dispatcher()
    {
        return &frame_dispatcher_;
    }

    //! @returns the allocator for the *can frame* write flow.
    TypedAllocator<CanFrameWriteFlow> *write_allocator()
    {
        HASSERT(write_allocator_.has_ever_seen_free_entries());
        return &write_allocator_;
    }

    //! Implementation class for receiving frames from CAN.
    class CanReadFlow;
    //! Implementation class for sending frames to CAN.
    class CanWriteFlow;

    //! @returns the asynchronous read/write object.
    CanReadFlow *pipe_member()
    {
        return pipe_member_.get();
    }

    //! @returns the alias cache for local nodes (vnodes and proxies)
    AliasCache *local_aliases()
    {
        return &localAliases_;
    }

    //! @returns the alias cache for remote nodes on this IF
    AliasCache *remote_aliases()
    {
        return &remoteAliases_;
    }

    //! @returns the alias cache for remote nodes on this IF
    AsyncAliasAllocator *alias_allocator()
    {
        return aliasAllocator_.get();
    }

    //! Sets the alias allocator for this If. Takes ownership of pointer.
    void set_alias_allocator(AsyncAliasAllocator *a);

    virtual void add_owned_flow(Executable *e);

private:
    //! Flow responsible for routing incoming messages to handlers.
    FrameDispatchFlow frame_dispatcher_;

    //! Handles asynchronous reading and writing from the device.
    std::unique_ptr<CanReadFlow> pipe_member_;

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

    //! Various implementation control flows that this interface owns.
    std::vector<std::unique_ptr<Executable>> owned_flows_;

    /** Allocator that holds (and mutex-controls) the frame write flow.
     *
     *  It is important that this allocator be destructed before the
     *  owned_flows_.
     */
    TypedAllocator<CanFrameWriteFlow> write_allocator_;

    //! Owns the alias allocator module.
    std::unique_ptr<AsyncAliasAllocator> aliasAllocator_;

    DISALLOW_COPY_AND_ASSIGN(AsyncIfCan);
};

} // namespace NMRAnet

#endif // _NMRAnetAsyncIfCan_hxx_
