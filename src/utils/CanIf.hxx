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
 * \file CanIf.hxx
 *
 * Defines a service for interfacing with a CANbus. The interface receives
 * incoming frames from the bus device and proxies it to various handlers that
 * the user may register for specific ranges of CAN identifiers.
 *
 * @author Balazs Racz
 * @date 11 May 2014
 */

#ifndef _UTILS_CANIF_HXX_
#define _UTILS_CANIF_HXX_

#include "nmranet_can.h"
#include "utils/PipeFlow.hxx"

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
     *  dispatcher. */
    static const id_type STANDARD_FRAME_BIT = (1U << 30);

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

    /* This will be aliased onto CanHubData::skipMember_. It is needed to keep
     * the two structures the same size for casting between them. */
    void *unused;
};

/** @todo(balazs.racz) make these two somehow compatible with each other. It's
 * not easy, because they use different ID functions and their size differs
 * a bit as well. */
typedef FlowInterface<Buffer<CanMessageData>> IncomingFrameHandler;
typedef FlowInterface<Buffer<CanHubData>> OutgoingFrameHandler;

class CanIf;

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
    CanFrameWriteFlow(CanIf *service)
        : ifCan_(service)
    {
    }

    virtual DynamicPool *pool();
    virtual void send(Buffer<CanHubData> *message,
                      unsigned priority = UINT_MAX);

private:
    CanIf *ifCan_;
};

/** This flow is responsible for taking data from the can HUB and sending it to
 * the IfCan's dispatcher. */
class CanFrameReadFlow : public OutgoingFrameHandler
{
public:
    CanFrameReadFlow(CanIf *service)
        : ifCan_(service)
    {
    }

    virtual DynamicPool *pool();
    virtual void send(Buffer<CanHubData> *message,
                      unsigned priority = UINT_MAX);

private:
    CanIf *ifCan_;
};

class CanIf
{
public:
    CanIf(Service *service, CanHubFlow *device);
    ~CanIf();

    typedef DispatchFlow<Buffer<CanMessageData>, 4> FrameDispatchFlow;

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

private:
    friend class CanFrameWriteFlow;
    // friend class CanFrameReadFlow;

    /** The device we need to send packets to. */
    CanHubFlow *device_;

    /** Flow responsible for writing packets to the CAN hub. */
    CanFrameWriteFlow frameWriteFlow_;

    /** Flow responsible for translating from CAN hub packets to dispatcher
     * packets. */
    CanFrameReadFlow frameReadFlow_;

    /// Flow responsible for routing incoming messages to handlers.
    FrameDispatchFlow frameDispatcher_;

    /// @returns the asynchronous read/write object.
    CanHubPortInterface *hub_port()
    {
        return &frameReadFlow_;
    }
    /// @returns the device (hub) that we need to send the packets to.
    CanHubFlow *device()
    {
        return device_;
    }
};

#endif // _UTILS_CANIF_HXX_
