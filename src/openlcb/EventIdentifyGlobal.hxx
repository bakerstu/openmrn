/** @copyright
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
 * @file EventIdentifyGlobal.hxx
 *
 * A complete OpenLCB stack for use in straightforward OpenLCB nodes.
 *
 * @author Stuart Baker
 * @date 4 April 2021
 */

#ifndef _OPENLCB_EVENTIDENTIFYGLOBAL_HXX_
#define _OPENLCB_EVENTIDENTIFYGLOBAL_HXX_

#include "openlcb/If.hxx"
#include "utils/Atomic.hxx"

namespace openlcb
{

/// Helper object for producing an Event Identify Global message.
class EventIdentifyGlobal : public StateFlowBase, private Atomic
{
public:
    /// Constructor.
    /// @param node Node ID associated with this object
    EventIdentifyGlobal(Node *node)
        : StateFlowBase(node->iface())
        , node_(node)
        , incomingMessage_(this)
        , timer_(this)
        , armed_(false)
        , aborted_(false)
    {
        start_flow(STATE(entry));
    }

    /// Destructor.
    ~EventIdentifyGlobal()
    {
    }

    /// Arm the EventIdentifyGlobal request
    void arm()
    {
        AtomicHolder h(this);
        if (!armed_)
        {
            armed_ = true;
            aborted_ = false;
            notify();
        }
    }

private:
    /// Helper flow for handling the incoming event Identify Global Message.
    class IncomingMessage : public IncomingMessageStateFlow
    {
    public:
        /// Constructor.
        /// @param parent_
        IncomingMessage(EventIdentifyGlobal *parent)
            : IncomingMessageStateFlow(parent_->node_->iface())
        {
            parent_->node_->iface()->dispatcher()->register_handler(
                this, Defs::MTI_EVENTS_IDENTIFY_GLOBAL, Defs::MTI_EXACT);
        }

        /// Destructor.
        ~IncomingMessage()
        {
            parent_->node_->iface()->dispatcher()->unregister_handler(
                this, Defs::MTI_EVENTS_IDENTIFY_GLOBAL, Defs::MTI_EXACT);
        }

    private:
        /// Entry point to state machine for receiving an Event Identify Global
        /// message.
        /// @return release_and_exit()
        Action entry() override
        {
            {
                AtomicHolder h(parent_);
                parent_->aborted_ = true;
            }
            return release_and_exit();
        }

        /// reference to the parent object
        EventIdentifyGlobal *parent_;
    };

    /// Entry/reset point into state machine.
    Action entry()
    {
        AtomicHolder h(this);
        armed_ = false;
        return wait_and_call(STATE(activate_timer));
    }

    /// We have been armed, start the timer.
    Action activate_timer()
    {
        // get a pseudo random number between 1.500 and 2.011 seconds
        long long timeout_msec = 1500 + node_->node_id() & 0x1FF;

        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(timeout_msec), STATE(timeout));
    }

    /// Will be called on the executor of the timer.
    /// @returns the new timer period, or one of the above special values.
    Action timeout()
    {
        AtomicHolder h(this);
        if (aborted_)
        {
            // no need to send Event Identify Global, already detected one
            return call_immediately(STATE(entry));
        }

        if (!node_->is_initialized())
        {
            // node not initialized yet, try again later
            return call_immediately(STATE(activate_timer));
        }

        // allocate a buffer for the Event Identify Global message
        return allocate_and_call(
            node_->iface()->global_message_write_flow(), STATE(fill_buffer));
    }

    /// Fill in allocated buffer and send Event Identify Global message
    Action fill_buffer()
    {
        auto *b = get_allocation_result(
            node_->iface()->addressed_message_write_flow());
        b->data()->reset(
            Defs::MTI_EVENTS_IDENTIFY_GLOBAL, node_->node_id(), EMPTY_PAYLOAD);
        node_->iface()->addressed_message_write_flow()->send(b);

        return call_immediately(STATE(entry));
    }

    /// alow access to IncomingMessage helper
    friend class IncomingMessage;

    Node *node_; ///< node to send message from
    IncomingMessage incomingMessage_; ///< handler for incoming messages
    StateFlowTimer timer_; ///< timer object for handling the timeout
    uint8_t armed_ : 1; ///< true if armed to send a message
    uint8_t aborted_ : 1; ///< true if message received, abort need to send
};

} // namespace openlcb

#endif // _OPENLCB_EVENTIDENTIFYGLOBAL_HXX_
