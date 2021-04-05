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
        , eventIdentifyGlobalHandler_(
              this, &EventIdentifyGlobal::event_identify_global)
        , node_(node)
        , timer_(this)
        , aborted_(false)
    {
        node_->iface()->dispatcher()->register_handler(
            &eventIdentifyGlobalHandler_, Defs::MTI_EVENTS_IDENTIFY_GLOBAL,
            Defs::MTI_EXACT);
    }

    /// Destructor.
    ~EventIdentifyGlobal()
    {
        node_->iface()->dispatcher()->unregister_handler(
            &eventIdentifyGlobalHandler_, Defs::MTI_EVENTS_IDENTIFY_GLOBAL,
            Defs::MTI_EXACT);
    }

    /// Arm the EventIdentifyGlobal request
    void arm()
    {
        AtomicHolder h(this);
        if (is_terminated())
        {
            aborted_ = false;
            start_flow(STATE(entry));
        }
    }

private:
    void event_identify_global(Buffer<GenMessage> *msg)
    {
        AtomicHolder h(this);
        aborted_ = true;
    }

    /// Entry/reset point into state machine.
    Action entry()
    {
        // get a pseudo random number between 1.500 and 2.011 seconds
        long long timeout_msec = 1500 + (node_->node_id() & 0x1FF);

        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(timeout_msec), STATE(timeout));
    }

    /// Will be called on the executor of the timer.
    Action timeout()
    {
        if (aborted_)
        {
            // no need to send Event Identify Global, already detected one
            return exit();
        }

        if (!node_->is_initialized())
        {
            // node not initialized yet, try again later
            return call_immediately(STATE(entry));
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
        if (aborted_)
        {
            // no need to send Event Identify Global, already detected one
            b->unref();
        }
        else
        {
            b->data()->reset(
                Defs::MTI_EVENTS_IDENTIFY_GLOBAL, node_->node_id(),
                EMPTY_PAYLOAD);
            node_->iface()->addressed_message_write_flow()->send(b);
        }

        return exit();
    }

    /// handler for incoming messages
    MessageHandler::GenericHandler eventIdentifyGlobalHandler_;
    Node *node_; ///< node to send message from
    StateFlowTimer timer_; ///< timer object for handling the timeout
    uint8_t aborted_ : 1; ///< true if message received, abort need to send
};

} // namespace openlcb

#endif // _OPENLCB_EVENTIDENTIFYGLOBAL_HXX_
