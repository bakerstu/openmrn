/** @copyright
 * Copyright (c) 2021, Stuart Baker
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
 * Implements the necessary logic to produce EventIdentifyGlobal messages.
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

/// Helper object for producing an Event Identify Global message. Once armed,
/// will produce an Event Identify Global message on the node's interface
/// at a pseudo random time between 1.500 and 2.011 seconds in the future. The
/// randomness comes a hash of the Node ID and aids in reducing the likeliness
/// of multiple nodes of the same design producing the same Event Identify
/// Global message unnecessarily. If another node produces the Event Identify
/// Global first (after arm but before timeout), then we abort the state
/// machine early.
class EventIdentifyGlobal : public StateFlowBase, private Atomic
{
public:
    /// Constructor.
    /// @param node Node ID associated with this object
    EventIdentifyGlobal(Node *node)
        : StateFlowBase(node->iface())
        , eventIdentifyGlobalHandler_(
              this, &EventIdentifyGlobal::handle_incoming_event_identify_global)
        , timer_(this)
        , node_(node)
        , aborted_(false)
        , deleteSelf_(false)
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

    /// Arm the EventIdentifyGlobal request. Multi-thread safe.
    /// @param delete_self true if the object should delete itself on next
    ///        state flow completion (one-shot).
    void arm(bool delete_self = false)
    {
        AtomicHolder h(this);
        if (is_terminated())
        {
            aborted_ = false;
            deleteSelf_ = delete_self;
            start_flow(STATE(entry));
        }
    }

private:
    /// Callback upon receiving a Defs::MTI_EVENTS_IDENTIFY_GLOBAL message.
    /// @param msg unused, need to unref to prevent memory leak
    void handle_incoming_event_identify_global(Buffer<GenMessage> *msg)
    {
        msg->unref();
        AtomicHolder h(this);
        aborted_ = true;
    }

    /// Entry/reset point into state machine.
    Action entry()
    {
        // get a pseudo random number between 1.500 and 2.011 seconds
        uint64_t h = node_->node_id() * 0x1c19a66d;
        uint32_t hh = h ^ (h >> 32);
        hh = hh ^ (hh >> 12) ^ (hh >> 24);
        long long timeout_msec = 1500 + (hh & 0x1FF);

        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(timeout_msec), STATE(timeout));
    }

    /// Will be called on the executor of the timer.
    Action timeout()
    {
        {
            // Making the state flow termination atomic cleans up a potential
            // race condition with an arm() call coming in after the if
            // statement but before the StateFlowBase::exit() call.
            AtomicHolder h(this);
            if (aborted_)
            {
                // no need to send Event Identify Global, already detected one
                return deleteSelf_ ? delete_this() : exit();
            }
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
            node_->iface()->global_message_write_flow());
        b->data()->reset(
            Defs::MTI_EVENTS_IDENTIFY_GLOBAL, node_->node_id(), EMPTY_PAYLOAD);
        node_->iface()->global_message_write_flow()->send(b);

        return deleteSelf_ ? delete_this() : exit();
    }

    /// handler for incoming messages
    MessageHandler::GenericHandler eventIdentifyGlobalHandler_;
    StateFlowTimer timer_; ///< timer object for handling the timeout
    Node *node_; ///< node to send message from
    uint8_t aborted_ : 1; ///< true if message received, abort need to send
    uint8_t deleteSelf_ : 1; ///< delete object upon state flow exit (one-shot)
};

} // namespace openlcb

#endif // _OPENLCB_EVENTIDENTIFYGLOBAL_HXX_
