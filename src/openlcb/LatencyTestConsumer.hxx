/** \copyright
 * Copyright (c) 2024, Balazs Racz
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
 * \file LatencyTestConsumer.hxx
 *
 * Utility class for determining the response latency of a node.
 *
 * @author Balazs Racz
 * @date 2 Feb 2024
 */

#ifndef _OPENLCB_LATENCYTESTCONSUMER_HXX_
#define _OPENLCB_LATENCYTESTCONSUMER_HXX_

#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// This event consumer works together with the hub_test application in order
/// to detect the response latency of a node. The theory of operation is that
/// hub_test sends out an identify consumer message with a given event ID, and
/// this consumer is going to respond. The hub_test then measures the response
/// latency. There is a big event range being advertised (32 bits), and
/// hub_test will send events with different IDs to be able to match request to
/// response.
///
/// Here in the consumer the only thing we need to do is respond to identify
/// consumer messages with consumer identified.
///
/// An additional feature is to be able to measure an arbitrary processing step
/// inside the node. For this purpose we have a hook function. When the
/// identify producer message comes in, we call the hook. When the measured
/// process completes, it should notify the given notifiable. Only thereafter
/// the consumer will reply on the bus. Requests' handling is not
/// parallelized. If the hook process cannot complete the requests fast enough,
/// the node will run out of memory and crash.
class LatencyTestConsumer : public SimpleEventHandler
{
public:
    /// To complete the hook, call the notifiable.
    using HookFn = std::function<void(Notifiable *)>;

    LatencyTestConsumer(Node *node, HookFn hook = nullptr)
        : node_(node)
        , hook_(hook)
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, EVENT_BASE), 32);
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(EVENT_BASE, 0xffffffff)),
            done->new_child());
    }

    void handle_identify_consumer(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        event_ = event;
        done_ = done;
        if (hook_)
        {
            hook_(new TempNotifiable([this]() { reply(); }));
        }
        else
        {
            reply();
        }
    }

private:
    void reply()
    {
        AutoNotify an(done_);
        event_->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
            eventid_to_buffer(event_->event), done_->new_child());
    }

    /// This is within NMRA ID 1, which is not assigned. Lower four bytes are
    /// the id.
    static constexpr EventId EVENT_BASE = 0x0900013900000000;

    /// Which node should be sending responses.
    Node *node_;

    /// The owner's hook will be invoked and run (asynchronous) before
    /// replying..
    HookFn hook_;

    /// Incoming event report we are working on.
    EventReport *event_;

    /// Will notify this after sending the reply.
    BarrierNotifiable *done_ {nullptr};
};

} // namespace openlcb

#endif // _OPENLCB_LATENCYTESTCONSUMER_HXX_
