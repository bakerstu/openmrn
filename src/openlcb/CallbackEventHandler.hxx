/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file CallbackEventHandler.hxx
 *
 * Defines an event handler implementation that is parametrized by callback
 * functions to define its properties.
 *
 * @author Balazs Racz
 * @date 18 December 2015
 */

#ifndef _OPENLCB_CALLBACKEVENTHANDLER_HXX_
#define _OPENLCB_CALLBACKEVENTHANDLER_HXX_

#include <functional>

#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Event handler implementation class that calls a user-specified
/// std::function callback every time an event report message for the given
/// event ID arrives. One event handler can request multiple event IDs to be
/// listened to. There are separate callbacks for event report and
/// requesting the current state for producer/consumer identified messages.
///
/// This is an alternate design pattern to creating subclasses of @ref
/// BitEventInterface.
class CallbackEventHandler : public SimpleEventHandler
{
public:
    /// This function (signature) is called every time a given event report
    /// arrives.
    ///
    /// @param registry_entry is the included event registry entry. The args
    /// bits are partially used internally.
    ///
    /// @param done may be used to create additional children; it does not need
    /// to be notified in the handler (the caller will do that once).
    typedef std::function<void(const EventRegistryEntry &registry_entry,
        EventReport *report, BarrierNotifiable *done)> EventReportHandlerFn;
    /// Function signature that returns the event state for the current
    /// registry entry. Implementors must note that the registry entry has to
    /// be used to figure out which bit this is and whether it is on or off.
    typedef std::function<EventState(const EventRegistryEntry &registry_entry,
        EventReport *report)> EventStateHandlerFn;
    /// This function (signature) is called when a producer/consumer identified
    /// message arrives.
    ///
    /// @param mti MTI_PRODUCER_IDENTIFIED_UNKNOWN or
    /// MTI_CONSUMER_IDENTIFIED_UNKNOWN. The state bits can be read from
    /// report->state.
    /// @param registry_entry is the included event registry entry. The args
    /// bits are partially used internally.
    /// @param report gives access to the event report, including source
    /// address, event ID, and Write Helpers.
    /// @param done may be used to create additional children; it does not need
    /// to be notified in the handler (the caller will do that once).
    typedef std::function<void(openlcb::Defs::MTI mti,
        const EventRegistryEntry &registry_entry, EventReport *report,
        BarrierNotifiable *done)>
        EventIdentifiedHandlerFn;

    enum RegistryEntryBits
    {
        /// Set this bit in the param entry_bits in order to mark the event as
        /// being produced. See {@ref add_entry}.
        IS_PRODUCER = (1U << 31),
        /// Set this bit in the param entry_bits in order to mark the event as
        /// being consumed. See {@ref add_entry}.
        IS_CONSUMER = (1U << 30),
        /// This is the mask of bits that can be used by the caller for storing
        /// arbitrary information next to the event registration.
        USER_BIT_MASK = IS_CONSUMER - 1,
    };

    /// Constructor.
    /// @param node defines which openlcb virtual node this event handler
    /// should export itself.
    /// @param report_handler will be called when an event report comes in from
    /// the network. May be null (e.g. if this is a producer only).
    /// @param state_handler will be called when the network inquires about the
    /// state of the current producer/consumer. May be null, in which case it
    /// will be reported as UNKNOWN state.
    /// @param identified_handler will be called when the network sends a state
    /// identified response. May be null. Write this handler if you need to
    /// send queries to the network.
    CallbackEventHandler(Node *node, EventReportHandlerFn report_handler,
        EventStateHandlerFn state_handler = nullptr,
        EventIdentifiedHandlerFn identified_handler = nullptr)
        : reportHandler_(std::move(report_handler))
        , stateHandler_(std::move(state_handler))
        , identifiedHandler_(std::move(identified_handler))
        , node_(node)
    {
    }

    ~CallbackEventHandler()
    {
        remove_all_entries();
    }

    /// Registers this event handler for a given event ID in the global event
    /// service's registry. This call is what actually creates the
    /// consumer/producer. It may be called multiple times to have a single
    /// object get notifications about multiple event IDs. All the calls from
    /// all entries will be forwarded to the same callback functions given in
    /// the constructor.
    /// @param event is the event ID to create the producer/consumer for.
    /// @param entry_bits set IS_PRODUCER to create a producer, IS_CONCUMER to
    /// create a consumer (or set both). The bits denoted by USER_BIT_MASK can
    /// be freely used to store arbitrary data, which will be available to the
    /// handler functions as registry_entry.user_arg.
    void add_entry(EventId event, uint32_t entry_bits)
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, event, entry_bits), 0);
    }

    /// Removes the registration of every single entry added so far.
    void remove_all_entries()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    /// Removes the registration of entries added before with a given user_arg
    /// value.
    /// @param user_arg argument to match on.
    void remove_entry(uint32_t entry_bits)
    {
        EventRegistry::instance()->unregister_handler(
            this, entry_bits, 0xFFFFFFFFu);
    }

    /// @return the node pointer for which this handler is exported.
    Node *node()
    {
        return node_;
    }

    void handle_event_report(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (reportHandler_)
        {
            reportHandler_(entry, event, done);
        }
        done->notify();
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (registry_entry.user_arg & IS_CONSUMER)
        {
            send_consumer_identified(registry_entry, event, done);
        }
        done->notify();
    };

    void handle_identify_producer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (registry_entry.user_arg & IS_PRODUCER)
        {
            send_producer_identified(registry_entry, event, done);
        }
        done->notify();
    };

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (registry_entry.user_arg & IS_PRODUCER)
        {
            send_producer_identified(registry_entry, event, done);
        }
        if (registry_entry.user_arg & IS_CONSUMER)
        {
            send_consumer_identified(registry_entry, event, done);
        }
        done->notify();
    };

    void handle_producer_identified(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (identifiedHandler_)
        {
            identifiedHandler_(
                Defs::MTI::MTI_PRODUCER_IDENTIFIED_UNKNOWN, entry, event, done);
        }
        done->notify();
    }

    void handle_consumer_identified(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        if (identifiedHandler_)
        {
            identifiedHandler_(
                Defs::MTI::MTI_CONSUMER_IDENTIFIED_UNKNOWN, entry, event, done);
        }
        done->notify();
    }

protected:
    /// Helper function for implementations.
    void send_producer_identified(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done)
    {
        EventState state =
            stateHandler_ ? stateHandler_(entry, event) : EventState::UNKNOWN;
        Defs::MTI mti = Defs::MTI_PRODUCER_IDENTIFIED_VALID + state;
        event->event_write_helper<1>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(entry.event),
            done->new_child());
    }

    /// Helper function for implementations.
    void send_consumer_identified(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done)
    {
        EventState state =
            stateHandler_ ? stateHandler_(entry, event) : EventState::UNKNOWN;
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID + state;
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(entry.event),
            done->new_child());
    }

private:
    /// Stores the user callback for event reports.
    EventReportHandlerFn reportHandler_;
    /// Stores the user callback for getting state for event identified
    /// responses.
    EventStateHandlerFn stateHandler_;
    /// Stores the user callback for P/C identified messages.
    EventIdentifiedHandlerFn identifiedHandler_;
    /// Node on which we are registered.
    Node *node_;
};

} // namespace openlcb

#endif // _OPENLCB_CALLBACKEVENTHANDLER_HXX_
