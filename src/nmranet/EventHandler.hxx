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
 * \file EventHandler.hxx
 * Interface for handling NMRAnet events.
 *
 * @author Balazs Racz
 * @date 29 September 2013
 */

#ifndef _NMRANET_EVENTHANDLER_HXX_
#define _NMRANET_EVENTHANDLER_HXX_

#include <stdint.h>

#include "executor/Notifiable.hxx"
#include "utils/AsyncMutex.hxx"
#include "utils/macros.h"
#include "nmranet/WriteHelper.hxx"

namespace nmranet
{

typedef uint64_t EventId;
class Node;
class EventHandler;

/*enum EventMask {
  EVENT_EXACT_MASK = 1,
  EVENT_ALL_MASK = 0xffffffffffffffffULL
  };*/

/// Shared notification structure that is assembled for each incoming
/// event-related message, and passed around to all event handlers.
typedef struct
{
    /// The event ID from the incoming message.
    EventId event;
    /// Specifies the mask in case the request is for an event range. The low
    /// bits are set to one, the high bits are set to zero. Ranges of size 1
    /// have
    /// mask==0, a range that covers all events has mask==0xffff...f.
    EventId mask;
    /// Information about the sender of the incoming event-related OpenLCB
    /// message. It is not specified whether the node_id or the alias is
    /// specified, but they are not both zero.
    NodeHandle src_node;
    /// nullptr for global messages; points to the specific virtual node for
    /// addressed events identify message.
    Node *dst_node;
    /// For producer/consumer identified messages, specifies the state of the
    /// producer/consumer as the sender of the message
    /// (valid/invalid/unknown/reserved).
    EventState state;
} EventReport;

/// Structure used in registering event handlers.
class EventRegistryEntry
{
public:
    /// Stores the event ID or beginning of range for which to register the
    /// given handler.
    EventId event;
    /// Pointer to the handler.
    EventHandler *handler;
    /// Opaque user argument. The event handlers may use this to store
    /// arbitrary data.
    uint32_t user_arg;
    EventRegistryEntry(EventHandler *_handler, EventId _event)
        : event(_event)
        , handler(_handler)
        , user_arg(0)
    {
    }
    EventRegistryEntry(EventHandler *_handler, EventId _event,
                       unsigned _user_arg)
        : event(_event)
        , handler(_handler)
        , user_arg(_user_arg)
    {
    }
};

// Static objects usable by all event handler implementations

// These allow event handlers to produce up to four messages per
// invocation. They are locked by the event-handler_mutex and always available
// at the entry to an event handler function.
extern WriteHelper event_write_helper1;
extern WriteHelper event_write_helper2;
extern WriteHelper event_write_helper3;
extern WriteHelper event_write_helper4;

/// Abstract base class for all event handlers. Instances of this class can
/// get registered with the event service to receive notifications of incoming
/// event messages from the bus.
class EventHandler
{
public:
    using EventReport = nmranet::EventReport;
    using EventRegistryEntry = nmranet::EventRegistryEntry;
    using EventId = nmranet::EventId;

    virtual ~EventHandler()
    {
    }

    /// Called on incoming EventReport messages. @param event stores
    /// information about the incoming message. Filled: src_node, event. Mask
    /// is always 1 (filled in). state is not filled in. @param registry_entry
    /// gives the registry entry for which the current handler is being
    /// called. @param done must be notified when the processing is done.
    virtual void handle_event_report(const EventRegistryEntry &registry_entry,
                                   EventReport *event,
                                   BarrierNotifiable *done) = 0;

    /// Called on another node sending ConsumerIdentified for this event.
    /// @param event stores information about the incoming message. Filled:
    /// event_id, mask=1, src_node, state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// done must be notified when the processing is done.
    virtual void
    HandleConsumerIdentified(const EventRegistryEntry &registry_entry,
                             EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    };

    /// Called on another node sending ConsumerRangeIdentified. @param event
    /// stores information about the incoming message. Filled: event id, mask
    /// (!= 1), src_node. Not filled: state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// done must be notified when the processing is done.
    virtual void
    HandleConsumerRangeIdentified(const EventRegistryEntry &registry_entry,
                                  EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on another node sending ProducerIdentified for this event.
    /// @param event stores information about the incoming message. Filled:
    /// event_id, mask=1, src_node, state.  @param registry_entry gives the
    /// registry entry for which the current handler is being called. @param
    /// user_arg is an opaque argument passed in from the registration. @param
    /// done must be notified when the processing is done.
    virtual void
    HandleProducerIdentified(const EventRegistryEntry &registry_entry,
                             EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on another node sending ProducerRangeIdentified for this
    /// event. @param event stores information about the incoming
    /// message. Filled: event id, mask (!= 1), src_node. Not filled: state.
    /// @param registry_entry gives the registry entry for which the current
    /// handler is being called. @param done must be notified when the
    /// processing is done.
    virtual void
    HandleProducerRangeIdentified(const EventRegistryEntry &registry_entry,
                                  EventReport *event, BarrierNotifiable *done)
    {
        done->notify();
    }

    /// Called on the need of sending out identification messages. @param event
    /// is NULL. This happens on startup, or when a global or addressed
    /// IdentifyGlobal message arrives. Might have destination node id! @param
    /// registry_entry gives the registry entry for which the current handler
    /// is being called. @param done must be notified when the processing is
    /// done.
    virtual void HandleIdentifyGlobal(const EventRegistryEntry &registry_entry,
                                      EventReport *event,
                                      BarrierNotifiable *done) = 0;

    /// Called on another node sending IdentifyConsumer. @param event stores
    /// information about the incoming message. Filled: src_node, event,
    /// mask=1. Not filled: state. @param registry_entry gives the registry
    /// entry for which the current handler is being called. @param done must
    /// be notified when the processing is done.
    virtual void
    HandleIdentifyConsumer(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) = 0;

    /// Called on another node sending IdentifyProducer. @param event stores
    /// information about the incoming message. Filled: src_node, event,
    /// mask=1. Not filled: state.  @param registry_entry gives the registry
    /// entry for which the current handler is being called. @param done must
    /// be notified when the processing is done.
    virtual void
    HandleIdentifyProducer(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) = 0;
};

typedef void (EventHandler::*EventHandlerFunction)(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done);

class EventIterator;

/// Global static object for registering event handlers.
///
/// Usage: create one of the implementation classes depending on the resource
/// requirements of your binary. In the event handlers constructor, register
/// the event handler via the singleton pointer.
///
/// @TODO(balazs.racz) transition to the usual Singleton class instead of
/// hand-initialized singleton pointer.
class EventRegistry
{
public:
    virtual ~EventRegistry();

    static EventRegistry *instance()
    {
        HASSERT(instance_);
        return instance_;
    }

    /** Computes the alignment mask for registering an event range. Updates the
     * event by rounding and returns the mask value to be sent to the
     * register_handler function.
     * @param event is the event id to be registered. Will be modified.
     * @param is the number of events to register from that offset. [event,
     * event+size) will be the registration range.
     */
    static unsigned align_mask(EventId *event, unsigned size);

    /// Adds a new event handler to the registry.
    virtual void register_handler(const EventRegistryEntry &entry,
                                  unsigned mask) = 0;
    /// Removes all registered instances of a given event handler pointer.
    virtual void unregister_handler(EventHandler *handler) = 0;

    /// Creates a new event iterator. Caller takes ownership of object.
    virtual EventIterator *create_iterator() = 0;

    /// Returns a monotonically increasing number that will change every time
    /// the set of registered event handlers change. Whenever this number
    /// changes, the iterators are invalidated and must be cleared.
    unsigned get_epoch()
    {
        return dirtyCounter_;
    }

protected:
    EventRegistry();

    /// Implementations must call this function from register and unregister
    /// handler to mark iterators being invalidated.
    void set_dirty()
    {
        ++dirtyCounter_;
    }

private:
    static EventRegistry *instance_;

    /// This counter will be incremented every time the set of event handlers
    /// change (and thus the event iterators are invalidated).
    unsigned dirtyCounter_ = 0;

    DISALLOW_COPY_AND_ASSIGN(EventRegistry);
};

}; /* namespace nmranet */

#endif // _NMRANET_EVENTHANDLER_HXX_
