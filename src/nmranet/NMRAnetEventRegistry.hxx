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
 * \file NMRAnetEventRegistry.hxx
 * Interface for handling NMRAnet events.
 *
 * @author Balazs Racz
 * @date 29 September 2013
 */

#ifndef _NMRAnetEventRegistry_hxx_
#define _NMRAnetEventRegistry_hxx_

#include <stdint.h>

#include "executor/notifiable.hxx"
#include "utils/AsyncMutex.hxx"
#include "utils/macros.h"
#include "nmranet/NMRAnetWriteFlow.hxx"

namespace NMRAnet
{

typedef uint64_t EventId;
class AsyncNode;

enum EventState {
  VALID = 0,
  INVALID = 1,
  UNKNOWN = 2,
  RESERVED = 3
};

/*enum EventMask {
  EVENT_EXACT_MASK = 1,
  EVENT_ALL_MASK = 0xffffffffffffffffULL
  };*/

typedef struct {
  EventId event;
  EventId mask;
  NodeHandle src_node;
  AsyncNode* dst_node;
  EventState state;
} EventReport;

// Static objects usable by all event handler implementations

// These allow event handlers to produce up to four messages per
// invocation. They are locked by the event-handler_mutex and always available
// at the entry to an event handler function.
extern WriteHelper event_write_helper1;
extern WriteHelper event_write_helper2;
extern WriteHelper event_write_helper3;
extern WriteHelper event_write_helper4;

class NMRAnetEventHandler {
public:
  virtual ~NMRAnetEventHandler() {}

  // Called on incoming EventReport messages. Filled: src_node, event. Mask is
  // always 1 (filled in). state is not filled in.
  virtual void HandleEventReport(EventReport* event,
                                 BarrierNotifiable* done) = 0;

  // Called on another node sending ConsumerIdentified for this event. Filled:
  // event_id, mask=1, src_node, state.
  virtual void HandleConsumerIdentified(EventReport* event,
                                        BarrierNotifiable* done) {
    done->notify();
  };

  // Called on another node sending ConsumerRangeIdentified. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleConsumerRangeIdentified(EventReport* event,
                                             BarrierNotifiable* done) {
    done->notify();
  }

  // Called on another node sending ProducerIdentified for this event. Filled: event_id, mask=1, src_node, state.
  virtual void HandleProducerIdentified(EventReport* event,
                                        BarrierNotifiable* done) {
    done->notify();
  }

  // Called on another node sending ProducerRangeIdentified for this event. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleProducerRangeIdentified(EventReport* event,
                                             BarrierNotifiable* done) {
    done->notify();
  }

  // Called on the need of sending out identification messages. event is
  // NULL. This happens on startup, or when a global or addressed
  // IdentifyGlobal message arrives. Might have destination node id!
  virtual void HandleIdentifyGlobal(EventReport* event, BarrierNotifiable* done) = 0;

  // Called on another node sending IdentifyConsumer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyConsumer(EventReport* event,
                                      BarrierNotifiable* done) = 0;

  // Called on another node sending IdentifyProducer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyProducer(EventReport* event,
                                      BarrierNotifiable* done) = 0;
};

typedef void (NMRAnetEventHandler::*EventHandlerFunction)(EventReport* event,
                                                          BarrierNotifiable* done);


// Abstract class for representing iteration through a container for event
// handlers.
class EventIterator {
protected:
    /// Creates an EventIterator.
    EventIterator() {}

public:
    virtual ~EventIterator() {}

    /** Steps the iteration.
     * @returns the next entry or NULL if the iteration is done.
     * May be called many times after the iteratin is ended and should
     * consistently return NULL. */
    virtual NMRAnetEventHandler* next_entry() = 0;

    /** Starts the iteration. If the iteration is not done yet, call
     * clear_iteration first.
     *
     * @param event is the event report to reset the iteration for. */
    virtual void init_iteration(EventReport* event) = 0;

    /** Stops iteration and resets iteration variables. */
    virtual void clear_iteration() = 0;
};

class NMRAnetEventRegistry {
public:
  virtual ~NMRAnetEventRegistry();

  static NMRAnetEventRegistry* instance() {
    HASSERT(instance_);
    return instance_;
  }

  // mask = 0 means exact event only. Mask = 64 means this is a global handler.
  virtual void register_handler(NMRAnetEventHandler* handler, EventId event, unsigned mask) = 0;
  virtual void unregister_handler(NMRAnetEventHandler* handler, EventId event, unsigned mask) = 0;
  
    // Creates a new event iterator. Caller takes ownership of object.
    virtual EventIterator* create_iterator() = 0;

protected:
  NMRAnetEventRegistry();

private:
  static NMRAnetEventRegistry* instance_;

  DISALLOW_COPY_AND_ASSIGN(NMRAnetEventRegistry);
};

}; /* namespace NMRAnet */

#endif  // _NMRAnetEventRegistry_hxx_
