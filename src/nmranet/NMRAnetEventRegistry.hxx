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
#include "utils/macros.h"
#include "nmranet_types.h"

typedef uint64_t EventId;

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
  node_handle_t src_node;
  node_t dst_node;
  EventState state;
} EventReport;

class NMRAnetEventHandler {
public:
  virtual ~NMRAnetEventHandler() {}

  // Called on incoming EventReport messages. Filled: src_node, event. Mask is
  // always 1 (filled in). state is not filled in.
  virtual void HandleEventReport(EventReport* event,
                                 Notifiable* done) = 0;

  // Called on another node sending ConsumerIdentified for this event. Filled:
  // event_id, mask=1, src_node, state.
  virtual void HandleConsumerIdentified(EventReport* event,
                                        Notifiable* done) {
    done->Notify();
  };

  // Called on another node sending ConsumerRangeIdentified. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleConsumerRangeIdentified(EventReport* event,
                                             Notifiable* done) {
    done->Notify();
  }

  // Called on another node sending ProducerIdentified for this event. Filled: event_id, mask=1, src_node, state.
  virtual void HandleProducerIdentified(EventReport* event,
                                        Notifiable* done) {
    done->Notify();
  }

  // Called on another node sending ProducerRangeIdentified for this event. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleProducerRangeIdentified(EventReport* event,
                                             Notifiable* done) {
    done->Notify();
  }

  // Called on the need of sending out identification messages. event is
  // NULL. This happens on startup, or when a global or addressed
  // IdentifyGlobal message arrives. Might have destination node id!
  virtual void HandleIdentifyGlobal(EventReport* event, Notifiable* done) = 0;

  // Called on another node sending IdentifyConsumer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyConsumer(EventReport* event,
                                      Notifiable* done) = 0;

  // Called on another node sending IdentifyProducer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyProducer(EventReport* event,
                                      Notifiable* done) = 0;
};

class NMRAnetEventRegistry {
public:
  static NMRAnetEventRegistry* instance() {
    HASSERT(instance_);
    return instance_;
  }

  // mask = 0 means exact event only. Mask = 64 means this is a global handler.
  virtual void RegisterHandler(NMRAnetEventHandler* handler, EventId event, unsigned mask) = 0;
  virtual void UnregisterHandler(NMRAnetEventHandler* handler, EventId event, unsigned mask) = 0;
  
  virtual NMRAnetEventHandler* EventHandler() = 0;

protected:
  NMRAnetEventRegistry();
  ~NMRAnetEventRegistry();

private:
  static NMRAnetEventRegistry* instance_;

  DISALLOW_COPY_AND_ASSIGN(NMRAnetEventRegistry);
};

#endif  // _NMRAnetEventRegistry_hxx_
