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
 * \file EventHandlerTemplates.hxx
 *
 * Defines partial implementations for event handlers that are usable for
 * multiple event handler types.
 *
 * @author Balazs Racz
 * @date 19 Octover 2013
 */

#ifndef _NMRAnet_EventHandlerTemplates_hxx_
#define _NMRAnet_EventHandlerTemplates_hxx_

#include "nmranet/NMRAnetEventRegistry.hxx"


// A proxy event handler has a single helper function that can iterate over the events contianed in this 
class ProxyEventHandler : pulic NMRAnetEventHandler {
 public:
  typedef (NMRAnetEventHandler::*EventHandlerFunction)(EventReport* event, Notifiable* done);

  virtual void ProxyEventHandler(EventHandlerFunction fn, EventReport* event, Notifiable* done) = 0;

  // Called on incoming EventReport messages. Filled: src_node, event. Mask is
  // always 1 (filled in). state is not filled in.
  virtual void HandleEventReport(EventReport* event,
                                 Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleEventReport, event, done);
  }

  // Called on another node sending ConsumerIdentified for this event. Filled:
  // event_id, mask=1, src_node, state.
  virtual void HandleConsumerIdentified(EventReport* event,
                                        Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleConsumerIdentified, event, done);
  };

  // Called on another node sending ConsumerRangeIdentified. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleConsumerRangeIdentified(EventReport* event,
                                             Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleConsumerRangeIdentified, event, done);
  }

  // Called on another node sending ProducerIdentified for this event. Filled: event_id, mask=1, src_node, state.
  virtual void HandleProducerIdentified(EventReport* event,
                                        Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleProducerIdentified, event, done);
  }

  // Called on another node sending ProducerRangeIdentified for this event. Filled: event id, mask (!= 1), src_node. Not filled: state.
  virtual void HandleProducerRangeIdentified(EventReport* event,
                                             Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleProducerRangeIdentified, event, done);
  }

  // Called on the need of sending out identification messages. This happens on
  // startup, or when a global or addressed IdentifyGlobal message
  // arrives. Might have destination node id!
  virtual void HandleIdentifyGlobal(Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleIdentifyGlobal, event, done);
  }

  // Called on another node sending IdentifyConsumer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyConsumer(EventReport* event,
                                      Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleIdentifyConsumer, event, done);
  }

  // Called on another node sending IdentifyProducer. Filled: src_node, event, mask=1. Not filled: state.
  virtual void HandleIdentifyProducer(EventReport* event,
                                      Notifiable* done) {
    ProxyEventHandler(&NMRAnetEventHandler::HandleIdentifyProducer, event, done);
  }
};



#endif // _NMRAnet_EventHandlerTemplates_hxx_
