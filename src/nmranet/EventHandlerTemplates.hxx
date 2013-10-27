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

// A proxy event handler has a single helper function that gets every event
// handler call with an indication of which call it is. It is helpful to create
// event containers that proxy calls to many event handler instances.
class ProxyEventHandler : public NMRAnetEventHandler {
 public:
  virtual ~ProxyEventHandler() {}

  typedef void (NMRAnetEventHandler::*EventHandlerFunction)(EventReport* event,
                                                            Notifiable* done);

  // This function will be called for any other incoming event handler
  // function.
  virtual void HandlerFn(EventHandlerFunction fn,
                         EventReport* event,
                         Notifiable* done) = 0;

#define DEFPROXYFN(FN)                                    \
  virtual void FN(EventReport* event, Notifiable* done) { \
    HandlerFn(&NMRAnetEventHandler::FN, event, done);     \
  }

  DEFPROXYFN(HandleEventReport);
  DEFPROXYFN(HandleConsumerIdentified);
  DEFPROXYFN(HandleConsumerRangeIdentified);
  DEFPROXYFN(HandleProducerIdentified);
  DEFPROXYFN(HandleProducerRangeIdentified);
  DEFPROXYFN(HandleIdentifyGlobal);
  DEFPROXYFN(HandleIdentifyConsumer);
  DEFPROXYFN(HandleIdentifyProducer);

#undef DEFPROXYFN
};

#endif  // _NMRAnet_EventHandlerTemplates_hxx_
