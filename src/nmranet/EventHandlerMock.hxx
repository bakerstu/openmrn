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
 * \file EventHandlerMock.hxx
 *
 * Helper utilities for testing event handlers.
 *
 * This file must only ever be included in unittests.
 *
 * @author Balazs Racz
 * @date 7 December 2013
 */

#ifndef _NMRANET_EVENTHANDLERMOCK_HXX_
#define _NMRANET_EVENTHANDLERMOCK_HXX_

#include "gmock/gmock.h"
#include "nmranet/EventHandler.hxx"

namespace nmranet {

/// Test handler for receiving incoming event related messages via the
/// EventService. Incoming messages need GoogleMock expectations.
class MockEventHandler : public EventHandler
{
public:
#define DEFPROXYFN(FN)                                                         \
    MOCK_METHOD3(FN, void(const EventRegistryEntry &, EventReport *event,      \
                          BarrierNotifiable *done))

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

}  // namespace nmranet

#endif // _NMRAnetEventHandlerTemplates_hxx_


