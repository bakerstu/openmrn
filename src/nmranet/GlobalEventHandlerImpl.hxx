/** \copyright
 * Copyright (c) 2014, Balazs Racz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
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
 * \file GlobalEventHandlerImpl.hxx
 *
 * Implementation headers shared between components of the global event
 * handler's various flows. An end-user will typically not need to include this
 * header.
 *
 * @author Balazs Racz
 * @date 20 April 2014
 */

#ifndef _NMRANET_GLOBAL_EVENT_HANDLER_IMPL_
#define _NMRANET_GLOBAL_EVENT_HANDLER_IMPL_

#include <memory>
#include <vector>

#include "nmranet/GlobalEventHandler.hxx"
#include "nmranet/NMRAnetEventRegistry.hxx"

namespace NMRAnet
{

class IncomingEventFlow;
class GlobalIdentifyFlow;
class NMRAnetEventHandler;

struct EventHandlerCall
{
    EventReport *rep;
    NMRAnetEventHandler *handler;
    EventHandlerFunction fn;
    void reset(EventReport *rep, NMRAnetEventHandler *handler, EventHandlerFunction fn)
    {
        this->rep = rep;
        this->handler = handler;
        this->fn = fn;
    }
};

class EventCallerFlow : public StateFlow<Buffer<EventHandlerCall>, QList<4> > {
public:
    EventCallerFlow(Service* service);

private:
    virtual Action entry() OVERRIDE;
    Action call_done();

    BarrierNotifiable n_;
};

class GlobalEventService::Impl
{
public:
    Impl(GlobalEventService* service);
    ~Impl();

    /* The implementation of the event registry. */
    std::unique_ptr<NMRAnetEventRegistry> registry;

    /** Flows that we own. There will be a few entries for each interface
     * registered. */
    std::vector<std::unique_ptr<Executable>> owned_flows_;

    /** This flow will serialize calls to NMRAnetEventHandler objects. All such
     * calls need to be sent to this flow. */
    EventCallerFlow callerFlow_;

    enum
    {
        // These address/mask should match all the messages carrying an event
        // id.
        MTI_VALUE_EVENT = If::MTI_EVENT_MASK,
        MTI_MASK_EVENT = If::MTI_EVENT_MASK,
        // These match the two event messages without event id: Global and
        // addressed identify all events.
        MTI_VALUE_GLOBAL =
            If::MTI_EVENTS_IDENTIFY_ADDRESSED & If::MTI_EVENTS_IDENTIFY_GLOBAL,
        MTI_MASK_GLOBAL = (~If::MTI_SIMPLE_MASK) & (~If::MTI_ADDRESS_MASK),
    };
};

/** Flow to receive incoming messages of event protocol, and dispatch them to
 * the global event handler. This flow runs on the executor of the event
 * service (and not necessarily the interface). */
class GlobalEventFlow : public IncomingMessageStateFlow
{
public:
    GlobalEventFlow(AsyncIf *interface, GlobalEventService *event_service);
    ~GlobalEventFlow();

protected:
    Action entry() OVERRIDE;
    Action iterate_next();

private:
    GlobalEventService* eventService_;

    // Statically allocated structure for calling the event handlers from the
    // main event queue.
    EventReport eventReport_;

    /** Iterator for generating the event handlers from the registry. */
    std::unique_ptr<EventIterator> iterator_;

    BarrierNotifiable n_;
    EventHandlerFunction fn_;

    AsyncIf *interface_;
};

} // namespace NMRAnet

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_IMPL_
