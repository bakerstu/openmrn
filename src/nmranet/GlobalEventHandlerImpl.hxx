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

class GlobalEventService::Impl
{
public:
    Impl();
    ~Impl();

    /* The implementation of the event handler. This will typically be a proxy
     * for registering all events. */
    std::unique_ptr<NMRAnetEventHandler> handler_;

    /** Flows that we own. There will be a few entries for each interface
     * registered. */
    std::vector<std::unique_ptr<Executable>> owned_flows_;

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
class GlobalEventFlow : public MessageStateFlowBase
{
public:
    GlobalEventFlow(GlobalEventService *service, AsyncIf *interface);
    ~GlobalEventFlow();

    /// Statically points to the global instance of the event handler.
    static GlobalEventFlow *instance;

protected:
    Action entry() OVERRIDE;

    Action call_handler();
    Action handler_finished();
    Action WaitForHandler();
    Action HandlerFinished();

    /** This function will mask the interface() function of
     * IncomingMessageStateFlow */
    AsyncIf *interface()
    {
        return interface_;
    }

    GlobalEventService *service()
    {
        return static_cast<GlobalEventService *>(
            MessageStateFlowBase::service());
    }

    /// Returns the NMRAnet message we received.
    NMRAnetMessage *nmsg()
    {
        return message()->data();
    }

private:
    // Statically allocated structure for calling the event handlers from the
    // main event queue.
    EventReport main_event_report_;

    AsyncIf *interface_;
};

} // namespace NMRAnet

#endif // _NMRANET_GLOBAL_EVENT_HANDLER_IMPL_
