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
 * \file RailcomBroadcastClient.hxx
 *
 * Client for receiving Railcom broadcast messages about trains in a block.
 *
 * @author Balazs Racz
 * @date 4 May 2024
 */

#ifndef _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_
#define _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_

#include <vector>
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/TractionDefs.hxx"

namespace openlcb
{

class RailcomBroadcastClient : public SimpleEventHandler
{
public:
    /** Constructor.
     * @param node The virtual node to send messages from (for identify responses).
     * @param railcomEventBase The base event ID for the Railcom broadcast range.
     *                         The bottom 16 bits will be cleared.
     */
    RailcomBroadcastClient(Node *node, uint64_t railcomEventBase);

    virtual ~RailcomBroadcastClient();

    /** Returns the list of locomotives currently reported in the block.
     * The NodeIDs are computed using the legacy DCC long address format.
     */
    const std::vector<NodeID>& current_locos() const
    {
        return locos_;
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
                             EventReport *event,
                             BarrierNotifiable *done) override;

    void handle_producer_identified(const EventRegistryEntry &registry_entry,
                                    EventReport *event,
                                    BarrierNotifiable *done) override;

    void handle_identify_global(const EventRegistryEntry &registry_entry,
                                EventReport *event,
                                BarrierNotifiable *done) override;

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
                                  EventReport *event,
                                  BarrierNotifiable *done) override;

private:
    /// Checks if the given event ID falls within our monitored range.
    bool is_our_event(uint64_t event_id) const;

    Node *node_;
    uint64_t railcomEventBase_;
    std::vector<NodeID> locos_;
};

} // namespace openlcb

#endif // _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_
