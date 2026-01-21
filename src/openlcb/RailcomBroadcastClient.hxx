/** \copyright
 * Copyright (c) 2026, Balazs Racz
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
 * @date 15 Jan 2026
 */

#ifndef _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_
#define _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_

#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/TractionDefs.hxx"
#include <vector>

namespace openlcb
{

/// Client class for receiving event based updates for the occupancy of a
/// railcom block. Consumes events for entry and exit reports from the railcom
/// broadcast flow, and maintains a list of locomotives that are currently in
/// the block.
class RailcomBroadcastClient : public SimpleEventHandler
{
public:
    /** Constructor.
     * @param node The virtual node to send messages from (for identify
     * responses).
     * @param railcom_event_base The base event ID for the Railcom broadcast
     * range. The bottom 16 bits will be cleared.
     */
    RailcomBroadcastClient(Node *node, uint64_t railcom_event_base);

    virtual ~RailcomBroadcastClient();

    /// This structure is kept in the current_locos array.
    struct LocoInfo
    {
        LocoInfo(NodeID id, bool is_west)
            : nodeId_(id)
        {
            set_west(is_west);
        }
        /// @return the Node ID of the locomotive.
        NodeID node_id() const
        {
            return nodeId_;
        }

        /// @return true if the orientation of the locomotive (according to the
        /// polarity of the railcom signal received) is west. This means if the
        /// locomotive were to get a "drive forward" command, it would
        /// physically move westbound.
        bool orientation_is_west() const
        {
            return isWest_ != 0;
        }

        bool empty()
        {
            return nodeId_ == 0;
        }

        /// Equality comparison on keys only.
        bool operator==(const LocoInfo &o)
        {
            return nodeId_ == o.nodeId_;
        }

    private:
        friend class RailcomBroadcastClient;

        void set_west(bool is_west)
        {
            isWest_ = is_west ? 1 : 0;
        }

        /// Locomotive Node ID.
        uint64_t nodeId_ : 48;
        /// Observed polarity of the RailCom messages, 1 = positive polarity
        /// (west).
        uint64_t isWest_ : 1;
        // There is additional space here for more properties; for example
        // those from RailCom ID3.
    };

    /** Returns the list of locomotives currently reported in the block.
     * The NodeIDs are computed using the legacy DCC long address format.
     */
    const std::vector<LocoInfo> &current_locos() const
    {
        return locos_;
    }

    /// The number returned by this function is changed every time the
    /// current_locos() array is different. This allows a user to keep track of
    /// whether their state is dirty or not.
    uint16_t seq()
    {
        return seq_;
    }

    /// @return the event base which we are listening to.
    uint64_t event_base()
    {
        return railcomEventBase_;
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override;

    void handle_producer_identified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override;

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override;

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override;

private:
    /// Checks if the given event ID falls within our monitored range.
    bool is_our_event(uint64_t event_id) const;

    /// Parses an event into a LocoInfo structure. This will contain the node
    /// ID and the direction bit.
    static LocoInfo parse_event(uint64_t event);

    /// Adds a locomotive to the locos_ array.
    void add_loco(LocoInfo n);
    /// Removes a locomotive from the locos_ array.
    void del_loco(LocoInfo n);

    /// Sequence number for data version in the locos_ array.
    uint16_t seq_ {0};
    /// OpenLCB node on which to export the consumer.
    Node *node_;
    /// Event ID with bottom 16 bits as zero. We are registered for this event
    /// range.
    uint64_t railcomEventBase_;
    /// Locomotives that are currently active in this range.
    std::vector<LocoInfo> locos_;
};

} // namespace openlcb

#endif // _OPENLCB_RAILCOMBROADCASTCLIENT_HXX_
