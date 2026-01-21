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
 * \file RailcomBroadcastClient.cxx
 *
 * Client for receiving Railcom broadcast messages about trains in a block.
 *
 * @author Balazs Racz
 * @date 15 Jan 2026
 */

#include "openlcb/RailcomBroadcastClient.hxx"

#include <algorithm>

#include "openlcb/Defs.hxx"
#include "openlcb/EventHandler.hxx"

namespace openlcb
{

RailcomBroadcastClient::RailcomBroadcastClient(
    Node *node, uint64_t railcom_event_base)
    : node_(node)
{
    railcomEventBase_ = railcom_event_base & ~0xFFFFULL;
    uint64_t registered_base = railcomEventBase_;
    // Register for 65536 events (16 bits)
    unsigned mask = EventRegistry::align_mask(&registered_base, 65536);
    EventRegistry::instance()->register_handler(
        EventRegistryEntry(this, registered_base), mask);
}

RailcomBroadcastClient::~RailcomBroadcastClient()
{
    EventRegistry::instance()->unregister_handler(this);
}

bool RailcomBroadcastClient::is_our_event(uint64_t event_id) const
{
    // Check if the event ID is within the range [base, base + 0xFFFF]
    return (event_id & ~0xFFFFULL) == railcomEventBase_;
}

RailcomBroadcastClient::LocoInfo RailcomBroadcastClient::node_id_from_event(
    uint64_t event)
{
    uint16_t lo = event & 0xFFFF;
    if (!lo)
    {
        return LocoInfo(0, true);
    }

    // Extract address from bottom 14 bits
    uint16_t address = event & 0x3FFF;
    NodeID id = 0;
    uint8_t partition = address >> 8;
    if (partition == dcc::Defs::ADR_MOBILE_SHORT ||
        partition == dcc::Defs::ADR_CONSIST_SHORT)
    {
        address &= 127;
        if (!address)
        {
            id = 0;
        }
        else
        {
            id = TractionDefs::train_node_id_from_legacy(
                dcc::TrainAddressType::DCC_SHORT_ADDRESS, address);
        }
    }
    else if (partition <= dcc::Defs::MAX_MOBILE_LONG)
    {
        id = TractionDefs::train_node_id_from_legacy(
            dcc::TrainAddressType::DCC_LONG_ADDRESS, address);
    }
    else
    {
        // Unknown railcom address partition
        id = 0;
    }
    unsigned dirbits = (event & 0xC000) >> 14;
    bool is_west = false;
    switch (dirbits)
    {
        case 0b11:
            // unknown direction
            is_west = false;
            break;
        case 0b01:
            is_west = true;
            break;
        case 0b10:
            is_west = false;
            break;
    }
    return LocoInfo(id, is_west);
}

void RailcomBroadcastClient::handle_event_report(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done)
{
    AutoNotify an(done);
    if (!is_our_event(event->event))
    {
        return;
    }

    auto id = node_id_from_event(event->event);
    add_loco(id);
}

void RailcomBroadcastClient::add_loco(LocoInfo id)
{
    if (id.empty())
    {
        // Unoccupied.
        if (locos_.size())
        {
            locos_.clear();
            seq_++;
        }
        return;
    }
    bool found = false;
    for (unsigned i = 0; i < locos_.size(); ++i)
    {
        if (locos_[i].node_id() == id.node_id())
        {
            found = true;
            locos_[i] = id;
            break;
        }
    }

    if (!found)
    {
        locos_.push_back(id);
        seq_++;
    }
}

void RailcomBroadcastClient::del_loco(LocoInfo id)
{
    auto it = std::remove(locos_.begin(), locos_.end(), id);
    if (it != locos_.end())
    {
        locos_.erase(it, locos_.end());
        seq_++;
    }
}

void RailcomBroadcastClient::handle_producer_identified(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done)
{
    if (!is_our_event(event->event))
    {
        return done->notify();
    }

    auto id = node_id_from_event(event->event);
    if (event->state == EventState::INVALID)
    {
        del_loco(id);
    }
    else if (event->state == EventState::VALID)
    {
        add_loco(id);
    }

    done->notify();
}

void RailcomBroadcastClient::handle_identify_global(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done)
{
    if (event->dst_node && event->dst_node != node_)
    {
        return done->notify();
    }

    // We are a consumer of these events.
    uint64_t range = EncodeRange(railcomEventBase_, 65536);
    event->event_write_helper<1>()->WriteAsync(node_,
        Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
        eventid_to_buffer(range), done->new_child());
    // Queries current state from the producer.
    event->event_write_helper<2>()->WriteAsync(node_,
        Defs::MTI_PRODUCER_IDENTIFY, WriteHelper::global(),
        eventid_to_buffer(railcomEventBase_), done->new_child());
    done->maybe_done();
}

void RailcomBroadcastClient::handle_identify_consumer(
    const EventRegistryEntry &registry_entry, EventReport *event,
    BarrierNotifiable *done)
{
    if (!is_our_event(event->event))
    {
        return done->notify();
    }

    // Report unknown state for specific events queried
    event->event_write_helper<1>()->WriteAsync(node_,
        Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
        eventid_to_buffer(event->event), done->new_child());
    done->maybe_done();
}

} // namespace openlcb
