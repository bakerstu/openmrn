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
 * \file WellKnownEventRangeConsumer.hxx
 *
 * Base class for consumers of well-known event ranges like DCC turnout and
 * sensors.
 *
 * @author Balazs Racz
 * @date 18 Jan 2026
 */

#ifndef _OPENLCB_WELLKNOWNEVENTRANGECONSUMER_HXX_
#define _OPENLCB_WELLKNOWNEVENTRANGECONSUMER_HXX_

#include <stdint.h>
#include <vector>

#include "openlcb/EventHandlerTemplates.hxx"

namespace openlcb
{

/// Configuration structure for defining event ranges.
struct EventRangeConfig
{
    /// Event ID base for the active state range.
    uint64_t activate_base;
    /// Event ID base for the inactive state range.
    uint64_t inactivate_base;
    /// Number of bits in the event registry mask (e.g., 12 for 4096 events).
    uint32_t mask_bits;
    /// Total number of state bits managed (e.g. 2048 or 4096).
    uint32_t state_bit_count;
};

/// Base class for consumers of well-known event ranges.
class WellKnownEventRangeConsumer : public SimpleEventHandler
{
public:
protected:
    /// Constructs a listener for well-known event ranges.
    /// @param node is the virtual node that will be listening for events.
    /// @param cfg is the configuration structure for the event ranges. The
    /// pointer has to remain valid through the entire life of the object. It
    /// is OK to store this in flash.
    WellKnownEventRangeConsumer(Node *node, const EventRangeConfig *cfg)
        : node_(node)
        , cfg_(cfg)
        , stateWordCount_((cfg->state_bit_count + 31) / 32)
    {
        lastSetState_ = new uint32_t[stateWordCount_];
        isStateKnown_ = new uint32_t[stateWordCount_];
        memset(lastSetState_, 0, stateWordCount_ * sizeof(uint32_t));
        memset(isStateKnown_, 0, stateWordCount_ * sizeof(uint32_t));

        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, cfg_->activate_base), cfg_->mask_bits);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(this, cfg_->inactivate_base), cfg_->mask_bits);
    }

    /// Destructor.
    ~WellKnownEventRangeConsumer()
    {
        EventRegistry::instance()->unregister_handler(this);
        delete[] lastSetState_;
        delete[] isStateKnown_;
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        if (registry_entry.event == cfg_->activate_base)
        {
            event->event_write_helper<1>()->WriteAsync(node_,
                Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
                eventid_to_buffer(
                    EncodeRange(cfg_->activate_base, 1UL << cfg_->mask_bits)),
                done->new_child());
        }
        if (registry_entry.event == cfg_->inactivate_base)
        {
            event->event_write_helper<2>()->WriteAsync(node_,
                Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
                eventid_to_buffer(
                    EncodeRange(cfg_->inactivate_base, 1UL << cfg_->mask_bits)),
                done->new_child());
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        uint32_t address;
        bool value;
        if (!parse_event(event->event, &address, &value))
        {
            return;
        }

        uint32_t word_index = address / 32;
        uint32_t bit_mask = 1UL << (address & 31);

        if (word_index >= stateWordCount_)
        {
            return;
        }

        isStateKnown_[word_index] |= bit_mask;
        if (value)
        {
            lastSetState_[word_index] |= bit_mask;
        }
        else
        {
            lastSetState_[word_index] &= ~bit_mask;
        }

        action_impl();
    }

    void handle_identify_consumer(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        uint32_t address;
        bool value;
        if (!parse_event(event->event, &address, &value))
        {
            return;
        }

        uint32_t word_index = address / 32;
        uint32_t bit_mask = 1UL << (address & 31);

        if (word_index >= stateWordCount_)
            return;

        EventState s;
        if (isStateKnown_[word_index] & bit_mask)
        {
            bool stored_state = (lastSetState_[word_index] & bit_mask) != 0;
            if (stored_state == value)
            {
                s = EventState::VALID;
            }
            else
            {
                s = EventState::INVALID;
            }
        }
        else
        {
            s = EventState::UNKNOWN;
        }
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID + s;
        event->event_write_helper<1>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(event->event),
            done->new_child());
    }

    /// Perform action after state change.
    virtual void action_impl() = 0;

    /// Parses an event into identifying properties.
    /// @param event the event ID to parse.
    /// @param address will be set to the 0-based binary address of the bit.
    /// @param value will be set to the boolean value this event represents.
    /// @return true if the event is in range (should be handled by this
    /// consumer), false if it should be ignored.
    virtual bool parse_event(EventId event, uint32_t *address, bool *value) = 0;

    /// Gets the current state of a bit.
    /// @param address 0-based binary address.
    /// @return true if set (normal/active), false if clear (reverse/inactive).
    /// If state is unknown, returns false (check is_state_known first).
    bool get_state(uint32_t address) const
    {
        uint32_t word_index = address / 32;
        if (word_index >= stateWordCount_)
        {
            return false;
        }
        return (lastSetState_[word_index] & (1UL << (address & 31))) != 0;
    }

    /// Checks if the state of a bit is known.
    /// @param address 0-based binary address.
    /// @return true if state is known.
    bool is_state_known(uint32_t address) const
    {
        uint32_t word_index = address / 32;
        if (word_index >= stateWordCount_)
            return false;
        return (isStateKnown_[word_index] & (1UL << (address & 31))) != 0;
    }

    /// OpenLCB node to export the consumer on.
    Node *node_;
    /// Configuration for this consumer.
    const EventRangeConfig *cfg_;

    /// Number of words in state arrays.
    uint32_t stateWordCount_;
    /// Stored state bits. Array of stateWordCount_ entries, owned by us.
    uint32_t *lastSetState_;
    /// Known state bits. Array of stateWordCount_ entries, owned by us.
    uint32_t *isStateKnown_;
};

} // namespace openlcb

#endif // _OPENLCB_WELLKNOWNEVENTRANGECONSUMER_HXX_
