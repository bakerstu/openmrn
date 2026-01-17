/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file DccAccyConsumer.hxx
 *
 * Consumer class that exports 2044 consecutive bits for DCC accessory control
 * packets.
 *
 * @author Balazs Racz
 * @date 3 Feb 2017
 */

#ifndef _OPENLCB_DCCACCYCONSUMER_HXX_
#define _OPENLCB_DCCACCYCONSUMER_HXX_

#include "dcc/TrackIf.hxx"
#include "openlcb/EventHandlerTemplates.hxx"
#include "openlcb/TractionDefs.hxx"
#include "openlcb/WellKnownEventRangeConsumer.hxx"

namespace openlcb
{

/// Specialized (DCC protocol) implementation of a DCC accessory consumer.
class DccAccyConsumer : public WellKnownEventRangeConsumer
{
public:
    static const EventRangeConfig* get_config()
    {
        static constexpr EventRangeConfig cfg = {
            TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
            TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
            12, // 4096 events (12 bits)
            2048 // 2048 state bits (4096 events map to 2048 outputs)
        };
        return &cfg;
    }

    /// Constructs a listener for DCC accessory control.
    /// @param node is the virtual node that will be listening for events and
    /// responding to Identify messages.
    /// @param track is the interface through which we will be writing DCC
    /// accessory packets.
    DccAccyConsumer(Node *node, dcc::TrackIf *track)
        : WellKnownEventRangeConsumer(node, get_config())
        , track_(track)
    {
    }

    /// Destructor.
    ~DccAccyConsumer()
    {
    }

protected:
    /// Parses an event into an openlcb accessory offset.
    bool parse_event(EventId event, uint32_t *index, bool *value) override
    {
        uint32_t dccAddress;
        if (event >= cfg_->activate_base &&
            event < cfg_->activate_base + (1UL << cfg_->mask_bits))
        {
            onOff_ = 1;
            dccAddress = event - cfg_->activate_base;
        }
        else if (event >= cfg_->inactivate_base &&
            event < cfg_->inactivate_base + (1UL << cfg_->mask_bits))
        {
            onOff_ = 0;
            dccAddress = event - cfg_->inactivate_base;
        }
        else
        {
            return false;
        }

        // Populate DccAccyConsumer specific members
        dccAddress_ = dccAddress;

        // Populate base class generic properties
        // Normal/Reverse is determined by LSB of dccAddress. 1 = Normal, 0 = Reverse.
        *value = (dccAddress & 1) != 0;

        // Index is dccAddress / 2.
        *index = dccAddress >> 1;

        return true;
    }

    /// Send the actual accessory command.
    void action_impl() override
    {
        dcc::TrackIf::message_type *pkt;
        mainBufferPool->alloc(&pkt);
        pkt->data()->add_dcc_basic_accessory(dccAddress_, onOff_);
        pkt->data()->packet_header.rept_count = 3;
        track_->send(pkt);
    }

private:
    /// Track to send DCC packets to.
    dcc::TrackIf *track_;

    /// Parsed event state: 1 = activate (C=1), 0 = deactivate (C=0).
    unsigned onOff_ : 1;
    /// Parsed event state: dcc address (0..4095) without inverting or encoding.
    unsigned dccAddress_ : 12;
};

/// Base (generic protocol) implementation of the DCC extended accessory
/// consumer. Unlike the basic accessory version, this one does not remember
/// the last set state.
class DccExtAccyConsumerBase : public SimpleEventHandler
{
protected:
    /// How may addresses are there for extended accessories.
    static constexpr unsigned NUM_ADDRESS = 2048;
    /// How may aspects are supported per accessory.
    static constexpr unsigned NUM_ASPECT = 256;
    /// Total number of events we are listening for.
    static constexpr unsigned NUM_EVENT = NUM_ASPECT * NUM_ADDRESS;
    
    /// Constructs a listener for DCC extended accessory control.
    /// @param node is the virtual node that will be listening for events and
    /// responding to Identify messages.
    DccExtAccyConsumerBase(Node *node)
        : node_(node)
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(
                this, TractionDefs::EXT_DCC_ACCESSORY_EVENT_BASE),
            11+8 /*number of bits*/);
    }

    /// Destructor.
    ~DccExtAccyConsumerBase()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
            eventid_to_buffer(EncodeRange(
                TractionDefs::EXT_DCC_ACCESSORY_EVENT_BASE, NUM_EVENT - 1)),
            done->new_child());
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (!parse_event(event->event))
        {
            return;
        }
        send_accy_command();
    }

    void handle_identify_consumer(const EventRegistryEntry &entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (!parse_event(event->event))
        {
            return;
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN, WriteHelper::global(),
            eventid_to_buffer(event->event), done->new_child());
    }

    /// Send the actual accessory command.
    virtual void send_accy_command() = 0;

    /// Parses an event into an openlcb accessory offset.
    /// @return true if the event is in the accessory range, false if this
    /// event can be ignored.
    /// @param on_off will be set to true if this is an activate event, false
    /// if it is an inactivate event.
    /// @param ofs will be set to the offset in the state_ arrays.
    /// @param mask will be set to a single bit value that marks the location
    /// in the state_ arrays.
    bool parse_event(EventId event)
    {
        if (event >= TractionDefs::EXT_DCC_ACCESSORY_EVENT_BASE &&
            event <
                TractionDefs::EXT_DCC_ACCESSORY_EVENT_BASE + NUM_EVENT)
        {
            aspect_ = event & 0xff;
            dccAddress_ = (event >> 8) & (NUM_ADDRESS - 1);
            return true;
        }
        else
        {
            return false;
        }
    }


    /// Parsed event state: dcc address (0..2047) without inverting or encoding.
    unsigned dccAddress_ : 11;
    /// Parsed event state: the aspect commanded.
    unsigned aspect_ : 8;

    /// OpenLCB node to export the consumer on.
    Node *node_;
};

/// Specialized (DCC protocol) implementation of a DCC extended accessory
/// consumer.
class DccExtAccyConsumer : public DccExtAccyConsumerBase
{
public:
    /// Constructs a listener for DCC accessory control.
    /// @param node is the virtual node that will be listening for events and
    /// responding to Identify messages.
    /// @param track is the interface through which we will be writing DCC
    /// accessory packets.
    DccExtAccyConsumer(Node *node, dcc::TrackIf *track)
        : DccExtAccyConsumerBase(node)
        , track_(track)
    {
    }

    /// Destructor.
    ~DccExtAccyConsumer()
    {
    }

private:
    /// Send the actual accessory command.
    void send_accy_command() override
    {
        dcc::TrackIf::message_type *pkt;
        mainBufferPool->alloc(&pkt);
        pkt->data()->add_dcc_ext_accessory(dccAddress_, aspect_);
        pkt->data()->packet_header.rept_count = 3;
        track_->send(pkt);
    }

    /// Track to send DCC packets to.
    dcc::TrackIf *track_;
};


} // namespace openlcb

#endif  // _OPENLCB_DCCACCYCONSUMER_HXX_
