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

namespace openlcb
{

/// Base (generic protocol) implementation of a DCC accessory consumer.
class DccAccyConsumerBase : public SimpleEventHandler
{
protected:
    /// Constructs a listener for DCC accessory control.
    /// @param node is the virtual node that will be listening for events and
    /// responding to Identify messages.
    DccAccyConsumerBase(Node *node)
        : node_(node)
    {
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(
                this, TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE),
            12);
        EventRegistry::instance()->register_handler(
            EventRegistryEntry(
                this, TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE),
            12);
        memset(lastSetState_, 0, sizeof(lastSetState_));
        memset(isStateKnown_, 0, sizeof(isStateKnown_));
    }

    /// Destructor.
    ~DccAccyConsumerBase()
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
        if (registry_entry.event ==
            TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE)
        {
            event->event_write_helper<1>()->WriteAsync(node_,
                Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
                eventid_to_buffer(EncodeRange(
                    TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
                    4095)),
                done->new_child());
        }
        if (registry_entry.event ==
            TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE)
        {
            event->event_write_helper<2>()->WriteAsync(node_,
                Defs::MTI_CONSUMER_IDENTIFIED_RANGE, WriteHelper::global(),
                eventid_to_buffer(EncodeRange(
                    TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE,
                    4095)),
                done->new_child());
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (!parse_event(event->event))
        {
            return;
        }
        uint32_t m = (1U) << eventMask_;
        isStateKnown_[eventOfs_] |= m;
        if (normalReverse_)
        {
            lastSetState_[eventOfs_] |= m;
        }
        else
        {
            lastSetState_[eventOfs_] &= ~m;
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
        uint32_t m = (1U) << eventMask_;
        EventState s;
        if (isStateKnown_[eventOfs_] & m)
        {
            if (lastSetState_[eventOfs_] & m)
            {
                // normal
                s = EventState::VALID;
            }
            else
            {
                // reversed
                s = EventState::INVALID;
            }
            if (normalReverse_ != 1)
            {
                // query was reversed. invert the event state
                s = invert_event_state(s);
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
        if (event >= TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
            event <
                TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
        {
            onOff_ = 1;
            dccAddress_ =
                event - TractionDefs::ACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
        }
        else if (event >=
                TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE &&
            event <
                TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE + 4096)
        {
            onOff_ = 0;
            dccAddress_ =
                event - TractionDefs::INACTIVATE_BASIC_DCC_ACCESSORY_EVENT_BASE;
        }
        else
        {
            return false;
        }
        normalReverse_ = dccAddress_ & 1;
        eventOfs_ = dccAddress_ >> 6;
        eventMask_ = (dccAddress_ >> 1) & 31;
        return true;
    }

    /// Parsed event state: 1 = activate (C=1), 2 = deactivate (C=0).
    unsigned onOff_ : 1;
    /// Parsed event state: 1 = normal (D0=1), 0 = reversed (D0=0).
    unsigned normalReverse_ : 1;
    /// Parsed event state: dcc address (0..4095) without inverting or encoding.
    unsigned dccAddress_ : 12;
    /// Parsed event state: offset in the state_ array.
    unsigned eventOfs_ : 6;
    /// Parsed event state: bit index (0..31) in the uint32 in the state_ array
    /// entry pointed to by eventOfs_.
    unsigned eventMask_ : 5;

    /// each bit determines what the last command sent to the accessory address
    /// was. bit==0 is reverse. bit==1 is normal.
    uint32_t lastSetState_[64];
    /// each bit determines whether we've sent a command to that accessory
    /// address yet or not. bit==0 means unknown state, bit==1 means state is
    /// known.
    uint32_t isStateKnown_[64];

    /// OpenLCB node to export the consumer on.
    Node *node_;
};

/// Specialized (DCC protocol) implementation of a DCC accessory consumer.
class DccAccyConsumer : public DccAccyConsumerBase
{
public:
    /// Constructs a listener for DCC accessory control.
    /// @param node is the virtual node that will be listening for events and
    /// responding to Identify messages.
    /// @param track is the interface through which we will be writing DCC
    /// accessory packets.
    DccAccyConsumer(Node *node, dcc::TrackIf *track)
        : DccAccyConsumerBase(node)
        , track_(track)
    {
    }

    /// Destructor.
    ~DccAccyConsumer()
    {
    }

private:
    /// Send the actual accessory command.
    void send_accy_command() override
    {
        dcc::TrackIf::message_type *pkt;
        mainBufferPool->alloc(&pkt);
        pkt->data()->add_dcc_basic_accessory(dccAddress_, onOff_);
        pkt->data()->packet_header.rept_count = 3;
        track_->send(pkt);
    }

    /// Track to send DCC packets to.
    dcc::TrackIf *track_;
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
