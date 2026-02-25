/** \copyright
 * Copyright (c) 2026, Rick Lull
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
 * \file ConfiguredExclusiveConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a group of GPIO pins to
 * implement mutually exclusive output selection. Receiving any one of the
 * configured events turns on the corresponding GPIO and turns off all other
 * GPIOs in the group.
 *
 * @author Rick Lull and Claude Code
 * @date 21 Feb 2026
 */

#ifndef _OPENLCB_CONFIGUREDEXCLUSIVECONSUMER_HXX_
#define _OPENLCB_CONFIGUREDEXCLUSIVECONSUMER_HXX_

#include "openlcb/ConfigRepresentation.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "utils/format_utils.hxx"

namespace openlcb
{

/// CDI Configuration for one output within an exclusive group. Each output
/// gets a single event (not a pair), because the other outputs in the group
/// are implicitly turned off.
CDI_GROUP(ExclusiveConsumerConfig);
/// Allows the user to assign a name for this output.
CDI_GROUP_ENTRY(description, StringConfigEntry<8>, //
    Name("Description"), Description("User name of this output."));
/// Specifies the event ID to select this output.
CDI_GROUP_ENTRY(event, EventConfigEntry, //
    Name("Event"),
    Description("Receiving this event ID will turn this output on and "
                "turn off all other outputs in the group."));
CDI_GROUP_END();

/// Consumer class for a group of GPIO outputs where exactly one output is
/// active at a time. When an event is received for any output in the group,
/// that output is turned on and all other outputs in the group are turned off.
///
/// This is useful for signal masts (where only one aspect is lit at a time),
/// route indicators, or any application where outputs are mutually exclusive.
///
/// Usage: ```
///
/// constexpr const Gpio *const kSignalGpio[] = {
///     RED_Pin::instance(), YELLOW_Pin::instance(), GREEN_Pin::instance(),
/// };
/// openlcb::ConfiguredExclusiveConsumer signal_consumer(stack.node(),
///    kSignalGpio, ARRAYSIZE(kSignalGpio),
///    cfg.seg().signal_consumers());
/// ```
class ConfiguredExclusiveConsumer : public ConfigUpdateListener,
                                    private SimpleEventHandler
{
public:
    typedef ExclusiveConsumerConfig config_entry_type;

    /// @param node is the OpenLCB node object from the stack.
    /// @param pins is the list of pins represented by the Gpio* object
    /// instances. Can be constant from FLASH space.
    /// @param size is the length of the list of pins array.
    /// @param config is the repeated group object from the configuration space
    /// that represents the locations of the events.
    template <unsigned N>
    __attribute__((noinline)) ConfiguredExclusiveConsumer(Node *node,
        const Gpio *const *pins, unsigned size,
        const RepeatedGroup<config_entry_type, N> &config)
        : node_(node)
        , pins_(pins)
        , size_(N)
        , activeIndex_(NONE_ACTIVE)
        , offset_(config)
    {
        // Mismatched sizing of the GPIO array from the configuration array.
        HASSERT(size == N);
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    ~ConfiguredExclusiveConsumer()
    {
        do_unregister();
        ConfigUpdateService::instance()->unregister_update_listener(this);
    }

    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE
    {
        AutoNotify n(done);

        if (!initial_load)
        {
            do_unregister();
        }
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            const config_entry_type cfg_ref(grp_ref.entry(i));
            EventId cfg_event = cfg_ref.event().read(fd);
            EventRegistry::instance()->register_handler(
                EventRegistryEntry(this, cfg_event, i), 0);
        }
        return REINIT_NEEDED; // Causes events identify.
    }

    void factory_reset(int fd) OVERRIDE
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            grp_ref.entry(i).description().write(fd, "");
        }
    }

    /// Factory reset helper function. Sets all names to something 1..N.
    /// @param fd passed on from factory reset argument.
    /// @param basename name of repeats.
    void factory_reset_names(int fd, const char *basename)
    {
        RepeatedGroup<config_entry_type, UINT_MAX> grp_ref(offset_.offset());
        for (unsigned i = 0; i < size_; ++i)
        {
            string v(basename);
            v.push_back(' ');
            char buf[10];
            unsigned_integer_to_buffer(i + 1, buf);
            v += buf;
            grp_ref.entry(i).description().write(fd, v);
        }
    }

    // Implementations for the event handler functions.

    void handle_identify_global(const EventRegistryEntry &registry_entry,
                              EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        if (event->dst_node && event->dst_node != node_)
        {
            return done->notify();
        }
        SendConsumerIdentified(registry_entry, event, done);
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
                                EventReport *event, BarrierNotifiable *done)
        OVERRIDE
    {
        if (event->event != registry_entry.event)
        {
            return done->notify();
        }
        SendConsumerIdentified(registry_entry, event, done);
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
                           EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        if (event->event != registry_entry.event)
        {
            return done->notify();
        }

        unsigned selected = registry_entry.user_arg;

        // Turn off all other outputs in the group.
        for (unsigned i = 0; i < size_; ++i)
        {
            if (i != selected)
            {
                pins_[i]->clr();
            }
        }
        // Turn on the selected output.
        pins_[selected]->set();
        activeIndex_ = selected;

        done->notify();
    }

private:
    /// Sentinel value meaning no output is currently active.
    static constexpr unsigned NONE_ACTIVE = UINT_MAX;

    /// Sends out a ConsumerIdentified message for the given registration
    /// entry.
    void SendConsumerIdentified(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done)
    {
        Defs::MTI mti = Defs::MTI_CONSUMER_IDENTIFIED_VALID;
        if (registry_entry.user_arg != activeIndex_)
        {
            mti++; // INVALID
        }
        event->event_write_helper<3>()->WriteAsync(node_, mti,
            WriteHelper::global(), eventid_to_buffer(registry_entry.event),
            done);
    }

    /// Removes registration of this event handler from the global event
    /// registry.
    void do_unregister()
    {
        EventRegistry::instance()->unregister_handler(this);
    }

    Node *node_;              //< virtual node to export the consumer on
    const Gpio *const *pins_; //< array of all GPIO pins to use
    size_t size_;             //< number of GPIO pins to export
    unsigned activeIndex_;    //< currently active output, or NONE_ACTIVE
    ConfigReference offset_;  //< Offset in the configuration space for our
    // configs.
};

} // namespace openlcb

#endif // _OPENLCB_CONFIGUREDEXCLUSIVECONSUMER_HXX_
