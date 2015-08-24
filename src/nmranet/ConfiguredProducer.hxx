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
 * \file ConfiguredProducer.hxx
 *
 * Producer class that uses CDI configuration and a GPIO template structure to
 * export a single bit as two event producers to OpenLCB.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _NMRANET_CONFIGUREDPRODUCER_HXX_
#define _NMRANET_CONFIGUREDPRODUCER_HXX_

#include "nmranet/PolledProducer.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"
#include "utils/ConfigUpdateService.hxx"
#include "utils/Debouncer.hxx"

namespace nmranet
{

CDI_GROUP(ProducerConfig);
CDI_GROUP_ENTRY(description, StringConfigEntry<15>, //
    Name("Description"),
    Description("User name of this input."));
CDI_GROUP_ENTRY(debounce, Uint8ConfigEntry, Name("Debounce parameter"), Description("Amount of time to wait for the input to stabilize before producing the event. Unit is 30 msec of time. Usually a value of 2-3 works well in a non-noisy environment. In high noise (train wheels for example) a setting between 8 -- 15 makes for a slower response time but a more stable signal.\nFormally, the parameter tells how many times of tries, each 30 msec apart, the input must have the same value in order for that value to be accepted and the event transition produced."));
CDI_GROUP_ENTRY(event_on, EventConfigEntry, //
    Name("Event On"),
    Description("This event will be produced when the input goes to HIGH."));
CDI_GROUP_ENTRY(event_off, EventConfigEntry, //
    Name("Event Off"),
    Description("This event will be produced when the input goes to LOW."));
CDI_GROUP_END();

extern "C" {
void ignore_fn();
}

class ConfiguredProducer : public ConfigUpdateListener
{
public:
    using Impl = GPIOBit;
    using ProducerClass = PolledProducer<QuiesceDebouncer, Impl>;

    template <class HW>
    ConfiguredProducer(Node *node, const ProducerConfig &cfg, const HW &)
        : producer_(QuiesceDebouncer::Options(3), node, 0, 0, HW::instance())
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        uint8_t debounce_arg = cfg_.debounce().read(fd);
        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        if (cfg_event_off != producer_.event_off() ||
            cfg_event_on != producer_.event_on())
        {
            auto saved_gpio = producer_.gpio_;
            auto saved_node = producer_.node();
            // Need to reinitialize the producer. We do this with in-place
            // destruction and construction.
            producer_.~ProducerClass();
            new (&producer_) ProducerClass(
                QuiesceDebouncer::Options(debounce_arg), saved_node,
                cfg_event_on, cfg_event_off, saved_gpio);
            return REINIT_NEEDED; // Causes events identify.
        }
        return UPDATED;
    }

    ///@TODO(balazs.racz): implement
    void factory_reset(int fd) OVERRIDE
    {
    }

    Polling *polling()
    {
        return &producer_;
    }

private:
    ProducerClass producer_;
    const ProducerConfig cfg_;
};

} // namespace nmranet

#endif // _NMRANET_CONFIGUREDPRODUCER_HXX_
