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
 * \file ConfiguredConsumer.hxx
 *
 * Consumer class that uses CDI configuration and a GPIO template structure to
 * export a single bit as two event consumers to OpenLCB.
 *
 * @author Balazs Racz
 * @date 13 June 2015
 */

#ifndef _NMRANET_CONFIGUREDCONSUMER_HXX_
#define _NMRANET_CONFIGUREDCONSUMER_HXX_

#include "nmranet/SimpleStack.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"

// TODO(balazs.racz) this is not nice but we shouldn't store a node
// pointer for every consumer separately.
extern nmranet::SimpleCanStack stack;

namespace nmranet {

BEGIN_GROUP(ConsumerConfig, base);
EXTEND_GROUP(ConsumerConfig, base, event_on, EventConfigEntry);
EXTEND_GROUP(ConsumerConfig, event_on, event_off, EventConfigEntry);
END_GROUP(ConsumerConfig, event_off);

class ConfiguredConsumer : public ConfigUpdateListener
{
public:
    typedef bool (*getter_fn_t)();
    typedef void (*setter_fn_t)(bool);

    class Impl : public BitEventInterface
    {
    public:
        Impl(EventId event_on, EventId event_off, getter_fn_t getter,
            setter_fn_t setter)
            : BitEventInterface(event_on, event_off)
            , getter_(getter)
            , setter_(setter)
        {
        }

        bool GetCurrentState() OVERRIDE
        {
            return getter_();
        }
        void SetState(bool new_value) OVERRIDE
        {
            setter_(new_value);
        }
        Node *node() OVERRIDE
        {
            return stack.node();
        }

    public:
        const getter_fn_t getter_;
        const setter_fn_t setter_;
    };

    template <class HW>
    ConfiguredConsumer(const ConsumerConfig &cfg, const HW &)
        : impl_(0, 0, &HW::get, &HW::set)
        , consumer_(&impl_)
        , cfg_(cfg)
    {
        ConfigUpdateService::instance()->register_update_listener(this);
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done)
    {
        AutoNotify n(done);
        EventId cfg_event_on = cfg_.event_on().read(fd);
        EventId cfg_event_off = cfg_.event_off().read(fd);
        if (cfg_event_off != impl_.event_off() ||
            cfg_event_on != impl_.event_on())
        {
            auto saved_setter = impl_.setter_;
            auto saved_getter = impl_.getter_;
            // Need to reinitialize the consumer. We do this with in-place
            // destruction and construction.
            consumer_.~BitEventConsumer();
            impl_.~Impl();
            new (&impl_)
                Impl(cfg_event_on, cfg_event_off, saved_getter, saved_setter);
            new (&consumer_) BitEventConsumer(&impl_);
            return REINIT_NEEDED; // Causes events identify.
        }
        return UPDATED;
    }

    ///@TODO(balazs.racz): implement
    void factory_reset(int fd) OVERRIDE
    {
    }

private:
    Impl impl_;
    BitEventConsumer consumer_;
    const ConsumerConfig cfg_;
};

}  // namespace nmranet

#endif // _NMRANET_CONFIGUREDCONSUMER_HXX_

