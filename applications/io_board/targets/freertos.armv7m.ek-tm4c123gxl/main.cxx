/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file main.cxx
 *
 * Main file for the io board application on the Tiva Launchpad board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

#include "os/os.h"
#include "can_frame.h"
#include "nmranet_config.h"

#include "os/TempFile.hxx"
#include "nmranet/SimpleStack.hxx"
#include "nmranet/SimpleNodeInfoMockUserFile.hxx"
#include "nmranet/EventHandlerTemplates.hxx"
#include "nmranet/ConfigRepresentation.hxx"
#include "utils/ConfigUpdateListener.hxx"

#include "freertos_drivers/ti/TivaGPIO.hxx"

extern const nmranet::NodeID NODE_ID;

OVERRIDE_CONST(gc_generate_newlines, 1);
OVERRIDE_CONST(main_thread_stack_size, 2500);
OVERRIDE_CONST(num_memory_spaces, 4);

nmranet::SimpleCanStack stack(NODE_ID);

nmranet::MockSNIPUserFile snip_user_file(
    "Default user name", "Default user description");
const char *const nmranet::SNIP_DYNAMIC_FILENAME =
    nmranet::MockSNIPUserFile::snip_user_file_path;

namespace nmranet
{
BEGIN_GROUP(ConsumerConfig, base);
EXTEND_GROUP(ConsumerConfig, base, event_on, EventConfigEntry);
EXTEND_GROUP(ConsumerConfig, event_on, event_off, EventConfigEntry);
END_GROUP(ConsumerConfig, event_off);

using AllConsumers = RepeatedGroup<ConsumerConfig, 3>;

BEGIN_GROUP(ProducerConfig, base);
EXTEND_GROUP(ProducerConfig, base, debounce, Uint8ConfigEntry);
EXTEND_GROUP(ProducerConfig, debounce, event_on, EventConfigEntry);
EXTEND_GROUP(ProducerConfig, event_on, event_off, EventConfigEntry);
END_GROUP(ProducerConfig, event_off);

using AllProducers = RepeatedGroup<ProducerConfig, 2>;

BEGIN_GROUP(ConfigDef, base);
EXTEND_GROUP(ConfigDef, base, snip_data, EmptyGroup<128>);
EXTEND_GROUP(ConfigDef, snip_data, consumers, AllConsumers);
EXTEND_GROUP(ConfigDef, consumers, producers, AllProducers);
END_GROUP(ConfigDef, consumers);

static_assert(ConfigDef::size() <= 256, "Need to adjust eeprom size");

typedef bool (*getter_fn_t)();
typedef void (*setter_fn_t)(bool);

class ConfiguredConsumer : public ConfigUpdateListener
{
public:
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
            return REINIT_NEEDED;
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

} // namespace nmranet

GPIO_PIN(LED_RED, LedPin, F, 1);
GPIO_PIN(LED_GREEN, LedPin, F, 3);
GPIO_PIN(LED_BLUE, LedPin, F, 2);

GPIO_PIN(SW1, GpioInputPU, F, 4);

nmranet::ConfigDef cfg(0);

nmranet::ConfiguredConsumer consumer_red(
    cfg.consumers().entry<0>(), LED_RED_Pin());
nmranet::ConfiguredConsumer consumer_green(
    cfg.consumers().entry<1>(), LED_GREEN_Pin());
nmranet::ConfiguredConsumer consumer_blue(
    cfg.consumers().entry<2>(), LED_BLUE_Pin());

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif

// Enable this to add sniffing through the usb or serial port.
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    stack.loop_executor();
    return 0;
}
