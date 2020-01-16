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
 * Main file for the io board application on the CC3220SF Launchpad board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#define LOGLEVEL INFO

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "openlcb/EventHandler.hxx"
#include "openlcb/EventHandlerTemplates.hxx"

#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "config.hxx"
#include "hardware.hxx"

#include "CC32xxWiFi.hxx"
#include "utils/stdio_logging.h"
#include "freertos/tc_ioctl.h"

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
//#define SNIFF_ON_SERIAL
#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011808ULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);


// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char *const openlcb::CONFIG_FILENAME = "/usr/dmxeeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 3000, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

class EepromTimerFlow : public StateFlowBase, protected Atomic
{
public:
    EepromTimerFlow()
        : StateFlowBase(stack.service())
        , isWaiting_(0)
        , isDirty_(0)
    {
        start_flow(STATE(test_and_sleep));
    }

    void wakeup() {
        AtomicHolder h(this);
        isDirty_ = 1;
        if (isWaiting_) {
            isWaiting_ = 0;
            notify();
        }
    }

private:
    Action test_and_sleep()
    {
        AtomicHolder h(this);
        if (isDirty_)
        {
            isDirty_ = 0;
            return sleep_and_call(
                &timer_, MSEC_TO_NSEC(1000), STATE(test_and_flush));
        }
        isWaiting_ = 1;
        return wait();
    }

    Action test_and_flush()
    {
        {
            AtomicHolder h(this);
            if (isDirty_)
            {
                // we received another write during the sleep. Go back to sleep.
                isDirty_ = 0;
                return sleep_and_call(
                    &timer_, MSEC_TO_NSEC(1000), STATE(test_and_flush));
            }
        }
        extern void eeprom_flush();
        eeprom_flush();
        return call_immediately(STATE(test_and_sleep));
    }

    StateFlowTimer timer_{this};
    // 1 if the flow is sleeping and needs to be notified to wake up.
    unsigned isWaiting_ : 1;
    // 1 if we received a notification from the eeprom handler.
    unsigned isDirty_ : 1;
} eepromTimerFlow_;

extern "C" {
void eeprom_updated_notification() {
    eepromTimerFlow_.wakeup();
}
}

class DmxDriver : public StateFlowBase {
public:
    DmxDriver() : StateFlowBase(stack.service()) {
        memset(channelData_, 0, sizeof(channelData_));
        fd_ = ::open("/dev/ser0", O_RDWR);
        HASSERT(fd_ >= 0);
        RS485_TX_EN_Pin::set(true);
        start_flow(STATE(wait_state));
    }

    void set_channel_value(unsigned ch, uint8_t value) {
        HASSERT(ch < NUM_CHANNEL);
        channelData_[ch + HEADER_PAYLOAD] = value;
    }

    uint8_t* get_channels() {
        return channelData_ + HEADER_PAYLOAD;
    }
    
    Action wait_state() {
        return sleep_and_call(&timer_, MSEC_TO_NSEC(5), STATE(send_data));
    }

    Action send_data() {
        // send break
        ::ioctl(fd_, TCSBRK, 0);
        return write_repeated(&helper_, fd_, channelData_, sizeof(channelData_),
            STATE(wait_state));
    }
    
    static constexpr unsigned NUM_CHANNEL = 32;

private:
    static constexpr unsigned HEADER_PAYLOAD = 1;

    StateFlowTimer timer_{this};
    StateFlowSelectHelper helper_{this};
    
    /// Serial device FD.
    int fd_;

    /// Raw data of what to send to the bus.
    uint8_t channelData_[NUM_CHANNEL + HEADER_PAYLOAD];
} g_dmx_driver;

class DmxSceneConsumer : private DefaultConfigUpdateListener,
                         private openlcb::SimpleEventHandler
{
public:
    DmxSceneConsumer(const dmx::AllSceneConfig &cfg, openlcb::Node *node,
        uint8_t *dmx_data, unsigned dmx_size)
        : cfg_(cfg)
        , node_(node)
        , dmxData_(dmx_data)
        , dmxSize_(dmx_size)
    {
    }

private:
    /// Called when the config file is loaded or changed.
    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        fd_ = fd;
        if (!parameterData_.empty())
        {
            openlcb::EventRegistry::instance()->unregister_handler(this);
        }
        unsigned cnt = 0;
        // first count all channels.
        for (unsigned i = 0; i < cfg_.num_repeats(); ++i)
        {
            const auto &e = cfg_.entry(i);
            for (unsigned c = 0; c < e.channels().num_repeats(); ++c)
            {
                const auto &ch = e.channels().entry(c);
                auto dmx_channel = ch.channel().read(fd);
                if (dmx_channel > 0 && dmx_channel <= 512)
                {
                    ++cnt;
                }
            }
        }
        // allocates memory
        parameterData_.resize(cnt);
        cnt = 0;
        // caches configuration in that memory and registers event handlers
        for (unsigned i = 0; i < cfg_.num_repeats(); ++i)
        {
            const auto &e = cfg_.entry(i);
            unsigned sequence_start = cnt;
            for (unsigned c = 0; c < e.channels().num_repeats(); ++c)
            {
                const auto &ch = e.channels().entry(c);
                auto dmx_channel = ch.channel().read(fd);
                if (dmx_channel > 0 && dmx_channel <= 512)
                {
                    auto &param = parameterData_[cnt];
                    param.delay = ch.delay().read(fd);
                    param.channel = dmx_channel;
                    param.value = ch.value().read(fd);
                    param.end = 0;
                    ++cnt;
                }
            }
            openlcb::EventId event = e.event().read(fd);
            if (cnt > sequence_start && event > 0)
            {
                // there was at least one channel set and the event id seems
                // valid

                // set sentinel
                parameterData_[cnt - 1].end = 1;
                // register event handler
                openlcb::EventRegistry::instance()->register_handler(
                    EventRegistryEntry(this, event, sequence_start), 0);
            }
        }

        if (parameterData_.empty())
        {
            return UPDATED;
        }
        else
        {
            return REINIT_NEEDED;
        }
    }

    void factory_reset(int fd) override
    {
        fd_ = fd;
        for (unsigned i = 0; i < cfg_.num_repeats(); ++i)
        {
            const auto &e = cfg_.entry(i);
            // events are factory reset automatically.
            e.name().write(fd, "");
            for (unsigned c = 0; c < e.channels().num_repeats(); ++c)
            {
                const auto &ch = e.channels().entry(c);
                CDI_FACTORY_RESET(ch.delay);
                CDI_FACTORY_RESET(ch.channel);
                CDI_FACTORY_RESET(ch.value);
            }
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        // Applies the parameter values we have stored.
        for (unsigned next = registry_entry.user_arg;
             next < parameterData_.size(); ++next)
        {
            const auto &d = parameterData_[next];
            if (d.channel < dmxSize_)
            {
                dmxData_[d.channel - 1] = d.value;
            }
            if (d.end)
                break;
        }
    }

    void handle_identify_global(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        if (event->dst_node && event->dst_node != node_)
        {
            return;
        }
        event->event_write_helper<1>()->WriteAsync(node_,
            openlcb::Defs::MTI_CONSUMER_IDENTIFIED_UNKNOWN,
            openlcb::WriteHelper::global(),
            openlcb::eventid_to_buffer(registry_entry.event),
            done->new_child());
    }

    void handle_identify_consumer(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) OVERRIDE
    {
        return handle_identify_global(registry_entry, event, done);
    }

    struct ParameterConfig
    {
        /// Delay in msec from the trigger.
        uint16_t delay;
        /// DMX channel number, 1..512 (0 not allowed).
        uint16_t channel : 10;
        /// Is this the end of the trigger sequence.
        uint16_t end : 1;
        /// Value to set on DMX channel.
        uint8_t value;
    };

    /// Stores all loaded parameters.
    std::vector<ParameterConfig> parameterData_;

    /// Configuration file offsets.
    dmx::AllSceneConfig cfg_;
    /// Configuration file FD.
    int fd_{-1};
    /// OpenLCB node to use as consumer.
    openlcb::Node *node_;
    /// The output DMX buffer. Has channel 1 data at offset 0.
    uint8_t *dmxData_;
    /// The number of channels supported.
    unsigned dmxSize_;
} g_scene_consumer{cfg.seg().scenes(), stack.node(),
    g_dmx_driver.get_channels(), g_dmx_driver.NUM_CHANNEL};

// Defines the GPIO ports used for the producers and the consumers.

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const kOutputGpio[] = {LED_5_Pin::instance(),
                                             LED_2_Pin::instance(),
                                             LED_3_Pin::instance()};

// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The MultiConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the hardware pin definition and the configuration from the CDI
// definition. The virtual node pointer comes from the stack object. The
// configuration structure comes from the CDI definition object, segment 'seg',
// in which there is a repeated group 'consumers'. The GPIO pins get assigned
// to the repetitions in the group in order.
openlcb::MultiConfiguredConsumer consumers(
    stack.node(), kOutputGpio, ARRAYSIZE(kOutputGpio), cfg.seg().legacy().consumers());

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().legacy().producers().entry<0>(), SW1_Pin());

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(
    stack.node(), {producer_sw1.polling()});


class DefaultName : private DefaultConfigUpdateListener {
private:
    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, "DMX controller board");
        cfg.userinfo().description().write(fd, "");
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        return UPDATED;
    }
} g_default_name;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().legacy().internal_config(), openlcb::CANONICAL_VERSION, false);

    // The necessary physical ports must be added to the stack.
    //
    // It is okay to enable multiple physical ports, in which case the stack
    // will behave as a bridge between them. For example enabling both the
    // physical CAN port and the USB port will make this firmware act as an
    // USB-CAN adapter in addition to the producers/consumers created above.
    //
    // If a port is enabled, it must be functional or else the stack will
    // freeze waiting for that port to send the packets out.
#if defined(HAVE_PHYSICAL_CAN_PORT)
    stack.add_can_port_select("/dev/can0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    extern CC32xxWiFi wifi;
    while (!wifi.wlan_ready())
    {
        wifi.connecting_update_blinker();
    }

    
    resetblink(WIFI_BLINK_CONNECTING);
    extern char WIFI_HUB_HOSTNAME[];
    extern int WIFI_HUB_PORT;
    stack.connect_tcp_gridconnect_hub(WIFI_HUB_HOSTNAME, WIFI_HUB_PORT);
    resetblink(0);

    openlcb::ReadWriteMemoryBlock dmx_space(g_dmx_driver.get_channels(), g_dmx_driver.NUM_CHANNEL);
    stack.memory_config_handler()->registry()->insert(stack.node(), 72, &dmx_space);
    
    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
