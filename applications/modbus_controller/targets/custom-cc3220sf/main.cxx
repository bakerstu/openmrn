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

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

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
#include "freertos_drivers/common/WifiDefs.hxx"
#include "console/FileCommands.hxx"

#include "utils/TcpLogging.hxx"
#include "utils/SocketClient.hxx"
#include "openlcb/TcpDefs.hxx"
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
extern const char *const openlcb::CONFIG_FILENAME = "/usr/modbuseeprom";
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


class ModbusReceiveLog : public StateFlowBase {
public:
    ModbusReceiveLog(int fd) : StateFlowBase(stack.service()), fd_(fd) {
        start_flow(STATE(wait_for_data));
    }

private:
    Action wait_for_data() {
        ofs_ = 0;
        return read_single(
            &helper_, fd_, buf_, BUFLEN - 1, STATE(complete_line));
    }

    Action complete_line() {
        ofs_ += BUFLEN - helper_.remaining_;
        return read_repeated_with_timeout(&helper_, MSEC_TO_NSEC(50), fd_,
            buf_ + ofs_, BUFLEN - ofs_ - 1, STATE(timedout));
    }

    Action timedout() {
        ofs_ = BUFLEN - helper_.remaining_;
        buf_[ofs_] = 0;
        LOG(WARNING, "modbus input: %s", buf_);
        return call_immediately(STATE(wait_for_data));
    }

    StateFlowTimedSelectHelper helper_{this};

    static constexpr unsigned BUFLEN = 30;
    char buf_[BUFLEN];
    unsigned ofs_;
    /// serial port file des.
    int fd_;
};

class ModbusDriver : public StateFlow<Buffer<string>, QList<1>> {
public:
    ModbusDriver() : StateFlow<Buffer<string>, QList<1>>(stack.service()) {
        fd_ = ::open("/dev/ser0", O_RDWR);
        HASSERT(fd_ >= 0);
        ::fcntl(fd_, F_SETFL, O_NONBLOCK);
        new ModbusReceiveLog(fd_);
    }

    Action entry() {
        LOG(INFO, "sending packet: %s", message()->data()->c_str());
        return write_repeated(&helper_, fd_, message()->data()->data(),
            message()->data()->size(), STATE(done_state));
    }

    Action done_state() {
        return release_and_exit();
    }
    
private:
    StateFlowSelectHelper helper_{this};
    
    /// Serial device FD.
    int fd_;
} g_modbus_driver;

static const char HEX_DIGITS[] = "0123456789ABCDEF";

static string byte_to_hex(uint8_t data) {
    string ret(2, ' ');
    ret[0] = HEX_DIGITS[data >> 4];
    ret[1] = HEX_DIGITS[data & 0xf];
    return ret;
}

static string uint16_to_hex(uint16_t data) {
    string ret(4, ' ');
    ret[0] = HEX_DIGITS[(data >> 12) & 0xf];
    ret[1] = HEX_DIGITS[(data >> 8) & 0xf];
    ret[2] = HEX_DIGITS[(data >> 4) & 0xf];
    ret[3] = HEX_DIGITS[data & 0xf];
    return ret;
}

static uint8_t get_nibble(char b) {
    if ('0' <= b && b <= '9') {
        return b - '0';
    }
    if ('a' <= b && b <= 'f') {
        return b - 'a' + 10;
    }
    if ('A' <= b && b <= 'F') {
        return b - 'A' + 10;
    }
    return 0xff;
}

static uint8_t hex_to_byte(const char* data) {
    return (get_nibble(data[0]) << 4) | get_nibble(data[1]);
}

class ModbusSceneConsumer : private DefaultConfigUpdateListener,
                         private openlcb::SimpleEventHandler
{
public:
    ModbusSceneConsumer(const modbus::AllSceneConfig &cfg, openlcb::Node *node,
                        FlowInterface<Buffer<string>> *modbus_driver)
        : cfg_(cfg)
        , node_(node)
        , driver_(modbus_driver)
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
            for (unsigned c = 0; c < e.packets().num_repeats(); ++c)
            {
                const auto &ch = e.packets().entry(c);
                auto modbus_channel = ch.channel().read(fd);
                auto payload = ch.packet().read(fd);
                if (modbus_channel > 0 || !payload.empty())
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
            for (unsigned c = 0; c < e.packets().num_repeats(); ++c)
            {
                const auto &ch = e.packets().entry(c);
                string packet = ch.packet().read(fd);
                auto modbus_channel = ch.channel().read(fd);
                if (packet.empty() && modbus_channel > 0) {
                    packet.push_back(':');
                    packet += byte_to_hex(modbus_channel);
                    packet += byte_to_hex(ch.function().read(fd));
                    packet += uint16_to_hex(ch.address().read(fd));
                    packet += uint16_to_hex(ch.value().read(fd));
                    packet += '!';
                }
                if (packet.back() == '!') {
                    packet.pop_back();
                    uint8_t sum = 0;
                    for (unsigned p = 1; p < packet.size() - 1; p += 2) {
                        sum += hex_to_byte(packet.data() + p);
                    }
                    sum = -sum;
                    packet += byte_to_hex(sum);
                }
                if (packet.empty()) continue;
                packet.push_back(0x0d);
                packet.push_back(0x0a);
                parameterData_[cnt].packet = std::move(packet);
                ++cnt;
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
            for (unsigned c = 0; c < e.packets().num_repeats(); ++c)
            {
                const auto &ch = e.packets().entry(c);
                ch.packet().write(fd, "");
                CDI_FACTORY_RESET(ch.channel);
                CDI_FACTORY_RESET(ch.function);
                CDI_FACTORY_RESET(ch.address);
                CDI_FACTORY_RESET(ch.value);
            }
        }
    }

    void handle_event_report(const EventRegistryEntry &registry_entry,
        EventReport *event, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        LOG(WARNING, "seen event report");
        // Applies the parameter values we have stored.
        for (unsigned next = registry_entry.user_arg;
             next < parameterData_.size(); ++next)
        {
            const auto &d = parameterData_[next];
            auto* b = driver_->alloc();
            b->data()->assign(d.packet);
            driver_->send(b);
            if (d.end)
            {
                break;
            }
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
        string packet;
        uint8_t end;
    };

    /// Stores all loaded parameters.
    std::vector<ParameterConfig> parameterData_;

    /// Configuration file offsets.
    modbus::AllSceneConfig cfg_;
    /// Configuration file FD.
    int fd_{-1};
    /// OpenLCB node to use as consumer.
    openlcb::Node *node_;
    /// Flow where to forward the packets to send.
    FlowInterface<Buffer<string>> *driver_;
} g_scene_consumer{cfg.seg().scenes(), stack.node(), &g_modbus_driver};

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
        cfg.userinfo().name().write(fd, "MODBUS controller board");
        cfg.userinfo().description().write(fd, "");
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override
    {
        AutoNotify an(done);
        return UPDATED;
    }
} g_default_name;

/** Callback that is called when the connection completes successfully.
 * @param fd socket descriptor
 * @param addr information about the other end of the connection
 * @param on_exit who to wakeup when the socket is closed
 */
void client_connect_callback(int fd, struct addrinfo *addr,
                             Notifiable *on_exit)
{
    /// @todo should we set the socket to non-blocking once select is supported?
    //::fcntl(fd_, F_SETFL, O_NONBLOCK);

    //stack.add_can_port_select(fd, on_error);
    create_gc_port_for_can_hub(stack.can_hub(), fd, on_exit);
    // Re-acquires all aliases and send out initialization complete.
    stack.restart_stack();
    resetblink(0);
}

SocketClient* g_socket_client = nullptr;

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().legacy().internal_config(), openlcb::CANONICAL_VERSION, false);

    TcpLoggingServer log_server(stack.service(), 12345);
    stack.start_tcp_hub_server(12021);
    
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
    g_socket_client = new SocketClient(stack.service(),
        openlcb::TcpDefs::MDNS_SERVICE_NAME_GRIDCONNECT_CAN_TCP, WIFI_HUB_HOSTNAME,
        WIFI_HUB_PORT, client_connect_callback);

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
