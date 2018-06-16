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

#define LOGLEVEL INFO

#include <algorithm>

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"
#include "utils/gc_format.h"

#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/ti/TivaCpuLoad.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/PersistentGPIO.hxx"
#include "freertos_drivers/common/BenchmarkCan.hxx"
#include "config.hxx"
#include "hardware.hxx"

// Writes all log data to stderr.
#include "utils/TcpLogging.hxx"

BenchmarkCan benchmark_can("/dev/fakecan0");

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
//#define SNIFF_ON_SERIAL
//#define SNIFF_ON_USB
//#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011804ULL;

// Sets up a comprehensive OpenLCB stack for a single virtual node. This stack
// contains everything needed for a usual peripheral node -- all
// CAN-bus-specific components, a virtual node, PIP, SNIP, Memory configuration
// protocol, ACDI, CDI, a bunch of memory spaces, etc.
openlcb::SimpleCanStack stack(NODE_ID);

SerialLoggingServer log_server(stack.service(), "/dev/ser0");

TivaCpuLoad<TivaCpuLoadDefHw> load_monitor;

#include "freertos_drivers/common/cpu_profile.hxx"

DEFINE_CPU_PROFILE_INTERRUPT_HANDLER(timer4a_interrupt_handler,
    MAP_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT));

class BenchmarkDriver : public StateFlowBase
{
public:
    BenchmarkDriver()
        : StateFlowBase(stack.service())
    {
        start_flow(STATE(log_and_wait));
    }

private:
    Action log_and_wait()
    {
        if (!SW1_Pin::get())
        {
            return call_immediately(STATE(do_benchmark));
        }
        return sleep_and_call(&timer_, MSEC_TO_NSEC(100), STATE(log_and_wait));
    }

    Action do_benchmark()
    {
        struct can_frame frame;
        LOG(INFO, "Starting benchmark.");
        auto ret = gc_format_parse("X195B4123N0501010118FF0123", &frame);
        HASSERT(0 == ret);
        startTime_ = OSTime::get_monotonic();
        benchmark_can.start_benchmark(&frame, COUNT);
        enable_profiling = 1;
        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(100), STATE(check_benchmark_state));
    }

    Action check_benchmark_state()
    {
        unsigned count = 1;
        auto end_time = benchmark_can.get_timestamp(&count);
        if (!count)
        {
            enable_profiling = 0;
            float spd = COUNT;
            float time = (end_time - startTime_);
            time /= 1e9;
            spd /= time;
            LOG(INFO, "Benchmark done. Time %d msec, speed %d pkt/sec",
                static_cast<int>((end_time - startTime_ + 500000) / 1000000),
                static_cast<int>(spd));
            return call_immediately(STATE(log_and_wait));
        }
        else
        {
            LOG(INFO, "benchmark: %u pkt left", count);
        }
        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(100), STATE(check_benchmark_state));
    }

    static constexpr unsigned COUNT = 10000;

    long long startTime_;
    StateFlowTimer timer_{this};
} benchmark_driver;

CpuLoadLog load_logger(stack.service());

// ConfigDef comes from config.hxx and is specific to the particular device and
// target. It defines the layout of the configuration memory space and is also
// used to generate the cdi.xml file. Here we instantiate the configuration
// layout. The argument of offset zero is ignored and will be removed later.
openlcb::ConfigDef cfg(0);
// Defines weak constants used by the stack to tell it which device contains
// the volatile configuration information. This device name appears in
// HwInit.cxx that creates the device drivers.
extern const char *const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 300, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

// Defines the GPIO ports used for the producers and the consumers.
// Defines the GPIO ports used for the producers and the consumers.

// The first LED is driven by the blinker device from BlinkerGPIO.hxx. WE just
// create an alias for symmetry.
typedef BLINKER_Pin XLED_B1_Pin;


// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The ConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the configuration configuration from the CDI definition, and
// the hardware pin definition. The virtual node pointer comes from the stack
// object. The configuration structure comes from the CDI definition object,
// segment 'seg', in which there is a repeated group 'consumers', and we assign
// the individual entries to the individual consumers. Each consumer gets its
// own GPIO pin.
openlcb::ConfiguredConsumer consumer_1(
    stack.node(), cfg.seg().consumers().entry<0>(), LED_B1_Pin());
openlcb::ConfiguredConsumer consumer_2(
    stack.node(), cfg.seg().consumers().entry<1>(), LED_B2_Pin());
openlcb::ConfiguredConsumer consumer_3(
    stack.node(), cfg.seg().consumers().entry<2>(), LED_B3_Pin());
openlcb::ConfiguredConsumer consumer_4(
    stack.node(), cfg.seg().consumers().entry<3>(), LED_B4_Pin());

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().producers().entry<0>(), SW1_Pin());
openlcb::ConfiguredProducer producer_sw2(
    stack.node(), cfg.seg().producers().entry<1>(), SW2_Pin());

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(
    stack.node(), {producer_sw1.polling(), producer_sw2.polling()});


/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, false);

    stack.add_can_port_select("/dev/fakecan0");

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
