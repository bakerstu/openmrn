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

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"

#include "config.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/LoggingGPIO.hxx"

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011409ULL;

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
extern const char *const openlcb::CONFIG_FILENAME = "/tmp/config_eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

// None of these pins exist in Linux.
typedef DummyPinWithRead LED_RED_Pin;
typedef DummyPinWithRead LED_GREEN_Pin;
typedef DummyPinWithRead LED_BLUE_Pin;
typedef DummyPinWithRead SW1_Pin;
typedef DummyPinWithRead SW2_Pin;

constexpr char PULSE1[] = "pulse1";
constexpr char PULSE2[] = "pulse2";
typedef LoggingPinWithRead<PULSE1> PULSE1_Pin;
typedef LoggingPinWithRead<PULSE2> PULSE2_Pin;

// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The ConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the configuration configuration from the CDI definition, and
// the hardware pin definition. The virtual node pointer comes from the stack
// object. The configuration structure comes from the CDI definition object,
// segment 'seg', in which there is a repeated group 'consumers', and we assign
// the individual entries to the individual consumers. Each consumer gets its
// own GPIO pin.
openlcb::ConfiguredConsumer consumer_red(
    stack.node(), cfg.seg().consumers().entry<0>(), LED_RED_Pin());
openlcb::ConfiguredConsumer consumer_green(
    stack.node(), cfg.seg().consumers().entry<1>(), LED_GREEN_Pin());
openlcb::ConfiguredConsumer consumer_blue(
    stack.node(), cfg.seg().consumers().entry<2>(), LED_BLUE_Pin());

openlcb::ConfiguredPulseConsumer consumer_pulse1(
    stack.node(), cfg.seg().pulseconsumers().entry<0>(), PULSE1_Pin());
openlcb::ConfiguredPulseConsumer consumer_pulse2(
    stack.node(), cfg.seg().pulseconsumers().entry<1>(), PULSE2_Pin());

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().producers().entry<0>(), SW1_Pin());
openlcb::ConfiguredProducer producer_sw2(
    stack.node(), cfg.seg().producers().entry<1>(), SW2_Pin());

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(
    stack.node(), {&consumer_pulse1, &consumer_pulse2});

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.create_config_file_if_needed(cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, openlcb::CONFIG_FILE_SIZE);
    // Connects to a TCP hub on the internet.
    //stack.connect_tcp_gridconnect_hub("28k.ch", 50007);
    stack.connect_tcp_gridconnect_hub("localhost", 12021);
    // Causes all packets to be dumped to stdout.
    stack.print_all_packets();
    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
