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
 * Main file for the io board application on the STM32 Nucleo board.
 *
 * @author Balazs Racz
 * @date 5 Jun 2015
 */

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/ConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "config.hxx"
#include "hardware.hxx"

// These preprocessor symbols are used to select which physical connections
// will be enabled in the main(). See @ref appl_main below.
//#define SNIFF_ON_SERIAL
//#define SNIFF_ON_USB
#define HAVE_PHYSICAL_CAN_PORT

// Changes the default behavior by adding a newline after each gridconnect
// packet. Makes it easier for debugging the raw device.
OVERRIDE_CONST(gc_generate_newlines, 1);
// Specifies how much RAM (in bytes) we allocate to the stack of the main
// thread. Useful tuning parameter in case the application runs out of memory.
OVERRIDE_CONST(main_thread_stack_size, 1300);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID = 0x050101011816ULL;

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
extern const char *const openlcb::CONFIG_FILENAME = "/dev/eeprom";
// The size of the memory space to export over the above device.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 512, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

// Object that handles factory reset for our config setup.
class CustomFactoryReset : public DefaultConfigUpdateListener {
public:
    void factory_reset(int fd) override
    {
        // Resets user names.
        cfg.userinfo().name().write(fd, "Default user name");
        cfg.userinfo().description().write(fd, "Default user description");
        // Makes the IO pin descriptions with default value.
        cfg.seg().consumers().entry<0>().description().write(fd, "LD3");
        cfg.seg().consumers().entry<1>().description().write(fd, "D3");
        cfg.seg().consumers().entry<2>().description().write(fd, "D4");
        cfg.seg().consumers().entry<3>().description().write(fd, "D5");
        cfg.seg().consumers().entry<4>().description().write(fd, "D6");
        
        cfg.seg().producers().entry<0>().description().write(fd, "A0");
        cfg.seg().producers().entry<1>().description().write(fd, "A1");
        cfg.seg().producers().entry<2>().description().write(fd, "A2");
        cfg.seg().producers().entry<3>().description().write(fd, "A3");
        for (unsigned i = 0; i < cfg.seg().producers().num_repeats(); ++i) {
            cfg.seg().producers().entry(i).debounce().write(fd, 3);
        }
    }

    UpdateAction apply_configuration(
        int fd, bool initial_load, BarrierNotifiable *done) override {
        done->notify();
        // Nothing to do; we don't read the configuration.
        return UPDATED;
    }
} g_custom_factory_reset;

// Instantiates the actual producer and consumer objects for the given GPIO
// pins from above. The ConfiguredConsumer class takes care of most of the
// complicated setup and operation requirements. We need to give it the virtual
// node pointer, the configuration configuration from the CDI definition, and
// the hardware pin definition. The virtual node pointer comes from the stack
// object. The configuration structure comes from the CDI definition object,
// segment 'seg', in which there is a repeated group 'consumers', and we assign
// the individual entries to the individual consumers. Each consumer gets its
// own GPIO pin.
openlcb::ConfiguredConsumer consumer_green(
    stack.node(), cfg.seg().consumers().entry<0>(), LED_GREEN_Pin());
openlcb::ConfiguredConsumer consumer_d3(
    stack.node(), cfg.seg().consumers().entry<1>(), OUT_D3_Pin());
openlcb::ConfiguredConsumer consumer_d4(
    stack.node(), cfg.seg().consumers().entry<2>(), OUT_D4_Pin());
openlcb::ConfiguredConsumer consumer_d5(
    stack.node(), cfg.seg().consumers().entry<3>(), OUT_D5_Pin());
openlcb::ConfiguredConsumer consumer_d6(
    stack.node(), cfg.seg().consumers().entry<4>(), OUT_D6_Pin());

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_a0(
    stack.node(), cfg.seg().producers().entry<0>(), IN_A0_Pin());
openlcb::ConfiguredProducer producer_a1(
    stack.node(), cfg.seg().producers().entry<1>(), IN_A1_Pin());
openlcb::ConfiguredProducer producer_a2(
    stack.node(), cfg.seg().producers().entry<2>(), IN_A2_Pin());
openlcb::ConfiguredProducer producer_a3(
    stack.node(), cfg.seg().producers().entry<3>(), IN_A3_Pin());

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(stack.node(),
    {
        producer_a0.polling(), //
        producer_a1.polling(), //
        producer_a2.polling(), //
        producer_a3.polling()  //
    });

/** Entry point to application.
 * @param argc number of command line arguments
 * @param argv array of command line arguments
 * @return 0, should never return
 */
int appl_main(int argc, char *argv[])
{
    stack.check_version_and_factory_reset(
        cfg.seg().internal_config(), openlcb::CANONICAL_VERSION, false);

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
#if defined(SNIFF_ON_USB)
    stack.add_gridconnect_port("/dev/serUSB0");
#endif
#if defined(SNIFF_ON_SERIAL)
    stack.add_gridconnect_port("/dev/ser0");
#endif

    // This command donates the main thread to the operation of the
    // stack. Alternatively the stack could be started in a separate stack and
    // then application-specific business logic could be executed ion a busy
    // loop in the main thread.
    stack.loop_executor();
    return 0;
}
