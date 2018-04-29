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
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/MmapGPIO.hxx"
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
static_assert(openlcb::CONFIG_FILE_SIZE <= 1000, "Need to adjust eeprom size");
// The SNIP user-changeable information in also stored in the above eeprom
// device. In general this could come from different eeprom segments, but it is
// simpler to keep them together.
extern const char *const openlcb::SNIP_DYNAMIC_FILENAME =
    openlcb::CONFIG_FILENAME;

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
    stack.node(), cfg.seg().nucleo_onboard().green_led(), LED_GREEN_Pin());

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_sw1(
    stack.node(), cfg.seg().nucleo_onboard().user_btn(), SW_USER_Pin());

// The producers need to be polled repeatedly for changes and to execute the
// debouncing algorithm. This class instantiates a refreshloop and adds the two
// producers to it.
openlcb::RefreshLoop loop(
    stack.node(), {producer_sw1.polling()});

// List of GPIO objects that will be used for the output pins. You should keep
// the constexpr declaration, because it will produce a compile error in case
// the list of pointers cannot be compiled into a compiler constant and thus
// would be placed into RAM instead of ROM.
constexpr const Gpio *const kDirectGpio[] = {
    TDRV1_Pin::instance(), TDRV2_Pin::instance(), //
    TDRV3_Pin::instance(), TDRV4_Pin::instance(), //
    TDRV5_Pin::instance(), TDRV6_Pin::instance(), //
    TDRV7_Pin::instance(), TDRV8_Pin::instance()  //
};

openlcb::MultiConfiguredConsumer direct_consumers(stack.node(), kDirectGpio,
    ARRAYSIZE(kDirectGpio), cfg.seg().direct_consumers());

constexpr const Gpio *const kServoGpio[] = {
    SRV1_Pin::instance(), SRV2_Pin::instance(), //
    SRV3_Pin::instance(), SRV4_Pin::instance(), //
    SRV5_Pin::instance(), SRV6_Pin::instance(), //
    SRV7_Pin::instance(), SRV8_Pin::instance()  //
};

openlcb::MultiConfiguredConsumer servo_consumers(stack.node(), kServoGpio,
    ARRAYSIZE(kServoGpio), cfg.seg().servo_consumers());

class FactoryResetHelper : public DefaultConfigUpdateListener {
public:
    UpdateAction apply_configuration(int fd, bool initial_load,
                                     BarrierNotifiable *done) OVERRIDE {
        AutoNotify n(done);
        return UPDATED;
    }

    void factory_reset(int fd) override
    {
        cfg.userinfo().name().write(fd, "Nucleo IO board");
        cfg.userinfo().description().write(
            fd, "OpenLCB DevKit + F091RC dev board.");
    }
} factory_reset_helper;

class SpiOutputShiftRegister : public StateFlowBase
{
public:
    SpiOutputShiftRegister(Service *dedicated_executor, const char *port,
        AsyncMutex *mutex, const Gpio *latch, uint32_t *storage, unsigned len_bytes,
        unsigned delay_msec = 50)
        : StateFlowBase(dedicated_executor)
        , asyncMutex_(mutex)
        , latch_(latch)
        , delayMsec_(delay_msec)
        , storage_(storage)
        , lenBytes_(len_bytes)
    {
        fd_ = ::open(port, O_WRONLY);
        HASSERT(fd_ >= 0);
        // test alignment
        HASSERT((((uintptr_t)storage) & 3) == 0);
        start_flow(STATE(wait_delay));
    }

private:
    Action wait_delay()
    {
        return sleep_and_call(
            &timer_, MSEC_TO_NSEC(delayMsec_), STATE(refresh));
    }

    Action refresh()
    {
        if (asyncMutex_) {
            return allocate_and_call(STATE(locked), asyncMutex_);
        } else {
            return call_immediately(STATE(locked));
        }
    }

    Action locked() {
        ::write(fd_, storage_, lenBytes_);
        latch_->write(Gpio::SET);
        usleep(15);
        latch_->write(Gpio::CLR);
        if (asyncMutex_) {
            asyncMutex_->Unlock();
        }
        return call_immediately(STATE(wait_delay));
    }

    /// Helper structure.
    StateFlowTimer timer_{this};
    /// File descriptor for the SPI port.
    int fd_;
    /// Async mutex if the SPI port needs it
    AsyncMutex* asyncMutex_;
    /// Latch to trigger activating the output.
    const Gpio* latch_;
    /// How much time to sleep between two refreshes.
    unsigned delayMsec_;
    /// Bit stream of the data to be written. This is purposefully aligned.
    uint32_t* storage_;
    /// How many bytes of data to shift out.
    unsigned lenBytes_;
};

Executor<1> io_executor("io_thread", 1, 1300);
Service io_service(&io_executor);

uint32_t output_register[1] = {0x000050A0};

SpiOutputShiftRegister internal_outputs(&io_service, "/dev/spi1.ioboard", nullptr, OUT_LAT_Pin::instance(), output_register, 2);

constexpr const MmapGpio PORTD_LINE1(output_register, 7, true);
constexpr const MmapGpio PORTD_LINE2(output_register, 6, true);
constexpr const MmapGpio PORTD_LINE3(output_register, 5, true);
constexpr const MmapGpio PORTD_LINE4(output_register, 4, true);
constexpr const MmapGpio PORTD_LINE5(output_register, 3, true);
constexpr const MmapGpio PORTD_LINE6(output_register, 2, true);
constexpr const MmapGpio PORTD_LINE7(output_register, 1, true);
constexpr const MmapGpio PORTD_LINE8(output_register, 0, true);

constexpr const MmapGpio PORTE_LINE1(output_register, 15, true);
constexpr const MmapGpio PORTE_LINE2(output_register, 14, true);
constexpr const MmapGpio PORTE_LINE3(output_register, 13, true);
constexpr const MmapGpio PORTE_LINE4(output_register, 12, true);
constexpr const MmapGpio PORTE_LINE5(output_register, 11, true);
constexpr const MmapGpio PORTE_LINE6(output_register, 10, true);
constexpr const MmapGpio PORTE_LINE7(output_register, 9, true);
constexpr const MmapGpio PORTE_LINE8(output_register, 8, true);

constexpr const Gpio *const kPortDEGpio[] = {
    &PORTD_LINE1, &PORTD_LINE2, &PORTD_LINE3, &PORTD_LINE4, //
    &PORTD_LINE5, &PORTD_LINE6, &PORTD_LINE7, &PORTD_LINE8, //
    &PORTE_LINE1, &PORTE_LINE2, &PORTE_LINE3, &PORTE_LINE4, //
    &PORTE_LINE5, &PORTE_LINE6, &PORTE_LINE7, &PORTE_LINE8  //
};

openlcb::MultiConfiguredConsumer portde_consumers(stack.node(), kPortDEGpio,
    ARRAYSIZE(kPortDEGpio), cfg.seg().portde_consumers());


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
