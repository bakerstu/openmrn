/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * Main file for the io board application on the Nucleo board with the DevKit IO
 * board plugged in.
 *
 * @author Balazs Racz
 * @date 18 April 2018
 */

#define _DEFAULT_SOURCE

#include <unistd.h>

#include "os/os.h"
#include "nmranet_config.h"

#include "openlcb/SimpleStack.hxx"
#include "openlcb/MultiConfiguredConsumer.hxx"
#include "openlcb/ConfiguredProducer.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "os/MmapGpio.hxx"
#include "config.hxx"
#include "hardware.hxx"
#include "PWM.hxx"

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
static_assert(openlcb::CONFIG_FILE_SIZE <= 1900, "Need to adjust eeprom size");
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

const unsigned servo_min = configCPU_CLOCK_HZ * 1 / 1000;
const unsigned servo_max = configCPU_CLOCK_HZ * 2 / 1000;

PWMGPO srv1_gpo(servo_channels[0], servo_min, servo_max);
PWMGPO srv2_gpo(servo_channels[1], servo_min, servo_max);
PWMGPO srv3_gpo(servo_channels[2], servo_min, servo_max);
PWMGPO srv4_gpo(servo_channels[3], servo_min, servo_max);

constexpr const Gpio *const kServoGpio[] = {
    &srv1_gpo, &srv2_gpo, &srv3_gpo, &srv4_gpo, //
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

/// Input/output handler for shift register based IO expansion. Periodically
/// mirrors bitmaps from memory to and from the external shift registers,
/// manipulating a latch GPIO appropriately. Uses a SPI port device. There is
/// no limit on the number of shift registers daisy chained. The input and
/// output shift registers should have independent daisy chains. It is okay if
/// there is no CS wired up for the shift registers as the latch operation will
/// cause all unrelated transfers to be ignored by the shift registers..
///
/// This state flow is blocking the executor during the SPI transfers and
/// should not be used on the main executor.
class SpiIOShiftRegister : public StateFlowBase
{
public:
    /// Constructor.
    /// 
    /// @param dedicated_executor is a serivce that is NOT on the main
    /// executor.
    /// @param port is the character device name for the SPI port
    /// @param mutex if not null, this (async) mutex will be acquired before
    /// the SPI port is touched. Allows mutual exclusion of multiple stateflows
    /// of this class.
    /// @param latch is the IO pin to operate the latch on the shift
    /// registers. The expectation is that output shift registers latch on the
    /// low to high edge of this IO, and input shift registers are shifting
    /// while high and loading on clocks while low. This matches the 74*595 and
    /// 74*166 series' behavior.
    /// @param output_storage bit array of the data to output to the shift
    /// registers. If nullptr, no output shift registers will be operated. Must
    /// be aligned to word boundary, but the individual bytes will be output in
    /// address-increasing order, i.e. the first byte to be output is the byte
    /// pointed by output_storage. (This is LSB-first on little endian
    /// architectures.)
    /// @param output_len_bytes how many 8-bit shift registers are daisy
    /// chained on the output.
    /// @param input_storage bit array where to put the data input from the
    /// shift registers. If nullptr, no input shift registers will be
    /// operated. Must be aligned to word boundary, but the individual bytes
    /// address-increasing order, i.e. the first byte to be output is the byte
    /// pointed by output_storage. (This is LSB-first on little endian
    /// architectures.)
    /// @param input_len_bytes how many 8-bit shift registers are daisy
    /// chained on the input. 
    /// @param delay_msec defines the delay between consecutive refreshes of
    /// the shift registers (inversely the refresh rate).
    SpiIOShiftRegister(Service *dedicated_executor, const char *port,
        AsyncMutex *mutex, const Gpio *latch, uint32_t *output_storage,
        unsigned output_len_bytes, uint32_t *input_storage = nullptr,
        unsigned input_len_bytes = 0, unsigned delay_msec = 50)
        : StateFlowBase(dedicated_executor)
        , asyncMutex_(mutex)
        , latch_(latch)
        , delayMsec_(delay_msec)
        , outputStorage_(output_storage)
        , outputLenBytes_(output_len_bytes)
        , inputStorage_(input_storage)
        , inputLenBytes_(input_len_bytes)
    {
        fd_ = ::open(port, O_RDWR);
        HASSERT(fd_ >= 0);
        // test alignment
        HASSERT((((uintptr_t)outputStorage_) & 3) == 0);
        HASSERT((((uintptr_t)inputStorage_) & 3) == 0);
        start_flow(STATE(wait_delay));
    }

    /// Executes a refresh cycle right now.
    void trigger() {
        timer_.ensure_triggered();
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
        latch_->write(Gpio::CLR);
        if (outputStorage_) {
            auto rb = ::write(fd_, outputStorage_, outputLenBytes_);
            HASSERT((size_t)rb == outputLenBytes_);
        } else {
            // need to get some clock cycles while lat==low to load the input
            // shift register
            uint8_t d = 0;
            ::write(fd_, &d, 1);
            latch_->write(Gpio::CLR);
        }
        usleep(2);
        latch_->write(Gpio::SET);
        if (inputStorage_) {
            auto rb = ::read(fd_, inputStorage_, inputLenBytes_);
            HASSERT((size_t)rb == inputLenBytes_);
        }
        usleep(2);
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
    uint32_t* outputStorage_;
    /// How many bytes of data to shift out.
    unsigned outputLenBytes_;
    /// Bit stream of the data to be read. This is purposefully aligned.
    uint32_t* inputStorage_;
    /// How many bytes of data to shift in.
    unsigned inputLenBytes_;
};

Executor<1> io_executor("io_thread", 1, 1300);
Service io_service(&io_executor);

uint32_t output_register[1] = {0x00000000};

SpiIOShiftRegister internal_outputs(&io_service, "/dev/spi1.ioboard", nullptr, OUT_LAT_Pin::instance(), output_register, 2);

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

openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_1(
    stack.node(), cfg.seg().snap_switches().entry<0>(), (const Gpio*)&PORTD_LINE1);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_2(
    stack.node(), cfg.seg().snap_switches().entry<1>(), (const Gpio*)&PORTD_LINE2);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_3(
    stack.node(), cfg.seg().snap_switches().entry<2>(), (const Gpio*)&PORTD_LINE3);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_4(
    stack.node(), cfg.seg().snap_switches().entry<3>(), (const Gpio*)&PORTD_LINE4);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_5(
    stack.node(), cfg.seg().snap_switches().entry<4>(), TDRV5_Pin::instance());
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_6(
    stack.node(), cfg.seg().snap_switches().entry<5>(), TDRV6_Pin::instance());
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_7(
    stack.node(), cfg.seg().snap_switches().entry<6>(), TDRV7_Pin::instance());
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_8(
    stack.node(), cfg.seg().snap_switches().entry<7>(), TDRV8_Pin::instance());

uint32_t input_register[2] = {0};

SpiIOShiftRegister internal_inputs(&io_service, "/dev/spi2", nullptr, INP_LAT_Pin::instance(), nullptr, 0, input_register, 3);

constexpr const MmapGpio PORTB_LINE1(input_register, 0, false);
constexpr const MmapGpio PORTB_LINE2(input_register, 1, false);
constexpr const MmapGpio PORTB_LINE3(input_register, 2, false);
constexpr const MmapGpio PORTB_LINE4(input_register, 3, false);
constexpr const MmapGpio PORTB_LINE5(input_register, 4, false);
constexpr const MmapGpio PORTB_LINE6(input_register, 5, false);
constexpr const MmapGpio PORTB_LINE7(input_register, 6, false);
constexpr const MmapGpio PORTB_LINE8(input_register, 7, false);

constexpr const MmapGpio PORTA_LINE1(input_register, 8, false);
constexpr const MmapGpio PORTA_LINE2(input_register, 9, false);
constexpr const MmapGpio PORTA_LINE3(input_register, 10, false);
constexpr const MmapGpio PORTA_LINE4(input_register, 11, false);
constexpr const MmapGpio PORTA_LINE5(input_register, 12, false);
constexpr const MmapGpio PORTA_LINE6(input_register, 13, false);
constexpr const MmapGpio PORTA_LINE7(input_register, 14, false);
constexpr const MmapGpio PORTA_LINE8(input_register, 15, false);

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_a1(
    stack.node(), cfg.seg().portab_producers().entry<0>(), (const Gpio*)&PORTA_LINE1);
openlcb::ConfiguredProducer producer_a2(
    stack.node(), cfg.seg().portab_producers().entry<1>(), (const Gpio*)&PORTA_LINE2);
openlcb::ConfiguredProducer producer_a3(
    stack.node(), cfg.seg().portab_producers().entry<2>(), (const Gpio*)&PORTA_LINE3);
openlcb::ConfiguredProducer producer_a4(
    stack.node(), cfg.seg().portab_producers().entry<3>(), (const Gpio*)&PORTA_LINE4);
openlcb::ConfiguredProducer producer_a5(
    stack.node(), cfg.seg().portab_producers().entry<4>(), (const Gpio*)&PORTA_LINE5);
openlcb::ConfiguredProducer producer_a6(
    stack.node(), cfg.seg().portab_producers().entry<5>(), (const Gpio*)&PORTA_LINE6);
openlcb::ConfiguredProducer producer_a7(
    stack.node(), cfg.seg().portab_producers().entry<6>(), (const Gpio*)&PORTA_LINE7);
openlcb::ConfiguredProducer producer_a8(
    stack.node(), cfg.seg().portab_producers().entry<7>(), (const Gpio*)&PORTA_LINE8);

// Similar syntax for the producers.
openlcb::ConfiguredProducer producer_b1(
    stack.node(), cfg.seg().portab_producers().entry<8>(), (const Gpio*)&PORTB_LINE1);
openlcb::ConfiguredProducer producer_b2(
    stack.node(), cfg.seg().portab_producers().entry<9>(), (const Gpio*)&PORTB_LINE2);
openlcb::ConfiguredProducer producer_b3(
    stack.node(), cfg.seg().portab_producers().entry<10>(), (const Gpio*)&PORTB_LINE3);
openlcb::ConfiguredProducer producer_b4(
    stack.node(), cfg.seg().portab_producers().entry<11>(), (const Gpio*)&PORTB_LINE4);
openlcb::ConfiguredProducer producer_b5(
    stack.node(), cfg.seg().portab_producers().entry<12>(), (const Gpio*)&PORTB_LINE5);
openlcb::ConfiguredProducer producer_b6(
    stack.node(), cfg.seg().portab_producers().entry<13>(), (const Gpio*)&PORTB_LINE6);
openlcb::ConfiguredProducer producer_b7(
    stack.node(), cfg.seg().portab_producers().entry<14>(), (const Gpio*)&PORTB_LINE7);
openlcb::ConfiguredProducer producer_b8(
    stack.node(), cfg.seg().portab_producers().entry<15>(), (const Gpio*)&PORTB_LINE8);

openlcb::RefreshLoop loopab(stack.node(),
    {
        producer_a1.polling(), producer_a2.polling(), //
        producer_a3.polling(), producer_a4.polling(), //
        producer_a5.polling(), producer_a6.polling(), //
        producer_a7.polling(), producer_a8.polling(), //
        producer_b1.polling(), producer_b2.polling(), //
        producer_b3.polling(), producer_b4.polling(), //
        producer_b5.polling(), producer_b6.polling(), //
        producer_b7.polling(), producer_b8.polling(), //
        &turnout_pulse_consumer_1,                    //
        &turnout_pulse_consumer_2,                    //
        &turnout_pulse_consumer_3,                    //
        &turnout_pulse_consumer_4,                    //
        &turnout_pulse_consumer_5,                    //
        &turnout_pulse_consumer_6,                    //
        &turnout_pulse_consumer_7,                    //
        &turnout_pulse_consumer_8                    //
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

    srv1_gpo.clr();
    srv2_gpo.clr();
    srv3_gpo.clr();
    srv4_gpo.clr();
    
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
