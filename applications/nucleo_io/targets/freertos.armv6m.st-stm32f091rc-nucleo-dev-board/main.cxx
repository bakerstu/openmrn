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
#include "openlcb/ServoConsumer.hxx"

#include "freertos_drivers/st/Stm32Gpio.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/MCP23017Gpio.hxx"
#include "os/MmapGpio.hxx"
#include "config.hxx"
#include "hardware.hxx"
#include "PWM.hxx"
#include "i2c.h"
#include "i2c-dev.h"

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
OVERRIDE_CONST(main_thread_stack_size, 2500);

// Specifies the 48-bit OpenLCB node identifier. This must be unique for every
// hardware manufactured, so in production this should be replaced by some
// easily incrementable method.
extern const openlcb::NodeID NODE_ID;

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
// The size of the memory space to export over the above device. We verify that
// the available eeprom is not too full (8192 max) to avoid quick wear-out
// of the flash. Recommended to have at least 10% spare.
extern const size_t openlcb::CONFIG_FILE_SIZE =
    cfg.seg().size() + cfg.seg().offset();
static_assert(openlcb::CONFIG_FILE_SIZE <= 7000, "Need to adjust eeprom size");

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

// servoPwmCountPerMs defined in hardware.hxx.
// PWM* servo_channels[] defined in HwInit.cxx
openlcb::ServoConsumer srv0(
    stack.node(), cfg.seg().servo_consumers().entry<0>(),
    servoPwmCountPerMs, servo_channels[0]);
openlcb::ServoConsumer srv1(
    stack.node(), cfg.seg().servo_consumers().entry<1>(),
    servoPwmCountPerMs, servo_channels[1]);
openlcb::ServoConsumer srv2(
    stack.node(), cfg.seg().servo_consumers().entry<2>(),
    servoPwmCountPerMs, servo_channels[2]);
openlcb::ServoConsumer srv3(
    stack.node(), cfg.seg().servo_consumers().entry<3>(),
    servoPwmCountPerMs, servo_channels[3]);

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
            fd, "OpenLCB DevKit + Nucleo dev board.");
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
    SpiIOShiftRegister(Service *dedicated_executor,
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
    {}

    /// Initializes SPI port. Called via main() to not rely on static init ordering.
    /// @param port is the character device name for the SPI port
    void init(const char *port) {
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

SpiIOShiftRegister internal_outputs(&io_service, nullptr, OUT_LAT_Pin::instance(), output_register, 2);

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
#ifndef PORTD_SNAP
    &PORTD_LINE1, &PORTD_LINE2, &PORTD_LINE3, &PORTD_LINE4, //
    &PORTD_LINE5, &PORTD_LINE6, &PORTD_LINE7, &PORTD_LINE8, //
#endif
    &PORTE_LINE1, &PORTE_LINE2, &PORTE_LINE3, &PORTE_LINE4, //
    &PORTE_LINE5, &PORTE_LINE6, &PORTE_LINE7, &PORTE_LINE8  //
};

openlcb::MultiConfiguredConsumer portde_consumers(stack.node(), kPortDEGpio,
    ARRAYSIZE(kPortDEGpio), cfg.seg().portde_consumers());

#ifdef PORTD_SNAP

openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_1(
    stack.node(), cfg.seg().snap_switches().entry<0>(), (const Gpio*)&PORTD_LINE1);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_2(
    stack.node(), cfg.seg().snap_switches().entry<1>(), (const Gpio*)&PORTD_LINE2);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_3(
    stack.node(), cfg.seg().snap_switches().entry<2>(), (const Gpio*)&PORTD_LINE3);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_4(
    stack.node(), cfg.seg().snap_switches().entry<3>(), (const Gpio*)&PORTD_LINE4);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_5(
    stack.node(), cfg.seg().snap_switches().entry<4>(), (const Gpio*)&PORTD_LINE5);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_6(
    stack.node(), cfg.seg().snap_switches().entry<5>(), (const Gpio*)&PORTD_LINE6);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_7(
    stack.node(), cfg.seg().snap_switches().entry<6>(), (const Gpio*)&PORTD_LINE7);
openlcb::ConfiguredPulseConsumer turnout_pulse_consumer_8(
    stack.node(), cfg.seg().snap_switches().entry<7>(), (const Gpio*)&PORTD_LINE8);

#endif

uint32_t input_register[2] = {0};

SpiIOShiftRegister internal_inputs(&io_service, nullptr, INP_LAT_Pin::instance(), nullptr, 0, input_register, 3);

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

#if NUM_MCPIOS > 0
MCP23017 exp0(&io_executor, 0, 0, 0);
MCP23017 exp1(&io_executor, 0, 0, 1);

constexpr const MCP23017Gpio IOEXT0_A0(&exp0, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT0_A1(&exp0, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT0_A2(&exp0, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT0_A3(&exp0, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT0_A4(&exp0, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT0_A5(&exp0, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT0_A6(&exp0, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT0_A7(&exp0, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT0_B0(&exp0, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT0_B1(&exp0, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT0_B2(&exp0, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT0_B3(&exp0, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT0_B4(&exp0, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT0_B5(&exp0, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT0_B6(&exp0, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT0_B7(&exp0, MCP23017::PORTB, 7);

constexpr const MCP23017Gpio IOEXT1_A0(&exp1, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT1_A1(&exp1, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT1_A2(&exp1, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT1_A3(&exp1, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT1_A4(&exp1, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT1_A5(&exp1, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT1_A6(&exp1, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT1_A7(&exp1, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT1_B0(&exp1, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT1_B1(&exp1, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT1_B2(&exp1, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT1_B3(&exp1, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT1_B4(&exp1, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT1_B5(&exp1, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT1_B6(&exp1, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT1_B7(&exp1, MCP23017::PORTB, 7);

#if NUM_MCPIOS > 2
MCP23017 exp10(&io_executor, 0, 1, 0);
MCP23017 exp11(&io_executor, 0, 1, 1);

constexpr const MCP23017Gpio IOEXT10_A0(&exp10, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT10_A1(&exp10, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT10_A2(&exp10, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT10_A3(&exp10, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT10_A4(&exp10, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT10_A5(&exp10, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT10_A6(&exp10, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT10_A7(&exp10, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT10_B0(&exp10, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT10_B1(&exp10, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT10_B2(&exp10, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT10_B3(&exp10, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT10_B4(&exp10, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT10_B5(&exp10, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT10_B6(&exp10, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT10_B7(&exp10, MCP23017::PORTB, 7);

constexpr const MCP23017Gpio IOEXT11_A0(&exp11, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT11_A1(&exp11, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT11_A2(&exp11, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT11_A3(&exp11, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT11_A4(&exp11, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT11_A5(&exp11, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT11_A6(&exp11, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT11_A7(&exp11, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT11_B0(&exp11, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT11_B1(&exp11, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT11_B2(&exp11, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT11_B3(&exp11, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT11_B4(&exp11, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT11_B5(&exp11, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT11_B6(&exp11, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT11_B7(&exp11, MCP23017::PORTB, 7);
#endif

#if NUM_MCPIOS > 4
MCP23017 exp20(&io_executor, 1, 0, 0);
MCP23017 exp21(&io_executor, 1, 0, 1);

constexpr const MCP23017Gpio IOEXT20_A0(&exp20, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT20_A1(&exp20, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT20_A2(&exp20, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT20_A3(&exp20, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT20_A4(&exp20, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT20_A5(&exp20, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT20_A6(&exp20, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT20_A7(&exp20, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT20_B0(&exp20, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT20_B1(&exp20, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT20_B2(&exp20, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT20_B3(&exp20, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT20_B4(&exp20, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT20_B5(&exp20, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT20_B6(&exp20, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT20_B7(&exp20, MCP23017::PORTB, 7);

constexpr const MCP23017Gpio IOEXT21_A0(&exp21, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT21_A1(&exp21, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT21_A2(&exp21, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT21_A3(&exp21, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT21_A4(&exp21, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT21_A5(&exp21, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT21_A6(&exp21, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT21_A7(&exp21, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT21_B0(&exp21, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT21_B1(&exp21, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT21_B2(&exp21, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT21_B3(&exp21, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT21_B4(&exp21, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT21_B5(&exp21, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT21_B6(&exp21, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT21_B7(&exp21, MCP23017::PORTB, 7);
#endif

#if NUM_MCPIOS > 6
MCP23017 exp30(&io_executor, 1, 1, 0);
MCP23017 exp31(&io_executor, 1, 1, 1);

constexpr const MCP23017Gpio IOEXT30_A0(&exp30, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT30_A1(&exp30, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT30_A2(&exp30, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT30_A3(&exp30, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT30_A4(&exp30, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT30_A5(&exp30, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT30_A6(&exp30, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT30_A7(&exp30, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT30_B0(&exp30, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT30_B1(&exp30, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT30_B2(&exp30, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT30_B3(&exp30, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT30_B4(&exp30, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT30_B5(&exp30, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT30_B6(&exp30, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT30_B7(&exp30, MCP23017::PORTB, 7);

constexpr const MCP23017Gpio IOEXT31_A0(&exp31, MCP23017::PORTA, 0);
constexpr const MCP23017Gpio IOEXT31_A1(&exp31, MCP23017::PORTA, 1);
constexpr const MCP23017Gpio IOEXT31_A2(&exp31, MCP23017::PORTA, 2);
constexpr const MCP23017Gpio IOEXT31_A3(&exp31, MCP23017::PORTA, 3);
constexpr const MCP23017Gpio IOEXT31_A4(&exp31, MCP23017::PORTA, 4);
constexpr const MCP23017Gpio IOEXT31_A5(&exp31, MCP23017::PORTA, 5);
constexpr const MCP23017Gpio IOEXT31_A6(&exp31, MCP23017::PORTA, 6);
constexpr const MCP23017Gpio IOEXT31_A7(&exp31, MCP23017::PORTA, 7);

constexpr const MCP23017Gpio IOEXT31_B0(&exp31, MCP23017::PORTB, 0);
constexpr const MCP23017Gpio IOEXT31_B1(&exp31, MCP23017::PORTB, 1);
constexpr const MCP23017Gpio IOEXT31_B2(&exp31, MCP23017::PORTB, 2);
constexpr const MCP23017Gpio IOEXT31_B3(&exp31, MCP23017::PORTB, 3);
constexpr const MCP23017Gpio IOEXT31_B4(&exp31, MCP23017::PORTB, 4);
constexpr const MCP23017Gpio IOEXT31_B5(&exp31, MCP23017::PORTB, 5);
constexpr const MCP23017Gpio IOEXT31_B6(&exp31, MCP23017::PORTB, 6);
constexpr const MCP23017Gpio IOEXT31_B7(&exp31, MCP23017::PORTB, 7);
#endif

constexpr const Gpio *const kPortExt0[] = {
    &IOEXT0_A0, &IOEXT0_A1, &IOEXT0_A2, &IOEXT0_A3, //
    &IOEXT0_A4, &IOEXT0_A5, &IOEXT0_A6, &IOEXT0_A7, //
    &IOEXT0_B0, &IOEXT0_B1, &IOEXT0_B2, &IOEXT0_B3, //
    &IOEXT0_B4, &IOEXT0_B5, &IOEXT0_B6, &IOEXT0_B7, //
    &IOEXT1_A0, &IOEXT1_A1, &IOEXT1_A2, &IOEXT1_A3, //
    &IOEXT1_A4, &IOEXT1_A5, &IOEXT1_A6, &IOEXT1_A7, //
    &IOEXT1_B0, &IOEXT1_B1, &IOEXT1_B2, &IOEXT1_B3, //
    &IOEXT1_B4, &IOEXT1_B5, &IOEXT1_B6, &IOEXT1_B7  //

#if NUM_MCPIOS > 2
    ,
    &IOEXT10_A0, &IOEXT10_A1, &IOEXT10_A2, &IOEXT10_A3, //
    &IOEXT10_A4, &IOEXT10_A5, &IOEXT10_A6, &IOEXT10_A7, //
    &IOEXT10_B0, &IOEXT10_B1, &IOEXT10_B2, &IOEXT10_B3, //
    &IOEXT10_B4, &IOEXT10_B5, &IOEXT10_B6, &IOEXT10_B7, //
    &IOEXT11_A0, &IOEXT11_A1, &IOEXT11_A2, &IOEXT11_A3, //
    &IOEXT11_A4, &IOEXT11_A5, &IOEXT11_A6, &IOEXT11_A7, //
    &IOEXT11_B0, &IOEXT11_B1, &IOEXT11_B2, &IOEXT11_B3, //
    &IOEXT11_B4, &IOEXT11_B5, &IOEXT11_B6, &IOEXT11_B7
#endif

#if NUM_MCPIOS > 4
    ,
    &IOEXT20_A0, &IOEXT20_A1, &IOEXT20_A2, &IOEXT20_A3, //
    &IOEXT20_A4, &IOEXT20_A5, &IOEXT20_A6, &IOEXT20_A7, //
    &IOEXT20_B0, &IOEXT20_B1, &IOEXT20_B2, &IOEXT20_B3, //
    &IOEXT20_B4, &IOEXT20_B5, &IOEXT20_B6, &IOEXT20_B7, //
    &IOEXT21_A0, &IOEXT21_A1, &IOEXT21_A2, &IOEXT21_A3, //
    &IOEXT21_A4, &IOEXT21_A5, &IOEXT21_A6, &IOEXT21_A7, //
    &IOEXT21_B0, &IOEXT21_B1, &IOEXT21_B2, &IOEXT21_B3, //
    &IOEXT21_B4, &IOEXT21_B5, &IOEXT21_B6, &IOEXT21_B7
#endif

#if NUM_MCPIOS > 6
    ,
    &IOEXT30_A0, &IOEXT30_A1, &IOEXT30_A2, &IOEXT30_A3, //
    &IOEXT30_A4, &IOEXT30_A5, &IOEXT30_A6, &IOEXT30_A7, //
    &IOEXT30_B0, &IOEXT30_B1, &IOEXT30_B2, &IOEXT30_B3, //
    &IOEXT30_B4, &IOEXT30_B5, &IOEXT30_B6, &IOEXT30_B7, //
    &IOEXT31_A0, &IOEXT31_A1, &IOEXT31_A2, &IOEXT31_A3, //
    &IOEXT31_A4, &IOEXT31_A5, &IOEXT31_A6, &IOEXT31_A7, //
    &IOEXT31_B0, &IOEXT31_B1, &IOEXT31_B2, &IOEXT31_B3, //
    &IOEXT31_B4, &IOEXT31_B5, &IOEXT31_B6, &IOEXT31_B7  //
#endif
};

openlcb::MultiConfiguredPC ext0_pcs(
    stack.node(), kPortExt0, ARRAYSIZE(kPortExt0), cfg.seg().ext0_pc());

#endif // if NUM_MCPIOS > 0

openlcb::RefreshLoop loopab(stack.node(),
    {
#if NUM_MCPIOS > 0
        ext0_pcs.polling(),                           //
#endif
        producer_a1.polling(), producer_a2.polling(), //
        producer_a3.polling(), producer_a4.polling(), //
        producer_a5.polling(), producer_a6.polling(), //
        producer_a7.polling(), producer_a8.polling(), //
        producer_b1.polling(), producer_b2.polling(), //
        producer_b3.polling(), producer_b4.polling(), //
        producer_b5.polling(), producer_b6.polling(), //
        producer_b7.polling(), producer_b8.polling(), //
#ifdef PORTD_SNAP
	&turnout_pulse_consumer_1,                    //
        &turnout_pulse_consumer_2,                    //
        &turnout_pulse_consumer_3,                    //
        &turnout_pulse_consumer_4,                    //
        &turnout_pulse_consumer_5,                    //
        &turnout_pulse_consumer_6,                    //
        &turnout_pulse_consumer_7,                    //
        &turnout_pulse_consumer_8                     //
#endif
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

#if NUM_MCPIOS > 0
    {
        int i2cfd = ::open("/dev/i2c0", O_RDWR);
        exp0.init(i2cfd);
        exp1.init(i2cfd);
#if NUM_MCPIOS > 2
        exp10.init(i2cfd);
        exp11.init(i2cfd);
#endif
#if NUM_MCPIOS > 4
        exp20.init(i2cfd);
        exp21.init(i2cfd);
#endif
#if NUM_MCPIOS > 6
        exp30.init(i2cfd);
        exp31.init(i2cfd);
#endif
    }
#endif // NUM_MCPIOS > 0

    internal_outputs.init("/dev/spi1.ioboard");
    internal_inputs.init("/dev/spi2");
    
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
