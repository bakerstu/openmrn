/** \copyright
 * Copyright (c) 2012, Stuart W Baker
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
 * \file HwInit.cxx
 * This file represents the hardware initialization for the TI Tiva MCU.
 *
 * @author Stuart W. Baker
 * @date 5 January 2013
 */

#include <new>
#include <cstdint>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "os/OS.hxx"

#include "hardware.hxx"

#include "freertos_drivers/ti/TivaDev.hxx"

#define TIVADCC_TIVA

#include "freertos_drivers/ti/TivaDCC.hxx"
#include "freertos_drivers/ti/TivaEEPROMEmulation.hxx"
#include "freertos_drivers/ti/TivaEEPROMBitSet.hxx"
#include "freertos_drivers/ti/TivaGPIO.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/common/GpioWrapper.hxx"
#include "openlcb/bootloader_hal.h"
#include "dcc/DccOutput.hxx"

struct Debug {
  // High between start_cutout and end_cutout from the TivaRailcom driver.
  typedef DummyPin RailcomDriverCutout;
  // Flips every time an uart byte is received error.
  typedef DummyPin RailcomError;
  // Flips every time an 'E0' byte is received in the railcom driver.
  typedef DummyPin RailcomE0;
  // Sets to 1 if we have seen any data from the railcom uart.
  typedef DummyPin RailcomAnyData;
  // Flips for every byte from the railcom uart.
  typedef DummyPin RailcomDataReceived;
  // Flipped for every packet that is sent from the railcom layer to the
  // application.
  typedef DummyPin RailcomPackets;

  typedef DummyPin RailcomCh2Data;
  typedef DummyPin RailcomRxActivate;
};
#include "freertos_drivers/ti/TivaRailcom.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** USB Device CDC serial driver instance */
static TivaCdc cdc0("/dev/serUSB0", INT_RESOLVE(INT_USB0_, 0));

/** UART 0 serial driver instance */
static TivaUart uart0("/dev/ser0", UART0_BASE, INT_RESOLVE(INT_UART0_, 0));

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

const unsigned TivaEEPROMEmulation::FAMILY = TM4C123;
const size_t EEPROMEmulation::SECTOR_SIZE = (4*1024);

static TivaEEPROMEmulation eeprom("/dev/eeprom", 1500);

extern StoredBitSet* g_gpio_stored_bit_set;
StoredBitSet* g_gpio_stored_bit_set = nullptr;
constexpr unsigned EEPROM_BIT_COUNT = 84;
constexpr unsigned EEPROM_BITS_PER_CELL = 28;

/// This variable will be set to 1 when a write arrives to the eeprom.
uint8_t eeprom_updated = 0;

// Overridesthe default behavior to keep track of eeprom writes.
void EEPROMEmulation::updated_notification()
{
    eeprom_updated = 1;
}

extern "C" {
void hw_set_to_safe(void);

void enter_bootloader()
{
    extern void (* const __interrupt_vector[])(void);
    if (__interrupt_vector[1] == __interrupt_vector[13] ||
        __interrupt_vector[13] == nullptr) {
        // No bootloader detected.
        return;
    }
    hw_set_to_safe();
    __bootloader_magic_ptr = REQUEST_BOOTLOADER;
    /* Globally disables interrupts. */
    asm("cpsid i\n");
    extern char __flash_start;
    asm volatile(" mov   r3, %[flash_addr] \n"
                 :
                 : [flash_addr] "r"(&__flash_start));
    /* Loads SP and jumps to the reset vector. */
    asm volatile(
        " ldr r0, [r3]\n"
        " mov sp, r0\n"
        " ldr r0, [r3, #4]\n"
        " bx  r0\n");
}
}

#define ONE_BIT_HALF_PERIOD  4480
#define ZERO_BIT_HALF_PERIOD 8000
#define STARTUP_DELAY_CYCLES 2
#define DEADBAND_ADJUST      80

struct RailcomDefs
{
    static const uint32_t CHANNEL_COUNT = 1;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 6;

    static const auto OS_INTERRUPT = INT_UART1;

    typedef RAILCOM_CH1_Pin CH1_Pin;

    static void hw_init() {
         CH1_Pin::hw_init();
    }

    static void set_input() {
        CH1_Pin::set_input();
    }

    static void set_hw() {
        CH1_Pin::set_hw();
    }

    static void enable_measurement(bool) {}
    static void disable_measurement() {}
    static bool need_ch1_cutout() { return true; }
    static uint8_t get_feedback_channel() { return 0xff; }
    static void middle_cutout_hook() {}

    /** @returns a bitmask telling which pins are active. Bit 0 will be set if
     * channel 0 is active (drawing current).*/
    static uint8_t sample() {
        uint8_t ret = 0;
        if (!CH1_Pin::get()) ret |= 1;
        return ret;
    }

    static unsigned get_timer_tick() {
        return MAP_TimerValueGet(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    }
};

uint32_t feedback_sample_overflow_count;
const uint32_t RailcomDefs::UART_BASE[] = {UART1_BASE};
const uint32_t RailcomDefs::UART_PERIPH[] = {SYSCTL_PERIPH_UART1};

static TivaRailcomDriver<RailcomDefs> railcom_driver("/dev/railcom");

// If 1, enabled spread spectrum randomization of the DCC timings.
uint8_t spreadSpectrum = 0;

struct DccHwDefs {
  /// base address of a capture compare pwm timer pair
  static const unsigned long CCP_BASE = TIMER0_BASE;
  /// an otherwise unused interrupt number (could be that of the capture compare pwm timer)
  static const unsigned long OS_INTERRUPT = INT_TIMER0A;
  /// base address of an interval timer
  static const unsigned long INTERVAL_BASE = TIMER1_BASE;
  /// interrupt number of the interval timer
  static const unsigned long INTERVAL_INTERRUPT = INT_TIMER1A;

  /// Defines how much time for railcom timing compared to the standard
  /// length this hardware needs. We have to start a bit earlier due to the
  /// slow FET turn-ons.
  static const int RAILCOM_CUTOUT_START_DELTA_USEC = -20;
  static const int RAILCOM_CUTOUT_MID_DELTA_USEC = 0;
  static const int RAILCOM_CUTOUT_END_DELTA_USEC = -10;
  static const int RAILCOM_CUTOUT_POST_DELTA_USEC = -16;
  /// Adds this to the negative half after the railcom cutout is done.
  static const int RAILCOM_CUTOUT_POST_NEGATIVE_DELTA_USEC = -4;
  /// The DCC end of packet bit (after the cutout) has an asymmetry due to the
  /// interrupt CPU latency, this constant tunes it to disappear. This time
  /// gets deducted from the second half.
  static const int RESYNC_DELAY_USEC = 7;

  /** These timer blocks will be synchronized once per packet, when the
   *  deadband delay is set up. */
  static const auto TIMER_SYNC = TIMER_0A_SYNC | TIMER_0B_SYNC | TIMER_1A_SYNC | TIMER_1B_SYNC;

  // Peripherals to enable at boot.
  static const auto CCP_PERIPH = SYSCTL_PERIPH_TIMER1;
  static const auto INTERVAL_PERIPH = SYSCTL_PERIPH_TIMER0;

  using PIN_H = ::BOOSTER_H_Pin;
  using PIN_L = ::BOOSTER_L_Pin;

  /** Defines whether the high driver pin is inverted or not. A non-inverted
   *  (value==false) pin will be driven high during the first half of the DCC
   *  bit (minus H_DEADBAND_DELAY_NSEC at the end), and low during the second
   *  half.  A non-inverted pin will be driven low as safe setting at
   *  startup. */
  static const bool PIN_H_INVERT = false;

  /** Defines whether the drive-low pin is inverted or not. A non-inverted pin
   *  (value==false) will be driven high during the second half of the DCC bit
   *  (minus L_DEADBAND_DELAY_NSEC), and low during the first half.  A
   *  non-inverted pin will be driven low as safe setting at startup. */
  static const bool PIN_L_INVERT = false;

  /** @returns the number of preamble bits to send exclusive of end of packet
   *  '1' bit */
  static int dcc_preamble_count() { return 16; }

  static bool generate_railcom_halfzero() { return true; }

  static void flip_led() {}

  /** the time (in nanoseconds) to wait between turning off the low driver and
   * turning on the high driver. */
  static const int H_DEADBAND_DELAY_NSEC = 0;
  /** the time (in nanoseconds) to wait between turning off the high driver and
   * turning on the low driver. */
  static const int L_DEADBAND_DELAY_NSEC = 0;

  /** number of outgoing messages we can queue */
  static const size_t Q_SIZE = 4;

  // Pins defined for railcom
  using RAILCOM_TRIGGER_Pin = InvertedGpio<::RAILCOM_TRIGGER_Pin>;
  static const auto RAILCOM_TRIGGER_DELAY_USEC = 6;

  struct BOOSTER_ENABLE_Pin
  {
      static void set(bool value)
      {
          if (value)
          {
              PIN_H::set_hw();
              PIN_L::set_hw();
          }
          else
          {
              PIN_H::set(PIN_H_INVERT);
              PIN_H::set_output();
              PIN_H::set(PIN_H_INVERT);

              PIN_L::set(PIN_L_INVERT);
              PIN_L::set_output();
              PIN_L::set(PIN_L_INVERT);
          }
      }

      static void hw_init() {
          PIN_H::hw_init();
          PIN_L::hw_init();
          set(false);
      }
  };

  using InternalBoosterOutput =
      DccOutputHwReal<DccOutput::TRACK, BOOSTER_ENABLE_Pin, RAILCOM_TRIGGER_Pin,
          1, RAILCOM_TRIGGER_DELAY_USEC, 0>;
  using Output1 = InternalBoosterOutput;
  using Output2 = DccOutputHwDummy<DccOutput::PGM>;
  using Output3 = DccOutputHwDummy<DccOutput::LCC>;

  static const auto RAILCOM_UART_BASE = UART1_BASE;
  static const auto RAILCOM_UART_PERIPH = SYSCTL_PERIPH_UART1;
  using RAILCOM_UARTPIN = ::RAILCOM_CH1_Pin;
};


static TivaDCC<DccHwDefs> tivaDCC("/dev/mainline", &railcom_driver);

DccOutput *get_dcc_output(DccOutput::Type type)
{
    switch (type)
    {
        case DccOutput::TRACK:
            return DccOutputImpl<DccHwDefs::InternalBoosterOutput>::instance();
        case DccOutput::PGM:
            return DccOutputImpl<DccHwDefs::Output2>::instance();
        case DccOutput::LCC:
            return DccOutputImpl<DccHwDefs::Output3>::instance();
    }
    return nullptr;
}

extern "C" {
/** Blink LED */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

void dcc_generator_init(void);

void hw_set_to_safe(void)
{
    DccHwDefs::InternalBoosterOutput::set_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
    GpioInit::hw_set_to_safe();
}

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    /* make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

static uint32_t nsec_per_clock = 0;
/// Calculate partial timing information from the tick counter.
long long hw_get_partial_tick_time_nsec(void)
{
    volatile uint32_t * tick_current_reg = (volatile uint32_t *)0xe000e018;
    long long tick_val = *tick_current_reg;
    tick_val *= nsec_per_clock;
    long long elapsed = (1ULL << NSEC_TO_TICK_SHIFT) - tick_val;
    if (elapsed < 0) elapsed = 0;
    return elapsed;
}

void timer5a_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    // Set output LED.
    BLINKER_RAW_Pin::set((rest_pattern & 1));

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
}

void uart1_interrupt_handler(void)
{
  railcom_driver.os_interrupt_handler();
}

void timer1a_interrupt_handler(void)
{
    tivaDCC.interrupt_handler();
}

void timer0a_interrupt_handler(void)
{
    tivaDCC.os_interrupt_handler();
}

void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    nsec_per_clock = 1000000000 / cm3_cpu_clock_hz;
    
    //
    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    //
    MAP_SysCtlPeripheralEnable(SW2_Pin::GPIO_PERIPH);
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_CR) |= 0x01;
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_LOCK) = 0;

    // Initializes all GPIO and hardware pins.
    GpioInit::hw_init();

    /* Setup the system clock. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    /* Blinker timer initialization. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, MAP_SysCtlClockGet() / 8);
    MAP_IntEnable(INT_TIMER5A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMER5A, 0);
    MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER5_BASE, TIMER_A);

    /* USB interrupt priority */
    MAP_IntPrioritySet(INT_USB0, 0xff); // USB interrupt low priority

    /* Initialize the DCC Timers and GPIO outputs */
    tivaDCC.hw_preinit();

    /* Checks the SW1 pin at boot time in case we want to allow for a debugger
     * to connect. */
    asm volatile ("cpsie i\n");
    do {
      if (!SW2_Pin::get()) {
        blinker_pattern = 0xAAAA;
      } else {
        blinker_pattern = 0;
      }
    } while (blinker_pattern || rest_pattern);

    asm volatile ("cpsid i\n");

    g_gpio_stored_bit_set = new EEPROMStoredBitSet<TivaEEPROMHwDefs<EEPROM_BIT_COUNT, EEPROM_BITS_PER_CELL>>(2, 2);
}

void hw_postinit(void)
{
    DccHwDefs::InternalBoosterOutput::clear_disable_reason(
        DccOutput::DisableReason::INITIALIZATION_PENDING);
}

}
