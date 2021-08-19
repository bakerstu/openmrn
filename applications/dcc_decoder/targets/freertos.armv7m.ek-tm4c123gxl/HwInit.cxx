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

#include "TivaDev.hxx"

#include "TivaEEPROMEmulation.hxx"
#include "TivaEEPROMBitSet.hxx"
#include "TivaGPIO.hxx"
#include "DummyGPIO.hxx"
#include "bootloader_hal.h"

struct Debug
{
    typedef DummyPin DccPacketDelay;
    typedef DummyPin DccDecodeInterrupts;
    typedef DummyPin DccPacketFinishedHook;
    typedef DummyPin CapTimerOverflow;
};

#include "TivaDCCDecoder.hxx"


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

struct DCCDecode
{
    static const auto TIMER_BASE = TIMER2_BASE;
    static const auto TIMER_PERIPH = SYSCTL_PERIPH_TIMER2;
    static const auto TIMER_INTERRUPT = INT_TIMER2B;
    static const auto TIMER = TIMER_B;
    static const auto CFG_TIM_CAPTURE =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME;
    static const auto CFG_RCOM_TIMER =
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC;

    // Interrupt bits.
    static const auto TIMER_CAP_EVENT = TIMER_CAPB_EVENT;
    static const auto TIMER_RCOM_MATCH = TIMER_TIMA_MATCH;

    // Sets the match register of TIMER to update immediately.
    static void clr_tim_mrsu() {
      HWREG(TIMER_BASE + TIMER_O_TAMR) &= ~(TIMER_TAMR_TAMRSU);
      HWREG(TIMER_BASE + TIMER_O_TAMR) |= (TIMER_TAMR_TAMIE);
    }

    static void cap_event_hook() {}

    static const auto RCOM_TIMER = TIMER_A;
    static const auto SAMPLE_PERIOD_CLOCKS = 60000;
    //static const auto SAMPLE_TIMER_TIMEOUT = TIMER_TIMA_TIMEOUT;
    static const auto RCOM_INTERRUPT = INT_TIMER2A;
    static const auto OS_INTERRUPT = INT_WTIMER2A;
    typedef DCC_IN_Pin NRZ_Pin;

    // 16-bit timer max + use 7 bits of prescaler.
    static const uint32_t TIMER_MAX_VALUE = 0x800000UL;
    static const uint32_t PS_MAX = 0x80;

    static_assert(SAMPLE_PERIOD_CLOCKS < TIMER_MAX_VALUE, "Cannot sample less often than the timer period");

    static const int Q_SIZE = 32;

    // after 5 overcurrent samples we get one occupancy sample
    static const uint32_t OCCUPANCY_SAMPLE_RATIO = 5;
    static inline void dcc_before_cutout_hook() {}
    static inline void dcc_packet_finished_hook() {}
    static inline void after_feedback_hook() {}
    /// How many usec later/earlier should the railcom cutout start happen.
    static int time_delta_railcom_pre_usec()
    {
        return 0;
    }

    /// How many usec later/earlier should the railcom cutout middle happen.
    static int time_delta_railcom_mid_usec()
    {
        return 0;
    }

    /// How many usec later/earlier should the railcom cutout end happen.
    static int time_delta_railcom_end_usec()
    {
        return 0;
    }
};

// Dummy implementation because we are not a railcom detector.
NoRailcomDriver railcom_driver;

/** The input pin for detecting the DCC signal. */
static TivaDccDecoder<DCCDecode> dcc_decoder0("/dev/dcc_decoder0", &railcom_driver);

extern "C" {
/** Blink LED */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
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

void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

void timer2b_interrupt_handler(void)
{
  dcc_decoder0.interrupt_handler();
}

void timer2a_interrupt_handler(void)
{
  dcc_decoder0.rcom_interrupt_handler();
}

void wide_timer2a_interrupt_handler(void)
{
  dcc_decoder0.os_interrupt_handler();
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

} // extern "C"
