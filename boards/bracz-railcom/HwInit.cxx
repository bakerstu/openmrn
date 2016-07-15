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
#include "utils/Charlieplex.hxx"
#include "TivaDev.hxx"
#include "TivaEEPROMEmulation.hxx"
#include "DummyGPIO.hxx"
#include "hardware.hxx"
#include "TivaDCCDecoder.hxx"
#include "TivaRailcom.hxx"
#include "custom/TivaGNDControl.hxx"
#include "custom/TivaDAC.hxx"
#include "bootloader_hal.h"
#include "TivaDCC.hxx"


/*struct Debug {
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
  };*/
#include "TivaRailcom.hxx"

// This symbol releases the charlieplex pins to do other stuff.
//#define FAKE_CHARLIE

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static TivaUart uart0("/dev/ser0", UART0_BASE, INT_RESOLVE(INT_UART0_, 0));

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

const unsigned TivaEEPROMEmulation::FAMILY = TM4C123;
const size_t EEPROMEmulation::SECTOR_SIZE = 4 * 1024;
static TivaEEPROMEmulation eeprom("/dev/eeprom", 1024);

const uint32_t RailcomDefs::UART_BASE[] = RAILCOM_BASE;
const uint32_t RailcomDefs::UART_PERIPH[] = RAILCOM_PERIPH;

TivaDAC<DACDefs> dac;
TivaGNDControl gnd_control;
TivaBypassControl bypass_control;

inline void DCCDecode::dcc_packet_finished_hook() {
  RailcomDefs::set_input();
  extern uint8_t dac_next_packet_mode;
  if (!dac_next_packet_mode) {
    extern DacSettings dac_occupancy;
    RailcomDefs::set_feedback_channel(0xff);
    dac.set(dac_occupancy);
  } else {
    extern DacSettings dac_overcurrent;
    RailcomDefs::set_feedback_channel(0xfe);
    dac.set(dac_overcurrent);
  }
}

inline void DCCDecode::dcc_before_cutout_hook() {
  extern DacSettings dac_railcom;
  dac.set(dac_railcom);
  RailcomDefs::set_hw();
}

inline void DCCDecode::after_feedback_hook() {
  extern uint8_t dac_next_packet_mode;
  if (!dac_next_packet_mode) {
    ++dac_next_packet_mode;
    extern DacSettings dac_overcurrent;
    RailcomDefs::set_feedback_channel(0xfe);
    dac.set(dac_overcurrent);
  } else {
    if (++dac_next_packet_mode >= OCCUPANCY_SAMPLE_RATIO) {
      dac_next_packet_mode = 0;
      extern DacSettings dac_occupancy;
      RailcomDefs::set_feedback_channel(0xff);
      dac.set(dac_occupancy);
    }
  }
}

const Gpio *const charlieplex_pins[] = {CHARLIE0_Pin::instance(),
                                        CHARLIE1_Pin::instance(),
                                        CHARLIE2_Pin::instance()};

#ifndef FAKE_CHARLIE
Charlieplex<3> stat_leds(charlieplex_pins);
#endif

unsigned *stat_led_ptr() {
#ifdef FAKE_CHARLIE
  static unsigned payload = 0;
  return &payload;
#else
  return stat_leds.payload();
#endif
}

void RailcomDefs::enable_measurement() {
  Debug::MeasurementEnabled::set(true);
  bypass_control.set(false);
  SysCtlDelay(26);
}

void RailcomDefs::disable_measurement() {
  bypass_control.set(true);
  Debug::MeasurementEnabled::set(false);
}

static TivaRailcomDriver<RailcomDefs> railcom_driver("/dev/railcom");

/** The input pin for detecting the DCC signal. */
static TivaDccDecoder<DCCDecode> nrz0("/dev/nrz0", &railcom_driver);


extern "C" {
/** Blink LED */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

void dcc_generator_init(void);

void hw_set_to_safe(void)
{
    GpioInit::hw_set_to_safe();
}

void enter_bootloader()
{
    /* Globally disables interrupts. */
    asm("cpsid i\n");
    hw_set_to_safe();
    __bootloader_magic_ptr = REQUEST_BOOTLOADER;
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

void reboot()
{
    MAP_SysCtlReset();
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

void timer4a_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

#ifndef FAKE_CHARLIE
    stat_leds.tick();
#endif
}


void timer2b_interrupt_handler(void)
{
  nrz0.interrupt_handler();
}

void timer2a_interrupt_handler(void)
{
  nrz0.rcom_interrupt_handler();
}

void uart1_interrupt_handler(void)
{
  railcom_driver.os_interrupt_handler();
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

    //
    // Unlock PF0 so we can change it to a GPIO input
    // Once we have enabled (unlocked) the commit register then re-lock it
    // to prevent further changes.  PF0 is muxed with NMI thus a special case.
    //
    MAP_SysCtlPeripheralEnable(SW2_Pin::GPIO_PERIPH);
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_CR) |= 0x01;
    HWREG(SW2_Pin::GPIO_BASE + GPIO_O_LOCK) = 0;

    MAP_SysCtlPeripheralEnable(DAC_TIMER_Pin::GPIO_PERIPH);
    HWREG(DAC_TIMER_Pin::GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(DAC_TIMER_Pin::GPIO_BASE + GPIO_O_CR) |= 0x80;
    HWREG(DAC_TIMER_Pin::GPIO_BASE + GPIO_O_LOCK) = 0;

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
    MAP_IntPrioritySet(INT_TIMER5A, 0x80);
    MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER5_BASE, TIMER_A);

    /* Charlieplexed pins */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    MAP_TimerLoadSet(TIMER4_BASE, TIMER_A, configCPU_CLOCK_HZ / 10000);
    MAP_IntEnable(INT_TIMER4A);
    // Still above kernel but not prio zero
    MAP_IntPrioritySet(INT_TIMER4A, 0x80);
    MAP_TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER4_BASE, TIMER_A);

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
}

}
