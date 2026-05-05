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
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "os/OS.hxx"
#include "freertos_drivers/ti/TivaDev.hxx"
#include "freertos_drivers/ti/TivaDCC.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"

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

#define ONE_BIT_HALF_PERIOD  4480
#define ZERO_BIT_HALF_PERIOD 8000
#define STARTUP_DELAY_CYCLES 2
#define DEADBAND_ADJUST      80

#define DECL_PIN(NAME, PORT, NUM)                \
  static const auto NAME##_PERIPH = SYSCTL_PERIPH_GPIO##PORT; \
  static const auto NAME##_BASE = GPIO_PORT##PORT##_BASE; \
  static const auto NAME##_PIN = GPIO_PIN_##NUM


struct RailcomDefs
{
    static const uint32_t CHANNEL_COUNT = 1;
    static const uint32_t UART_PERIPH[CHANNEL_COUNT];
    static const uint32_t UART_BASE[CHANNEL_COUNT];
    // Make sure there are enough entries here for all the channels times a few
    // DCC packets.
    static const uint32_t Q_SIZE = 6;

    static const auto OS_INTERRUPT = INT_UART1;

    GPIO_HWPIN(CH1, GpioHwPin, B, 0, U1RX, UART);

    static void hw_init() {
         CH1_Pin::hw_init();
    }

    static void set_input() {
        CH1_Pin::set_input();
    }

    static void set_hw() {
        CH1_Pin::set_hw();
    }

    /** @returns a bitmask telling which pins are active. Bit 0 will be set if
     * channel 0 is active (drawing current).*/
    static uint8_t sample() {
        uint8_t ret = 0;
        if (!CH1_Pin::get()) ret |= 1;
        return ret;
    }
};

const uint32_t RailcomDefs::UART_BASE[] = {UART1_BASE};
const uint32_t RailcomDefs::UART_PERIPH[] = {SYSCTL_PERIPH_UART1};

static TivaRailcomDriver<RailcomDefs> railcom_driver("/dev/railcom");

struct DccHwDefs {
  /// base address of a capture compare pwm timer pair
  static const unsigned long CCP_BASE = TIMER0_BASE;
  /// an otherwise unused interrupt number (could be that of the capture compare pwm timer)
  static const unsigned long OS_INTERRUPT = INT_TIMER0A;
  /// base address of an interval timer
  static const unsigned long INTERVAL_BASE = TIMER1_BASE;
  /// interrupt number of the interval timer
  static const unsigned long INTERVAL_INTERRUPT = INT_TIMER1A;

  /** These timer blocks will be synchronized once per packet, when the
   *  deadband delay is set up. */
  static const auto TIMER_SYNC = TIMER_0A_SYNC | TIMER_0B_SYNC | TIMER_1A_SYNC | TIMER_1B_SYNC;

  // Peripherals to enable at boot.
  static const auto CCP_PERIPH = SYSCTL_PERIPH_TIMER1;
  static const auto INTERVAL_PERIPH = SYSCTL_PERIPH_TIMER0;
  static const auto PIN_H_GPIO_PERIPH = SYSCTL_PERIPH_GPIOB;
  static const auto PIN_L_GPIO_PERIPH = SYSCTL_PERIPH_GPIOB;

  static const auto PIN_H_GPIO_CONFIG = GPIO_PB6_T0CCP0;
  static const auto PIN_L_GPIO_CONFIG = GPIO_PB7_T0CCP1;

  static const auto PIN_H_GPIO_BASE = GPIO_PORTB_BASE;
  static const auto PIN_L_GPIO_BASE = GPIO_PORTB_BASE;

  static const auto PIN_H_GPIO_PIN = GPIO_PIN_6;
  static const auto PIN_L_GPIO_PIN = GPIO_PIN_7;

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

  static void flip_led() {}

  /** the time (in nanoseconds) to wait between turning off the low driver and
   * turning on the high driver. */
  static const int H_DEADBAND_DELAY_NSEC = 250;
  /** the time (in nanoseconds) to wait between turning off the high driver and
   * turning on the low driver. */
  static const int L_DEADBAND_DELAY_NSEC = 250;

  /** @returns true to produce the RailCom cutout, else false */
  static bool railcom_cutout() { return false; }

  /** number of outgoing messages we can queue */
  static const size_t Q_SIZE = 4;


  // Pins defined for railcom
  //DECL_PIN(RAILCOM_TRIGGER, B, 4);
  DECL_PIN(RAILCOM_TRIGGER, D, 6);
  static const auto RAILCOM_TRIGGER_INVERT = true;

  static const auto RAILCOM_UART_BASE = UART1_BASE;
  static const auto RAILCOM_UART_PERIPH = SYSCTL_PERIPH_UART1;
  DECL_PIN(RAILCOM_UARTPIN, B, 0);
  static const auto RAILCOM_UARTPIN_CONFIG = GPIO_PB0_U1RX;
};


static TivaDCC<DccHwDefs> tivaDCC("/dev/mainline", &railcom_driver);

extern "C" {
/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void dcc_generator_init(void);

void hw_set_to_safe(void)
{
    tivaDCC.disable_output();
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
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1,
                     (rest_pattern & 1) ? GPIO_PIN_1 : 0);
    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
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

    /* Setup the system clock. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    /* Red LED pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);

    /* Blinker timer initialization. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, MAP_SysCtlClockGet() / 8);
    MAP_IntEnable(INT_TIMER5A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMER5A, 0);
    //MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    //MAP_TimerEnable(TIMER5_BASE, TIMER_A);

    /* UART0 pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* USB0 pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    /* CAN pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_GPIOPinConfigure(GPIO_PE4_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PE5_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* Blue LED pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    /* Green LED pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    /* USB interrupt priority */
    MAP_IntPrioritySet(INT_USB0, 0xff); // USB interrupt low priority

    /* Initialize the DCC Timers and GPIO outputs */
    tivaDCC.hw_init();
}

/** Timer interrupt for DCC packet handling.
 */
void timer1a_interrupt_handler(void)
{
    tivaDCC.interrupt_handler();
}

}
