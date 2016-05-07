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

#include <cstdint>
#include <new>

#include "hardware.hxx"

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
#include "TivaDev.hxx"
#include "TivaDCC.hxx"
#include "TivaGPIO.hxx"
#include "TivaFlash.hxx"
#include "TivaEEPROMEmulation.hxx"

#include "commandstation/dcc_control.hxx"
#include "DccHardware.hxx"
#include "DummyGPIO.hxx"


struct Debug {
  // High between start_cutout and end_cutout from the TivaRailcom driver.
  typedef RC_DEBUG_Pin RailcomDriverCutout;
  // Flips every time an uart byte is received error.
  typedef DummyPin RailcomError;
  // Flips every time an 'E0' byte is received in the railcom driver.
  typedef DummyPin RailcomE0;
  typedef DummyPin RailcomCh2Data;

    typedef DummyPin RailcomDataReceived;
    typedef DummyPin RailcomAnyData;
    typedef DummyPin RailcomPackets;
};
#include "TivaRailcom.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

extern char __automata_start[];
extern char __automata_end[];
extern char __flash_page_size;

static TivaFlash flash0("/etc/automata", __automata_start, __automata_end - __automata_start, reinterpret_cast<uint32_t>(&__flash_page_size));

const unsigned TivaEEPROMEmulation::FAMILY = TM4C129;
const size_t EEPROMEmulation::SECTOR_SIZE = (16 * 1024);
static TivaEEPROMEmulation eeprom("/dev/eeprom", 4*1024);
const bool EEPROMEmulation::SHADOW_IN_RAM = true;

/** USB Device CDC serial driver instance */
static TivaCdc cdc0("/dev/serUSB0", INT_RESOLVE(INT_USB0_, 0));

/** UART 0 serial driver instance */
static TivaUart uart2("/dev/ser0", UART2_BASE, INT_RESOLVE(INT_UART2_, 0));

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

// Bit storing whether our dcc output is enabled or not.
static bool g_dcc_on = false;

TivaRailcomDriver<RailcomDefs> railcom_driver("/dev/railcom");
TivaDCC<DccHwDefs> dcc_hw("/dev/mainline", &railcom_driver);

extern "C" {
/** Timer interrupt for DCC packet handling.
 */
void timer0a_interrupt_handler(void)
{
  dcc_hw.interrupt_handler();
}

void timer1a_interrupt_handler(void)
{
  dcc_hw.os_interrupt_handler();
}

void uart6_interrupt_handler(void)
{
  railcom_driver.os_interrupt_handler();
}

void uart2_interrupt_handler(void)
{
  uart2.interrupt_handler();
}


void hw_set_to_safe(void)
{
    dcc_hw.disable_output();
    AccHwDefs::ACC_ENABLE_Pin::hw_set_to_safe();
}

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

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
    io::BlinkerLed::set(rest_pattern & 1);
    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
}

void diewith(uint32_t pattern)
{
    hw_set_to_safe();
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** Configures a GPIO pin to directly drive a LED (with 8mA output drive). */
void set_gpio_led(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0xff);
    MAP_GPIOPinTypeGPIOOutput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(port, pin, 0xff);
}


/** Configures a GPIO pin to directly drive a LED (with 2mA output drive). */
void set_gpio_drive_low(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOOutput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(port, pin, 0);
}

/** Configures a GPIO pin to directly drive a LED (with 2mA output drive). */
void set_gpio_drive_high(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0xff);
    MAP_GPIOPinTypeGPIOOutput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(port, pin, 0xff);
}

/** Configures a gpio pin for input with external pullup. */
void set_gpio_extinput(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOInput(port, pin);
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
}

/** Configures a gpio pin for input with internal pullup. */
void set_gpio_puinput(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOInput(port, pin);
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

void enable_dcc() {
    g_dcc_on = true;
    io::TrackOnLed::set(true);
    dcc_hw.enable_output();
}

void disable_dcc() {
    dcc_hw.disable_output();
    g_dcc_on = false;
    io::TrackOnLed::set(false);
}

void setshorted_dcc() {
    g_dcc_on = false;
    io::TrackOnLed::set(false);
    dcc_hw.output_set_shorted();
}

bool query_dcc() {
  return g_dcc_on;
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    GpioInit::hw_init();

    /* First we take care of the important pins that control the power
     * transistors */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Q2/Q3 are wired together with A2-A3 on the timer output pins. Let's turn
    // them into high-impedance mode and turn off pullups.
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_2 | GPIO_PIN_3); 
    // Disables any pullup/down.
    MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_2 | GPIO_PIN_3,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); 

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    dcc_hw.hw_init();
    disable_dcc();
    RC_DEBUG_Pin::hw_init();

    // controls the accessory bus.
    AccHwDefs::ACC_ENABLE_Pin::hw_set_to_safe();

    /* Setup the system clock. */
    MAP_SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL |
                           SYSCTL_OSC_MAIN | SYSCTL_CFG_VCO_480,
                           120000000);

    // USB pins initialization.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xff;
    MAP_GPIOPinConfigure(GPIO_PD6_USB0EPEN);
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPinTypeUSBDigital(GPIO_PORTD_BASE, GPIO_PIN_6);
    MAP_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_4);
    MAP_IntPrioritySet(INT_USB0, 0xff); // USB interrupt low priority

    /* blinker LED pin initialization */
    io::BlinkerLed::hw_init();

    /* Blinker timer initialization. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, configCPU_CLOCK_HZ / 8);
    MAP_TimerControlStall(TIMER5_BASE, TIMER_A, true);
    MAP_IntEnable(INT_TIMER5A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMER5A, 0);
    MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER5_BASE, TIMER_A);

    /* UART0 pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_GPIOPinConfigure(GPIO_PD4_U2RX);
    MAP_GPIOPinConfigure(GPIO_PD5_U2TX);
    MAP_GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /* CAN pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    /* USB0 pin initialization */
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    //MAP_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    /* USB interrupt priority */

    /* Boosterpack LEDs */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    LED_RED_Pin::hw_init();
    LED_YELLOW_Pin::hw_init();
    LED_BLUE_Pin::hw_init();
#ifdef HW_V1
    set_gpio_extinput(LED_GREEN_Pin::GPIO_BASE, LED_GREEN_Pin::GPIO_PIN);
#elif defined(HW_V2)
    LED_GREEN_Pin::hw_init();
#else
#error define version with HWVER=HW_V1 or HWVER=HW_V2
#endif
    // onboard LEDs.
    LED_B1_Pin::hw_init(); LED_B1_Pin::set(true);
    LED_B2_Pin::hw_init(); LED_B2_Pin::set(true);
    LED_B3_Pin::hw_init(); LED_B3_Pin::set(true);
    LED_B4_Pin::hw_init(); LED_B4_Pin::set(true);

    // Switches
    USR_SW1_Pin::hw_init();
    USR_SW2_Pin::hw_init();

    /* Timer hardware for DCC signal generation */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);


    /* Temporary enable Irq for blinking. */
    asm("cpsie i\n");

    io::TrackOnLed::set(true);

    blinker_pattern = 0x80002;
    uint32_t counter = (configCPU_CLOCK_HZ / 3);
    while (!USR_SW1_Pin::get() || counter) {
      if (counter) --counter;
    }

    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    LED_B1_Pin::set(false);
    LED_B2_Pin::set(false);
    LED_B3_Pin::set(false);
    LED_B4_Pin::set(false);

}

}  // extern C
