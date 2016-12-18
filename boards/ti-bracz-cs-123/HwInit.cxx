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
#include "TivaDev.hxx"
#include "hardware.hxx"
#include "DccHardware.hxx"
#include "DummyGPIO.hxx"
#include "TivaEEPROMEmulation.hxx"


#include "TivaRailcom.hxx"
#include "TivaDCC.hxx"


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

/** I2C driver */
static TivaI2C i2c1("/dev/i2c1", I2C1_BASE, INT_I2C1);

const unsigned TivaEEPROMEmulation::FAMILY = TM4C123;
const size_t EEPROMEmulation::SECTOR_SIZE = (4*1024);

static TivaEEPROMEmulation eeprom("/dev/eeprom", 2000);

// Bit storing whether our dcc output is enabled or not.
static bool g_dcc_on = false;

TivaRailcomDriver<RailcomDefs> railcom_driver("/dev/railcom");
TivaDCC<DccHwDefs> dcc_hw("/dev/mainline", &railcom_driver);

extern "C" {

void hw_set_to_safe(void)
{
    dcc_hw.disable_output();
}

void timer1a_interrupt_handler(void)
{
    dcc_hw.interrupt_handler();
}

void timer0a_interrupt_handler(void)
{
  dcc_hw.os_interrupt_handler();
}

void uart1_interrupt_handler(void)
{
  railcom_driver.os_interrupt_handler();
}

/** Blink LED */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

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
    MAP_GPIOPinWrite(LED_BLUE, 0xff);
    dcc_hw.enable_output();
}

void disable_dcc() {
    dcc_hw.disable_output();
    g_dcc_on = false;
    MAP_GPIOPinWrite(LED_BLUE, 0);
}

void setshorted_dcc() {
    g_dcc_on = false;
    MAP_GPIOPinWrite(LED_BLUE, 0);
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

    SW1_Pin::hw_init();

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    // Sets DCC hardware to safe state.
    // These pins are parallel-connected to the outputs.
    set_gpio_extinput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    /* Initialize the DCC Timers and GPIO outputs */
    dcc_hw.hw_init();
    disable_dcc();

    /* Setup the system clock. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    /* Red LED pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    set_gpio_led(LED_RED);

    /* Blinker timer initialization. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, MAP_SysCtlClockGet() / 8);
    MAP_IntEnable(INT_TIMER5A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMER5A, 0);
    MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER5_BASE, TIMER_A);

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

    /* I2C pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    MAP_GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    MAP_GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    MAP_GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

    /* Blue LED pin initialization */
    set_gpio_led(LED_BLUE);
    set_gpio_led(LED_GREEN);
    MAP_GPIOPinWrite(LED_GREEN, 0);
    MAP_GPIOPinWrite(LED_RED, 0);
    MAP_GPIOPinWrite(LED_BLUE, 0);

    /* USB interrupt priority */
    MAP_IntPrioritySet(INT_USB0, 0xff); // USB interrupt low priority


    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    /* Inserts about 1s of wait in the initialization sequence in order to
     * allow a debugger to connect to the target. */
    rest_pattern = 0xAAAA;
    blinker_pattern = 0;
    asm("cpsie i\n");
    while (rest_pattern) {
      if (!SW1_Pin::get()) {
        blinker_pattern = 0xAAAA;
      } else {
        blinker_pattern = 0;
      }
    }
    asm("cpsid i\n");
}

}  // extern "C"
