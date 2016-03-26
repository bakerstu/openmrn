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
#include "TivaEEPROMEmulation.hxx"
#include "hardware.hxx"

#include "FreeRTOSTCP.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static TivaUart uart2("/dev/ser0", UART2_BASE, INT_RESOLVE(INT_UART2_, 0));

extern "C" void uart2_interrupt_handler(void)
{
    uart2.interrupt_handler();
}

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

/** USB Device CDC serial driver instance */
static TivaCdc cdc0("/dev/serUSB0", INT_RESOLVE(INT_USB0_, 0));

const unsigned TivaEEPROMEmulation::FAMILY = TM4C129;
const size_t EEPROMEmulation::SECTOR_SIZE = (1024 * 16);

static TivaEEPROMEmulation eeprom("/dev/eeprom", 1024);

/** FreeRTOS TCPIP instance */
static FreeRTOSTCP tcpip;

extern "C" {
/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    /// @todo (balazs.racz) make a timer event trigger immediately
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
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
                     (rest_pattern & 1) ? GPIO_PIN_1 : 0);
    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
}

void hw_set_to_safe(void)
{
}

void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    hw_set_to_safe();
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    // Globally disables interrupts until the FreeRTOS scheduler is up.
    asm("cpsid i\n");

    // Setup the system clock.
    MAP_SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_USE_PLL |
                           SYSCTL_OSC_MAIN | SYSCTL_CFG_VCO_480,
                           120000000);

    // Unlocks the gpio pin that is mapped onto NMI.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xff;

    // Initalizes all declared pins from hardware.hxx.
    GpioInit::hw_init();

    // Blinker timer initialization.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    MAP_TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMER5_BASE, TIMER_A, configCPU_CLOCK_HZ / 8);
    MAP_TimerControlStall(TIMER5_BASE, TIMER_A, true);
    MAP_IntEnable(INT_TIMER5A);

    // This interrupt should hit even during kernel operations.
    MAP_IntPrioritySet(INT_TIMER5A, 0);
    MAP_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMER5_BASE, TIMER_A);

    // USB interrupt is low priority.
    MAP_IntPrioritySet(INT_USB0, 0xff);
}

void hw_init()
{
    // Start the FreeRTOS TCPIP processes
    tcpip.start();
}

}
