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
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin.h"
#include "os/OS.hxx"
#include "DummyGPIO.hxx"
#include "CC32xxUart.hxx"
#include "CC32xxWiFi.hxx"
#include "hardware.hxx"
#include "bootloader_hal.h"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static CC32xxUart uart0("/dev/ser0", UARTA0_BASE, INT_UARTA0);

/** Wi-Fi instance */
static CC32xxWiFi wifi;

extern "C"
{

extern void (* const __interrupt_vector[])(void);
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

/** Blink LED */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

void dcc_generator_init(void);

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

void timer3a_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMERA3_BASE, TIMER_TIMA_TIMEOUT);
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

/** Initialize the processor hardware pre C runtime init.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* Setup the interrupt vector table */
    MAP_IntVTableBaseSet((unsigned long)&__interrupt_vector[0]);

    /* Setup the system clock. */
    PRCMCC3200MCUInit();

    /* initilize pin modes:
     *   PIN_01:  PIN_MODE_0 - GPIO10
     *   PIN_02:  PIN_MODE_0 - GPIO11
     *   PIN_04:  PIN_MODE_0 - GPIO13
     *   PIN_15:  PIN_MODE_0 - GPIO22
     *   PIN_55:  PIN_MODE_3 - UARTA0_TX
     *   PIN_57:  PIN_MODE_3 - UARTA0_RX
     *   PIN_64:  PIN_MODE_0 - GPIO9
     */
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    GpioInit::hw_init();

    /* Blinker timer initialization. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA3_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMERA3_BASE, TIMER_A, cm3_cpu_clock_hz / 8);
    MAP_IntEnable(INT_TIMERA3A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMERA3A, 0);
    MAP_TimerIntEnable(TIMERA3_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMERA3_BASE, TIMER_A);

    //GPIOPinWrite(unsigned long GPIO, unsigned char ucPins, unsigned char ucVal)
#if 0
    /* Checks the SW2 pin at boot time in case we want to allow for a debugger
     * to connect. */
    asm volatile ("cpsie i\n");
    do {
      if (SW2_Pin::get()) {
        blinker_pattern = 0xAAAA;
      } else {
        blinker_pattern = 0;
      }
    } while (blinker_pattern || rest_pattern);
    asm volatile ("cpsid i\n");
#endif
}

/** Initialize the processor hardware post C runtime init.
 */
void hw_init(void)
{
    wifi.instance()->start();
}

} /* extern "C" */
