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
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "os/OS.hxx"
#include "TivaDev.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
//static TivaUart uart0("/dev/ser0", UART0_BASE, INT_RESOLVE(INT_UART0_, 0));

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

extern "C" {
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
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_6,
                     (rest_pattern & 1) ? GPIO_PIN_6 : 0);
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

/** Configures a gpio pin for active-high output interfacing with the railroad
 * (such as solenoids or relays). This output type is normally low and it is
 * important not to keep it high or floating for too long even during boot. */
void set_gpio_output(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOOutput(port, pin); 
    MAP_GPIOPinWrite(port, pin, 0);
}

/** Configures a gpio pin for input with external pullup. */
void set_gpio_extinput(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOInput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD); 
}

/** Configures a gpio pin for a button/switch (input with pullup). */
void set_gpio_switch(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0);
    MAP_GPIOPinTypeGPIOInput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); 
}

/** Configures a GPIO pin to directly drive a LED (with 8mA output drive). */
void set_gpio_led(uint32_t port, uint32_t pin) {
    MAP_GPIOPinWrite(port, pin, 0xff);
    MAP_GPIOPinTypeGPIOOutput(port, pin); 
    MAP_GPIOPadConfigSet(port, pin, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD); 
    MAP_GPIOPinWrite(port, pin, 0xff);
}

void hw_set_to_safe(void) {
  
    set_gpio_output(GPIO_PORTC_BASE, GPIO_PIN_4); // Rel0
    set_gpio_output(GPIO_PORTC_BASE, GPIO_PIN_5); // Rel1
    set_gpio_output(GPIO_PORTG_BASE, GPIO_PIN_5); // Rel2
    set_gpio_output(GPIO_PORTF_BASE, GPIO_PIN_3); // Rel3

    set_gpio_output(GPIO_PORTE_BASE, GPIO_PIN_4); // Out0
    set_gpio_output(GPIO_PORTE_BASE, GPIO_PIN_5); // Out1
    set_gpio_output(GPIO_PORTD_BASE, GPIO_PIN_0); // Out2
    set_gpio_output(GPIO_PORTD_BASE, GPIO_PIN_1); // Out3
    set_gpio_output(GPIO_PORTD_BASE, GPIO_PIN_2); // Out4
    set_gpio_output(GPIO_PORTD_BASE, GPIO_PIN_3); // Out5
    set_gpio_output(GPIO_PORTE_BASE, GPIO_PIN_2); // Out6
    set_gpio_output(GPIO_PORTE_BASE, GPIO_PIN_3); // Out7


}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* We initialize the output pins to their default state before doing the
     * clock switching to avoid floating pins that interface to the
     * railroad. */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);


    set_gpio_led(GPIO_PORTD_BASE, GPIO_PIN_6); // Red led
    set_gpio_led(GPIO_PORTB_BASE, GPIO_PIN_0); // Yellow led
    set_gpio_led(GPIO_PORTD_BASE, GPIO_PIN_5); // Green led
    set_gpio_led(GPIO_PORTG_BASE, GPIO_PIN_1); // Blue led 1

    set_gpio_led(GPIO_PORTB_BASE, GPIO_PIN_6); // Blue led (for sw)
    set_gpio_led(GPIO_PORTB_BASE, GPIO_PIN_7); // Gold led (for sw)

    hw_set_to_safe(); // initializes all output pins.

    set_gpio_extinput(GPIO_PORTA_BASE, 0xff);  // In0..7 -- all bits.
    set_gpio_switch(GPIO_PORTC_BASE, GPIO_PIN_6);  // Blue button
    set_gpio_switch(GPIO_PORTC_BASE, GPIO_PIN_7);  // Gold button

    /* Setup the system clock. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_20MHZ);

    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);*/

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
    /*MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    */

    /* USB0 pin initialization */
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    /* CAN pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_GPIOPinConfigure(GPIO_PB4_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PB5_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_GPIOPinConfigure(GPIO_PB1_U1TX);
    MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);
}
}
