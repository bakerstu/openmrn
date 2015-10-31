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
#include "hardware.hxx"
#include "TivaEEPROMEmulation.hxx"
#include "TivaDCCDecoder.hxx"
#include "TivaRailcom.hxx"
#include "bootloader_hal.h"

//#define ERASE_ALL_EEPROM

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
//static TivaUart uart2("/dev/ser2", UART2_BASE, INT_RESOLVE(INT_UART2_, 0));

/** CAN 0 CAN driver instance */
static TivaCan can0("/dev/can0", CAN0_BASE, INT_RESOLVE(INT_CAN0_, 0));

#ifdef USE_WII_CHUCK
static TivaI2C i2c4("/dev/i2c4", I2C4_BASE, INT_I2C4);
#endif

#ifdef HAVE_RAILCOM
TivaRailcomDriver<RailcomHw> railcom_driver("/dev/railcom");

/** The input pin for detecting the DCC signal. */
static TivaDccDecoder<DCCDecode> nrz0("/dev/nrz0", &railcom_driver);
#endif

const unsigned TivaEEPROMEmulation::FAMILY = TM4C123;
const size_t EEPROMEmulation::SECTOR_SIZE = (1024);
static TivaEEPROMEmulation eeprom("/dev/eeprom", 512);

extern "C" {

void hw_set_to_safe(void);

void enter_bootloader()
{
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

void reboot()
{
    MAP_SysCtlReset();
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
    BlinkerLed::set((rest_pattern & 1));
    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
}

#ifdef HAVE_RAILCOM
void wide_timer4a_interrupt_handler(void)
{
  nrz0.interrupt_handler();
}

void wide_timer4b_interrupt_handler(void)
{
  nrz0.os_interrupt_handler();
}

void uart2_interrupt_handler(void)
{
  railcom_driver.os_interrupt_handler();
}
#endif

void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

void hw_set_to_safe(void) {
    OUT0_Pin::hw_set_to_safe();
    OUT1_Pin::hw_set_to_safe();
    OUT2_Pin::hw_set_to_safe();
    OUT3_Pin::hw_set_to_safe();
    OUT4_Pin::hw_set_to_safe();
    OUT5_Pin::hw_set_to_safe();
    OUT6_Pin::hw_set_to_safe();
    OUT7_Pin::hw_set_to_safe();

    REL0_Pin::hw_set_to_safe();
    REL1_Pin::hw_set_to_safe();
    REL2_Pin::hw_set_to_safe();
    REL3_Pin::hw_set_to_safe();
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

    hw_set_to_safe(); // initializes all output pins.

    LED_RED_RAW_Pin::hw_init();
    LED_GREEN_Pin::hw_init();
    LED_YELLOW_Pin::hw_init();
    LED_BLUE_Pin::hw_init();

    LED_BLUE_SW_Pin::hw_init();
    LED_GOLD_SW_Pin::hw_init();

    IN0_Pin::hw_init();
    IN1_Pin::hw_init();
    IN2_Pin::hw_init();
    IN3_Pin::hw_init();
    IN4_Pin::hw_init();
    IN5_Pin::hw_init();
    IN6_Pin::hw_init();
    IN7_Pin::hw_init();

    BUT_BLUE_Pin::hw_init();
    BUT_GOLD_Pin::hw_init();

#ifdef HAVE_RAILCOM
    // Railcom pins
    RC4_SHADOW_Pin::hw_init();
#endif

    /* Setup the system clock. */
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_20MHZ);

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

#ifdef USE_WII_CHUCK
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
    MAP_GPIOPinConfigure(GPIO_PG2_I2C4SCL);
    MAP_GPIOPinConfigure(GPIO_PG3_I2C4SDA);
    MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_2);
    MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_3);
#endif

    /* CAN pin initialization */
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_GPIOPinConfigure(GPIO_PB4_CAN0RX);
    MAP_GPIOPinConfigure(GPIO_PB5_CAN0TX);
    MAP_GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    /// TODO(balazs.racz) reenable this for the TX of the signalbus.
    //MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    MAP_GPIOPinConfigure(GPIO_PB1_U1TX);
    MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);
    //DBG_SIGNAL_Pin::hw_init();


#ifdef ERASE_ALL_EEPROM
    for (uint32_t address = (uint32_t) __eeprom_start;
         address < (uint32_t) __eeprom_start;
         address += TivaEEPROMEmulation::TM4C123) {
        MAP_FlashErase(address);
    }
#endif    

    /*    MAP_GPIOIntTypeSet(IN0_Pin::GPIO_BASE, IN0_Pin::GPIO_PIN, GPIO_BOTH_EDGES);
    HASSERT(GPIO_INT_PIN_0 == GPIO_PIN_0);
    MAP_GPIOIntEnable(IN0_Pin::GPIO_BASE, IN0_Pin::GPIO_PIN);
    MAP_IntEnable(INT_GPIOA);*/
}

/*void porta_interrupt_handler(void)
{
  MAP_GPIOIntClear(IN0_Pin::GPIO_BASE, IN0_Pin::GPIO_PIN);
  LED_GOLD_SW_Pin::set(IN0_Pin::get());
  }*/

}
