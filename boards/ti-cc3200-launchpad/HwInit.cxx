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
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin.h"
#include "os/OS.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/ti/CC32xxUart.hxx"
#include "freertos_drivers/net_cc32xx/CC32xxWiFi.hxx"
#include "freertos_drivers/common/MCP2515Can.hxx"
#include "hardware.hxx"
#include "openlcb/bootloader_hal.h"

//#define USE_MCP2515
#if defined(USE_MCP2515)
#include "freertos_drivers/ti/CC32xxSPI.hxx"
#endif

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static CC32xxUart uart0("/dev/ser0", UARTA0_BASE, INT_UARTA0);

#if defined(USE_MCP2515)
static void mcp2515_cs_assert();
static void mcp2515_cs_deassert();
static void mcp2515_int_enable();
static void mcp2515_int_disable();

/** SPI 0.0 driver instance */
static CC32xxSPI spi0_0("/dev/spidev0.0", GSPI_BASE, INT_GSPI,
                        mcp2515_cs_assert, mcp2515_cs_deassert);


static MCP2515Can can0("/dev/can0", mcp2515_int_enable, mcp2515_int_disable);

/** Assert the chip select for the MCP2515 CAN controller.
 */
static void mcp2515_cs_assert()
{
    MCP2515_CS_N_Pin::set(false);
}

/** Deassert the chip select for the MCP2515 CAN controller.
 */
static void mcp2515_cs_deassert()
{
    MCP2515_CS_N_Pin::set(true);
}

/** Enable MCP2515 CAN controller interrupt.
 */
static void mcp2515_int_enable()
{
    GPIOIntEnable(GPIOA0_BASE, GPIO_INT_PIN_7);
}

/** Disable MCP2515 CAN controller interrupt.
 */
static void mcp2515_int_disable()
{
    GPIOIntDisable(GPIOA0_BASE, GPIO_INT_PIN_7);
}
#endif
/** Wi-Fi instance */
static CC32xxWiFi wifi;

extern "C"
{

extern void (* const __interrupt_vector[])(void);
void hw_set_to_safe(void);

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

void timer2a_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(TIMERA2_BASE, TIMER_TIMA_TIMEOUT);
    // Set output LED.
    BLINKER_RAW_Pin::set((rest_pattern & 1));

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
}

#if defined(USE_MCP2515)
/** PORTA0 interrupt handler.
 */
void porta0_interrupt_handler(void)
{
    long status = GPIOIntStatus(GPIOA0_BASE, true);

    if (status & GPIO_INT_PIN_7)
    {
        /* MCP2515 CAN Controller interrupt */
        can0.interrupt_handler();
    }
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

/** Initialize the processor hardware pre C runtime init.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    nsec_per_clock = 1000000000 / cm3_cpu_clock_hz;

    /* Setup the interrupt vector table */
    MAP_IntVTableBaseSet((unsigned long)&__interrupt_vector[0]);

    // Disables all interrupts that the bootloader might have enabled.
    HWREG(NVIC_DIS0) = 0xffffffffU;
    HWREG(NVIC_DIS1) = 0xffffffffU;
    HWREG(NVIC_DIS2) = 0xffffffffU;
    HWREG(NVIC_DIS3) = 0xffffffffU;
    HWREG(NVIC_DIS4) = 0xffffffffU;
    HWREG(NVIC_DIS5) = 0xffffffffU;

    /* Setup the system clock. */
    PRCMCC3200MCUInit();

    /* initilize pin modes:
     *   PIN_01:  PIN_MODE_3 - GT_PWM07
     *   PIN_02:  PIN_MODE_0 - GPIO11
     *   PIN_04:  PIN_MODE_0 - GPIO13
     *   PIN_05:  PIN_MODE_7 - GSPI_CLK
     *   PIN_06:  PIN_MODE_7 - GSPI_MISO
     *   PIN_07:  PIN_MODE_7 - GSPI_MOSI
     *   PIN_08:  PIN_MODE_0 - GPIO17
     *   PIN_15:  PIN_MODE_0 - GPIO22
     *   PIN_55:  PIN_MODE_3 - UARTA0_TX
     *   PIN_57:  PIN_MODE_3 - UARTA0_RX
     *   PIN_64:  PIN_MODE_0 - GPIO9
     */
    MAP_PinTypeTimer(PIN_01, PIN_MODE_3);
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    MAP_PinTypeSPI(PIN_05, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_06, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_07, PIN_MODE_7);
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    GpioInit::hw_init();

    /* Blinker timer initialization. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA2_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(TIMERA2_BASE, TIMER_A, cm3_cpu_clock_hz / 8);
    MAP_IntEnable(INT_TIMERA2A);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(INT_TIMERA2A, 0);
    MAP_TimerIntEnable(TIMERA2_BASE, TIMER_TIMA_TIMEOUT);
    MAP_TimerEnable(TIMERA2_BASE, TIMER_A);

#if defined(USE_MCP2515)
    /* MCP2515 CAN Controller reset hold */
    MCP2515_RESET_N_Pin::set(false);

    /* MCP2515 CAN Controller clock timer initialization. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA3, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA3_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
    MAP_TimerLoadSet(TIMERA3_BASE, TIMER_A, 3);
    MAP_TimerMatchSet(TIMERA3_BASE, TIMER_A, 1);
    MAP_TimerEnable(TIMERA3_BASE, TIMER_A);

    /* MCP2515 CAN Controller gpio interrupt initialization */
    GPIOIntTypeSet(GPIOA0_BASE, GPIO_PIN_7, GPIO_LOW_LEVEL);
    MAP_IntEnable(INT_GPIOA0);

    /* MCP2515 CAN Controller reset release */
    MCP2515_RESET_N_Pin::set(true);
#endif
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

/** Initialize the processor hardware post platform init.
 */
void hw_postinit(void)
{
#if defined(USE_MCP2515)
    can0.init("/dev/spidev0.0", 8000000, config_nmranet_can_bitrate());
#endif
}

} /* extern "C" */
