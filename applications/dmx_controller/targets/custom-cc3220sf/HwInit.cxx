/** \copyright
 * Copyright (c) 2017, Balazs Racz
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
 * This file represents the hardware initialization for the TI CC3220SF Wireless MCU.
 *
 * @author Balazs Racz
 * @date 29 March 2017
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
#include "driverlib/utils.h"
#include "os/OS.hxx"
#include "DummyGPIO.hxx"
#include "CC32xxUart.hxx"
#include "CC32xxSPI.hxx"
#include "CC32xxWiFi.hxx"
#include "MCP2515Can.hxx"
#include "CC32xxEEPROMEmulation.hxx"
#include "hardware.hxx"
#include "bootloader_hal.h"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** Assert the RS-485 transmit enable line.
 */
static void rs485_enable_assert()
{
    RS485_TX_EN_Pin::set(true);
    UtilsDelay(100);
}

/** Deassert the RS-485 transmit enable line.
 */
static void rs485_enable_deassert()
{
    RS485_TX_EN_Pin::set(false);
}

/** UART 0 serial driver instance */
static CC32xxUart uart0("/dev/ser0", UARTA0_BASE, INT_UARTA0, 250000,
                        CC32xxUart::CS8, true,
                        rs485_enable_assert, rs485_enable_deassert);

/** Wi-Fi instance */
CC32xxWiFi wifi;

static CC32xxEEPROMEmulation eeprom("/usr/dmxeeprom", 1500);

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
    GPIOIntEnable(MCP2515_INT_N_Pin::GPIO_BASE, MCP2515_INT_N_Pin::GPIO_PIN);
}

/** Disable MCP2515 CAN controller interrupt.
 */
static void mcp2515_int_disable()
{
    GPIOIntDisable(MCP2515_INT_N_Pin::GPIO_BASE, MCP2515_INT_N_Pin::GPIO_PIN);
}

/** SPI 0.0 driver instance */
static CC32xxSPI spi0_0("/dev/spidev0.0", GSPI_BASE, INT_GSPI,
                        mcp2515_cs_assert, mcp2515_cs_deassert);

static MCP2515Can can0("/dev/can0", mcp2515_int_enable, mcp2515_int_disable);

extern void eeprom_flush() {
    eeprom.flush();
}

extern "C"
{

void __attribute__((__weak__)) eeprom_updated_notification() {
    eeprom_flush();
}


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
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void timer1a_interrupt_handler(void)
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

void diewith(uint32_t pattern)
{
    /* Globally disables interrupts. */
    asm("cpsid i\n");
    hw_set_to_safe();

    resetblink(pattern);
    while (1)
    {
        if (MAP_TimerIntStatus(BLINKER_TIMER_BASE, true) & BLINKER_TIMER_TIMEOUT)
        {
            timer1a_interrupt_handler();
        }
    }
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

    // initilize pin modes:
    init_pinmux<0>();

    /* Blinker timer initialization. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA1, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(BLINKER_TIMER_BASE, TIMER_CFG_PERIODIC);
    MAP_TimerLoadSet(BLINKER_TIMER_BASE, BLINKER_TIMER, cm3_cpu_clock_hz / 8);
    MAP_IntEnable(BLINKER_TIMER_INT);

    /* This interrupt should hit even during kernel operations. */
    MAP_IntPrioritySet(BLINKER_TIMER_INT, 0);
    MAP_TimerIntEnable(BLINKER_TIMER_BASE, BLINKER_TIMER_TIMEOUT);
    MAP_TimerEnable(BLINKER_TIMER_BASE, BLINKER_TIMER);

    /* MCP2515 CAN Controller reset hold */
    MCP2515_RESET_N_Pin::set(false);

    /* MCP2515 CAN Controller clock timer initialization. */
    MAP_PRCMPeripheralClkEnable(PRCM_TIMERA2, PRCM_RUN_MODE_CLK);
    MAP_TimerConfigure(TIMERA2_BASE,
        TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_PWM | TIMER_CFG_A_PERIODIC);
    MAP_TimerLoadSet(TIMERA2_BASE, TIMER_B, 3);
    MAP_TimerMatchSet(TIMERA2_BASE, TIMER_B, 1);
    MAP_TimerEnable(TIMERA2_BASE, TIMER_B);

    /* MCP2515 CAN Controller gpio interrupt initialization */
    GPIOIntTypeSet(MCP2515_INT_N_Pin::GPIO_BASE, MCP2515_INT_N_Pin::GPIO_PIN, GPIO_LOW_LEVEL);
    MAP_IntPrioritySet(MCP2515_INT_N_Pin::GPIO_INT, configKERNEL_INTERRUPT_PRIORITY);
    MAP_IntEnable(MCP2515_INT_N_Pin::GPIO_INT);

    /* MCP2515 CAN Controller reset release */
    MCP2515_RESET_N_Pin::set(true);

    /* Checks the SW2 pin at boot time in case we want to allow for a debugger
     * to connect.
    asm volatile ("cpsie i\n");
    do {
      if (SW2_Pin::get()) {
        blinker_pattern = 0xAAAA;
      } else {
        blinker_pattern = 0;
      }
    } while (blinker_pattern || rest_pattern);
    asm volatile ("cpsid i\n"); */
}

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

/** Initialize the processor hardware post C runtime init.
 */
void hw_init(void)
{
    // Initialize the SPI driver for the CC32xx network processor connection.
    extern void SPI_init(void);
    SPI_init();

    wifi.instance()->start();
}

/** Initialize the processor hardware post platform init.
 */
void hw_postinit(void)
{
    can0.init("/dev/spidev0.0", 20000000, config_nmranet_can_bitrate());
    SyncNotifiable n;
    wifi.run_on_network_thread([&n]() {
        eeprom.mount();
        n.notify();
    });
    n.wait_for_notification();
}

} /* extern "C" */
