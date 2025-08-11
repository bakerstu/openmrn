/** \copyright
 * Copyright (c) 2016, Stuart W. Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * This file represents the hardware initialization for the TI CC32xx.
 *
 * @author Stuart W. Baker
 * @date 30 May 2016
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
#include "driverlib/utils.h"
#include "os/OS.hxx"
#include "freertos_drivers/common/DummyGPIO.hxx"
#include "freertos_drivers/ti/CC32xxUart.hxx"
#include "freertos_drivers/ti/CC32xxSPI.hxx"
#include "freertos_drivers/net_cc32xx/CC32xxWiFi.hxx"
#include "freertos_drivers/ti/CC32xxDeviceFile.hxx"
#include "freertos_drivers/ti/CC32xxEEPROMEmulation.hxx"
#include "hardware.hxx"
#include "openlcb/bootloader_hal.h"
#include "portmacro.h"

static_assert(INT_PRIORITY_LVL_1 >= configMAX_SYSCALL_INTERRUPT_PRIORITY,
    "Must adjust priority levels so that the network processor interrupt can "
    "call FreeRTOS system functions.");

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

//static CC32xxDeviceFile pref_file("/usr/app.bin");

const size_t EEPROMEmulation::SECTOR_SIZE = 3*1024;
extern CC32xxEEPROMEmulation* eeprom;
CC32xxEEPROMEmulation* eeprom = nullptr;

extern "C"
{

extern void (* const __interrupt_vector[])(void);
void hw_set_to_safe(void);

/** Blink LED pattern */
uint32_t blinker_pattern = 0;
static volatile uint32_t rest_pattern = 0;

/** Set all hardware to safe state.
 */
void hw_set_to_safe(void)
{
    GpioInit::hw_set_to_safe();
    /** @todo make sure to drive all I/O low that is connected to hardware
     * that can be disabled, for example, through a load switch
     */
}

/** Reset the blink pattern.
 * @param pattern pattern corresponding to crash type
 */
void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    /* make a timer event trigger immediately */
}

/** Setup the blink pattern.
 * @param pattern pattern corresponding to crash type
 */
void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

/** timer for blinking LED pattern.
 */
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

/** Crash reporting.
 * @param pattern pattern corresponding to crash type
 */
void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

extern void destructor(void);

void destructor(void) {
    diewith(BLINK_DIE_NMI);
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

    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_02, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_03, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_04, PIN_MODE_0, false);
    MAP_PinTypeSPI(PIN_05, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_06, PIN_MODE_7);
    MAP_PinTypeSPI(PIN_07, PIN_MODE_7);
    MAP_PinTypeGPIO(PIN_08, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_16, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_17, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_18, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_53, PIN_MODE_0, false);
    MAP_PinTypeUART(PIN_55, PIN_MODE_3);
    MAP_PinTypeUART(PIN_57, PIN_MODE_3);
    MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_59, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_61, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_63, PIN_MODE_0, false);
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_PinConfigSet(PIN_05, PIN_STRENGTH_2MA, PIN_TYPE_STD);
    MAP_PinConfigSet(PIN_07, PIN_STRENGTH_2MA, PIN_TYPE_STD);
    MAP_PinConfigSet(PIN_08, PIN_STRENGTH_2MA, PIN_TYPE_STD);

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
}

/** Initialize the processor hardware post C runtime init.
 */
void hw_init(void)
{
    wifi.instance()->start();
}

// This code needs to run after the scheduler has been started.
void hw_postinit(void)
{
    /*SyncNotifiable n;
    wifi.run_on_network_thread([&n]() {
        eeprom = new CC32xxEEPROMEmulation("/usr/eeprom", 1000);
        n.notify();
    });
    n.wait_for_notification();
    */
}

} /* extern "C" */
