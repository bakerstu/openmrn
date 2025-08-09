/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 *
 * Hardware setup and interrupt routines for the MSPM0L1304 based button-led
 * board.
 *
 * @author Balazs Racz
 * @date 12 Jan 2025
 */

#include <ti/devices/msp/msp.h>

#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include "utils/blinker.h"
#include "freertos_drivers/ti/MSPM0SpiPixelStrip.hxx"

#include "hardware.hxx"

#define POWER_STARTUP_DELAY (16)

#define CPUCLK_FREQ 32000000

uint8_t pix[3] = {3, 3, 3};

SpiPixelStrip strip0(SPI0, 1, pix);

/// Sets the onboard pixel to a given color.
/// @param color is 0x00RRGGBB 
void set_pix(uint32_t color) {
  pix[0] = (color >> 8) & 0xff;
  pix[1] = (color >> 16) & 0xff;
  pix[2] = (color >> 0) & 0xff;
  strip0.update_sync();
}

extern "C" {

void resetblink(uint32_t p)
{
    pix[1] = p ? 30 : 3;
    strip0.update_sync();
}

void setblink(uint32_t p)
{
    resetblink(p);
}

void usleep(uint32_t usecs);

void __attribute__((noreturn)) diewith(uint32_t pattern)
{
  uint32_t p = 1;
  pix[0] = pix[1] = pix[2] = 0;
  while (1) {
    resetblink(p & 1);
    usleep(200000);
    p >>= 1;
    if (!p) p = pattern;
  }
}

// These are replicated from os.c in order not to pull in os.o in the linking
// phase. That file has dependencies on freertos which does not link under a
// bare target.
int g_death_lineno;

void __attribute__((noreturn)) abort(void)
{
  diewith(BLINK_DIE_ABORT);
}

extern uint64_t g_time_msec;
uint64_t g_time_msec = 0;

extern uint64_t current_time_msec()
{
    return g_time_msec;
}

void hw_set_to_safe(void)
{ }

static const DL_SPI_Config gSPI_0_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_15,
    .bitOrder    = DL_SPI_BIT_ORDER_LSB_FIRST,
    .chipSelectPin = DL_SPI_CHIP_SELECT_NONE,
};

static const DL_SPI_ClockConfig gSPI_0_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

void setup_spi() {
    DL_SPI_setClockConfig(SPI0, (DL_SPI_ClockConfig *) &gSPI_0_clockConfig);
    DL_SPI_init(SPI0, (DL_SPI_Config *) &gSPI_0_config);
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     500000 = (32000000)/((1 + 31) * 2)
     *
     * Desired bit time is 0.27 to 0.4 usec per bit, which is 3.7MHz to 2.5MHz.
     * That is 12.8 to 8.64 multiplier for a 32MHz processor. We aim for
     * multiplier 10, which is SCR of 4.
     *
     *     (32000000)/((1 + 4) * 2) = 3.2MHz. That is 0.312 usec per bit.
     *
     * A 0 bit will be 0.31us high 0.62us low.
     * A 1 bit will be 0.62us high 0.31us low.
     */
    DL_SPI_setBitRateSerialClockDivider(SPI0, 4);
    DL_SPI_enable(SPI0);
}


static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

void setup_uart() {
    DL_UART_Main_setClockConfig(UART0, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART0, (DL_UART_Main_Config *) &gUART_0Config);
    
}


/*
 * Timer clock configuration to be sourced by BUSCLK (32 MHz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1 MHz = 32 MHz / (1 * (31 + 1))
 */
static const DL_TimerG_ClockConfig g_usec_timer_ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 31U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (500 ms * 128 Hz) - 1
 */
static const DL_TimerG_TimerConfig g_usec_timer_TimerConfig = {
    .timerMode  = DL_TIMER_TIMER_MODE_ONE_SHOT_UP,
    .period     = 65534,
    .startTimer = DL_TIMER_STOP,
    .genIntermInt = DL_TIMER_INTERM_INT_DISABLED,
};

void setup_timer() {
    DL_TimerG_setClockConfig(USEC_TIMER,
        (DL_TimerG_ClockConfig *) &g_usec_timer_ClockConfig);
    DL_TimerG_enableClock(USEC_TIMER);
    DL_TimerG_initTimerMode(USEC_TIMER,
        (DL_TimerG_TimerConfig *) &g_usec_timer_TimerConfig);
    DL_Timer_setLoadValue(USEC_TIMER, 65535);
    //DL_TimerG_enableInterrupt(USEC_TIMER , DL_TIMERG_INTERRUPT_ZERO_EVENT);
}

extern void (* const __interrupt_vector[])(void);

void hw_preinit(void)
{
    /* Globally disables interrupts. */
    asm("cpsid i\n");

    SCB->VTOR = (volatile uint32_t)&__interrupt_vector;
    
    DL_GPIO_reset(GPIOA);
    DL_GPIO_enablePower(GPIOA);

    DL_SPI_reset(SPI0);
    DL_SPI_enablePower(SPI0);

    DL_UART_Main_reset(UART0);
    DL_UART_Main_enablePower(UART0);

    DL_TimerG_reset(USEC_TIMER);
    DL_TimerG_enablePower(USEC_TIMER);
    
    delay_cycles(POWER_STARTUP_DELAY);

    GpioInit::hw_init();

    /* Module-Specific Initializations*/
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

    // Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    /*
     * Initializes the SysTick period to 1 ms,
     * enables the interrupt, and starts the SysTick Timer
     */
    DL_SYSTICK_config(CPUCLK_FREQ / 1000);

    setup_spi();

    setup_uart();

    setup_timer();
    
    /// Pinmux setup.
    DL_GPIO_initPeripheralOutputFunctionFeatures(
        IOMUX_PINCM19 /*PA18*/, IOMUX_PINCM19_PF_SPI0_PICO,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initPeripheralInputFunction(
        IOMUX_PINCM23 /*PA22*/, IOMUX_PINCM23_PF_UART0_RX);

    DL_GPIO_initPeripheralOutputFunctionFeatures(
        IOMUX_PINCM24 /*PA23*/, IOMUX_PINCM24_PF_UART0_TX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
        DL_GPIO_DRIVE_STRENGTH_HIGH, DL_GPIO_HIZ_ENABLE);

    // Enables interrupts.
    asm("cpsie i\n");
}

extern int appl_main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
    return appl_main(argc, argv);
}

// These are usually defined by freertos, but we are running in no-OS mode due
// to the small flash.

void SVC_Handler(void)
{ }

void SysTick_Handler(void)
{
    g_time_msec++;
}

void PendSV_Handler(void)
{ }

void usleep(uint32_t usecs)
{
  do {
    uint32_t limit = usecs;
    if (limit > 10000) limit = 10000;
    DL_Timer_stopCounter(USEC_TIMER);
    DL_Timer_setTimerCount(USEC_TIMER, 0);
    DL_Timer_startCounter(USEC_TIMER);
    while (DL_Timer_getTimerCount(USEC_TIMER) < limit) {}
    usecs -= limit;
  } while (usecs > 0);

#if 0  
    auto end = current_time_msec() + (usecs + 999) / 1000;
    while (current_time_msec() < end) {
        __WFI();
    }
#endif    
}

// This is needed to avoid pulling in os.o, which has a dependency on freertos.
void ignore_fn() {}

} // extern C

