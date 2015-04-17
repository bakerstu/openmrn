/** \copyright
 * Copyright (c) 2015, Stuart W Baker
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
 * This file represents the hardware initialization for the LPC1769 MCU.
 *
 * @author Stuart W. Baker
 * @date 12 April 2015
 */

#include <new>
#include <cstdint>

#include "chip.h"

#include "os/OS.hxx"
#include "Lpc17xx40xxUart.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static LpcUart uart0("/dev/ser0", LPC_UART3, UART3_IRQn);

extern "C" {
const uint32_t OscRateIn = 12000000;
const uint32_t RTCOscRateIn = 32768;

/* Pin muxing configuration */
STATIC const PINMUX_GRP_T pinmuxing[] = {
    {0,  0,   IOCON_MODE_INACT  | IOCON_FUNC2},    /* TXD3 */
    {0,  1,   IOCON_MODE_PULLUP | IOCON_FUNC2},    /* RXD3 */
    {0,  4,   IOCON_MODE_INACT  | IOCON_FUNC2},    /* CAN-RD2 */
    {0,  5,   IOCON_MODE_INACT  | IOCON_FUNC2},    /* CAN-TD2 */
    {0,  22,  IOCON_MODE_INACT  | IOCON_FUNC0},    /* Led 0 */
};

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

/** put hardware in a safe state.
 */
void hw_set_to_safe(void)
{
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

/** Interrupt handler for blinking LED.
 */
void timer3_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    Chip_TIMER_ClearMatch(LPC_TIMER3, 1);
    // Set output LED.
    Chip_GPIO_WritePortBit(LPC_GPIO, 0, 22, (rest_pattern & 1) ? true : false);

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;
    rest_pattern >>= 1;
}

/** Fault handler for assert.
 * @param pattern LED pattern for error
 */
void diewith(uint32_t pattern)
{
    vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}


/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* setup clock */
    Chip_SetupXtalClocking();
    Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);

    /* enable GPIO */
    Chip_GPIO_Init(LPC_GPIO);

    /* enable pinmux */
    Chip_IOCON_Init(LPC_IOCON);

    /* setup pinmuxing */
    Chip_IOCON_SetPinMuxing(LPC_IOCON, pinmuxing,
                            sizeof(pinmuxing) / sizeof(PINMUX_GRP_T));

    /* Blinker timer initialization */
    Chip_TIMER_Init(LPC_TIMER3);
    uint32_t timerFreq = Chip_Clock_GetSystemClockRate();
    Chip_TIMER_Reset(LPC_TIMER3);
    Chip_TIMER_MatchEnableInt(LPC_TIMER3, 1);
    Chip_TIMER_SetMatch(LPC_TIMER3, 1, (timerFreq / 11));
    Chip_TIMER_ResetOnMatchEnable(LPC_TIMER3, 1);
    Chip_TIMER_Enable(LPC_TIMER3);
    NVIC_ClearPendingIRQ(TIMER3_IRQn);
    NVIC_EnableIRQ(TIMER3_IRQn);
}

} /* extern "C" */
