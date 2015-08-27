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

#include "stm32f0xx_hal_rcc.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_gpio_ex.h"

#include "os/OS.hxx"
#include "Stm32Uart.hxx"
#include "Stm32Can.hxx"
#include "hardware.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart0("/dev/ser0", USART1, USART1_IRQn);

/** CAN 0 CAN driver instance */
static Stm32Can can0("/dev/can0");

extern "C" {

/** Blink LED */
uint32_t blinker_pattern = 0;
//static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
}

void resetblink(uint32_t pattern)
{
    BLINKER_RAW_Pin::set(pattern ? true : false);
    blinker_pattern = pattern;
    /* make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void timer5a_interrupt_handler(void)
{
#if 0
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
#endif
}

void diewith(uint32_t pattern)
{
    //vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** Setup the system clock */
static void clock_setup(void)
{
    /* reset clock configuration to default state */
    RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

#define USE_EXTERNAL_8_MHz_CLOCK_SOURCE 0
    /* configure PLL:  8 MHz * 6 = 48 MHz */
#if USE_EXTERNAL_8_MHz_CLOCK_SOURCE
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_SW_HSE;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE));
#else
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_SW_HSI;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI));
#endif
    /* enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    /* set PLL as system clock */
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL));
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 48 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

    /* setup the system clock */
    clock_setup();

    /* enable peripheral clocks */
    __GPIOA_CLK_ENABLE();
    __GPIOB_CLK_ENABLE();
    __GPIOC_CLK_ENABLE();
    __USART1_CLK_ENABLE();
    __CAN_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;

    /* USART1 pinmux on PA9 and PA10 */
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_HIGH;
    gpio_init.Alternate = GPIO_AF1_USART1;
    gpio_init.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin       = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode      = GPIO_MODE_AF_PP;
    gpio_init.Pull      = GPIO_PULLUP;
    gpio_init.Speed     = GPIO_SPEED_HIGH;
    gpio_init.Alternate = GPIO_AF4_CAN;
    gpio_init.Pin       = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin       = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    GpioInit::hw_init();
}

}
