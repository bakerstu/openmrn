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

#include "freertos_drivers/st/stm32f_hal_conf.hxx"
#include "stm32f1xx.h"

#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_rcc_ex.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_gpio_ex.h"
#include "stm32f1xx_hal_flash.h"

#include "os/OS.hxx"
//#include "Stm32F0xxUart.hxx"
#include "freertos_drivers/st/Stm32Can.hxx"
#include "freertos_drivers/st/Stm32Gpio.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
// static Stm32Uart uart1("/dev/ser0", USART1, USART1_IRQn);

/** CAN 0 CAN driver instance */
static Stm32Can can0("/dev/can0");

GPIO_PIN(LED1, LedPin, A, 5);
GPIO_PIN(LED2, LedPin, A, 1);

static const Gpio* blinker_gpio = LED1_Pin::instance();

extern "C" {

const unsigned long cm3_cpu_clock_hz = 72000000;

/** Blink LED */
uint32_t blinker_pattern = 0;
// static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
}

void resetblink(uint32_t pattern)
{
    //LED2_Pin::set(!!pattern);
    blinker_gpio->write(!!pattern);
    //GPIOA->BSRR = pattern ? (1 << 1) : (1 << 17);
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
    // vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

// /** Setup the system clock */
// static void clock_setup(void)
// {
//     /* reset clock configuration to default state */
//     RCC->CR = RCC_CR_HSITRIM_4 | RCC_CR_HSION;
//     while (!(RCC->CR & RCC_CR_HSIRDY));

// #define USE_EXTERNAL_8_MHz_CLOCK_SOURCE 0
//     /* configure PLL:  8 MHz * 6 = 48 MHz */
// #if USE_EXTERNAL_8_MHz_CLOCK_SOURCE
//     RCC->CR |= RCC_CR_HSEON;
//     while (!(RCC->CR & RCC_CR_HSERDY));
//     RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV |
// RCC_CFGR_SW_HSE;
//     while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE));
// #else
//     RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI_PREDIV |
// RCC_CFGR_SW_HSI;
//     while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI));
// #endif
//     /* enable PLL */
//     RCC->CR |= RCC_CR_PLLON;
//     while (!(RCC->CR & RCC_CR_PLLRDY));

//     /* set PLL as system clock */
//     RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
//     while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL));
// }

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    // Resets USB module in case we are coming from the bootloader
    __HAL_RCC_USB_FORCE_RESET();
    for (volatile int i = 0; i < 10; ++i)
        ;
    __HAL_RCC_USB_CLK_DISABLE();
    __HAL_RCC_USB_RELEASE_RESET();

    SystemInit();

    extern void (*const __interrupt_vector[])(void);
    SCB->VTOR = (uint32_t) & __interrupt_vector;

    // Switches to internal clock while we reconfigure the external oscillator.
    HAL_RCC_DeInit();
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};

    /* Configure PLL ------------------------------------------------------*/
    /* Assumes 8 MHz ext clock. */
    /* PLL configuration: PLLCLK = (HSE) * PLLMUL = (8) * 9 = 72 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 72 / 1 = 72
     * MHz */
    /* Enable HSE and activate PLL with HSE as source */
    oscinitstruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscinitstruct.HSEState = RCC_HSE_ON;
    oscinitstruct.LSEState = RCC_LSE_OFF;
    oscinitstruct.HSIState = RCC_HSI_ON;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscinitstruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&oscinitstruct) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                               RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2) != HAL_OK)
    {
        /* Initialization Error */
        while (1)
            ;
    }

    /* these FLASH settings enable opertion at 48 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    //__USART1_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    /* setup pinmux */
    GPIO_InitTypeDef gpio_init = {0};

    /* Setup output LEDs */
    gpio_init.Pin = GPIO_PIN_1 | GPIO_PIN_5;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* USART1 pinmux on PA9 and PA10 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init.Alternate = GPIO_AF1_USART1;
    gpio_init.Pin = GPIO_PIN_9;
    // HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_10;
    // HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    __HAL_AFIO_REMAP_CAN1_2();
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    // gpio_init.Alternate = GPIO_AF4_CAN;
    gpio_init.Mode = GPIO_MODE_AF_INPUT;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    LED1_Pin::hw_init();
    LED2_Pin::hw_init();
}

/// UART1 interrupt handler.
void usart1_interrupt_handler(void)
{
    // uart1.interrupt_handler();
}

}
