/** \copyright
 * Copyright (c) 2020, Balazs Racz
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
 * This file represents the hardware initialization for the STM32F091RC Nucelo
 * board (bare) with a DCC decoder driver.
 *
 * @author Balazs Racz
 * @date April 10, 2020
 */

#include <new>
#include <cstdint>

#include "stm32f0xx_hal_conf.h"
#include "stm32f0xx_hal_rcc.h"
#include "stm32f0xx_hal_flash.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_hal_gpio_ex.h"
#include "stm32f0xx_hal_dma.h"
#include "stm32f0xx_hal_tim.h"
#include "stm32f0xx_hal.h"

#include "os/OS.hxx"
#include "Stm32Uart.hxx"
#include "Stm32Can.hxx"
#include "Stm32EEPROMEmulation.hxx"
#include "Stm32RailcomSender.hxx"
#include "hardware.hxx"
#include "DummyGPIO.hxx"

struct Debug
{
    typedef DummyPin DccPacketDelay;
    typedef DummyPin DccDecodeInterrupts;
    typedef DummyPin DccPacketFinishedHook;
    typedef DummyPin CapTimerOverflow;
};

#include "Stm32DCCDecoder.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart2("/dev/ser0", USART2, USART2_IRQn);

/** RailCom sender UART */
static Stm32RailcomSender railcomUart("/dev/ser1", USART1, USART1_IRQn);

/** CAN 0 CAN driver instance */
static Stm32Can can0("/dev/can0");

/** EEPROM emulation driver. The file size might be made bigger. */
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 512);

const size_t EEPROMEmulation::SECTOR_SIZE = 2048;

struct DccDecoderHW
{
    /// The Pin (declared as GPIO input) where the DCC signal comes in.
    using NRZ_Pin = ::DCC_IN_Pin;
    /// Alternate Mode selector for the DCC_IN pin to put it into timer mode.
    static constexpr auto CAPTURE_AF_MODE = GPIO_AF1_TIM3;

    /// Takes an occupancy feedback sample every 3 milliseconds. The clock of
    /// the timer ticks every usec. This is only useful if there is a railcom
    /// driver that can also measure current used in forward mode.
    static constexpr uint32_t SAMPLE_PERIOD_TICKS = 3000;

    /// Length of the ring buffer for packets waiting for userspace to read.
    static constexpr unsigned Q_SIZE = 6;

    /// Defines the timer resource matching the Capture pin. It can be either a
    /// 16-bit or a 32-bit timer.
    static const auto CAPTURE_TIMER = TIM3_BASE;
    /// Which channel of the timer we should be capturing on. Defined by the
    /// Capture pin.
    static constexpr uint32_t CAPTURE_CHANNEL = TIM_CHANNEL_1;
    /// Interrupt flag for the given capture channel.
    static constexpr auto CAPTURE_IF = TIM_FLAG_CC1;
    /// Digital filter to apply to the captured stream for edge detection. The
    /// 0b1000 value needs 6 consecutive samples at f_CLK/8 to be the same
    /// value to trigger the edge detection. This is about 1 usec at 48 MHz.
    static constexpr unsigned CAPTURE_FILTER = 0b1000;
    /// Interrupt vector number for the capture timer resource.
    static constexpr auto CAPTURE_IRQn = TIM3_IRQn;

    /// Hook called in a P0 interrupt context every edge.
    static void cap_event_hook() {}
    /// Hook called in a P0 interrupt context before the DCC cutout is enabled.
    static inline void dcc_before_cutout_hook() {}
    /// Hook called in a P0 interrupt context when a full DCC packet is
    /// received. This is after the packet ending one bit, or after the cutout.
    static inline void dcc_packet_finished_hook() {}
    /// Hook called in a P0 interrupt context after we instructed the railcom
    /// driver to take a feedback sample.
    static inline void after_feedback_hook() {}

    /// How many usec later/earlier should the railcom cutout start happen.
    static int time_delta_railcom_pre_usec()
    {
        return 80 - 26;
    }

    /// How many usec later/earlier should the railcom cutout middle happen.
    static int time_delta_railcom_mid_usec()
    {
        return 193 - 185;
    }

    /// How many usec later/earlier should the railcom cutout end happen.
    static int time_delta_railcom_end_usec()
    {
        return 0;
    }
    
    /// Second timer resource that will be used to measure microseconds for the
    /// railcom cutout. May be the same as the Capture Timer, if there are at
    /// least two channels on that timer resource.
    static const auto USEC_TIMER = TIM3_BASE;
    /// Channel to use for the timing purpose. This channel shall NOT be
    /// connected to a pin.
    static constexpr uint32_t USEC_CHANNEL = TIM_CHANNEL_2;
    /// Interrupt flag for the USEC_CHANNEL.
    static constexpr auto USEC_IF = TIM_FLAG_CC2;
    static_assert(TIM_FLAG_CC2 == TIM_IT_CC2,
        "Flag and interrupt registers must be in parallel. The HAL driver is "
        "broken.");
    /// Interrupt vector number for the usec timer resource.
    static constexpr auto TIMER_IRQn = TIM3_IRQn;

    /// An otherwise unused interrupt vector number, which can be used as a
    /// software interrupt in a kernel-compatible way.
    static constexpr auto OS_IRQn = TSC_IRQn;
};

Stm32DccDecoder<DccDecoderHW> dcc_decoder0(
    "/dev/dcc_decoder0", &railcomUart);

extern "C" {

void set_dcc_interrupt_processor(dcc::PacketProcessor *p)
{
    dcc_decoder0.set_packet_processor(p);
}

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
}

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    rest_pattern = pattern ? 1 : 0;
    BLINKER_RAW_Pin::set(pattern ? true : false);
    /* make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}


const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};
const uint32_t HSEValue = 8000000;


void timer14_interrupt_handler(void)
{
    //
    // Clear the timer interrupt.
    //
    TIM14->SR = ~TIM_IT_UPDATE;

    // Set output LED.
    BLINKER_RAW_Pin::set(rest_pattern & 1);

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
    {
        rest_pattern = blinker_pattern;
    }
}

void diewith(uint32_t pattern)
{
    // vPortClearInterruptMask(0x20);
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
    while (!(RCC->CR & RCC_CR_HSIRDY))
        ;

#define USE_EXTERNAL_8_MHz_CLOCK_SOURCE 1
/* configure PLL:  8 MHz * 6 = 48 MHz */
#if USE_EXTERNAL_8_MHz_CLOCK_SOURCE
    RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
    while (!(RCC->CR & RCC_CR_HSERDY))
        ;
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_SW_HSE;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE))
        ;
#else
    RCC->CFGR = RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLSRC_HSI_PREDIV | RCC_CFGR_SW_HSI;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI))
        ;
#endif
    /* enable PLL */
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY))
        ;

    /* set PLL as system clock */
    RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL;
    while (!((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL))
        ;
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART2 pinmux on PA2 and PA3 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF1_USART2;
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* USART1 pinmux on railCom TX pin PB6 with open drain and pullup */
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF0_USART1;
    gpio_init.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    
    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF4_CAN;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    GpioInit::hw_init();

    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = TIM14;
    TimHandle.Init.Period = configCPU_CLOCK_HZ / 10000 / 8;
    TimHandle.Init.Prescaler = 10000;
    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    TimHandle.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    {
        /* Initialization Error */
        HASSERT(0);
    }
    if (HAL_TIM_Base_Start_IT(&TimHandle) != HAL_OK)
    {
        /* Starting Error */
        HASSERT(0);
    }
    __HAL_DBGMCU_FREEZE_TIM14();
    SetInterruptPriority(TIM14_IRQn, 0);
    NVIC_EnableIRQ(TIM14_IRQn);
}

void timer3_interrupt_handler(void) {
    // Both the capture and the timer feature use the same timer resource, so
    // the interrupt is shared. These function calls are a noop if the
    // respective IF bit is not set in the interrupt status register.
    dcc_decoder0.interrupt_handler();
    dcc_decoder0.rcom_interrupt_handler();
}

void touch_interrupt_handler(void) {
    dcc_decoder0.os_interrupt_handler();
    portYIELD_FROM_ISR(true);
}

/// UART2 interrupt handler.
void uart2_interrupt_handler(void)
{
    uart2.interrupt_handler();
}

}
