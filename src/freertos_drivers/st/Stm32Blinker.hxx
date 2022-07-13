/** \copyright
 * Copyright (c) 2022, Balazs Racz
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
 * \file Stm32Blinker.hxx
 * Implements the crash blinker using an STM32 timer.
 *
 * @author Balazs Racz
 * @date 10 April 2022
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32BLINKER_HXX_
#define _FREERTOS_DRIVERS_ST_STM32BLINKER_HXX_

/* How to use:

Add the following code to the hardware.hxx
```
    #define BLINKER_INTERRUPT_HANDLER timer14_interrupt_handler

    struct BlinkerHw
    {
        static constexpr uint32_t TIMER_BASE = TIM14_BASE;
        static constexpr auto TIMER_IRQn = TIM14_IRQn; 
        static void clock_enable()
        {
            __HAL_RCC_TIM14_CLK_ENABLE();
        }
    };
```

then
```
#include "freertos_drivers/st/Stm32Blinker.hxx"
```

in HwInit.cxx after hardware.hxx was included. Call setup_blinker() in
hw_preinit.
*/

#ifndef BLINKER_INTERRUPT_HANDLER
#error must include hardware.hxx before Stm32Blinker.hxx
#endif

/// @return constexpr timer instance typecast to the correct struct pointer.
static inline TIM_TypeDef *get_blinker_timer()
{
    return (TIM_TypeDef *)BlinkerHw::TIMER_BASE;
}

extern "C" {

/// Call this function in hw_preinit to set up the blinker timer.
void setup_blinker()
{
    BlinkerHw::clock_enable();
    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = get_blinker_timer();
    TimHandle.Init.Period = configCPU_CLOCK_HZ / 10000 / 5;
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
    NVIC_SetPriority(BlinkerHw::TIMER_IRQn, 0);
    NVIC_EnableIRQ(BlinkerHw::TIMER_IRQn);
}

/// Stores the canonical pattern for the blinker.
uint32_t blinker_pattern = 0;
/// Stores what is left of the pattern during the current period.
static uint32_t rest_pattern = 0;

extern void hw_set_to_safe(void);

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    rest_pattern = pattern ? 1 : 0;
    BLINKER_RAW_Pin::set(pattern ? true : false);
    /* todo: make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void BLINKER_INTERRUPT_HANDLER(void)
{
    //
    // Clear the timer interrupt.
    //
    get_blinker_timer()->SR = ~TIM_IT_UPDATE;

    // Set output LED.
    BLINKER_RAW_Pin::set(rest_pattern & 1);

    // Shift and maybe reset pattern.
    rest_pattern >>= 1;
    if (!rest_pattern)
    {
        rest_pattern = blinker_pattern;
    }
}

void wait_with_blinker(void)
{
    if (get_blinker_timer()->SR & TIM_IT_UPDATE)
    {
        BLINKER_INTERRUPT_HANDLER();
    }
}

void diewith(uint32_t pattern)
{
    asm("cpsid i\n");
    hw_set_to_safe();
    resetblink(pattern);
    while (1)
    {
        wait_with_blinker();
    }
}

} // extern "C"

#endif // _FREERTOS_DRIVERS_ST_STM32BLINKER_HXX_
