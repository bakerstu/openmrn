/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * This file represents the hardware initialization for the STM32G0B1RE Nucelo
 * board (bare).
 *
 * @author Balazs Racz & Brian Barnt
 * @date August 26, 2023
 */

#include <new>
#include <cstdint>

#include "stm32g0xx_hal_conf.h"
#include "stm32g0xx_hal.h"

#include "os/OS.hxx"
#include "Stm32Uart.hxx"
#include "Stm32Can.hxx"
#include "Stm32EEPROMEmulation.hxx"
#include "hardware.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart0("/dev/ser0", USART2, USART2_LPUART2_IRQn);

/** CAN 0 CAN driver instance */
//static Stm32Can can0("/dev/can0");
// todo: Need FDCAN driver here...

/** EEPROM emulation driver. The file size might be made bigger. */
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 512);

const size_t EEPROMEmulation::SECTOR_SIZE = 2048;

extern "C" {

/** Blink LED */
uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void hw_set_to_safe(void)
{
}

void reboot()
{
    NVIC_SystemReset();
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


void diewith(uint32_t pattern)
{
    // vPortClearInterruptMask(0x20);
    asm("cpsie i\n");

    resetblink(pattern);
    while (1)
        ;
}

/** CPU clock speed. */
const unsigned long cm0p_cpu_clock_hz = 16000000UL;
uint32_t SystemCoreClock;
const uint32_t AHBPrescTable[16] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL, 6UL, 7UL, 8UL, 9UL};
const uint32_t APBPrescTable[8]  = {0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL};
const uint32_t HSEValue = 8000000UL;
//const uint32_t HSIValue = 16000000UL;

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 16000000
  *            HCLK(Hz)                       = 16000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSI Frequency(Hz)              = 16000000
  *            HSI PREDIV                     = 1
  *            PLLMUL                         = 9
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void clock_setup(void)
{
    HAL_RCC_DeInit();
    
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

/** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

    HAL_RCC_OscConfig(&RCC_OscInitStruct); 
    	
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    HASSERT(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) == HAL_OK);

    // This will fail if the clocks are somehow misconfigured.
    HASSERT(SystemCoreClock == cm0p_cpu_clock_hz);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
     /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 72 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

    /* setup the system clock */
    clock_setup();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART2 pinmux on PA2 and PA3 */
    gpio_init.Mode =        GPIO_MODE_AF_PP;
    gpio_init.Pull =        GPIO_NOPULL;
    gpio_init.Speed =       GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate =   GPIO_AF1_USART2;
    gpio_init.Pin =         GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin =         GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PC4 and PC5 */
    gpio_init.Mode =        GPIO_MODE_AF_PP;
    gpio_init.Pull =        GPIO_NOPULL;
    gpio_init.Speed =       GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate =   GPIO_AF3_FDCAN1;
    gpio_init.Pin =         GPIO_PIN_4;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    gpio_init.Pin =         GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    GpioInit::hw_init();

    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance =            TIM17;
    TimHandle.Init.Period =         configCPU_CLOCK_HZ / 10000 / 8;
    TimHandle.Init.Prescaler =      10000;
    TimHandle.Init.ClockDivision =  0;
    TimHandle.Init.CounterMode =    TIM_COUNTERMODE_UP;
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
    __HAL_DBGMCU_FREEZE_TIM17();
    SetInterruptPriority(TIM17_FDCAN_IT1_IRQn, 0);
    NVIC_EnableIRQ(TIM17_FDCAN_IT1_IRQn);
}

void usart2_interrupt_handler(void)
{
    Stm32Uart::interrupt_handler(1);
}

// timer17_interrupt_handler()
//
// The timer 17 handler is not called via the nvic table since
// it is shared with the fdcan_it1 handler.
//
void timer17_interrupt_handler(void)
{
  // Check for a timer 17 interrupt
    if ((TIM17->SR & TIM_IT_UPDATE) != 0)
    {
        //
        // Clear the timer interrupt.
        //
        TIM17->SR = ~TIM_IT_UPDATE;

        // Set output LED.
        BLINKER_RAW_Pin::set(rest_pattern & 1);

        // Shift and maybe reset pattern.
        rest_pattern >>= 1;
        if (!rest_pattern)
        {
            rest_pattern = blinker_pattern;
        }
    }  // nope, not timer 17

}  // ~timer17_interrupt_handler

// timer17_fdcan_it1_interrupt_handler()
//
// the timer17/fdcan_it1 isr is invoked from the interrupt vector table.  It 
// in turn calls each of the shared isr routines: timer17, fdcan1_it1 and 
// fdcan2_it2.
//

void timer17_fdcan_it1_interrupt_handler(void)
{
    timer17_interrupt_handler();
    // todo: Fix fdcan Instances...
//    Stm32Can::instances[0]->rx_interrupt_handler();
//    Stm32Can::instances[0]->tx_interrupt_handler();
 //   Stm32Can::instances[1]->rx_interrupt_handler();
 //   Stm32Can::instances[1]->tx_interrupt_handler();
  
}  // ~timer17_fdcan_it1_interrupt_handler()

}  // end of extern "C"
