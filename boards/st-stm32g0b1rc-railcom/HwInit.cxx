/** \copyright
 * Copyright (c) 2025, Balazs Racz
 * Copyright (c) 2023, Brian Barnt
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
 * This file represents the hardware initialization for the G0B1-based RailCom
 * board.
 *
 * @author Balazs Racz & Brian Barnt
 * @date 18 Jan 2025
 */

#include <new>
#include <cstdint>

#include "freertos_drivers/st/stm32g0xx_hal_conf.h"
#include "stm32g0xx_hal.h"

#include "os/OS.hxx"
#include "freertos_drivers/st/Stm32Uart.hxx"
#include "freertos_drivers/st/Stm32Can.hxx"
#include "freertos_drivers/st/Stm32EEPROMEmulation.hxx"
#include "freertos_drivers/st/Stm32SpiPixelStrip.hxx"
#include "freertos_drivers/st/Stm32MCan.hxx"
#include "hardware.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart3("/dev/ser0", USART3, USART3_4_5_6_LPUART1_IRQn);

/** CAN 0 CAN driver instance */
static Stm32MCan can0("/dev/can0", FDCAN1, TIM16_FDCAN_IT0_IRQn);

extern FDCAN_GlobalTypeDef* fdcan1;
FDCAN_GlobalTypeDef* fdcan1 = FDCAN1;


/** EEPROM emulation driver. The file size might be made bigger. */
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 512);

const size_t EEPROMEmulation::SECTOR_SIZE = 2048;

uint8_t pix[9*3] = {3, 3, 3};

Stm32SpiPixelStrip strip0(SPI1, 9, pix);


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
    pix[0] = rest_pattern & 1 ? 0x20 : 3;
    strip0.update_sync();
    /* make a timer event trigger immediately */
}

void setblink(uint32_t pattern)
{
    resetblink(pattern);
}

void blinker_interrupt_handler(void)
{
    // Check for a timer 17 interrupt
    if ((TIM17->SR & TIM_IT_UPDATE) != 0)
    {
        //
        // Clear the timer interrupt.
        //
        TIM17->SR = ~TIM_IT_UPDATE;

#if 0
        // Set output LED.
        BLINKER_RAW_Pin::set(rest_pattern & 1);
#else
        pix[0] = rest_pattern & 1 ? 0x20 : 3;
        strip0.update_sync();
#endif
        
        // Shift and maybe reset pattern.
        rest_pattern >>= 1;
        if (!rest_pattern)
        {
            rest_pattern = blinker_pattern;
        }
    }  // nope, not timer 17
}

void wait_with_blinker(void)
{
    while (1)
    {
        blinker_interrupt_handler();
    }
}

void diewith(uint32_t pattern)
{
    // vPortClearInterruptMask(0x20);
    asm("cpsid i\n");

    resetblink(pattern);
    wait_with_blinker();
}

/** CPU clock speed. */
uint32_t SystemCoreClock = 0;
const uint32_t AHBPrescTable[16] = {0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL, 6UL, 7UL, 8UL, 9UL};
const uint32_t APBPrescTable[8]  = {0UL, 0UL, 0UL, 0UL, 1UL, 2UL, 3UL, 4UL};
const uint32_t HSEValue = 16000000UL;
//const uint32_t HSIValue = 16000000UL;

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB Prescaler                  = 1
  *            HSE Frequency(Hz)              = 16000000
  *            PLLMUL (N)                     = 8
  *            P, Q, R DIV                    = 2
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
static void clock_setup(void)
{
    HAL_RCC_DeInit();
    
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;    
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HASSERT(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    HASSERT(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) == HAL_OK);

    // This will fail if the clocks are somehow misconfigured.
    HASSERT(SystemCoreClock == cpu_clock_hz);

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;

    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/// We don't need the HAL tick configuration code to run. FreeRTOS will take
/// care of that.
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
    return HAL_OK;
}

/** Initialize the processor hardware.
 */
void hw_preinit(void)
{
     /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    asm("cpsid i\n");

    /* these FLASH settings enable opertion at 64 MHz */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_2);

    /* setup the system clock */
    clock_setup();

    /* enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();

    __HAL_RCC_SPI1_CLK_ENABLE();    

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART3 pinmux on PC10 and PC11 */
    gpio_init.Mode =        GPIO_MODE_AF_PP;
    gpio_init.Pull =        GPIO_NOPULL;
    gpio_init.Speed =       GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate =   GPIO_AF0_USART3;
    gpio_init.Pin =         GPIO_PIN_10;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    gpio_init.Pin =         GPIO_PIN_11;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode =        GPIO_MODE_AF_PP;
    gpio_init.Pull =        GPIO_NOPULL;
    gpio_init.Speed =       GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate =   GPIO_AF3_FDCAN1;
    gpio_init.Pin =         GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin =         GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* SPI MOSI on PB5 */
    gpio_init.Mode =        GPIO_MODE_AF_PP;
    gpio_init.Pull =        GPIO_NOPULL;
    gpio_init.Speed =       GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate =   GPIO_AF0_SPI1;
    gpio_init.Pin =         GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    
    GpioInit::hw_init();

    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance =            TIM17;
    TimHandle.Init.Period =         configCPU_CLOCK_HZ / 10000 / 5;
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

    fdcan1 = FDCAN1;
}

void hw_init(void)
{
    strip0.hw_init(SPI_BAUDRATEPRESCALER_16);
    strip0.update_sync();

    can0.init(64000000, config_nmranet_can_bitrate(),
        (3 * config_nmranet_can_bitrate()) / 1000);
}

void uart2_lpuart2_interrupt_handler(void)
{
    // no instance.
}

void uart3_4_5_6_lpuart1_interrupt_handler(void)
{
    uart3.interrupt_handler();
}

void timer16_fdcan_it0_interrupt_handler(void)
{
    can0.interrupt_handler();
}

void timer17_fdcan_it1_interrupt_handler(void)
{
    blinker_interrupt_handler();
}

}  // end of extern "C"
