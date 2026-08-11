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
 * This file represents the hardware initialization for the STM32F303RE Nucelo
 * board with the DevKit IO board plugged in.
 *
 * @author Balazs Racz
 * @date April 18, 2018
 */

#define _DEFAULT_SOURCE

#include <new>
#include <cstdint>

#include "freertos_drivers/st/stm32f3xx_hal_conf.h"
#include "stm32f3xx_hal.h"

#include "os/OS.hxx"
#include "freertos_drivers/st/Stm32Uart.hxx"
#include "freertos_drivers/st/Stm32Can.hxx"
#include "freertos_drivers/st/Stm32SPI.hxx"
#include "freertos_drivers/st/Stm32I2C.hxx"
#include "freertos_drivers/st/Stm32EEPROMEmulation.hxx"
#include "freertos_drivers/st/Stm32PWM.hxx"
#include "hardware.hxx"

/** override stdin */
const char *STDIN_DEVICE = "/dev/ser0";

/** override stdout */
const char *STDOUT_DEVICE = "/dev/ser0";

/** override stderr */
const char *STDERR_DEVICE = "/dev/ser0";

/** UART 0 serial driver instance */
static Stm32Uart uart2("/dev/ser0", USART2, USART2_IRQn);

/** CAN 0 CAN driver instance */
static Stm32Can can0("/dev/can0");

/** EEPROM emulation driver. The file size might be made bigger. */
static Stm32EEPROMEmulation eeprom0("/dev/eeprom", 8192);
// originally 4000

/** UART 0 serial driver instance */
static Stm32I2C i2c1("/dev/i2c0", I2C1, I2C1_EV_IRQn, I2C1_ER_IRQn);

/** How many bytes of flash should hold the entire dataset. Must be an integer
 * multiple of the minimum erase length (which is the flash page length, for
 * the STM32F0 | STM32F3 it is 2 kbytes). The file size maximum is half this
 * value. */
const size_t EEPROMEmulation::SECTOR_SIZE = 16384;

Stm32PWMGroup servo_timer(TIM3, (configCPU_CLOCK_HZ * 6 / 1000 + 65535) / 65536,
                          configCPU_CLOCK_HZ * 6 / 1000);

extern PWM* const servo_channels[];
/// The order of these channels follows the schematic arrangement of MCU pins
/// to logical servo ports.
PWM * const servo_channels[4] = { //
    Stm32PWMGroup::get_channel(&servo_timer, 4),
    Stm32PWMGroup::get_channel(&servo_timer, 2),
    Stm32PWMGroup::get_channel(&servo_timer, 3),
    Stm32PWMGroup::get_channel(&servo_timer, 1)};

/// Recursive mutex for SPI1 peripheral.
OSMutex spi1_lock(true);

static void noop_cs() {
}

/// SPI1 driver for io-board peripherals
static Stm32SPI spi1_0(
    "/dev/spi1.ioboard", SPI1, SPI1_IRQn, noop_cs, noop_cs, &spi1_lock);

static void spi1_ext_cs_assert() {
    EXT_CS_Pin::set(false);
}

static void spi1_ext_cs_deassert() {
    EXT_CS_Pin::set(true);
}

/// SPI1 driver for the expansion port.
static Stm32SPI spi1_1("/dev/spi1.ext", SPI1, SPI1_IRQn, &spi1_ext_cs_assert,
    &spi1_ext_cs_deassert, &spi1_lock);

/// SPI2 driver for the onboard input ports.
static Stm32SPI spi2("/dev/spi2", SPI2, SPI2_IRQn, &noop_cs, &noop_cs, nullptr);

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

void i2c1_ev_interrupt_handler(void)
{
    i2c1.event_interrupt_handler();
}

void i2c1_er_interrupt_handler(void)
{
    i2c1.error_interrupt_handler();
}

/// TIM17 shares this interrupt with certain features of timer1
void tim1_trg_com_interrupt_handler(void)
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
const unsigned long cm3_cpu_clock_hz = 72000000;
uint32_t SystemCoreClock;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8]  = {0, 0, 0, 0, 1, 2, 3, 4};

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 8000000
  *            HSE PREDIV                     = 1
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

    /* Enable HSE Oscillator and activate PLL with HSE as source on bypass
     * mode. This allows using the MCO clock output from the ST_Link part of
     * the nucleo board and freeing up the other clock pin for GPIO. */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

    HAL_RCC_OscConfig(&RCC_OscInitStruct); 
    	
    /* Select PLL as system clock source and configure the HCLK, PCLK1 and
     * PCLK2 clocks dividers
     */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;  
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    HASSERT(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) == HAL_OK);

    // This will fail if the clocks are somehow misconfigured.
    HASSERT(SystemCoreClock == cm3_cpu_clock_hz);
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
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* setup pinmux */
    GPIO_InitTypeDef gpio_init;
    memset(&gpio_init, 0, sizeof(gpio_init));

    /* USART2 pinmux on PA2 and PA3 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF7_USART2;
    gpio_init.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    gpio_init.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    /* CAN pinmux on PB8 and PB9 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    // Disables pull-ups because this is a 5V tolerant pin.
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF9_CAN;
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* SPI1 pinmux on PB3, PB4, and PB5 */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF5_SPI1;
    gpio_init.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* SPI2 pinmux on PB13 (SCK), and PB14 (MISO2) */
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF5_SPI2;
    gpio_init.Pin = GPIO_PIN_14;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    /* I2C1 pinmux on PB6 (SCL), and PB7 (SDA) */
    gpio_init.Mode = GPIO_MODE_AF_OD;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    gpio_init.Alternate = GPIO_AF4_I2C1;
    gpio_init.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOB, &gpio_init);
    gpio_init.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio_init);

    GpioInit::hw_init();

    // Switches over servo timer pins to timer mode.
    // PC6-7-8-9
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init.Alternate = GPIO_AF2_TIM3;
    gpio_init.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    gpio_init.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    gpio_init.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    gpio_init.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio_init);
    
    /* Initializes the blinker timer. */
    TIM_HandleTypeDef TimHandle;
    memset(&TimHandle, 0, sizeof(TimHandle));
    TimHandle.Instance = TIM17;
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
    __HAL_DBGMCU_FREEZE_TIM17();
    SetInterruptPriority(TIM17_IRQn, 0);
    NVIC_EnableIRQ(TIM17_IRQn);
}

void usart2_interrupt_handler(void)
{
    uart2.interrupt_handler();
}

}
