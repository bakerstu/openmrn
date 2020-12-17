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
 * \file startup.c
 * This file sets up the runtime environment for ST STM32F3 MCUs.
 *
 * @author Stuart W. Baker
 * @date 25 August 2015
 */

#include <stdint.h>

#include "FreeRTOSConfig.h"

/* prototypes */
extern unsigned long *__stack;
extern void reset_handler(void);
static void nmi_handler(void);
static void hard_fault_handler(void);
static void mpu_fault_handler(void);
static void bus_fault_handler(void);
static void usage_fault_handler(void);

extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void __libc_init_array(void);

extern int main(int argc, char *argv[]);

extern void debug_interrupt_handler(void);
extern void wwdg_interrupt_handler(void);
extern void pvd_pvm_interrupt_handler(void);
extern void tamp_stamp_interrupt_handler(void);
extern void rtc_wkup_interrupt_handler(void);
extern void flash_interrupt_handler(void);
extern void rcc_interrupt_handler(void);
extern void exti0_interrupt_handler(void);
extern void exti1_interrupt_handler(void);
extern void exti2_interrupt_handler(void);
extern void exti3_interrupt_handler(void);
extern void exti4_interrupt_handler(void);
extern void dma1_channel1_interrupt_handler(void);
extern void dma1_channel2_interrupt_handler(void);
extern void dma1_channel3_interrupt_handler(void);
extern void dma1_channel4_interrupt_handler(void);
extern void dma1_channel5_interrupt_handler(void);
extern void dma1_channel6_interrupt_handler(void);
extern void dma1_channel7_interrupt_handler(void);
extern void adc1_interrupt_handler(void);
extern void can1_tx_interrupt_handler(void);
extern void can1_rx0_interrupt_handler(void);
extern void can1_rx1_interrupt_handler(void);
extern void can1_sce_interrupt_handler(void);
extern void exti9_5_interrupt_handler(void);
extern void tim1_brk_tim15_interrupt_handler(void);
extern void tim1_up_tim16_interrupt_handler(void);
extern void tim1_trg_com_interrupt_handler(void);
extern void tim1_cc_interrupt_handler(void);
extern void tim2_interrupt_handler(void);
extern void tim3_interrupt_handler(void);
extern void i2c1_ev_interrupt_handler(void);
extern void i2c1_er_interrupt_handler(void);
extern void i2c2_ev_interrupt_handler(void);
extern void i2c2_er_interrupt_handler(void);
extern void spi1_interrupt_handler(void);
extern void spi2_interrupt_handler(void);
extern void usart1_interrupt_handler(void);
extern void usart2_interrupt_handler(void);
extern void usart3_interrupt_handler(void);
extern void exti15_10_interrupt_handler(void);
extern void rtc_alarm_interrupt_handler(void);
extern void sdmmc1_interrupt_handler(void);
extern void spi3_interrupt_handler(void);
extern void uart4_interrupt_handler(void);
extern void tim6_dac_interrupt_handler(void);
extern void tim7_interrupt_handler(void);
extern void dma2_channel1_interrupt_handler(void);
extern void dma2_channel2_interrupt_handler(void);
extern void dma2_channel3_interrupt_handler(void);
extern void dma2_channel4_interrupt_handler(void);
extern void dma2_channel5_interrupt_handler(void);
extern void dfsdm1_flt0_interrupt_handler(void);
extern void dfsdm1_flt1_interrupt_handler(void);
extern void comp_interrupt_handler(void);
extern void lptim1_interrupt_handler(void);
extern void lptim2_interrupt_handler(void);
extern void usb_interrupt_handler(void);
extern void dma2_channel6_interrupt_handler(void);
extern void dma2_channel7_interrupt_handler(void);
extern void lpuart1_interrupt_handler(void);
extern void quadspi_interrupt_handler(void);
extern void i2c3_ev_interrupt_handler(void);
extern void i2c3_er_interrupt_handler(void);
extern void sai1_interrupt_handler(void);
extern void swpmi1_interrupt_handler(void);
extern void tsc_interrupt_handler(void);
extern void aes_interrupt_handler(void);
extern void rng_interrupt_handler(void);
extern void fpu_interrupt_handler(void);
extern void crs_interrupt_handler(void);
extern void i2c4_ev_interrupt_handler(void);
extern void i2c4_er_interrupt_handler(void);
extern void ignore_fn(void);

extern const unsigned long cm3_cpu_clock_hz;

/** Exception table */
__attribute__((
    section(".interrupt_vector"))) void (*const __interrupt_vector[])(void) = {
    (void (*)(void))(&__stack),   /**<  0 initial stack pointer */
    reset_handler,                     /**<  1 reset vector */
    nmi_handler,                       /**<  2 non-maskable interrupt */
    hard_fault_handler,                /**<  3 hard fault */
    mpu_fault_handler,                 /**<  4 reserved */
    bus_fault_handler,                 /**<  5 reserved */
    usage_fault_handler,               /**<  6 reserved */
    0,                                 /**<  7 reserved */
    0,                                 /**<  8 reserved */
    0,                                 /**<  9 reserved */
    0,                                 /**< 10 reserved */
    SVC_Handler,                       /**< 11 SV call */
    debug_interrupt_handler,           /**< 12 reserved */
    0,                                 /**< 13 reserved -- bootloader appentry */
    PendSV_Handler,                    /**< 14 pend SV */
    SysTick_Handler,                   /**< 15 system tick */
    wwdg_interrupt_handler,            /**< 0 Window WatchDog Interrupt */
    pvd_pvm_interrupt_handler,         /**< 1 PVD/PVM1/PVM3/PVM4 through EXTI Line
                                          detection Interrupts */
    tamp_stamp_interrupt_handler,      /**< 2 Tamper and TimeStamp interrupts through
                                          the EXTI line */
    rtc_wkup_interrupt_handler,        /**< 3 RTC Wakeup interrupt through the EXTI
                                          line */
    flash_interrupt_handler,           /**< 4 FLASH global Interrupt */
    rcc_interrupt_handler,             /**< 5 RCC global Interrupt */
    exti0_interrupt_handler,           /**< 6 EXTI Line0 Interrupt */
    exti1_interrupt_handler,           /**< 7 EXTI Line1 Interrupt */
    exti2_interrupt_handler,           /**< 8 EXTI Line2 Interrupt */
    exti3_interrupt_handler,           /**< 9 EXTI Line3 Interrupt */
    exti4_interrupt_handler,           /**< 10 EXTI Line4 Interrupt */
    dma1_channel1_interrupt_handler,   /**< 11 DMA1 Channel 1 global Interrupt */
    dma1_channel2_interrupt_handler,   /**< 12 DMA1 Channel 2 global Interrupt */
    dma1_channel3_interrupt_handler,   /**< 13 DMA1 Channel 3 global Interrupt */
    dma1_channel4_interrupt_handler,   /**< 14 DMA1 Channel 4 global Interrupt */
    dma1_channel5_interrupt_handler,   /**< 15 DMA1 Channel 5 global Interrupt */
    dma1_channel6_interrupt_handler,   /**< 16 DMA1 Channel 6 global Interrupt */
    dma1_channel7_interrupt_handler,   /**< 17 DMA1 Channel 7 global Interrupt */
    adc1_interrupt_handler,            /**< 18 ADC1 global Interrupt */
    can1_tx_interrupt_handler,         /**< 19 CAN1 TX Interrupt */
    can1_rx0_interrupt_handler,        /**< 20 CAN1 RX0 Interrupt */
    can1_rx1_interrupt_handler,        /**< 21 CAN1 RX1 Interrupt */
    can1_sce_interrupt_handler,        /**< 22 CAN1 SCE Interrupt */
    exti9_5_interrupt_handler,         /**< 23 External Line[9:5] Interrupts */
    tim1_brk_tim15_interrupt_handler,  /**< 24 TIM1 Break interrupt and TIM15
                                          global interrupt */
    tim1_up_tim16_interrupt_handler,   /**< 25 TIM1 Update Interrupt and TIM16
                                          global interrupt */
    tim1_trg_com_interrupt_handler,    /**< 26 TIM1 Trigger and Commutation
                                          Interrupt */
    tim1_cc_interrupt_handler,         /**< 27 TIM1 Capture Compare Interrupt */
    tim2_interrupt_handler,            /**< 28 TIM2 global Interrupt */
    tim3_interrupt_handler,            /**< 29 TIM3 global Interrupt */
    0,                                 /**< 30 */
    i2c1_ev_interrupt_handler,         /**< 31 I2C1 Event Interrupt */
    i2c1_er_interrupt_handler,         /**< 32 I2C1 Error Interrupt */
    i2c2_ev_interrupt_handler,         /**< 33 I2C2 Event Interrupt */
    i2c2_er_interrupt_handler,         /**< 34 I2C2 Error Interrupt */
    spi1_interrupt_handler,            /**< 35 SPI1 global Interrupt */
    spi2_interrupt_handler,            /**< 36 SPI2 global Interrupt */
    usart1_interrupt_handler,          /**< 37 USART1 global Interrupt */
    usart2_interrupt_handler,          /**< 38 USART2 global Interrupt */
    usart3_interrupt_handler,          /**< 39 USART3 global Interrupt */
    exti15_10_interrupt_handler,       /**< 40 External Line[15:10] Interrupts */
    rtc_alarm_interrupt_handler,       /**< 41 RTC Alarm (A and B) through EXTI Line
                                         Interrupt */
    0,                                 /**< 42 */
    0,                                 /**< 43 */
    0,                                 /**< 44 */
    0,                                 /**< 45 */
    0,                                 /**< 46 */
    0,                                 /**< 47 */
    0,                                 /**< 48 */
    sdmmc1_interrupt_handler,          /**< 49 SDMMC1 global Interrupt */
    0,                                 /**< 50 */
    spi3_interrupt_handler,            /**< 51 SPI3 global Interrupt */
    uart4_interrupt_handler,           /**< 52 UART4 global Interrupt */
    0,                                 /**< 53 */
    tim6_dac_interrupt_handler,        /**< 54 TIM6 global and DAC1&2 underrun error
                                          interrupts */
    tim7_interrupt_handler,            /**< 55 TIM7 global interrupt */
    dma2_channel1_interrupt_handler,   /**< 56 DMA2 Channel 1 global Interrupt */
    dma2_channel2_interrupt_handler,   /**< 57 DMA2 Channel 2 global Interrupt */
    dma2_channel3_interrupt_handler,   /**< 58 DMA2 Channel 3 global Interrupt */
    dma2_channel4_interrupt_handler,   /**< 59 DMA2 Channel 4 global Interrupt */
    dma2_channel5_interrupt_handler,   /**< 60 DMA2 Channel 5 global Interrupt */
    dfsdm1_flt0_interrupt_handler,     /**< 61 DFSDM1 Filter 0 global Interrupt */
    dfsdm1_flt1_interrupt_handler,     /**< 62 DFSDM1 Filter 1 global Interrupt */
    0,                                 /**< 63 */
    comp_interrupt_handler,            /**< 64 COMP1 and COMP2 Interrupts */
    lptim1_interrupt_handler,          /**< 65 LP TIM1 interrupt */
    lptim2_interrupt_handler,          /**< 66 LP TIM2 interrupt */
    usb_interrupt_handler,             /**< 67 USB event Interrupt */
    dma2_channel6_interrupt_handler,   /**< 68 DMA2 Channel 6 global interrupt */
    dma2_channel7_interrupt_handler,   /**< 69 DMA2 Channel 7 global interrupt */
    lpuart1_interrupt_handler,         /**< 70 LP UART1 interrupt */
    quadspi_interrupt_handler,         /**< 71 Quad SPI global interrupt */
    i2c3_ev_interrupt_handler,         /**< 72 I2C3 event interrupt */
    i2c3_er_interrupt_handler,         /**< 73 I2C3 error interrupt */
    sai1_interrupt_handler,            /**< 74 Serial Audio Interface 1 global interrupt */
    0,                                 /**< 75 */
    swpmi1_interrupt_handler,          /**< 76 SWPMI1 global interrupt */
    tsc_interrupt_handler,             /**< 77 Touch Sense Controller global interrupt */
    0,                                 /**< 78 */
    aes_interrupt_handler,             /**< 79 AES global interrupt */
    rng_interrupt_handler,             /**< 80 RNG global interrupt */
    fpu_interrupt_handler,             /**< 81 FPU global interrupt */
    crs_interrupt_handler,             /**< 82 CRS global interrupt */
    i2c4_ev_interrupt_handler,         /**< 83 I2C4 Event interrupt */
    i2c4_er_interrupt_handler,         /**< 84 I2C4 Error interrupt */
    ignore_fn                          /**< forces the linker to add this fn */
};

/** Get the system clock requency.
 * @return SystemCoreClock
 */
__attribute__((__weak__)) uint32_t HAL_RCC_GetSysClockFreq(void)
{
    return cm3_cpu_clock_hz;
}

/** Stub function to make the HAL happy.  We don't need it for any of our
 * drivers.
 *
 * @return 0
 */
uint32_t HAL_GetTick(void)
{
    return 0;
}

#include "../boards/armv7m/default_handlers.h"

void debug_interrupt_handler(void)
    __attribute__ ((weak, alias ("default_interrupt_handler")));
void wwdg_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void pvd_pvm_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tamp_stamp_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void rtc_wkup_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void flash_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void rcc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_channel7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void adc1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can1_tx_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can1_rx0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can1_rx1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can1_sce_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti9_5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_brk_tim15_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_up_tim16_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_trg_com_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_cc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c1_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c1_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c2_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c2_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usart1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usart2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usart3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void exti15_10_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void rtc_alarm_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sdmmc1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void uart4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim6_dac_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void comp_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lptim1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lptim2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usb_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_channel7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lpuart1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void quadspi_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c3_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c3_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sai1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void swpmi1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tsc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void aes_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void rng_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void fpu_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void crs_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c4_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c4_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
