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
 * This file sets up the runtime environment for ST STM32G0x1 MCUs.
 *
 * @author Stuart W. Baker
 * @date 20 April 2015
 * @author Brian Barnt & Balazs Racz
 * @date 26 August 2023
 */

#include <stdint.h>

#include "FreeRTOSConfig.h"

/* prototypes */
extern void watchdog_interrupt_handler(void);
extern void pvd_vddio2_interrupt_handler(void);
extern void rtc_interrupt_handler(void);
extern void flash_interrupt_handler(void);
extern void rcc_crs_interrupt_handler(void);
extern void external0_1_interrupt_handler(void);
extern void external2_3_interrupt_handler(void);
extern void external4_15_interrupt_handler(void);
extern void ucpd_usb_interrupt_handler(void);
extern void dma_ch1_interrupt_handler(void);
extern void dma_ch2_3_interrupt_handler(void);
extern void dma_ch4_5_6_7_dma2_ch1_2_3_4_5_interrupt_handler(void);
extern void adc_comp_interrupt_handler(void);
extern void timer1_break_update_trigger_commutation_interrupt_handler(void);
extern void timer1_cc_interrupt_handler(void);
extern void timer2_interrupt_handler(void);
extern void timer3_4_interrupt_handler(void);
extern void timer6_dac_lptim1_interrupt_handler(void);
extern void timer7_lptim2_interrupt_handler(void);
extern void timer14_interrupt_handler(void);
extern void timer15_interrupt_handler(void);
extern void timer16_fdcan_it0_interrupt_handler(void);
extern void timer17_fdcan_it1_interrupt_handler(void);
extern void i2c1_interrupt_handler(void);
extern void i2c2_3_interrupt_handler(void);
extern void spi1_interrupt_handler(void);
extern void spi2_3_interrupt_handler(void);
extern void uart1_interrupt_handler(void);
extern void uart2_lpuart2_interrupt_handler(void);
extern void uart3_4_5_6_lpuart1_interrupt_handler(void);
extern void cec_interrupt_handler(void);
extern void aes_rng_interrupt_handler(void);

extern const unsigned long cpu_clock_hz;

/** CPU clock speed. */
const unsigned long cpu_clock_hz = 64000000;
extern uint32_t SystemCoreClock;

#include "../boards/armv6m/default_handlers.h"

/** Exception table */
__attribute__ ((section(".interrupt_vector")))
void (* const __interrupt_vector[])(void) =
{
    (void (*)(void))(&__stack),      /**<   0 initial stack pointer */
    reset_handler,                   /**<   1 reset vector */
    nmi_handler,                     /**<   2 non-maskable interrupt */
    hard_fault_handler,              /**<   3 hard fault */
    0,                               /**<   4 reserved */
    0,                               /**<   5 reserved */
    0,                               /**<   6 reserved */
    0,                               /**<   7 reserved */
    0,                               /**<   8 reserved */
    0,                               /**<   9 reserved */
    0,                               /**<  10 reserved */
    SVC_Handler,                     /**<  11 SV call */
    0,                               /**<  12 reserved */
    reset_handler,                   /**<  13 reserved -- bootloader appentry */
    PendSV_Handler,                  /**<  14 pend SV */
    SysTick_Handler,                 /**<  15 system tick */
    watchdog_interrupt_handler,      /**<  16 watchdog timer */

    /** PVD and VDDIO2 supply comparator + EXTI lines[31,16] */
    pvd_vddio2_interrupt_handler,    /**<  17 */

    /** real time clock + EXTI lines[19,17,20] */
    rtc_interrupt_handler,           /**<  18 */
    flash_interrupt_handler,         /**<  19 flash global interrupt */
    rcc_crs_interrupt_handler,       /**<  20 RCC and CRS global interrupt */
    external0_1_interrupt_handler,   /**<  21 EXTI line[1:0] */
    external2_3_interrupt_handler,   /**<  22 EXTI line[3:2] */
    external4_15_interrupt_handler,  /**<  23 EXTI line[15:4] */
    ucpd_usb_interrupt_handler,      /**<  24 USB Power */
    dma_ch1_interrupt_handler,       /**<  25 DMA channel 1 */

    /** DMA channel 2 and 3, DMA2 channel 1 and 2 */
    dma_ch2_3_interrupt_handler,     /* 26 */

    /** DMA channel 4, 5, 6, and 7, DMA2 channel 3, 4, and 5 */
    dma_ch4_5_6_7_dma2_ch1_2_3_4_5_interrupt_handler, /* 27 */
    adc_comp_interrupt_handler,      /**<  28 ADC and COMP + EXTI line[22:21] */

    /** timer 1 break, update, trigger, and commutation */
    timer1_break_update_trigger_commutation_interrupt_handler, /* 29 */
    timer1_cc_interrupt_handler,     /**<  30 timer 1 capture compare */
    timer2_interrupt_handler,        /**<  31 timer 2 */
    timer3_4_interrupt_handler,      /**<  32 timer 3 */
    timer6_dac_lptim1_interrupt_handler, /**<  33 timer 6 and DAC underrun */
    timer7_lptim2_interrupt_handler,        /**<  34 timer 7 */
    timer14_interrupt_handler,       /**<  35 timer 14 */
    timer15_interrupt_handler,       /**<  36 timer 15 */
    timer16_fdcan_it0_interrupt_handler,       /**<  37 timer 16 */
    timer17_fdcan_it1_interrupt_handler,       /**<  38 timer 17 */
    i2c1_interrupt_handler,          /**<  39 I2C1 + EXTI line[23] */
    i2c2_3_interrupt_handler,        /**<  40 I2C2 */
    spi1_interrupt_handler,          /**<  41 SPI1 */
    spi2_3_interrupt_handler,        /**<  42 SPI2 */
    uart1_interrupt_handler,         /**<  43 UART1 + EXTI line[25] */
    uart2_lpuart2_interrupt_handler, /**<  44 UART2 + EXTI line[26] */

    /** UART3, UART4, UART5, UART6, UART7, UART8 + EXTI line[28] */
    uart3_4_5_6_lpuart1_interrupt_handler, /*  45 */
    cec_interrupt_handler,           /**<  46 CEC and CAN + EXTI line[27] */
    aes_rng_interrupt_handler,       /**<  47 USB + EXTI line[18] */

    ignore_fn                        /**< forces the linker to add this fn */
};

/** Get the system clock requency.
 * @return SystemCoreClock
*/
uint32_t HAL_RCC_GetSysClockFreq(void)
{
    return cpu_clock_hz;
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

extern void resetblink(unsigned long pattern);
//extern void diewith(unsigned pattern);

void watchdog_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pvd_vddio2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void rtc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void flash_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void rcc_crs_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external0_1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external2_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external4_15_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ucpd_usb_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));


void dma_ch1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_ch2_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void  dma_ch4_5_6_7_dma2_ch1_2_3_4_5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc_comp_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1_break_update_trigger_commutation_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1_cc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3_4_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer6_dac_lptim1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer7_lptim2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer14_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer15_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer16_fdcan_it0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer17_fdcan_it1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c2_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi2_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart2_lpuart2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart3_4_5_6_lpuart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void cec_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void aes_rng_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
