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
 * This file sets up the runtime environment for ST STM32F0x1, STM32F0x2, and
 * STM32F0x8 MCUs.
 *
 * @author Stuart W. Baker
 * @date 20 April 2015
 */

#include <stdint.h>

#include "FreeRTOSConfig.h"

extern void watchdog_interrupt_handler(void);
extern void pvd_interrupt_handler(void);
extern void tamper_interrupt_handler(void);
extern void rtc_interrupt_handler(void);
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
extern void adc1_2_interrupt_handler(void);
extern void usb_hp_can1_tx_interrupt_handler(void);
extern void usb_lp_can1_rx0_interrupt_handler(void);
extern void can1_rx1_interrupt_handler(void);
extern void can1_sce_interrupt_handler(void);
extern void exti9_5_interrupt_handler(void);
extern void tim1_brk_interrupt_handler(void);
extern void tim1_up_interrupt_handler(void);
extern void tim1_trg_com_interrupt_handler(void);
extern void tim1_cc_interrupt_handler(void);
extern void tim2_interrupt_handler(void);
extern void tim3_interrupt_handler(void);
extern void tim4_interrupt_handler(void);
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
extern void usbwakeup_interrupt_handler(void);
extern void tim8_brk_interrupt_handler(void);
extern void tim8_up_interrupt_handler(void);
extern void tim8_trg_com_interrupt_handler(void);
extern void tim8_cc_interrupt_handler(void);
extern void adc3_interrupt_handler(void);
extern void fsmc_interrupt_handler(void);
extern void sdio_interrupt_handler(void);
extern void tim5_interrupt_handler(void);
extern void spi3_interrupt_handler(void);
extern void uart4_interrupt_handler(void);
extern void uart5_interrupt_handler(void);
extern void tim6_interrupt_handler(void);
extern void tim7_interrupt_handler(void);
extern void dma2_channel1_interrupt_handler(void);
extern void dma2_channel2_interrupt_handler(void);
extern void dma2_channel3_interrupt_handler(void);
extern void dma2_channel4_5_interrupt_handler(void);
extern void dma2_channel5_interrupt_handler(void);
extern void eth_interrupt_handler(void);
extern void eth_wkup_interrupt_handler(void);
extern void can2_tx_interrupt_handler(void);
extern void can2_rx0_interrupt_handler(void);
extern void can2_rx1_interrupt_handler(void);
extern void can2_sce_interrupt_handler(void);
extern void otg_fs_interrupt_handler(void);

/** Exception table */
__attribute__((section(".interrupt_vector"))) void (
    *const __interrupt_vector[])(
    void) = {(void (*)(void))(&__stack), /**<   0 initial stack pointer */
             reset_handler,              /**<   1 reset vector */
             nmi_handler,                /**<   2 non-maskable interrupt */
             hard_fault_handler,         /**<   3 hard fault */
             memmanage_handler,          /**<   4 reserved */
             busfault_handler,           /**<   5 reserved */
             usagefault_handler,         /**<   6 reserved */
             0,                          /**<   7 reserved */
             0,                          /**<   8 reserved */
             0,                          /**<   9 reserved */
             0,                          /**<  10 reserved */
             SVC_Handler,                /**<  11 SV call */
             debugmon_handler,           /**<  12 reserved */
             0,               /**<  13 reserved -- bootloader appentry */
             PendSV_Handler,  /**<  14 pend SV */
             SysTick_Handler, /**<  15 system tick */
             watchdog_interrupt_handler,        /**<  16 watchdog timer */
             pvd_interrupt_handler,             /**< 17 */
             tamper_interrupt_handler,          /**< 18 */
             rtc_interrupt_handler,             /**< 19 */
             flash_interrupt_handler,           /**< 20 */
             rcc_interrupt_handler,             /**< 21 */
             exti0_interrupt_handler,           /**< 22 */
             exti1_interrupt_handler,           /**< 23 */
             exti2_interrupt_handler,           /**< 24 */
             exti3_interrupt_handler,           /**< 25 */
             exti4_interrupt_handler,           /**< 26 */
             dma1_channel1_interrupt_handler,   /**< 27 */
             dma1_channel2_interrupt_handler,   /**< 28 */
             dma1_channel3_interrupt_handler,   /**< 29 */
             dma1_channel4_interrupt_handler,   /**< 30 */
             dma1_channel5_interrupt_handler,   /**< 31 */
             dma1_channel6_interrupt_handler,   /**< 32 */
             dma1_channel7_interrupt_handler,   /**< 33 */
             adc1_2_interrupt_handler,          /**< 34 */
             usb_hp_can1_tx_interrupt_handler,  /**< 35 */
             usb_lp_can1_rx0_interrupt_handler, /**< 36 */
             can1_rx1_interrupt_handler,        /**< 37 */
             can1_sce_interrupt_handler,        /**< 38 */
             exti9_5_interrupt_handler,         /**< 39 */
             tim1_brk_interrupt_handler,        /**< 40 */
             tim1_up_interrupt_handler,         /**< 41 */
             tim1_trg_com_interrupt_handler,    /**< 42 */
             tim1_cc_interrupt_handler,         /**< 43 */
             tim2_interrupt_handler,            /**< 44 */
             tim3_interrupt_handler,            /**< 45 */
             tim4_interrupt_handler,            /**< 46 */
             i2c1_ev_interrupt_handler,         /**< 47 */
             i2c1_er_interrupt_handler,         /**< 48 */
             i2c2_ev_interrupt_handler,         /**< 49 */
             i2c2_er_interrupt_handler,         /**< 50 */
             spi1_interrupt_handler,            /**< 51 */
             spi2_interrupt_handler,            /**< 52 */
             usart1_interrupt_handler,          /**< 53 */
             usart2_interrupt_handler,          /**< 54 */
             usart3_interrupt_handler,          /**< 55 */
             exti15_10_interrupt_handler,       /**< 56 */
             rtc_alarm_interrupt_handler,       /**< 57 */
             usbwakeup_interrupt_handler,       /**< 58 */
             tim8_brk_interrupt_handler,        /**< 59 */
             tim8_up_interrupt_handler,         /**< 60 */
             tim8_trg_com_interrupt_handler,    /**< 61 */
             tim8_cc_interrupt_handler,         /**< 62 */
             adc3_interrupt_handler,            /**< 63 */
             fsmc_interrupt_handler,            /**< 64 */
             0,                                 /**< 65 */
             0,                                 /**< 66 */
             spi3_interrupt_handler,            /**< 67 */
             uart4_interrupt_handler,           /**< 68 */
             uart5_interrupt_handler,           /**< 69 */
             tim6_interrupt_handler,            /**< 70 */
             tim7_interrupt_handler,            /**< 71 */
             dma2_channel1_interrupt_handler,   /**< 72 */
             dma2_channel2_interrupt_handler,   /**< 73 */
             dma2_channel3_interrupt_handler,   /**< 74 */
             dma2_channel4_interrupt_handler,   /**< 75 */
             dma2_channel5_interrupt_handler,   /**< 76 */
             adc4_interrupt_handler,            /**< 77 */
             0,                                 /**< 78 */
             0,                                 /**< 79 */
             comp1_2_3_interrupt_handler,       /**< 80 */
             comp4_5_6_interrupt_handler,       /**< 81 */
             comp7_interrupt_handler,           /**< 82 */
             0,                                 /**< 83 */
             0,                                 /**< 84 */
             0,                                 /**< 85 */
             0,                                 /**< 86 */
             0,                                 /**< 87 */
             i2c3_ev_interrupt_handler,         /**< 88 */
             i2c3_er_interrupt_handler,         /**< 89 */
             usb_hp_interrupt_handler,          /**< 90 */
             usb_lp_interrupt_handler,          /**< 91 */
             usb_wakeup_interrupt_handler,      /**< 92 */
             tim20_brk_interrupt_handler,       /**< 93 */
             tim20_up_interrupt_handler,        /**< 94 */
             tim20_trig_com_interrupt_handler,  /**< 95 */
             tim20_cc_interrupt_handler,        /**< 96 */
             fpu_interrupt_handler,             /**< 97 */
             0,                                 /**< 98 */
             0,                                 /**< 99 */
             spi4_interrupt_handler,            /**< 100 */
             ignore_fn /**< forces the linker to add this fn */
};

static_assert(ARRAYSIZE(__interrupt_vector) == 85);

/*extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;*/


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
// extern void diewith(unsigned pattern);

volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr;  /* Link register. */
volatile uint32_t pc;  /* Program counter. */
volatile uint32_t psr; /* Program status register. */

/* These are volatile to try and prevent the compiler/linker optimising them
 * away as the variables never actually get used.
 */
typedef struct hardfault_args
{
    volatile unsigned long stacked_r0;
    volatile unsigned long stacked_r1;
    volatile unsigned long stacked_r2;
    volatile unsigned long stacked_r3;
    volatile unsigned long stacked_r12;
    volatile unsigned long stacked_lr;
    volatile unsigned long stacked_pc;
    volatile unsigned long stacked_psr;
    volatile unsigned long _CFSR;
    volatile unsigned long _HFSR;
    volatile unsigned long _DFSR;
    volatile unsigned long _AFSR;
    volatile unsigned long _BFAR;
    volatile unsigned long _MMAR;
} HardfaultArgs;

volatile HardfaultArgs hf_args;

/** Decode the stack state prior to an exception occuring.  This code is
 * inspired by FreeRTOS.
 * @param address address of the stack
 */
__attribute__((__naked__, optimize("-O0"))) void hard_fault_handler_c(
    unsigned long *hardfault_args)
{
    hw_set_to_safe();

    hf_args.stacked_r0 = ((unsigned long)hardfault_args[0]);
    hf_args.stacked_r1 = ((unsigned long)hardfault_args[1]);
    hf_args.stacked_r2 = ((unsigned long)hardfault_args[2]);
    hf_args.stacked_r3 = ((unsigned long)hardfault_args[3]);
    hf_args.stacked_r12 = ((unsigned long)hardfault_args[4]);
    hf_args.stacked_lr = ((unsigned long)hardfault_args[5]);
    hf_args.stacked_pc = ((unsigned long)hardfault_args[6]);
    hf_args.stacked_psr = ((unsigned long)hardfault_args[7]);

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    hf_args._CFSR = (*((volatile unsigned long *)(0xE000ED28)));

    // Hard Fault Status Register
    hf_args._HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

    // Debug Fault Status Register
    hf_args._DFSR = (*((volatile unsigned long *)(0xE000ED30)));

    // Auxiliary Fault Status Register
    hf_args._AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    hf_args._MMAR = (*((volatile unsigned long *)(0xE000ED34)));
    // Bus Fault Address Register
    hf_args._BFAR = (*((volatile unsigned long *)(0xE000ED38)));

    __asm("BKPT #0\n"); // Break into the debugger

    /* When the following line is hit, the variables contain the register
     * values. */
    if (hf_args.stacked_r0 || hf_args.stacked_r1 || hf_args.stacked_r2 ||
        hf_args.stacked_r3 || hf_args.stacked_r12 || hf_args.stacked_lr ||
        hf_args.stacked_pc || hf_args.stacked_psr || hf_args._CFSR ||
        hf_args._HFSR || hf_args._DFSR || hf_args._AFSR || hf_args._MMAR ||
        hf_args._BFAR)
    {
        resetblink(BLINK_DIE_HARDFAULT);
        for (;;)
            ;
    }
}

/** The fault handler implementation.  This code is inspired by FreeRTOS.
 */
static void hard_fault_handler(void)
{
    __asm volatile("MOVS   R0, #4                  \n"
                   "MOV    R1, LR                  \n"
                   "TST    R0, R1                  \n"
                   "BEQ    _MSP                    \n"
                   "MRS    R0, PSP                 \n"
                   "B      hard_fault_handler_c    \n"
                   "_MSP:                          \n"
                   "MRS    R0, MSP                 \n"
                   "B      hard_fault_handler_c    \n"
                   "BX    LR\n");
}

static void nmi_handler(void)
{
    for (; /* forever */;)
    {
    }
}
#if 0
static void mpu_fault_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}

static void bus_fault_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}

static void usage_fault_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}
#endif

/** This is the default handler for exceptions not defined by the application.
 */
void default_interrupt_handler(void) __attribute__((weak));
void default_interrupt_handler(void)
{
    while (1)
        ;
    diewith(BLINK_DIE_UNEXPIRQ);
}

void watchdog_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void pvd_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tamper_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void rtc_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void flash_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void rcc_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti0_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti4_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel4_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel6_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma1_channel7_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void adc1_2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usb_hp_can1_tx_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usb_lp_can1_rx0_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can1_rx1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can1_sce_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti9_5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim1_brk_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim1_up_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim1_trg_com_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim1_cc_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim4_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void i2c1_ev_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void i2c1_er_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void i2c2_ev_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void i2c2_er_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void spi1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void spi2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usart1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usart2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usart3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void exti15_10_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void rtc_alarm_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void usbwakeup_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim8_brk_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim8_up_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim8_trg_com_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim8_cc_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void adc3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void fsmc_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void sdio_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void spi3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void uart4_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void uart5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim6_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void tim7_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma2_channel1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma2_channel2_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma2_channel3_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma2_channel4_5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void dma2_channel5_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void eth_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void eth_wkup_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can2_tx_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can2_rx0_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can2_rx1_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void can2_sce_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
void otg_fs_interrupt_handler(void)
    __attribute__((weak, alias(default_interrupt_handler)));
