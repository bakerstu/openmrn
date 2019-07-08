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
extern void dma1_stream0_interrupt_handler(void);
extern void dma1_stream1_interrupt_handler(void);
extern void dma1_stream2_interrupt_handler(void);
extern void dma1_stream3_interrupt_handler(void);
extern void dma1_stream4_interrupt_handler(void);
extern void dma1_stream5_interrupt_handler(void);
extern void dma1_stream6_interrupt_handler(void);
extern void adc1_2_3_interrupt_handler(void);
extern void can1_tx_interrupt_handler(void);
extern void can1_rx0_interrupt_handler(void);
extern void can1_rx1_interrupt_handler(void);
extern void can1_sce_interrupt_handler(void);
extern void exti9_5_interrupt_handler(void);
extern void tim1_brk_tim9_interrupt_handler(void);
extern void tim1_up_tim10_interrupt_handler(void);
extern void tim1_trg_com_tim11_interrupt_handler(void);
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
extern void usbotg_fs_wakeup_interrupt_handler(void);
extern void tim8_brk_tim12_interrupt_handler(void);
extern void tim8_up_tim13_interrupt_handler(void);
extern void tim8_trg_com_tim14_interrupt_handler(void);
extern void tim8_cc_interrupt_handler(void);
extern void dma1_stream7_interrupt_handler(void);
extern void fmc_interrupt_handler(void);
extern void sdmmc1_interrupt_handler(void);
extern void tim5_interrupt_handler(void);
extern void spi3_interrupt_handler(void);
extern void uart4_interrupt_handler(void);
extern void uart5_interrupt_handler(void);
extern void tim6_dac12underrun_interrupt_handler(void);
extern void tim7_interrupt_handler(void);
extern void dma2_stream0_interrupt_handler(void);
extern void dma2_stream1_interrupt_handler(void);
extern void dma2_stream2_interrupt_handler(void);
extern void dma2_stream3_interrupt_handler(void);
extern void dma2_stream4_interrupt_handler(void);
extern void eth_interrupt_handler(void);
extern void eth_wkup_interrupt_handler(void);
extern void can2_tx_interrupt_handler(void);
extern void can2_rx0_interrupt_handler(void);
extern void can2_rx1_interrupt_handler(void);
extern void can2_sce_interrupt_handler(void);
extern void usbotg_fs_interrupt_handler(void);
extern void dma2_stream5_interrupt_handler(void);
extern void dma2_stream6_interrupt_handler(void);
extern void dma2_stream7_interrupt_handler(void);
extern void usart6_interrupt_handler(void);
extern void i2c3_ev_interrupt_handler(void);
extern void i2c3_er_interrupt_handler(void);
extern void usbotg_hs_ep1_out_interrupt_handler(void);
extern void usbotg_hs_ep1_in_interrupt_handler(void);
extern void usbotg_oth_hs_wakeup_interrupt_handler(void);
extern void usbotg_hs_interrupt_handler(void);
extern void dcmi_interrupt_handler(void);
extern void cryp_interrupt_handler(void);
extern void hash_rng_interrupt_handler(void);
extern void fpu_interrupt_handler(void);
extern void uart7_interrupt_handler(void);
extern void uart8_interrupt_handler(void);
extern void spi4_interrupt_handler(void);
extern void spi5_interrupt_handler(void);
extern void spi6_interrupt_handler(void);
extern void sai1_interrupt_handler(void);
extern void lcd_tft_interrupt_handler(void);
extern void lcd_tft_err_interrupt_handler(void);
extern void dma2d_interrupt_handler(void);
extern void sai2_interrupt_handler(void);
extern void quadspi_interrupt_handler(void);
extern void lptim1_interrupt_handler(void);
extern void hdmi_cec_interrupt_handler(void);
extern void i2c4_ev_interrupt_handler(void);
extern void i2c4_er_interrupt_handler(void);
extern void spdif_rx_interrupt_handler(void);
extern void dsihost_interrupt_handler(void);
extern void dfsdm1_flt0_interrupt_handler(void);
extern void dfsdm1_flt1_interrupt_handler(void);
extern void dfsdm1_flt2_interrupt_handler(void);
extern void dfsdm1_flt3_interrupt_handler(void);
extern void sdmmc2_interrupt_handler(void);
extern void can3_tx_interrupt_handler(void);
extern void can3_rx0_interrupt_handler(void);
extern void can3_rx1_interrupt_handler(void);
extern void can3_sce_interrupt_handler(void);
extern void jpeg_interrupt_handler(void);
extern void mdios_interrupt_handler(void);

extern void ignore_fn(void);

extern const unsigned long cm3_cpu_clock_hz;

/** Exception table */
__attribute__((
    section(".interrupt_vector"))) void (*const __interrupt_vector[])(void) = {
    (void (*)(void))(&__stack),      /**<  0 initial stack pointer */
    reset_handler,                   /**<  1 reset vector */
    nmi_handler,                     /**<  2 non-maskable interrupt */
    hard_fault_handler,              /**<  3 hard fault */
    mpu_fault_handler,               /**<  4 reserved */
    bus_fault_handler,               /**<  5 reserved */
    usage_fault_handler,             /**<  6 reserved */
    0,                               /**<  7 reserved */
    0,                               /**<  8 reserved */
    0,                               /**<  9 reserved */
    0,                               /**< 10 reserved */
    SVC_Handler,                     /**< 11 SV call */
    debug_interrupt_handler,         /**< 12 reserved */
    0,                               /**< 13 reserved -- bootloader appentry */
    PendSV_Handler,                  /**< 14 pend SV */
    SysTick_Handler,                 /**< 15 system tick */
    watchdog_interrupt_handler,      /**< 16 watchdog timer */
    pvd_interrupt_handler,           /**< 17 */
    tamper_interrupt_handler,        /**< 18 */
    rtc_interrupt_handler,           /**< 19 */
    flash_interrupt_handler,         /**< 20 */
    rcc_interrupt_handler,           /**< 21 */
    exti0_interrupt_handler,         /**< 22 */
    exti1_interrupt_handler,         /**< 23 */
    exti2_interrupt_handler,         /**< 24 */
    exti3_interrupt_handler,         /**< 25 */
    exti4_interrupt_handler,         /**< 26 */
    dma1_stream0_interrupt_handler,  /**< 27 */
    dma1_stream1_interrupt_handler,  /**< 28 */
    dma1_stream2_interrupt_handler,  /**< 29 */
    dma1_stream3_interrupt_handler,  /**< 30 */
    dma1_stream4_interrupt_handler,  /**< 31 */
    dma1_stream5_interrupt_handler,  /**< 32 */
    dma1_stream6_interrupt_handler,  /**< 33 */
    adc1_2_3_interrupt_handler,      /**< 34 */
    can1_tx_interrupt_handler,       /**< 35 */
    can1_rx0_interrupt_handler,      /**< 36 */
    can1_rx1_interrupt_handler,      /**< 37 */
    can1_sce_interrupt_handler,      /**< 38 */
    exti9_5_interrupt_handler,       /**< 39 */
    tim1_brk_tim9_interrupt_handler, /**< 40 */
    tim1_up_tim10_interrupt_handler, /**< 41 */
    tim1_trg_com_tim11_interrupt_handler,   /**< 42 */
    tim1_cc_interrupt_handler,              /**< 43 */
    tim2_interrupt_handler,                 /**< 44 */
    tim3_interrupt_handler,                 /**< 45 */
    tim4_interrupt_handler,                 /**< 46 */
    i2c1_ev_interrupt_handler,              /**< 47 */
    i2c1_er_interrupt_handler,              /**< 48 */
    i2c2_ev_interrupt_handler,              /**< 49 */
    i2c2_er_interrupt_handler,              /**< 50 */
    spi1_interrupt_handler,                 /**< 51 */
    spi2_interrupt_handler,                 /**< 52 */
    usart1_interrupt_handler,               /**< 53 */
    usart2_interrupt_handler,               /**< 54 */
    usart3_interrupt_handler,               /**< 55 */
    exti15_10_interrupt_handler,            /**< 56 */
    rtc_alarm_interrupt_handler,            /**< 57 */
    usbotg_fs_wakeup_interrupt_handler,     /**< 58 */
    tim8_brk_tim12_interrupt_handler,       /**< 59 */
    tim8_up_tim13_interrupt_handler,        /**< 60 */
    tim8_trg_com_tim14_interrupt_handler,   /**< 61 */
    tim8_cc_interrupt_handler,              /**< 62 */
    dma1_stream7_interrupt_handler,         /**< 63 */
    fmc_interrupt_handler,                  /**< 64 */
    sdmmc1_interrupt_handler,               /**< 65 */
    tim5_interrupt_handler,                 /**< 66 */
    spi3_interrupt_handler,                 /**< 67 */
    uart4_interrupt_handler,                /**< 68 */
    uart5_interrupt_handler,                /**< 69 */
    tim6_dac12underrun_interrupt_handler,   /**< 70 */
    tim7_interrupt_handler,                 /**< 71 */
    dma2_stream0_interrupt_handler,         /**< 72 */
    dma2_stream1_interrupt_handler,         /**< 73 */
    dma2_stream2_interrupt_handler,         /**< 74 */
    dma2_stream3_interrupt_handler,         /**< 75 */
    dma2_stream4_interrupt_handler,         /**< 76 */
    eth_interrupt_handler,                  /**< 77 */
    eth_wkup_interrupt_handler,             /**< 78 */
    can2_tx_interrupt_handler,              /**< 79 */
    can2_rx0_interrupt_handler,             /**< 80 */
    can2_rx1_interrupt_handler,             /**< 81 */
    can2_sce_interrupt_handler,             /**< 82 */
    usbotg_fs_interrupt_handler,            /**< 83 */
    dma2_stream5_interrupt_handler,         /**< 84 */
    dma2_stream6_interrupt_handler,         /**< 85 */
    dma2_stream7_interrupt_handler,         /**< 86 */
    usart6_interrupt_handler,               /**< 87 */
    i2c3_ev_interrupt_handler,              /**< 88 */
    i2c3_er_interrupt_handler,              /**< 89 */
    usbotg_hs_ep1_out_interrupt_handler,    /**< 90 */
    usbotg_hs_ep1_in_interrupt_handler,     /**< 91 */
    usbotg_oth_hs_wakeup_interrupt_handler, /**< 92 */
    usbotg_hs_interrupt_handler,            /**< 93 */
    dcmi_interrupt_handler,                 /**< 94 */
    cryp_interrupt_handler,                 /**< 95 */
    hash_rng_interrupt_handler,             /**< 96 */
    fpu_interrupt_handler,                  /**< 97 */
    uart7_interrupt_handler,                /**< 98 */
    uart8_interrupt_handler,                /**< 99 */
    spi4_interrupt_handler,                 /**< 100 */
    spi5_interrupt_handler,                 /**< 101 */
    spi6_interrupt_handler,                 /**< 102 */
    sai1_interrupt_handler,                 /**< 103 */
    lcd_tft_interrupt_handler,              /**< 104 */
    lcd_tft_err_interrupt_handler,          /**< 105 */
    dma2d_interrupt_handler,                /**< 106 */
    sai2_interrupt_handler,                 /**< 107 */
    quadspi_interrupt_handler,              /**< 108 */
    lptim1_interrupt_handler,               /**< 109 */
    hdmi_cec_interrupt_handler,             /**< 110 */
    i2c4_ev_interrupt_handler,              /**< 111 */
    i2c4_er_interrupt_handler,              /**< 112 */
    spdif_rx_interrupt_handler,             /**< 113 */
    dsihost_interrupt_handler,              /**< 114 */
    dfsdm1_flt0_interrupt_handler,          /**< 115 */
    dfsdm1_flt1_interrupt_handler,          /**< 116 */
    dfsdm1_flt2_interrupt_handler,          /**< 117 */
    dfsdm1_flt3_interrupt_handler,          /**< 118 */
    sdmmc2_interrupt_handler,               /**< 119 */
    can3_tx_interrupt_handler,              /**< 120 */
    can3_rx0_interrupt_handler,             /**< 121 */
    can3_rx1_interrupt_handler,             /**< 122 */
    can3_sce_interrupt_handler,             /**< 123 */
    jpeg_interrupt_handler,                 /**< 124 */
    mdios_interrupt_handler,                /**< 125 */
    ignore_fn /**< forces the linker to add this fn */
};

/*extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;*/


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

extern unsigned long __data_section_table;
extern unsigned long __data_section_table_end;
extern unsigned long __bss_section_table;
extern unsigned long __bss_section_table_end;

extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

/** This hardware initialization code will be called before C++ global objects
 * are initialized. */
extern void hw_preinit(void);

extern void hw_set_to_safe(void);

/** Startup the C/C++ runtime environment.
 */
void reset_handler(void)
{
    /* Globally disables interrupts until the FreeRTOS scheduler is up. */
    __asm("cpsid i\n");

    unsigned long *section_table_addr = &__data_section_table;

    /* copy ram load sections from flash to ram */
    while (section_table_addr < &__data_section_table_end)
    {
        unsigned long *src = (unsigned long *)*section_table_addr++;
        unsigned long *dst = (unsigned long *)*section_table_addr++;
        unsigned long  len = (unsigned long)  *section_table_addr++;

        for ( ; len; len -= 4)
        {
            *dst++ = *src++;
        }
    }

    /* zero initialize bss segment(s) */
    while (section_table_addr < &__bss_section_table_end)
    {
        unsigned long *zero = (unsigned long *)*section_table_addr++;
        unsigned long  len  = (unsigned long)  *section_table_addr++;
        
        for ( ; len; len -= 4)
        {
            *zero++ = 0;
        }
    }

    hw_preinit();

    /* call static constructors */
    __libc_init_array();

    /* execute main */
    char *argv[] = {0};
    main(0, argv);

    for ( ; /* forever */ ;)
    {
        /* if we ever return from main, loop forever */
    }
}

extern void resetblink(unsigned long pattern);
//extern void diewith(unsigned pattern);


    /* These are volatile to try and prevent the compiler/linker optimising them
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
/** Fault handling information */
typedef struct fault_information
{
    volatile unsigned long stacked_r0 ;
    volatile unsigned long stacked_r1 ;
    volatile unsigned long stacked_r2 ;
    volatile unsigned long stacked_r3 ;
    volatile unsigned long stacked_r12 ;
    volatile unsigned long stacked_lr ;
    volatile unsigned long stacked_pc ;
    volatile unsigned long stacked_psr ;
    volatile unsigned long _CFSR ;
    volatile unsigned long _HFSR ;
    volatile unsigned long _DFSR ;
    volatile unsigned long _AFSR ;
    volatile unsigned long _BFAR ;
    volatile unsigned long _MMAR ;
} FaultInformation;

/** Global instance so that it can be added to the watch expressions */
volatile FaultInformation faultInfo;

/** Decode the stack state prior to an exception occuring.  This code is
 * inspired by FreeRTOS.
 * @param address address of the stack
 */
__attribute__((optimize("-O0"))) void hard_fault_handler_c( unsigned long *hardfault_args )
{
    /* force a reference in the local variables for debug */
    volatile FaultInformation *fault_info = &faultInfo;

    fault_info->stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    fault_info->stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    fault_info->stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    fault_info->stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    fault_info->stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    fault_info->stacked_lr = ((unsigned long)hardfault_args[5]) ;
    fault_info->stacked_pc = ((unsigned long)hardfault_args[6]) ;
    fault_info->stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    fault_info->_CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
                                                                                    
    // Hard Fault Status Register
    fault_info->_HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    fault_info->_DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    fault_info->_AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    fault_info->_MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    fault_info->_BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    hw_set_to_safe();

    __asm("BKPT #0\n") ; // Break into the debugger

    /* When the following line is hit, the variables contain the register values. */
    if (fault_info->stacked_r0  || fault_info->stacked_r1  ||
        fault_info->stacked_r2  || fault_info->stacked_r3  ||
        fault_info->stacked_r12 || fault_info->stacked_lr  ||
        fault_info->stacked_pc  || fault_info->stacked_psr ||
        fault_info->_CFSR       || fault_info->_HFSR       ||
        fault_info->_DFSR       || fault_info->_AFSR       ||
        fault_info->_MMAR       || fault_info->_BFAR)
    {
        resetblink(BLINK_DIE_HARDFAULT);
        for( ;; );
    }
}

/** The fault handler implementation.  This code is inspired by FreeRTOS.
 */
__attribute__((__naked__)) static void hard_fault_handler(void)
{
    __asm volatile
    (
        " tst   lr, #4               \n"
        " ite   eq                   \n"
        " mrseq r0, msp              \n"
        " mrsne r0, psp              \n"
        " b     hard_fault_handler_c \n"
        " bx    lr                   \n"
    );
}


static void nmi_handler(void)
{
    for (; /* forever */;)
    {
    }
}

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

/** This is the default handler for exceptions not defined by the application.
 */
void default_interrupt_handler(void) __attribute__((weak));
void default_interrupt_handler(void)
{
    volatile unsigned irqno = (*((uint32_t*)0xE000ED04)) & 0xff;
    (void) irqno;
    while (1)
        ;
    diewith(BLINK_DIE_UNEXPIRQ);
}

void debug_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));


void watchdog_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void pvd_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tamper_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void rtc_interrupt_handler(void)
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
void dma1_stream0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void adc1_2_3_interrupt_handler(void)
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
void tim1_brk_tim9_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_up_tim10_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_trg_com_tim11_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim1_cc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim4_interrupt_handler(void)
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
void usbotg_fs_wakeup_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim8_brk_tim12_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim8_up_tim13_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim8_trg_com_tim14_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim8_cc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma1_stream7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void fmc_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sdmmc1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void uart4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void uart5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim6_dac12underrun_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void tim7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void eth_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void eth_wkup_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can2_tx_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can2_rx0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can2_rx1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can2_sce_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usbotg_fs_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2_stream7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usart6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c3_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c3_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usbotg_hs_ep1_out_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usbotg_hs_ep1_in_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usbotg_oth_hs_wakeup_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void usbotg_hs_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dcmi_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void cryp_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void hash_rng_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void fpu_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void uart7_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void uart8_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi5_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spi6_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sai1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lcd_tft_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lcd_tft_err_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dma2d_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sai2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void quadspi_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void lptim1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void hdmi_cec_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c4_ev_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void i2c4_er_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void spdif_rx_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dsihost_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void dfsdm1_flt3_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void sdmmc2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can3_tx_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can3_rx0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can3_rx1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void can3_sce_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void jpeg_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
void mdios_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
