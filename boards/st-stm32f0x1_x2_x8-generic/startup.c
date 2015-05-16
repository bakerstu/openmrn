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

/* prototypes */
extern unsigned long *__stack;
extern void reset_handler(void);
static void nmi_handler(void);
static void hard_fault_handler(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void __libc_init_array(void);

extern int main(int argc, char *argv[]);

extern void watchdog_interrupt_handler(void);
extern void pvd_vddio2_interrupt_handler(void);
extern void rtc_interrupt_handler(void);
extern void flash_interrupt_handler(void);
extern void rcc_crs_interrupt_handler(void);
extern void external0_1_interrupt_handler(void);
extern void external2_3_interrupt_handler(void);
extern void external4_15_interrupt_handler(void);
extern void touch_interrupt_handler(void);
extern void dma_ch1_interrupt_handler(void);
extern void dma_ch2_3_dma2_ch1_2_interrupt_handler(void);
extern void dma_ch4_5_6_7_dma2_ch3_4_5_interrupt_handler(void);
extern void adc_comp_interrupt_handler(void);
extern void timer1_break_update_trigger_commutation_interrupt_handler(void);
extern void timer1_cc_interrupt_handler(void);
extern void timer2_interrupt_handler(void);
extern void timer3_interrupt_handler(void);
extern void timer6_dac_interrupt_handler(void);
extern void timer7_interrupt_handler(void);
extern void timer14_interrupt_handler(void);
extern void timer15_interrupt_handler(void);
extern void timer16_interrupt_handler(void);
extern void timer17_interrupt_handler(void);
extern void i2c1_interrupt_handler(void);
extern void i2c2_interrupt_handler(void);
extern void spi1_interrupt_handler(void);
extern void spi2_interrupt_handler(void);
extern void uart1_interrupt_handler(void);
extern void uart2_interrupt_handler(void);
extern void uart3_4_5_6_7_8_interrupt_handler(void);
extern void cec_can_interrupt_handler(void);
extern void usb_interrupt_handler(void);

extern void ignore_fn(void);

extern const unsigned long cpu_clock_hz;

/** CPU clock speed. */
const unsigned long cpu_clock_hz = 48000000;
uint32_t SystemCoreClock = 48000000;

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
    0,                               /**<  13 reserved -- bootloader appentry */
    PendSV_Handler,                  /**<  14 pend SV */
    SysTick_Handler,                 /**<  15 system tick */
    watchdog_interrupt_handler,      /**<  16 watchdog timer */

    /** PVD and VDDIO2 supply comparator + EXTI lines[31,16] */
    pvd_vddio2_interrupt_handler,    /**<  17 */

    /** real time clock + EXTI lines[19,17,20] */
    rtc_interrupt_handler,           /**<  18 */
    flash_interrupt_handler,         /**<  19 flash global interrupt */
    rcc_crs_interrupt_handler,       /**<  20 RCC and CRS global interrupt */
    external0_1_interrupt_handler,  /**<  21 EXTI line[1:0] */
    external2_3_interrupt_handler,  /**<  22 EXTI line[3:2] */
    external4_15_interrupt_handler, /**<  23 EXTI line[15:4] */
    touch_interrupt_handler,         /**<  24 touch sensing */
    dma_ch1_interrupt_handler,       /**<  25 DMA channel 1 */

    /** DMA channel 2 and 3, DMA2 channel 1 and 2 */
    dma_ch2_3_dma2_ch1_2_interrupt_handler, /* 26 */

    /** DMA channel 4, 5, 6, and 7, DMA2 channel 3, 4, and 5 */
    dma_ch4_5_6_7_dma2_ch3_4_5_interrupt_handler, /* 27 */
    adc_comp_interrupt_handler,      /**<  28 ADC and COMP + EXTI line[22:21] */

    /** timer 1 break, update, trigger, and commutation */
    timer1_break_update_trigger_commutation_interrupt_handler, /* 29 */
    timer1_cc_interrupt_handler,     /**<  30 timer 1 capture compare */
    timer2_interrupt_handler,        /**<  31 timer 2 */
    timer3_interrupt_handler,        /**<  32 timer 3 */
    timer6_dac_interrupt_handler,    /**<  33 timer 6 and DAC underrun */
    timer7_interrupt_handler,        /**<  34 timer 7 */
    timer14_interrupt_handler,       /**<  35 timer 14 */
    timer15_interrupt_handler,       /**<  36 timer 15 */
    timer16_interrupt_handler,       /**<  37 timer 16 */
    timer17_interrupt_handler,       /**<  38 timer 17 */
    i2c1_interrupt_handler,          /**<  39 I2C1 + EXTI line[23] */
    i2c2_interrupt_handler,          /**<  40 I2C2 */
    spi1_interrupt_handler,          /**<  41 SPI1 */
    spi2_interrupt_handler,          /**<  42 SPI2 */
    uart1_interrupt_handler,         /**<  43 UART1 + EXTI line[25] */
    uart2_interrupt_handler,         /**<  44 UART2 + EXTI line[26] */

    /** UART3, UART4, UART5, UART6, UART7, UART8 + EXTI line[28] */
    uart3_4_5_6_7_8_interrupt_handler, /*  45 */
    cec_can_interrupt_handler,       /**<  46 CEC and CAN + EXTI line[27] */
    usb_interrupt_handler,           /**<  47 USB + EXTI line[18] */

    ignore_fn                        /**< forces the linker to add this fn */
};

extern unsigned long __data_section_table;
extern unsigned long __data_section_table_end;
extern unsigned long __bss_section_table;
extern unsigned long __bss_section_table_end;

extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;

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

/** This hardware initialization code will be called before C++ global objects
 * are initialized. */
extern void hw_preinit(void);

/** Sets the hardware outputs to a safe state. Called when the program crashes
 * handler. */
/*void hw_set_to_safe(void) __attribute__ ((weak));
void hw_set_to_safe(void)
{
}*/
extern void hw_set_to_safe(void);


/** Startup the C/C++ runtime environment.
 */
void reset_handler(void)
{
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

volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */


/* These are volatile to try and prevent the compiler/linker optimising them
 * away as the variables never actually get used. 
 */
typedef struct hardfault_args
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
} HardfaultArgs;

volatile HardfaultArgs hf_args;


/** Decode the stack state prior to an exception occuring.  This code is
 * inspired by FreeRTOS.
 * @param address address of the stack
 */
__attribute__((__naked__, optimize("-O0"))) void hard_fault_handler_c( unsigned long *hardfault_args )
{
    hw_set_to_safe();

    hf_args.stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    hf_args.stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    hf_args.stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    hf_args.stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    hf_args.stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    hf_args.stacked_lr = ((unsigned long)hardfault_args[5]) ;
    hf_args.stacked_pc = ((unsigned long)hardfault_args[6]) ;
    hf_args.stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    hf_args._CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
                                                                                    
    // Hard Fault Status Register
    hf_args._HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    hf_args._DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    hf_args._AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    hf_args._MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    hf_args._BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    __asm("BKPT #0\n") ; // Break into the debugger

    /* When the following line is hit, the variables contain the register values. */
    if (hf_args.stacked_r0 || hf_args.stacked_r1  || hf_args.stacked_r2 ||
        hf_args.stacked_r3 || hf_args.stacked_r12 || hf_args.stacked_lr ||
        hf_args.stacked_pc || hf_args.stacked_psr || hf_args._CFSR      ||
        hf_args._HFSR      || hf_args._DFSR       || hf_args._AFSR      ||
        hf_args._MMAR      || hf_args._BFAR)
    {
        resetblink(BLINK_DIE_HARDFAULT);
        for( ;; );
    }
}

/** The fault handler implementation.  This code is inspired by FreeRTOS.
 */
static void hard_fault_handler(void)
{
    __asm volatile
    (
        "MOVS   R0, #4                  \n"
        "MOV    R1, LR                  \n"
        "TST    R0, R1                  \n"
        "BEQ    _MSP                    \n"
        "MRS    R0, PSP                 \n"
        "B      hard_fault_handler_c    \n"
        "_MSP:                          \n"
        "MRS    R0, MSP                 \n"
        "B      hard_fault_handler_c    \n"
        "BX    LR\n"
    );
}

static void nmi_handler(void)
{
    for ( ; /* forever */ ; )
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
void default_interrupt_handler(void) __attribute__ ((weak));
void default_interrupt_handler(void)
{
    while(1);
    diewith(BLINK_DIE_UNEXPIRQ);
}

void watchdog_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pvd_vddio2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void rtc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void flash_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void rcc_crs_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external0_1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external2_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external4_15_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void touch_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_ch1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_ch2_3_dma2_ch1_2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_ch4_5_6_7_dma2_ch3_4_5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc_comp_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1_break_update_trigger_commutation_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1_cc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer6_dac_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer7_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer14_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer15_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer16_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer17_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart3_4_5_6_7_8_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void cec_can_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void usb_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
