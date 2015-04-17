/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * This file sets up the runtime environment for TI Stellaris/Tiva MCUs.
 *
 * @author Stuart W. Baker
 * @date 4 May 2014
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
extern void debug_interrupt_handler(void);

extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void __libc_init_array(void);

extern int main(int argc, char *argv[]);

extern void watchdog_interrupt_handler(void);
extern void timer0_interrupt_handler(void);
extern void timer1_interrupt_handler(void);
extern void timer2_interrupt_handler(void);
extern void timer3_interrupt_handler(void);
extern void uart0_interrupt_handler(void);
extern void uart1_interrupt_handler(void);
extern void uart2_interrupt_handler(void);
extern void uart3_interrupt_handler(void);
extern void pwm1_interrupt_handler(void);
extern void i2c0_interrupt_handler(void);
extern void i2c1_interrupt_handler(void);
extern void i2c2_interrupt_handler(void);
extern void spi_interrupt_handler(void);
extern void ssp0_interrupt_handler(void);
extern void ssp1_interrupt_handler(void);
extern void pll0_interrupt_handler(void);
extern void rtc_interrupt_handler(void);
extern void external0_interrupt_handler(void);
extern void external1_interrupt_handler(void);
extern void external2_interrupt_handler(void);
extern void external3_interrupt_handler(void);
extern void adc_interrupt_handler(void);
extern void bod_interrupt_handler(void);
extern void usb_interrupt_handler(void);
extern void can_interrupt_handler(void);
extern void gpdma_interrupt_handler(void);
extern void i2s_interrupt_handler(void);
extern void ethernet_interrupt_handler(void);
extern void repetitive_interrupt_handler(void);
extern void mcpwm_interrupt_handler(void);
extern void qei_interrupt_handler(void);
extern void pll1_interrupt_handler(void);
extern void usb_activity_interrupt_handler(void);
extern void can_activity_interrupt_handler(void);
extern void ignore_fn(void);

extern const unsigned long cm3_cpu_clock_hz;
/** CPU clock speed. */
const unsigned long cm3_cpu_clock_hz = 100000000;

/** Exception table */
__attribute__ ((section(".interrupt_vector")))
void (* const __interrupt_vector[])(void) =
{
    (void (*)(void))(&__stack),      /**<   0 initial stack pointer */
    reset_handler,                   /**<   1 reset vector */
    nmi_handler,                     /**<   2 non-maskable interrupt */
    hard_fault_handler,              /**<   3 hard fault */
    mpu_fault_handler,               /**<   4 memory managment */
    bus_fault_handler,               /**<   5 bus Fault */
    usage_fault_handler,             /**<   6 usage fault */
    0,                               /**<   7 reserved */
    0,                               /**<   8 reserved */
    0,                               /**<   9 reserved */
    0,                               /**<  10 reserved */
    SVC_Handler,                     /**<  11 SV call */
    debug_interrupt_handler,         /**<  12 debug monitor */
    0,                               /**<  13 reserved */
    PendSV_Handler,                  /**<  14 pend SV */
    SysTick_Handler,                 /**<  15 system tick */
    watchdog_interrupt_handler,      /**<  16 watchdog timer*/
    timer0_interrupt_handler,        /**<  17 timer 0 */
    timer1_interrupt_handler,        /**<  18 timer 1 */
    timer2_interrupt_handler,        /**<  19 timer 2 */
    timer3_interrupt_handler,        /**<  20 timer 3 */
    uart0_interrupt_handler,         /**<  21 UART0 */
    uart1_interrupt_handler,         /**<  22 UART1 */
    uart2_interrupt_handler,         /**<  23 UART2 */
    uart3_interrupt_handler,         /**<  24 UART3 */
    pwm1_interrupt_handler,           /**<  25 PWM generator */
    i2c0_interrupt_handler,          /**<  26 I2C0 */
    i2c1_interrupt_handler,          /**<  27 I2C1 */
    i2c2_interrupt_handler,          /**<  28 I2C2 */
    spi_interrupt_handler,           /**<  29 SPI */
    ssp0_interrupt_handler,          /**<  30 SSP0 */
    ssp1_interrupt_handler,          /**<  31 SSP1 */
    pll0_interrupt_handler,          /**<  32 PLL0 (main PLL) */
    rtc_interrupt_handler,           /**<  33 real time clock */
    external0_interrupt_handler,     /**<  34 external interrupt 0 */
    external1_interrupt_handler,     /**<  35 external interrupt 1 */
    external2_interrupt_handler,     /**<  36 external interrupt 2 */
    external3_interrupt_handler,     /**<  37 external interrupt 3 */
    adc_interrupt_handler,           /**<  38 ADC */
    bod_interrupt_handler,           /**<  39 brown out detect */
    usb_interrupt_handler,           /**<  40 USB */
    can_interrupt_handler,           /**<  41 CAN */
    gpdma_interrupt_handler,         /**<  42 DMA channels 1 and 2 */
    i2s_interrupt_handler,           /**<  43 I2S */
    ethernet_interrupt_handler,      /**<  44 Ethernet */
    repetitive_interrupt_handler,    /**<  45 repetitive interrupt timer */
    mcpwm_interrupt_handler,         /**<  46 motion control PWM */
    qei_interrupt_handler,           /**<  47 quadrature encoder interface */
    pll1_interrupt_handler,          /**<  48 PLL1 (USB PLL) */
    usb_activity_interrupt_handler,  /**<  49 USB activity */
    can_activity_interrupt_handler,  /**<  50 CAN activity */
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
    away as the variables never actually get used.  If the debugger won't show the
    values of the variables, make them global my moving their declaration outside
    of this function. */
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

/** Decode the stack state prior to an exception occuring.  This code is
 * inspired by FreeRTOS.
 * @param address address of the stack
 */
__attribute__((__naked__, optimize("-O0"))) void hard_fault_handler_c( unsigned long *hardfault_args )
{
    hw_set_to_safe();

    stacked_r0 = ((unsigned long)hardfault_args[0]) ;
    stacked_r1 = ((unsigned long)hardfault_args[1]) ;
    stacked_r2 = ((unsigned long)hardfault_args[2]) ;
    stacked_r3 = ((unsigned long)hardfault_args[3]) ;
    stacked_r12 = ((unsigned long)hardfault_args[4]) ;
    stacked_lr = ((unsigned long)hardfault_args[5]) ;
    stacked_pc = ((unsigned long)hardfault_args[6]) ;
    stacked_psr = ((unsigned long)hardfault_args[7]) ;

    // Configurable Fault Status Register
    // Consists of MMSR, BFSR and UFSR
    _CFSR = (*((volatile unsigned long *)(0xE000ED28))) ;   
                                                                                    
    // Hard Fault Status Register
    _HFSR = (*((volatile unsigned long *)(0xE000ED2C))) ;

    // Debug Fault Status Register
    _DFSR = (*((volatile unsigned long *)(0xE000ED30))) ;

    // Auxiliary Fault Status Register
    _AFSR = (*((volatile unsigned long *)(0xE000ED3C))) ;

    // Read the Fault Address Registers. These may not contain valid values.
    // Check BFARVALID/MMARVALID to see if they are valid values
    // MemManage Fault Address Register
    _MMAR = (*((volatile unsigned long *)(0xE000ED34))) ;
    // Bus Fault Address Register
    _BFAR = (*((volatile unsigned long *)(0xE000ED38))) ;

    __asm("BKPT #0\n") ; // Break into the debugger

    /* When the following line is hit, the variables contain the register values. */
    if (stacked_r0 || stacked_r1 || stacked_r2 || stacked_r3 || stacked_r12 ||
        stacked_lr || stacked_pc || stacked_psr || _CFSR || _HFSR || _DFSR ||
        _AFSR || _MMAR || _BFAR)
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
void default_interrupt_handler(void) __attribute__ ((weak));
void default_interrupt_handler(void)
{
    while(1);
    diewith(BLINK_DIE_UNEXPIRQ);
}

void debug_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void watchdog_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssp0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssp1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pll0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void rtc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void external3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void bod_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void usb_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void can_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void gpdma_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2s_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ethernet_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void repetitive_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void mcpwm_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void qei_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pll1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void usb_activity_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void can_activity_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));

