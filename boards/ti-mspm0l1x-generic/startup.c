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
 * This file sets up the runtime environment for TI MSPM0L130x MCUs.
 *
 * @author Stuart W. Baker
 * @date 20 April 2015
 */

#include <stdint.h>

#include <FreeRTOSConfig.h>

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

/* device specific interrupt handlers */
extern void group0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void group1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void timg1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void adc0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void spi0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void uart1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void uart0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void timg0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void timg2_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void timg4_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void i2c0_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void i2c1_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));
extern void dma_interrupt_handler(void)
    __attribute__((weak, alias("default_interrupt_handler")));

extern void ignore_fn(void);

extern const unsigned long cpu_clock_hz;

/** CPU clock speed. */
const unsigned long cpu_clock_hz = 32000000;
uint32_t SystemCoreClock = 32000000;

/** Exception table */
__attribute__ ((section(".interrupt_vector")))
void (* const __interrupt_vector[])(void) =
{
    (void (*)(void))(&__stack),                    /* The initial stack pointer */
    reset_handler,                         /* the reset handler         */
    nmi_handler,                           /* the nmi handler           */
    hard_fault_handler,                    /* the hard fault handler    */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    SVC_Handler,                           /* svcall handler            */
    0,                                     /* reserved                  */
    reset_handler,                         /* reserved -- bootloader appentry */
    PendSV_Handler,                        /* the pendsv handler        */
    SysTick_Handler,                       /* systick handler           */
    group0_interrupt_handler,                     /* group0 interrupt handler  */
    group1_interrupt_handler,                     /* group1 interrupt handler  */
    timg1_interrupt_handler,                      /* timg1 interrupt handler   */
    0,                                     /* reserved                  */
    adc0_interrupt_handler,                       /* adc0 interrupt handler    */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    spi0_interrupt_handler,                       /* spi0 interrupt handler    */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    uart1_interrupt_handler,                      /* uart1 interrupt handler   */
    0,                                     /* reserved                  */
    uart0_interrupt_handler,                      /* uart0 interrupt handler   */
    timg0_interrupt_handler,                      /* timg0 interrupt handler   */
    0,                                     /* reserved                  */
    timg2_interrupt_handler,                      /* timg2 interrupt handler   */
    0,                                     /* reserved                  */
    timg4_interrupt_handler,                      /* timg4 interrupt handler   */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    i2c0_interrupt_handler,                       /* i2c0 interrupt handler    */
    i2c1_interrupt_handler,                       /* i2c1 interrupt handler    */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    0,                                     /* reserved                  */
    dma_interrupt_handler,                        /* dma interrupt handler     */

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
