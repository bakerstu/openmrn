/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * This file sets up the runtime environment for TI CC3200 MCUs.
 *
 * @author Stuart W. Baker
 * @date 8 March 2016
 */

#include <stdint.h>

#include "FreeRTOSConfig.h"

/* we define this our selves because TivaWare forces us to otherwise bring in 
 * a device specific header to define this.  We want to keep this file generic
 * to all Cortex-M based TI MCU's
 */
#define NVIC_INT_CTRL_R (*((volatile uint32_t *)0xE000ED04))

#define NVIC_INT_DIS_BASE ((volatile uint32_t *)0xE000E180)

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
extern void porta0_interrupt_handler(void);
extern void porta1_interrupt_handler(void);
extern void porta2_interrupt_handler(void);
extern void porta3_interrupt_handler(void);
extern void uart0_interrupt_handler(void);
extern void uart1_interrupt_handler(void);
extern void i2c0_interrupt_handler(void);
extern void adc0_channel0_interrupt_handler(void);
extern void adc0_channel1_interrupt_handler(void);
extern void adc0_channel2_interrupt_handler(void);
extern void adc0_channel3_interrupt_handler(void);
extern void watchdog_interrupt_handler(void);
extern void timer0a_interrupt_handler(void);
extern void timer0b_interrupt_handler(void);
extern void timer1a_interrupt_handler(void);
extern void timer1b_interrupt_handler(void);
extern void timer2a_interrupt_handler(void);
extern void timer2b_interrupt_handler(void);
extern void timer3a_interrupt_handler(void);
extern void timer3b_interrupt_handler(void);
extern void dma_software_interrupt_handler(void);
extern void dma_error_interrupt_handler(void);
extern void i2s0_interrupt_handler(void);
extern void camera_interrupt_handler(void);
extern void network_interrupt_handler(void);
extern void spi0_interrupt_handler(void);
extern void ignore_fn(void);

extern const unsigned long cm3_cpu_clock_hz;
/** CPU clock speed. */
const unsigned long cm3_cpu_clock_hz = 80000000;

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
    reset_handler,                   /**<  13 reserved -- bootloader appentry */
    PendSV_Handler,                  /**<  14 pend SV */
    SysTick_Handler,                 /**<  15 system tick */
    porta0_interrupt_handler,        /**<  16 GPIO port A0 (GPIO  0 -  7) */
    porta1_interrupt_handler,        /**<  17 GPIO port A1 (GPIO  8 - 15) */
    porta2_interrupt_handler,        /**<  18 GPIO port A2 (GPIO 16 - 23) */
    porta3_interrupt_handler,        /**<  19 GPIO port A3 (GPIO 24 - 31) */
    0,                               /**<  20 reserved */
    uart0_interrupt_handler,         /**<  21 UART0 */
    uart1_interrupt_handler,         /**<  22 UART1 */
    0,                               /**<  23 reserved */
    i2c0_interrupt_handler,          /**<  24 I2C0 */
    0,                               /**<  25 reserved */
    0,                               /**<  26 reserved */
    0,                               /**<  27 reserved */
    0,                               /**<  28 reserved */
    0,                               /**<  29 reserved */
    adc0_channel0_interrupt_handler, /**<  30 ADC0 Channel 0 */
    adc0_channel1_interrupt_handler, /**<  31 ADC0 Channel 1 */
    adc0_channel2_interrupt_handler, /**<  32 ADC0 Channel 2 */
    adc0_channel3_interrupt_handler, /**<  33 ADC0 Channel 3 */
    watchdog_interrupt_handler,      /**<  34 watchdog timer */
    timer0a_interrupt_handler,       /**<  35 timer 0A */
    timer0b_interrupt_handler,       /**<  36 timer 0B */
    timer1a_interrupt_handler,       /**<  37 timer 1A */
    timer1b_interrupt_handler,       /**<  38 timer 1B */
    timer2a_interrupt_handler,       /**<  39 timer 2A */
    timer2b_interrupt_handler,       /**<  40 timer 2B */
    0,                               /**<  41 reserved */
    0,                               /**<  42 reserved */
    0,                               /**<  43 reserved */
    0,                               /**<  44 reserved */
    0,                               /**<  45 reserved */
    0,                               /**<  46 reserved */
    0,                               /**<  47 reserved */
    0,                               /**<  48 reserved */
    0,                               /**<  49 reserved */
    0,                               /**<  50 reserved */
    timer3a_interrupt_handler,       /**<  51 timer 3A */
    timer3b_interrupt_handler,       /**<  52 timer 3B */
    0,                               /**<  53 reserved */
    0,                               /**<  54 reserved */
    0,                               /**<  55 reserved */
    0,                               /**<  56 reserved */
    0,                               /**<  57 reserved */
    0,                               /**<  58 reserved */
    0,                               /**<  59 reserved */
    0,                               /**<  60 reserved */
    0,                               /**<  61 reserved */
    dma_software_interrupt_handler,  /**<  62 uDMA software */
    dma_error_interrupt_handler,     /**<  63 uDMA error */
    0,                               /**<  64 reserved */
    0,                               /**<  65 reserved */
    0,                               /**<  66 reserved */
    0,                               /**<  67 reserved */
    0,                               /**<  68 reserved */
    0,                               /**<  69 reserved */
    0,                               /**<  70 reserved */
    0,                               /**<  71 reserved */
    0,                               /**<  72 reserved */
    0,                               /**<  73 reserved */
    0,                               /**<  74 reserved */
    0,                               /**<  75 reserved */
    0,                               /**<  76 reserved */
    0,                               /**<  77 reserved */
    0,                               /**<  78 reserved */
    0,                               /**<  79 reserved */
    0,                               /**<  80 reserved */
    0,                               /**<  81 reserved */
    0,                               /**<  82 reserved */
    0,                               /**<  83 reserved */
    0,                               /**<  84 reserved */
    0,                               /**<  85 reserved */
    0,                               /**<  86 reserved */
    0,                               /**<  87 reserved */
    0,                               /**<  88 reserved */
    0,                               /**<  89 reserved */
    0,                               /**<  90 reserved */
    0,                               /**<  91 reserved */
    0,                               /**<  92 reserved */
    0,                               /**<  93 reserved */
    0,                               /**<  94 reserved */
    0,                               /**<  95 reserved */
    0,                               /**<  96 reserved */
    0,                               /**<  97 reserved */
    0,                               /**<  98 reserved */
    0,                               /**<  99 reserved */
    0,                               /**< 100 reserved */
    0,                               /**< 101 reserved */
    0,                               /**< 102 reserved */
    0,                               /**< 103 reserved */
    0,                               /**< 104 reserved */
    0,                               /**< 105 reserved */
    0,                               /**< 106 reserved */
    0,                               /**< 107 reserved */
    0,                               /**< 108 reserved */
    0,                               /**< 109 reserved */
    0,                               /**< 110 reserved */
    0,                               /**< 111 reserved */
    0,                               /**< 112 reserved */
    0,                               /**< 113 reserved */
    0,                               /**< 114 reserved */
    0,                               /**< 115 reserved */
    0,                               /**< 116 reserved */
    0,                               /**< 117 reserved */
    0,                               /**< 118 reserved */
    0,                               /**< 119 reserved */
    0,                               /**< 120 reserved */
    0,                               /**< 121 reserved */
    0,                               /**< 122 reserved */
    0,                               /**< 123 reserved */
    0,                               /**< 124 reserved */
    0,                               /**< 125 reserved */
    0,                               /**< 126 reserved */
    0,                               /**< 127 reserved */
    0,                               /**< 128 reserved */
    0,                               /**< 129 reserved */
    0,                               /**< 130 reserved */
    0,                               /**< 131 reserved */
    0,                               /**< 132 reserved */
    0,                               /**< 133 reserved */
    0,                               /**< 134 reserved */
    0,                               /**< 135 reserved */
    0,                               /**< 136 reserved */
    0,                               /**< 137 reserved */
    0,                               /**< 138 reserved */
    0,                               /**< 139 reserved */
    0,                               /**< 140 reserved */
    0,                               /**< 141 reserved */
    0,                               /**< 142 reserved */
    0,                               /**< 143 reserved */
    0,                               /**< 144 reserved */
    0,                               /**< 145 reserved */
    0,                               /**< 146 reserved */
    0,                               /**< 147 reserved */
    0,                               /**< 148 reserved */
    0,                               /**< 149 reserved */
    0,                               /**< 150 reserved */
    0,                               /**< 151 reserved */
    0,                               /**< 152 reserved */
    0,                               /**< 153 reserved */
    0,                               /**< 154 reserved */
    0,                               /**< 155 reserved */
    0,                               /**< 156 reserved */
    0,                               /**< 157 reserved */
    0,                               /**< 158 reserved */
    0,                               /**< 159 reserved */
    0,                               /**< 160 reserved */
    0,                               /**< 161 reserved */
    0,                               /**< 162 reserved */
    0,                               /**< 163 reserved */
    0,                               /**< 164 reserved */
    0,                               /**< 165 reserved */
    0,                               /**< 166 reserved */
    0,                               /**< 167 reserved */
    0,                               /**< 168 reserved */
    0,                               /**< 169 reserved */
    0,                               /**< 170 reserved */
    0,                               /**< 171 reserved */
    0,                               /**< 172 reserved */
    0,                               /**< 173 reserved */
    0,                               /**< 174 reserved */
    0,                               /**< 175 reserved */
    0,                               /**< 176 reserved */
    i2s0_interrupt_handler,          /**< 177 I2S0 */
    0,                               /**< 178 reserved */
    camera_interrupt_handler,        /**< 179 Camera Interface */
    0,                               /**< 180 reserved */
    0,                               /**< 181 reserved */
    0,                               /**< 182 reserved */
    0,                               /**< 183 reserved */
    0,                               /**< 184 reserved */
    0,                               /**< 185 reserved */
    0,                               /**< 186 Netowrk */
    network_interrupt_handler,       /**< 187 reserved */
    0,                               /**< 188 reserved */
    0,                               /**< 189 reserved */
    0,                               /**< 190 reserved */
    0,                               /**< 191 reserved */
    spi0_interrupt_handler,          /**< 192 SPI0 */
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
    asm("cpsid i\n");

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
        // Restores the real stack pointer
        " mov   sp, r0               \n"
        // Simulates a BL instruction form the original PC
        " ldr   lr, [r0, 20]       \n"
        " b     hard_fault_handler_c \n"
        " bx    lr                   \n"
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

volatile unsigned long _INTCTRL = 0;

/** This is the default handler for exceptions not defined by the application.
 */
void default_interrupt_handler(void) __attribute__ ((weak));
void default_interrupt_handler(void)
{
    _INTCTRL = NVIC_INT_CTRL_R;
    while(1);
    diewith(BLINK_DIE_UNEXPIRQ);
}

void debug_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porta0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porta1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porta2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porta3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_channel0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_channel1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_channel2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_channel3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void watchdog_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer0a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer0b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
extern void dma_software_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
extern void dma_error_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2s0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void camera_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void network_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void spi0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));

