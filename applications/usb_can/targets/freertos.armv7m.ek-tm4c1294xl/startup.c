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

/* we define this our selves because TivaWare forces us to otherwise bring in 
 * a device specific header to define this.  We want to keep this file generic
 * to all Cortex-M based TI MCU's
 */
#define NVIC_INT_CTRL_R (*((volatile uint32_t *)0xE000ED04))

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
extern void porta_interrupt_handler(void);
extern void portb_interrupt_handler(void);
extern void portc_interrupt_handler(void);
extern void portd_interrupt_handler(void);
extern void porte_interrupt_handler(void);
extern void uart0_interrupt_handler(void);
extern void uart1_interrupt_handler(void);
extern void ssi0_interrupt_handler(void);
extern void i2c0_interrupt_handler(void);
extern void pwm0_fault_interrupt_handler(void);
extern void pwm0_0_interrupt_handler(void);
extern void pwm0_1_interrupt_handler(void);
extern void pwm0_2_interrupt_handler(void);
extern void qei0_interrupt_handler(void);
extern void adc0_seq0_interrupt_handler(void);
extern void adc0_seq1_interrupt_handler(void);
extern void adc0_seq2_interrupt_handler(void);
extern void adc0_seq3_interrupt_handler(void);
extern void watchdog_interrupt_handler(void);
extern void timer0a_interrupt_handler(void);
extern void timer0b_interrupt_handler(void);
extern void timer1a_interrupt_handler(void);
extern void timer1b_interrupt_handler(void);
extern void timer2a_interrupt_handler(void);
extern void timer2b_interrupt_handler(void);
extern void comparator0_interrupt_handler(void);
extern void comparator1_interrupt_handler(void);
extern void comparator2_interrupt_handler(void);
extern void system_control_interrupt_handler(void);
extern void flash_interrupt_handler(void);
extern void portf_interrupt_handler(void);
extern void portg_interrupt_handler(void);
extern void porth_interrupt_handler(void);
extern void uart2_interrupt_handler(void);
extern void ssi1_interrupt_handler(void);
extern void timer3a_interrupt_handler(void);
extern void timer3b_interrupt_handler(void);
extern void i2c1_interrupt_handler(void);
extern void qei1_interrupt_handler(void);
extern void can0_interrupt_handler(void);
extern void can1_interrupt_handler(void);
extern void ethernet_interrupt_handler(void);
extern void hibernation_interrupt_handler(void);
extern void usb0_interrupt_handler(void);
extern void pwm0_3_interrupt_handler(void);
extern void dma_software_interrupt_handler(void);
extern void dma_error_interrupt_handler(void);
extern void adc1_seq0_interrupt_handler(void);
extern void adc1_seq1_interrupt_handler(void);
extern void adc1_seq2_interrupt_handler(void);
extern void adc1_seq3_interrupt_handler(void);
extern void i2s0_interrupt_handler(void);
extern void epi_interrupt_handler(void);
extern void portj_interrupt_handler(void);
extern void portk_interrupt_handler(void);
extern void portl_interrupt_handler(void);
extern void ssi2_interrupt_handler(void);
extern void ssi3_interrupt_handler(void);
extern void uart3_interrupt_handler(void);
extern void uart4_interrupt_handler(void);
extern void uart5_interrupt_handler(void);
extern void uart6_interrupt_handler(void);
extern void uart7_interrupt_handler(void);
extern void i2c2_interrupt_handler(void);
extern void i2c3_interrupt_handler(void);
extern void timer4a_interrupt_handler(void);
extern void timer4b_interrupt_handler(void);
extern void timer5a_interrupt_handler(void);
extern void timer5b_interrupt_handler(void);
extern void timer6a_interrupt_handler(void);
extern void timer6b_interrupt_handler(void);
extern void timer7a_interrupt_handler(void);
extern void timer7b_interrupt_handler(void);
extern void system_except_interrupt_handler(void);
extern void i2c4_interrupt_handler(void);
extern void i2c5_interrupt_handler(void);
extern void i2c6_interrupt_handler(void);
extern void i2c7_interrupt_handler(void);
extern void i2c8_interrupt_handler(void);
extern void i2c9_interrupt_handler(void);
extern void portm_interrupt_handler(void);
extern void portn_interrupt_handler(void);
extern void portp_interrupt_handler(void);
extern void portp0_interrupt_handler(void);
extern void portp1_interrupt_handler(void);
extern void portp2_interrupt_handler(void);
extern void portp3_interrupt_handler(void);
extern void portp4_interrupt_handler(void);
extern void portp5_interrupt_handler(void);
extern void portp6_interrupt_handler(void);
extern void portp7_interrupt_handler(void);
extern void pwm1_0_interrupt_handler(void);
extern void pwm1_1_interrupt_handler(void);
extern void pwm1_2_interrupt_handler(void);
extern void pwm1_3_interrupt_handler(void);
extern void pwm1_fault_interrupt_handler(void);
extern void fp_interrupt_handler(void);
extern void tamper_interrupt_handler(void);
extern void ignore_fn(void);

extern const unsigned long cm3_cpu_clock_hz;
/** CPU clock speed. */
const unsigned long cm3_cpu_clock_hz = 120000000;

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
    porta_interrupt_handler,         /**<  16 GPIO port A */
    portb_interrupt_handler,         /**<  17 GPIO port B */
    portc_interrupt_handler,         /**<  18 GPIO port C */
    portd_interrupt_handler,         /**<  19 GPIO port D */
    porte_interrupt_handler,         /**<  20 GPIO port E */
    uart0_interrupt_handler,         /**<  21 UART0 */
    uart1_interrupt_handler,         /**<  22 UART1 */
    ssi0_interrupt_handler,          /**<  23 SSI0 */
    i2c0_interrupt_handler,          /**<  24 I2C0 */
    pwm0_fault_interrupt_handler,    /**<  25 PWM0 fault */
    pwm0_0_interrupt_handler,        /**<  26 PWM0 generator 0 */
    pwm0_1_interrupt_handler,        /**<  27 PWM0 generator 1 */
    pwm0_2_interrupt_handler,        /**<  28 PWM0 generator 2 */
    qei0_interrupt_handler,          /**<  29 QEI0 */
    adc0_seq0_interrupt_handler,     /**<  30 ADC0 Sequence 0 */
    adc0_seq1_interrupt_handler,     /**<  31 ADC0 Sequence 1 */
    adc0_seq2_interrupt_handler,     /**<  32 ADC0 Sequence 2 */
    adc0_seq3_interrupt_handler,     /**<  33 ADC0 Sequence 3 */
    watchdog_interrupt_handler,      /**<  34 watchdog timers 0 and 1 */
    timer0a_interrupt_handler,       /**<  35 timer 0A */
    timer0b_interrupt_handler,       /**<  36 timer 0B */
    timer1a_interrupt_handler,       /**<  37 timer 1A */
    timer1b_interrupt_handler,       /**<  38 timer 1B */
    timer2a_interrupt_handler,       /**<  39 timer 2A */
    timer2b_interrupt_handler,       /**<  40 timer 2B */
    comparator0_interrupt_handler,   /**<  41 analog comparator 0 */
    comparator1_interrupt_handler,   /**<  42 analog comparator 1 */
    comparator2_interrupt_handler,   /**<  43 analog comparator 2 */
    system_control_interrupt_handler,/**<  44 system control */
    flash_interrupt_handler,         /**<  45 FLASH memory Ccontrol */
    portf_interrupt_handler,         /**<  46 GPIO port F */
    portg_interrupt_handler,         /**<  47 GPIO port G */
    porth_interrupt_handler,         /**<  48 GPIO port H */
    uart2_interrupt_handler,         /**<  49 UART2 */
    ssi1_interrupt_handler,          /**<  50 SSI1 */
    timer3a_interrupt_handler,       /**<  51 timer 3A */
    timer3b_interrupt_handler,       /**<  52 timer 3B */
    i2c1_interrupt_handler,          /**<  53 I2C1 */
    can0_interrupt_handler,          /**<  54 CAN0 */
    can1_interrupt_handler,          /**<  55 CAN1 */
    ethernet_interrupt_handler,      /**<  56 Ethernet MAC */
    hibernation_interrupt_handler,   /**<  57 hibernation module */
    usb0_interrupt_handler,          /**<  58 USB0 */
    pwm0_3_interrupt_handler,        /**<  59 PWM0 generator 3 */
    dma_software_interrupt_handler,  /**<  60 uDMA software */
    dma_error_interrupt_handler,     /**<  61 uDMA error */
    adc1_seq0_interrupt_handler,     /**<  62 ADC1 Sequence 0 */
    adc1_seq1_interrupt_handler,     /**<  63 ADC1 Sequence 1 */
    adc1_seq2_interrupt_handler,     /**<  64 ADC1 Sequence 2 */
    adc1_seq3_interrupt_handler,     /**<  65 ADC1 Sequence 3 */
    epi_interrupt_handler,           /**<  66 EPI */
    portj_interrupt_handler,         /**<  67 GPIO port J */
    portk_interrupt_handler,         /**<  68 GPIO port K */
    portl_interrupt_handler,         /**<  69 GPIO port L */
    ssi2_interrupt_handler,          /**<  70 SSI2 */
    ssi3_interrupt_handler,          /**<  71 SSI3 */
    uart3_interrupt_handler,         /**<  72 UART3 */
    uart4_interrupt_handler,         /**<  73 UART4 */
    uart5_interrupt_handler,         /**<  74 UART5 */
    uart6_interrupt_handler,         /**<  75 UART6 */
    uart7_interrupt_handler,         /**<  76 UART7 */
    i2c2_interrupt_handler,          /**<  77 I2C2 */
    i2c3_interrupt_handler,          /**<  78 I2C3 */
    timer4a_interrupt_handler,       /**<  79 timer 4A */
    timer4b_interrupt_handler,       /**<  80 timer 4B */
    timer5a_interrupt_handler,       /**<  81 timer 5A */
    timer5b_interrupt_handler,       /**<  82 timer 5B */
    fp_interrupt_handler,            /**<  83 Floating-Point Exception */
    0,                               /**<  84 reserved */
    0,                               /**<  85 reserved */
    i2c4_interrupt_handler,          /**<  86 I2C4 */
    i2c5_interrupt_handler,          /**<  87 I2C5 */
    portm_interrupt_handler,         /**<  88 GPIO port M */
    portn_interrupt_handler,         /**<  89 GPIO port N */
    0,                               /**<  90 reserved */
    tamper_interrupt_handler,        /**<  91 tamper */
    portp_interrupt_handler,         /**<  92 GPIO port P (summary or P0) */
    portp1_interrupt_handler,        /**<  93 GPIO port P1 */
    portp2_interrupt_handler,        /**<  94 GPIO port P2 */
    portp3_interrupt_handler,        /**<  95 GPIO port P3 */
    portp4_interrupt_handler,        /**<  96 GPIO port P4 */
    portp5_interrupt_handler,        /**<  97 GPIO port P5 */
    portp6_interrupt_handler,        /**<  98 GPIO port P6 */
    portp7_interrupt_handler,        /**<  99 GPIO port P7 */
    portp_interrupt_handler,         /**< 100 GPIO port Q (summary or Q0) */
    portp1_interrupt_handler,        /**< 101 GPIO port Q1 */
    portp2_interrupt_handler,        /**< 102 GPIO port Q2 */
    portp3_interrupt_handler,        /**< 103 GPIO port Q3 */
    portp4_interrupt_handler,        /**< 104 GPIO port Q4 */
    portp5_interrupt_handler,        /**< 105 GPIO port Q5 */
    portp6_interrupt_handler,        /**< 106 GPIO port Q6 */
    portp7_interrupt_handler,        /**< 107 GPIO port Q7 */
    0,                               /**< 108 reserved */
    0,                               /**< 109 reserved */
    0,                               /**< 110 reserved */
    0,                               /**< 111 reserved */
    0,                               /**< 112  reserved */
    0,                               /**< 113 reserved */
    timer6a_interrupt_handler,       /**< 114 timer 5A */
    timer6b_interrupt_handler,       /**< 115 timer 5B */
    timer7a_interrupt_handler,       /**< 116 timer 5A */
    timer7b_interrupt_handler,       /**< 117 timer 5B */
    i2c6_interrupt_handler,          /**< 118 I2C6 */
    i2c7_interrupt_handler,          /**< 119 I2C7 */
    0,                               /**< 120 reserved */
    0,                               /**< 121 reserved */
    0,                               /**< 122 reserved */
    0,                               /**< 123 reserved */
    0,                               /**< 124 reserved */
    i2c8_interrupt_handler,          /**< 125 I2C8 */
    i2c9_interrupt_handler,          /**< 126 I2C9 */
    0,                               /**< 127 reserved */
    0,                               /**< 128 reserved */
    0,                               /**< 129 reserved */
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
void porta_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portb_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portc_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portd_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porte_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssi0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm0_fault_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm0_0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm0_1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm0_2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void qei0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_seq0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_seq1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_seq2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc0_seq3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void watchdog_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer0a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer0b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer1b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer2b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void comparator0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void comparator1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void comparator2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void system_control_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void flash_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portf_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portg_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void porth_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssi1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer3b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void qei1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void can0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void can1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ethernet_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void hibernation_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void usb0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm0_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_software_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void dma_error_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc1_seq0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc1_seq1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc1_seq2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void adc1_seq3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2s0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void epi_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portj_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portk_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portl_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssi2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void ssi3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart4_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart6_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void uart7_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer4a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer4b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer5a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer5b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer6a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer6b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer7a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void timer7b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void system_except_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c4_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c6_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c7_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c8_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c9_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portm_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portn_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp0_interrupt_handler(void) __attribute__ ((weak, alias ("portp_interrupt_handler")));
void portp1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp4_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp6_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void portp7_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_0_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_1_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_2_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_3_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void pwm1_fault_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void fp_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void tamper_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
