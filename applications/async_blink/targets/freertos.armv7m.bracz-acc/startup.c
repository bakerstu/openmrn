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
extern void wide_timer0a_interrupt_handler(void);
extern void wide_timer0b_interrupt_handler(void);
extern void wide_timer1a_interrupt_handler(void);
extern void wide_timer1b_interrupt_handler(void);
extern void wide_timer2a_interrupt_handler(void);
extern void wide_timer2b_interrupt_handler(void);
extern void wide_timer3a_interrupt_handler(void);
extern void wide_timer3b_interrupt_handler(void);
extern void wide_timer4a_interrupt_handler(void);
extern void wide_timer4b_interrupt_handler(void);
extern void wide_timer5a_interrupt_handler(void);
extern void wide_timer5b_interrupt_handler(void);
extern void system_except_interrupt_handler(void);
extern void i2c4_interrupt_handler(void);
extern void i2c5_interrupt_handler(void);
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
    qei1_interrupt_handler,          /**<  54 QEI1 */
    can0_interrupt_handler,          /**<  55 CAN0 */
    can1_interrupt_handler,          /**<  56 CAN1 */
    0,                               /**<  57 reserved */
    ethernet_interrupt_handler,      /**<  58 ethernet controller */
    hibernation_interrupt_handler,   /**<  59 hibernation module */
    usb0_interrupt_handler,          /**<  60 USB */
    pwm0_3_interrupt_handler,        /**<  61 PWM0 generator 3 */
    dma_software_interrupt_handler,  /**<  62 uDMA software */
    dma_error_interrupt_handler,     /**<  63 uDMA error */
    adc1_seq0_interrupt_handler,     /**<  64 ADC1 Sequence 0 */
    adc1_seq1_interrupt_handler,     /**<  65 ADC1 Sequence 1 */
    adc1_seq2_interrupt_handler,     /**<  66 ADC1 Sequence 2 */
    adc1_seq3_interrupt_handler,     /**<  67 ADC1 Sequence 3 */
    i2s0_interrupt_handler,          /**<  68 I2S0 */
    epi_interrupt_handler,           /**<  69 EPI */
    portj_interrupt_handler,         /**<  70 GPIO port J */
    portk_interrupt_handler,         /**<  71 GPIO port K */
    portl_interrupt_handler,         /**<  72 GPIO port L */
    ssi2_interrupt_handler,          /**<  73 SSI2 */
    ssi3_interrupt_handler,          /**<  74 SSI3 */
    uart3_interrupt_handler,         /**<  75 UART3 */
    uart4_interrupt_handler,         /**<  76 UART4 */
    uart5_interrupt_handler,         /**<  77 UART5 */
    uart6_interrupt_handler,         /**<  78 UART6 */
    uart7_interrupt_handler,         /**<  79 UART7 */
    0,                               /**<  80 reserved */
    0,                               /**<  81 reserved */
    0,                               /**<  82 reserved */
    0,                               /**<  83 reserved */
    i2c2_interrupt_handler,          /**<  84 I2C2 */
    i2c3_interrupt_handler,          /**<  85 I2C3 */
    timer4a_interrupt_handler,       /**<  86 timer 4A */
    timer4b_interrupt_handler,       /**<  87 timer 4B */
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
    timer5a_interrupt_handler,       /**< 108 timer 5A */
    timer5b_interrupt_handler,       /**< 109 timer 5B */
    wide_timer0a_interrupt_handler,  /**< 110 wide timer 0A */
    wide_timer0b_interrupt_handler,  /**< 111 wide timer 0B */
    wide_timer1a_interrupt_handler,  /**< 112 wide timer 1A */
    wide_timer1b_interrupt_handler,  /**< 113 wide timer 1B */
    wide_timer2a_interrupt_handler,  /**< 114 wide timer 2A */
    wide_timer2b_interrupt_handler,  /**< 115 wide timer 2B */
    wide_timer3a_interrupt_handler,  /**< 116 wide timer 3A */
    wide_timer3b_interrupt_handler,  /**< 117 wide timer 3B */
    wide_timer4a_interrupt_handler,  /**< 118 wide timer 4A */
    wide_timer4b_interrupt_handler,  /**< 119 wide timer 4B */
    wide_timer5a_interrupt_handler,  /**< 120 wide timer 5A */
    wide_timer5b_interrupt_handler,  /**< 121 wide timer 5B */
    system_except_interrupt_handler, /**< 122 system exception (imprecise) */
    0,                               /**< 123 reserved */
    0,                               /**< 124 reserved */
    i2c4_interrupt_handler,          /**< 125 I2C2 */
    i2c5_interrupt_handler,          /**< 126 I2C3 */
    portm_interrupt_handler,         /**< 127 GPIO port M */
    portn_interrupt_handler,         /**< 128 GPIO port N */
    0,                               /**< 129 reserved */
    0,                               /**< 130 reserved */
    0,                               /**< 131 reserved */
    portp_interrupt_handler,         /**< 132 GPIO port P (summary or P0) */
    portp1_interrupt_handler,        /**< 133 GPIO port P1 */
    portp2_interrupt_handler,        /**< 134 GPIO port P2 */
    portp3_interrupt_handler,        /**< 135 GPIO port P3 */
    portp4_interrupt_handler,        /**< 136 GPIO port P4 */
    portp5_interrupt_handler,        /**< 137 GPIO port P5 */
    portp6_interrupt_handler,        /**< 138 GPIO port P6 */
    portp7_interrupt_handler,        /**< 139 GPIO port P7 */
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
    pwm1_0_interrupt_handler,        /**< 150 PWM1 generator 0 */
    pwm1_1_interrupt_handler,        /**< 151 PWM1 generator 1 */
    pwm1_2_interrupt_handler,        /**< 152 PWM1 generator 2 */
    pwm1_2_interrupt_handler,        /**< 153 PWM1 generator 3 */
    pwm1_fault_interrupt_handler,    /**< 154 PWM1 fault */
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
volatile int wait_for_continue = 0;
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

    __asm("BKPT #0\n") ; // Break into the debugger

    resetblink(BLINK_DIE_HARDFAULT);
    while (!wait_for_continue) {}
    // Masks all interrupts before returning.
    NVIC_INT_DIS_BASE[0] = 0xFFFFFFFFU;
    NVIC_INT_DIS_BASE[1] = 0xFFFFFFFFU;
    NVIC_INT_DIS_BASE[2] = 0xFFFFFFFFU;
    NVIC_INT_DIS_BASE[3] = 0xFFFFFFFFU;
    NVIC_INT_DIS_BASE[4] = 0xFFFFFFFFU;
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
    diewith(BLINK_DIE_UNEXPIRQ);
    while(1);
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
void wide_timer0a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer0b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer1a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer1b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer2a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer2b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer3a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer3b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer4a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer4b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer5a_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void wide_timer5b_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void system_except_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c4_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
void i2c5_interrupt_handler(void) __attribute__ ((weak, alias ("default_interrupt_handler")));
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
