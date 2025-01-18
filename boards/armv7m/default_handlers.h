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
 * \file default_handlers.h
 * Contains definitions for the standard handlers of a Cortex-M3 processor.
 *
 * @author Balazs Racz
 * @date 10 June 2015
 */

#ifdef BOARDS_DEFAULT_HANDLERS_H
#error Only include defualt_handlers into the toplevel startup.c
#endif

#define BOARDS_DEFAULT_HANDLERS_H

#include "utils/macros.h"

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

extern void ignore_fn(void);

extern unsigned long __data_section_table;
extern unsigned long __data_section_table_end;
extern unsigned long __bss_section_table;
extern unsigned long __bss_section_table_end;

#define NVIC_INT_CTRL_R (*((volatile uint32_t *)0xE000ED04))

/** This hardware initialization code will be called before C++ global objects
 * are initialized. */
extern void hw_preinit(void);
extern void resetblink(unsigned long pattern);

/** Sets the hardware outputs to a safe state. Called when the program crashes
 * handler. */
/*void hw_set_to_safe(void) __attribute__ ((weak));
void hw_set_to_safe(void)
{
}*/
extern void hw_set_to_safe(void);

#ifndef SKIP_RESET_HANDLER
/** Startup the C/C++ runtime environment.
 */
void reset_handler(void)
{
    __asm("cpsid i\n");

    unsigned long *section_table_addr = &__data_section_table;

    /* copy ram load sections from flash to ram */
    while (section_table_addr < &__data_section_table_end)
    {
        unsigned long *src = (unsigned long *)*section_table_addr++;
        unsigned long *dst = (unsigned long *)*section_table_addr++;
        long len = (long)*section_table_addr++;

        for (; len > 0; len -= 4)
        {
            *dst++ = *src++;
        }
    }

    /* zero initialize bss segment(s) */
    while (section_table_addr < &__bss_section_table_end)
    {
        unsigned long *zero = (unsigned long *)*section_table_addr++;
        long len = (unsigned long)*section_table_addr++;

        for (; len > 0; len -= 4)
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

    for (; /* forever */;)
    {
        /* if we ever return from main, loop forever */
    }
}
#endif // SKIP_RESET_HANDLER

void hard_fault_handler_step_2(unsigned long *hardfault_args);
void hard_fault_handler_step_3(void);


__attribute__((__naked__)) static void hard_fault_handler(void)
{
// set switch to 0 in order to alternatively recreate the previous stack frame
// and halt the CPU
#if 1
    __asm volatile
    (
        " mov r0, #4                 \n"
        " mov r1, lr                 \n"
        " tst r0, r1                 \n"

#if 1 // code rewritten for ARMv6m
        " bne has_bit_two            \n"
        " mrs r0, msp                \n"
        " b test_done                \n"
        "has_bit_two:                \n"
        " mrs r0, psp                \n"
        "test_done:                  \n"
#else // original code        
        " ite   eq                   \n"  // check if LR & (1<<2)
        " mrseq r0, msp              \n"  // if 0 (bit clear), load msp
        " mrsne r0, psp              \n"  // if nonzero (bit set), load psp
#endif
        
        " ldr r1, [r0, #24]          \n"
        " ldr r2, =hard_fault_handler_step_2 \n"
        " bx r2 \n");
#else
    __asm volatile
    (
        " tst   lr, #4 \n"

#if 1 // code rewritten for ARMv6m
        " bnz has_bit_two            \n"
        " mrs r0, msp                \n"
        " b test_done                \n"
        "has_bit_two:                \n"
        " mrs r0, psp                \n"
        "test_done:                  \n"
#else // original code        
        " ite   eq                   \n"  // check if LR & (1<<2)
        " mrseq r0, msp              \n"  // if 0 (bit clear), load msp
        " mrsne r0, psp              \n"  // if nonzero (bit set), load psp
#endif

        " mov   sp, r0 \n"
        " bkpt  #1     \n"
        " bx    lr     \n"
    );
#endif
#if 0
    // saves our return address
        " mov   r2, lr               \n"
        // Simulates a BL instruction from the original PC. Moves the PC to LR,
        // overwrites PC with our return address.
        " ldr   r3, =g_saved_lr      \n"
        " ldr   r1, [r0, #20]         \n"
        " str   r1, [r3]             \n"
        " ldr   r3, =g_saved_pc      \n"
        " ldr   r1, [r0, #24]         \n"
        " str   r1, [r3]             \n"
        " str   r1, [r0, #20]         \n"
        // Overwrites hard fault return address with our breakpoint.
        " ldr   r3, =hard_fault_stub \n"
        " str   r3, [r0, #24]         \n"
        " bx    r2\n"
//        " b     hardfault_return \n"
//        " b     hard_fault_handler_c \n"
        " bx    lr                   \n"
    );
    g_saved_lr = g_saved_pc = 0;
#endif

}

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
static uint8_t seenFault = 0;

__attribute__((optimize("-O0"),unused)) void hard_fault_handler_step_2(unsigned long *hardfault_args)
{
    /* force a reference in the local variables for debug */
    volatile FaultInformation *fault_info = &faultInfo;
    if (!seenFault)
    {
        seenFault = 1;
        fault_info->stacked_r0 = ((unsigned long)hardfault_args[0]);
        fault_info->stacked_r1 = ((unsigned long)hardfault_args[1]);
        fault_info->stacked_r2 = ((unsigned long)hardfault_args[2]);
        fault_info->stacked_r3 = ((unsigned long)hardfault_args[3]);
        fault_info->stacked_r12 = ((unsigned long)hardfault_args[4]);
        fault_info->stacked_lr = ((unsigned long)hardfault_args[5]);
        fault_info->stacked_pc = ((unsigned long)hardfault_args[6]);
        fault_info->stacked_psr = ((unsigned long)hardfault_args[7]);

        // Configurable Fault Status Register
        // Consists of MMSR, BFSR and UFSR
        fault_info->_CFSR = (*((volatile unsigned long *)(0xE000ED28)));

        // Hard Fault Status Register
        fault_info->_HFSR = (*((volatile unsigned long *)(0xE000ED2C)));

        // Debug Fault Status Register
        fault_info->_DFSR = (*((volatile unsigned long *)(0xE000ED30)));

        // Auxiliary Fault Status Register
        fault_info->_AFSR = (*((volatile unsigned long *)(0xE000ED3C)));

        // Read the Fault Address Registers. These may not contain valid values.
        // Check BFARVALID/MMARVALID to see if they are valid values
        // MemManage Fault Address Register
        fault_info->_MMAR = (*((volatile unsigned long *)(0xE000ED34)));
        // Bus Fault Address Register
        fault_info->_BFAR = (*((volatile unsigned long *)(0xE000ED38)));
    }
    else
    {
        // Double fault. Let's not try doing anything smart anymore, just halt.
        while (1)
        {
            extern void wait_with_blinker(void);
            wait_with_blinker();
        }
    }

    hw_set_to_safe();
    __asm volatile ("cpsid i\n");

    // Simulates a BL instruction from the original PC. Moves the PC to LR,
    // overwrites PC with our return address.
    hardfault_args[5] = hardfault_args[6];
    hardfault_args[6] = (unsigned long)&hard_fault_handler_step_3;

    resetblink(BLINK_DIE_HARDFAULT);

    C_STATIC_ASSERT(((uint8_t*)&faultInfo) + 12 == ((uint8_t*)&faultInfo.stacked_r3), faultinfo_memory_layout_not_as_expected);

    // Returning from here will pop the exception stack and get to
    // hard_fault_handler_step_3.
}

/// This function will be called in an infinite loop from the hard fualt handler.
///
/// Define it in HwInit.cxx to enable blinking during hard faults. The function
/// should check the timer interrupt flag and if set, call the timer interrupt
/// handler inline, then return.
void wait_with_blinker(void) __attribute__ ((weak));
void wait_with_blinker(void)
{
    // noop, but disables all interrupts. Normally there would be an
    // implementation of this weak function in HwInit.cxx.
    __asm volatile("cpsid i");
}

void hard_fault_handler_step_3(void) {
    const uint32_t C_DEBUGEN = 0x00000001;
    uint32_t debugreg = *(volatile uint32_t*)0xE000EDF0;
    if (debugreg & C_DEBUGEN) {
        __asm volatile(
            " ldr r0, =faultInfo \n"
            " ldr r3, [r0, #12]   \n"
            " ldr r2, [r0, #8]    \n"
            " ldr r1, [r0, #4]    \n"
            " ldr r0, [r0, #0]    \n"
            " BKPT #1            \n"
            ::: "r0", "r1", "r2", "r3");
    }
    while (1)
    {
        // In gdb use `break hard_fault_debug` to get the best possible
        // backtrace if you find a target in this infinite loop.
        __asm volatile (
            " cpsid i\n"
            " ldr r0, =faultInfo \n"
            " ldr r3, [r0, #12]   \n"
            " ldr r2, [r0, #8]    \n"
            " ldr r1, [r0, #4]    \n"
            " ldr r0, [r0, #0]    \n"
            " nop                \n"
            " nop                \n"
            " .global hard_fault_debug \n"
            "hard_fault_debug:   \n"
            " nop                \n"
            ::: "r0", "r1", "r2", "r3"
            );
        wait_with_blinker();
    }
}

void nmi_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}

void mpu_fault_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}

void bus_fault_handler(void)
{
    for ( ; /* forever */ ; )
    {
    }
}

void usage_fault_handler(void)
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
