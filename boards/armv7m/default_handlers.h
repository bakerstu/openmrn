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

#ifndef SKIP_RESET_HANDLER
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
        unsigned long len = (unsigned long)*section_table_addr++;

        for (; len; len -= 4)
        {
            *dst++ = *src++;
        }
    }

    /* zero initialize bss segment(s) */
    while (section_table_addr < &__bss_section_table_end)
    {
        unsigned long *zero = (unsigned long *)*section_table_addr++;
        unsigned long len = (unsigned long)*section_table_addr++;

        for (; len; len -= 4)
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

static uint32_t g_saved_lr;
static uint32_t g_saved_pc;

__attribute__((__naked__)) static void hard_fault_handler(void)
{
    __asm volatile
    (
        " tst   lr, #4               \n"
        " ite   eq                   \n"
        " mrseq r0, msp              \n"
        " mrsne r0, psp              \n"
        // saves our return address
        " mov   r2, lr               \n"
        // Simulates a BL instruction from the original PC. Moves the PC to LR,
        // overwrites PC with our return address.
        " ldr   r3, =g_saved_lr      \n"
        " ldr   r1, [r0, 20]         \n"
        " str   r1, [r3]             \n"
        " ldr   r3, =g_saved_pc      \n"
        " ldr   r1, [r0, 24]         \n"
        " str   r1, [r3]             \n"
        " str   r1, [r0, 20]         \n"
        // Overwrites hard fault return address with our breakpoint.
        " ldr   r3, =hard_fault_stub \n"
        " str   r3, [r0, 24]         \n"
        " bx    r2\n"
//        " b     hardfault_return \n"
//        " b     hard_fault_handler_c \n"
        " bx    lr                   \n"
    );
    g_saved_lr = g_saved_pc = 0;
}

void hard_fault_stub(void) {
    __asm volatile(
        " BKPT #1\n"
        );
    while (1);
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
