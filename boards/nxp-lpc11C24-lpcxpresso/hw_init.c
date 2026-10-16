//*****************************************************************************
//   +--+
//   | ++----+
//   +-++    |
//     |     |
//   +-+--+  |
//   | +--+--+
//   +----+    Copyright (c) 2009-12 Code Red Technologies Ltd.
//
// Microcontroller Startup code for use with Red Suite
//
// Version : 120126
//
// Software License Agreement
//
// The software is owned by Code Red Technologies and/or its suppliers, and is
// protected under applicable copyright laws.  All rights are reserved.  Any
// use in violation of the foregoing restrictions may subject the user to
// criminal
// sanctions under applicable laws, as well as to civil liability for the breach
// of the terms and conditions of this license.
//
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// USE OF THIS SOFTWARE FOR COMMERCIAL DEVELOPMENT AND/OR EDUCATION IS SUBJECT
// TO A CURRENT END USER LICENSE AGREEMENT (COMMERCIAL OR EDUCATIONAL) WITH
// CODE RED TECHNOLOGIES LTD.
//
//*****************************************************************************

#include <stdlib.h>
#include <string.h>

#include "LPC11xx.h"
#include "core_cm0.h"
#include "FreeRTOSConfig.h"

#if defined(__cplusplus)
#ifdef __REDLIB__
#error Redlib does not support C++
#else
//*****************************************************************************
//
// The entry point for the C++ library startup
//
//*****************************************************************************
#endif
#endif

const unsigned long cpu_clock_hz = 48000000;

// extern "C" {
extern void __libc_init_array(void);
//}
void raw_hw_init(void);

#define WEAK __attribute__((weak))
#define ALIAS(f) __attribute__((weak, alias(#f)))

// Code Red - if CMSIS is being used, then SystemInit() routine
// will be called by startup code rather than in application's main()
#if defined(__USE_CMSIS)
#include "system_LPC11xx.h"
#endif

//*****************************************************************************
#if defined(__cplusplus)
extern "C" {
#endif

//*****************************************************************************
//
// Forward declaration of the default handlers. These are aliased.
// When the application defines a handler (with the same name), this will
// automatically take precedence over these weak definitions
//
//*****************************************************************************
void ResetISR(void);
WEAK void NMI_Handler(void);
WEAK void HardFault_Handler(void);
WEAK void SVC_Handler(void);
WEAK void PendSV_Handler(void);
WEAK void SysTick_Handler(void);
WEAK void IntDefaultHandler(void);
//*****************************************************************************
//
// Forward declaration of the specific IRQ handlers. These are aliased
// to the IntDefaultHandler, which is a 'forever' loop. When the application
// defines a handler (with the same name), this will automatically take
// precedence over these weak definitions
//
//*****************************************************************************

void CAN_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP1_IRQHandler(void) ALIAS(IntDefaultHandler);
void I2C_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER16_0_IRQHandler(void);
void TIMER16_1_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER32_0_IRQHandler(void) ALIAS(IntDefaultHandler);
void TIMER32_1_IRQHandler(void) ALIAS(IntDefaultHandler);
void SSP0_IRQHandler(void) ALIAS(IntDefaultHandler);
void UART_IRQHandler(void) ALIAS(IntDefaultHandler);
void ADC_IRQHandler(void) ALIAS(IntDefaultHandler);
void WDT_IRQHandler(void) ALIAS(IntDefaultHandler);
void BOD_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT3_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT2_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT1_IRQHandler(void) ALIAS(IntDefaultHandler);
void PIOINT0_IRQHandler(void) ALIAS(IntDefaultHandler);
void WAKEUP_IRQHandler(void) ALIAS(IntDefaultHandler);

//*****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for redlib based applications
// main() is the entry point for newlib based applications
//
//*****************************************************************************
//
// The entry point for the application.
// __main() is the entry point for Redlib based applications
// main() is the entry point for Newlib based applications
//
//*****************************************************************************
#if defined(__REDLIB__)
extern void __main(void);
#endif
extern int main(void);
//*****************************************************************************
//
// External declaration for the pointer to the stack top from the Linker Script
//
//*****************************************************************************
extern void _vStackTop(void);

//*****************************************************************************
#if defined(__cplusplus)
} // extern "C"
#endif
//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
extern void (*const g_pfnVectors[])(void);
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    &_vStackTop,       // The initial stack pointer
    ResetISR,          // The reset handler
    NMI_Handler,       // The NMI handler
    HardFault_Handler, // The hard fault handler
    0,                 // Reserved
    0,                 // Reserved
    0,                 // Reserved
    0,                 // Reserved
    0,                 // Reserved
    0,                 // Reserved
    0,                 // Reserved
    SVC_Handler,       // SVCall handler
    0,                 // Reserved
    0,                 // Reserved
    PendSV_Handler,    // The PendSV handler
    SysTick_Handler,   // The SysTick handler

    // Wakeup sources for the I/O pins:
    //   PIO0 (0:11)
    //   PIO1 (0)
    WAKEUP_IRQHandler,    // PIO0_0  Wakeup
    WAKEUP_IRQHandler,    // PIO0_1  Wakeup
    WAKEUP_IRQHandler,    // PIO0_2  Wakeup
    WAKEUP_IRQHandler,    // PIO0_3  Wakeup
    WAKEUP_IRQHandler,    // PIO0_4  Wakeup
    WAKEUP_IRQHandler,    // PIO0_5  Wakeup
    WAKEUP_IRQHandler,    // PIO0_6  Wakeup
    WAKEUP_IRQHandler,    // PIO0_7  Wakeup
    WAKEUP_IRQHandler,    // PIO0_8  Wakeup
    WAKEUP_IRQHandler,    // PIO0_9  Wakeup
    WAKEUP_IRQHandler,    // PIO0_10 Wakeup
    WAKEUP_IRQHandler,    // PIO0_11 Wakeup
    WAKEUP_IRQHandler,    // PIO1_0  Wakeup
    CAN_IRQHandler,       // C_CAN Interrupt
    SSP1_IRQHandler,      // SPI/SSP1 Interrupt
    I2C_IRQHandler,       // I2C0
    TIMER16_0_IRQHandler, // CT16B0 (16-bit Timer 0)
    TIMER16_1_IRQHandler, // CT16B1 (16-bit Timer 1)
    TIMER32_0_IRQHandler, // CT32B0 (32-bit Timer 0)
    TIMER32_1_IRQHandler, // CT32B1 (32-bit Timer 1)
    SSP0_IRQHandler,      // SPI/SSP0 Interrupt
    UART_IRQHandler,      // UART0
    0,                    // Reserved
    0,                    // Reserved
    ADC_IRQHandler,       // ADC   (A/D Converter)
    WDT_IRQHandler,       // WDT   (Watchdog Timer)
    BOD_IRQHandler,       // BOD   (Brownout Detect)
    0,                    // Reserved
    PIOINT3_IRQHandler,   // PIO INT3
    PIOINT2_IRQHandler,   // PIO INT2
    PIOINT1_IRQHandler,   // PIO INT1
    PIOINT0_IRQHandler,   // PIO INT0
};

//*****************************************************************************
// Functions to carry out the initialization of RW and BSS data sections. These
// are written as separate functions rather than being inlined within the
// ResetISR() function in order to cope with MCUs with multiple banks of
// memory.
//*****************************************************************************
__attribute__((section(".after_vectors"))) void
data_init(unsigned int romstart, unsigned int start, unsigned int len)
{
    unsigned int* pulDest = (unsigned int*)start;
    unsigned int* pulSrc = (unsigned int*)romstart;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = *pulSrc++;
}

__attribute__((section(".after_vectors"))) void bss_init(unsigned int start,
                                                         unsigned int len)
{
    unsigned int* pulDest = (unsigned int*)start;
    unsigned int loop;
    for (loop = 0; loop < len; loop = loop + 4)
        *pulDest++ = 0;
}

#ifndef USE_OLD_STYLE_DATA_BSS_INIT
//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the location of various points in the "Global Section Table". This table is
// created by the linker via the Code Red managed linker script mechanism. It
// contains the load address, execution address and length of each RW data
// section and the execution and length of each BSS (zero initialized) section.
//*****************************************************************************
extern unsigned int __data_section_table;
extern unsigned int __data_section_table_end;
extern unsigned int __bss_section_table;
extern unsigned int __bss_section_table_end;
#else
//*****************************************************************************
// The following symbols are constructs generated by the linker, indicating
// the load address, execution address and length of the RW data section and
// the execution and length of the BSS (zero initialized) section.
// Note that these symbols are not normally used by the managed linker script
// mechanism in Red Suite/LPCXpresso 3.6 (Windows) and LPCXpresso 3.8 (Linux).
// They are provide here simply so this startup code can be used with earlier
// versions of Red Suite which do not support the more advanced managed linker
// script mechanism introduced in the above version. To enable their use,
// define "USE_OLD_STYLE_DATA_BSS_INIT".
//*****************************************************************************
extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;
#endif

extern uint32_t __start_ram;
extern uint32_t __end_ram;

//*****************************************************************************
// Reset entry point for your code.
// Sets up a simple runtime environment and initializes the C/C++
// library.
//*****************************************************************************
__attribute__((section(".after_vectors"))) __attribute__((naked)) void
ResetISR(void)
{
    // Fills the memory with a debug pattern.
    for (uint32_t* d = &__start_ram; d < &__end_ram; ++d)
    {
        *d = 0xdbdbdbdb;
    }

#ifndef USE_OLD_STYLE_DATA_BSS_INIT
    //
    // Copy the data sections from flash to SRAM.
    //
    unsigned int LoadAddr, ExeAddr, SectionLen;
    unsigned int* SectionTableAddr;

    // Load base address of Global Section Table
    SectionTableAddr = &__data_section_table;

    // Copy the data sections from flash to SRAM.
    while (SectionTableAddr < &__data_section_table_end)
    {
        LoadAddr = *SectionTableAddr++;
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        data_init(LoadAddr, ExeAddr, SectionLen);
    }
    // At this point, SectionTableAddr = &__bss_section_table;
    // Zero fill the bss segment
    while (SectionTableAddr < &__bss_section_table_end)
    {
        ExeAddr = *SectionTableAddr++;
        SectionLen = *SectionTableAddr++;
        bss_init(ExeAddr, SectionLen);
    }
#else
    // Use Old Style Data and BSS section initialization.
    // This will only initialize a single RAM bank.
    unsigned int* LoadAddr, *ExeAddr, *EndAddr, SectionLen;

    // Copy the data segment from flash to SRAM.
    LoadAddr = &_etext;
    ExeAddr = &_data;
    EndAddr = &_edata;
    SectionLen = (void*)EndAddr - (void*)ExeAddr;
    data_init((unsigned int)LoadAddr, (unsigned int)ExeAddr, SectionLen);
    // Zero fill the bss segment
    ExeAddr = &_bss;
    EndAddr = &_ebss;
    SectionLen = (void*)EndAddr - (void*)ExeAddr;
    bss_init((unsigned int)ExeAddr, SectionLen);
#endif

#ifdef __USE_CMSIS
    SystemInit();
#endif

    raw_hw_init();
    //#if defined (__cplusplus)
    //
    // Call C++ library initialisation
    //
    __libc_init_array();
//#endif

#if defined(__REDLIB__)
    // Call the Redlib library, which in turn calls main()
    __main();
#else
    main();
#endif
    //
    // main() shouldn't return, but if it does, we'll just enter an infinite
    // loop
    //
    while (1)
    {
        ;
    }
}

//*****************************************************************************
// Default exception handlers. Override the ones here by defining your own
// handler routines in your application code.
//*****************************************************************************
__attribute__((section(".after_vectors"))) void NMI_Handler(void)
{
    diewith(BLINK_DIE_NMI);
}
__attribute__((section(".after_vectors"))) void HardFault_Handler(void)
{
    diewith(BLINK_DIE_HARDFAULT);
    // setblink(BLINK_DIE_HARDFAULT);
}
__attribute__((section(".after_vectors"))) void SVC_Handler(void)
{
    diewith(BLINK_DIE_SVC);
}
__attribute__((section(".after_vectors"))) void PendSV_Handler(void)
{
    diewith(BLINK_DIE_PENDSV);
}
__attribute__((section(".after_vectors"))) void SysTick_Handler(void)
{
    diewith(BLINK_DIE_TICK);
}

//*****************************************************************************
//
// Processor ends up here if an unexpected interrupt occurs or a specific
// handler is not present in the application code.
//
//*****************************************************************************
__attribute__((section(".after_vectors"))) void IntDefaultHandler(void)
{
    diewith(BLINK_DIE_UNEXPIRQ);
}

uint32_t blinker_pattern = 0;
static uint32_t rest_pattern = 0;

void TIMER16_0_IRQHandler(void)
{
    LPC_GPIO0->MASKED_ACCESS[1 << 7] = (rest_pattern & 1) ? (1 << 7) : 0;
    rest_pattern >>= 1;
    if (!rest_pattern)
        rest_pattern = blinker_pattern;

    LPC_TMR16B0->IR = 1; // Resets interrupt.
    NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
}

void setblink(uint32_t pattern)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 7);
    blinker_pattern = pattern;
    rest_pattern = 0;
    LPC_TMR16B0->TCR = 2;  // stop & reset timer
    LPC_TMR16B0->CTCR = 0; // timer mode
    // prescale to 1 ms per tick
    LPC_TMR16B0->PR = configCPU_CLOCK_HZ / 1000; // 48000 - fits the 16bit
    LPC_TMR16B0->MR0 = 125;
    LPC_TMR16B0->MCR = 3; // reset and interrupt on match 0

    NVIC_SetPriority(TIMER_16_0_IRQn, 0);
    NVIC_EnableIRQ(TIMER_16_0_IRQn);

    LPC_TMR16B0->TCR = 1; // Timer go.
}

void resetblink(uint32_t pattern)
{
    blinker_pattern = pattern;
    rest_pattern = pattern;
    // Makes a timer event trigger immediately.
    LPC_TMR16B0->TC = LPC_TMR16B0->MR0 - 2;
}

void diewith(uint32_t pattern)
{
    SysTick->CTRL = 0; // Turns off systick to avoid task switching.
    setblink(pattern);
    __enable_irq();
    for (;;)
    {
    }
}

extern void modules_init(void);
void modules_init(void) __attribute__((weak));
void modules_init(void)
{
}
extern uint32_t* heap_end; // Used from sbrk_r implementation.

void hw_idle_hook(void)
{
    // We check that the main stack has not yet reached the top of heap.
    if (*heap_end != 0xdbdbdbdb)
    {
        diewith(BLINK_DIE_STACKOVERFLOW);
    }
}

void appl_hw_init(void)
{
    // Re-initializes the top of heap space with debug bytes.
    for (uint32_t* d = heap_end; d < &__end_ram; ++d)
    {
        *d = 0xdbdbdbdb;
    }
}

void raw_hw_init(void)
{
    /* Enable AHB clock to the GPIO domain. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 6);

    // Turns on debug LED.
    LPC_GPIO0->DIR |= (1 << 7);
    LPC_GPIO0->MASKED_ACCESS[1 << 7] = (1 << 7);
    __enable_irq();

    setblink(0x8000000A);
}

void hw_init(void)
{
    modules_init();
}

extern char __impure_data_size;

struct _reent* allocate_reent(void)
{
    int reent_size = (int)&__impure_data_size;
    struct _reent* data = malloc(reent_size);
    // This is not particularly safe, but a good approximation of how reent is
    // initialized. Unfortunately newlib-nano does not have an appropriate reent
    // header.
    memset(data, 0, reent_size);
    return data;
}

// This gets rid of about 50 kbytes of flash code that is unnecessarily
// linked into the binary.
void __wrap___cxa_pure_virtual(void)
{
    abort();
}

// This removes 400 bytes of memory allocated at startup for the atexit
// structure.
int __wrap___cxa_atexit(void)
{
    return 0;
}

void __wrap_exit(int r)
{
    abort();
}

extern void destructor(void);

void destructor() {
    diewith(0x8000AAAA);
}
