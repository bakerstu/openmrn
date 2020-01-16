/*
    FreeRTOS V7.3.0 - Copyright (C) 2012 Real Time Engineers Ltd.

    FEATURES AND PORTS ARE ADDED TO FREERTOS ALL THE TIME.  PLEASE VISIT 
    http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!
    
    ***************************************************************************
     *                                                                       *
     *    Having a problem?  Start by reading the FAQ "My application does   *
     *    not run, what could be wrong?"                                     *
     *                                                                       *
     *    http://www.FreeRTOS.org/FAQHelp.html                               *
     *                                                                       *
    ***************************************************************************

    
    http://www.FreeRTOS.org - Documentation, training, latest versions, license 
    and contact details.  
    
    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool.

    Real Time Engineers ltd license FreeRTOS to High Integrity Systems, who sell 
    the code with commercial support, indemnification, and middleware, under 
    the OpenRTOS brand: http://www.OpenRTOS.com.  High Integrity Systems also
    provide a safety engineered and independently SIL3 certified version under 
    the SafeRTOS brand: http://www.SafeRTOS.com.
*/

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifndef __LANGUAGE_ASSEMBLY__
#include "utils/blinker.h"
#endif


/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

/* ***************************************************************************
 * Cortex-M3 specific defines
 *************************************************************************** */
#if defined(GCC_ARMCM3)

#define configCPU_CLOCK_HZ             ( cm3_cpu_clock_hz )
#define configMINIMAL_STACK_SIZE       ( ( unsigned short ) 256 )
#define configTOTAL_HEAP_SIZE          ( ( size_t ) ( 7000 ) )
#define configTIMER_TASK_STACK_DEPTH   256

#define configKERNEL_INTERRUPT_PRIORITY         255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     0xa0 /* equivalent to 191, or priority 5. */

// change #if to 1 in order to enable asserts for the kernel
#if 1
#ifdef __cplusplus
extern "C" {
#endif
extern int g_death_lineno;
#ifdef __cplusplus
}
#endif  // cplusplus
#define configASSERT( x ) do { if (!(x)) { g_death_lineno = __LINE__; diewith(BLINK_DIE_ASSERT); }} while(0)
#endif

/// @todo(balazs.racz) i implemented diewith for the launchpad ek-xxx, so this is not needed anymore.
//#define diewith( x ) abort()
// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern const unsigned long cm3_cpu_clock_hz;
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* ***************************************************************************
 * Cortex-M0 specific defines
 *************************************************************************** */
#elif defined (GCC_ARMCM0)

#define configCPU_CLOCK_HZ             ( cpu_clock_hz )
#define configMINIMAL_STACK_SIZE       ( ( unsigned short ) 256 )
//#define configTOTAL_HEAP_SIZE          ( ( size_t ) ( 7000 ) )
#define configTIMER_TASK_STACK_DEPTH   256

// change #if to 1 in order to enable asserts for the kernel
#if 0
#ifdef __cplusplus
extern "C" {
#endif
extern int g_death_lineno;
#ifdef __cplusplus
}
#endif  // cplusplus
#define configASSERT( x ) do { if (!(x)) { g_death_lineno = __LINE__; diewith(BLINK_DIE_ASSERT); }} while(0)
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern const unsigned long cpu_clock_hz;
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler


/* ***************************************************************************
 * LPC2368 specific defines
 *************************************************************************** */
#elif defined(TARGET_LPC2368)

#include "lpc23xx.h"

// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus
#define configASSERT( x ) if (!(x)) diewith(BLINK_DIE_ASSERT)

/* Value to use on old rev '-' devices. */
//#define configPINSEL2_VALUE   0x50151105

/* Value to use on rev 'A' and newer devices. */
#define configPINSEL2_VALUE     0x50150105

#ifndef configPINSEL2_VALUE
        #error Please uncomment one of the two configPINSEL2_VALUE definitions above, depending on the revision of the LPC2000 device being used.
#endif

#define configCPU_CLOCK_HZ          ( ( unsigned long ) 48000000 )      /* =12Mhz xtal multiplied by 5 using the PLL. */
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 104 )
#define configTOTAL_HEAP_SIZE           ( ( size_t ) ( 18 * 1024 ) )

#define configTIMER_TASK_STACK_DEPTH   384 //256

#elif defined(TARGET_LPC1768)

// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus
#define configASSERT( x ) if (!(x)) diewith(BLINK_DIE_ASSERT)

#define configCPU_CLOCK_HZ          ( ( unsigned long ) 96000000 )      /* =12Mhz xtal multiplied by 5 using the PLL. */
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 104 )
#define configTOTAL_HEAP_SIZE           ( ( size_t ) ( 18 * 1024 ) )

#define configTIMER_TASK_STACK_DEPTH   256

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define configKERNEL_INTERRUPT_PRIORITY         255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     191 /* equivalent to 0xa0, or priority 5. */

#elif defined(TARGET_LPC11Cxx)

#define configTIMER_TASK_STACK_DEPTH   80

// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif
#define configASSERT( x ) if (!(x)) diewith(BLINK_DIE_ASSERT)

#define configCPU_CLOCK_HZ          ( ( unsigned long ) 48000000 )      /* =12Mhz xtal multiplied by 5 using the PLL. */
/* Idle task stack uses this size */
#define configMINIMAL_STACK_SIZE        ( ( unsigned short ) 33 )
#define configTOTAL_HEAP_SIZE           ( ( size_t ) ( 3000 ) )

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#elif defined(TARGET_PIC32MX)

#define MIPSNO16 __attribute__((nomips16))

#define configCPU_CLOCK_HZ             ( pic32_cpu_clock_hz )
#define configPERIPHERAL_CLOCK_HZ      ( pic32_periph_clock_hz )
#define configMINIMAL_STACK_SIZE       ( 90 )
#define configISR_STACK_SIZE           ( 512 )
#define configTOTAL_HEAP_SIZE          ( ( size_t ) 9000 )
#define configTIMER_TASK_STACK_DEPTH   ( 190 )
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configCHECK_FOR_STACK_OVERFLOW 3

/* The priority at which the tick interrupt runs.  This should probably be
kept at 1. */
#define configKERNEL_INTERRUPT_PRIORITY			0x01

/* The maximum interrupt priority from which FreeRTOS.org API functions can
be called.  Only API functions that end in ...FromISR() can be used within
interrupts. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY	0x03

#ifndef __LANGUAGE_ASSEMBLY__
// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern const unsigned long pic32_cpu_clock_hz;
extern const unsigned long pic32_periph_clock_hz;
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus
#define configASSERT( x ) if (!(x)) diewith(BLINK_DIE_ASSERT)
#endif // assembly

#else

#error please provide the FreeRTOSConfig.h for your target

#endif // Switch target

/* ***************************************************************************
 * Common defines used for all targets
 *************************************************************************** */
#define configTICK_RATE_HZ             ( ( portTickType ) 953 )
#define NSEC_TO_TICK_SHIFT             20
#define configUSE_PREEMPTION           1
#define configUSE_IDLE_HOOK            1
#define configUSE_TICK_HOOK            0

#define configMAX_TASK_NAME_LEN        ( 16 )
#define configUSE_TRACE_FACILITY       0
#define configUSE_16_BIT_TICKS         0
#define configIDLE_SHOULD_YIELD        0
#define configUSE_MUTEXES              1
#define configUSE_RECURSIVE_MUTEXES    1
#define configUSE_COUNTING_SEMAPHORES  1
#define configUSE_NEWLIB_REENTRANT     1
#define configUSE_CO_ROUTINES          0
#ifndef configCHECK_FOR_STACK_OVERFLOW
#define configCHECK_FOR_STACK_OVERFLOW 2
#endif

#define configMAX_PRIORITIES            ( 5 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

#define configUSE_APPLICATION_TASK_TAG 1

#define configUSE_TIMERS               1
#define configTIMER_QUEUE_LENGTH       16
#define configTIMER_TASK_PRIORITY      (configMAX_PRIORITIES - 2)
#define INCLUDE_xTimerGetTimerDaemonTaskHandle 1

#if tskKERNEL_VERSION_MAJOR >= 9
#define configSUPPORT_STATIC_ALLOCATION     1
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#endif

/* Enable thread local storage, only active on FreeRTOS 9.x+ */
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 1
#define TLS_INDEX_SELECT_EVENT_BIT 0 /* Wakeup event for platform select() */

/* backwards compatibility */
#if !defined(vPortClearInterruptMask)
    #define vPortClearInterruptMask(x)      vPortSetBASEPRI(x)
#endif

#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_pcTaskGetTaskName 1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet        1
#define INCLUDE_uxTaskPriorityGet       1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskCleanUpResources   1
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1
#define INCLUDE_xTaskGetIdleTaskHandle   1
#define INCLUDE_xEventGroupSetBitFromISR 1
#define INCLUDE_xTimerPendFunctionCall  1
#ifndef __LANGUAGE_ASSEMBLY__
#ifndef TARGET_LPC11Cxx
/** This trace macro is called from the tick interrupt; we use it for
 * collecting CPU load information. */
void cpuload_tick(unsigned);
//#define traceTASK_INCREMENT_TICK( count ) cpuload_tick()
#endif

#endif

#ifndef MIPSNO16
#define MIPSNO16
#endif

#endif /* FREERTOS_CONFIG_H */
