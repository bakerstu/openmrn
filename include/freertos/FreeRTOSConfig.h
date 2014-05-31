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

#ifdef GCC_ARMCM3

#define configCPU_CLOCK_HZ             ( ( unsigned long ) 20000000 )
#define configMINIMAL_STACK_SIZE       ( ( unsigned short ) 50 )
#define configTOTAL_HEAP_SIZE          ( ( size_t ) ( 7000 ) )
#define configTIMER_TASK_STACK_DEPTH   256

#define configKERNEL_INTERRUPT_PRIORITY         255
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     191 /* equivalent to 0xa0, or priority 5. */


/// @todo(balazs.racz) i implemented diewith for the launchpad ek-xxx, so this is not needed anymore.
//#define diewith( x ) abort()
// Assertion facility
#ifdef __cplusplus
extern "C" {
#endif
extern void diewith(unsigned long);
extern unsigned long blinker_pattern;
#ifdef __cplusplus
}
#endif  // cplusplus

#define BLINK_DIE_UNEXPIRQ 0x800002CA // 3-1-1
#define BLINK_DIE_HARDFAULT 0x80000ACA // 3-1-2


/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names - or at least those used in the unmodified vector table. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

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

#define configTIMER_TASK_STACK_DEPTH   256

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

#elif TARGET_PIC32MX

#define configCPU_CLOCK_HZ             ( ( unsigned long ) 80000000 )
#define configMINIMAL_STACK_SIZE       ( ( unsigned short ) 190 )
#define configTOTAL_HEAP_SIZE          ( ( size_t ) ( 32000 ) )
#define configTIMER_TASK_STACK_DEPTH   1500
#define configISR_STACK_SIZE					( 400 )
#define configPERIPHERAL_CLOCK_HZ      ( ( unsigned long ) configCPU_CLOCK_HZ/2 )

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
#define configUSE_CO_ROUTINES          0
#define configCHECK_FOR_STACK_OVERFLOW 2

#define configMAX_PRIORITIES        ( ( unsigned portBASE_TYPE ) 5 )
#define configMAX_CO_ROUTINE_PRIORITIES ( 2 )

#define configUSE_APPLICATION_TASK_TAG 1

#ifdef TARGET_LPC11Cxx
#define configUSE_TIMERS               0
#else
#define configUSE_TIMERS               1
#endif
#define configTIMER_QUEUE_LENGTH       16
#define configTIMER_TASK_PRIORITY      (configMAX_PRIORITIES/2)
#define INCLUDE_xTimerGetTimerDaemonTaskHandle 1

#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_pcTaskGetTaskName 1

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */

#define INCLUDE_vTaskPrioritySet        0
#define INCLUDE_uxTaskPriorityGet       0
#define INCLUDE_vTaskDelete             0
#define INCLUDE_vTaskCleanUpResources   1
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1
#define INCLUDE_xTaskGetIdleTaskHandle   1

#ifndef __LANGUAGE_ASSEMBLY__

typedef struct task_switched_in
{
    struct _reent *reent; /**< newlib thread specific data (errno, etc...) */
} TaskSwitchedIn;

/* We can use this hook in order to change out the newlib struct __reent fo
 * each thread.  This is setup in os_thread_create. */
#define traceTASK_SWITCHED_IN()                                           \
{                                                                         \
    TaskSwitchedIn *task_switched_in;                                      \
    task_switched_in = (TaskSwitchedIn*)(prvGetTCBFromHandle(NULL)->pxTaskTag); \
    if (task_switched_in) _impure_ptr = task_switched_in->reent;         \
}

#endif

#ifndef diewith

#define BLINK_DIE_OUTOFMEM 0x80008CCA // 3-2-1
#define BLINK_DIE_ASSERT 0x80028CCA  // 3-2-2
#define BLINK_DIE_STACKOVERFLOW 0x800A8CCA  // 3-2-3
#define BLINK_DIE_OUTOFMEMSTACK 0x802A8CCA  // 3-2-4
#define BLINK_DIE_STACKCOLLIDE 0x80AA8CCA  // 3-2-5

#define BLINK_DIE_ABORT 0x8000CCCA  // 3-3
#define BLINK_DIE_WATCHDOG 0x8002CCCA // 3-3-1
#define BLINK_DIE_STARTUP 0x800ACCCA // 3-3-2

#endif


#endif /* FREERTOS_CONFIG_H */
