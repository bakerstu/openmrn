/* ISRwrapper.s - PIC32MX - Instantiate wrappers for C-language ISRs
**
** Define ISR wrapper and vector branch code to support "C" ISRs.
** See "ISRwrapper.inc" for details...
**
** History:
**   20090830 DRNadler: CPP demo example
**
**
** Copyright (c) 2009 Dave Nadler
**
** The authors hereby grant permission to use, copy, modify, distribute,
** and license this software and its documentation for any purpose, provided
** that existing copyright notices are retained in all copies and that this
** notice is included verbatim in any distributions.  No written agreement,
** license, or royalty fee is required for any of the authorized uses.
** Modifications to this software may be copyrighted by their authors
** and need not follow the licensing terms described here, provided that
** the new terms are clearly indicated on the first page of each file where
** they apply.
*/

#include "proc/p32mx795f512h.h"
.include "ISRwrapper.inc"
        

        RAW_ISR_wrapper _TIMER_1_VECTOR,vPortTickInterruptHandler
        RAW_ISR_wrapper _CORE_SOFTWARE_0_VECTOR,vPortYieldISR
        
// See p32mx440f256h.h etc. for vector assignments for specific processor.
// Create wrappers for ISRs used in a specific application.
//	ISR_wrapper  4,c_vector_4_ISR   // timer 1 (POSIX time emulation, delay manager support)
//	ISR_wrapper	24,c_vector_24_ISR  // UART 1 - RS232 (SN10, diagnostics)
