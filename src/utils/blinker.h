/** \copyright
 * Copyright (c) 2013, Balazs Racz
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
 * \file blinker.h
 *
 * An portable interface to access a single LED on hardware boards.
 *
 * @author Balazs Racz
 * @date 31 May 2014
 */

#ifndef _UTILS_BLINKER_H_
#define _UTILS_BLINKER_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLINK_DIE_UNEXPIRQ 0x800002CA // 3-1-1
#define BLINK_DIE_HARDFAULT 0x80000ACA // 3-1-2
#define BLINK_DIE_NMI 0x8002A0CA       /* 3-1-3 */
#define BLINK_DIE_SVC 0x800AA0CA       /* 3-1-4 */
#define BLINK_DIE_PENDSV 0x802AA0CA    /* 3-1-5 */
#define BLINK_DIE_TICK 0x80AAA0CA      /* 3-1-6 */

#define BLINK_DIE_OUTOFMEM 0x80008CCA // 3-2-1
#define BLINK_DIE_ASSERT 0x80028CCA  // 3-2-2
#define BLINK_DIE_STACKOVERFLOW 0x800A8CCA  // 3-2-3
#define BLINK_DIE_OUTOFMEMSTACK 0x802A8CCA  // 3-2-4
#define BLINK_DIE_STACKCOLLIDE 0x80AA8CCA  // 3-2-5

#define BLINK_DIE_ABORT 0x8000CCCA  // 3-3
#define BLINK_DIE_WATCHDOG 0x8002CCCA // 3-3-1
#define BLINK_DIE_STARTUP 0x800ACCCA // 3-3-2

/** Initializes the blinker routine with a specific blinking pattern.
 *
 * @param pattern is the blinking pattern. */
extern void setblink(uint32_t);

/** Changes the blinking pattern.
 *
 * @param pattern is the new blinking pattern. */
extern void resetblink(uint32_t);

/** The currently active blinking pattern. */
extern uint32_t blinker_pattern;

/** Sets a blinking pattern and never returns. */
extern void diewith(uint32_t);

#ifdef __cplusplus
}
#endif  // cplusplus

#endif // _UTILS_BLINKER_H_
