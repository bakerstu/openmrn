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
