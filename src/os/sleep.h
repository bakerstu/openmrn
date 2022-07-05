/** \copyright
 * Copyright (c) 2021, Balazs Racz
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
 * \file sleep.h
 * 
 * Declares cross-platform sleep functions.
 *
 * @author Balazs Racz
 * @date 10 Sep 2021
 */

#ifndef _OS_SLEEP_H_
#define _OS_SLEEP_H_

#include <inttypes.h>

#ifndef EMSCRIPTEN
/// Sleep a given number of microseconds. The granularity of sleep depends on
/// the operating system, for FreeRTOS sleeping less than 1 msec is not
/// possible with this function.
/// @param microseconds how long to sleep.
static void microsleep(uint32_t microseconds) __attribute__((weakref("usleep")));
#endif

/// Executes a busy loop for a given amount of time. It is recommended to use
/// this only for small number of microseconds (e.g. <100 usec).
/// @param microseconds how long to delay.
extern "C" void microdelay(uint32_t microseconds);

#endif // _OS_SLEEP_H_
