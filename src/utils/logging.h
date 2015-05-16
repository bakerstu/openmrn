/** \copyright
 * Copyright (c) 2013, Stuart W Baker
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
 * \file logging.h
 * Facility to do debug printf's on a configurable loglevel.
 *
 * @author Balazs Racz
 * @date 3 August 2013
 */

#ifndef _UTILS_LOGGING_H_
#define _UTILS_LOGGING_H_

#ifndef __cplusplus
#ifndef __STDC_VERSION__
#define __STDC_VERSION__ 199901L
#endif
#endif

#include <stdio.h>
#include <inttypes.h>
#include "os/os.h"

static const int FATAL = 0;
static const int ERROR = 1;
static const int WARNING = 2;
static const int INFO = 3;
static const int VERBOSE = 4;

#if defined(__linux__) || defined(GCC_ARMCM3) || defined (GCC_ARMCM0)
#define LOCKED_LOGGING
#endif

#ifdef LOCKED_LOGGING
extern os_mutex_t g_log_mutex;
#define LOCK_LOG os_mutex_lock(&g_log_mutex)
#define UNLOCK_LOG os_mutex_unlock(&g_log_mutex)
#else
#define LOCK_LOG
#define UNLOCK_LOG
#endif

#ifdef __cplusplus
#define GLOBAL_LOG_OUTPUT ::log_output
#else
#define GLOBAL_LOG_OUTPUT log_output
#endif

#define LOG(level, message...)                                                 \
    do                                                                         \
    {                                                                          \
        if (LOGLEVEL >= level)                                                 \
        {                                                                      \
            LOCK_LOG;                                                          \
            int sret = snprintf(logbuffer, sizeof(logbuffer), message);        \
            if (sret > (int)sizeof(logbuffer))                                 \
                sret = sizeof(logbuffer);                                      \
            GLOBAL_LOG_OUTPUT(logbuffer, sret);                                \
            UNLOCK_LOG;                                                        \
        }                                                                      \
    } while (0)

#ifdef __linux__
extern char logbuffer[4096];
#else
extern char logbuffer[256];
#endif

#ifndef LOGLEVEL
#ifdef __FreeRTOS__
#define LOGLEVEL FATAL
#else
#define LOGLEVEL INFO
#endif // not FreeRTOS
#endif // ifndef LOGLEVEL

#ifdef __cplusplus
extern "C" {
#endif
void log_output(char *buf, int size);
void print_errno_and_exit(const char *where);
#ifdef __cplusplus
}
#endif

#define ERRNOCHECK(where, x...)                                                \
    do                                                                         \
    {                                                                          \
        if ((x) < 0)                                                           \
        {                                                                      \
            print_errno_and_exit(where);                                       \
        }                                                                      \
    } while (0)

#endif // _UTILS_LOGGING_H_
