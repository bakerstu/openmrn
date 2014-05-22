
#ifndef _LOGGING_H_
#define _LOGGING_H_

#ifndef __cplusplus
#ifndef __STDC_VERSION__
#define __STDC_VERSION__ 199901L
#endif
#endif

#include <stdio.h>
#include <inttypes.h>

static const int FATAL = 0;
static const int ERROR = 1;
static const int WARNING = 2;
static const int INFO = 3;
static const int VERBOSE = 4;

#define LOG(level, message...)                                                 \
    do                                                                         \
    {                                                                          \
        if (LOGLEVEL >= level)                                                 \
        {                                                                      \
            int sret = snprintf(logbuffer, sizeof(logbuffer), message);        \
            if (sret > (int)sizeof(logbuffer))                                 \
                sret = sizeof(logbuffer);                                      \
            log_output(logbuffer, sret);                                       \
        }                                                                      \
    } while (0)

extern char logbuffer[256];

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

#endif // _LOGGING_H_
