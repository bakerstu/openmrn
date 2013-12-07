
#ifndef _LOGGING_H_
#define _LOGGING_H_

#ifndef __STDC_VERSION__
#define __STDC_VERSION__ 199901L
#endif

#include <stdio.h>

#define FATAL 0
/// @todo (Stuart Baker) does not build against MinGW
// #define ERROR 1
#define WARNING 2
#define INFO 3
#define VERBOSE 4

extern char logbuffer[256];

#define LOG(level, message...) do { if (LOGLEVEL >= level) { int sret = snprintf(logbuffer, sizeof(logbuffer), message); if (sret > (int)sizeof(logbuffer)) sret = sizeof(logbuffer); log_output(logbuffer, sret); } } while(0)


#ifndef LOGLEVEL
#define LOGLEVEL INFO
#endif

#ifdef __cplusplus
extern "C" {
#endif
void log_output(char* buf, int size);
void PrintErrnoAndExit(const char* where);
#ifdef __cplusplus
}
#endif


#define ERRNOCHECK(where, x...) do { if ((x) < 0) PrintErrnoAndExit(where); } while(0)

#endif // _LOGGING_H_
