
#include "logging.h"

char logbuffer[256];

#ifdef MBED_USE_STDIO_LOGGING  // TARGET_LPC1768

extern "C" { void send_stdio_serial_message(const char* data); }

void log_output(char* buf, int size) {
    if (size <= 0) return;
    buf[size] = '\0';
    send_stdio_serial_message(buf);
}

#elif defined(__linux__)

#include <stdio.h>

void log_output(char* buf, int size) {
    if (size <= 0) return;
    fwrite(buf, size, 1, stderr);
    fwrite("\n", 1, 1, stderr);
}

#else

__attribute__((weak)) void log_output(char* buf, int size) {}

#endif
