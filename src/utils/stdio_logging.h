/// Include this file into main.cxx to log stuff to stdio on FreeRTOS.

#include <stdio.h>

void log_output(char* buf, int size) {
    if (size <= 0) return;
    fwrite(buf, size, 1, stderr);
    fwrite("\n", 1, 1, stderr);
}



