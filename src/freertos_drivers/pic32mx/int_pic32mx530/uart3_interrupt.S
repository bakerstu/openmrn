#include "proc/p32mx530f128h.h"
#include <xc.h>
#include <sys/asm.h>

#include "freertos_drivers/pic32mx/ISRwrapper.inc"

FREERTOS_ISR_wrapper _UART_3_VECTOR,uart3_interrupt
