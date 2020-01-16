#include "proc/p32mx795f512h.h"
#include <xc.h>
#include <sys/asm.h>

#include "freertos_drivers/pic32mx/ISRwrapper.inc"

FREERTOS_ISR_wrapper _UART_4_VECTOR,uart4_interrupt
