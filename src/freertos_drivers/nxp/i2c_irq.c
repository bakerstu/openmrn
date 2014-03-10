// This file contains a trampoline for ARM->thumb mode for the LPC2368 i2c irq
// handler.

#if defined(__FreeRTOS__) && defined(TARGET_LPC2368)

#include "cmsis_nvic.h"

extern void RealIrqHandler();

void I2C_IRQHandler(void) INTERRUPT_ATTRIBUTE;

void I2C_IRQHandler(void)
{
    RealIrqHandler();
#ifdef INTERRUPT_ACK
    LPC_VIC->Address = 0;
#endif
}

#endif // freertos && lpc2368
