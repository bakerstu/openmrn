#ifndef _OPENMRN_ARDUINO_STM32F_HAL_CONF_HXX
#define _OPENMRN_ARDUINO_STM32F_HAL_CONF_HXX

#include <stm32yyxx_hal_conf.h>

static inline void SetInterruptPriority(uint32_t irq, uint8_t priority)
{
    NVIC_SetPriority((IRQn_Type)irq, priority >> (8U - __NVIC_PRIO_BITS));
}

#ifndef configKERNEL_INTERRUPT_PRIORITY
#define configKERNEL_INTERRUPT_PRIORITY 0xA0
#endif

#endif // _OPENMRN_ARDUINO_STM32F_HAL_CONF_HXX
