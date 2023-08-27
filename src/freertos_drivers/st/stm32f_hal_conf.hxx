/** @copyright
 * Copyright (c) 2019, Balazs Racz
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
 * @file stm32f_hal_conf.hxx
 * Master include file that brings in the appropriate HAL driver for the chip.
 *
 * @author Balazs Racz
 * @date 13 July 2019
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32F_HAL_CONF_HXX_
#define _FREERTOS_DRIVERS_ST_STM32F_HAL_CONF_HXX_

#if defined(STM32F030x6) || defined(STM32F031x6) || defined(STM32F038xx) ||    \
    defined(STM32F030x8) || defined(STM32F030xC) || defined(STM32F042x6) ||    \
    defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) ||    \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) ||    \
    defined(STM32F072xB) || defined(STM32F078xx) || defined(STM32F091xC) ||    \
    defined(STM32F098xx)
#include "stm32f0xx_hal_conf.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_hal_conf.h"
#elif defined(STM32F303xC) || defined(STM32F303xE)
#include "stm32f3xx_hal_conf.h"
#elif defined(STM32F767xx)
#include "stm32f7xx_hal_conf.h"
#elif defined(STM32L432xx) || defined(STM32L431xx)
#include "stm32l4xx_hal_conf.h"
#elif defined(STM32G0B1xx)
#include "stm32g0xx_hal_conf.h"
#else
#error "STM32F_HAL_CONF unsupported STM32 device"
#endif

#endif // _FREERTOS_DRIVERS_ST_STM32F_HAL_CONF_HXX_
