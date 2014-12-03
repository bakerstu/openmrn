/** \copyright
 * Copyright (c) 2014, Balazs Racz
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
 * \file TivaGPIO.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on Tiva MCUs.
 *
 * @author Balazs Racz
 * @date 2 Dec 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

#define DECL_PIN(NAME, PORT, NUM)                                              \
    static const auto NAME##_PERIPH = SYSCTL_PERIPH_GPIO##PORT;                \
    static const auto NAME##_BASE = GPIO_PORT##PORT##_BASE;                    \
    static const auto NAME##_PIN = GPIO_PIN_##NUM

#define DECL_HWPIN(NAME, PORT, NUM, CONFIG)                                    \
    DECL_PIN(NAME, PORT, NUM);                                                 \
    static const auto NAME##_CONFIG = GPIO_P##PORT##NUM##_##CONFIG

#endif //_FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
