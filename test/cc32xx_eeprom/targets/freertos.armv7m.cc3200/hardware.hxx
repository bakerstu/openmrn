/** \copyright
 * Copyright (c) 2016, Stuart W. Baker
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are strictly prohibited without written consent.
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
 * \file hardware.hxx
 * This file represents the hardware configuration for CC32xx GPIO.
 *
 * @author Stuart W. Baker
 * @date 30 May 2016
 */

#ifndef _HARDWARE_HXX_
#define _HARDWARE_HXX_

#include "freertos_drivers/ti/CC3200GPIO.hxx"
#include "driverlib/rom_map.h"
#include "utils/GpioInitializer.hxx"
#include "freertos_drivers/common/BlinkerGPIO.hxx"

GPIO_PIN(KEY_WAKEUP, GpioInputPin, A0, 4);

GPIO_PIN(I0, GpioInputPin, A0, 6);
GPIO_PIN(I1, GpioInputPin, A0, 7);

GPIO_PIN(BLINKER_RAW, GpioOutputSafeLow, A1, 3);

GPIO_PIN(H1, GpioOutputSafeLow, A1, 2);
GPIO_PIN(H2, GpioOutputSafeHigh, A2, 6);

GPIO_PIN(H3, GpioOutputSafeHigh, A2, 1);
GPIO_PIN(H4, GpioOutputSafeLow, A1, 5);

GPIO_PIN(H5, GpioOutputSafeLow, A2, 7);
GPIO_PIN(H6, GpioOutputSafeLow, A0, 3);
GPIO_PIN(H7, GpioOutputSafeLow, A3, 6);

GPIO_PIN(H8, GpioOutputSafeHigh, A3, 0);
GPIO_PIN(H9, GpioOutputSafeHigh, A3, 4);

// Create an initializer that can initialize all the GPIO pins in one shot
typedef GpioInitializer<KEY_WAKEUP_Pin,
                        I0_Pin, I1_Pin, //
                        BLINKER_RAW_Pin,
                        H1_Pin, H2_Pin, H3_Pin, H4_Pin, //
                        H5_Pin, H6_Pin, H7_Pin, H8_Pin, //
                        H9_Pin> GpioInit;

#endif // _HARDWARE_HXX_
