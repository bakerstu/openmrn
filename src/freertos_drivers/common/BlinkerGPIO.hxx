/** \copyright
 * Copyright (c) 2015, Balazs Racz
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
 * \file BlinkerGPIO.hxx
 *
 * GPIO-abstraction implementation for the blinker bit.
 *
 * @author Balazs Racz
 * @date 19 Jun 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_BLINKER_GPIO_HXX_
#define _FREERTOS_DRIVERS_COMMON_BLINKER_GPIO_HXX_

#include "utils/blinker.h"
#include "os/Gpio.hxx"
#include "GpioWrapper.hxx"

/** GPIO-abstraction implementation for the blinker bit. This allows a
 * GPIO-style usage of the LED that is controlled by the blinker code in the
 * FreeRTOS-compatible hardware implementations. */
struct BLINKER_Pin
{
    /// Initialize the hardware (noop).
    static void hw_init()
    {
    }
    /// Set the hardware to safe setting (noop).
    static void hw_set_to_safe()
    {
    }
    /// Set the pin to a specific value. @param value what to set the pin to.
    static void set(bool value)
    {
        resetblink(value ? 1 : 0);
    }
    /// Toggle the pin (set to opposite value than previously).
    static void toggle()
    {
        resetblink(get() ? 0 : 1);
    }
    /// @return zero if the pin is OFF, non-zero otherwise.
    static bool get()
    {
        return blinker_pattern;
    }
    /// @return true, as the BLINKER_Pin cannot be set to an input.
    static bool is_output()
    {
        return true;
    }
    /// @return static virtual Gpio instance.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<BLINKER_Pin>::instance();
    }
};

#endif // _FREERTOS_DRIVERS_COMMON_BLINKER_GPIO_HXX_
