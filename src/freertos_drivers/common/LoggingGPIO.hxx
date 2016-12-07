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
 * \file LoggingGPIO.hxx
 *
 * GPIO-abstraction of a nonexistant pin with printing status to stdout.
 *
 * @author Balazs Racz
 * @date 21 Aug 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_LOGGINGGPIO_HXX_
#define _FREERTOS_DRIVERS_COMMON_LOGGINGGPIO_HXX_

#include "GpioWrapper.hxx"

/// GPIO Pin definition structure with no actual pin behind it. All writes to
/// this pin will be logged to stdout. Reads from this pin will not compile.
template<const char* message>
struct LoggingPin
{
    /// Empty. No initialization needed.
    static void hw_init()
    {
    }
    /// Empty. No initialization needed.
    static void hw_set_to_safe()
    {
    }
    /// Sets the output pin level (generating a log output). @param value is
    /// true if output should be HIGH.
    static void set(bool value)
    {
        if (lastValue_ == value) return;
        LOG(INFO, "Logging pin %s = %s", message, value ? "ON" : "OFF");
        lastValue_ = value;
    }
    /// Toggles the output pin level.
    static void toggle()
    {
        set(!lastValue_);
        LOG(INFO, "Logging pin %s: TOGGLE", message);
    }

    /// Returns whether this is an output pin or not.
    static bool is_output()
    {
        return true;
    }

protected:
    /// Last output value.
    static bool lastValue_;
};

template<const char* message>
bool LoggingPin<message>::lastValue_ = false;

/// GPIO Pin definition structure with no actual pin behind it. All writes to
/// this pin will be logged to stdout. Reads will always return false.
template<const char* message>
struct LoggingPinWithRead : public LoggingPin<message>
{
    /// Returns the input pin level.
    static bool get()
    {
        return LoggingPinWithRead::lastValue_;
    }

    /// Returns whether this is an output pin or not.
    static bool is_output()
    {
        return false;
    }

    /// @return the static Gpio instance object controlling this pin.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<LoggingPinWithRead>::instance();
    }
};

#endif // _FREERTOS_DRIVERS_COMMON_LOGGINGGPIO_HXX_
