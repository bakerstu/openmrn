/** @copyright
 * Copyright (c) 2018 Stuart W Baker
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
 * @file PWM.hxx
 * This file implements an abstract interface for a PWM driver.
 *
 * @author Stuart W. Baker
 * @date 6 January 2018
 */

#include "utils/macros.h"

/// Abstract interface for a PWM driver.
class PWM
{
public:
    /// Set PWM period.
    /// @param PWM period in counts
    virtual void set_period(uint32_t counts) = 0;

    /// Get PWM period.
    /// @return PWM period in counts
    virtual uint32_t get_period() = 0;

    /// Sets the duty cycle.
    /// @param counts duty cycle in counts
    virtual void set_duty(uint32_t counts) = 0;

    /// Gets the duty cycle.
    /// @return counts duty cycle in counts
    virtual uint32_t get_duty() = 0;

    /// Sets the duty cycle.
    /// @param counts duty cycle in percent
    void set_duty_percent(uint8_t percent)
    {
        set_duty(((get_period() * percent) + 50) / 100);
    }

    /// Gets the duty cycle.
    /// @return counts duty cycle in percent
    uint8_t get_duty_percent()
    {
        return (((get_duty() * 100) + (get_period() >> 1)) / get_period());
    }

    /// Get max period supported
    /// @return period in counts
    virtual uint32_t get_period_max() = 0;

    /// Get min period supported
    /// @return period in counts
    virtual uint32_t get_period_min() = 0;

protected:
    /// Constructor.
    PWM()
    {
    }

    /// Destructor.
    ~PWM()
    {
    }

private:

    DISALLOW_COPY_AND_ASSIGN(PWM);
};
