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

#ifndef _FREERTOS_DRIVERS_COMMON_PWM_HXX_
#define _FREERTOS_DRIVERS_COMMON_PWM_HXX_

#include "utils/macros.h"

#include "os/Gpio.hxx"

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
    /// @param counts duty cycle in percent, range 0 to 100
    void set_duty_percent(uint8_t percent)
    {
        HASSERT(percent <= 100);
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

/// General Purpose Output specialization of a PWM Bit.
class PWMGPO : public Gpio
{
public:
    /// Constructor.
    /// @param instance reference to the chip
    /// @param on_counts PWM count for the "on" state
    /// @param off_counts PWM count for the "off" state
    PWMGPO(PWM *instance, uint32_t on_counts, uint32_t off_counts)
        : Gpio()
        , instance_(instance)
        , onCounts_(on_counts)
        , offCounts_(off_counts)
    {
    }

    virtual ~PWMGPO()
    {
    }

    /// Writes a GPO pin (set or clear to a specific state).
    /// @param new_state the desired output state.  See @ref Value.
    void write(Value new_state) const override
    {
        new_state ? set() : clr();
    }

    /// Retrieves the current @ref Value of a GPO output sate (requested).
    /// @return @ref SET if currently high, @ref CLR if currently low
    Value read() const override
    {
        return instance_->get_duty() == onCounts_ ? Gpio::SET : Gpio::CLR;
    }

    /// Sets the GPO pin to high.
    void set() const override
    {
        instance_->set_duty(onCounts_);
    }

    /// Clears the GPO pin to low.
    void clr() const override
    {
        instance_->set_duty(offCounts_);
    }

    /// Sets the GPO direction (does nothing).
    /// @param dir @ref INPUT or @ref OUTPUT
    void set_direction(Gpio::Direction dir) const override
    {
        HASSERT(dir == Gpio::Direction::DOUTPUT);
    }

    /// Gets the GPO direction.
    /// @return always returns @ref OUTPUT
    Direction direction() const override
    {
        return Gpio::Direction::DOUTPUT;
    }

    uint32_t get_on_counts() const
    {
        return onCounts_;
    }

    uint32_t get_off_counts() const
    {
        return offCounts_;
    }

private:
    /// PWM instance
    PWM *instance_;

    /// PWM counts for when the GPO is "on"
    uint32_t onCounts_;

    /// PWM counts for when the GPO is "off"
    uint32_t offCounts_;

    DISALLOW_COPY_AND_ASSIGN(PWMGPO);
};

#endif // _FREERTOS_DRIVERS_COMMON_PWM_HXX_
