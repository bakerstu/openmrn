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
 * \file Gpio.hxx
 *
 * Generic implementation header for GPIO.
 *
 * @author Stuart Baker, Balazs Racz
 * @date 1 July 2015
 */

#ifndef _OS_GPIO_HXX_
#define _OS_GPIO_HXX_

#include "utils/macros.h"

/** OS-independent abstraction for GPIO.
 */
class Gpio
{
public:
    constexpr Gpio()
    {
    }
    virtual ~Gpio();

    /** Defines the options for GPIO level. */
    enum Value : bool
    {
        CLR = false,
        SET = true,
        LOW = CLR,
        HIGH = SET
    };

    /** Defines the options for GPIO direction. This enum must always be used
     * fully qualified (i.e. Gpio::Direction::INPUT and
     * Gpio::Direction::OUTPUT). */
    enum class Direction
    {
        INPUT,
        OUTPUT,
    };

    /** Writes a GPIO output pin (set or clear to a specific state).
     * @param new_state the desired output state.
     */
    virtual void write(Value new_state) = 0;

    /** Retrieve the current @ref Value of a GPIO input pin.
     * @return @ref SET if currently high, @ref CLR if currently low.
     */
    virtual Value read() = 0;

    /** Test the GPIO input pin to see if it is set.
     * @return true if input pin is currently high, false if currently low.
     */
    bool is_set()
    {
        return value() == SET;
    }

    /** Test the GPIO input pin to see if it is clear.
     * @return true if input pin is currently low, false if currently high.
     */
    bool is_clr()
    {
        return value() == CLR;
    }

    /** Set the GPIO output pin to high.
     */
    virtual void set() = 0;

    /** Clear the GPIO output pin to low.
     */
    virtual void clr() = 0;

    /** Set the GPIO direction.
     * @param dir @ref INPUT or @ref OUTPUT
     */
    virtual void set_direction(Direction dir) = 0;

    /** Get the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     */
    virtual Direction direction() = 0;
};

#endif /* _FREERTOS_DRIVERS_COMMON_GPIOGENERIC_HXX_ */
