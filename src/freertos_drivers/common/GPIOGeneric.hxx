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
 * \file GPIOGeneric.hxx
 *
 * Generic implementation header for GPIO.
 *
 * @author Stuart Baker
 * @date 1 July 2015
 */

#ifndef _FREERTOS_DRIVERS_COMMON_GPIOGENERIC_HXX_
#define _FREERTOS_DRIVERS_COMMON_GPIOGENERIC_HXX_

#include "utils/macros.h"

/** Tiva specific implementation of GPIO.  For the Tiva devices, the GPIO
 * number mapping can be found in the @ref GpioMapping enumeration.
 */
class Gpio
{
public:
    /** GPIO setup mode */
    enum Mode
    {
        INPUT     = 0x00, /**< GPIO is an input */
        OUTPUT    = 0x01, /**< GPIO is an output */
        PULL_UP   = 0x02, /**< GPIO has week pull up enabled */
        PULL_DOWN = 0x04, /**< GPIO has week pull down enabled */
        LED       = 0x10, /**< GPIO drives an LED, configure for high current */
        INVERT    = 0x80, /**< GPIO data is inverted */
    };

    /** Values representing the voltage on a GPIO pin */
    enum Value
    {
        CLR = false, /**< GPIO is clear, in other words, currently a '0' */
        SET = true   /**< GPIO is set, in other words, currently a '1' */
    };

    /** Constructor.
     * @param GPIO number
     * @param mode GPIO mode settings
     * @param safe default "safe" value, may go unused for input only pins
     */
    Gpio(unsigned number, Mode mode, Value safe);

    /** Destructor.
     */
    ~Gpio();

    /** Set or clear the GPIO based on its value
     * @param v @ref Value to apply to the GPIO
     */
    void value(Value v)
    {
        if (v == CLR)
        {
            clr();
        }
        else
        {
            set();
        }
    }

    /** Retrieve the current @ref Value of the GPIO pin.
     * @return @ref SET if currently a '1', @ref CLR if currently a '0'.
     *         Note: if the GPIO is inverted, this has the effect of
     *         returning the opposite value
     */
    Value value()
    {
        return is_clr() ? CLR : SET;
    }

    /** Write GPIO to a safe state value as defined during construction.
     */
    void safe()
    {
        value(safeValue ? SET : CLR);
    }

    /** Test the GPIO pin to see if it is set.
     * @return true if currently a '1', false CLR if currently a '0'.
     *         Note: if the GPIO is inverted, this has the effect of
     *         returning the opposite value
     */
    virtual bool is_set() = 0;

    /** Test the GPIO pin to see if it is clear.
     * @return true if currently a '0', false CLR if currently a '1'.
     *         Note: if the GPIO is inverted, this has the effect of
     *         returning the opposite value
     */
    virtual bool is_clr() = 0;

    /** Set the GPIO to a '1'.  Note: if the GPIO is inverted, this could
     * have the opposite effect of clearing the GPIO to a value of '0'.
     */
    virtual void set() = 0;

    /** Clear the GPIO to a '0'.  Note: if the GPIO is inverted, this has
     * have the opposite effect of setting the GPIO to a value of '1'.
     */
    virtual void clr() = 0;

    /** Set the GPIO direction.
     * @param mode @ref INPUT or @ref OUTPUT
     */
    virtual void direction(Mode mode) = 0;

    /** Get the GPIO direction.
     * @return @ref INPUT or @ref OUTPUT
     */
    virtual Mode direction() = 0;

    /** Find a GPIO by referencing its number.
     * @return Gpio instance pointer if found, else nullptr if not found
     */
    static Gpio *find(unsigned number);

protected:
    /** Add the GPIO number to our list of known GPIO.
     */
    void track();

    /** Pin number of GPIO */
    uint8_t pin;

    /** Bit index on GPIO port */
    uint8_t bit;

    /** Value (1 := SET, 0 := CLR) of GPIO in "safe" state.  We store as a
     * uint8_t instead of type @ref Value in order to pack into less memory
     */
    uint8_t safeValue;

    /** 1 if pin in inverted value on read/write.  We store as a uint8_t
     * instead of a bool to ensure it get packed with the other data members
     * around it for the smallest size.
     */
    uint8_t invert;

    /** next GPIO in linked list */
    Gpio *next;

    /** first GPIO in linked list */
    static Gpio *first;

private:
    /* default constructor */
    Gpio()
    {
    }

    DISALLOW_COPY_AND_ASSIGN(Gpio);
};


#endif /* _FREERTOS_DRIVERS_COMMON_GPIOGENERIC_HXX_ */

