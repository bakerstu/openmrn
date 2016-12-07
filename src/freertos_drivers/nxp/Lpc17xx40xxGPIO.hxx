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
 * \file Lpc17xx40xxGPIO.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on LpcOpen MCUs.
 *
 * @author Balazs Racz
 * @date 2 Dec 2014
 */

#ifndef _FREERTOS_DRIVERS_NXP_LPC17XX40XXGPIO_HXX_
#define _FREERTOS_DRIVERS_NXP_LPC17XX40XXGPIO_HXX_

#include "chip.h"
#include "core_cm3.h"
#include "gpio_17xx_40xx.h"
#include "iocon_17xx_40xx.h"
#include "os/Gpio.hxx"
#include "GpioWrapper.hxx"

/// Static GPIO implementation for the NXP LPC microcontrollers.
/// @param PORT is the port number (like 0, 1, 2). E.g. for P1.12 this is 1.
/// @param PIN is the number of th epin in the port. E.g. for P1.12 this is 12.
template <uint8_t PORT, uint8_t PIN> struct LpcGpioPin
{
    /// @return the port number on the LPC17xx (e.g. 0 for pin P0_12).
    static constexpr uint8_t port()
    {
        return PORT;
    }
    /// @return the pin number on the LPC17xx (e.g. 12 for pin P0_12).
    static constexpr uint8_t pin()
    {
        return PIN;
    }
    /// Sets the output pin to a given level.
    static void set(bool value)
    {
        Chip_GPIO_SetPinState(LPC_GPIO, port(), pin(), value);
    }
    /// Toggles the output pin level.
    static void toggle()
    {
        Chip_GPIO_SetPinToggle(LPC_GPIO, port(), pin());
    }
    /// @return true if the pin is currently seeing HIGH input level, otherwiae
    /// false.
    static bool get()
    {
        return Chip_GPIO_GetPinState(LPC_GPIO, port(), pin());
    }

    /// @return an os-indepentent Gpio abstraction instance for use in
    /// libraries.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<LpcGpioPin<PORT, PIN>>::instance();
    }

    /// @return true if the pin is configured as an output.
    static bool is_output()
    {
        return Chip_GPIO_GetPinDIR(LPC_GPIO, port(), pin());
    }

protected:
    /// Configures pin as output.
    static void set_output()
    {
        Chip_GPIO_SetPinDIROutput(LPC_GPIO, port(), pin());
    }
    /// Configures pin as input.
    static void set_input()
    {
        Chip_GPIO_SetPinDIRInput(LPC_GPIO, port(), pin());
    }
};

/// GPIO output pin base class for NXP LPC microcontrollers.
template <class Defs, bool SAFE_VALUE> struct GpioOutputPin : public Defs
{
    using Defs::port;
    using Defs::pin;
    using Defs::set_output;
    using Defs::set;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Chip_GPIO_Init(LPC_GPIO);
        Chip_IOCON_Init(LPC_IOCON);
        Chip_IOCON_PinMuxSet(
            LPC_IOCON, port(), pin(), IOCON_FUNC0 | IOCON_MODE_INACT);
        hw_set_to_safe();
    }
    /// Sets the output pin to a safe value.
    static void hw_set_to_safe()
    {
        set_output();
        set(SAFE_VALUE);
    }
};

/// Defines a GPIO output pin, initialized to be an output pin with low level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLow : public GpioOutputPin<Defs, false>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true>
{
};

/// Defines a GPIO output pin with high-current drive and low initialization
/// level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct LedPin : public GpioOutputPin<Defs, false>
{
};

/// Common class for GPIO input pins.
template <class Defs, uint32_t GPIO_PULL> struct GpioInputPin : public Defs
{
public:
    using Defs::port;
    using Defs::pin;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Chip_GPIO_Init(LPC_GPIO);
        Chip_IOCON_Init(LPC_IOCON);
        Chip_IOCON_PinMuxSet(LPC_IOCON, port(), pin(), IOCON_FUNC0 | GPIO_PULL);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
};

/// GPIO Input pin with weak pull up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPU : public GpioInputPin<Defs, IOCON_MODE_PULLUP>
{
};

/// GPIO Input pin with weak pull down.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPD : public GpioInputPin<Defs, IOCON_MODE_PULLDOWN>
{
};

/// GPIO Input pin in standard configuration (no pull).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputNP : public GpioInputPin<Defs, IOCON_MODE_INACT>
{
};

/// GPIO Input pin in repeater configuration (pull the same direction as sensed
/// on the input).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputRep : public GpioInputPin<Defs, IOCON_MODE_REPEATER>
{
};

/// Helper macro for defining GPIO pins.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declares FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param PORT is the number of the port, such as 0
///
/// @param NUM is the pin number, such as 3
///
/// Example:
///  GPIO_PIN(FOO, LedPin, 0, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, PORT, NUM)                                   \
    typedef BaseClass<LpcGpioPin<PORT, NUM>> NAME##_Pin

#endif // _FREERTOS_DRIVERS_NXP_LPC17XX40XXGPIO_HXX_
