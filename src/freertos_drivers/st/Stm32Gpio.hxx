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
 * \file Stm32Gpio.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on STM32 microcontrollers.
 *
 * @author Balazs Racz
 * @date 24 Aug 2015
 */

#ifndef _FREERTOS_DRIVERS_ST_STM32GPIO_HXX_
#define _FREERTOS_DRIVERS_ST_STM32GPIO_HXX_

#include "os/Gpio.hxx"
#include "GpioWrapper.hxx"

#if defined(STM32F072xB) || defined(STM32F091xC)
#include "stm32f0xx_hal_gpio.h"
#elif defined(STM32F103xB)
#include "stm32f1xx_hal_gpio.h"
#elif defined(STM32F303xC)
#include "stm32f3xx_hal_gpio.h"
#else
#error Dont know what STM32 chip you have.
#endif

/// Static GPIO implementation for the STM32 microcontrollers. Do not use
/// directly: use @ref GPIO_PIN macro.
/// @param PORT is the port base address pointer (e.g. GPIOC) 
/// @param PIN is the GPIO_PIN_# value for the pin number.
/// @param PIN_NUM is the number of the pin in the port. Zero-based.
template <uint32_t GPIOx, uint16_t PIN, uint8_t PIN_NUM> struct Stm32GpioDefs
{
    /// @return the PIO structure of the give gpio port.
    static GPIO_TypeDef *port()
    {
        return (GPIO_TypeDef *)GPIOx;
    }

    /// @return the pin number within the port.
    static uint16_t pin()
    {
        return PIN;
    }

    /// Sets the output pin to a given level.
    static void set(bool value)
    {
        if (value)
        {
#if defined(STM32F303xC)
            port()->BSRRL = pin();
#else
            port()->BSRR = pin();
#endif
        }
        else
        {
#if defined(STM32F303xC)
            port()->BSRRH = pin();
#else
            port()->BSRR = pin() << 16;
#endif
        }
    }

    /// Sets the output pin to a given level.
    static bool get()
    {
        return port()->IDR & pin();
    }

    /// @return a os-indepentent Gpio abstraction instance for use in
    /// libraries.
    static constexpr const Gpio *instance()
    {
        return GpioWrapper<Stm32GpioDefs<GPIOx, PIN, PIN_NUM>>::instance();
    }

    /// @return whether this pin is configured as an output.
    static bool is_output()
    {
        uint8_t* mode = (uint8_t*)port();
        uint8_t m = mode[PIN_NUM >> 1];
        if (PIN_NUM & 1) { m >>= 4; }
        return (m & 3) != 0;
    }

};

template <class Defs, bool SAFE_VALUE> struct GpioOutputPin : public Defs
{
    using Defs::port;
    using Defs::pin;
    using Defs::set;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin = pin();
        gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(port(), &gpio_init);
    }
    /// Sets the output pin to a safe value.
    static void hw_set_to_safe()
    {
        hw_init();
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

/// Defines a GPIO output pin for driving an LED. The MCU must be in spec for
/// the necessary output current.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct LedPin : public GpioOutputPin<Defs, false>
{
};

template <class Defs, uint32_t PULL_MODE> struct GpioInputPin : public Defs
{
    using Defs::port;
    using Defs::pin;
    using Defs::set;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin = pin();
        gpio_init.Mode = GPIO_MODE_INPUT;
        gpio_init.Pull = PULL_MODE;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(port(), &gpio_init);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// @return true if the pin is set to drive an output.
    static bool is_output()
    {
        return false;
    }
};

/// GPIO Input pin with weak pull up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPU : public GpioInputPin<Defs, GPIO_PULLUP>
{
};

/// GPIO Input pin with weak pull down.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPD : public GpioInputPin<Defs, GPIO_PULLDOWN>
{
};

/// GPIO Input pin in standard configuration (no pull).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputNP : public GpioInputPin<Defs, GPIO_NOPULL>
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
/// @param PORTNAME is a letter defining which port this is (like A, B,
/// C). E.g. for PC8 this is C.
///
/// @param NUM is the pin number, such as 8 for PC8.
///
/// Example:
///  GPIO_PIN(FOO, LedPin, 0, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, PORTNAME, NUM)                               \
    typedef BaseClass<Stm32GpioDefs<(uint32_t)(GPIO ## PORTNAME ## _BASE), GPIO_PIN_ ## NUM, NUM>> NAME##_Pin

#endif // _FREERTOS_DRIVERS_ST_STM32GPIO_HXX_
