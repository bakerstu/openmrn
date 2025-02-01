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

#include "GpioWrapper.hxx"
#include "os/Gpio.hxx"

#include "stm32f_hal_conf.hxx"

/// Static GPIO implementation for the STM32 microcontrollers. Do not use
/// directly: use @ref GPIO_PIN macro.
/// @param PORT is the port base address pointer (e.g. GPIOC)
/// @param PIN is the GPIO_PIN_# value for the pin number.
/// @param PIN_NUM is the number of the pin in the port. Zero-based.
template <uint32_t GPIOx, uint16_t PIN, uint8_t PIN_NUM> struct Stm32GpioDefs
{
    /// @return the GPIO structure of the given gpio port.
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
// These alternatives are needed for old releases of STM32F HAL library.
#if 0 && (defined(STM32F303xC) || defined(STM32F303xE))
            port()->BSRRL = pin();
#else
            port()->BSRR = pin();
#endif
        }
        else
        {
#if 0 && (defined(STM32F303xC) || defined(STM32F303xE))
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

    /// Changes the output pin level.
    static void toggle()
    {
        set(!get());
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
        uint8_t *mode = (uint8_t *)port();
        uint8_t m = mode[PIN_NUM >> 1];
        if (PIN_NUM & 1)
        {
            m >>= 4;
        }
        return (m & 3) != 0;
    }
};

template <class Defs, bool SAFE_VALUE> struct GpioOutputPin : public Defs
{
    using Defs::pin;
    using Defs::port;
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
        set(SAFE_VALUE);
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
{ };

/// Defines a GPIO output pin, initialized to be an output pin with high level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true>
{ };

/// Defines a GPIO output pin for driving an LED. The MCU must be in spec for
/// the necessary output current.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct LedPin : public GpioOutputPin<Defs, false>
{ };

template <class Defs, uint32_t PULL_MODE> struct GpioInputPin : public Defs
{
    using Defs::pin;
    using Defs::port;
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
    static constexpr bool is_output()
    {
        return false;
    }
};

/// GPIO Input pin with weak pull up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPU : public GpioInputPin<Defs, GPIO_PULLUP>
{ };

/// GPIO Input pin with weak pull down.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPD : public GpioInputPin<Defs, GPIO_PULLDOWN>
{ };

/// GPIO Input pin in standard configuration (no pull).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputNP : public GpioInputPin<Defs, GPIO_NOPULL>
{ };

#include "utils/OptionalArgs.hxx"

/// Configuration options for an STM32 GPIO pin.
struct Stm32GpioOptionDefs
{
    /// Mode: GPIO_MODE_xx value, including GPIO_MODE_INPUT,
    /// GPIO_MODE_OUTPUT_PP, GPIO_MODE_OUTPUT_OD, GPIO_MODE_AF_PP,
    /// GPIO_MODE_AF_OD.
    DECLARE_OPTIONALARG(GpioMode, gpio_mode, uint32_t, 0, GPIO_MODE_INPUT);
    /// Open Drain. false is push-pull, true is open-drain. Only used for AF
    /// and Output modes. When true, switches a PP mode to OD mode.
    DECLARE_OPTIONALARG(OD, gpio_od, bool, 1, false);
    /// Pullup/Pulldown configuration. One of GPIO_NOPULL, GPIO_PULLUP,
    /// GPIO_PULLDOWN.
    DECLARE_OPTIONALARG(Pull, pull, uint32_t, 2, GPIO_NOPULL);
    /// Speed configuration. One of GPIO_SPEED_FREQ_LOW,
    /// GPIO_SPEED_FREQ_MEDIUM, GPIO_SPEED_FREQ_HIGH, GPIO_SPEED_FREQ_VERYHIGH.
    DECLARE_OPTIONALARG(Speed, speed, uint32_t, 3, GPIO_SPEED_FREQ_LOW);
    /// Used for selecting alternate function. Many values are possible, for
    /// example GPIO_AF0_UART3 or GPIO_AF3_FDCAN1. Typically AF0 to AF7, or on
    /// some chips until AF15, and the peripheral name can be a large
    /// variety. Take the values from the datasheet or the specific chip's
    /// header file.
    DECLARE_OPTIONALARG(AfMode, afmode, uint32_t, 4, 0xffffffff);

    /// Sets the GPIO port base, like GPIOA_BASE, GPIOB_BASE, etc. Required.
    DECLARE_OPTIONALARG(PeriphBase, periph_base, uint32_t, 10, 0xffffffff);
    /// Sets the GPIO pin number, like GPIO_PIN_0, GPIO_PIN_1, etc. Required.
    DECLARE_OPTIONALARG(Pin, pin, uint16_t, 11, 0xffff);
    /// Sets the GPIO pin number, 0..15. Required.
    DECLARE_OPTIONALARG(PinNum, pin_num, uint8_t, 11, 0xff);

    using Base = OptionalArg<Stm32GpioOptionDefs, GpioMode, OD, Pull, Speed,
        AfMode, PeriphBase, Pin, PinNum>;
};

class Stm32GpioOptions : public Stm32GpioOptionDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(Stm32GpioOptions, Stm32GpioOptionDefs::Base);

    DEFINE_OPTIONALARG(GpioMode, gpio_mode, uint32_t);
    DEFINE_OPTIONALARG(OD, gpio_od, bool);
    DEFINE_OPTIONALARG(Pull, pull, uint32_t);
    DEFINE_OPTIONALARG(Speed, speed, uint32_t);
    DEFINE_OPTIONALARG(AfMode, afmode, uint32_t);

    DEFINE_OPTIONALARG(PeriphBase, periph_base, uint32_t);
    DEFINE_OPTIONALARG(Pin, pin, uint16_t);
    DEFINE_OPTIONALARG(PinNum, pin_num, uint8_t);

    constexpr bool is_af() const
    {
        return (gpio_mode() == GPIO_MODE_AF_PP) ||
            (gpio_mode() == GPIO_MODE_AF_OD);
    }

    constexpr bool is_od() const
    {
        return (gpio_mode() == GPIO_MODE_AF_OD) ||
            (gpio_mode() == GPIO_MODE_OUTPUT_OD);
    }

    template <typename... Args>
    static constexpr void __attribute__((always_inline))
    fill_options(GPIO_InitTypeDef *gpio_init, Args... args)
    {
#if 0
        constexpr Stm32GpioOptions ao(args...);
        static_assert(IS_GPIO_MODE(Stm32GpioOptions(args...).gpio_mode()), "Incorrect gpio mode");
        static_assert(IS_GPIO_SPEED(ao.speed()), "Incorrect gpio speed");
        static_assert(IS_GPIO_PULL(ao.pull()), "Incorrect gpio pull");
        static_assert((ao.is_af() && ao.has_afmode()) ||
                (!ao.is_af() && !ao.has_afmode()),
            "Must specify AfMode for any AF modes.");
#endif        
        gpio_init->Mode = Stm32GpioOptions(args...).gpio_mode();
        gpio_init->Pin = Stm32GpioOptions(args...).pin();
        gpio_init->Pull = Stm32GpioOptions(args...).pull();
        gpio_init->Speed = Stm32GpioOptions(args...).speed();
        gpio_init->Alternate = Stm32GpioOptions(args...).has_afmode()
            ? Stm32GpioOptions(args...).afmode()
            : 0;
    }

    /// @return the GPIO structure of the given gpio port.
    constexpr GPIO_TypeDef *port() const
    {
        return (GPIO_TypeDef *)periph_base();
    }
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
#define GPIO_XPIN(NAME, BaseClass, PORTNAME, NUM, ARGS...)                     \
    struct NAME##_PinDefs                                                      \
    {                                                                          \
        using PeriphBase = Stm32GpioOptions::PeriphBase;                       \
        using Pin = Stm32GpioOptions::Pin;                                     \
        using PinNum = Stm32GpioOptions::PinNum;                               \
        using GpioMode = Stm32GpioOptions::GpioMode;                           \
        using Pull = Stm32GpioOptions::Pull;                                   \
        using Speed = Stm32GpioOptions::Speed;                                 \
        using AfMode = Stm32GpioOptions::AfMode;                               \
        static constexpr Pull PullUp()                                         \
        {                                                                      \
            return Pull(GPIO_PULLUP);                                          \
        }                                                                      \
        static constexpr Pull PullDown()                                       \
        {                                                                      \
            return Pull(GPIO_PULLDOWN);                                        \
        }                                                                      \
        static constexpr Pull NoPull()                                         \
        {                                                                      \
            return Pull(GPIO_NOPULL);                                          \
        }                                                                      \
        static constexpr GpioMode Output()                                     \
        {                                                                      \
            return GpioMode(GPIO_MODE_OUTPUT_PP);                              \
        }                                                                      \
        static constexpr GpioMode OutputOd()                                   \
        {                                                                      \
            return GpioMode(GPIO_MODE_OUTPUT_OD);                              \
        }                                                                      \
        static constexpr GpioMode Af()                                         \
        {                                                                      \
            return GpioMode(GPIO_MODE_AF_PP);                                  \
        }                                                                      \
        static constexpr GpioMode AfOd()                                       \
        {                                                                      \
            return GpioMode(GPIO_MODE_AF_OD);                                  \
        }                                                                      \
        static constexpr Stm32GpioOptions opts()                               \
        {                                                                      \
            return Stm32GpioOptions(PeriphBase(GPIO##PORTNAME##_BASE),         \
                Pin(GPIO_PIN_##NUM), PinNum(NUM), ##ARGS);                     \
        }                                                                      \
    };                                                                         \
    typedef BaseClass<NAME##_PinDefs> NAME##_Pin

template <class Defs>
struct GpioHwPin : public Stm32GpioDefs<Defs::opts().periph_base(),
                       Defs::opts().pin(), Defs::opts().pin_num()>
{
    /// Implements hw_init functionality for this pin only.
    static void hw_init()
    {
        GPIO_InitTypeDef gpio_init = {0};
        Stm32GpioOptions::fill_options(&gpio_init, Defs::opts());
        HAL_GPIO_Init(Defs::opts().port(), &gpio_init);
    }

    /// Implements hw_set_to_safe functionality for this pin only.
    static void hw_set_to_safe()
    {
        hw_init();
    }

    /// Switches the GPIO pin to the hardware peripheral. */
    static void set_hw()
    {
        hw_init();
    }

    static constexpr Stm32GpioOptions opts() {
        return Defs::opts();
    }
    
    static constexpr Stm32GpioOptions output_opts()
    {
        return Stm32GpioOptions(Stm32GpioOptions::GpioMode(Defs::opts().is_od()
                                        ? GPIO_MODE_OUTPUT_OD
                                        : GPIO_MODE_OUTPUT_PP),
            Defs::opts());
    }

    static constexpr Stm32GpioOptions input_opts()
    {
        return Stm32GpioOptions(
            Stm32GpioOptions::GpioMode(GPIO_MODE_INPUT), Defs::opts());
    }

    /// Switches the GPIO pin to an output pin. Maintains the OD status from
    /// the original options.
    static void set_output()
    {
        GPIO_InitTypeDef gpio_init = {0};
        output_opts().fill_options(&gpio_init);
        HAL_GPIO_Init(Defs::opts().port(), &gpio_init);
    }

    /// Switches the GPIO pin to an input pin. Maintains the pull from the
    /// original options.
    static void set_input()
    {
        GPIO_InitTypeDef gpio_init = {0};
        input_opts().fill_options(&gpio_init);
        HAL_GPIO_Init(Defs::opts().port(), &gpio_init);
    }
}; // class GpioHwPin

#if 0

template <uint32_t afnum, uint32_t afmode = GPIO_AF_PP,
    uint32_t pull = GPIO_NOPULL, uint32_t speed = GPIO_SPEED_FREQ_LOW>
struct GpioHwParams
{
    /// Used for setting pullup pulldown, example value GPIO_NOPULL,
    /// GPIO_PULLUP, GPIO_PULLDOWN.
    static constexpr uint32_t PULL_MODE = pull;
    /// Used for setting slew rate, example value GPIO_SPEED_FREQ_LOW,
    /// GPIO_SPEED_FREQ_HIGH.
    static constexpr uint32_t SPEED_FREQ = speed;
    /// Used for setting alternate function mode, example GPIO_AF_PP,
    /// GPIO_AF_OD.
    static constexpr uint32_t AF_MODE = afmode;
    
    static constexpr uint32_t AF_NUM = afnum;
    
    using Input = GpioInputPin;
};

template<class Defs>
struct GpioHwPin : public Defs
{
    using Defs::PULL_MODE;
    using Defs::SPEED_FREQ;
    using Defs::AF_MODE;    

    /// Implements hw_init functionality for this pin only.
    static void hw_init()
    {
        GPIO_InitTypeDef gpio_init = {0};
        gpio_init.Pin = pin();
        gpio_init.Mode = AF_MODE;
        gpio_init.Pull = PULL_MODE;
        gpio_init.Speed = SPEED_FREQ;
        gpio_init.Alternate = GPIO_AF3_FDCAN1;
        HAL_GPIO_Init(port(), &gpio_init);


        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
        gpio_init.Alternate = GPIO_AF3_FDCAN1;
        gpio_init.Pin = GPIO_PIN_8;
        HAL_GPIO_Init(GPIOB, &gpio_init);
        gpio_init.Pin = GPIO_PIN_9;
        HAL_GPIO_Init(GPIOB, &gpio_init);
    }

    /// Implements hw_set_to_safe functionality for this pin only.
    static void hw_set_to_safe()
    {
        Defs::Safe::hw_init();
    }

    /// Switches the GPIO pin to the hardware peripheral. */
    static void set_hw()
    {
        hw_init();
    }

    /// Switches the GPIO pin to an output pin. Note that this will set the pin
    /// to the SAFE value. Use the set() command afterwards to define the
    /// desired value.
    static void set_output()
    {
        Defs::Output::hw_init();
        static_assert(
            Defs::Output::is_output() == true, "Output pin is not correct");
    }

    /** Switches the GPIO pin to an input pin. Use the get() command to
     * retrieve the value.
     *
     * @param drive_type specifies whether there should be a weak pullup
     * (GPIO_PIN_TYPE_STD_WPU), pull-down (GPIO_PIN_TYPE_STD_WPD) or standard
     * pin (default, GPIO_PIN_TYPE_STD)*/
    static void set_input()
    {
        Defs::Input::hw_init();
        static_assert(Defs::Output::is_output() == true);
    }

/// Helper macro for defining GPIO pins with a specific hardware config on the
/// Tiva microcontrollers.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param PORTNAME is the letter (e.g. D)
///
/// @param NUM is the pin number, such as 3
///
/// @param CONFIG is the suffix of the symbol that defines the pinmux for the
/// hardware, e.g. U7TX.
///
/// @param TYPE is a suffix for a TivaWare GPIOPinType function such as UART
/// for GPIOPinTypeUART() to set the pin to the hardware configuration.
#define GPIO_HWPIN(NAME, SafeType, PORTNAME, NUM, AFTYPE, AFNUM, Args...)      \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_HWPIN(GPIO, PORT, NUM, CONFIG, TYPE);                             \
        using Params = HwPinParams<Args...>;                                   \
    };                                                                         \
    typedef GpioHwPin<NAME##Defs> NAME##_Pin

#endif // if 0, for HWPIN draft.

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
    typedef BaseClass<                                                         \
        Stm32GpioDefs<(uint32_t)(GPIO##PORTNAME##_BASE), GPIO_PIN_##NUM, NUM>> \
        NAME##_Pin

#endif // _FREERTOS_DRIVERS_ST_STM32GPIO_HXX_
