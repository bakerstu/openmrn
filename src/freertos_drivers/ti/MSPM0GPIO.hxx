/** \copyright
 * Copyright (c) 2025, Balazs Racz
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
 * \file MSPM0GPIO.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on TI MSPM0 MCUs.
 *
 * @author Balazs Racz
 * @date 10 May 2025
 */

#ifndef _FREERTOS_DRIVERS_TI_MSPM0GPIO_HXX_
#define _FREERTOS_DRIVERS_TI_MSPM0GPIO_HXX_

#include "os/Gpio.hxx"
#include "utils/OptionalArgs.hxx"

#include <ti/driverlib/driverlib.h>

// These macro aliases are needed because the device header is sometimes using
// double digit pin numbers like 07, sometimes single digit like 7.
#define DL_GPIO_PIN_00 DL_GPIO_PIN_0
#define DL_GPIO_PIN_01 DL_GPIO_PIN_1
#define DL_GPIO_PIN_02 DL_GPIO_PIN_2
#define DL_GPIO_PIN_03 DL_GPIO_PIN_3
#define DL_GPIO_PIN_04 DL_GPIO_PIN_4
#define DL_GPIO_PIN_05 DL_GPIO_PIN_5
#define DL_GPIO_PIN_06 DL_GPIO_PIN_6
#define DL_GPIO_PIN_07 DL_GPIO_PIN_7
#define DL_GPIO_PIN_08 DL_GPIO_PIN_8
#define DL_GPIO_PIN_09 DL_GPIO_PIN_9

/// Defines an enum for setting which pin mode a given gpio pad should be set
/// to. Used in the Mode() setting of the GpioOptions.
enum MspM0PinMode
{
    DIGITAL_INPUT,
    DIGITAL_OUTPUT,
    PERIPHERAL_INPUT,
    PERIPHERAL_OUTPUT,
    ANALOG
};

/// Configuration options for an STM32 GPIO pin.
struct MspM0GpioOptionDefs
{
    /// Mode: Selects which pin mode to use (input/output, GPIO or peripheral
    /// or analog).
    DECLARE_OPTIONALARG(Mode, mode, MspM0PinMode, 0, (MspM0PinMode)-1);
    /// Pullup/Pulldown configuration. One of DL_GPIO_RESISTOR_NONE (default),
    /// DL_GPIO_RESISTOR_PULL_UP, DL_GPIO_RESISTOR_PULL_DOWN.
    DECLARE_OPTIONALARG(Pull, pull, DL_GPIO_RESISTOR, 1, DL_GPIO_RESISTOR_NONE);
    /// Open-Drain configuration. One of DL_GPIO_HIZ_ENABLE, DL_GPIO_HIZ_DISABLE
    /// (default)
    DECLARE_OPTIONALARG(OD, od, DL_GPIO_HIZ, 2, DL_GPIO_HIZ_DISABLE);
    /// Inversion configuration. One of DL_GPIO_INVERSION_ENABLE,
    /// DL_GPIO_INVERSION_DISABLE (default).
    DECLARE_OPTIONALARG(
        Invert, invert, DL_GPIO_INVERSION, 3, DL_GPIO_INVERSION_DISABLE);
    /// GPIO drive strength configuration. One of DL_GPIO_DRIVE_STRENGTH_LOW
    /// (default), DL_GPIO_DRIVE_STRENGTH_HIGH.
    DECLARE_OPTIONALARG(DriveStrength, drive_strength, DL_GPIO_DRIVE_STRENGTH,
        4, DL_GPIO_DRIVE_STRENGTH_LOW);
    /// Selects peripheral function for peripheral input/output.  example
    /// IOMUX_PINCM19_PF_SPI0_PICO. Take the values from the datasheet or the
    /// specific chip's header file. The default value is 1, which is GPIO.
    DECLARE_OPTIONALARG(Function, function, uint32_t, 5, 1);

    /// Specifies the safe value for an output, used during startup and
    /// in hw_set_to_safe. True is high, false is low.
    DECLARE_OPTIONALARG(Safe, safe, bool, 9, false);

    /// Sets the pin number in the form of a CM index. Take this value from the
    /// datasheet, e.g. pad 13 would be IOMUX_PINCM13. Required. Note that pad
    /// numbers do not match to GPIO numbers and also not to pin numbers on any
    /// specific package!
    DECLARE_OPTIONALARG(CM, cm, IOMUX_PINCM, 10, (IOMUX_PINCM)0xffffffff);
    /// Sets the GPIO port base, like GPIOA_BASE, GPIOB_BASE, etc. Required.
    DECLARE_OPTIONALARG(GpioBase, gpio_base, uint32_t, 11, 0xffffffff);
    /// Sets the GPIO pin number, which is a bit mask (i.e. 1<<0, to 1<<31), or
    /// DL_GPIO_PIN_0, to DL_GPIO_PIN_31.  Required.
    DECLARE_OPTIONALARG(Pin, pin, uint32_t, 12, 0);

    using Base = OptionalArg<MspM0GpioOptionDefs, Mode, Pull, OD, Invert,
        DriveStrength, Function, Safe, CM, GpioBase, Pin>;
};

/// Constexpr class for representing the actual options that are defined on a
/// particular pin. An object of this class gets constexpr-constructed by the
/// code from the symbols like Pull(), DriveStrength() and their arguments, but
/// never should actually appear in a binary. Instead, it has only constexpr
/// functions to query certain properties. Beyond the base functions like
/// pull() and pin() there are also some convenience functions, also
/// constexpr, that process the raw returned values into something more
/// intuitive or easier to use.
class MspM0GpioOptions : public MspM0GpioOptionDefs::Base
{
public:
    INHERIT_CONSTEXPR_CONSTRUCTOR(MspM0GpioOptions, MspM0GpioOptionDefs::Base);

    DEFINE_OPTIONALARG(Mode, mode, MspM0PinMode);
    DEFINE_OPTIONALARG(Pull, pull, DL_GPIO_RESISTOR);
    DEFINE_OPTIONALARG(OD, od, DL_GPIO_HIZ);
    DEFINE_OPTIONALARG(Invert, invert, DL_GPIO_INVERSION);
    DEFINE_OPTIONALARG(DriveStrength, drive_strength, DL_GPIO_DRIVE_STRENGTH);
    DEFINE_OPTIONALARG(Function, function, uint32_t);

    DEFINE_OPTIONALARG(Safe, safe, bool);

    DEFINE_OPTIONALARG(CM, cm, IOMUX_PINCM);
    DEFINE_OPTIONALARG(GpioBase, gpio_base, uint32_t);
    DEFINE_OPTIONALARG(Pin, pin, uint32_t);

    /// @return true if the desired mode is a peripheral alternate function.
    constexpr bool is_af() const
    {
        return (mode() == PERIPHERAL_INPUT) || (mode() == PERIPHERAL_OUTPUT);
    }

    /// @return the GPIO structure of the given gpio port.
    constexpr GPIO_Regs *port() const
    {
        return (GPIO_Regs *)gpio_base();
    }
};

/// Generic GPIO class implementation.
template <unsigned GPIO_BASE, unsigned GPIO_PIN> class Mspm0Gpio : public Gpio
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the rodata section.
    constexpr Mspm0Gpio()
    {
    }

    void write(Value new_state) const override
    {
        *pin_address_w() = (new_state ? 1 : 0);
    }

    void set() const override
    {
        *pin_address_w() = 1;
    }

    void clr() const override
    {
        *pin_address_w() = 0;
    }

    Value read() const override
    {
        return *pin_address_r() ? VHIGH : VLOW;
    }

    void set_direction(Direction dir) const override
    {
        if (dir == Direction::DOUTPUT)
        {
            DL_GPIO_enableOutput(port(), pinmask());
        }
        else
        {
            DL_GPIO_disableOutput(port(), pinmask());
        }
    }

    Direction direction() const override
    {
        if (port()->DOE31_0 & pinmask())
        {
            // high is output
            return Direction::DOUTPUT;
        }
        else
        {
            return Direction::DINPUT;
        }
    }

private:
    /// Static instance variable that can be used for libraries expecting a
    /// generic Gpio pointer. This instance variable will be initialized by the
    /// linker and (assuming the application developer initialized the hardware
    /// pins in hw_preinit) is accessible, including virtual methods at static
    /// constructor time.
    static const Mspm0Gpio instance_;

    /// Computes the memory address where the bit referring to this pin can be
    /// accessed for read. This is an 8-bit address and only one bit is in it --
    /// all other bits are zero.
    /// @return magic address.
    constexpr volatile uint8_t *pin_address_r() const
    {
        return reinterpret_cast<volatile uint8_t *>(&port()->DIN0_3) + GPIO_PIN;
    }

    /// Computes the memory address where the bit referring to this pin can be
    /// accessed for write. This is an 8-bit address and only one bit is active
    /// in it, all other bits writes are ignored.
    /// @return magic address.
    constexpr volatile uint8_t *pin_address_w() const
    {
        return reinterpret_cast<volatile uint8_t *>(&port()->DOUT0_3) +
            GPIO_PIN;
    }

    /// @return the port's register overlay structure.
    constexpr GPIO_Regs *port() const
    {
        return reinterpret_cast<GPIO_Regs *>(GPIO_BASE);
    }

    /// @return the bit in the pinmask for the given pin. This is used in a
    /// number of APIs.
    constexpr uint32_t pinmask() const
    {
        return 1u << GPIO_PIN;
    }
};

static constexpr MspM0GpioOptions GpioInputNP {
    MspM0GpioOptions::Mode(DIGITAL_INPUT)};

static constexpr MspM0GpioOptions GpioInputPU {
    MspM0GpioOptions::Mode(DIGITAL_INPUT),
    MspM0GpioOptions::Pull(DL_GPIO_RESISTOR_PULL_UP)};

/// Static GPIO pin struct for MSP M0.
template <class Defs> struct MspM0GpioPin
{
    static_assert(Defs::opts.mode() == DIGITAL_INPUT ||
            Defs::opts.mode() == DIGITAL_OUTPUT ||
            Defs::opts.mode() == PERIPHERAL_INPUT ||
            Defs::opts.mode() == PERIPHERAL_OUTPUT ||
            Defs::opts.mode() == ANALOG,
        "Pin's mode is not set.");
    template<typename U = Defs>
    static std::enable_if_t<U::opts.mode() == DIGITAL_INPUT>
    hw_init()
    {
        DL_GPIO_enablePower(Defs::opts.port());
        static_assert(1 == Defs::opts.function(),
            "GPIO input should not have peripheral function");
        DL_GPIO_initDigitalInputFeatures(Defs::opts.cm(), Defs::opts.invert(),
            Defs::opts.pull(), DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP_DISABLE);
    }
    template<typename U = Defs>
    static std::enable_if_t<U::opts.mode() == DIGITAL_OUTPUT>
    hw_init()
    {
        DL_GPIO_enablePower(Defs::opts.port());
        static_assert(1 == Defs::opts.function(),
            "GPIO output should not have peripheral function");
        DL_GPIO_initDigitalOutputFeatures(Defs::opts.cm(), Defs::opts.invert(),
            Defs::opts.pull(), Defs::opts.drive_strength(), Defs::opts.od());
        enable_input();
        set(Defs::opts.safe());
        DL_GPIO_enableOutput(Defs::opts.port(), Defs::opts.pin());
    }
    template<typename U = Defs>
    static typename std::enable_if<U::opts.mode() == PERIPHERAL_INPUT>::type
    hw_init()
    {
        DL_GPIO_enablePower(Defs::opts.port());
        static_assert(
            1 != Defs::opts.function(), "Missing peripheral function");
        DL_GPIO_initPeripheralInputFunctionFeatures(Defs::opts.cm(),
            Defs::opts.function(), Defs::opts.invert(), Defs::opts.pull(),
            DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    }
    template<typename U = Defs>
    static typename std::enable_if<U::opts.mode() == PERIPHERAL_OUTPUT>::type
    hw_init()
    {
        DL_GPIO_enablePower(Defs::opts.port());
        static_assert(
            1 != Defs::opts.function(), "Missing peripheral function");
        DL_GPIO_initPeripheralOutputFunctionFeatures(Defs::opts.cm(),
            Defs::opts.function(), Defs::opts.invert(), Defs::opts.pull(),
            Defs::opts.drive_strength(), Defs::opts.od());
        enable_input();
    }
    template<typename U = Defs>
    static typename std::enable_if<U::opts.mode() == ANALOG>::type hw_init()
    {
        DL_GPIO_enablePower(Defs::opts.port());
        DL_GPIO_initPeripheralAnalogFunction(Defs::opts.cm());
    }

    /// Enables the input structure in the IOMUX pad. This is needed when gpio
    /// output or peripheral output config is selected but we want to use the
    /// get() function.
    static void enable_input()
    {
        IOMUX->SECCFG.PINCM[Defs::opts.cm()] |= IOMUX_PINCM_INENA_ENABLE;
    }

    static void hw_set_to_safe()
    {
        if (Defs::opts.mode() == DIGITAL_OUTPUT)
        {
            set(Defs::opts.safe());
            DL_GPIO_enableOutput(Defs::opts.port(), Defs::opts.pin());
        }
    }

    /// Sets the output pin to a specified value; @param value if true, output
    /// is set to HIGH otherwise LOW.
    static void set(bool value)
    {
        douts()[Defs::PIN_NUM] = value;
    }

    /// @return current value of an input pin, if true HIGH, of false LOW.
    static bool get()
    {
        return dins()[Defs::PIN_NUM];
    }

    /// Changes the value of an output pin.
    static void toggle()
    {
        DL_GPIO_togglePins(Defs::port(), Defs::opts.pin());
    }

    /// Helper function to compute the pointer for the digital output.
    static constexpr uint8_t *douts()
    {
        return (uint8_t *)(&Defs::opts.port()->DOUT3_0);
    }
    /// Helper function to compute the pointer for the digital input.
    static constexpr uint8_t *dins()
    {
        return (uint8_t *)(&Defs::opts.port()->DIN3_0);
    }
};

/// Helper macro for defining GPIO pins on the Mspm0 microcontrollers.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseDefs is a template, such as @ref GpioOutputSafeHigh or @ref
/// GpioOutputSafeLow, or GpioInput.
///
/// @param MUX is the iomux number (e.g. 23 for PINCM23)
///
/// @param PORT is the GPIO port letter (e.g. A)
///
/// @param NUM is the pin number in two-digit form (e.g. 03)
///
/// Example:
///  GPIO_PIN(LED, GpioOutputSafeLow, 1, A, 0);
///  ...
///  LED_Pin::set(true);
#define GPIO_PIN(NAME, BaseDefs, MUX, PORT, NUM, ARGS...)                      \
    struct NAME##Defs                                                          \
    {                                                                          \
        using Mode = MspM0GpioOptions::Mode;                                   \
        using Pull = MspM0GpioOptions::Pull;                                   \
        using OD = MspM0GpioOptions::OD;                                       \
        using Invert = MspM0GpioOptions::Invert;                               \
        using DriveStrength = MspM0GpioOptions::DriveStrength;                 \
        using Function = MspM0GpioOptions::Function;                           \
        using Safe = MspM0GpioOptions::Safe;                                   \
        using CM = MspM0GpioOptions::CM;                                       \
        using GpioBase = MspM0GpioOptions::GpioBase;                           \
        using Pin = MspM0GpioOptions::Pin;                                     \
        static_assert(1 == IOMUX_PINCM##MUX##_PF_GPIO##PORT##_DIO##NUM,        \
            "Wrong GPIO port/number or wrong IOMUX number");                   \
        static constexpr MspM0GpioOptions opts {GpioBase(GPIO##PORT##_BASE),   \
            Pin(DL_GPIO_PIN_##NUM), CM(IOMUX_PINCM##MUX), ##ARGS, BaseDefs};   \
        static constexpr unsigned PIN_NUM = NUM;                               \
    };                                                                         \
    typedef MspM0GpioPin<NAME##Defs> NAME##_Pin;

/// Defines the linker symbol for the wrapped Gpio instance.
template <unsigned GPIO_BASE, unsigned GPIO_PIN>
const Mspm0Gpio<GPIO_BASE, GPIO_PIN> Mspm0Gpio<GPIO_BASE, GPIO_PIN>::instance_;

#endif //_FREERTOS_DRIVERS_TI_MSPM0GPIO_HXX_
