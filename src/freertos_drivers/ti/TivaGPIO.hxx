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
 * \file TivaGPIO.hxx
 *
 * Helper declarations for using GPIO pins (both for GPIO and other hardware)
 * on Tiva MCUs.
 *
 * @author Balazs Racz
 * @date 2 Dec 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
#define _FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_

#include "os/Gpio.hxx"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom_map.h"

/// Helper macro for declaring a pin.
///
/// @deprecated, use @ref GPIO_PIN instead.
#define DECL_PIN(NAME, PORT, NUM)                                              \
    static const auto NAME##_PERIPH = SYSCTL_PERIPH_GPIO##PORT;                \
    static const auto NAME##_BASE = GPIO_PORT##PORT##_BASE;                    \
    static const auto NAME##_PIN = GPIO_PIN_##NUM;                             \

/// Helper macro for declaring a GPIO pin wiht a specific hardware config (not
/// GPIO but a different hardware muxed onto the same pin).
///
/// @deprecated, use @ref GPIO_PIN instead.
#define DECL_HWPIN(NAME, PORT, NUM, CONFIG, Type)                              \
    DECL_PIN(NAME, PORT, NUM);                                                 \
    static void NAME##_SetPinType()                                            \
    {                                                                          \
        MAP_GPIOPinType##Type(NAME##_BASE, NAME##_PIN);                        \
    }                                                                          \
    static const auto NAME##_CONFIG = GPIO_P##PORT##NUM##_##CONFIG

template <class Defs, bool SAFE_VALUE> struct GpioOutputPin;
template <class Defs, uint32_t GPIO_PULL> struct GpioInputPin;

/// Generic GPIO class implementation.
template <unsigned GPIO_BASE, unsigned GPIO_PIN> class TivaGpio : public Gpio
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the data section.
    constexpr TivaGpio()
    {
    }

    void write(Value new_state) const OVERRIDE
    {
        *pin_address() = (new_state ? 0xff : 0);
    }

    void set() const OVERRIDE
    {
        *pin_address() = 0xff;
    }

    void clr() const OVERRIDE
    {
        *pin_address() = 0;
    }

    Value read() const OVERRIDE
    {
        return *pin_address() ? VHIGH : VLOW;
    }

    void set_direction(Direction dir) const OVERRIDE
    {
        if (dir == Direction::DOUTPUT)
        {
            GPIOPinTypeGPIOOutput(GPIO_BASE, GPIO_PIN);
        }
        else
        {
            GPIOPinTypeGPIOInput(GPIO_BASE, GPIO_PIN);
        }
    }

    Direction direction() const OVERRIDE
    {
        uint32_t mode = GPIODirModeGet(GPIO_BASE, GPIO_PIN);
        switch (mode)
        {
            default:
                HASSERT(0);
            case GPIO_DIR_MODE_IN:
                return Direction::DINPUT;
            case GPIO_DIR_MODE_OUT:
                return Direction::DOUTPUT;
        }
    }

private:
    template <class Defs> friend struct GpioShared;
    /// Static instance variable that can be used for libraries expectiong a
    /// generic Gpio pointer. This instance variable will be initialized by the
    /// linker and (assuming the application developer initialized the hardware
    /// pins in hw_preinit) is accessible, including virtual methods at static
    /// constructor time.
    static const TivaGpio instance_;

    /// Computes the memory address where the bit referring to this pin can be
    /// accessed. This address is bit-masked to the single individual pin, so
    /// only ever one bit can be read to be non-zero, and setting any other bit
    /// than the desired has no effect. This allows write with 0xff and 0x00 to
    /// set/clear and read != 0 to test.
    /// @return magic address.
    constexpr volatile uint8_t *pin_address() const
    {
        return reinterpret_cast<volatile uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
    }
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <unsigned GPIO_BASE, unsigned GPIO_PIN>
const TivaGpio<GPIO_BASE, GPIO_PIN> TivaGpio<GPIO_BASE, GPIO_PIN>::instance_;

/// Shared class that defines static functions used by both GpioInputPin,
/// GpioOutputPin and GpioHwPin.
template<class Defs> struct GpioShared : public Defs {
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;

    /// Used to unlock special consideration pins such as JTAG or NMI pins.
    static void unlock()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_SysCtlDelay(26);
        HWREG(GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_BASE + GPIO_O_CR) |= GPIO_PIN;
        HWREG(GPIO_BASE + GPIO_O_LOCK) = 0;
    }
    /// Used to lock special consideration pins such as JTAG or NMI pins.
    static void lock()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        HWREG(GPIO_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_BASE + GPIO_O_CR) &= ~GPIO_PIN;
        HWREG(GPIO_BASE + GPIO_O_LOCK) = 0;
    }  

    /// Sets the output pin to a specified value; @param value if true, output
    /// is set to HIGH otherwise LOW.
    static void set(bool value)
    {
        HWREGB(ptr_address()) = value ? 0xff : 0;
    }
    /// @return current value of an input pin, if true HIGH, of false LOW.
    static bool get()
    {
        return HWREGB(ptr_address()) != 0;
    }
    /// Changes the value of an output pin.
    static void toggle()
    {
        set(!get());
    }

    /// @return the IO address of the port that controls the specific pin.
    static constexpr uint32_t ptr_address()
    {
        return GPIO_BASE + (((unsigned)GPIO_PIN) << 2);
    }

    /// @return static Gpio ovject instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &TivaGpio<GPIO_BASE, GPIO_PIN>::instance_;
    }
};

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs, bool SAFE_VALUE>
struct GpioOutputPin : public GpioShared<Defs>
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    using GpioShared<Defs>::set;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeGPIOOutput(GPIO_BASE, GPIO_PIN);
        set(SAFE_VALUE);
    }
    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        hw_init();
    }

    /// @return true if this pin is an output pin.
    static bool is_output()
    {
        return true;
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

/// Defines a GPIO output pin with high-current drive and a defined
/// initialization level
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs, bool SAFE_VALUE> struct GpioOutputOD : public GpioOutputPin<Defs, SAFE_VALUE>
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        GpioOutputPin<Defs, SAFE_VALUE>::hw_init();
        MAP_GPIOPadConfigSet(
            GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
    }
};

/// Defines a GPIO open-drain output pin with low initialization level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
using GpioOutputODSafeLow = GpioOutputOD<Defs, false>;

/// Defines a GPIO open-drain output pin with high (==not pulled low)
/// initialization level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
using GpioOutputODSafeHigh = GpioOutputOD<Defs, true>;


/// Defines a GPIO output pin with high-current drive and low initialization
/// level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct LedPin : public GpioOutputPin<Defs, false>
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initialized the hardware pin.
    static void hw_init()
    {
        GpioOutputPin<Defs, false>::hw_init();
        MAP_GPIOPadConfigSet(
            GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_8MA_SC, GPIO_PIN_TYPE_STD);
    }
};

/// Helper macro for defining GPIO pins on the Tiva microcontrollers.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param PORT is the letter (e.g. D)
///
/// @param NUM is the pin number, such as 3
///
/// Example:
///  GPIO_PIN(FOO, LedPin, D, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, PORT, NUM)                                   \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_PIN(GPIO, PORT, NUM);                                             \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

/// Common class for GPIO input pins.
template <class Defs, uint32_t GPIO_PULL>
struct GpioInputPin : public GpioShared<Defs>
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeGPIOInput(GPIO_BASE, GPIO_PIN);
        MAP_GPIOPadConfigSet(GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PULL);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// @return true if the pin is set to an output.
    static bool is_output()
    {
        return false;
    }
};

/// GPIO Input pin with weak pull up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPU : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD_WPU>
{
};

/// GPIO Input pin with weak pull down.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputPD : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD_WPD>
{
};

/// GPIO Input pin in standard configuration (no pull).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioInputNP : public GpioInputPin<Defs, GPIO_PIN_TYPE_STD>
{
};

/// Common class for GPIO input/output pins.
template <class Defs> struct GpioInputOutputPin : public GpioShared<Defs>
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeGPIOInput(GPIO_BASE, GPIO_PIN);
        MAP_GPIOPadConfigSet(GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA,
             GPIO_PIN_TYPE_STD);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// Sets the direction of the I/O pin.
    /// @param direction direction to set pin to
    static void set_direction(Gpio::Direction direction)
    {
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN,
            direction == Gpio::Direction::DINPUT ?
            GPIO_DIR_MODE_IN : GPIO_DIR_MODE_OUT);
    }
    /// @return true if the pin is set to an output.
    static bool is_output()
    {
        return (MAP_GPIODirModeGet(GPIO_BASE, GPIO_PIN) == GPIO_DIR_MODE_OUT);
    }
};

/// GPIO Input pin in ADC configuration (analog).
///
/// This pin cannot be read or written directly (will fail compilation).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioADCPin : public Defs
{
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Implements hw_init functionality for this pin only.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_HW);
        MAP_GPIOPadConfigSet(
            GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_ANALOG);
    }
    /// Implements hw_set_to_safe functionality for this pin only.
    static void hw_set_to_safe()
    {
        /// TODO(balazs.racz): we need to somehow specify what to do to be safe.
        /// Options are drive low, drive high, input std, input wpu, input wpd.
        hw_init();
    }
};

/// GPIO Input pin for USB.
///
/// This pin cannot be read or written directly (will fail compilation).
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioUSBAPin : public Defs
{
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinTypeUSBAnalog(GPIO_BASE, GPIO_PIN);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
};

/// GPIO pin in a hardware configuration (via pinmux: UART, I2C, CAN, etc).
///
/// The pin can be switched to hardware, input and output mode. In input.output
/// mode the pin can be read and written using the usual set() and get()
/// functions.
///
/// Do not use this class directly. Use @ref GPIO_HWPIN instead.
template <class Defs> struct GpioHwPin : public GpioShared<Defs>
{
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    using Defs::GPIO_CONFIG;
    using Defs::GPIO_SetPinType;

    /// Implements hw_init functionality for this pin only.
    static void hw_init()
    {
        MAP_SysCtlPeripheralEnable(GPIO_PERIPH);
        MAP_GPIOPinConfigure(GPIO_CONFIG);
        set_hw();
    }

    /// Implements hw_set_to_safe functionality for this pin only.
    static void hw_set_to_safe()
    {
        /// @todo(balazs.racz): we need to somehow specify what to do to be
        /// safe.  Options are drive low, drive high, input std, input wpu,
        /// input wpd.
        hw_init();
    }

    /** Switches the GPIO pin to the hardware peripheral. */
    static void set_hw()
    {
        Defs::GPIO_SetPinType();
    }

    /** Switches the GPIO pin to an output pin. Use the set() command to define
     * the value. */
    static void set_output()
    {
        MAP_GPIOPinTypeGPIOOutput(GPIO_BASE, GPIO_PIN);
        MAP_GPIOPadConfigSet(
            GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);
    }

    /** Switches the GPIO pin to an input pin. Use the get() command to
     * retrieve the value.
     *
     * @param drive_type specifies whether there should be a weak pullup
     * (GPIO_PIN_TYPE_STD_WPU), pull-down (GPIO_PIN_TYPE_STD_WPD) or standard
     * pin (default, GPIO_PIN_TYPE_STD)*/
    static void set_input(uint32_t drive_type = GPIO_PIN_TYPE_STD)
    {
        MAP_GPIOPinTypeGPIOInput(GPIO_BASE, GPIO_PIN);
        MAP_GPIOPadConfigSet(
            GPIO_BASE, GPIO_PIN, GPIO_STRENGTH_2MA, drive_type);
    }
};

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
/// @param PORT is the letter (e.g. D)
///
/// @param NUM is the pin number, such as 3
///
/// @param CONFIG is the suffix of the symbol that defines the pinmux for the
/// hardware, e.g. U7TX.
///
/// @param TYPE is a suffix for a TivaWare GPIOPinType function such as UART
/// for GPIOPinTypeUART() to set the pin to the hardware configuration.
#define GPIO_HWPIN(NAME, BaseClass, PORT, NUM, CONFIG, TYPE)                   \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_HWPIN(GPIO, PORT, NUM, CONFIG, TYPE);                             \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

#endif //_FREERTOS_DRIVERS_TI_TIVAGPIO_HXX_
