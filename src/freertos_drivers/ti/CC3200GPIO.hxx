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
 * \file CC3200GPIO.hxx
 *
 * Helper declarations for using GPIO pins on CC3200 MCUs.
 *
 * Note about barriers:
 *
 * GCC is extremely efficient at optimizing the memory access instructions that
 * we use for writing to GPIO output pins. In many cases writing to multiple
 * GPIO pins turns into back-to-back Thumb instructions, and the hardware
 * peripheral seems to be unable to process bus transactions this fast. We
 * therefore add a barrier after each GPIO write. The barrier ensures that the
 * execution continues after the GPIO write only once the transaction is
 * successfully completed by the bus. We tested that back to back GPIO writes
 * are operational. The barrier also ensures correct sequencing against other
 * effects of the running program. One GPIO write is about 50 nsec (4 clock
 * cycles), the shortest pulse we can generate is 100 nsec.
 *
 * @author Balazs Racz
 * @date 2 Dec 2014
 */

#ifndef _FREERTOS_DRIVERS_TI_CC3200GPIO_HXX_
#define _FREERTOS_DRIVERS_TI_CC3200GPIO_HXX_

#include "os/Gpio.hxx"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/prcm.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom_map.h"


/// Helper macro for declaring a pin.
///
/// @deprecated, use @ref GPIO_PIN instead.
#define DECL_PIN(NAME, PORT, NUM)                           \
    static const auto NAME##_PERIPH = PRCM_GPIO##PORT;      \
    static const auto NAME##_BASE = GPIO##PORT##_BASE; \
    static const auto NAME##_PIN = GPIO_PIN_##NUM; \
    static const auto NAME##_INT = INT_GPIO##PORT
    

template <class Defs, bool SAFE_VALUE> struct GpioOutputPin;
template <class Defs> struct GpioInputPin;

/// Generic GPIO class implementation.
template <unsigned GPIO_BASE, unsigned GPIO_PIN> class CC3200Gpio : public Gpio
{
public:
    /// This constructor is constexpr which ensures that the object can be
    /// initialized in the data section.
    constexpr CC3200Gpio()
    {
    }

    void write(Value new_state) const OVERRIDE
    {
        *pin_address() = (new_state ? 0xff : 0);
        /// See note at the top of the file about barriers.
        __asm__ volatile("dsb" : : : "memory");
    }

    void set() const OVERRIDE
    {
        *pin_address() = 0xff;
        /// See note at the top of the file about barriers.
        __asm__ volatile("dsb" : : : "memory");
    }

    void clr() const OVERRIDE
    {
        *pin_address() = 0;
        /// See note at the top of the file about barriers.
        __asm__ volatile("dsb" : : : "memory");
    }

    Value read() const OVERRIDE
    {
        return *pin_address() ? VHIGH : VLOW;
    }

    void set_direction(Direction dir) const OVERRIDE
    {
        if (dir == Direction::DOUTPUT)
        {
            MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_OUT);
        }
        else
        {
            MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_IN);
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
    template <class Defs, bool SAFE_VALUE> friend struct GpioOutputPin;
    template <class Defs> friend struct GpioInputPin;
    /// Static instance variable that can be used for libraries expectiong a
    /// generic Gpio pointer. This instance variable will be initialized by the
    /// linker and (assuming the application developer initialized the hardware
    /// pins in hw_preinit) is accessible, including virtual methods at static
    /// constructor time.
    static const CC3200Gpio instance_;

    /// Computes the memory address where the bit referring to this pin can be
    /// accessed. This address is bit-masked to the single individual pin, so
    /// only ever one bit can be read to be non-zero, and setting any other bit
    /// than the desired has no effect. This allows write with 0xff and 0x00 to
    /// set/clear and read != 0 to test. @return memory address.
    constexpr volatile uint8_t *pin_address() const
    {
        return reinterpret_cast<volatile uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
    }
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <unsigned GPIO_BASE, unsigned GPIO_PIN>
const CC3200Gpio<GPIO_BASE, GPIO_PIN> CC3200Gpio<GPIO_BASE, GPIO_PIN>::instance_;

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs, bool SAFE_VALUE> struct GpioOutputPin : public Defs
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_PRCMPeripheralClkEnable(GPIO_PERIPH, PRCM_RUN_MODE_CLK);
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_OUT);
        set(SAFE_VALUE);
    }
    /// Sets the output pin to a safe value.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// Sets the output pin to a defined value. @param value if true, output
    /// will be set to HIGH, otherwise to LOW.
    static void __attribute__((always_inline)) set(bool value)
    {
        volatile uint8_t *ptr = reinterpret_cast<uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        *ptr = value ? 0xff : 0;
        /// See note at the top of the file about barriers.
        __asm__ volatile("dsb" : : : "memory");
    }
    /// @return current value of the input pin: if true HIGH.
    static bool __attribute__((always_inline)) get()
    {
        const volatile uint8_t *ptr = reinterpret_cast<const uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        return *ptr;
    }
    /// Changes the value of an output pin.
    static void __attribute__((always_inline)) toggle()
    {
        set(!get());
    }

    /// @return static Gpio ovject instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &CC3200Gpio<GPIO_BASE, GPIO_PIN>::instance_;
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

/// Helper macro for defining GPIO pins on the CC3200 microcontrollers.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param PORT is the letter/number (e.g A0, A1, A2, A3)
///
/// @param NUM is the pin number, such as 3
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeHigh, A0, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, PORT, NUM)                                   \
    struct NAME##Defs                                                          \
    {                                                                          \
        DECL_PIN(GPIO, PORT, NUM);                                             \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

/// Common class for GPIO input pins.
template <class Defs> struct GpioInputPin : public Defs
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_PRCMPeripheralClkEnable(GPIO_PERIPH, PRCM_RUN_MODE_CLK);
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_IN);
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// @return true if the pin input is seeing HIGH.
    static bool get()
    {
        const volatile uint8_t *ptr = reinterpret_cast<const uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        return *ptr;
    }
    /// @return true if the pin is set to an output.
    static bool is_output()
    {
        return false;
    }
    /// @return the static Gpio instance controlling this pin.
    static constexpr const Gpio *instance()
    {
        return &CC3200Gpio<GPIO_BASE, GPIO_PIN>::instance_;
    }
};

/// Common class for GPIO input pins.
template <class Defs> struct GpioInputOutputPin : public Defs
{
public:
    using Defs::GPIO_PERIPH;
    using Defs::GPIO_BASE;
    using Defs::GPIO_PIN;
    /// Initializes the hardware pin.
    static void hw_init()
    {
        MAP_PRCMPeripheralClkEnable(GPIO_PERIPH, PRCM_RUN_MODE_CLK);
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN, GPIO_DIR_MODE_IN);
    }
    /// Sets the output pin to a safe value.
    static void hw_set_to_safe()
    {
        hw_init();
    }
    /// Sets the output pin to a defined value. @param value if true, output
    /// will be set to HIGH, otherwise to LOW.
    static void __attribute__((always_inline)) set(bool value)
    {
        volatile uint8_t *ptr = reinterpret_cast<uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        *ptr = value ? 0xff : 0;
        /// See note at the top of the file about barriers.
        __asm__ volatile("dsb" : : : "memory");
    }
    /// @return current value of the input pin: if true HIGH.
    static bool __attribute__((always_inline)) get()
    {
        const volatile uint8_t *ptr = reinterpret_cast<const uint8_t *>(
            GPIO_BASE + (((unsigned)GPIO_PIN) << 2));
        return *ptr;
    }
    /// Changes the value of an output pin.
    static void __attribute__((always_inline)) toggle()
    {
        set(!get());
    }

    /// @return static Gpio ovject instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &CC3200Gpio<GPIO_BASE, GPIO_PIN>::instance_;
    }

    /// Sets the direction of the I/O pin.
    /// @param direction direction to set pin to
    static void set_direction(Gpio::Direction direction)
    {
        MAP_GPIODirModeSet(GPIO_BASE, GPIO_PIN,
            direction == Gpio::Direction::DINPUT ?
            GPIO_DIR_MODE_IN : GPIO_DIR_MODE_OUT);
    }

    /// @return true if this pin is an output pin.
    static bool is_output()
    {
        return (MAP_GPIODirModeGet(GPIO_BASE, GPIO_PIN) == GPIO_DIR_MODE_OUT);
    }
};

#endif //_FREERTOS_DRIVERS_TI_CC3200GPIO_HXX_
