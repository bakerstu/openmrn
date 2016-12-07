/** \copyright
 * Copyright (c) 2016, Balazs Racz
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
 * \file Esp8266Gpio.hxx
 *
 * Helper declarations for using GPIO pins on the Esp8266 MCU.
 *
 * @author Balazs Racz
 * @date 16 May 2016
 */

#ifndef _DRIVERS_ESP8266_ESP8266GPIO_HXX_
#define _DRIVERS_ESP8266_ESP8266GPIO_HXX_

#include "os/Gpio.hxx"

extern "C" {
#include <gpio.h>
#include <eagle_soc.h>

/**  
  * @brief   Enable GPIO16 pin output.
  */
void gpio16_output_conf(void);

/**  
  * @brief   Set GPIO16 output level.
  * 
  * @param   value : GPIO16 output level.
  */
void gpio16_output_set(uint8 value);

/**  
  * @brief   Enable GPIO16 pin intput.
  */
void gpio16_input_conf(void);

/**  
  * @brief   Sample the value of GPIO16 input.
  * 
  * @return  the level  of GPIO16 input.
  */
uint8 gpio16_input_get(void);


}

/// Static map from gpio number to pinmux register address.
constexpr uint32_t pinmux_to_gpio_arr[] = {
    PERIPHS_IO_MUX_GPIO0_U,    // 0
    PERIPHS_IO_MUX_U0TXD_U,    // 1
    PERIPHS_IO_MUX_GPIO2_U,    // 2
    PERIPHS_IO_MUX_U0RXD_U,    // 3
    PERIPHS_IO_MUX_GPIO4_U,    // 4
    PERIPHS_IO_MUX_GPIO5_U,    // 5
    PERIPHS_IO_MUX_SD_CLK_U,   // 6
    PERIPHS_IO_MUX_SD_DATA0_U, // 7
    PERIPHS_IO_MUX_SD_DATA1_U, // 8
    PERIPHS_IO_MUX_SD_DATA2_U, // 9
    PERIPHS_IO_MUX_SD_DATA3_U, // 10
    PERIPHS_IO_MUX_SD_CMD_U,   // 11
    PERIPHS_IO_MUX_MTDI_U,     // 12
    PERIPHS_IO_MUX_MTCK_U,     // 13
    PERIPHS_IO_MUX_MTMS_U,     // 14
    PERIPHS_IO_MUX_MTDO_U,     // 15
};

/// Constexpr function that takes a GPIO number and converts it to the pinmux
/// register address.
///
/// @param gpio_pin_num the number of the pin (0..15).
/// @return the pinmux register address, e.g. PERIPHS_IO_MUX_U0RXD_U for pin 3.
constexpr uint32_t gpio_num_to_pinmux_reg(int gpio_pin_num)
{
    return pinmux_to_gpio_arr[gpio_pin_num];
}

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <int PIN_NUM, uint32_t MUXREG, uint32_t FUNC_GPIO>
class Esp8266StaticGpio
{
public:
    /// Number of pin (0..15).
    static constexpr const uint32_t PIN = PIN_NUM;
    /// Bit in a 16-bit register that is assigned to this pin.
    static constexpr const uint32_t PIN_BIT = (1 << PIN_NUM);
    /// Address of the pinmux register that belongs to this pin.
    static constexpr const uint32_t PIN_MUX_REG = MUXREG;
    /// Number of the GPIO function in this specific pinmux register.
    static constexpr const uint32_t PIN_MUX_FUNC_GPIO = FUNC_GPIO;

    /// Chooses GPIO on the pinmux.
    static void set_gpio()
    {
        PIN_FUNC_SELECT(PIN_MUX_REG, PIN_MUX_FUNC_GPIO);
    }

    /// Sets pin to output.
    static void set_output()
    {
        GPIO_REG_WRITE(GPIO_ENABLE_W1TS_ADDRESS, PIN_BIT);
    }

    /// Sets pin to input.
    static void set_input()
    {
        GPIO_REG_WRITE(GPIO_ENABLE_W1TC_ADDRESS, PIN_BIT);
    }

    /// Turns on pullup.
    static void set_pullup_on()
    {
        CLEAR_PERI_REG_MASK(PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP2 | PERIPHS_IO_MUX_SLEEP_PULLUP | PERIPHS_IO_MUX_SLEEP_PULLUP2);
        SET_PERI_REG_MASK(PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP);
    }

    /// Turns off pullup.
    static void set_pullup_off()
    {
        CLEAR_PERI_REG_MASK(
            PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP | PERIPHS_IO_MUX_PULLUP2);
    }

    /// Turns on pulldown (does not work).
    static void set_pulldown_on()
    {
        CLEAR_PERI_REG_MASK(PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP | PERIPHS_IO_MUX_SLEEP_PULLUP | PERIPHS_IO_MUX_SLEEP_PULLUP2);
        SET_PERI_REG_MASK(PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP2);
    }

    /// Turns off pulldown (does not work).
    static void set_pulldown_off()
    {
        CLEAR_PERI_REG_MASK(
            PIN_MUX_REG, PERIPHS_IO_MUX_PULLUP | PERIPHS_IO_MUX_PULLUP2);
    }

    /// Sets output to HIGH.
    static void set_on()
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TS_ADDRESS, PIN_BIT);
    }

    /// Sets output to LOW.
    static void set_off()
    {
        GPIO_REG_WRITE(GPIO_OUT_W1TC_ADDRESS, PIN_BIT);
    }

    /// @return input pin level.
    static bool get()
    {
        return (GPIO_REG_READ(GPIO_IN_ADDRESS) & PIN_BIT) != 0;
    }

    /// Set output pin level. @param value is the level to set to.
    static void set(bool value)
    {
        if (value)
        {
            set_on();
        }
        else
        {
            set_off();
        }
    }

    /// Toggles output pin value.
    static void toggle()
    {
        set(!get());
    }

    /// @return true if pin is configured as an output pin.
    static bool is_output()
    {
        return GPIO_REG_READ(GPIO_ENABLE_ADDRESS) & PIN_BIT;
    }
};

template <class Base, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::set(SAFE_VALUE);
        Base::set_output();
        Base::set_gpio();
        Base::set(SAFE_VALUE);
    }
    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        Base::set(SAFE_VALUE);
    }
    /// Sets the output pinm @param value if true, output is set to HIGH, if
    /// false, output is set to LOW.
    static void set(bool value)
    {
        if (INVERT)
        {
            Base::set(!value);
        }
        else
        {
            Base::set(value);
        }
    }
};

/// Output pin that switches between high and low state by turning on pullup
/// and pulldowns. Useful when an external override like a button is
/// expected. NOTE: does not seem to work on the esp8266, as the pulldown bit
/// seems to be doing nothing.
template <class Base>
struct GpioPullOutPin : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::set_input();
        Base::set_gpio();
        Base::set_pullup_on();
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        Base::set_pullup_on();
    }
    /// Sets the value of an output pin to a specific state; @param value if
    /// tru output pin will be set to HIGH else LOW.
    static void set(bool value)
    {
        if (value) {
            Base::set_pullup_on();
        } else {
            Base::set_pulldown_on();
        }
    }
    /// Sets the output pin to HIGH.
    static void set_on() {
        Base::set_pullup_on();
    }
    /// Sets the output pin to LOW.
    static void set_off() {
        Base::set_pulldown_on();
    }
};

/// Defines a GPIO output pin, initialized to be an output pin with low level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLow : public GpioOutputPin<Defs, false>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with low
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeLowInvert : public GpioOutputPin<Defs, false, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high level.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHigh : public GpioOutputPin<Defs, true>
{
};

/// Defines a GPIO output pin, initialized to be an output pin with high
/// level. All set() commands are acted upon by inverting the value.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs>
struct GpioOutputSafeHighInvert : public GpioOutputPin<Defs, true, true>
{
};

/// Parametric GPIO input class.
/// @param Base is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param PUEN is true if the pullup should be enabled.
template <class Base, bool PUEN> struct GpioInputPar : public Base
{
public:
    /// Initializes the hardware pin.
    static void hw_init()
    {
        Base::set_input();
        if (PUEN)
        {
            Base::set_pullup_on();
        }
        else
        {
            Base::set_pullup_off();
        }
        Base::set_gpio();
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }
};

/// Defines a GPIO input pin. No pull-up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputNP : public GpioInputPar<Defs, false>
{
};

/// Defines a GPIO input pin with pull-up.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPU : public GpioInputPar<Defs, true>
{
};

/// Helper macro for defining GPIO pins on the ESP8266 microcontroller.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 (range: 0..15; GPIO16 is not
/// supported)
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM)                                         \
    typedef BaseClass<Esp8266StaticGpio<NUM, gpio_num_to_pinmux_reg(NUM),      \
        FUNC_GPIO##NUM>> NAME##_Pin

#endif // _DRIVERS_ESP8266_ESP8266GPIO_HXX_
