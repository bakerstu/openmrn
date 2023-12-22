/** \copyright
 * Copyright (c) 2020, Mike Dunston
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
 * \file Esp32Gpio.hxx
 *
 * Helper declarations for using GPIO pins via the ESP-IDF APIs.
 *
 * @author Mike Dunston
 * @date 27 March 2020
 */

#ifndef _DRIVERS_ESP32GPIO_HXX_
#define _DRIVERS_ESP32GPIO_HXX_

#include "freertos_drivers/arduino/GpioWrapper.hxx"
#include "os/Gpio.hxx"
#include "utils/logging.h"
#include "utils/macros.h"

#include <driver/adc.h>
#include <driver/gpio.h>
#include <esp_idf_version.h>
#include <soc/adc_channel.h>

// esp_rom_gpio.h is a target agnostic replacement for esp32/rom/gpio.h
#if __has_include(<esp_rom_gpio.h>)
#include <esp_rom_gpio.h>
#elif ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,0,0)
#include <rom/gpio.h>
#else
#include <esp32/rom/gpio.h>
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,0,0)
#include <soc/gpio_struct.h>
#endif

#if defined(CONFIG_IDF_TARGET_ESP32C3)
/// Helper macro to test if a pin has been configured for output.
///
/// This is necessary since ESP-IDF does not expose gpio_get_direction(pin).
#define IS_GPIO_OUTPUT(pin) (GPIO_IS_VALID_OUTPUT_GPIO(pin) &&                 \
                             GPIO.enable.data & BIT(pin & 25))
#else // NOT ESP32-C3
/// Helper macro to test if a pin has been configured for output.
///
/// This is necessary since ESP-IDF does not expose gpio_get_direction(pin).
#define IS_GPIO_OUTPUT(pin) (GPIO_IS_VALID_OUTPUT_GPIO(pin) &&                 \
                             (pin < 32) ? GPIO.enable & BIT(pin & 31) :        \
                                          GPIO.enable1.data & BIT(pin & 31))
#endif // CONFIG_IDF_TARGET_ESP32C3

template <class Defs, bool SAFE_VALUE, bool INVERT> struct GpioOutputPin;
template <class Defs, bool PUEN, bool PDEN> struct GpioInputPin;

/// Defines a GPIO output pin. Writes to this structure will change the output
/// level of the pin. Reads will return the pin's current level.
///
/// The pin is set to output at initialization time, with the level defined by
/// `SAFE_VALUE'.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <gpio_num_t PIN_NUM, bool INVERTED = false>
class Esp32Gpio : public Gpio
{
public:
#if CONFIG_IDF_TARGET_ESP32S3
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 48, "Valid pin range is 0..48.");
    static_assert(!(PIN_NUM >= 22 && PIN_NUM <= 25)
                , "Pin does not exist");
    static_assert(!(PIN_NUM >= 26 && PIN_NUM <= 32)
                , "Pin is reserved for flash usage.");
#if defined(CONFIG_SPIRAM_MODE_OCT) || defined(CONFIG_ESPTOOLPY_OCT_FLASH)
    static_assert(!(PIN_NUM >= 33 && PIN_NUM <= 37)),
                  "Pin is not available when Octal SPI mode is enabled.");
#endif // ESP32S3 with Octal SPI
#elif CONFIG_IDF_TARGET_ESP32S2
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 46, "Valid pin range is 0..46.");
    static_assert(!(PIN_NUM >= 22 && PIN_NUM <= 25)
                , "Pin does not exist");
    static_assert(!(PIN_NUM >= 26 && PIN_NUM <= 32)
                , "Pin is reserved for flash usage.");
#elif CONFIG_IDF_TARGET_ESP32C3
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 21, "Valid pin range is 0..21.");
    // these pins are connected to the embedded flash and are not exposed on
    // the DevKitM-1 board.
    static_assert(!(PIN_NUM >= 11 && PIN_NUM <= 17)
                , "Pin is reserved for flash usage.");
#else // ESP32
    static_assert(PIN_NUM >= 0 && PIN_NUM <= 39, "Valid pin range is 0..39.");
    static_assert(PIN_NUM != 24, "Pin does not exist");
    static_assert(!(PIN_NUM >= 28 && PIN_NUM <= 31), "Pin does not exist");
    // Most commercially available boards include a capacitor on these two
    // pins, however, there is no available define that can be used to allow
    // usage of these when building for a custom PCB using the bare QFN chip.
    // static_assert(PIN_NUM != 37, "Pin is connected to GPIO 36 via capacitor.");
    // static_assert(PIN_NUM != 38, "Pin is connected to GPIO 39 via capacitor.");
#if defined(ESP32_PICO)
    static_assert(!(PIN_NUM >= 6 && PIN_NUM <= 8)
                , "Pin is reserved for flash usage.");
    static_assert(PIN_NUM != 11 && PIN_NUM != 16 && PIN_NUM != 17
                , "Pin is reserved for flash usage.");
#else
    static_assert(!(PIN_NUM >= 6 && PIN_NUM <= 11)
                , "Pin is reserved for flash usage.");
#if defined(BOARD_HAS_PSRAM)
    static_assert(PIN_NUM != 16 && PIN_NUM != 17
                , "Pin is reserved for PSRAM usage.");
#endif // BOARD_HAS_PSRAM
#endif // ESP32_PICO
#endif // CONFIG_IDF_TARGET_ESP32S2 / CONFIG_IDF_TARGET_ESP32S3

    /// Sets the output state of the connected GPIO pin.
    ///
    /// @param new_state State to set the GPIO pin to.
    void write(Value new_state) const override
    {
        if (INVERTED)
        {
            LOG(VERBOSE, "Esp32Gpio(%d) write %s", PIN_NUM,
                new_state == Value::SET ? "CLR" : "SET");
            ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, new_state == Value::CLR));
        }
        else
        {
            LOG(VERBOSE, "Esp32Gpio(%d) write %s", PIN_NUM,
                new_state == Value::SET ? "SET" : "CLR");
            ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, new_state));
        }
    }

    /// Reads the current state of the connected GPIO pin.
    /// @return @ref SET if currently high, @ref CLR if currently low.
    Value read() const override
    {
        return (Value)gpio_get_level(PIN_NUM);
    }

    /// Sets output to HIGH.
    void set() const override
    {
        write(Value::SET);
    }

    /// Sets output to LOW.
    void clr() const override
    {
        write(Value::CLR);
    }

    /// Sets the direction of the connected GPIO pin.
    void set_direction(Direction dir) const override
    {
        if (dir == Direction::DOUTPUT)
        {
            HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(PIN_NUM));
            // using GPIO_MODE_INPUT_OUTPUT instead of GPIO_MODE_OUTPUT so that
            // we can read the IO state
            ESP_ERROR_CHECK(
                gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT_OUTPUT));
            LOG(VERBOSE, "Esp32Gpio(%d) configured as OUTPUT", PIN_NUM);
        }
        else
        {
            ESP_ERROR_CHECK(gpio_set_direction(PIN_NUM, GPIO_MODE_INPUT));
            LOG(VERBOSE, "Esp32Gpio(%d) configured as INPUT", PIN_NUM);
        }
    }

    /// Gets the GPIO direction.
    /// @return @ref DINPUT or @ref DOUTPUT
    Direction direction() const override
    {
        if (IS_GPIO_OUTPUT(PIN_NUM))
        {
            return Direction::DOUTPUT;
        }
        return Direction::DINPUT;
    }
private:
    template <class Defs, bool SAFE_VALUE, bool INVERT> friend struct GpioOutputPin;
    template <class Defs, bool PUEN, bool PDEN> friend struct GpioInputPin;
    /// Static instance variable that can be used for libraries expecting a
    /// generic Gpio pointer. This instance variable will be initialized by the
    /// linker and (assuming the application developer initialized the hardware
    /// pins in hw_preinit) is accessible, including virtual methods at static
    /// constructor time.
    static const Esp32Gpio instance_;
};

/// Defines the linker symbol for the wrapped Gpio instance.
template <gpio_num_t PIN_NUM, bool INVERTED>
const Esp32Gpio<PIN_NUM, INVERTED> Esp32Gpio<PIN_NUM, INVERTED>::instance_;

/// Parametric GPIO output class.
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param SAFE_VALUE is the initial value for the GPIO output pin.
/// @param INVERT inverts the high/low state of the pin when set.
template <class Defs, bool SAFE_VALUE, bool INVERT = false>
struct GpioOutputPin : public Defs
{
public:
    using Defs::PIN_NUM;
// compile time sanity check that the selected pin is valid for output.
#if CONFIG_IDF_TARGET_ESP32S2
    static_assert(PIN_NUM != 46, "Pin 46 can not be used for output.");
#elif CONFIG_IDF_TARGET_ESP32
    static_assert(PIN_NUM < 34, "Pins 34 and above can not be used as output.");
#endif // CONFIG_IDF_TARGET_ESP32S2 / CONFIG_IDF_TARGET_ESP32S3

    /// Initializes the hardware pin.
    static void hw_init()
    {
        LOG(VERBOSE,
            "[Esp32Gpio] Configuring output pin %d, default value: %d",
            PIN_NUM, SAFE_VALUE);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,3,0)
        gpio_pad_select_gpio(PIN_NUM);
#else // IDF v4.4 (or later)
        esp_rom_gpio_pad_select_gpio(PIN_NUM);
#endif // IDF v4.3 (or earlier)
        gpio_config_t cfg;
        memset(&cfg, 0, sizeof(gpio_config_t));
        cfg.pin_bit_mask = BIT64(PIN_NUM);
        // using GPIO_MODE_INPUT_OUTPUT instead of GPIO_MODE_OUTPUT so that
        // we can read the IO state
        cfg.mode = GPIO_MODE_INPUT_OUTPUT;
        ESP_ERROR_CHECK(gpio_config(&cfg));
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, SAFE_VALUE));
    }

    /// Sets the hardware pin to a safe value.
    static void hw_set_to_safe()
    {
        ESP_ERROR_CHECK(gpio_set_level(PIN_NUM, SAFE_VALUE));
    }

    /// Toggles the state of the pin to the opposite of what it is currently.
    static void toggle()
    {
        instance()->write(!instance()->read());
    }

    /// Sets the output pin @param value if true, output is set to HIGH, if
    /// false, output is set to LOW.
    static void set(bool value)
    {
        instance()->write(value);
    }

    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &Esp32Gpio<PIN_NUM, INVERT>::instance_;
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
/// @param Defs is the GPIO pin's definition base class, supplied by the
/// GPIO_PIN macro.
/// @param PUEN is true if the pull-up should be enabled.
/// @param PDEN is true if the pull-down should be enabled.
template <class Defs, bool PUEN, bool PDEN> struct GpioInputPin : public Defs
{
public:
    using Defs::PIN_NUM;
#if CONFIG_IDF_TARGET_ESP32S2
    // GPIO 45 and 46 typically have pull-down resistors.
    static_assert(!PUEN || (PUEN && (PIN_NUM != 45 && PIN_NUM != 46)),
                  "GPIO 45 and 46 typically have built-in pull-down "
                  "resistors, enabling pull-up is not possible.");
    // GPIO 0 typically has a pull-up resistor
    static_assert(!PDEN || (PDEN && PIN_NUM != 0),
                  "GPIO 0 typically has a built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#elif CONFIG_IDF_TARGET_ESP32S3
    // GPIO 0 typically has a pull-up resistor
    static_assert(!PDEN || (PDEN && PIN_NUM != 0),
                  "GPIO 0 typically has a built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#elif CONFIG_IDF_TARGET_ESP32C3
    // GPIO 9 typically has a pull-up resistor
    static_assert(!PDEN || (PDEN && PIN_NUM != 9),
                  "GPIO 9 typically has a built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#else // ESP32
    // GPIO 2, 4 and 12 typically have pull-down resistors.
    static_assert(!PUEN ||
                  (PUEN && (PIN_NUM != 2 && PIN_NUM != 4 && PIN_NUM != 12)),
                  "GPIO 2, 4, 12 typically have built-in pull-down resistors, "
                  "enabling pull-up is not possible.");
    // GPIO 0, 5 and 15 typically have pull-up resistors.
    static_assert(!PDEN ||
                  (PDEN && (PIN_NUM != 0 && PIN_NUM != 5 && PIN_NUM == 15)),
                  "GPIO 0, 5, 15 typically have built-in pull-up resistors, "
                  "enabling pull-down is not possible.");
#endif // CONFIG_IDF_TARGET_ESP32S2
    /// Initializes the hardware pin.
    static void hw_init()
    {
        LOG(VERBOSE, "[Esp32Gpio] Configuring input pin %d, PUEN: %d, PDEN: %d",
            PIN_NUM, PUEN, PDEN);
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4,3,0)
        gpio_pad_select_gpio(PIN_NUM);
#else // IDF v4.4 (or later)
        esp_rom_gpio_pad_select_gpio(PIN_NUM);
#endif // IDF v4.3 (or earlier)
        gpio_config_t cfg;
        memset(&cfg, 0, sizeof(gpio_config_t));
        cfg.pin_bit_mask = BIT64(PIN_NUM);
        // using GPIO_MODE_INPUT_OUTPUT instead of GPIO_MODE_OUTPUT so that
        // we can read the IO state
        cfg.mode = GPIO_MODE_INPUT;
        if (PUEN)
        {
            cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        }
        if (PDEN)
        {
            cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
        }
        ESP_ERROR_CHECK(gpio_config(&cfg));
    }
    /// Sets the hardware pin to a safe state.
    static void hw_set_to_safe()
    {
        hw_init();
    }

    /// @return static Gpio object instance that controls this output pin.
    static constexpr const Gpio *instance()
    {
        return &Esp32Gpio<PIN_NUM>::instance_;
    }
};

/// Defines a GPIO input pin with pull-up and pull-down disabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputNP : public GpioInputPin<Defs, false, false>
{
};

/// Defines a GPIO input pin with pull-up enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPU : public GpioInputPin<Defs, true, false>
{
};

/// Defines a GPIO input pin with pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPD : public GpioInputPin<Defs, false, true>
{
};

/// Defines a GPIO input pin with pull-up and pull-down enabled.
///
/// Do not use this class directly. Use @ref GPIO_PIN instead.
template <class Defs> struct GpioInputPUPD : public GpioInputPin<Defs, true, true>
{
};

/// Defines an ADC input pin.
///
/// Do not use this class directly. Use @ref ADC_PIN instead.
template <class Defs> struct Esp32ADCInput : public Defs
{
public:
    using Defs::CHANNEL;
    using Defs::PIN;
    using Defs::ATTEN;
    using Defs::BITS;
#if CONFIG_IDF_TARGET_ESP32
    static const adc_unit_t UNIT = PIN >= 30 ? ADC_UNIT_1 : ADC_UNIT_2;
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    static const adc_unit_t UNIT = PIN <= 10 ? ADC_UNIT_1 : ADC_UNIT_2;
#elif CONFIG_IDF_TARGET_ESP32C3
    static const adc_unit_t UNIT = PIN <= 4 ? ADC_UNIT_1 : ADC_UNIT_2;
#endif
    static void hw_init()
    {
        LOG(VERBOSE,
            "[Esp32ADCInput] Configuring ADC%d:%d input pin %d, "
            "attenuation %d, bits %d",
            UNIT + 1, CHANNEL, PIN, ATTEN, BITS);

        if (UNIT == ADC_UNIT_1)
        {
            ESP_ERROR_CHECK(adc1_config_width(BITS));
            ESP_ERROR_CHECK(
                adc1_config_channel_atten((adc1_channel_t)CHANNEL, ATTEN));
        }
        else
        {
            ESP_ERROR_CHECK(
                adc2_config_channel_atten((adc2_channel_t)CHANNEL, ATTEN));
        }
    }

    /// NO-OP
    static void hw_set_to_safe()
    {
        // NO-OP
    }

    /// NO-OP
    static void set(bool value)
    {
        // NO-OP
    }

    static int sample()
    {
        int value = 0;
        if (UNIT == ADC_UNIT_1)
        {
            value = adc1_get_raw((adc1_channel_t)CHANNEL);
        }
        else
        {
            ESP_ERROR_CHECK(
                adc2_get_raw((adc2_channel_t)CHANNEL, BITS, &value));
        }
        return value;
    }
};

/// Helper macro for defining GPIO pins on the ESP32. 
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
///
/// @param BaseClass is the initialization structure, such as @ref LedPin, or
/// @ref GpioOutputSafeHigh or @ref GpioOutputSafeLow.
///
/// @param NUM is the pin number, such as 3 (see below for usable range).
///
/// There are multiple variations available for the ESP32: ESP32, WROVER,
/// WROVER-B, PICO, ESP32-Solo, ESP32-S2, ESP32-C3. Each of these have slight
/// differences in the available pins.
///
/// ESP32: Valid pin range is 0..39 with the following restrictions:
///    - 0       : Bootstrap pin, pull-up resistor on most modules.
///    - 1       : UART0 TX, serial console.
///    - 2       : Bootstrap pin, pull-down resistor on most modules.
///    - 3       : UART0 RX, serial console.
///    - 4       : Pull-down resistor on most modules.
///    - 5       : Bootstrap pin, pull-up resistor on most modules.
///    - 6 - 11  : Used for on-board flash. If you have the PICO-D4 see the
///                section below.
///    - 12      : Bootstrap pin, pull-down resistor on most modules.
///    - 15      : Bootstrap pin, pull-up resistor on most modules.
///    - 24      : Does not exist.
///    - 37, 38  : Not exposed on most modules and will have a capacitor
///                connected to 36 and 39 under the metal shielding of the
///                module. The capacitor is typically 270pF.
///    - 34 - 39 : These pins are INPUT only.
/// NOTE: ESP32 covers the ESP32-WROOM-32, DOWD, D2WD, S0WD, U4WDH and the
/// ESP32-Solo.
///
/// ESP32-PICO-D4: Nearly the same as ESP32 but with the following differences:
///    - 9, 10   : Available for use, other modules use these for the on-board
///                flash.
///    - 16, 17  : Used for flash and/or PSRAM.
///
/// ESP32-PICO-V3: Nearly the same as ESP32-PICO-D4 with the following
/// differences:
///    - 7, 8       : Available for use, however validation checks will prevent
///                   usage due to no compile time constant available to
///                   uniquely identify this variant.
///    - 20         : Available for use.
///    - 16 - 18, 23: Not available, however validations checks will not
///                   prevent usage of GPIO 18 or 23 due to no compile time
///                   constant available to uniquely identify this variant.
///
/// ESP32-WROVER and WROVER-B: Nearly the same as ESP32 but with the following
/// differences:
///    - 16, 17  : Reserved for PSRAM on WROVER/WROVER-B modules.
///
/// ESP32-S2: Valid pin range is 0..46 with the following notes:
///    - 0       : Bootstrap pin, pull-up resistor on most modules.
///    - 18      : Most modules have an RGB LED on this pin.
///    - 19      : USB OTG D-, available to use if not using this
///                functionality.
///    - 20      : USB OTG D+, available to use if not using this
///                functionality.
///    - 22 - 25 : Do not exist.
///    - 26 - 32 : Used for on-board flash and/or PSRAM.
///    - 43      : UART0 TX, serial console.
///    - 44      : UART0 RX, serial console.
///    - 45      : Bootstrap pin, pull-down resistor on most modules. Note:
///                ESP32-S2-Kaluga-1 modules have an RGB LED on this pin that
///                can be enabled via a jumper.
///    - 46      : Bootstrap pin, pull-down resistor on most modules, INPUT only.
///
/// ESP32-C3: Valid pin range is 0..21 with the following notes:
///    - 8       : Bootstrap pin, most modules have an RGB LED on this pin.
///    - 9       : Bootstrap pin, connected to built-in pull-up resistor, may
///                also have an external pull-up resistor.
///    - 11 - 17 : Used for flash and/or PSRAM.
///    - 18      : USB CDC-ACM D- / JTAG, available to use if not using this
///                functionality.
///    - 19      : USB CDC ACM D+ / JTAG, available to use if not using this
///                functionality.
///    - 20      : UART0 RX, serial console.
///    - 21      : UART0 TX, serial console.
///
/// ESP8685: This is an ESP32-C3 in a smaller package without SPI pins exposed.
///
/// ESP32-S3: Valid pin range is 0..48 with the following notes:
///    - 0       : Bootstrap pin, pull-up resistor on most modules.
///    - 18      : Most modules have an RGB LED on this pin.
///    - 19      : USB OTG D-, available to use if not using this
///                functionality.
///    - 20      : USB OTG D+, available to use if not using this
///                functionality.
///    - 22 - 25 : Do not exist.
///    - 26 - 32 : Used for on-board flash and/or PSRAM.
///    - 33 - 37 : Used for on-board flash and/or PSRAM if Octal SPI mode is
///                enabled.
///    - 39 - 42 : JTAG interface (pins in order: MTCK, MTDO, MTDI, MTMS)
///    - 43      : UART0 TX, serial console.
///    - 44      : UART0 RX, serial console.
///    - 45      : Bootstrap pin, pull-down resistor on most modules.
///    - 46      : Bootstrap pin, pull-down resistor on most modules.
///    - 48      : Most modules have an RGB LED on this pin.
///
/// Pins marked as having a pull-up or pull-down resistor are typically 10kOhm.
///
/// Any pins marked as a bootstrap pin can alter the behavior of the ESP32 if
/// they are not in the default/expected state on startup. Consult the
/// schematic and/or datasheet for more details.
///
/// The built in pull-up/pull-down resistor for all ESP32 variants are
/// typically 45kOhm.
///
/// SoC datasheet references:
/// ESP32: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf
/// ESP32-WROVER: https://www.espressif.com/sites/default/files/documentation/esp32-wrover_datasheet_en.pdf
/// ESP32-WROVER-B: https://www.espressif.com/sites/default/files/documentation/esp32-wrover-b_datasheet_en.pdf
/// ESP32-PICO-D4: https://www.espressif.com/sites/default/files/documentation/esp32-pico-d4_datasheet_en.pdf
/// ESP32-PICO-V3: https://www.espressif.com/sites/default/files/documentation/esp32-pico-v3_datasheet_en.pdf
/// ESP32-S2: https://www.espressif.com/sites/default/files/documentation/esp32-s2_datasheet_en.pdf
/// ESP32-S2-WROVER: https://www.espressif.com/sites/default/files/documentation/esp32-s2-wrover_esp32-s2-wrover-i_datasheet_en.pdf
/// ESP32-C3: https://www.espressif.com/sites/default/files/documentation/esp32-c3_datasheet_en.pdf
/// ESP32-S3: https://www.espressif.com/sites/default/files/documentation/esp32-s3_datasheet_en.pdf
/// ESP8685: https://www.espressif.com/sites/default/files/documentation/esp8685_datasheet_en.pdf
///
/// SoC technical references:
/// ESP32: https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf
/// ESP32-S2: https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf
/// ESP32-C3: https://www.espressif.com/sites/default/files/documentation/esp32-c3_technical_reference_manual_en.pdf
/// ESP32-S3: https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf
///
/// Module schematic references:
/// DevKitC v4: https://dl.espressif.com/dl/schematics/esp32_devkitc_v4-sch-20180607a.pdf
/// DevKitC v2: https://dl.espressif.com/dl/schematics/ESP32-Core-Board-V2_sch.pdf
/// WROVER KIT v4.1: https://dl.espressif.com/dl/schematics/ESP-WROVER-KIT_V4_1.pdf
/// WROVER KIT v3: https://dl.espressif.com/dl/schematics/ESP-WROVER-KIT_SCH-3.pdf
/// WROVER KIT v2: https://dl.espressif.com/dl/schematics/ESP-WROVER-KIT_SCH-2.pdf
/// WROVER KIT v1: https://dl.espressif.com/dl/schematics/ESP32-DevKitJ-v1_sch.pdf
/// PICO KIT v4.1: https://dl.espressif.com/dl/schematics/esp32-pico-kit-v4.1_schematic.pdf
/// PICO KIT v3: https://dl.espressif.com/dl/schematics/esp32-pico-kit-v3_schematic.pdf
/// ESP32-S2-Saola-1: https://dl.espressif.com/dl/schematics/ESP32-S2-SAOLA-1_V1.1_schematics.pdf
/// ESP32-S2-Kaluga-1: https://dl.espressif.com/dl/schematics/ESP32-S2-Kaluga-1_V1_3_SCH_20200526A.pdf
/// ESP32-C3-DevKitM-1: https://dl.espressif.com/dl/schematics/SCH_ESP32-C3-DEVKITM-1_V1_20200915A.pdf
///
/// NOTE: The WROVER KIT v1 is also known as DevKitJ and is RED colored PCB
/// that supports both WROVER and WROOM-32 modules.
///
/// Example:
///  GPIO_PIN(FOO, GpioOutputSafeLow, 3);
///  ...
///  FOO_Pin::set(true);
#define GPIO_PIN(NAME, BaseClass, NUM)                                         \
    struct NAME##Defs                                                          \
    {                                                                          \
        static const gpio_num_t PIN_NUM = (gpio_num_t)NUM;                     \
    public:                                                                    \
        static const gpio_num_t pin()                                          \
        {                                                                      \
            return PIN_NUM;                                                    \
        }                                                                      \
    };                                                                         \
    typedef BaseClass<NAME##Defs> NAME##_Pin

/// Helper macro for an ADC GPIO input on the ESP32.
///
/// @param NAME is the basename of the declaration. For NAME==FOO the macro
/// declared FOO_Pin as a structure on which the read-write functions will be
/// available.
/// @param ADC_CHANNEL is the ADC channel to configure.
/// @param ATTENUATION is the voltage range for the ADC input.
/// @param BIT_RANGE is the bit range to configure the ADC to use.
///
/// Supported ATTENUATION values and voltage ranges:
/// ADC_ATTEN_DB_0   - 0dB attenuaton gives full-scale voltage 1.1V
/// ADC_ATTEN_DB_2_5 - 2.5dB attenuation gives full-scale voltage 1.5V
/// ADC_ATTEN_DB_6   - 6dB attenuation gives full-scale voltage 2.2V
/// ADC_ATTEN_DB_11  - 11dB attenuation gives full-scale voltage 3.9V
///
/// Supported BIT_RANGE values and ADC sample values:
/// ADC_WIDTH_BIT_9  - 0-511
/// ADC_WIDTH_BIT_10 - 0-1023
/// ADC_WIDTH_BIT_11 - 0-2047
/// ADC_WIDTH_BIT_12 - 0-4065
/// ADC_WIDTH_BIT_13 - 0-8191 -- Only valid on the ESP32-S2 and ESP32-S3.
/// NOTE: When using ADC1_CHANNEL_X this bit range will be applied to all
/// ADC1 channels, it is not recommended to mix values for ADC1 channels.
///
/// Supported ADC_CHANNEL values and pin assignments for the ESP32:
/// ADC1_CHANNEL_0 : 36
/// ADC1_CHANNEL_1 : 37 -- NOTE: Not recommended for use, see note below.
/// ADC1_CHANNEL_2 : 38 -- NOTE: Not recommended for use, see note below.
/// ADC1_CHANNEL_3 : 39
/// ADC1_CHANNEL_4 : 32
/// ADC1_CHANNEL_5 : 33
/// ADC1_CHANNEL_6 : 34
/// ADC1_CHANNEL_7 : 35
/// ADC2_CHANNEL_0 : 4  -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_1 : 0  -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_2 : 2  -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_3 : 15 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_4 : 13 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_5 : 12 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_6 : 14 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_7 : 27 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_8 : 25 -- NOTE: Not usable when WiFi is active.
/// ADC2_CHANNEL_9 : 29 -- NOTE: Not usable when WiFi is active.
/// NOTE: ADC1_CHANNEL_1 and ADC1_CHANNEL_2 typically have a capacitor which
/// connects to ADC1_CHANNEL_0 or ADC1_CHANNEL_3. The only known exception to
/// this is for some ESP32-PICO-D4/ESP32-PICO-V3 based boards, confirm on the
/// board schematic before using these pins.
///
/// Supported ADC_CHANNEL values and pin assignments for the ESP32-S2/ESP32-S3:
/// ADC1_CHANNEL_0 : 1
/// ADC1_CHANNEL_1 : 2
/// ADC1_CHANNEL_2 : 3
/// ADC1_CHANNEL_3 : 4
/// ADC1_CHANNEL_4 : 5
/// ADC1_CHANNEL_5 : 6
/// ADC1_CHANNEL_6 : 7
/// ADC1_CHANNEL_7 : 8
/// ADC1_CHANNEL_8 : 9
/// ADC1_CHANNEL_9 : 10
/// ADC2_CHANNEL_0 : 11
/// ADC2_CHANNEL_1 : 12
/// ADC2_CHANNEL_2 : 13
/// ADC2_CHANNEL_3 : 14
/// ADC2_CHANNEL_4 : 15
/// ADC2_CHANNEL_5 : 16
/// ADC2_CHANNEL_6 : 17
/// ADC2_CHANNEL_7 : 18
/// ADC2_CHANNEL_8 : 19 -- NOTE: This pin is also used for USB PHY (D-).
/// ADC2_CHANNEL_9 : 20 -- NOTE: This pin is also used for USB PHY (D+).
///
/// Supported ADC_CHANNEL values and pin assignments for the ESP32-C3:
/// ADC1_CHANNEL_0 : 0
/// ADC1_CHANNEL_1 : 1
/// ADC1_CHANNEL_2 : 2
/// ADC1_CHANNEL_3 : 3
/// ADC1_CHANNEL_4 : 4
/// ADC2_CHANNEL_0 : 5
///
/// Example:
///   ADC_PIN(SENSE, ADC1_CHANNEL_0, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12);
///   ...
///   int level = SENSE_Pin::sample();
#define ADC_PIN(NAME, ADC_CHANNEL, ATTENUATION, BIT_RANGE)                     \
    struct NAME##Defs                                                          \
    {                                                                          \
        static const adc_channel_t CHANNEL = (adc_channel_t)ADC_CHANNEL;       \
        static const gpio_num_t PIN = (gpio_num_t)ADC_CHANNEL##_GPIO_NUM;      \
        static const adc_atten_t ATTEN = (adc_atten_t)ATTENUATION;             \
        static const adc_bits_width_t BITS = (adc_bits_width_t)BIT_RANGE;      \
    public:                                                                    \
        static const gpio_num_t pin()                                          \
        {                                                                      \
            return PIN;                                                        \
        }                                                                      \
        static const adc_channel_t channel()                                   \
        {                                                                      \
            return CHANNEL;                                                    \
        }                                                                      \
    };                                                                         \
    typedef Esp32ADCInput<NAME##Defs> NAME##_Pin

#endif // _DRIVERS_ESP32GPIO_HXX_