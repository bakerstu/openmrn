/** \copyright
 * Copyright (c) 2023, Mike Dunston
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
 * \file Esp32AdcOneShot.hxx
 *
 * Helper declarations for using ADC pins via the ADC One-Shot ESP-IDF APIs.
 *
 * @author Mike Dunston
 * @date 8 Feburary 2023
 */

#ifndef _DRIVERS_ESP32ADCONESHOT_HXX_
#define _DRIVERS_ESP32ADCONESHOT_HXX_

#include "utils/logging.h"

#if defined(__has_include)
#if __has_include(<driver/adc_types_legacy.h>)
// include legacy types so ADC1_CHANNEL_0 and ADC1_CHANNEL_0_GPIO_NUM etc are
// defined, soc/adc_channel.h includes references to these on release/v5.0
// branch and is planned for update in later relase versions.
#include <driver/adc_types_legacy.h>
#endif // __has_include driver/adc_types_legacy.h
#endif // has_include

#include <esp_adc/adc_oneshot.h>
#include <soc/adc_channel.h>

/// Defines an ADC input pin.
///
/// Do not use this class directly. Use @ref ADC_PIN instead.
template <class Defs> struct Esp32ADCInput : public Defs
{
public:
    using Defs::ATTEN;
    using Defs::BITS;
    using Defs::CHANNEL;
    using Defs::PIN;
    using Defs::HANDLE;

    static void hw_init()
    {
        // due to using #if/#elif/#endif it is not possible to include this in
        // the ADC_PIN wrapper code.
        const adc_oneshot_unit_init_cfg_t unit_config =
        {
#if CONFIG_IDF_TARGET_ESP32
            .unit_id = PIN >= 30 ? ADC_UNIT_1 : ADC_UNIT_2,
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
            .unit_id = PIN <= 10 ? ADC_UNIT_1 : ADC_UNIT_2,
#elif CONFIG_IDF_TARGET_ESP32C3
            .unit_id = PIN <= 4 ? ADC_UNIT_1 : ADC_UNIT_2,
#endif
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        const adc_oneshot_chan_cfg_t channel_config =
        {
            .atten = ATTEN,
            .bitwidth = BITS,
        };

        LOG(VERBOSE,
            "[Esp32ADCInput] Configuring ADC%d:%d input pin %d, "
            "attenuation %d, bits %d",
            unit_config.unit_id, CHANNEL, PIN, ATTEN, BITS);
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_config, &HANDLE));
        ESP_ERROR_CHECK(
            adc_oneshot_config_channel(HANDLE, CHANNEL, &channel_config));
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
        ESP_ERROR_CHECK(adc_oneshot_read(HANDLE, CHANNEL, &value));
        return value;
    }
};

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
        static const adc_bitwidth_t BITS = (adc_bitwidth_t)BIT_RANGE;          \
        static adc_oneshot_unit_handle_t HANDLE;                               \
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
    adc_oneshot_unit_handle_t NAME##Defs::HANDLE;                              \
    typedef Esp32ADCInput<NAME##Defs> NAME##_Pin

#endif // _DRIVERS_ESP32ADCONESHOT_HXX_
