/** \copyright
 * Copyright (c) 2021, Mike Dunston
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
 * \file Esp32Ledc.hxx
 *
 * ESP-IDF LEDC adapter that exposes a PWM interface.
 *
 * @author Mike Dunston
 * @date 1 June 2021
 */

#ifndef _DRIVERS_ESP32LEDC_HXX_
#define _DRIVERS_ESP32LEDC_HXX_

#include "freertos_drivers/common/PWM.hxx"
#include "utils/logging.h"
#include "utils/macros.h"
#include "utils/Uninitialized.hxx"

#include <driver/ledc.h>
#include <pthread.h>

namespace openmrn_arduino
{

/// ESP32 LEDC provider for PWM like output on GPIO pins.
///
/// This class allows creation of up to eight PWM outputs using a single PWM
/// frequency. All outputs in a single @ref Esp32Ledc instance will share the
/// same PWM frequency and will be assigned a channel sequentially.
///
/// When the more than one PWM frequency is needed for outputs it is required
/// to create multiple @ref Esp32Ledc instances, each with a unique LEDC timer
/// (LEDC_TIMER_0 through LEDC_TIMER_3) and a unique first channel
/// (LED_CHANNEL_0 through LED_CHANNEL_5 or LED_CHANNEL_7 depending on ESP32
/// variant).
///
/// The ESP32-C3 only supports six channels, whereas other variants support
/// eight. Creating more than one @ref Esp32Ledc instance will not increase the
/// number of outputs.
///
/// Example of usage:
///```
/// OpenMRN openmrn(NODE_ID);
/// Esp32Ledc ledc({16,    // Channel 0
///                 17,    // Channel 1
///                 18});  // Channel 2
/// ServoConsumer servo_0(openmrn.stack()->node(), cfg.seg().servo(), 1000,
///                       ledc.get_channel(1));
/// PWMGPO led_0(ledc.get_channel(0), 2500, 0);
/// void setup() {
///   ...
///   ledc.hw_init();
///   openmrn.begin();
///   openmrn.stack()->set_tx_activity_led(&led_0);
///   ledc.fade_channel_over_time(2, 2500, SEC_TO_MSEC(5));
///   ...
/// }
///```
class Esp32Ledc
{
public:
    /// Constructor.
    ///
    /// @param pins is the collection of output pins to use for this instance.
    /// @param first_channel is the first LEDC channel to use for this
    /// Esp32Ledc instance, default is LEDC_CHANNEL_0.
    /// @param timer_resolution is the resolution of the LEDC timer, default is
    /// 12bit.
    /// @param timer_hz is the LEDC timer tick frequency, default is 5kHz.
    /// @param timer_num is the LEDC timer to use, default is LEDC_TIMER_0.
    /// @param timer_mode is the LED timer mode to use, default is
    /// LEDC_LOW_SPEED_MODE.
    /// @param timer_clock is the LEDC timer clock source, default is
    /// LEDC_AUTO_CLK.
    ///
    /// Note: For @param timer_mode an additional value of LEDC_HIGH_SPEED_MODE
    /// is supported *ONLY* on the base ESP32 variant. Other variants of the
    /// ESP32 only support LEDC_LOW_SPEED_MODE.
    Esp32Ledc(const std::initializer_list<uint8_t> &pins,
              const ledc_channel_t first_channel = LEDC_CHANNEL_0,
              const ledc_timer_bit_t timer_resolution = LEDC_TIMER_12_BIT,
              const uint32_t timer_hz = 5000,
              const ledc_timer_t timer_num = LEDC_TIMER_0,
              const ledc_mode_t timer_mode = LEDC_LOW_SPEED_MODE,
              const ledc_clk_cfg_t timer_clock = LEDC_AUTO_CLK)
        : firstChannel_(first_channel)
        , pins_(pins)
    {
        // Ensure the pin count is valid and within range of usable channels.
        HASSERT(pins_.size() > 0 &&
                pins_.size() <= (LEDC_CHANNEL_MAX - first_channel));
        memset(&timerConfig_, 0, sizeof(ledc_timer_config_t));
        // timerConfig_.speed_mode will be assigned the SOC default mode, which
        // is either HIGH speed or LOW speed depending on the hardware support.
        timerConfig_.duty_resolution = timer_resolution;
        timerConfig_.freq_hz = timer_hz;
        timerConfig_.speed_mode = timer_mode;
        timerConfig_.timer_num = timer_num;
        timerConfig_.clk_cfg = timer_clock;
    }

    /// Initializes the LEDC peripheral.
    ///
    /// @param pins are the gpio pins to assign to the LEDC channels.
    ///
    /// NOTE: Depending on the target ESP32 device the number of LEDC channels
    /// available will be either six or eight. Exceeding this number of pins
    /// will generate a runtime failure.
    void hw_init()
    {
        LOG(VERBOSE,
            "[Esp32Ledc:%d] Configuring timer (resolution:%d, frequency:%"
            PRIu32 ")",
            timerConfig_.timer_num,
            (1 << (uint8_t)timerConfig_.duty_resolution) - 1,
            timerConfig_.freq_hz);
        ESP_ERROR_CHECK(ledc_timer_config(&timerConfig_));
        size_t count = 0;
        for (uint8_t pin : pins_)
        {
            HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(pin));

            ledc_channel_t led_channel =
                static_cast<ledc_channel_t>(firstChannel_ + count);
            LOG(VERBOSE, "[Esp32Ledc:%d] Configuring LEDC channel %d on GPIO %d",
                timerConfig_.timer_num, led_channel, pin);
            ledc_channel_config_t config;
            memset(&config, 0, sizeof(ledc_channel_config_t));
            config.gpio_num = pin;
            config.speed_mode = timerConfig_.speed_mode;
            config.channel = led_channel;
            config.timer_sel = timerConfig_.timer_num;
            ESP_ERROR_CHECK(ledc_channel_config(&config));
            channels_[count].emplace(this, led_channel, pin);
            count++;
        }
    }

    /// @return one PWM output.
    /// @param id is the output number, zero based for this @ref Esp32Ledc
    /// instance.
    PWM *get_channel(unsigned id)
    {
        HASSERT(id <= (LEDC_CHANNEL_MAX - firstChannel_));
        return &*channels_[id];
    }

    /// Transitions a PWM output from the current duty to the target duty over
    /// the provided time period.
    ///
    /// @param id is the output number, zero based for this @ref Esp32Ledc
    /// instance.
    /// @param target_duty target duty value, default is zero.
    /// @param fade_period number of milliseconds to use for the
    /// transition, default is 1000 milliseconds.
    /// @param fade_mode controls if this call is blocking or non-blocking,
    /// default is non-blocking.
    ///
    /// NOTE: One a fade request has been submitted to the hardware it can not
    /// be canceled and no other requests can be submitted or processed until
    /// the previous request has completed.
    void fade_channel_over_time(unsigned id, uint32_t target_duty = 0,
                                uint32_t fade_period = 1000,
                                ledc_fade_mode_t fade_mode = LEDC_FADE_NO_WAIT)
    {
        HASSERT(id <= (LEDC_CHANNEL_MAX - firstChannel_));
        ledc_channel_t channel =
            static_cast<ledc_channel_t>(firstChannel_ + id);
        HASSERT(0 == pthread_once(&ledcFadeOnce_, &Esp32Ledc::ledc_fade_setup));
        ESP_ERROR_CHECK(
            ledc_set_fade_time_and_start(timerConfig_.speed_mode, channel,
                 target_duty, fade_period, fade_mode));
    }

    /// Static entry point for configuring the LEDC hardware fade controller.
    ///
    /// NOTE: This method should not be invoked by the user code, it will be
    /// called automatically the first time @ref fade_channel_over_time is
    /// called on any instance of @ref Esp32Ledc.
    static void ledc_fade_setup()
    {
        // allocate the interrupt as low/medium priority (C code supported) and
        // the interrupt can be shared with other usages (if necessary).
        const int INTR_MODE_FLAGS =
            ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_SHARED;
        LOG(VERBOSE, "[Esp32Ledc] Initializing LED Fade ISR");
        ESP_ERROR_CHECK(ledc_fade_func_install(INTR_MODE_FLAGS));
    }
private:
    class Channel : public PWM
    {
    public:
        Channel(Esp32Ledc *parent, ledc_channel_t channel, uint8_t pin)
            : parent_(parent)
            , channel_(channel)
        {
        }

        void set_period(uint32_t counts) override
        {
            parent_->set_period(counts);
        }

        uint32_t get_period() override
        {
            return parent_->get_period();
        }

        void set_duty(uint32_t counts) override
        {
            parent_->set_duty(channel_, counts);
        }

        uint32_t get_duty() override
        {
            return parent_->get_duty(channel_);
        }

        uint32_t get_period_max() override
        {
            return parent_->get_period_max();
        }

        uint32_t get_period_min() override
        {
            return parent_->get_period_min();
        }
    private:
        Esp32Ledc *parent_;
        ledc_channel_t channel_;
    };

    /// Set PWM period.
    /// @param counts PWM timer frequency in Hz.
    ///
    /// NOTE: This will apply to *ALL* PWM outputs that use the same timer.
    void set_period(uint32_t counts)
    {
        ESP_ERROR_CHECK(
            ledc_set_freq(timerConfig_.speed_mode, timerConfig_.timer_num,
                counts));
    }

    /// Get PWM period.
    /// @return PWM timer frequency in Hz.
    uint32_t get_period()
    {
        return ledc_get_freq(timerConfig_.speed_mode, timerConfig_.timer_num);
    }

    /// Sets the duty cycle.
    /// @param channel PWM channel to configure
    /// @param counts duty cycle in counts
    void set_duty(ledc_channel_t channel, uint32_t counts)
    {
        ESP_ERROR_CHECK(
            ledc_set_duty(timerConfig_.speed_mode, channel, counts));
        ESP_ERROR_CHECK(ledc_update_duty(timerConfig_.speed_mode, channel));
    }

    /// Gets the duty cycle.
    /// @param channel PWM channel to retrieve the duty cycle for.
    /// @return counts duty cycle in counts
    uint32_t get_duty(ledc_channel_t channel)
    {
        return ledc_get_duty(timerConfig_.speed_mode, channel);
    }

    /// Get max period supported by the underlying LEDC timer.
    /// @return period in counts.
    uint32_t get_period_max()
    {
        return ((1 << (uint8_t)timerConfig_.duty_resolution) - 1);
    }

    /// Get min period supported by the underlying LEDC timer.
    /// @return period in counts.
    uint32_t get_period_min()
    {
        return 0;
    }

    /// First LEDC Channel for this @ref Esp32Ledc.
    const ledc_channel_t firstChannel_;

    /// LEDC Timer configuration settings.
    ledc_timer_config_t timerConfig_;

    /// Protects the initialization of LEDC Fade ISR hook.
    static pthread_once_t ledcFadeOnce_;

    /// @ref PWM instances connected to LEDC channels.
    uninitialized<Channel> channels_[LEDC_CHANNEL_MAX];

    /// Collection of GPIO pins in use by this @ref Esp32Ledc.
    std::vector<uint8_t> pins_;

    DISALLOW_COPY_AND_ASSIGN(Esp32Ledc);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32Ledc;

#endif // _DRIVERS_ESP32LEDC_HXX_