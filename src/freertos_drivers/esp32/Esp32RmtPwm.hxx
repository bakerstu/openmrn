/** @copyright
 * Copyright (c) 2026, Stuart Baker
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
 * @file Esp32RmtPwm.hxx
 *
 * ESP-IDF RMT adapter that exposes a PWM interface.
 *
 * @author Stuart Baker
 * @date 8 August 2026
 */

#ifndef _DRIVERS_ESP32RMTPWM_HXX_
#define _DRIVERS_ESP32RMTPWM_HXX_

#include "freertos_drivers/common/PWM.hxx"
#include "utils/logging.h"
#include "utils/macros.h"
#include "utils/Uninitialized.hxx"

#include <driver/gpio.h>
#include <driver/rmt_encoder.h>
#include <driver/rmt_tx.h>
#include <soc/soc_caps.h>

#include <vector>

namespace openmrn_arduino
{

/// ESP32 RMT provider for PWM like output on GPIO pins.
///
/// Unlike @ref Esp32Ledc, which multiplexes several outputs onto one shared
/// LEDC timer, each pin passed to a @ref Esp32RmtPwm instance is assigned
/// its own, fully independent RMT TX channel. There is no shared-timer or
/// "first channel" concept, and each output's frequency can be changed
/// independently of the others.
///
/// The waveform for one PWM period is expressed as a single RMT symbol and
/// transmitted with an infinite loop count, so the hardware repeats it
/// forever with no CPU or interrupt involvement once started. Because the
/// RMT hardware treats a symbol half with a duration of literal zero as an
/// end-of-transmission marker, a 0% or 100% duty cycle is represented as two
/// consecutive halves at the same (constant) level rather than a single
/// zero-duration half -- this still yields an exact, glitch-free 0% or 100%
/// output, not an approximation.
///
/// Changing the duty cycle or the frequency of a channel requires briefly
/// disabling and re-enabling that channel in order to interrupt its
/// in-progress infinite-loop transmission before a new one can be queued
/// (this is the mechanism documented by the ESP-IDF RMT driver itself for
/// terminating a looped transmission). This causes a brief, sub-millisecond
/// glitch on the affected output only -- acceptable for LED dimming and
/// similar uses.
///
/// The number of RMT channels available is limited and varies by ESP32
/// variant (as few as two usable TX channels on the ESP32-C3), and is
/// considerably smaller than the number of LEDC channels available on the
/// same hardware. The constructor will HASSERT if more pins are requested
/// than SOC_RMT_TX_CANDIDATES_PER_GROUP allows for the target SoC.
///
/// Example of usage:
///```
/// OpenMRN openmrn(NODE_ID);
/// Esp32RmtPwm rmt_pwm({16,    // pin gets its own RMT channel
///                      17,
///                      18});
/// ServoConsumer servo_0(openmrn.stack()->node(), cfg.seg().servo(), 1000,
///                       rmt_pwm.get_channel(1));
/// PWMGPO led_0(rmt_pwm.get_channel(0), 2500, 0);
/// void setup() {
///   ...
///   rmt_pwm.hw_init();
///   openmrn.begin();
///   openmrn.stack()->set_tx_activity_led(&led_0);
///   ...
/// }
///```
class Esp32RmtPwm
{
public:
    /// Constructor using std::vector.
    ///
    /// @param pins is the collection of output pins to use for this
    /// instance; each pin is assigned its own independent RMT TX channel.
    /// @param resolution_bits is the PWM duty cycle resolution in bits,
    /// range 1 to 15, default is 8 (256 duty steps).
    /// @param freq_hz is the initial PWM frequency in Hz for all channels,
    /// default is 20kHz.
    /// @param clk_src is the RMT peripheral clock source, default is
    /// RMT_CLK_SRC_DEFAULT. Per ESP-IDF, channels that belong to the same
    /// RMT group must use the same clock source.
    /// @param mem_block_symbols is the number of RMT hardware memory block
    /// symbols to reserve per channel, default is
    /// SOC_RMT_MEM_WORDS_PER_CHANNEL (exactly one hardware memory block per
    /// channel; only a single symbol is ever needed by this driver, so the
    /// default is already conservative).
    /// @param trans_queue_depth is the RMT transmit queue depth, default is
    /// 1, since only one (looped) transmission is ever in flight per
    /// channel.
    Esp32RmtPwm(const std::vector<uint8_t> &pins,
                uint8_t resolution_bits = 8, uint32_t freq_hz = 20000,
                rmt_clock_source_t clk_src = RMT_CLK_SRC_DEFAULT,
                size_t mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
                size_t trans_queue_depth = 1)
        : pins_(pins)
        , resolutionBits_(resolution_bits)
        , freqHz_(freq_hz)
        , clkSrc_(clk_src)
        , memBlockSymbols_(mem_block_symbols)
        , transQueueDepth_(trans_queue_depth)
    {
        init_helper();
    }

    /// Constructor using std::initializer_list.
    ///
    /// @param pins is the collection of output pins to use for this
    /// instance; each pin is assigned its own independent RMT TX channel.
    /// @param resolution_bits is the PWM duty cycle resolution in bits,
    /// range 1 to 15, default is 8 (256 duty steps).
    /// @param freq_hz is the initial PWM frequency in Hz for all channels,
    /// default is 20kHz.
    /// @param clk_src is the RMT peripheral clock source, default is
    /// RMT_CLK_SRC_DEFAULT. Per ESP-IDF, channels that belong to the same
    /// RMT group must use the same clock source.
    /// @param mem_block_symbols is the number of RMT hardware memory block
    /// symbols to reserve per channel, default is
    /// SOC_RMT_MEM_WORDS_PER_CHANNEL (exactly one hardware memory block per
    /// channel; only a single symbol is ever needed by this driver, so the
    /// default is already conservative).
    /// @param trans_queue_depth is the RMT transmit queue depth, default is
    /// 1, since only one (looped) transmission is ever in flight per
    /// channel.
    Esp32RmtPwm(const std::initializer_list<uint8_t> &pins,
                uint8_t resolution_bits = 8, uint32_t freq_hz = 20000,
                rmt_clock_source_t clk_src = RMT_CLK_SRC_DEFAULT,
                size_t mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL,
                size_t trans_queue_depth = 1)
        : pins_(pins)
        , resolutionBits_(resolution_bits)
        , freqHz_(freq_hz)
        , clkSrc_(clk_src)
        , memBlockSymbols_(mem_block_symbols)
        , transQueueDepth_(trans_queue_depth)
    {
        init_helper();
    }

    /// Initializes the RMT peripheral channels and starts each output
    /// transmitting a 0% duty cycle (off).
    void hw_init()
    {
        for (size_t i = 0; i < pins_.size(); i++)
        {
            uint8_t pin = pins_[i];
            HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(pin));

            LOG(VERBOSE,
                "[Esp32RmtPwm] Configuring RMT channel on GPIO %d "
                "(resolution:%d, frequency:%" PRIu32 ")",
                pin, 1 << resolutionBits_, freqHz_);

            rmt_tx_channel_config_t config;
            memset(&config, 0, sizeof(rmt_tx_channel_config_t));
            config.gpio_num = static_cast<gpio_num_t>(pin);
            config.clk_src = clkSrc_;
            config.resolution_hz = freqHz_ * (1u << resolutionBits_);
            config.mem_block_symbols = memBlockSymbols_;
            config.trans_queue_depth = transQueueDepth_;
            config.intr_priority = 0;

            rmt_channel_handle_t channel = nullptr;
            ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &channel));

            rmt_copy_encoder_config_t encoder_config;
            memset(&encoder_config, 0, sizeof(rmt_copy_encoder_config_t));
            rmt_encoder_handle_t encoder = nullptr;
            ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &encoder));

            ESP_ERROR_CHECK(rmt_enable(channel));

            channels_[i].emplace(channel, encoder,
                static_cast<gpio_num_t>(pin), resolutionBits_, freqHz_,
                clkSrc_, memBlockSymbols_, transQueueDepth_);
        }
    }

    /// @return one PWM output.
    /// @param id is the output number, zero based for this @ref Esp32RmtPwm
    /// instance.
    PWM *get_channel(unsigned id)
    {
        HASSERT(id < channels_.size());
        return &*channels_[id];
    }

private:
    /// Initialization helper for construction.
    void init_helper()
    {
        HASSERT(pins_.size() > 0);
        // SOC_RMT_TX_CANDIDATES_PER_GROUP is the number of RMT channels
        // capable of transmit in this SoC's single RMT group (as opposed to
        // SOC_RMT_CHANNELS_PER_GROUP, which also counts RX-only channels).
        // This is considerably smaller than the number of LEDC channels
        // available on the same hardware, e.g. only 2 on the ESP32-C3.
        HASSERT(pins_.size() <= SOC_RMT_TX_CANDIDATES_PER_GROUP);
        HASSERT(resolutionBits_ >= 1 && resolutionBits_ <= 15);
        channels_.resize(pins_.size());
    }

    class Channel : public PWM
    {
    public:
        Channel(rmt_channel_handle_t channel, rmt_encoder_handle_t encoder,
                gpio_num_t pin, uint8_t resolution_bits, uint32_t freq_hz,
                rmt_clock_source_t clk_src, size_t mem_block_symbols,
                size_t trans_queue_depth)
            : channel_(channel)
            , encoder_(encoder)
            , pin_(pin)
            , resolutionBits_(resolution_bits)
            , periodTicks_(1u << resolution_bits)
            , dutyTicks_(0)
            , freqHz_(freq_hz)
            , clkSrc_(clk_src)
            , memBlockSymbols_(mem_block_symbols)
            , transQueueDepth_(trans_queue_depth)
        {
            apply_duty();
        }

        void set_period(uint32_t counts) override
        {
            HASSERT(counts > 0);

            LOG(VERBOSE,
                "[Esp32RmtPwm:%d] Reconfiguring frequency to %" PRIu32 " Hz",
                pin_, counts);

            ESP_ERROR_CHECK(rmt_disable(channel_));
            ESP_ERROR_CHECK(rmt_del_channel(channel_));

            rmt_tx_channel_config_t config;
            memset(&config, 0, sizeof(rmt_tx_channel_config_t));
            config.gpio_num = pin_;
            config.clk_src = clkSrc_;
            config.resolution_hz = counts * periodTicks_;
            config.mem_block_symbols = memBlockSymbols_;
            config.trans_queue_depth = transQueueDepth_;
            config.intr_priority = 0;

            channel_ = nullptr;
            ESP_ERROR_CHECK(rmt_new_tx_channel(&config, &channel_));
            ESP_ERROR_CHECK(rmt_enable(channel_));

            freqHz_ = counts;
            apply_duty();
        }

        uint32_t get_period() override
        {
            return freqHz_;
        }

        void set_duty(uint32_t counts) override
        {
            HASSERT(counts <= periodTicks_);
            dutyTicks_ = counts;
            apply_duty();
        }

        uint32_t get_duty() override
        {
            return dutyTicks_;
        }

        uint32_t get_period_max() override
        {
            return periodTicks_;
        }

        uint32_t get_period_min() override
        {
            return 0;
        }

    private:
        /// (Re)transmits the RMT symbol representing the current duty cycle
        /// as an infinite loop. Neither half of the symbol is ever given a
        /// literal zero duration, since the RMT hardware treats that as an
        /// end-of-transmission marker; a 0% or 100% duty cycle is instead
        /// represented as two consecutive halves at the same level.
        void apply_duty()
        {
            rmt_symbol_word_t symbol;
            if (dutyTicks_ == 0)
            {
                symbol.level0 = 0;
                symbol.duration0 = periodTicks_ - 1;
                symbol.level1 = 0;
                symbol.duration1 = 1;
            }
            else if (dutyTicks_ >= periodTicks_)
            {
                symbol.level0 = 1;
                symbol.duration0 = periodTicks_ - 1;
                symbol.level1 = 1;
                symbol.duration1 = 1;
            }
            else
            {
                symbol.level0 = 1;
                symbol.duration0 = dutyTicks_;
                symbol.level1 = 0;
                symbol.duration1 = periodTicks_ - dutyTicks_;
            }

            ESP_ERROR_CHECK(rmt_disable(channel_));
            ESP_ERROR_CHECK(rmt_enable(channel_));

            rmt_transmit_config_t tx_config;
            memset(&tx_config, 0, sizeof(rmt_transmit_config_t));
            tx_config.loop_count = -1;
            ESP_ERROR_CHECK(rmt_transmit(
                channel_, encoder_, &symbol, sizeof(symbol), &tx_config));
        }

        /// RMT TX channel handle for this output.
        rmt_channel_handle_t channel_;

        /// RMT copy encoder handle for this output.
        rmt_encoder_handle_t encoder_;

        /// GPIO pin driven by this output.
        gpio_num_t pin_;

        /// PWM duty cycle resolution in bits.
        uint8_t resolutionBits_;

        /// Number of RMT ticks in one PWM period, i.e. (1 << resolutionBits_).
        uint32_t periodTicks_;

        /// Current duty cycle, in ticks, range 0 to periodTicks_ inclusive.
        uint32_t dutyTicks_;

        /// Current PWM frequency, in Hz.
        uint32_t freqHz_;

        /// RMT peripheral clock source in use for this channel.
        rmt_clock_source_t clkSrc_;

        /// RMT hardware memory block symbols reserved for this channel.
        size_t memBlockSymbols_;

        /// RMT transmit queue depth for this channel.
        size_t transQueueDepth_;
    };

    /// Collection of GPIO pins in use by this @ref Esp32RmtPwm.
    std::vector<uint8_t> pins_;

    /// PWM duty cycle resolution in bits, common to all channels at
    /// construction time (subsequent per-channel frequency changes via
    /// @ref Channel::set_period do not change this).
    uint8_t resolutionBits_;

    /// Initial PWM frequency, in Hz, common to all channels at construction
    /// time.
    uint32_t freqHz_;

    /// RMT peripheral clock source, common to all channels.
    rmt_clock_source_t clkSrc_;

    /// RMT hardware memory block symbols reserved per channel.
    size_t memBlockSymbols_;

    /// RMT transmit queue depth per channel.
    size_t transQueueDepth_;

    /// @ref PWM instances connected to RMT channels.
    std::vector<uninitialized<Channel>> channels_;

    DISALLOW_COPY_AND_ASSIGN(Esp32RmtPwm);
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32RmtPwm;

#endif // _DRIVERS_ESP32RMTPWM_HXX_
