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
 * \file Esp32WS2812.cxx
 *
 * WS2812 LED strip controller using the RMT peripheral to generate the LED
 * data stream.
 *
 * @author Mike Dunston
 * @date 3 May 2021
 */

// Ensure we only compile this code for the ESP32
#ifdef ESP32

#include "freertos_drivers/esp32/Esp32WS2812.hxx"
#include "utils/logging.h"

namespace openmrn_arduino
{

static uint32_t timeZeroHighTicks = 0;
static uint32_t timeZeroLowTicks = 0;
static uint32_t timeOneHighTicks = 0;
static uint32_t timeOneLowTicks = 0;

/// Number of microseconds to transmit a HIGH signal for a ZERO bit.
static constexpr uint16_t TIME_ZERO_HIGH_NSEC = 350;

/// Number of microseconds to transmit a LOW signal for a ZERO bit.
static constexpr uint16_t TIME_ZERO_LOW_NSEC = 1000;

/// Number of microseconds to transmit a HIGH signal for a ONE bit.
static constexpr uint16_t TIME_ONE_HIGH_NSEC = 1000;

/// Number of microseconds to transmit a LOW signal for a ONE bit.
static constexpr uint16_t TIME_ONE_LOW_NSEC = 350;

/// WS2812 to RMT data translation adapter for the ESP32 RMT peripheral.
///
/// @param src is the source data to be translated.
/// @param dest is the RMT data buffer to translate into.
/// @param src_size is the number of bytes in the source buffer.
/// @param wanted_num is the number of bits to translate.
/// @param translated_size is the number of bytes translated by this method.
/// @param item_num is the number of RMT items translated by this method.
///
/// Note: this method is called from an ISR context and all data access must be
/// from DRAM, be a constant and be 32 bit aligned.
static void IRAM_ATTR ws2812_rmt_adapter(const void *src, rmt_item32_t *dest,
    size_t src_size, size_t wanted_num, size_t *translated_size,
    size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    rmt_item32_t *pdest = dest;
    const rmt_item32_t bit0 = {{{ timeZeroHighTicks, 1, timeZeroLowTicks, 0 }}};
    const rmt_item32_t bit1 = {{{ timeOneHighTicks, 1, timeOneLowTicks, 0 }}};
    while (size < src_size && num < wanted_num)
    {
        LOG(VERBOSE, "ESP-WS2812: %zu:%02x", size, *psrc);
        for (int i = 7; i >= 0; i--)
        {
            // bytes are sent in MSB order
            if (*psrc & (1 << i))
            {
                LOG(VERBOSE, "ESP-WS2812: %zu:%d 1", size, i);
                pdest->val =  bit1.val;
            }
            else
            {
                LOG(VERBOSE, "ESP-WS2812: %zu:%d 0", size, i);
                pdest->val =  bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }

    *translated_size = size;
    *item_num = num;
}

// constructor
Esp32WS2812::Esp32WS2812(const gpio_num_t output_pin,
                         const rmt_channel_t channel,
                         const size_t led_count)
                       : OSThread(),
                         pin_(output_pin),
                         channel_(channel),
                         ledCount_(led_count),
                         ledBufferSize_(led_count * 3),
                         sem_(0)
{
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    // ESP32-C3 has four RMT channels with the first two allocated for TX and
    // second two for RX.
    HASSERT(channel >= RMT_CHANNEL_0 && channel <= RMT_CHANNEL_1);
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
    // ESP32-S2 has four RMT channels.
    HASSERT(channel >= RMT_CHANNEL_0 && channel <= RMT_CHANNEL_3);
#elif defined(CONFIG_IDF_TARGET_ESP32)
    // ESP32 has eight RMT channels.
    HASSERT(channel >= RMT_CHANNEL_0 && channel <= RMT_CHANNEL_7);
#endif
    HASSERT(GPIO_IS_VALID_OUTPUT_GPIO(output_pin));
}

// destructor
Esp32WS2812::~Esp32WS2812()
{
    free(ledDataBuffer_);
}

// initialize the perhipheral
void Esp32WS2812::hw_init()
{
    LOG(VERBOSE, "ESP-WS2812(%d): Allocating %u bytes for %zu LEDs", channel_,
        ledBufferSize_, ledCount_);
    // Allocate the LED color data buffer
    ledDataBuffer_ = (uint8_t *)calloc(1, ledBufferSize_);
    HASSERT(ledDataBuffer_ != NULL);

    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(pin_, channel_);
    // reconfigure the clock to 40Mhz
    config.clk_div = 2;

    LOG(VERBOSE, "ESP-WS2812(%d): Configuring RMT using GPIO %d", channel_,
        pin_);
    ESP_ERROR_CHECK(rmt_config(&config));

    LOG(VERBOSE, "ESP-WS2812(%d): Installing RMT driver", channel_);
    ESP_ERROR_CHECK(rmt_driver_install(channel_, 0, 0));

    LOG(VERBOSE, "ESP-WS2812(%d): Installing RMT translator", channel_);
    ESP_ERROR_CHECK(rmt_translator_init(channel_, ws2812_rmt_adapter));

    if (timeZeroHighTicks == 0 || timeZeroLowTicks == 0 ||
        timeOneHighTicks == 0 || timeOneLowTicks == 0)
    {
        // convert the number of microseconds for high/low signals to the
        // number of RMT ticks required.
        uint32_t counter_clk_hz = 0;
        ESP_ERROR_CHECK(rmt_get_counter_clock(channel_, &counter_clk_hz));
        float ratio = (float)counter_clk_hz / 1e9;
        timeZeroHighTicks = (uint32_t)(ratio * TIME_ZERO_HIGH_NSEC);
        timeZeroLowTicks = (uint32_t)(ratio * TIME_ZERO_LOW_NSEC);
        timeOneHighTicks = (uint32_t)(ratio * TIME_ONE_HIGH_NSEC);
        timeOneLowTicks = (uint32_t)(ratio * TIME_ONE_LOW_NSEC);
        LOG(VERBOSE,
            "ESP-WS2812: ratio: %6.4f, T0H:%d, T0L:%d, T1H:%d, T1L:%d",
            ratio, timeZeroHighTicks, timeZeroLowTicks, timeOneHighTicks,
            timeOneLowTicks);
    }

    if (ledCount_ > 1)
    {
        LOG(VERBOSE, "ESP-WS2812(%d): Starting background update thread",
            channel_);
        // start the background update thread with the name as: rmt.{channel}
        char name[10];
        strcpy(name, "rmt.");
        name[4] = '0' + channel_;
        name[5] = '\0';
        start(name, TASK_PRIORITY, 2048);
    }
    clear();
}

void Esp32WS2812::set_led_color(const size_t index, const uint8_t red,
                                const uint8_t green, const uint8_t blue,
                                bool update, const uint32_t timeout_ms)
{
    HASSERT(index < ledCount_);

    // Check if we are updating the LED color, if not then we can skip the
    // update.
    if (ledDataBuffer_[(index * 3) + 0] != green ||
        ledDataBuffer_[(index * 3) + 1] != red ||
        ledDataBuffer_[(index * 3) + 2] != blue)
    {
        LOG(VERBOSE,
            "ESP-WS2812(%d): Setting LED(%zu) to (r:%d,g:%d,b:%d)", channel_,
            index, red, green, blue);
        // TODO: Add conversion for color orders other than GRB
        ledDataBuffer_[(index * 3) + 0] = green & 0xFF;
        ledDataBuffer_[(index * 3) + 1] = red & 0xFF;
        ledDataBuffer_[(index * 3) + 2] = blue & 0xFF;
        if (ledCount_ > 1)
        {
            sem_.post();
        }
        else if (update)
        {
            LOG(VERBOSE, "ESP-WS2812(%d): Updating LEDs", channel_);
            ESP_ERROR_CHECK(
                rmt_write_sample(
                    channel_, ledDataBuffer_, ledBufferSize_, true));
            ESP_ERROR_CHECK(
                rmt_wait_tx_done(channel_, pdMS_TO_TICKS(timeout_ms)));
            LOG(VERBOSE, "ESP-WS2812(%d): LEDs updated", channel_);
        }
    }
}

void Esp32WS2812::get_led_color(const size_t index, uint8_t *red,
                                uint8_t *green, uint8_t *blue)
{
    HASSERT(index < ledCount_);

    // TODO: Add conversion for color orders other than GRB
    *green = ledDataBuffer_[(index * 3) + 0];
    *red   = ledDataBuffer_[(index * 3) + 1];
    *blue  = ledDataBuffer_[(index * 3) + 2];
}

void Esp32WS2812::clear(const uint8_t red, const uint8_t green,
                        const uint8_t blue, const uint32_t timeout_ms)
{
    // if the color values are all zero we can set the buffer data directly.
    if (red == 0 && green == 0 && blue == 0)
    {
        LOG(VERBOSE, "ESP-WS2812(%d): Turning off ALL LEDs", channel_);
        memset(ledDataBuffer_, 0, ledBufferSize_);
    }
    else
    {
        LOG(VERBOSE,
            "ESP-WS2812(%d): Clearing all LEDs to (r:%d,g:%d,b:%d)", channel_,
            red, green, blue);
        // iterate through the LEDs and set each individually.
        for(size_t index = 0; index < ledCount_; index++)
        {
            // TODO: Add conversion for color orders other than GRB
            ledDataBuffer_[(index * 3) + 0] = green  & 0xFF;
            ledDataBuffer_[(index * 3) + 1] = red & 0xFF;
            ledDataBuffer_[(index * 3) + 2] = blue & 0xFF;
        }
    }
    // if we have more than one LED notify the background update task.
    if (ledCount_ > 1)
    {
        sem_.post();
    }
    else
    {
        LOG(VERBOSE, "ESP-WS2812(%d): Updating LEDs", channel_);
        ESP_ERROR_CHECK(
            rmt_write_sample(channel_, ledDataBuffer_, ledBufferSize_, true));
        ESP_ERROR_CHECK(rmt_wait_tx_done(channel_, pdMS_TO_TICKS(timeout_ms)));
        LOG(VERBOSE, "ESP-WS2812(%d): LEDs updated", channel_);
    }
}

void *Esp32WS2812::entry()
{
    LOG(VERBOSE, "ESP-WS2812(%d): Update thread starting", channel_);
    for ( ; /* forever */ ; )
    {
        LOG(VERBOSE, "ESP-WS2812(%d): Waiting for update request", channel_);
        sem_.wait();
        ESP_ERROR_CHECK(
            rmt_write_sample(channel_, ledDataBuffer_, ledBufferSize_, true));
        ESP_ERROR_CHECK(rmt_wait_tx_done(channel_, portMAX_DELAY));
        LOG(VERBOSE, "ESP-WS2812(%d): LEDs updated", channel_);
    }

    return NULL;
}

} // namespace openmrn_arduino

#endif // ESP32