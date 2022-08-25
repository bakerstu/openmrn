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
 * \file Esp32WS2812.hxx
 *
 * WS2812 LED strip controller using the RMT peripheral to generate the LED
 * data stream.
 *
 * @author Mike Dunston
 * @date 3 May 2021
 */

#ifndef _DRIVERS_ESP32WS3218_HXX_
#define _DRIVERS_ESP32WS3218_HXX_

#include "os/Gpio.hxx"
#include "os/OS.hxx"

#include <driver/gpio.h>
#include <driver/rmt.h>

namespace openmrn_arduino
{

/// WS2812 driver via the ESP32 RMT peripheral.
///
/// Provides the ability to control one (or more) addressable WS2812/SK68XX
/// LEDs via a single GPIO pin. The number of output channels varies based on
/// the ESP32 variant in use.
///
/// Usable RMT channels for ESP32 variants:
/// ESP32 : RMT_CHANNEL_0 through RMT_CHANNEL_7.
/// ESP32-S2 : RMT_CHANNEL_0 through RMT_CHANNEL_3.
/// ESP32-S3 : RMT_CHANNEL_0 through RMT_CHANNEL_3.
/// ESP32-C3 : RMT_CHANNEL_0 and RMT_CHANNEL_1.
///
/// Additional RMT channels exist on ESP32-S3 and ESP32-C3 but they are
/// restricted to RX only. ESP32 and ESP32-S2 do not have restrictions on the
/// RMT channel usage.
///
/// The output data for the WS2812 is based on a series of 24 pulses matching
/// either of the two waves below:
///
/// WS2812 ZERO:
///    ----------
///    |  350   |
/// ---|  nsec  |     850     ---
///             |     nsec    |
///             ---------------
/// WS2812 ONE:
///    ---------------
///    |     800     |
/// ---|     nsec    |  450   ---
///                  |  nsec  |
///                  ----------
///
/// A third wave may be observed in the datastream of around 280 nanoseconds as
/// a LOW signal. This is a reset pulse and will reset the LEDs for new color
/// data. However, this pulse is not currently being generated and LEDs are
/// correctly displaying the expected colors. The reset pulse may be added in
/// the future.
///
/// LED Color data is transmitted in MSB format as: Green, Red, Blue.
///
/// The WS2812 LEDs use three primary colors, each with 256 levels of
/// brightness, allowing around 16M colors. In most cases values 64 and below
/// will yield the best color renditions.
///
/// For 1024 LEDs the total transmission time is around 30 milliseconds which is
/// around 30 updates to all LEDs per second.
///
/// When only one LED is connected (as seen on some DevKit-C boards) LED color
/// data will be transmitted inline with the call to set_led_color or clear.
/// When more than one LED is connected a background thread will be used to
/// transmit the updated LED color data.
///
/// This class will allocate 3 bytes per LED as a transmit buffer, it is not
/// double buffered at this time.
class Esp32WS2812 : private OSThread
{
public:
    /// Constructor.
    ///
    /// @param output_pin is the GPIO pin connected to the first LED in the
    /// strip.
    /// @param channel is the RMT channel to use for transmitting LED data.
    /// @param led_count is the number of LEDs in the strip.
    ///
    /// Note: when the number of LEDs is greater than one a background thread
    /// will be started as part of @ref hw_init. When only one LED is defined
    /// the @ref set_led_color and @ref clear methods will block until the LED
    /// data has been transfered to the RMT peripheral.
    Esp32WS2812(const gpio_num_t output_pin, const rmt_channel_t channel,
                const size_t led_count);

    /// Destructor.
    ~Esp32WS2812();

    /// Initializes the RMT peripheral.
    ///
    /// Note: this will start a background update thread if there is more than
    /// one LED configured.
    void hw_init();

    /// Sets an individual LED to the specified color.
    ///
    /// @param index is the zero based index of the LED in the strip.
    /// @param red is the red portion of the color to set the LED to.
    /// @param green is the green portion of the color to set the LED to.
    /// @param blue is the blue portion of the color to set the LED to.
    /// @param update_timeout_ms is how many milliseconds to wait for LED
    /// data transmit to complete before giving up, this is only applicable
    /// for single LED output configurations.
    void set_led_color(const size_t index, const uint8_t red,
                       const uint8_t green, const uint8_t blue,
                       const uint32_t update_timeout_ms = 100);

    /// Sets an individual LED to the specified color.
    ///
    /// @param index is the zero based index of the LED in the strip.
    /// @param red is the red portion of the color that the LED is set to.
    /// @param green is the green portion of the color that the LED is set to.
    /// @param blue is the blue portion of the color that the LED is set to.
    void get_led_color(const size_t index, uint8_t *red, uint8_t *green,
                       uint8_t *blue);

    /// Clears all LEDs to the specified color.
    ///
    /// @param red is the red portion of the color to set all LEDs to.
    /// @param green is the green portion of the color to set all LEDs to.
    /// @param blue is the blue portion of the color to set all LEDs to.
    /// @param update_timeout_ms is how many milliseconds to wait for LED
    /// data transmit to complete before giving up, this is only applicable
    /// for single LED output configurations.
    ///
    /// Note: the default clear to color is OFF.
    void clear(const uint8_t red = 0, const uint8_t green = 0,
               const uint8_t blue = 0, const uint32_t update_timeout_ms = 50);

    /// @return The maximum number of configured LEDs.
    size_t get_max_leds()
    {
        return ledCount_;
    }

private:
    /// Priority for the task performing the LED data transmission.
    /// Note: this will only be used when the number of LEDs is greater than 1.
    static constexpr UBaseType_t TASK_PRIORITY = 3;

    /// GPIO pin that is connected to the first LED in the strip.
    const gpio_num_t pin_;

    /// RMT channel being used to transmit pixel data.
    const rmt_channel_t channel_;

    /// Number of LEDs in the strip.
    const size_t ledCount_;

    /// Size of the LED data buffer.
    const size_t ledBufferSize_;

    /// Raw LED data stored in green, red, blue order.
    uint8_t *ledDataBuffer_;

    /// This is used to wakeup the background thread to start transmission of
    /// LED data.
    /// Note: this will only be used when the number of LEDs is greater than 1.
    OSSem sem_;

    /// User entry point for the created thread.
    /// @return exit status
    void *entry() override;
};

/// Implementation of @ref Gpio which routes the on/off calls to a single
/// WS2812/SK68xx LED.
class Esp32WS2812Gpio : public Gpio
{
public:
    /// Constructor.
    ///
    /// @param parent is the @ref Esp32WS2812 instance that this LED belongs to.
    /// @param index is the index in the strip.
    /// @param red_on is the red portion to set this LED to when it is ON.
    /// @param red_off is the red portion to set this LED to when it is OFF.
    /// @param green_on is the green portion to set this LED to when it is ON.
    /// @param green_off is the green portion to set this LED to when it is OFF.
    /// @param blue_on is the blue portion to set this LED to when it is ON.
    /// @param blue_off is the blue portion to set this LED to when it is OFF.
    Esp32WS2812Gpio(Esp32WS2812 *parent, const size_t index,
                    const uint8_t red_on, const uint8_t red_off,
                    const uint8_t green_on, const uint8_t green_off,
                    const uint8_t blue_on, const uint8_t blue_off)
                    : parent_(parent), index_(index),
                    redOn_(red_on), redOff_(red_off),
                    greenOn_(green_on), greenOff_(green_off),
                    blueOn_(blue_on), blueOff_(blue_off)
    {
        HASSERT(parent_ != NULL);
        HASSERT(index < parent_->get_max_leds());
    }

    /// Sets the output state of the connected GPIO pin.
    ///
    /// @param new_state State to set the GPIO pin to.
    void write(Value new_state) const override
    {
        if (new_state)
        {
            parent_->set_led_color(index_, redOn_, greenOn_, blueOn_);
        }
        else
        {
            parent_->set_led_color(index_, redOff_, greenOff_, blueOff_);
        }
    }

    /// Reads the current state of the connected GPIO pin.
    Value read() const override
    {
        uint8_t red = 0, green = 0, blue = 0;
        parent_->get_led_color(index_, &red, &green, &blue);
        if (red == redOn_ && green == greenOn_ && blue == blueOn_)
        {
            return Value::SET;
        }
        return Value::CLR;
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
        // no-op
    }

    /// Gets the GPIO direction.
    /// @return @ref DINPUT or @ref DOUTPUT
    Direction direction() const override
    {
        return Direction::DOUTPUT;
    }

    /// Inverts the current state of the LED color.
    void toggle()
    {
        write((Value)!read());
    }
private:
    Esp32WS2812 *parent_;
    const size_t index_;
    const uint8_t redOn_;
    const uint8_t redOff_;
    const uint8_t greenOn_;
    const uint8_t greenOff_;
    const uint8_t blueOn_;
    const uint8_t blueOff_;
};

} // namespace openmrn_arduino

using openmrn_arduino::Esp32WS2812;
using openmrn_arduino::Esp32WS2812Gpio;

#endif // _DRIVERS_ESP32WS3218_HXX_