/** @copyright
 * Copyright (c) 2023, Stuart W Baker
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
 * @file BitBangI2C.hxx
 * This file implements a bit bang implementation of I2C.
 *
 * @author Stuart W. Baker
 * @date 28 March 2023
 */

#ifndef _FREERTOS_DRIVERS_COMMON_BITBANG_I2C_HXX_
#define _FREERTOS_DRIVERS_COMMON_BITBANG_I2C_HXX_

#include "I2C.hxx"
#include "os/Gpio.hxx"
#include "os/OS.hxx"

/// Implement a bit-banged I2C master. A periodic timer [interrupt] should call
/// the BitBangI2C::tick_interrupt() method at a rate that is one half the
/// desired clock rate. For example, for a 100kHz bus, call once every 5
/// microseconds. The tick should be enabled to start. The driver will
/// enable/disable the tick as needed to save on spurious interrupts.
class BitBangI2C : public I2C
{
public:
    /// Constructor.
    /// @param name name of this device instance in the file system
    /// @param sda_gpio Gpio instance of the SDA line
    /// @param scl_gpio Gpio instance of the SCL line
    /// @param enable_tick callback to enable ticks
    /// @param disable_tick callback to disable ticks
    BitBangI2C(const char *name, const Gpio *sda_gpio, const Gpio *scl_gpio,
               void (*enable_tick)(), void (*disable_tick)())
        : I2C(name)
        , sda_(sda_gpio)
        , scl_(scl_gpio)
        , enableTick_(enable_tick)
        , disableTick_(disable_tick)
        , msg_(nullptr)
        , sem_()
        , state_(State::STOP) // start-up with a stop sequence
        , stateStop_(StateStop::SDA_CLR)
        , count_(0)
        , stop_(true)
    {
        gpio_set(sda_);
        gpio_clr(scl_);
    }

    /// Destructor.
    ~BitBangI2C()
    {
    }

    /// Called at a periodic tick, when enabled.
    void tick_interrupt();

private:
    /// High level I2C States
    enum class State
    {
        START, ///< start state
        ADDRESS, ///< address state
        DATA_TX, ///< data TX state
        DATA_RX, ///< data RX state
        STOP, ///< stop state
    };

    /// Low level I2C start states
    enum class StateStart
    {
        SDA_SET, ///< start sequence
        SCL_SET, ///< start sequence
        SDA_CLR, ///< start sequence
        SCL_CLR, ///< start sequence
        FIRST = SDA_SET, ///< first start sequence state
        LAST = SCL_CLR, /// last start sequence state
    };

    /// Low level I2C stop states
    enum class StateStop
    {
        SDA_CLR, ///< stop sequence
        SCL_SET, ///< stop sequence
        SDA_SET, ///< stop sequence
        FIRST = SDA_CLR, ///< first stop sequence state
        LAST = SDA_SET, ///< last stop sequence state
    };

    /// Low level I2C data TX states
    enum class StateTx
    {
        DATA_7_SCL_SET, ///< data TX sequence
        DATA_7_SCL_CLR, ///< data TX sequence
        DATA_6_SCL_SET, ///< data TX sequence
        DATA_6_SCL_CLR, ///< data TX sequence
        DATA_5_SCL_SET, ///< data TX sequence
        DATA_5_SCL_CLR, ///< data TX sequence
        DATA_4_SCL_SET, ///< data TX sequence
        DATA_4_SCL_CLR, ///< data TX sequence
        DATA_3_SCL_SET, ///< data TX sequence
        DATA_3_SCL_CLR, ///< data TX sequence
        DATA_2_SCL_SET, ///< data TX sequence
        DATA_2_SCL_CLR, ///< data TX sequence
        DATA_1_SCL_SET, ///< data TX sequence
        DATA_1_SCL_CLR, ///< data TX sequence
        DATA_0_SCL_SET, ///< data TX sequence
        DATA_0_SCL_CLR, ///< data TX sequence
        ACK_SDA_SCL_SET, ///< data TX sequence
        ACK_SCL_CLR, ///< data TX sequence
        FIRST = DATA_7_SCL_SET, ///< first data TX sequence state
        LAST = ACK_SCL_CLR, ///< last data TX sequence state
    };

    /// Low level I2C data RX states
    enum class StateRx
    {
        DATA_7_SCL_SET, ///< data RX sequence
        DATA_7_SCL_CLR, ///< data RX sequence
        DATA_6_SCL_SET, ///< data RX sequence
        DATA_6_SCL_CLR, ///< data RX sequence
        DATA_5_SCL_SET, ///< data RX sequence
        DATA_5_SCL_CLR, ///< data RX sequence
        DATA_4_SCL_SET, ///< data RX sequence
        DATA_4_SCL_CLR, ///< data RX sequence
        DATA_3_SCL_SET, ///< data RX sequence
        DATA_3_SCL_CLR, ///< data RX sequence
        DATA_2_SCL_SET, ///< data RX sequence
        DATA_2_SCL_CLR, ///< data RX sequence
        DATA_1_SCL_SET, ///< data RX sequence
        DATA_1_SCL_CLR, ///< data RX sequence
        DATA_0_SCL_SET, ///< data RX sequence
        DATA_0_SCL_CLR, ///< data RX sequence
        ACK_SDA_SCL_SET, ///< data RX sequence
        ACK_SCL_CLR, ///< data RX sequence
        FIRST = DATA_7_SCL_SET, ///< first data RX sequence state
        LAST = ACK_SCL_CLR, ///< last data RX sequence state
    };

    /// Allow pre-increment definition
    friend StateStart &operator++(StateStart &);

    /// Allow pre-increment definition
    friend StateStop &operator++(StateStop &);

    /// Allow pre-increment definition
    friend StateTx &operator++(StateTx &);

    /// Allow pre-increment definition
    friend StateRx &operator++(StateRx &);

    /// Execute start state machine.
    /// @return true if the sub-state machine is finished.
    bool state_start();

    /// Execute stop state machine.
    /// @return true if the sub-state machine is finished.
    bool state_stop();

    /// Execute data TX state machine.
    /// @param data value to send
    /// @return true if the sub-state machine is finished, count_ may be
    ///         negative to indicate an error.
    bool state_tx(uint8_t data);

    /// Execute data RX state machine.
    /// @param location to shift data into
    /// @param nack send a NACK in the (N)ACK slot
    /// @return true if the sub-state machine is finished.
    bool state_rx(uint8_t *data, bool nack);

    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

    /// Method to transmit/receive the data.
    /// @param msg message to transact.
    /// @param stop produce a stop condition at the end of the transfer
    /// @return bytes transfered upon success or -1 with errno set
    int transfer(struct i2c_msg *msg, bool stop) override;

    /// Set the GPIO state.
    /// @param gpio GPIO to set
    void gpio_set(const Gpio *gpio)
    {
        gpio->set_direction(Gpio::Direction::DINPUT);
    }

    /// Clear the GPIO state.
    /// @param gpio GPIO to clear
    void gpio_clr(const Gpio *gpio)
    {
        gpio->clr();
        gpio->set_direction(Gpio::Direction::DOUTPUT);
        gpio->clr();
    }

    const Gpio *sda_; ///< GPIO for the SDA line
    const Gpio *scl_; ///< GPIO for the SCL line
    void (*enableTick_)(void); ///< Enable the timer tick
    void (*disableTick_)(void); ///< Disable the timer tick
    struct i2c_msg *msg_; ///< I2C message to presently act upon  
    OSSem sem_; ///< semaphore to wakeup task level from ISR
    State state_; ///< state machine state
    union
    {
        StateStart stateStart_; ///< I2C start state machine state
        StateStop stateStop_; ///< I2C stop state machine state
        StateTx stateTx_; ///< I2C data TX state machine state
        StateRx stateRx_; ///< I2C data RX state machine state
    };
    int count_; ///< the count of data bytes transferred, error if < 0
    bool stop_; ///< if true, issue stop condition at the end of the message
};

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2C::StateStart &operator++(BitBangI2C::StateStart &x)
{
    if (x >= BitBangI2C::StateStart::FIRST && x <= BitBangI2C::StateStart::LAST)
    {
        x = static_cast<BitBangI2C::StateStart>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2C::StateStop &operator++(BitBangI2C::StateStop &x)
{
    if (x >= BitBangI2C::StateStop::FIRST && x <= BitBangI2C::StateStop::LAST)
    {
        x = static_cast<BitBangI2C::StateStop>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2C::StateTx &operator++(BitBangI2C::StateTx &x)
{
    if (x >= BitBangI2C::StateTx::FIRST && x <= BitBangI2C::StateTx::LAST)
    {
        x = static_cast<BitBangI2C::StateTx>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2C::StateRx &operator++(BitBangI2C::StateRx &x)
{
    if (x >= BitBangI2C::StateRx::FIRST && x <= BitBangI2C::StateRx::LAST)
    {
        x = static_cast<BitBangI2C::StateRx>(static_cast<int>(x) + 1);
    }
    return x;
}


#endif // _FREERTOS_DRIVERS_COMMON_BITBANG_I2C_HXX_
