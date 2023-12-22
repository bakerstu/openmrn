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

/// Consolidate all the state machine enumerations for easy operator
/// overloading.
class BitBangI2CStates
{
protected:
    /// Default constructor.
    BitBangI2CStates()
    {
    }

    /// High level I2C States
    enum class State : uint8_t
    {
        START, ///< start state
        ADDRESS, ///< address state
        DATA_TX, ///< data TX state
        DATA_RX, ///< data RX state
        RECOVERY, ///< recovery state
        STOP, ///< stop state
    };

    /// Low level I2C start states
    enum class StateStart : uint8_t
    {
        SDA_SET, ///< start sequence
        SCL_SET, ///< start sequence
        SDA_CLR, ///< start sequence
        FIRST = SDA_SET, ///< first start sequence state
        LAST = SDA_CLR, /// last start sequence state
    };

    /// Low level I2C stop states
    enum class StateStop : uint8_t
    {
        SDA_CLR, ///< stop sequence
        SCL_SET, ///< stop sequence
        SDA_SET, ///< stop sequence
        FIRST = SDA_CLR, ///< first stop sequence state
        LAST = SDA_SET, ///< last stop sequence state
    };

    /// Low level I2C data TX states
    enum class StateTx : uint8_t
    {
        DATA_7_SCL_CLR, ///< data TX sequence
        DATA_7_SCL_SET, ///< data TX sequence
        DATA_6_SCL_CLR, ///< data TX sequence
        DATA_6_SCL_SET, ///< data TX sequence
        DATA_5_SCL_CLR, ///< data TX sequence
        DATA_5_SCL_SET, ///< data TX sequence
        DATA_4_SCL_CLR, ///< data TX sequence
        DATA_4_SCL_SET, ///< data TX sequence
        DATA_3_SCL_CLR, ///< data TX sequence
        DATA_3_SCL_SET, ///< data TX sequence
        DATA_2_SCL_CLR, ///< data TX sequence
        DATA_2_SCL_SET, ///< data TX sequence
        DATA_1_SCL_CLR, ///< data TX sequence
        DATA_1_SCL_SET, ///< data TX sequence
        DATA_0_SCL_CLR, ///< data TX sequence
        DATA_0_SCL_SET, ///< data TX sequence
        ACK_SDA_SET_SCL_CLR, ///< data TX sequence
        ACK_SCL_SET, ///< data TX sequence
        ACK_SCL_CLR, ///< data TX sequence
        FIRST = DATA_7_SCL_CLR, ///< first data TX sequence state
        LAST = ACK_SCL_CLR, ///< last data TX sequence state
    };

    /// Low level I2C data RX states
    enum class StateRx : uint8_t
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

    /// Low level I2C data RX states
    enum class StateRecovery : uint8_t
    {
        SDA_SET,        ///< recovery sequence, start with SDA high
        DATA_7_SCL_SET, ///< recovery sequence
        DATA_7_SCL_CLR, ///< recovery sequence
        DATA_6_SCL_SET, ///< recovery sequence
        DATA_6_SCL_CLR, ///< recovery sequence
        DATA_5_SCL_SET, ///< recovery sequence
        DATA_5_SCL_CLR, ///< recovery sequence
        DATA_4_SCL_SET, ///< recovery sequence
        DATA_4_SCL_CLR, ///< recovery sequence
        DATA_3_SCL_SET, ///< recovery sequence
        DATA_3_SCL_CLR, ///< recovery sequence
        DATA_2_SCL_SET, ///< recovery sequence
        DATA_2_SCL_CLR, ///< recovery sequence
        DATA_1_SCL_SET, ///< recovery sequence
        DATA_1_SCL_CLR, ///< recovery sequence
        DATA_0_SCL_SET, ///< recovery sequence
        DATA_0_SCL_CLR, ///< recovery sequence
        ACK_SCL_SET, ///< recovery sequence
        ACK_SCL_CLR, ///< recovery sequence
        FIRST = SDA_SET, ///< first recovery sequence state
        LAST = ACK_SCL_CLR, ///< last recovery sequence state
    };

    /// Allow pre-increment definition
    friend StateStart &operator++(StateStart &);

    /// Allow pre-increment definition
    friend StateStop &operator++(StateStop &);

    /// Allow pre-increment definition
    friend StateTx &operator++(StateTx &);

    /// Allow pre-increment definition
    friend StateRx &operator++(StateRx &);

    /// Allow pre-increment definition
    friend StateRecovery &operator++(StateRecovery &);
};

/// Implement a bit-banged I2C master. A periodic timer [interrupt] should call
/// the BitBangI2C::tick_interrupt() method at a rate that is one half the
/// desired clock rate. For example, for a 100kHz bus, call once every 5
/// microseconds. The tick should be enabled to start. The driver will call
/// HW::tick_enable()/HW::tick_disable() to enable/disable the tick as needed
/// to save on spurious interrupts.
///
/// Example:
/// @code
/// struct I2CHwDefs
/// {
///     class SCL_Pin
///     {
///     public:
///         static void set(bool on)
///         {
///             ...
///         }
///         static bool get()
///         {
///             ...
///         }
///         static void hw_init()
///         {
///             ...
///         }
///     };
///
///     class SDA_Pin
///     {
///     public:
///         static void set(bool on)
///         {
///             ...
///         }
///         static bool get()
///         {
///             ...
///         }
///         static void hw_init()
///         {
///             ...
///         }
///     };
///
///     static void tick_enable()
///     {
///         ...
///     }
///
///     static void tick_disable()
///     {
///         ...
///     }
/// };
///
/// BitBangI2C<I2CHwDefs> bitBangI2C("/dev/i2c0", disable_tick, enable_tick);
/// @endcode
///
/// @tparam HW hardware interface to the access the SCL and SDA I/O lines
template <class HW> class BitBangI2C : protected BitBangI2CStates
                                     , public I2C
                                     , private Atomic
{
public:
    /// Constructor.
    /// @param name name of this device instance in the file system
    BitBangI2C(const char *name)
        : I2C(name)
        , msg_(nullptr)
        , sem_()
        , count_(0)
        , state_(State::STOP) // start-up with a stop sequence
        , stateStop_(StateStop::SDA_CLR)
        , stop_(true)
        , clockStretchActive_(false)
    {
        HW::SDA_Pin::set(1);
        HW::SCL_Pin::set(0);
    }

    /// Destructor.
    ~BitBangI2C()
    {
    }

    /// Called at a periodic tick, when enabled.
    inline void tick_interrupt();

private:
    /// Execute state machine for sending start condition.
    /// @return true if the sub-state machine is finished, after doing the work
    ///         required by the current tick.
    inline bool state_start();

    /// Execute state machine for sending the stop condition.
    /// @return true if the sub-state machine is finished, after doing the work
    ///         required by the current tick.
    inline bool state_stop();

    /// Execute data TX state machine.
    /// @param data value to send
    /// @return true if the sub-state machine is finished, after doing the work
    ///         required by the current tick. count_ may be negative to
    ///         indicate an error.
    inline bool state_tx(uint8_t data);

    /// Execute data RX state machine.
    /// @param location to shift data into
    /// @param nack send a NACK in the (N)ACK slot
    /// @return true if the sub-state machine is finished, after doing the work
    ///         required by the current tick.
    inline bool state_rx(uint8_t *data, bool nack);

    /// Execute recovery state machine.
    /// @return true if the sub-state machine is finished, after doing the work
    ///         required by the current tick.
    inline bool state_recovery();

    void enable() override {} /**< function to enable device */
    void disable() override {} /**< function to disable device */

    /// Method to transmit/receive the data.
    /// @param msg message to transact.
    /// @param stop produce a stop condition at the end of the transfer
    /// @return bytes transfered upon success or -1 with errno set
    inline int transfer(struct i2c_msg *msg, bool stop) override;

    struct i2c_msg *msg_; ///< I2C message to presently act upon  
    OSSem sem_; ///< semaphore to wakeup task level from ISR
    int count_; ///< the count of data bytes transferred, error if < 0
    State state_; ///< state machine state
    union
    {
        StateStart stateStart_; ///< I2C start state machine state
        StateStop stateStop_; ///< I2C stop state machine state
        StateTx stateTx_; ///< I2C data TX state machine state
        StateRx stateRx_; ///< I2C data RX state machine state
        StateRecovery stateRecovery_; ///< I2C recovery state machine state
    };
    bool stop_; ///< if true, issue stop condition at the end of the message
    bool clockStretchActive_; ///< true if the slave is clock stretching.
};

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2CStates::StateStart &operator++(BitBangI2CStates::StateStart &x)
{
    if (x >= BitBangI2CStates::StateStart::FIRST &&
        x < BitBangI2CStates::StateStart::LAST)
    {
        x = static_cast<BitBangI2CStates::StateStart>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2CStates::StateStop &operator++(BitBangI2CStates::StateStop &x)
{
    if (x >= BitBangI2CStates::StateStop::FIRST &&
        x < BitBangI2CStates::StateStop::LAST)
    {
        x = static_cast<BitBangI2CStates::StateStop>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2CStates::StateTx &operator++(BitBangI2CStates::StateTx &x)
{
    if (x >= BitBangI2CStates::StateTx::FIRST &&
        x < BitBangI2CStates::StateTx::LAST)
    {
        x = static_cast<BitBangI2CStates::StateTx>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2CStates::StateRx &operator++(BitBangI2CStates::StateRx &x)
{
    if (x >= BitBangI2CStates::StateRx::FIRST &&
        x < BitBangI2CStates::StateRx::LAST)
    {
        x = static_cast<BitBangI2CStates::StateRx>(static_cast<int>(x) + 1);
    }
    return x;
}

/// Pre-increment operator overload
/// @param x starting value
/// @return incremented value
inline BitBangI2CStates::StateRecovery &operator++(
    BitBangI2CStates::StateRecovery &x)
{
    if (x >= BitBangI2CStates::StateRecovery::FIRST &&
        x < BitBangI2CStates::StateRecovery::LAST)
    {
        x = static_cast<BitBangI2CStates::StateRecovery>(
            static_cast<int>(x) + 1);
    }
    return x;
}

//
// BitBangI2C::tick_interrupt()
//
template <class HW> __attribute__((optimize("-O3")))
inline void BitBangI2C<HW>::tick_interrupt()
{
    bool exit = false;
    switch (state_)
    {
        // start sequence
        case State::START:
            if (state_start())
            {
                // start done, send the address
                state_ = State::ADDRESS;
                stateTx_ = StateTx::FIRST;
            }
            break;
        case State::ADDRESS:
        {
            /// @todo only supporting 7-bit I2C addresses at the moment.
            uint8_t address = msg_->addr << 1;
            if (msg_->flags & I2C_M_RD)
            {
                address |= 0x1;
            }
            if (state_tx(address))
            {
                if (count_ < 0)
                {
                    // Some error occured, likely an unexpected NACK. Send a
                    // stop in order to shutdown gracefully.
                    state_ = State::STOP;
                    stateStop_ = StateStop::FIRST;
                }
                else
                {
                    if (msg_->flags & I2C_M_RD)
                    {
                        state_ = State::DATA_RX;
                        stateRx_ = StateRx::FIRST;
                    }
                    else
                    {
                        state_ = State::DATA_TX;
                        stateTx_ = StateTx::FIRST;
                    }
                }
            }
            break;
        }
        case State::DATA_TX:
            if (state_tx(msg_->buf[count_]))
            {
                if (count_ < 0)
                {
                    // Some error occured, likely an unexpected NACK. Send a
                    // stop in order to shutdown gracefully.
                    state_ = State::STOP;
                    stateStop_ = StateStop::FIRST;
                }
                else if (++count_ >= msg_->len)
                {
                    // All of the data has been transmitted.
                    if (stop_)
                    {
                        state_ = State::STOP;
                        stateStop_ = StateStop::FIRST;
                    }
                    else
                    {
                        exit = true;
                    }
                }
                else
                {
                    // Setup the next byte TX
                    stateTx_ = StateTx::FIRST;
                }
            }
            break;
        case State::DATA_RX:
            if (state_rx(msg_->buf + count_, (count_ + 1 >= msg_->len)))
            {
                if (count_ < 0)
                {
                    // Some error occured, likely an unexpected NACK. Send a
                    // stop in order to shutdown gracefully.
                    state_ = State::STOP;
                    stateStop_ = StateStop::FIRST;
                }
                else if (++count_ >= msg_->len)
                {
                    // All of the data has been received.
                    if (stop_)
                    {
                        state_ = State::STOP;
                        stateStop_ = StateStop::FIRST;
                    }
                    else
                    {
                        exit = true;
                    }
                }
                else
                {
                    // Setup the next byte RX
                    stateRx_ = StateRx::FIRST;
                }
            }
            break;
        case State::RECOVERY:
            if (state_recovery())
            {
                // Recovery always concludes with a stop condition
                state_ = State::STOP;
                stateStop_ = StateStop::FIRST;
            }
            break;
        case State::STOP:
            exit = state_stop();
            break;
    }

    if (exit)
    {
        HW::tick_disable();
        int woken = 0;
        sem_.post_from_isr(&woken);
        os_isr_exit_yield_test(woken);
    }
}

//
// BitBangI2C::state_start()
//
template <class HW> __attribute__((optimize("-O3")))
inline bool BitBangI2C<HW>::state_start()
{
    switch (stateStart_)
    {
        // start sequence
        case StateStart::SDA_SET:
            HW::SDA_Pin::set(1);
            ++stateStart_;
            break;
        case StateStart::SCL_SET:
            HW::SCL_Pin::set(1);
            ++stateStart_;
            break;
        case StateStart::SDA_CLR:
            HW::SDA_Pin::set(0);
            ++stateStart_;
            return true;
    }
    return false;
}

//
// BitBangI2C::state_stop()
//
template <class HW> __attribute__((optimize("-O3")))
inline bool BitBangI2C<HW>::state_stop()
{
    switch (stateStop_)
    {
        case StateStop::SDA_CLR:
            HW::SDA_Pin::set(0);
            ++stateStop_;
            break;
        case StateStop::SCL_SET:
            HW::SCL_Pin::set(1);
            ++stateStop_;
            break;
        case StateStop::SDA_SET:
            HW::SDA_Pin::set(1);
            stop_ = false;
            return true; // exit
    }
    return false;
}

//
// BitBangI2C::state_tx()
//
template <class HW> __attribute__((optimize("-O3")))
inline bool BitBangI2C<HW>::state_tx(uint8_t data)
{
    // I2C is specified such that the data on the SDA line must be stable
    // during the high period of the clock, and the data line can only change
    // when SCL is Low.
    //
    // This means that the sequence always has to be
    // 1a. SCL := low
    // 1b. set/clear SDA
    // 2. wait a cycle for lines to settle
    // 3a. SCL := high
    // 3b. this edge is when the receiver samples
    // 4. wait a cycle, lines are stable
    switch(stateTx_)
    {
        case StateTx::DATA_7_SCL_CLR:
        case StateTx::DATA_6_SCL_CLR:
        case StateTx::DATA_5_SCL_CLR:
        case StateTx::DATA_4_SCL_CLR:
        case StateTx::DATA_3_SCL_CLR:
        case StateTx::DATA_2_SCL_CLR:
        case StateTx::DATA_1_SCL_CLR:
        case StateTx::DATA_0_SCL_CLR:
        {
            // We can only change the data when SCL is low.
            HW::SCL_Pin::set(0);
            // The divide by 2 factor (shift right by 1) is because the enum
            // states alternate between *_SCL_SET and *_SCL_CLR states in
            // increasing value.
            uint8_t mask = 0x80 >>
                ((static_cast<int>(stateTx_) -
                  static_cast<int>(StateTx::DATA_7_SCL_CLR)) >> 1);
            if (data & mask)
            {
                HW::SDA_Pin::set(1);
            }
            else
            {
                HW::SDA_Pin::set(0);
            }
            ++stateTx_;
            break;
        }
        case StateTx::DATA_7_SCL_SET:
        case StateTx::DATA_6_SCL_SET:
        case StateTx::DATA_5_SCL_SET:
        case StateTx::DATA_4_SCL_SET:
        case StateTx::DATA_3_SCL_SET:
        case StateTx::DATA_2_SCL_SET:
        case StateTx::DATA_1_SCL_SET:
        case StateTx::DATA_0_SCL_SET:
            // Data is sampled by the slave following this state transition.
            HW::SCL_Pin::set(1);
            ++stateTx_;
            break;
        case StateTx::ACK_SDA_SET_SCL_CLR:
            HW::SCL_Pin::set(0);
            HW::SDA_Pin::set(1);
            ++stateTx_;
            break;
        case StateTx::ACK_SCL_SET:
            HW::SCL_Pin::set(1);
            ++stateTx_;
            break;
        case StateTx::ACK_SCL_CLR:
            if (HW::SCL_Pin::get() == false)
            {
                // Clock stretching, do the same state again.
                clockStretchActive_ = true;
                break;
            }
            if (clockStretchActive_)
            {
                // I2C spec requires minimum 4 microseconds after the SCL line
                // goes high following a clock stretch for the SCL line to be
                // pulled low again by the master. Do the same state one more
                // time to ensure this.
                clockStretchActive_ = false;
                break;
            }
            bool ack = (HW::SDA_Pin::get() == false);
            HW::SCL_Pin::set(0);
            if (!ack)
            {
                count_ = -EIO;
            }
            return true; // done
    }
    return false;
}

//
// BitBangI2C::state_rx()
//
template <class HW> __attribute__((optimize("-O3")))
inline bool BitBangI2C<HW>::state_rx(uint8_t *data, bool nack)
{
    switch(stateRx_)
    {
        case StateRx::DATA_7_SCL_SET:
            HW::SDA_Pin::set(1); // Always start with SDA high.
            // fall through
        case StateRx::DATA_6_SCL_SET:
        case StateRx::DATA_5_SCL_SET:
        case StateRx::DATA_4_SCL_SET:
        case StateRx::DATA_3_SCL_SET:
        case StateRx::DATA_2_SCL_SET:
        case StateRx::DATA_1_SCL_SET:
        case StateRx::DATA_0_SCL_SET:
            HW::SCL_Pin::set(1);
            ++stateRx_;
            break;
        case StateRx::DATA_7_SCL_CLR:
        case StateRx::DATA_6_SCL_CLR:
        case StateRx::DATA_5_SCL_CLR:
        case StateRx::DATA_4_SCL_CLR:
        case StateRx::DATA_3_SCL_CLR:
        case StateRx::DATA_2_SCL_CLR:
        case StateRx::DATA_1_SCL_CLR:
        case StateRx::DATA_0_SCL_CLR:
            if (HW::SCL_Pin::get() == false)
            {
                // Clock stretching, do the same state again.
                clockStretchActive_ = true;
                break;
            }
            if (clockStretchActive_)
            {
                // I2C spec requires minimum 4 microseconds after the SCL line
                // goes high following a clock stretch for the SCL line to be
                // pulled low again by the master or for the master to sample
                // the SDA data. Do the same state one more time to ensure this.
                clockStretchActive_ = false;
                break;
            }
            *data <<= 1;
            if (HW::SDA_Pin::get() == true)
            {
                *data |= 0x01;
            }
            HW::SCL_Pin::set(0);
            if (stateRx_ == StateRx::DATA_0_SCL_CLR && !nack)
            {
                // Send the ACK. If a NACK, SDA is already set.
                HW::SDA_Pin::set(0);
            }
            ++stateRx_;
            break;
        case StateRx::ACK_SDA_SCL_SET:
            HW::SCL_Pin::set(1);
            ++stateRx_;
            break;
        case StateRx::ACK_SCL_CLR:
            if (HW::SCL_Pin::get() == false)
            {
                // Clock stretching, do the same state again.
                clockStretchActive_ = true;
                break;
            }
            if (clockStretchActive_)
            {
                // I2C spec requires minimum 4 microseconds after the SCL line
                // goes high following a clock stretch for the SCL line to be
                // pulled low again by the master. Do the same state one more
                // time to ensure this.
                clockStretchActive_ = false;
                break;
            }
            HW::SCL_Pin::set(0);
            HW::SDA_Pin::set(1);
            return true;
    }
    return false;
}

//
// BitBangI2C::state_rx()
//
template <class HW> __attribute__((optimize("-O3")))
inline bool BitBangI2C<HW>::state_recovery()
{
    switch(stateRecovery_)
    {
        case StateRecovery::SDA_SET:
            HW::SDA_Pin::set(1); // Always start with SDA high.
            break;
        case StateRecovery::DATA_7_SCL_SET:
        case StateRecovery::DATA_6_SCL_SET:
        case StateRecovery::DATA_5_SCL_SET:
        case StateRecovery::DATA_4_SCL_SET:
        case StateRecovery::DATA_3_SCL_SET:
        case StateRecovery::DATA_2_SCL_SET:
        case StateRecovery::DATA_1_SCL_SET:
        case StateRecovery::DATA_0_SCL_SET:
        case StateRecovery::ACK_SCL_SET:
            HW::SCL_Pin::set(1);
            break;
        case StateRecovery::DATA_7_SCL_CLR:
        case StateRecovery::DATA_6_SCL_CLR:
        case StateRecovery::DATA_5_SCL_CLR:
        case StateRecovery::DATA_4_SCL_CLR:
        case StateRecovery::DATA_3_SCL_CLR:
        case StateRecovery::DATA_2_SCL_CLR:
        case StateRecovery::DATA_1_SCL_CLR:
        case StateRecovery::DATA_0_SCL_CLR:
        case StateRecovery::ACK_SCL_CLR:
            HW::SCL_Pin::set(0);
            break;
    }
    if (stateRecovery_ == StateRecovery::LAST)
    {
        return true;
    }

    ++stateRecovery_;
    return false;
}

//
// BitBangI2C::transfer()
//
template <class HW>
inline int BitBangI2C<HW>::transfer(struct i2c_msg *msg, bool stop)
{
    while (stop_)
    {
        // No need for a timed wait here because the stop state machine is not
        // gated by what happens on the bus (e.g. zero stretching).
        sem_.wait();
    }
    if (msg->len == 0)
    {
        // Message must have length of at least 1.
        return -EINVAL;
    }

    // Flush/reset semaphore.
    while (sem_.timedwait(0) == 0)
    {
    }

    {
        AtomicHolder h(this);
        // Reset state for a start/restart.
        msg_ = msg;
        count_ = 0;
        stop_ = stop;
        state_ = State::START;
        stateStart_ = StateStart::FIRST;

        // Enable tick timer.
        HW::tick_enable();
    }

    // We wait a minimum of 10 msec to account for any rounding in the "tick"
    // rate conversion. msg_->len is at least 1. We assume that worst ~50kHz
    // is the slowest that anyone will try to run the I2C bus, which will
    // result in just under 200 microseconds per byte. This leaves some room
    // for clock stretching.
    long long wait = std::max(MSEC_TO_NSEC(msg_->len), MSEC_TO_NSEC(10));
    if (sem_.timedwait(wait) == 0)
    {
        return count_;
    }
    else
    {
        // Flush/reset semaphore.
        while (sem_.timedwait(0) == 0)
        {
        }

        // Some kind of problem occured. Try a recovery operation.
        {
            AtomicHolder h(this);
            state_ = State::RECOVERY;
            stateRecovery_ = StateRecovery::FIRST;
        }

        // We don't care about the sempahore return value because there is
        // isn't much more we can do anyways. Just ensure stop_ is false and
        // ticks are disabled. There is actually no reason to expect the
        // semaphore to timeout, so this is really just defensive programming.
        sem_.timedwait(wait);

        // stop_ must be false for the next call of this method. It should only
        // be true on first entry at start to "reset" the bus to a known state.
        // On a timeout, it may not have been reset back to false.
        {
            AtomicHolder h(this);
            stop_ = false;
            HW::tick_disable();
        }

        return -ETIMEDOUT;
    }
}

#endif // _FREERTOS_DRIVERS_COMMON_BITBANG_I2C_HXX_
