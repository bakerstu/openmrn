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
 * @file BitBangI2C.cxx
 * This file implements a bit bang implementation of I2C.
 *
 * @author Stuart W. Baker
 * @date 28 March 2023
 */

#include "BitBangI2C.hxx"

//
// BitBangI2C::tick_interrupt()
//
void BitBangI2C::tick_interrupt()
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
                else if (++count_ == msg_->len)
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
            }
            break;
        case State::DATA_RX:
            if (state_rx(msg_->buf + count_, (count_ + 1 == msg_->len)))
            {
                if (count_ < 0)
                {
                    // Some error occured, likely an unexpected NACK
                    exit = true;
                }
                else if (++count_ == msg_->len)
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
            }
            break;
        case State::STOP:
            exit = state_stop();
            break;
    }

    if (exit)
    {
        disableTick_();
        int woken = 0;
        sem_.post_from_isr(&woken);
        os_isr_exit_yield_test(woken);
    }
}

//
// BitBangI2C::state_start()
//
bool BitBangI2C::state_start()
{
    switch (stateStart_)
    {
        // start sequence
        case StateStart::SDA_SET:
            gpio_set(sda_);
            ++stateStart_;
            break;
        case StateStart::SCL_SET:
            gpio_set(scl_);
            ++stateStart_;
            break;
        case StateStart::SDA_CLR:
            gpio_clr(sda_);
            ++stateStart_;
            break;
        case StateStart::SCL_CLR:
            gpio_clr(scl_);
            return true;
    }
    return false;
}

//
// BitBangI2C::state_stop()
//
bool BitBangI2C::state_stop()
{
    switch (stateStop_)
    {
        case StateStop::SDA_CLR:
            gpio_clr(sda_);
            ++stateStop_;
            break;
        case StateStop::SCL_SET:
            gpio_set(scl_);
            ++stateStop_;
            break;
        case StateStop::SDA_SET:
            gpio_set(sda_);
            stop_ = false;
            return true; // exit
    }
    return false;
}

//
// BitBangI2C::state_tx()
//
bool BitBangI2C::state_tx(uint8_t data)
{
    switch(stateTx_)
    {
        case StateTx::DATA_7_SCL_SET:
        case StateTx::DATA_6_SCL_SET:
        case StateTx::DATA_5_SCL_SET:
        case StateTx::DATA_4_SCL_SET:
        case StateTx::DATA_3_SCL_SET:
        case StateTx::DATA_2_SCL_SET:
        case StateTx::DATA_1_SCL_SET:
        case StateTx::DATA_0_SCL_SET:
        {
            uint8_t mask = 0x80 >>
                ((static_cast<int>(stateTx_) -
                  static_cast<int>(StateTx::DATA_7_SCL_SET)) >> 1);
            if (data & mask)
            {
                gpio_set(sda_);
            }
            else
            {
                gpio_clr(sda_);
            }
            gpio_set(scl_);
            ++stateTx_;
            break;
        }
        case StateTx::DATA_7_SCL_CLR:
        case StateTx::DATA_6_SCL_CLR:
        case StateTx::DATA_5_SCL_CLR:
        case StateTx::DATA_4_SCL_CLR:
        case StateTx::DATA_3_SCL_CLR:
        case StateTx::DATA_2_SCL_CLR:
        case StateTx::DATA_1_SCL_CLR:
        case StateTx::DATA_0_SCL_CLR:
            gpio_clr(scl_);
            ++stateTx_;
            break;
        case StateTx::ACK_SDA_SCL_SET:
            gpio_set(sda_);
            gpio_set(scl_);
            ++stateTx_;
            break;
        case StateTx::ACK_SCL_CLR:
            bool ack = (sda_->read() == Gpio::Value::CLR);
            gpio_clr(scl_);
            if (!ack)
            {
                count_ = -EIO;
            }
            stateTx_ = StateTx::DATA_7_SCL_SET;
            return true; // done
    }
    return false;
}

//
// BitBangI2C::state_rx()
//
bool BitBangI2C::state_rx(uint8_t *data, bool nack)
{
    switch(stateRx_)
    {
        case StateRx::DATA_7_SCL_SET:
            gpio_set(sda_); // Always start with SDA high.
            // fall through
        case StateRx::DATA_6_SCL_SET:
        case StateRx::DATA_5_SCL_SET:
        case StateRx::DATA_4_SCL_SET:
        case StateRx::DATA_3_SCL_SET:
        case StateRx::DATA_2_SCL_SET:
        case StateRx::DATA_1_SCL_SET:
        case StateRx::DATA_0_SCL_SET:
        {
            gpio_set(scl_);
            if (scl_->read() == Gpio::Value::SET)
            {
                // not clock stretching, move on.
                ++stateRx_;
            }
            // else clock stretching, stay in this state
            break;
        }
        case StateRx::DATA_7_SCL_CLR:
        case StateRx::DATA_6_SCL_CLR:
        case StateRx::DATA_5_SCL_CLR:
        case StateRx::DATA_4_SCL_CLR:
        case StateRx::DATA_3_SCL_CLR:
        case StateRx::DATA_2_SCL_CLR:
        case StateRx::DATA_1_SCL_CLR:
        case StateRx::DATA_0_SCL_CLR:
            *data <<= 1;
            if (sda_->read() == Gpio::Value::SET)
            {
                *data |= 0x01;
            }
            gpio_clr(scl_);
            ++stateRx_;
            break;
        case StateRx::ACK_SDA_SCL_SET:
            if (nack)
            {
                // Send NACK.
                gpio_set(sda_);
            }
            else
            {
                // Send ACK.
                gpio_clr(sda_);
            }
            gpio_set(scl_);
            ++stateRx_;
            break;
        case StateRx::ACK_SCL_CLR:
            gpio_clr(scl_);
            gpio_set(sda_);
            stateRx_ = StateRx::DATA_7_SCL_SET;
            return true;
    }
    return false;
}

//
// BitBangI2C::transfer()
//
int BitBangI2C::transfer(struct i2c_msg *msg, bool stop)
{
    while (stop_)
    {
        // waiting for the initial "stop" condition on reset
        sem_.wait();
    }
    if (msg_->len == 0)
    {
        // Message must have length of at least 1.
        return -EINVAL;
    }
    msg_ = msg;
    count_ = 0;
    stop_ = stop;
    state_ = State::START;
    stateStart_ = StateStart::FIRST;
    enableTick_();
    sem_.wait();
    return count_;
}

