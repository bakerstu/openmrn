/** \copyright
 * Copyright (c) 2018, Balazs Racz
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
 * \file Stm32I2C.hxx
 *
 * This file implements an I2C device driver layer on top of the STM32 Cube
 * middleware.
 *
 * @author Balazs Racz
 * @date 28 Oct 2018
 */

#ifndef _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_
#define _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_

#include <fcntl.h>
#include <unistd.h>
#include "i2c.h"
#include "i2c-dev.h"

#include "executor/Executor.hxx"

class MCP23017 {
public:
    // 50 Hz polling
    static constexpr long long POLLING_DELAY = MSEC_TO_NSEC(20);

    /// Constructor
    /// @param a2 address bit 2
    /// @param a1 address bit 1
    /// @param a0 address bit 0
    MCP23017(ExecutorBase* executor, bool a2, bool a1, bool a0)
        : executor_(executor) {
        unsigned num = 0;
        if (a2) {
            num |= 0b100;
        }
        if (a1) {
            num |= 0b010;
        }
        if (a0) {
            num |= 0b001;
        }
        i2cAddress_ = BASE_ADDRESS | num;
    }

    /// Initializes the device.
    /// @param i2c_path is the path to the device driver of the i2c port.
    void init(const char* i2c_path) {
        int fd = ::open(i2c_path, O_RDWR);
        HASSERT(fd >= 0);
        init(fd);
    }

    /// Initializes the device.
    /// @param i2c_fd is the file descriptor of the i2c port to use.
    void init(int i2c_fd)
    {
        fd_ = i2c_fd;

        dir_[0] = dir_[1] = 0xff;
        dirty_ = DIRTY_DIR;
        update_out();
        update_in();

        timer_.start(POLLING_DELAY);
    }

    enum {
        PORTA = 0,
        PORTB = 1,
    };

private:
    DISALLOW_COPY_AND_ASSIGN(MCP23017);

    friend class MCP23017Gpio;

    enum Registers {
        IODIRA = 0x0,
        IODIRB = 0x1,
        IPOLA = 0x2,
        IPOLB = 0x3,
        GPINTENA = 0x4,
        GPINTENB = 0x5,
        DEFVALA = 0x6,
        DEFVALB = 0x7,
        INTCONA = 0x8,
        INTCONB = 0x9,
        IOCON = 0xA,
        GPPUA = 0xC,
        GPPUB = 0xD,
        INTFA = 0xE,
        INTFB = 0xF,
        INTCAPA = 0x10,
        INTCAPB = 0x11,
        RGPIOA = 0x12,
        RGPIOB = 0x13,
        OLATA = 0x14,
        OLATB = 0x15
    };

    enum {
        /// I2C address of the first device.
        BASE_ADDRESS = 0b0100000,

        DIRTY_DIR = 1,
        DIRTY_LAT = 2,
    };

    /// Updates the output direction and latch registers if any of it is dirty.
    void update_out()
    {
        if (dirty_ & DIRTY_DIR)
        {
            dirty_ &= ~DIRTY_DIR;
            register_write(IODIRA, dir_, 2);
        }
        if (dirty_ & DIRTY_LAT)
        {
            dirty_ &= ~DIRTY_LAT;
            register_write(OLATA, lat_, 2);
        }
    }

    /// Updates input registers.
    void update_in()
    {
        register_read(RGPIOA, gpio_, 2);
    }

    /// Writes one or more (sequential) registers in the MCP23017.
    /// @param reg starting register offset.
    /// @param data payload to write
    /// @param len number of bytes to write.
    /// Returns when the write is complete.
    void register_write(uint8_t reg, const uint8_t* data, uint16_t len) {
        uint8_t dat[3];
        dat[0] = reg;
        dat[1] = data[0];
        if (len > 1) {
            dat[2] = data[1];
        }
        HASSERT(len <= 2);
        struct i2c_msg msgs[] =
        {
            {
                .addr = i2cAddress_,
                .flags = 0,
                .len = (uint16_t)(len + 1),
                .buf = dat
            }
        };

        struct i2c_rdwr_ioctl_data ioctl_data =
            {.msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    /// Reads one or more (sequential) registers in the MCP23017.
    /// @param reg starting register offset.
    /// @param data where to write payload
    /// @param len number of registers to read.
    /// Returns when the read is complete.
    void register_read(uint8_t reg, uint8_t* data, uint16_t len) {
        struct i2c_msg msgs[] =
        {
            {
                .addr = i2cAddress_,
                .flags = 0,
                .len = 1,
                .buf = &reg
            },
            {
                .addr = i2cAddress_,
                .flags = I2C_M_RD,
                .len = len,
                .buf = data
            }
        };

        struct i2c_rdwr_ioctl_data ioctl_data =
            {.msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    class RefreshTimer : public ::Timer {
    public:
        RefreshTimer(MCP23017 *parent)
            : ::Timer(parent->executor_->active_timers())
            , parent_(parent)
        {
        }

        long long timeout() override
        {
            parent_->update_out();
            parent_->update_in();
            return RESTART;
        };

    private:
        MCP23017* parent_;
    };

    /// Executor. We will be blocking this for the I2C IO.
    ExecutorBase* executor_;
    /// Timer instance to schedule work on the executor.
    RefreshTimer timer_{this};
    /// I2C port file descriptor.
    int fd_;
    /// Address of this particular device on the I2C port.
    uint8_t i2cAddress_;

    /// Bit mask of registers that need updating.
    uint8_t dirty_ = 0;

    /// Shadow of the direction registers.
    uint8_t dir_[2];
    /// Shadow of the input registers.
    uint8_t gpio_[2];
    /// Shadow of the latch registers.
    uint8_t lat_[2];
};


class MCP23017Gpio : public Gpio {
public:
    constexpr MCP23017Gpio(MCP23017 *const parent, unsigned port, unsigned pin)
        : parent_(parent)
        , port_(port)
        , pinBit_(1 << pin)
    {
    }

    void set() const override {
        write(HIGH);
    }

    void clr() const override {
        write(LOW);
    }

    void write(Value new_state) const override
    {
        bool old_state = !!(parent_->lat_[port_] & pinBit_);
        if (new_state != old_state)
        {
            if (new_state)
            {
                parent_->lat_[port_] |= pinBit_;
            }
            else
            {
                parent_->lat_[port_] &= ~pinBit_;
            }
            parent_->dirty_ |= parent_->DIRTY_LAT;
        }
    }

    Value read() const override
    {
        return (parent_->gpio_[port_] & pinBit_) ? HIGH : LOW;
    }

    void set_direction(Direction dir) const override
    {
        uint8_t desired = (dir == Direction::OUTPUT) ? 0 : pinBit_;
        if (desired != ((parent_->dir_[port_] & pinBit_)))
        {
            parent_->dir_[port_] =
                (parent_->dir_[port_] & (~pinBit_)) | desired;
            parent_->dirty_ |= parent_->DIRTY_DIR;
        }
    }

    Direction direction() const override
    {
        if (parent_->dir_[port_] & pinBit_)
        {
            return Direction::INPUT;
        }
        else
        {
            return Direction::OUTPUT;
        }
    }

private:
    DISALLOW_COPY_AND_ASSIGN(MCP23017Gpio);

    MCP23017* const parent_;
    /// 0 or 1 for portA/B
    const uint8_t port_;
    /// one bit that denotes the output pin.
    const uint8_t pinBit_;
};


#endif // _FREERTOS_DRIVERS_COMMON_MCP23017GPIO_HXX_
