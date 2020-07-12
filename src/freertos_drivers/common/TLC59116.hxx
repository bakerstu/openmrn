/** \copyright
 * Copyright (c) 2020, Rick Lull
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
 * \file TLC59116.hxx
 *
 * Hardware-independent driver for the TLC59116 LED driver chip using the
 * generic I2C driver.
 *
 * @author Rick Lull
 * @date 19 May 2020
 */

#ifndef _FREERTOS_DRIVERS_COMMON_TLC59116_HXX_
#define _FREERTOS_DRIVERS_COMMON_TLC59116_HXX_

#include <fcntl.h>
#include <unistd.h>
#include "i2c.h"
#include "i2c-dev.h"

#include "executor/Executor.hxx"

class TLC59116 : private Atomic
{
public:
    // 50 Hz polling
    static constexpr long long POLLING_DELAY = MSEC_TO_NSEC(20);

    /// Constructor
    /// @param a3 address bit 3
    /// @param a2 address bit 2
    /// @param a1 address bit 1
    /// @param a0 address bit 0
    TLC59116(ExecutorBase *executor, bool, a3 bool a2, bool a1, bool a0)
        : executor_(executor)
    {
        unsigned num = 0;
        if (a3)
	{
	    num |= 0b1000;
	}
	if (a2)
        {
            num |= 0b0100;
        }
        if (a1)
        {
            num |= 0b0010;
        }
        if (a0)
        {
            num |= 0b0001;
        }
        i2cAddress_ = BASE_ADDRESS | num;
    }

    /// Initializes the device.
    /// @param i2c_path is the path to the device driver of the i2c port.
    void init(const char *i2c_path)
    {
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

    enum
    {
        PORTA = 0,
        PORTB = 1,
    };

private:
    DISALLOW_COPY_AND_ASSIGN(TLC59116);

    friend class TLC59116Led;

    enum Registers
    {
        MODE1 = 0x0,
	MODE2 = 0x1,
	PWM0 = 0x2,
	PWM1 = 0x3,
	PWM2 = 0x4,
	PWM3 = 0x5,
	PWM4 = 0x6,
	PWM5 = 0x7,
	PWM6 = 0x8,
	PWM7 = 0x9,
	PWM8 = 0xA,
	PWM9 = 0xB,
	PWM10 = 0xC,
	PWM11 = 0xD,
	PWM12 = 0xE,
	PWM13 = 0xF,
	PWM14 = 0x10,
	PWM15 = 0x11,
	GRPPWM = 0x12,
	GRPFREQ = 0x13,
	LEDOUT0 = 0x14, // Contains LED0-3     Starting from bit 0, each LED has a pair of bits.
	LEDOUT1 = 0x15, // Contains LED4-7      Bits: 00 = Off		10 = LED is at individual brightness thru PWM register
	LEDOUT2 = 0x16, // Contains LED8-11           01 = Fully On	11 = LED is at individual brightness thru PWM register
	LEDOUT3 = 0x17, // Contains LED12-16				AND group brightness can be controll thru GRPPWM register
	SUBADR1 = 0x18,
	SUBADR2 = 0x19,
	SUBADR3 = 0X1A,
	ALLCALLADR = 0x1B,
	IREF = 0x1C,
	EFLAG1 = 0x1D, // Errors on channel (LED) 0-7
	EFLAG2 = 0x1E // Errors on 8-15
	    
	//    IODIRA = 0x0,
        //IODIRB = 0x1,
        //IPOLA = 0x2,
        //IPOLB = 0x3,
        //GPINTENA = 0x4,
        //GPINTENB = 0x5,
        //DEFVALA = 0x6,
        //DEFVALB = 0x7,
        //INTCONA = 0x8,
        //INTCONB = 0x9,
        //IOCON = 0xA,
        //GPPUA = 0xC,
        //GPPUB = 0xD,
        //INTFA = 0xE,
        //INTFB = 0xF,
        //INTCAPA = 0x10,
        //INTCAPB = 0x11,
        //RGPIOA = 0x12,
        //RGPIOB = 0x13,
        //OLATA = 0x14,
        //OLATB = 0x15
    };

    enum
    {
        /// I2C address of the first device. First address is 0x60 on TLC59116. Note that address 0x68 is 
	///  reserved as an broadcast (all call) to every TLC59116 on the bus. Addresses 0x68 thru 0x6C
	///  have some sort of reservation. You can get 10 TLCs on a single I2C bus without any other effort. 
        BASE_ADDRESS = 0b1100000,

        DIRTY_DIR = 1,
        DIRTY_LAT = 2,
    };

    /// Updates the output direction and latch registers if any of it is dirty.
    void update_out()
    {
        bool push_dir = false;
        bool push_lat = false;
        uint8_t d[2];
        uint8_t l[2];
        {
            AtomicHolder h(this);
            d[0] = dir_[0];
            d[1] = dir_[1];
            l[0] = lat_[0];
            l[1] = lat_[1];
            if (dirty_ & DIRTY_DIR)
            {
                dirty_ &= ~DIRTY_DIR;
                push_dir = true;
            }
            if (dirty_ & DIRTY_LAT)
            {
                dirty_ &= ~DIRTY_LAT;
                push_lat = true;
            }
        }
        if (push_dir)
        {
            register_write(IODIRA, d, 2);
        }
        if (push_lat)
        {
            register_write(OLATA, l, 2);
        }
    }

    /// Updates input registers.
    void update_in()
    {
        register_read(RGPIOA, gpio_, 2);
    }

    /// Writes one or more (sequential) registers in the TLC.
    /// @param reg starting register offset.
    /// @param data payload to write
    /// @param len number of bytes to write.
    /// Returns when the write is complete.
    void register_write(uint8_t reg, const uint8_t *data, uint16_t len)
    {
        uint8_t dat[3];
        dat[0] = reg;
        dat[1] = data[0];
        if (len > 1)
        {
            dat[2] = data[1];
        }
        HASSERT(len <= 2);
        struct i2c_msg msgs[] = {{.addr = i2cAddress_,
            .flags = 0,
            .len = (uint16_t)(len + 1),
            .buf = dat}};

        struct i2c_rdwr_ioctl_data ioctl_data = {
            .msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    /// Reads one or more (sequential) registers in the MCP23017.
    /// @param reg starting register offset.
    /// @param data where to write payload
    /// @param len number of registers to read.
    /// Returns when the read is complete.
    void register_read(uint8_t reg, uint8_t *data, uint16_t len)
    {
        struct i2c_msg msgs[] = {
            {.addr = i2cAddress_, .flags = 0, .len = 1, .buf = &reg},
            {.addr = i2cAddress_, .flags = I2C_M_RD, .len = len, .buf = data}};

        struct i2c_rdwr_ioctl_data ioctl_data = {
            .msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(fd_, I2C_RDWR, &ioctl_data);
    }

    /// This timer runs in the parent executor and upon every timeout it
    /// executes the update / polling of the IO expander.
    class RefreshTimer : public ::Timer
    {
    public:
        RefreshTimer(TLC59116 *parent)
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
        TLC59116 *parent_;
    };

    /// Executor. We will be blocking this for the I2C IO.
    ExecutorBase *executor_;
    /// Timer instance to schedule work on the executor.
    RefreshTimer timer_{this};
    /// I2C port file descriptor.
    int fd_;

    /// Shadow of the direction registers.
    uint8_t dir_[2];
    /// Shadow of the latch registers.
    uint8_t lat_[2];
    /// Shadow of the input registers.
    uint8_t gpio_[2];

    /// Address of this particular device on the I2C port.
    uint8_t i2cAddress_;

    /// Bit mask of registers that need updating.
    uint8_t dirty_ = 0;
};

class TLC59116Led
//: public Gpio //GPIO parent class, somewhere.. I don't think we need it since this is outputs only
{
public:
    constexpr TLC59116Led(unsigned channel, bool state) // channel is LED Output, bool state is on | off
        , port_(port)
        , pinBit_(1 << pin)
    {
    }

    void set() const override
    {
        write(VHIGH);
    }

    void clr() const override
    {
        write(VLOW);
    }

    void write(Value new_state) const override
    {
        AtomicHolder h(parent_);
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
        if (parent_->dir_[port_] & pinBit_)
        {
            // input. Use gpio_.
            return (parent_->gpio_[port_] & pinBit_) ? VHIGH : VLOW;
        }
        else
        {
            // output. Use lat_.
            return (parent_->lat_[port_] & pinBit_) ? VHIGH : VLOW;
        }
    }

    }

private:
    DISALLOW_COPY_AND_ASSIGN(TLC59116Led);

    MCP23017 *const parent_;
    /// 0 or 1 for portA/B
    const uint8_t port_;
    /// one bit that denotes the output pin.
    const uint8_t pinBit_;
};
endif // _FREERTOS_DRIVERS_COMMON_TLC59116_HXX_
