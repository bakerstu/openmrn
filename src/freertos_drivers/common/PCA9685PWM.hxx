/** @copyright
 * Copyright (c) 2018 Stuart W Baker
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
 * @file PCA9685PWM.hxx
 * This file implements a PWM driver for the PCA9685.
 *
 * @author Stuart W. Baker
 * @date 6 January 2018
 */

#include <fcntl.h>
#include <unistd.h>
#include "stropts.h"
#include "i2c.h"
#include "i2c-dev.h"

#include "PWM.hxx"


#include "os/OS.hxx"

class PCA9685PWMBit;

/// Agragate of 16 PWM channels for a PCA9685PWM
class PCA9685PWM : public OSThread
{
public:
    /// maximum number of PWM channels supported by the PCA9685
    static constexpr size_t NUM_CHANNELS = 16;

    /// maximum number of PWM counts supported by the PCA9685
    static constexpr size_t MAX_PWM_COUNTS = 4096;

    /// Constructor.
    PCA9685PWM()
        : sem_(0)
        , i2c_(-1)
        , dirty_(0)
        , i2cAddress_(0)
    {
        duty_.fill(0);
    }

    /// Initialize device.
    /// @param name name of the I2C device
    /// @param i2c_address I2C address of the device on the bus
    /// @param pwm_frequency target PWM frequency (will truncate at minimum and
    ///                      maximum prescaler values)
    /// @param external_clock frequency of an external clock in Hz, -1
    ///                       if internal clock is used
    void init(const char * name, uint8_t i2c_address,
              uint16_t pwm_freq = 200, int32_t external_clock_freq = -1)
    {
        uint32_t clock_freq = external_clock_freq >= 0 ? external_clock_freq :
                                                         25000000;
        i2cAddress_ = i2c_address;

        i2c_ = ::open(name, O_RDWR);
        HASSERT(i2c_ >= 0);
        ::ioctl(i2c_, I2C_SLAVE, i2cAddress_);

        HASSERT(clock_freq <= 50000000);
        HASSERT(pwm_freq < (clock_freq / (4096 * 4)));

        Mode1 mode1;
        mode1.aI = 1;
        mode1.sleep = 1;
        mode1.allCall = 0;
        register_write(MODE1, mode1.byte);

        uint8_t prescaler = (clock_freq / (4096 * (uint32_t)pwm_freq)) - 1;
        register_write(PRE_SCALE, prescaler);

        if (external_clock_freq < 0)
        {
            /* if using internal clock */
            mode1.sleep = 0;
            register_write(MODE1, mode1.byte);
        }

        Mode2 mode2;
        mode2.och = 1;
        register_write(MODE2, mode2.byte);

        /* start the thread at the highest priority in the system */
        start(name, configMAX_PRIORITIES - 1, 1024);
    }

    /// Destructor.
    ~PCA9685PWM()
    {
    }

private:
    /// Important device register offsets
    enum Registers
    {
        MODE1 = 0, ///< mode 1 settings
        MODE2, ///< mode 2 settings

        LED0_ON_L = 6, ///< first LED control register

        PRE_SCALE = 254, ///< clock prescale divider
    };

    /// Represent the mode 1 register.
    union Mode1
    {
        /// Constructor
        Mode1()
            : byte(0x01)
        {
        }

        uint8_t byte; ///< full byte value
        struct
        {
            uint8_t allCall : 1; ///< respond to "all call" address
            uint8_t sub3    : 1; ///< sub-address 3
            uint8_t sub2    : 1; ///< sub-address 2
            uint8_t sub1    : 1; ///< sub-address 1
            uint8_t sleep   : 1; ///< sleep enabled
            uint8_t aI      : 1; ///< auto increment
            uint8_t extClk  : 1; ///< external clock
            uint8_t restart : 1; ///< restart
        };
    };

    /// Represent the mode 2 register.
    union Mode2
    {
        /// Constructor.
        Mode2()
            : byte(0x04)
        {
        }

        uint8_t byte; ///< full byte data
        struct
        {
            uint8_t outNE  : 2; ///< output not enable
            uint8_t outDrv : 1; ///< 1 = push/pull, 0 = open drain
            uint8_t och    : 1; ///< 1 = updata on ack, 0 = update on stop
            uint8_t invert : 1; ///< invert output
            uint8_t unused : 3; ///< unused
        };
    };

    /// Representation of the replicative 4 LED control register
    struct LedCtl
    {
        union On
        {
            /// Constructor.
            On()
                : word(0x0000)
            {
            }
            uint16_t word; ///< full word data
            struct
            {
                uint16_t counts : 12; ///< turn on counts
                uint16_t fullOn :  1; ///< set if full on
                uint16_t unused :  3; ///< unused
            };
        };
        union Off
        {
            /// Constructor.
            Off()
                : word(0x1000)
            {
            }
            uint16_t word; ///< full word data
            struct
            {
                uint16_t counts  : 12; ///< turn off counts
                uint16_t fullOff :  1; ///< set if full off
                uint16_t unused  :  3; ///< unused
            };
        };
        On on; ///< on registers instance
        Off off; ///< off registers instance
    };

    /// User entry point for the created thread.
    /// @return exit status
    void *entry() override;

    /// Bit modify to an I2C register.
    /// @param address address to modify
    /// @param data data to modify
    /// @param mask mask of data to modify, bits where mask is 1 will be
    ///             overwritten, bits where mask is zero will be kept
    void bit_modify(Registers address, uint8_t data, uint8_t mask)
    {
        uint8_t addr = address;
        uint8_t rd_data;
        struct i2c_msg msgs[] =
        {
            {
                .addr = i2cAddress_,
                .flags = 0,
                .len = 1,
                .buf = &addr
            },
            {
                .addr = i2cAddress_,
                .flags = I2C_M_RD,
                .len = 1,
                .buf = &rd_data
            }
        };
        
        struct i2c_rdwr_ioctl_data ioctl_data =
            {.msgs = msgs, .nmsgs = ARRAYSIZE(msgs)};

        ::ioctl(i2c_, I2C_RDWR, &ioctl_data);

        rd_data &= ~mask;
        rd_data |= (data & mask);

        register_write(address, rd_data);
    }

    /// Set the pwm duty cycle
    /// @param channel channel index (0 through 15)
    /// @param counts counts for PWM duty cycle
    void set_pwm_duty(unsigned channel, uint16_t counts)
    {
        HASSERT(channel < NUM_CHANNELS);
        duty_[channel] = counts;

        portENTER_CRITICAL();
        dirty_ |= 0x1 << channel;
        portEXIT_CRITICAL();

        sem_.post();
    }

    /// Get the pwm duty cycle
    /// @param channel channel index (0 through 15)
    /// @return counts for PWM duty cycle
    uint16_t get_pwm_duty(unsigned channel)
    {
        HASSERT(channel < NUM_CHANNELS);
        return duty_[channel];
    }

    /// Set the pwm duty cycle
    /// @param channel channel index (0 through 15)
    /// @param counts counts for PWM duty cycle
    void write_pwm_duty(unsigned channel, uint16_t counts)
    {
        LedCtl ctl;
        if (counts >= MAX_PWM_COUNTS)
        {
            ctl.on.fullOn = 1;
            ctl.off.fullOff = 0;
        }
        else if (counts == 0)
        {
            ctl.on.fullOn = 0;
            ctl.off.fullOff = 1;
        }
        else
        {
            // the "256" count offset is to help average the current accross
            // all 16 channels when the duty cycle is low
            ctl.on.counts = (channel * 256);
            ctl.off.counts = (counts + (channel * 256)) % 0x1000;
        }

        htole16(ctl.on.word);
        htole16(ctl.off.word);

        Registers offset = (Registers)(LED0_ON_L + (channel * 4));
        register_write_multiple(offset, &ctl, sizeof(ctl));
    }

    /// Write to an I2C register.
    /// @param address address to write to
    /// @param data data to write
    void register_write(Registers address, uint8_t data)
    {
        uint8_t payload[] = {address, data};
        ::write(i2c_, payload, sizeof(payload));
    }

    /// Write to multiple sequential I2C registers.
    /// @param address address to start write at
    /// @param data array of data to write
    /// @param count number of data registers to write in sequence
    void register_write_multiple(Registers address, void *data, size_t count)
    {
        uint8_t payload[count + 1];
        payload[0] = address;
        memcpy(payload + 1, data, count);
        ::write(i2c_, payload, sizeof(payload));
    }

    /// Allow access to private members
    friend PCA9685PWMBit;

    /// Wakeup for the thread processing
    OSSem sem_;

    /// I2C file descriptor
    int i2c_;

    /// local cache of the duty cycles
    std::array<uint16_t, NUM_CHANNELS> duty_;

    /// set if the duty_ value is updated (bit mask)
    uint16_t dirty_;

    /// I2C address of the device
    uint8_t i2cAddress_;

    DISALLOW_COPY_AND_ASSIGN(PCA9685PWM);
};

/// Specialization of the PWM abstract interface for the PCA9685
class PCA9685PWMBit : public PWM
{
public:
    /// Constructor.
    /// @param instance reference to the chip
    /// @param index channel index on the chip (0 through 15) 
    PCA9685PWMBit(PCA9685PWM *instance, unsigned index)
        : PWM()
        , instance_(instance)
        , index_(index)
    {
        HASSERT(index < PCA9685PWM::NUM_CHANNELS);
    }

    /// Destructor.
    ~PCA9685PWMBit()
    {
    }

private:
    /// Set PWM period.
    /// @param PWM period in counts
    void set_period(uint32_t counts) override
    {
        HASSERT(counts == PCA9685PWM::MAX_PWM_COUNTS);
    };

    /// Get PWM period.
    /// @return PWM period in counts
    uint32_t get_period() override
    {
        return PCA9685PWM::MAX_PWM_COUNTS;
    }

    /// Sets the duty cycle.
    /// @param counts duty cycle in counts
    void set_duty(uint32_t counts) override
    {
        instance_->set_pwm_duty(index_, counts);
    }

    /// Gets the duty cycle.
    /// @return counts duty cycle in counts
    uint32_t get_duty() override
    {
        return instance_->get_pwm_duty(index_);
    }

    /// Get max period supported
    /// @return period in counts
    uint32_t get_period_max() override
    {
        return get_period();
    }

    /// Get min period supported
    /// @return period in counts
    uint32_t get_period_min() override
    {
        return get_period();
    }

    /// instance pointer to the whole chip complement
    PCA9685PWM *instance_;

    /// bit index within PCA9685
    unsigned index_;

    DISALLOW_COPY_AND_ASSIGN(PCA9685PWMBit);
};

