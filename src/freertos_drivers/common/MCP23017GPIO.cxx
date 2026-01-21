/** @copyright
 * Copyright (c) 2017 Stuart W Baker
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
 * @file MCP23017GIO.cxx
 * This file implements a GPIO driver for the MCP23017 I/O expander.
 *
 * @author Stuart W. Baker
 * @date 7 October 2017
 */

#include "MCP23017GPIO.hxx"

#include <unistd.h>
#include <fcntl.h>
#include <stropts.h>

#include "i2c.h"
#include "i2c-dev.h"

/*
 * MCP23017Base::init()
 */
void MCP23017Base::init(const char *i2c_name, uint8_t i2c_address)
{
    fd_ = ::open(i2c_name, O_RDWR);
    HASSERT(fd_ >= 0);

    i2cAddress_ = i2c_address;
    ioctl(fd_, I2C_SLAVE, i2cAddress_);

    start("mcp23017", 0, 1024);
}

/*
 * MCP23017Base::entry()
 */
void *MCP23017Base::entry()
{
    long long interrupt_lockout = 0;
    uint16_t data_out_last[chips()];
    uint16_t direction_last[chips()];

    for (uint8_t i = 0; i < chips(); ++i)
    {
        /* initialize last data */
        data_out_last[i] = get_data_out(i);
        direction_last[i] = get_direction(i);

        /* initialize the device */
        reg_write(IOCON, 0x68, i);
        reg_write(GPINTENA, 0xFF, i);
        reg_write(GPINTENB, 0xFF, i);
        reg_write(GPPUA, 0xFF, i);
        reg_write(GPPUB, 0xFF, i);

        /* get the starting input port data */
        set_data_in(reg_read(GPIOA, i) + (reg_read(GPIOB, i) << 8), i);
    }

    for ( ; /* forever */ ; )
    {
        bool wait = true;
        long long now = OSTime::get_monotonic();
        if (now < interrupt_lockout)
        {
            /* we need to enable interrupts only after a debounce delay */
            wait = (sem_.timedwait(interrupt_lockout - now) != 0);
        }
        if (wait)
        {
            /* wait for interrupt or application write */
            interrupt_enable();
            sem_.wait();
        }

        /* possibly reset the interrupt debounce hold off */
        now = OSTime::get_monotonic();
        uint8_t int_status = 0;
        for (uint8_t i = 0; i < chips(); ++i)
        {
            /* flag a possible interrupt status being active */
            int_status |= reg_read(INTFA, i);
            int_status |= reg_read(INTFB, i);
        }
        if (int_status)
        {
            /* there was an interrupt driven change */
            interrupt_lockout = now + intLockTime_;
        }

        for (uint8_t i = 0; i < chips(); ++i)
        {
            /* read remote input port data and update local copy*/
            set_data_in(reg_read(GPIOA, i) + (reg_read(GPIOB, i) << 8), i);

            if (data_out_last[i] != get_data_out(i))
            {
                /* flush any output latch changes */
                data_out_last[i] = get_data_out(i);
                reg_write(OLATA, data_out_last[i] & 0xFF, i);
                reg_write(OLATB, data_out_last[i] >> 8, i);
            }

            if (direction_last[i] != get_direction(i))
            {
                /* flush any direction changes */
                direction_last[i] = get_direction(i);
                reg_write(IODIRA, direction_last[i] & 0xFF, i);
                reg_write(IODIRB, direction_last[i] >> 8, i);
                /* disable interrupts on outputs */
                reg_write(GPINTENA, direction_last[i], i);
                reg_write(GPINTENB, direction_last[i], i);
            }
        }
    }

    return nullptr;
}

/*
 * MCP23017Base::reg_write()
 */
void MCP23017Base::reg_write(Registers reg, uint8_t value, uint8_t index)
{
    uint8_t wr_buf[] = {reg, value};

    struct i2c_msg msg[1];

    msg[0].addr = i2cAddress_ + index;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = wr_buf;

    struct i2c_rdwr_ioctl_data rdwr_ioctl_data = {msg, 1};

    ::ioctl(fd_, I2C_RDWR, &rdwr_ioctl_data);
}

/*
 * MCP23017Base::reg_read()
 */
uint8_t MCP23017Base::reg_read(Registers reg, uint8_t index)
{
    uint8_t wr_buf[] = {reg};
    uint8_t rd_data;

    struct i2c_msg msg[2];

    msg[0].addr = i2cAddress_ + index;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = wr_buf;

    msg[1].addr = i2cAddress_ + index;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = &rd_data;

    struct i2c_rdwr_ioctl_data rdwr_ioctl_data = {msg, 2};

    ::ioctl(fd_, I2C_RDWR, &rdwr_ioctl_data);

    return rd_data;
}


/* 
 * MCP23017Base::interrupt_handler()
 */
void MCP23017Base::interrupt_handler()
{
    int woken = false;
    interrupt_disable();
    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}
