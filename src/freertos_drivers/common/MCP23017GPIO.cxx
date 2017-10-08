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
 * MCP23017::init()
 */
void MCP23017::init(const char *i2c_name, uint8_t i2c_address)
{
    fd_ = ::open(i2c_name, O_RDWR);
    HASSERT(fd_ >= 0);

    ioctl(fd_, I2C_SLAVE, i2c_address);
    i2cAddress_ = i2c_address;

    start("mcp23017", 0, 1024);
}

/*
 * MCP23017::entry()
 */
void *MCP23017::entry()
{
    register_write(IOCON, 0x68);
    register_write(GPINTENA, 0xFF);
    register_write(GPINTENB, 0xFF);
    register_write(GPPUA, 0xFF);
    register_write(GPPUB, 0xFF);

    dataInA_ = register_read(GPIOA);
    dataInB_ = register_read(GPIOB);

    long long interrupt_lockout = 0;

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
        interrupt_disable();

        /* read interrupt status */
        uint16_t int_status;
        int_status = register_read(INTFA);
        int_status += register_read(INTFB) << 8;

        if (int_status)
        {
            interrupt_lockout = OSTime::get_monotonic() + intLockTime_;
        }

        /* read remote data and update local data copy*/
        dataInA_ = register_read(GPIOA);
        dataInB_ = register_read(GPIOB);

        if (directionShaddow_ != direction_)
        {
            /* flush any direction changes */
            direction_ = directionShaddow_;
            register_write(IODIRA, direction_ & 0xFF);
            register_write(IODIRB, direction_ >> 8);
        }

        portENTER_CRITICAL();
        if ((dataShaddow_ & ~direction_) != (dataOut_ & ~direction_))
        {
            dataOut_ = dataShaddow_;
            portEXIT_CRITICAL();

            register_write(OLATA, dataOut_ & 0xFF);
            register_write(OLATB, dataOut_ >> 8);
        }
        else
        {
            portEXIT_CRITICAL();
        }
    }

    return nullptr;
}

/*
 * MCP23017::register_write()
 */
void MCP23017::register_write(Registers reg, uint8_t value)
{
    uint8_t wr_buf[] = {reg, value};

    ::write(fd_, wr_buf, sizeof(wr_buf));
}

/*
 * MCP23017::register_read()
 */
uint8_t MCP23017::register_read(Registers reg)
{
    uint8_t wr_reg = reg;
    uint8_t rd_data;

    struct i2c_msg read_msg[2];

    read_msg[0].addr = i2cAddress_;
    read_msg[0].flags = 0;
    read_msg[0].len = 1;
    read_msg[0].buf = &wr_reg;

    read_msg[1].addr = i2cAddress_;
    read_msg[1].flags = I2C_M_RD;
    read_msg[1].len = 1;
    read_msg[1].buf = &rd_data;

    struct i2c_rdwr_ioctl_data rdwr_ioctl_data = {read_msg, 2};

    ::ioctl(fd_, I2C_RDWR, &rdwr_ioctl_data);

    return rd_data;
}


/* 
 * interrupt_handler()
 */
void MCP23017::interrupt_handler()
{
    int woken = false;
    interrupt_disable();
    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}
