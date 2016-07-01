/** \copyright
 * Copyright (c) 2014, Stuart W Baker
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
 * \file TivaI2C.cxx
 * This file implements a UART device driver layer specific to Tivaware.
 *
 * @author Stuart W. Baker
 * @date 6 May 2014
 */

#include <algorithm>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "TivaDev.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static TivaI2C *instances[10] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 */
TivaI2C::TivaI2C(const char *name, unsigned long base, uint32_t interrupt)
    : I2C(name)
    , base(base)
    , interrupt(interrupt)
    , msg_(NULL)
    , stop_(false)
    , count_(0)
    , sem()
{
    
    switch (base)
    {
        default:
            HASSERT(0);
        case I2C0_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
            instances[0] = this;
            break;
        case I2C1_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
            instances[1] = this;
            break;
        case I2C2_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
            instances[2] = this;
            break;
        case I2C3_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
            instances[3] = this;
            break;
        case I2C4_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
            instances[4] = this;
            break;
        case I2C5_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C5);
            instances[5] = this;
            break;
        case I2C6_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);
            instances[6] = this;
            break;
        case I2C7_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
            instances[7] = this;
            break;
        case I2C8_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C8);
            instances[8] = this;
            break;
        case I2C9_BASE:
            MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C9);
            instances[9] = this;
            break;
    }

    /* default to 100 KHz mode */
    MAP_I2CMasterInitExpClk(base, cm3_cpu_clock_hz, false);

    /* We set the priority so that it is slightly lower than the highest needed
     * for FreeRTOS compatibility. This will ensure that CAN interrupts take
     * precedence over UART. */
    MAP_IntPrioritySet(interrupt,
                       std::min(0xff, configKERNEL_INTERRUPT_PRIORITY + 0x20));
    MAP_I2CMasterIntEnableEx(base, I2C_MASTER_INT_TIMEOUT |
                                   I2C_MASTER_INT_DATA);
}

/** Method to transmit/receive the data.
 * @param msg message to transact.
 * @param stop produce a stop condition at the end of the transfer
 * @return bytes transfered upon success or -1 with errno set
 */
int TivaI2C::transfer(struct i2c_msg *msg, bool stop)
{
    int bytes = msg->len;

    if (msg->flags & I2C_M_RD)
    {
        /* this is a read transfer */
	    MAP_I2CMasterSlaveAddrSet(base, msg->addr, true);

        /* generate start */
        if (stop && msg->len == 1)
        {
            /* single byte transfer with stop */
            MAP_I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_RECEIVE);
        }
        else
        {
            /* more than one byte to transfer */
            MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_START);
        }
    }
    else
    {
        /* this is a write transfer */
	    MAP_I2CMasterSlaveAddrSet(base, msg->addr, false);
        MAP_I2CMasterDataPut(base, *msg->buf);

        /* generate start */
        if (stop && msg->len == 1)
        {
            /* single byte transfer with stop */
            MAP_I2CMasterControl(base, I2C_MASTER_CMD_SINGLE_SEND);
        }
        else
        {
            /* more than one byte to transfer */
            MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_START);
        }
    }

    msg_ = msg;
    count_ = 0;
    stop_ = stop;
    
    MAP_IntEnable(interrupt);
    sem.wait();

    return count_ < 0 ? count_ : bytes;    
}

/// Debugging variable that holds the last error encountered by the driver. For
/// gdb access.
volatile uint32_t g_error;
/// Debugging variable that holds the last master int status seen by the
/// driver. For gdb access.
volatile uint32_t g_status;

/** Common interrupt handler for all I2C devices.
 */
void TivaI2C::interrupt_handler()
{
    int woken = 0;
    uint32_t error;
    uint32_t status;

    g_error = error = I2CMasterErr(base);
    g_status = status = MAP_I2CMasterIntStatusEx(base, true);
    MAP_I2CMasterIntClearEx(base, status);

    if (error & I2C_MCS_ARBLST)
    {
        count_ = -EIO;
        goto post;
    }
    else if (error & I2C_MCS_DATACK)
    {
        count_ = -EIO;
        goto post;
    }
    else if (error & I2C_MCS_ADRACK)
    {
        count_ = -EIO;
        goto post;
    }
    else if (status & I2C_MASTER_INT_TIMEOUT)
    {
        count_ = -ETIMEDOUT;
        goto post;
    }
    else if (status & I2C_MASTER_INT_DATA)
    {
        if (msg_->flags & I2C_M_RD)
        {
            /* this is a read transfer */
            msg_->buf[count_++] = I2CMasterDataGet(base);
            if (count_ == msg_->len)
            {
                /* transfer is complete */
                goto post;
            }
            if (stop_ && count_ == (msg_->len - 1))
            {
                /* single byte transfer with stop */
                MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
            else
            {
                /* more than one byte left to transfer */
                MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }
        }
        else
        {
            /* this is a write transfer */
            if (++count_ == msg_->len)
            {
                /* transfer is complete */
                goto post;
            }
            MAP_I2CMasterDataPut(base, msg_->buf[count_]);
            if (stop_ && count_ == (msg_->len - 1))
            {
                /* single byte transfer with stop */
                MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_FINISH);
            }
            else
            {
                /* more than one byte left to transfer */
                MAP_I2CMasterControl(base, I2C_MASTER_CMD_BURST_SEND_CONT);
            }
        }
        return;
    }
    else// if (error)
    {
        count_ = -EIO;
        goto post;
    }
/*    else
    {
        // should never get here
        HASSERT(0 && "I2C invalid status");
        }*/
post:
    MAP_IntDisable(interrupt);
    sem.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}

extern "C" {
/** I2C0 interrupt handler.
 */
void i2c0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}

/** I2C1 interrupt handler.
 */
void i2c1_interrupt_handler(void)
{
    if (instances[1])
    {
        instances[1]->interrupt_handler();
    }
}

/** I2C2 interrupt handler.
 */
void i2c2_interrupt_handler(void)
{
    if (instances[2])
    {
        instances[2]->interrupt_handler();
    }
}

/** I2C3 interrupt handler.
 */
void i2c3_interrupt_handler(void)
{
    if (instances[3])
    {
        instances[3]->interrupt_handler();
    }
}
/** I2C4 interrupt handler.
 */
void i2c4_interrupt_handler(void)
{
    if (instances[4])
    {
        instances[4]->interrupt_handler();
    }
}

/** I2C5 interrupt handler.
 */
void i2c5_interrupt_handler(void)
{
    if (instances[5])
    {
        instances[5]->interrupt_handler();
    }
}

/** I2C6 interrupt handler.
 */
void i2c6_interrupt_handler(void)
{
    if (instances[6])
    {
        instances[6]->interrupt_handler();
    }
}

/** I2C7 interrupt handler.
 */
void i2c7_interrupt_handler(void)
{
    if (instances[7])
    {
        instances[7]->interrupt_handler();
    }
}

/** I2C8 interrupt handler.
 */
void i2c8_interrupt_handler(void)
{
    if (instances[8])
    {
        instances[8]->interrupt_handler();
    }
}

/** I2C9 interrupt handler.
 */
void i2c9_interrupt_handler(void)
{
    if (instances[9])
    {
        instances[9]->interrupt_handler();
    }
}

} // extern C
