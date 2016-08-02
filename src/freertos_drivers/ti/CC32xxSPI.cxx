/** \copyright
 * Copyright (c) 2016, Stuart W Baker
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
 * \file CC32xxSPI.cxx
 * This file implements a SPI device driver layer specific to CC32xx.
 *
 * @author Stuart W. Baker
 * @date 30 May 2016
 */

#include <algorithm>

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/spi.h"
#include "driverlib/interrupt.h"
#include "driverlib/prcm.h"

#include "CC32xxSPI.hxx"

/** Instance pointers help us get context from the interrupt handler(s) */
static CC32xxSPI *instances[1] = {NULL};

/** Constructor.
 * @param name name of this device instance in the file system
 * @param base base address of this device
 * @param interrupt interrupt number of this device
 * @param cs_assert function pointer to a method that asserts chip select
 * @param cs_deassert function pointer to a method that deasserts chip
 *                    select
 * @param bus_lock the user must provide a shared mutex if the device
 *                 instance represents more than one chip select on the
 *                 same bus interface.
 */
CC32xxSPI::CC32xxSPI(const char *name, unsigned long base, uint32_t interrupt,
                     ChipSelectMethod cs_assert, ChipSelectMethod cs_deassert,
                     OSMutex *bus_lock)
    : SPI(name, cs_assert, cs_deassert, bus_lock)
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
        case GSPI_BASE:
            MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
            MAP_PRCMPeripheralReset(PRCM_GSPI);
            clock = MAP_PRCMPeripheralClockGet(PRCM_GSPI);
            instances[0] = this;
            break;
    }

    MAP_SPIReset(base);

    MAP_SPIConfigSetExpClk(base, clock, speedHz, SPI_MODE_MASTER,
                           SPI_SUB_MODE_3, (/*SPI_SW_CTRL_CS |*/ SPI_3PIN_MODE |
                                            SPI_TURBO_OFF | SPI_WL_8));

    MAP_SPIEnable(base);
}

/** Method to transmit/receive the data.
 * @param msg message(s) to transact.
 * @return bytes transfered upon success, -errno upon failure
 */
int CC32xxSPI::transfer(struct spi_ioc_transfer *msg)
{
    SPITransfer(base, (unsigned char*)msg->tx_buf,
                    (unsigned char*)msg->rx_buf, msg->len, 0);

    return msg->len;
}

/// Debugging helper.
volatile uint32_t g_error;
/// Debugging helper.
volatile uint32_t g_status;

/** Common interrupt handler for all SPI devices.
 */
void CC32xxSPI::interrupt_handler()
{
#if 0
    int woken = 0;
    uint32_t error;
    uint32_t status;

    g_error = error = SPIMasterErr(base);
    g_status = status = MAP_SPIMasterIntStatusEx(base, true);
    MAP_SPIMasterIntClearEx(base, status);

    if (error & SPI_MCS_ARBLST)
    {
        count_ = -EIO;
        goto post;
    }
    else if (error & SPI_MCS_DATACK)
    {
        count_ = -EIO;
        goto post;
    }
    else if (error & SPI_MCS_ADRACK)
    {
        count_ = -EIO;
        goto post;
    }
    else if (status & SPI_MASTER_INT_TIMEOUT)
    {
        count_ = -ETIMEDOUT;
        goto post;
    }
    else if (status & SPI_MASTER_INT_DATA)
    {
        if (msg_->flags & SPI_M_RD)
        {
            /* this is a read transfer */
            msg_->buf[count_++] = SPIMasterDataGet(base);
            if (count_ == msg_->len)
            {
                /* transfer is complete */
                goto post;
            }
            if (stop_ && count_ == (msg_->len - 1))
            {
                /* single byte transfer with stop */
                MAP_SPIMasterControl(base, SPI_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
            else
            {
                /* more than one byte left to transfer */
                MAP_SPIMasterControl(base, SPI_MASTER_CMD_BURST_RECEIVE_CONT);
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
            MAP_SPIMasterDataPut(base, msg_->buf[count_]);
            if (stop_ && count_ == (msg_->len - 1))
            {
                /* single byte transfer with stop */
                MAP_SPIMasterControl(base, SPI_MASTER_CMD_BURST_SEND_FINISH);
            }
            else
            {
                /* more than one byte left to transfer */
                MAP_SPIMasterControl(base, SPI_MASTER_CMD_BURST_SEND_CONT);
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
        HASSERT(0 && "SPI invalid status");
        }*/
post:
    MAP_IntDisable(interrupt);
    sem.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
#endif
}

extern "C" {
/** SPI0 interrupt handler.
 */
void spi0_interrupt_handler(void)
{
    if (instances[0])
    {
        instances[0]->interrupt_handler();
    }
}
} // extern C
