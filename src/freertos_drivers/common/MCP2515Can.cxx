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
 * @file MCP2515Can.cxx
 * This file implements the CAN driver for the MCP2515 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 3 January 2017
 */

#include "MCP2515Can.hxx"

#include <fcntl.h>

#define DEBUG 0

const MCP2515Can::MCP2515Baud MCP2515Can::baudTable[] =
{
    /* 20 MHz clock source
     * TQ = (2 * BRP) / freq = (2 * 4) / 20 MHz = 400 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 20 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * PS1 = 8 TQ
     * PS2 = 4 TQ
     * sample time = (1 TQ + 7 TQ + 8 TQ) / 20 TQ = 80%
     * SJW = PS2 - 1 = 4 - 1 = 3
     * SJW = 3 * 400 nsec = 1.2 usec
     */
    {20000000, 125000, {(4 - 1), (2 - 1)},
                       {(7 - 1), (8 - 1), 0, 1},
                       {(4 - 1), 0, 0}},
    /* 8 MHz clock source
     * TQ = (2 * BRP) / freq = (2 * 2) / 8 MHz = 500 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 16 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 4 TQ
     * PS1 = 8 TQ
     * PS2 = 3 TQ
     * sample time = (1 TQ + 4 TQ + 8 TQ) / 16 TQ = 81.25%
     * SJW = PS2 - 1 = 3 - 1 = 2
     * SJW = 2 * 500 nsec = 1 usec
     */
    {8000000, 125000, {(2 - 1), (2 - 1)},
                      {(4 - 1), (8 - 1), 0, 1},
                      {(3 - 1), 0, 0}}
};


/*
 * init()
 */
void MCP2515Can::init(const char *spi_name, uint32_t freq, uint32_t baud)
{
    spi = ::open(spi_name, O_RDWR);
    HASSERT(spi >= 0);

    /* reset device */
    reset();

    /* wait until device is in configuration mode */
    while ((register_read(CANSTAT) & 0xE0) != 0x80);

    /* find valid timing settings for the requested frequency and buad rates */
    for (size_t i = 0; i < (sizeof(baudTable) / sizeof(baudTable[0])); ++i)
    {
        if (baudTable[i].freq == freq && baudTable[i].baud == baud)
        {
            register_write(CNF1, baudTable[i].cnf1.data);
            register_write(CNF2, baudTable[i].cnf2.data);
            register_write(CNF3, baudTable[i].cnf3.data);

            /* setup RX Buf 0 and 1 to receive any message */
            register_write(RXB0CTRL, 0x60);
            register_write(RXB1CTRL, 0x60);

            /* enable error and receive interrupts */
            register_write(CANINTE, MERR | ERRI | RX1I | RX0I);

            /* put the device into normal operation mode */
            register_write(CANCTRL, 0x00);

            /* wait until device is in normal mode */
            while ((register_read(CANSTAT) & 0xE0) != 0x00);

            return;
        }
    }

    /* unsupported frequency */
    HASSERT(0);
}

/*
 * enable()
 */
void MCP2515Can::enable()
{
    /* there is a mutex lock above us, so the following sequence is atomic */
    if (!is_created())
    {
        /* start the thread */
        /** @todo make this the highest possible thread priority */
        start(name, 0, 1024);
    }

    /* reset device */
    //reset();

    interrupt_enable();
}

/*
 * disable()
 */
void MCP2515Can::disable()
{
    interrupt_disable();

    register_write(TXB0CTRL, 0x00);
    register_write(TXB1CTRL, 0x00);
    portENTER_CRITICAL();
    /* flush out any transmit data in the pipleline */
    txBuf->flush();
    txPending = 0;
    portEXIT_CRITICAL();
}

/* 
 * tx_msg()
 */
void MCP2515Can::tx_msg()
{
    /* the node lock_ will be locked by the caller */
    if (txPending < 3)
    {
        struct can_frame *can_frame;

        if (txBuf->data_read_pointer(&can_frame))
        {
            /* find an empty buffer */
            int index = 0;
            if (txPending & 0x1)
            {
                /* buffer 0 already in use */
                index = 1;
            }

            Buffer tx_buf;
            memset(&tx_buf, 0, sizeof(tx_buf));

            if (can_frame->can_eff)
            {
                /* extended frame */
                tx_buf.eid0 = ((can_frame->can_id & 0x000000FF) >> 0);
                tx_buf.eid8 = ((can_frame->can_id & 0x0000FF00) >> 8);
                tx_buf.sidl = ((can_frame->can_id & 0x00030000) >> 16) +
                              ((can_frame->can_id & 0x001C0000) >> 13);
                tx_buf.sidh = ((can_frame->can_id & 0x1FE00000) >> 21);
                tx_buf.sidl |= 0x08;
            }
            else
            {
                /* standard frame */
                tx_buf.sidl = (can_frame->can_id & 0x00000007) << 5;
                tx_buf.sidh = (can_frame->can_id & 0x000007F8) >> 3;
            }
            memcpy(tx_buf.data, can_frame->data, 8);
            tx_buf.dlc = can_frame->can_dlc;
            if (can_frame->can_rtr)
            {
                tx_buf.dlc |= 0x40;
            }
            txPending |= (0x1 << index);
            txBuf->consume(1);

            portEXIT_CRITICAL();
            /* bump up priority of the other buffer so it will transmit first
             * if it is pending
             */
            bit_modify(index == 0 ? TXB1CTRL : TXB0CTRL, 0x01, 0x03);
            /* load the tranmsit buffer */
            buffer_write(index, &tx_buf);
            /* request to send at lowest priority */
            bit_modify(index == 0 ? TXB0CTRL : TXB1CTRL, 0x08, 0x0B);
            /* enable transmit interrupt */
            bit_modify(CANINTE, TX0I << index, TX0I << index);
            portENTER_CRITICAL();
        }
    }
}

/*
 * rx_msg()
 */
void MCP2515Can::rx_msg(int index)
{
    Buffer rx_buf;
    buffer_read(index, &rx_buf);
    struct can_frame *can_frame;

    portENTER_CRITICAL();
    if (rxBuf->data_write_pointer(&can_frame))
    {
        can_frame->can_rtr = (rx_buf.dlc & 0x40) ? 1 : 0;
        if (rx_buf.sidl & 0x08)
        {
            /* extended frame */
            can_frame->can_eff = 1;
            can_frame->can_rtr = (rx_buf.dlc & 0x40) ? 1 : 0;
            can_frame->can_id = (((uint32_t)rx_buf.eid0 & 0xFF) <<  0) +
                                (((uint32_t)rx_buf.eid8 & 0xFF) <<  8) +
                                (((uint32_t)rx_buf.sidl & 0x03) << 16) +
                                (((uint32_t)rx_buf.sidl & 0xE0) << 13) +
                                (((uint32_t)rx_buf.sidh & 0xFF) << 21);
        }
        else
        {
            /* standard frame */
            can_frame->can_eff = 0;
            can_frame->can_rtr = (rx_buf.sidl & 0x01) ? 1 : 0;
            can_frame->can_id = ((uint32_t)rx_buf.sidl >> 5) +
                                ((uint32_t)rx_buf.sidh << 3);
        }
        can_frame->can_err = 0;
        memcpy(can_frame->data, rx_buf.data, 8);
        can_frame->can_dlc = rx_buf.dlc & 0x0F;

        rxBuf->advance(1);
        ++numReceivedPackets_;
        rxBuf->signal_condition();
    }
    else
    {
        /* receive overrun occured */
        ++overrunCount;
    }
    portEXIT_CRITICAL();
}

/*
 * entry()
 */
void *MCP2515Can::entry()
{
    for ( ; /* forever */ ; )
    {
#if DEBUG
        int result = sem.timedwait(SEC_TO_NSEC(1));
        lock_.lock();

        if (result != 0)
        {
            spi_ioc_transfer xfer[2];
            memset(xfer, 0, sizeof(xfer));
            uint8_t wr_data[2] = {READ, 0};
            xfer[0].tx_buf = (unsigned long)wr_data;
            xfer[0].len = sizeof(wr_data);
            xfer[1].rx_buf = (unsigned long)regs;
            xfer[1].len = sizeof(regs);
            xfer[1].cs_change = 1;
            ::ioctl(spi, SPI_IOC_MESSAGE(2), xfer);
            lock_.unlock();
            continue;
        }
#else
        sem.wait();
        lock_.lock();
#endif
        /* read status flags */
        uint8_t canintf = register_read(CANINTF);

        if (canintf & MERR)
        {
            /* message error interrupt active */
        }
        if (canintf & ERRI)
        {
            /* error interrupt active */
            register_write(TXB0CTRL, 0x00);
            register_write(TXB1CTRL, 0x00);
            portENTER_CRITICAL();
            ++softErrorCount;
            /* flush out any transmit data in the pipleline */
            txBuf->flush();
            txPending = 0;
            txBuf->signal_condition();
            portEXIT_CRITICAL();
        }
        if (canintf & RX0I)
        {
            /* receive interrupt active */
            rx_msg(0);
        }

        if (txPending)
        {
            /* transmit interrupt active and transmission complete */
            if (canintf & TX0I)
            {
                txPending &= ~0x1;
                bit_modify(CANINTE, 0, TX0I);
                bit_modify(CANINTF, 0, TX0I);
            }
            if (canintf & TX1I)
            {
                txPending &= ~0x2;
                bit_modify(CANINTE, 0, TX1I);
                bit_modify(CANINTF, 0, TX1I);
            }

            portENTER_CRITICAL();
            tx_msg();
            portEXIT_CRITICAL();
        }

        /* Refresh status flags just in case RX1 buffer became active
         * before we could finish reading out RX0 buffer.  This ussually
         * won't happen because we should be able to respond to incoming
         * messages fast enough to only use RX0 buffer.
         */
        canintf = register_read(CANINTF);
        if (canintf & RX1I)
        {
            /* receive interrupt active */
            rx_msg(1);
        }
        lock_.unlock();

        interrupt_enable();
    }

    return NULL;
}

/* 
 * interrupt_handler()
 */
void MCP2515Can::interrupt_handler()
{
    int woken = false;
    interrupt_disable();
    sem.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}
