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

/*
 * MCP2515Can()
 */
MCP2515Can::MCP2515Can(const char *name, const char *spi_name, uint32_t freq,
                       void (*interrupt_enable)(void),
                       void (*interrupt_disable)(void))
    : Can(name)
    , OSThread()
    , interrupt_enable(interrupt_enable)
    , interrupt_disable(interrupt_disable)
    , txPending(0)
    , spi(::open(spi_name, O_RDWR))
    , sem()
{
    HASSERT(spi >= 0);

    switch (freq)
    {
        default:
            /* unsupported frequency */
            HASSERT(0);
        case 20000000:
            /* 20 MHz clock source
             * TQ = (2 * BRP) / freq = (2 * 5) / 20 MHz = 500 nsec
             * Baud = 125 kHz
             * bit time = 1 / 125 kHz = 8 usec = 16 TQ
             * SyncSeg = 1 TQ
             * PropSeg = 4 TQ
             * PS1 = 8 TQ
             * PS2 = 3 TQ
             * sample time = (1 TQ + 4 TQ + 8 TQ) / 3 TQ = 81.25%
             * SJW = PS2 - 1 = 3 - 1 = 2
             */
            register_write(CNF1, 0x44);
            register_write(CNF2, 0xBB);
            register_write(CNF3, 0x02);
            break;
    }
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
    reset();

    /* setup RX Buf 0 and 1 to receive any message */
    register_write(RXB0CTRL, 0x60);
    register_write(RXB1CTRL, 0x60);

    /* enable error and receive interrupts */
    register_write(CANINTE, MERR | ERRI | RX1I | RX0I);

    /* put the device into normal operation mode */
    register_write(CANCTRL, 0x00);

    interrupt_enable();
}

/*
 * disable()
 */
void MCP2515Can::disable()
{
    interrupt_disable();

    /* reset device */
    reset();
}

/* 
 * tx_msg_locked()
 */
void MCP2515Can::tx_msg_locked()
{
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
                tx_buf.sidh = ((can_frame->can_id & 0x1FE00000) >> 19);
                tx_buf.sidl |= 0x08;
                tx_buf.sidh |= 0x08;
            }
            else
            {
                /* standard frame */
                tx_buf.sidl = (can_frame->can_id & 0x00000007) << 5;
                tx_buf.sidh = (can_frame->can_id & 0x000007F8) >> 3;
            }
            memcpy(&tx_buf.d0, can_frame->data, 8);
            tx_buf.dlc = can_frame->can_dlc;
            if (can_frame->can_rtr)
            {
                tx_buf.dlc |= 0x40;
            }
            txPending |= (0x1 << index);

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
        if (can_frame->can_eff == (rx_buf.sidl & 0x08))
        {
            /* extended frame */
            can_frame->can_eff = 1;
            can_frame->can_rtr = (rx_buf.dlc & 0x40) ? 1 : 0;
            can_frame->can_id = (((uint32_t)rx_buf.eid0 & 0xFF) <<  0) +
                                (((uint32_t)rx_buf.eid8 & 0xFF) <<  8) +
                                (((uint32_t)rx_buf.sidl & 0x03) << 16) +
                                (((uint32_t)rx_buf.sidl & 0xE0) << 13) +
                                (((uint32_t)rx_buf.sidh & 0xFF) << 19);
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
        memcpy(can_frame->data, &rx_buf.d0, 8);
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
        sem.wait();

        /* read status flags */
        uint8_t canintf = register_read(CANINTF);

        if (canintf & MERR)
        {
            /* message error interrupt active */
        }
        if (canintf & ERRI)
        {
            /* error interrupt active */
            lock_.lock();
            register_write(TXB0CTRL, 0x00);
            register_write(TXB1CTRL, 0x00);
            portENTER_CRITICAL();
            ++softErrorCount;
            /* flush out any transmit data in the pipleline */
            txBuf->flush();
            txPending = 0;
            txBuf->signal_condition();
            portEXIT_CRITICAL();
            lock_.unlock();
        }
        if (canintf & RX0I)
        {
            /* receive interrupt active */
            rx_msg(0);
        }

        lock_.lock();
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
            tx_msg_locked();
            portEXIT_CRITICAL();
        }
        lock_.unlock();

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
