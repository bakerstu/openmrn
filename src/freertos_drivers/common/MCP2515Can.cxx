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

#define _DEFAULT_SOURCE

#include "MCP2515Can.hxx"

#include <fcntl.h>
#include <compiler.h>

const MCP2515Can::MCP2515Baud MCP2515Can::BAUD_TABLE[] =
{
    /* 20 MHz clock source
     * TQ = (2 * BRP) / freq = (2 * 5) / 20 MHz = 500 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 16 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * PS1 = 4 TQ
     * PS2 = 4 TQ
     * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
     * SJW = PS2 - 1 = 4 - 1 = 3
     * SJW = 3 * 500 nsec = 1.5 usec
     *
     * Oscillator Tolerance:
     *     4 / (2 * ((13 * 16) - 4)) = 0.980%
     *     3 / (20 * 16) = 0.938%
     *     = 0.938%
     */
    {20000000, 125000, {(5 - 1), (3 - 1)},
                       {(7 - 1), (4 - 1), 0, 1},
                       {(4 - 1), 0, 0}},
    /* 8 MHz clock source
     * TQ = (2 * BRP) / freq = (2 * 2) / 8 MHz = 500 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 16 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * PS1 = 4 TQ
     * PS2 = 4 TQ
     * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
     * SJW = PS2 - 1 = 4 - 1 = 3
     * SJW = 3 * 500 nsec = 1.5 usec
     *
     * Oscillator Tolerance:
     *     4 / (2 * ((13 * 16) - 4)) = 0.980%
     *     3 / (20 * 16) = 0.938%
     *     = 0.938%
     */
    {8000000, 125000, {(2 - 1), (3 - 1)},
                      {(7 - 1), (4 - 1), 0, 1},
                      {(4 - 1), 0, 0}}
};

/*
 * init()
 */
void MCP2515Can::init(const char *spi_name, uint32_t freq, uint32_t baud)
{
    spiFd_ = ::open(spi_name, O_RDWR);
    HASSERT(spiFd_ >= 0);

    ::ioctl(spiFd_, SPI_IOC_GET_OBJECT_REFERENCE, &spi_);
    HASSERT(spi_);

    /* configure SPI bus settings */
    uint8_t spi_mode = SPI_MODE_0;
    uint8_t spi_bpw = 8;
    uint32_t spi_max_speed_hz = freq / 2;
    if (spi_max_speed_hz > SPI_MAX_SPEED_HZ)
    {
        spi_max_speed_hz = SPI_MAX_SPEED_HZ;
    }
    ::ioctl(spiFd_, SPI_IOC_WR_MODE, &spi_mode);
    ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
    ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);

    /* reset device */
    reset();

    /* wait until device is in configuration mode */
    while ((register_read(CANSTAT) & 0xE0) != 0x80);

    /* find valid timing settings for the requested frequency and buad rates */
    for (size_t i = 0; i < ARRAYSIZE(BAUD_TABLE); ++i)
    {
        if (BAUD_TABLE[i].freq_ == freq && BAUD_TABLE[i].baud_ == baud)
        {
            register_write(CNF1, BAUD_TABLE[i].cnf1_.data_);
            register_write(CNF2, BAUD_TABLE[i].cnf2_.data_);
            register_write(CNF3, BAUD_TABLE[i].cnf3_.data_);

            /* setup RX Buf 0 to receive any message, and if RX Buf 0 is full,
             * the new message will roll over into RX Buf 1 */
            register_write(RXB0CTRL, 0x64);
            register_write(RXB1CTRL, 0x60);

            /* setup TXnRTS and RXnBF pins as inputs and outputs respectively */
            register_write(BFPCTRL, 0x0C | (gpoData_ << 4));
            register_write(TXRTSCTRL, 0x00);
            gpiData_ = (register_read(TXRTSCTRL) >> 3) & 0x7;

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
        /* start the thread at the highest priority in the system */
        start(name, get_priority_max(), 2048);
    }

    /* clear interrupt flags */
    register_write(CANINTF, 0);

    /* enable error and receive interrupts */
    register_write(CANINTE, MERR | ERRI | RX1I | RX0I);

    interruptEnable_();
}

/*
 * disable()
 */
void MCP2515Can::disable()
{
    interruptDisable_();

    /* disable all interrupt sources */
    register_write(CANINTE, 0);

    register_write(TXB0CTRL, 0x00);
    register_write(TXB1CTRL, 0x00);
    portENTER_CRITICAL();
    /* flush out any transmit data in the pipleline */
    txBuf->flush();
    txPending_ = 0;
    portEXIT_CRITICAL();
}

/* 
 * tx_msg_locked()
 */
__attribute__((optimize("-O3")))
void MCP2515Can::tx_msg_locked()
{
#if MCP2515_NULL_TX
    /* throw away the CAN frames after consuming them */
    struct can_frame *can_frame;

    portENTER_CRITICAL();
    if (txBuf->data_read_pointer(&can_frame))
    {
        txBuf->consume(1);
        txBuf->signal_condition();
    }
    portEXIT_CRITICAL();
#else
    /* the node lock_ will be locked by the caller */
    if (txPending_ < 3)
    {
        struct can_frame *can_frame;

        /* find an empty buffer */
        int index = (txPending_ & 0x1) ? 1 : 0;

        portENTER_CRITICAL();
        if (txBuf->data_read_pointer(&can_frame))
        {
            /* build up a transmit BufferWrite structure */
            BufferWrite buffer(index, can_frame);
            txBuf->consume(1);
            portEXIT_CRITICAL();

            /* bump up priority of the other buffer so it will
             * transmit first if it is pending
             */
            bit_modify(index == 0 ? TXB1CTRL : TXB0CTRL, 0x01, 0x03);

            /* load the tranmit buffer */
            buffer_write(&buffer, can_frame);

            txPending_ |= (0x1 << index);

            /* request to send at lowest priority */
            bit_modify(index == 0 ? TXB0CTRL : TXB1CTRL, 0x08, 0x0B);
            bit_modify(CANINTE, TX0I << index, TX0I << index);
            txBuf->signal_condition();
        }
        else
        {
            portEXIT_CRITICAL();
        }
    }
#endif
}

/*
 * MCP2515Can::ioctl()
 */
int MCP2515Can::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (key == SIOCGCANSTATE)
    {
        *((can_state_t*)data) = state_;
        return 0;
    }
    return -EINVAL;
}

/*
 * entry()
 */
__attribute__((optimize("-O3")))
void *MCP2515Can::entry()
{
    for ( ; /* forever */ ; )
    {
#if MCP2515_DEBUG
        int result = sem_.timedwait(SEC_TO_NSEC(1));

        if (result != 0)
        {
            lock_.lock();
            spi_ioc_transfer xfer[2];
            memset(xfer, 0, sizeof(xfer));
            uint8_t wr_data[2] = {READ, 0};
            xfer[0].tx_buf = (unsigned long)wr_data;
            xfer[0].len = sizeof(wr_data);
            xfer[1].rx_buf = (unsigned long)regs_;
            xfer[1].len = sizeof(regs_);
            xfer[1].cs_change = 1;
            ::ioctl(spiFd_, SPI_IOC_MESSAGE(2), xfer);
            lock_.unlock();
            continue;
        }
#else
        sem_.wait();
#endif
        lock_.lock();

        /* read status flags */
        uint8_t canintf = register_read(CANINTF);

        if (UNLIKELY((canintf & ERRI)) || UNLIKELY((canintf & MERR)))
        {
            /* error handling, read error flag register */
            uint8_t eflg = register_read(EFLG);

            /* clear error status flag */
            bit_modify(CANINTF, 0, ERRI | MERR);

            if (eflg & (RX0OVR | RX1OVR))
            {
                /* receive overrun */
                ++overrunCount;

                /* clear error flag */
                bit_modify(EFLG, 0, (RX0OVR | RX1OVR));
            }
            if (eflg & TXBO)
            {
                /* bus off */
                ++busOffCount;
                state_ = CAN_STATE_BUS_OFF;
            }
            if ((eflg & TXEP) || (eflg & RXEP))
            {
                /* error passive state */
                ++softErrorCount;
                state_ = CAN_STATE_BUS_PASSIVE;

                /* flush out any transmit data in the pipleline */
                register_write(TXB0CTRL, 0x00);
                register_write(TXB1CTRL, 0x00);
                bit_modify(CANINTE, 0, TX0I | TX1I);
                bit_modify(CANINTF, 0, TX0I | TX1I);

                portENTER_CRITICAL();
                txBuf->flush();
                portEXIT_CRITICAL();
                txBuf->signal_condition();

                txPending_ = 0;
            }
        }

        if (canintf & RX0I)
        {
            /* receive interrupt active */
            state_ = CAN_STATE_ACTIVE;
            BufferRead buffer(0);
            buffer_read(&buffer);
            struct can_frame *can_frame;

            portENTER_CRITICAL();
            if (LIKELY(rxBuf->data_write_pointer(&can_frame)))
            {
                buffer.build_struct_can_frame(can_frame);
                rxBuf->advance(1);
                rxBuf->signal_condition();
                ++numReceivedPackets_;
            }
            else
            {
                /* receive overrun occured */
                ++overrunCount;
            }
            portEXIT_CRITICAL();
        }

        /* RX Buf 1 can only be full if RX Buf 0 was also previously full.
         * It is extremely unlikely that RX Buf 1 will ever be full.
         */
        if (UNLIKELY(canintf & RX1I))
        {
            /* receive interrupt active */
            state_ = CAN_STATE_ACTIVE;
            BufferRead buffer(1);
            buffer_read(&buffer);
            struct can_frame *can_frame;

            portENTER_CRITICAL();
            if (LIKELY(rxBuf->data_write_pointer(&can_frame)))
            {
                buffer.build_struct_can_frame(can_frame);
                rxBuf->advance(1);
                rxBuf->signal_condition();
                ++numReceivedPackets_;
            }
            else
            {
                /* receive overrun occured */
                ++overrunCount;
            }
            portEXIT_CRITICAL();
        }

        if (txPending_)
        {
            /* transmit interrupt active and transmission complete */
            if (canintf & TX0I)
            {
                state_ = CAN_STATE_ACTIVE;
                txPending_ &= ~0x1;
                bit_modify(CANINTE, 0, TX0I);
                bit_modify(CANINTF, 0, TX0I);
                ++numTransmittedPackets_;
            }
            if (canintf & TX1I)
            {
                state_ = CAN_STATE_ACTIVE;
                txPending_ &= ~0x2;
                bit_modify(CANINTE, 0, TX1I);
                bit_modify(CANINTF, 0, TX1I);
                ++numTransmittedPackets_;
            }

            while (txPending_ < 3)
            {
                struct can_frame *can_frame;

                /* find an empty buffer */
                int index = (txPending_ & 0x1) ? 1 : 0;

                portENTER_CRITICAL();
                if (txBuf->data_read_pointer(&can_frame))
                {
                    /* build up a transmit BufferWrite structure */
                    BufferWrite buffer(index, can_frame);
                    txBuf->consume(1);
                    portEXIT_CRITICAL();

                    /* bump up priority of the other buffer so it will
                     * transmit first if it is pending
                     */
                    bit_modify(index == 0 ? TXB1CTRL : TXB0CTRL, 0x01, 0x03);

                    /* load the tranmit buffer */
                    buffer_write(&buffer, can_frame);

                    txPending_ |= (0x1 << index);

                    /* request to send at lowest priority */
                    bit_modify(index == 0 ? TXB0CTRL : TXB1CTRL, 0x08, 0x0B);
                    bit_modify(CANINTE, TX0I << index, TX0I << index);
                    txBuf->signal_condition();
                }
                else
                {
                    portEXIT_CRITICAL();
                    break;
                }
            }
        }

        if (UNLIKELY(ioPending_))
        {
            ioPending_ = false;
            /* write the latest GPO data */
            register_write(BFPCTRL, 0x0C | (gpoData_ << 4));

            /* get the latest GPI data */
            gpiData_ = (register_read(TXRTSCTRL) >> 3) & 0x7;
        }
        lock_.unlock();

        interruptEnable_();
    }

    return NULL;
}

/* 
 * interrupt_handler()
 */
__attribute__((optimize("-O3")))
void MCP2515Can::interrupt_handler()
{
    int woken = false;
    interruptDisable_();
    sem_.post_from_isr(&woken);
    os_isr_exit_yield_test(woken);
}
