/** @copyright
 * Copyright (c) 2020 Stuart W Baker
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
 * @file TCAN4550Can.cxx
 * This file implements the CAN driver for the TCAN4550 CAN Controller.
 *
 * @author Stuart W. Baker
 * @date 26 February 2020
 */

#include "TCAN4550Can.hxx"

#include <fcntl.h>

const TCAN4550Can::TCAN4550Baud TCAN4550Can::BAUD_TABLE[] =
{
    /* 20 MHz clock source
     * TQ = BRP / freq = 10 / 20 MHz = 500 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 16 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * Seg1 = 4 TQ
     * Seg2 = 4 TQ
     * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
     * SJW = Seg - 1 = 4 - 1 = 3
     * SJW = 3 * 500 nsec = 1.5 usec
     *
     * Oscillator Tolerance:
     *     4 / (2 * ((13 * 16) - 4)) = 0.980%
     *     3 / (20 * 16) = 0.938%
     *     = 0.938%
     */
    {20000000, 125000, {.dsjw = (3 - 1),
                        .dtseg2 = (4 - 1),
                        .dtseg1 = (11 - 1),
                        .dbrp = (10 - 1),
                        .tdc = 0}},
    /* 20 MHz clock source
     * TQ = BRP / freq = 10 / 20 MHz = 500 nsec
     * Baud = 125 kHz
     * bit time = 1 / 125 kHz = 8 usec = 16 TQ
     * SyncSeg = 1 TQ
     * PropSeg = 7 TQ
     * Seg1 = 4 TQ
     * Seg2 = 4 TQ
     * sample time = (1 TQ + 7 TQ + 4 TQ) / 16 TQ = 75%
     * SJW = Seg - 1 = 4 - 1 = 3
     * SJW = 3 * 500 nsec = 1.5 usec
     *
     * Oscillator Tolerance:
     *     4 / (2 * ((13 * 16) - 4)) = 0.980%
     *     3 / (20 * 16) = 0.938%
     *     = 0.938%
     */
    {40000000, 125000, {.dsjw = (3 - 1),
                        .dtseg2 = (4 - 1),
                        .dtseg1 = (11 - 1),
                        .dbrp = (20 - 1),
                        .tdc = 0}},
};


//
// init()
//
void TCAN4550Can::init(const char *spi_name, uint32_t freq, uint32_t baud)
{
    spiFd_ = ::open(spi_name, O_RDWR);
    HASSERT(spiFd_ >= 0);

    ::ioctl(spiFd_, SPI_IOC_GET_OBJECT_REFERENCE, &spi_);
    HASSERT(spi_);

    // configure SPI bus settings
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

    // transition to "Normal" mode with sleep and watchdog disabled
    {
        Mode mode;
        mode.sweDis = 1; // disable sleep
        mode.wdEnable = 0; // disable watchdog
        mode.modeSel = 2; // normal mode

        lock_.lock();
        register_write(MODE, mode.data);
        lock_.unlock();
    }
    {
        Cccr cccr;
        cccr.cce = 1; // configuration change enable
        register_write(CCCR, cccr.data);

        // clear MRAM
        for (unsigned offset = 0; offset < MRAM_SIZE_WORDS;
             offset += MRAMMessageClear::DATA_SIZE)
        {
            MRAMMessageClear msg;
            mram_write(offset, &msg, sizeof(msg));
        }

        // ---- Memory layout ----
        //
        // +-----------------------+
        // | RX FIFO 0, buf 0      | 0x000
        // | ...                   |
        // | RX FIFO 0, buf 31     | 0x1F0
        // +-----------------------+
        // | TX Event FIFO, buf 0  | 0x200
        // | ...                   |
        // | TX Event FIFO, buf 31 | 0x2F8
        // +-----------------------+
        // | TX Buf 0 (FIFO)       | 0x300
        // | ...                   |
        // | TX Buf 31 (FIFO)      |
        // +-----------------------+
        // | TX Buf 32             | 0x500
        // | ...                   |
        // | TX Buf 63             | 0x6F0
        // +-----------------------+
        // | Unused                | 0x700
        // +-----------------------+
    }
}

//
// enable()
//
void TCAN4550Can::enable()
{
    /* there is a mutex lock above us, so the following sequence is atomic */
    if (!is_created())
    {
        /* start the thread at the highest priority in the system */
        start(name, get_priority_max(), 2048);
    }

    interruptEnable_();
}

//
// disable()
//
void TCAN4550Can::disable()
{
    interruptDisable_();

    portENTER_CRITICAL();
    // flush out any transmit data in the pipleline
    txBuf->flush();
    //txPending_ = 0;
    portEXIT_CRITICAL();
}

//
// entry()
//
__attribute__((optimize("-O3")))
void *TCAN4550Can::entry()
{
#if 0
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
#endif
    return NULL;
}

