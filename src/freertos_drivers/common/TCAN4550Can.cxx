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

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

#include "TCAN4550Can.hxx"

#include <fcntl.h>
#include <unistd.h>

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
    {20000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (10 - 1)}},
    /* 40 MHz clock source
     * TQ = BRP / freq = 10 / 40 MHz = 500 nsec
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
    {40000000, 125000, {(3 - 1), (4 - 1), (11 - 1), (20 - 1)}},
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
    uint32_t spi_max_speed_hz = freq / 4;
    if (spi_max_speed_hz > SPI_MAX_SPEED_HZ)
    {
        spi_max_speed_hz = SPI_MAX_SPEED_HZ;
    }
    ::ioctl(spiFd_, SPI_IOC_WR_MODE, &spi_mode);
    ::ioctl(spiFd_, SPI_IOC_WR_BITS_PER_WORD, &spi_bpw);
    ::ioctl(spiFd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_max_speed_hz);

    // lock SPI bus access
    OSMutexLock locker(&lock_);

    // transition to "Normal" mode with sleep and watchdog disabled
    {
        Mode mode;
        mode.sweDis = 1;   // disable sleep
        mode.wdEnable = 0; // disable watchdog
        mode.modeSel = 2;  // normal mode

        register_write(MODE, mode.data);
    }
    {
        Cccr cccr; // default is initialization mode
        register_write(CCCR, cccr.data);
        do
        {
            cccr.data = register_read(CCCR);
        } while (cccr.init == 0);

        cccr.cce = 1; // configuration change enable
        register_write(CCCR, cccr.data);
        do
        {
            cccr.data = register_read(CCCR);
        } while (cccr.cce == 0);
    }

    // diasable all TCAN interrupts
    register_write(INTERRUPT_ENABLE, 0);

    // clear MRAM
    for (unsigned offset = 0; offset < MRAM_SIZE_WORDS;
         offset += (MRAMMessageClear::DATA_SIZE * 4))
    {
        MRAMMessageClear msg;
        mram_write(offset, &msg, sizeof(msg));
    }

    // ---- Memory layout ----
    //
    // +-----------------------+
    // | RX FIFO 0, buf 0      | 0x0000
    // | ...                   |
    // | RX FIFO 0, buf 63     | 0x03F0
    // +-----------------------+
    // | TX Event FIFO, buf 0  | 0x0400
    // | ...                   |
    // | TX Event FIFO, buf 15 | 0x0478
    // +-----------------------+
    // | TX Buf 0              | 0x0480
    // | ...                   |
    // | TX Buf 15             | 0x0570
    // +-----------------------+
    // | TX Buf 16 (FIFO)      | 0x0580
    // | ...                   |
    // | TX Buf 31 (FIFO)      | 0x0670
    // +-----------------------+
    // | Unused                | 0x0680
    // +-----------------------+

    {
        // setup RX FIFO 0
        Rxfxc rxf0c;
        rxf0c.fsa = 0x0000; // FIFO start address
        rxf0c.fs = 64;      // FIFO size
        register_write(RXF0C, rxf0c.data);
    }
    {
        // setup TX configuration
        Txbc txbc;
        txbc.tbsa = 0x0480; // buffers start address
        txbc.ndtb = 16;     // number of dedicated transmit buffers
        txbc.tfqs = 16;     // FIFO/queue size
        register_write(TXBC, txbc.data);
    }
    {
        // setup TX buffer element size
        Txesc txesc;
        txesc.tbds = 0; // 8 byte data field size
        register_write(TXESC, txesc.data);
    }
    {

        // setup TX event FIFO
        Txefc txefc;
        txefc.efsa = 0x0400;        // event FIFO start address
        txefc.efs = 16;             // event FIFO size
        txefc.efwm = txefc.efs / 2; // event FIFO watermark
        register_write(TXEFC, txefc.data);
    }

    // Setup timing
    for ( size_t i = 0; i < ARRAYSIZE(BAUD_TABLE); ++i)
    {
        if (BAUD_TABLE[i].freq == freq && BAUD_TABLE[i].baud == baud)
        {
            register_write(NBTP, BAUD_TABLE[i].nbtp.data);

            Cccr cccr;
            do
            {
                cccr.init = 0; // normal operation
                cccr.cce = 0;  // configuration change disnable
                register_write(CCCR, cccr.data);
                cccr.data = register_read(CCCR);
            } while (cccr.init == 1);

            return;
        }
    }

    // unsupported frequency
    HASSERT(0);
}

//
// enable()
//
void TCAN4550Can::enable()
{
    // there is a mutex lock above us, so the following sequence is atomic
    if (!is_created())
    {
        // start the thread at the highest priority in the system
        start(name, get_priority_max(), 2048);
    }

    {
        // clear MCAN interrupts
        MCANInterrupt mcan_interrupt;
        mcan_interrupt.data = 0x3FFFFFFF;
        register_write(IR, mcan_interrupt.data);
    }
    {
        // enable MCAN interrupts
        mcanInterruptEnable_.data = 0; // start with all interrupts disabled
        mcanInterruptEnable_.rf0l = 1; // RX FIFO 0 message lost
        mcanInterruptEnable_.ep = 1;   // error passive
        mcanInterruptEnable_.bo = 1;   // bus-off status
        register_write(IE, mcanInterruptEnable_.data);
    }
    {
        // enable interrupt line 0
        Ile ile;
        ile.eint0 = 1;
        register_write(ILE, ile.data);
    }
    {
        // enable normal operation mode
        Cccr cccr;
        cccr.init = 0; // normal operation mode, enables CAN bus access
        register_write(CCCR, cccr.data);
    }

    interruptEnable_();
}

//
// disable()
//
void TCAN4550Can::disable()
{
    interruptDisable_();

    {
        // enable initalization mode
        Cccr cccr;
        cccr.init = 1; // initialization mode, disables CAN bus access
        register_write(CCCR, cccr.data);
    }
    {
        // disable interrupt line 0
        Ile ile;
        register_write(ILE, ile.data);
    }
    {
        // disable MCAN interrupts
        mcanInterruptEnable_.data = 0;
        register_write(IE, mcanInterruptEnable_.data);
    }

    flush_buffers();
}

//
// TCAN4550Can::flush_buffers()
//
void TCAN4550Can::flush_buffers()
{
    {
        AtomicHolder h(this);
        txBuf->flush();
    }

    // lock SPI bus access
    OSMutexLock locker(&lock_);

    // get the rx status (FIFO fill level)
    Rxfxs rxf0s;
    rxf0s.data = register_read(RXF0S);

    /// @todo this could be made more efficeint, but does it matter?
    while (rxf0s.ffl)
    {
        // acknowledge the next FIFO index
        Rxfxa rxf0a;
        rxf0a.fai = rxf0s.fgi;
        register_write(RXF0A, rxf0a.data);

        // increment index and count
        if (++rxf0s.fgi >= RX_FIFO_SIZE)
        {
            rxf0s.fgi = 0;
        }
        --rxf0s.ffl;
    }
}

//
// TCAN4550Can::read()
//
ssize_t TCAN4550Can::read(File *file, void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    struct can_frame *data = (struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        size_t frames_read = 0;
        MRAMRXBuffer *mram_rx_buffer = reinterpret_cast<MRAMRXBuffer*>(data);

        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);
            Rxfxs rxf0s;
            rxf0s.data = register_read(RXF0S);
            if (rxf0s.ffl)
            {
                static_assert(sizeof(struct can_frame) == sizeof(MRAMRXBuffer));

                // clip to the continous buffer memory available
                frames_read = std::min(RX_FIFO_SIZE - rxf0s.fgi, rxf0s.ffl);

                // clip to the number of asked for frames
                frames_read = std::min(frames_read, count);

                // read from MRAM
                rxbuf_read(0x0000 + (rxf0s.fgi * sizeof(MRAMRXBuffer)),
                           mram_rx_buffer, frames_read);

                // acknowledge the last FIFO index read
                Rxfxa rxf0a;
                rxf0a.fai = rxf0s.fgi + (frames_read - 1);
                register_write(RXF0A, rxf0a.data);
            }
        }
        // shuffle data for structure translation
        for (size_t i = 0; i < frames_read; ++i)
        {
            data[i].can_dlc = mram_rx_buffer[i].dlc;
            data[i].can_rtr = mram_rx_buffer[i].rtr;
            data[i].can_eff = mram_rx_buffer[i].xtd;
            data[i].can_err = mram_rx_buffer[i].esi;
            data[i].can_id &= 0x1FFFFFFF;
            if (!data[i].can_eff)
            {
                // standard frame
                data[i].can_id >>= 18;
            }
        }

        if (frames_read == 0)
        {
            // no more data to receive/
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                {
                    // lock SPI bus access
                    OSMutexLock locker(&lock_);

                    // enable receive interrupt
                    mcanInterruptEnable_.rf0n = 1;
                    register_write(IE, mcanInterruptEnable_.data);
                }
                // wait for data to come in
                rxBuf->block_until_condition(file, true);
            }
        }

        count -= frames_read;
        result += frames_read;
        data += frames_read;
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

//
// TCAN4550Can::write()
//
__attribute__((optimize("-O0")))
ssize_t TCAN4550Can::write(File *file, const void *buf, size_t count)
{
    HASSERT((count % sizeof(struct can_frame)) == 0);

    const struct can_frame *data = (const struct can_frame*)buf;
    ssize_t result = 0;

    count /= sizeof(struct can_frame);

    while (count)
    {
        size_t frames_written = 0;
        // note: This casts away the const qualifier. The buffer must be in
        //       mutable memory for this to work.
        MRAMTXBuffer *mram_tx_buffer = (MRAMTXBuffer*)(data);

        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);
#if 0
            do
            {
                // Get the TX event FIFO status
                Txefs txefs;
                txefs.data = register_read(TXEFS);

            } while (txefs.effl);
#endif
            // Get the TX FIFO/queu status
            Txfqs txfqs;
            txfqs.data = register_read(TXFQS);
            if (txfqs.tffl)
            {
                static_assert(sizeof(struct can_frame) == sizeof(MRAMTXBuffer));

                // clip to the continous buffer memory available
                frames_written = std::min(
                    TX_FIFO_SIZE - (txfqs.tfqpi - TX_DEDICATED_BUFFER_COUNT),
                    txfqs.tffl);

                // clip to the number of provided frames
                frames_written = std::min(frames_written, count);

                uint32_t txbar = 0;
                uint32_t put_index = txfqs.tfqpi;

                // shuffle data for structure translation
                for (size_t i = 0; i < frames_written; ++i, ++put_index)
                {
                    if (!data[i].can_eff)
                    {
                        // standard frame
                        mram_tx_buffer[i].id <<= 18;
                        mram_tx_buffer[i].id &= 0x1FFFFFFF;
                    }
                    mram_tx_buffer[i].rtr = data[i].can_rtr;
                    mram_tx_buffer[i].xtd = data[i].can_eff;
                    mram_tx_buffer[i].esi = data[i].can_err;
                    mram_tx_buffer[i].dlc = data[i].can_dlc;
                    mram_tx_buffer[i].brs = 0;
                    mram_tx_buffer[i].fdf = 0;
                    mram_tx_buffer[i].efc = 0;
                    mram_tx_buffer[i].mm = 0;
                    txbar |= 0x1 << put_index;
                }

                // write to MRAM
                txbuf_write(0x0480 + (txfqs.tfqpi * sizeof(MRAMTXBuffer)),
                            mram_tx_buffer, frames_written);

                // add transmission requests
                register_write(TXBAR, txbar);
            }
        }

        if (frames_written == 0)
        {
            /* no more data to receive */
            if ((file->flags & O_NONBLOCK) || result > 0)
            {
                break;
            }
            else
            {
                {
                    // lock SPI bus access
                    OSMutexLock locker(&lock_);

                    // enable transmit watermark interrupt
                    mcanInterruptEnable_.tefw = 1;
                    register_write(IE, mcanInterruptEnable_.data);
                }
                // wait for space to be available
                txBuf->block_until_condition(file, false);
            }
        }
        else
        {
            result += frames_written;
            count -= frames_written;
            data += frames_written;
        }
    }

    if (!result && (file->flags & O_NONBLOCK))
    {
        return -EAGAIN;
    }

    return result * sizeof(struct can_frame);
}

//
// TCQN4550Can::select()
//
bool TCAN4550Can::select(File* file, int mode)
{
    bool retval = false;
    switch (mode)
    {
        case FREAD:
        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);

            // read RX FIFO status
            Rxfxs rxf0s;
            rxf0s.data = register_read(RXF0S);
            if (rxf0s.ffl)
            {
                // RX FIFO has data
                retval = true;
            }
            else
            {
                // enable receive interrupt
                mcanInterruptEnable_.rf0n = 1;
                register_write(IE, mcanInterruptEnable_.data);

                // register for wakeup
                AtomicHolder h(this);
                rxBuf->select_insert();
            }
            break;
        }
        case FWRITE:
        {
            // lock SPI bus access
            OSMutexLock locker(&lock_);

            // read TX FIFO status
            Txfqs txfqs;
            txfqs.data = register_read(TXFQS);
            if (txfqs.tffl)
            {
                // TX FIFO has space
                retval = true;
            }
            else
            {
                // enable transmit watermark interrupt
                mcanInterruptEnable_.tefw = 1;
                register_write(IE, mcanInterruptEnable_.data);

                // register for wakeup
                AtomicHolder h(this);
                txBuf->select_insert();
            }
            break;
        }
        default:
            return Can::select(file, mode);
    }

    return retval;
}

//
// TCAN4550Can::ioctl()
//
int TCAN4550Can::ioctl(File *file, unsigned long int key, unsigned long data)
{
    if (key == SIOCGCANSTATE)
    {
        *((can_state_t*)data) = state_;
        return 0;
    }
    return -EINVAL;
}

//
// TCAN4550Can::tx_msg()
//
__attribute__((optimize("-O3")))
void TCAN4550Can::tx_msg()
{
    /* The caller has us in a critical section, we need to exchange a
     * critical section lock for a mutex lock since event handling happens
     * in thread context and not in interrupt context like it would in a
     * typical CAN device driver.
     */
    portEXIT_CRITICAL();
    lock_.lock();
    //tx_msg_locked();
    {
        /// @todo throw on the floor for now so we can text only RX
        AtomicHolder h_(this);
        txBuf->flush();
    }
    lock_.unlock();
    portENTER_CRITICAL();
}

//
// entry()
//
__attribute__((optimize("-O0")))
void *TCAN4550Can::entry()
{
    for ( ; /* forever */ ; )
    {
#if TCAN4550_DEBUG
        int result = sem_.timedwait(SEC_TO_NSEC(1));

        if (result != 0)
        {
            OSMutexLock locker(&lock_);
            MRAMMessage msg;
            msg.cmd = READ;
            msg.addrH = 0x10;
            msg.addrL = 0x00;
            msg.length = 64;

            spi_ioc_transfer xfer[2];
            xfer[0].tx_buf = (unsigned long)&msg;
            xfer[0].rx_buf = 0;
            xfer[0].len = sizeof(MRAMMessage);
            xfer[1].tx_buf = 0;
            xfer[1].rx_buf = (unsigned long)regs_;
            xfer[1].len = sizeof(regs_);

            spi_->transfer_with_cs_assert_polled(xfer, 2);
            for (unsigned i = 0; i < 64; ++i)
            {
                regs_[i] = be32toh(regs_[i]);
            }
            continue;
        }
#else
        sem_.wait();
#endif

        // lock SPI bus access
        OSMutexLock locker(&lock_);

        // read TCAN status flags
        uint32_t status = register_read(INTERRUPT_STATUS);

        // clear TCAN status flags
        register_write(INTERRUPT_STATUS, status);

#if TCAN4550_DEBUG
        status_ = status;
        enable_ = register_read(INTERRUPT_ENABLE);
        spiStatus_ = register_read(STATUS);
#endif
        // read status flags
        MCANInterrupt mcan_interrupt;
        mcan_interrupt.data = register_read(IR);

        // clear status flags
        register_write(IR, mcan_interrupt.data);


        // error handling
        if (mcan_interrupt.bo || mcan_interrupt.ep || mcan_interrupt.rf0l)
        {
            // read protocol status
            Psr psr;
            psr.data = register_read(PSR);

            if (mcan_interrupt.rf0l)
            {
                // receive overrun
                ++overrunCount;
            }
            if (mcan_interrupt.bo)
            {
                if (psr.bo)
                {
                    // bus off
                    ++busOffCount;
                    state_ = CAN_STATE_BUS_OFF;
                }
            }
            if (mcan_interrupt.ep)
            {
                if (psr.ep)
                {
                    // error passive state
                    ++softErrorCount;
                    state_ = CAN_STATE_BUS_PASSIVE;

                    // flush out any transmit data in the pipeline
                    //register_write(TXB0CTRL, 0x00);
                    //register_write(TXB1CTRL, 0x00);
                    //bit_modify(CANINTE, 0, TX0I | TX1I);
                    //bit_modify(CANINTF, 0, TX0I | TX1I);

                    portENTER_CRITICAL();
                    txBuf->flush();
                    portEXIT_CRITICAL();
                    txBuf->signal_condition();
                }
            }
        }

        // tranmission watermark
        if (mcan_interrupt.tefw && mcanInterruptEnable_.tefw)
        {
            // signal anyone waiting
            txBuf->signal_condition();

            // disable transmit watermark interrupt
            mcanInterruptEnable_.tefw = 0;
            register_write(IE, mcanInterruptEnable_.data);
        }

        // transmission complete
        if (mcan_interrupt.tc && mcanInterruptEnable_.tc)
        {
            /// @todo fill in if we support SW FIFO's
        }

        // received new message
        if (mcan_interrupt.rf0n && mcanInterruptEnable_.rf0n)
        {
            // signal anyone waiting
            rxBuf->signal_condition();

            // disable receive interrupt
            mcanInterruptEnable_.rf0n = 0;
            register_write(IE, mcanInterruptEnable_.data);
        }

        interruptEnable_();
    }

    return NULL;
}

